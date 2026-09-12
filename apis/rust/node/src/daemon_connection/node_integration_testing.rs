use std::{
    fs::File,
    io::Write,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use arrow::array::{Array, RecordBatch, StructArray};
use arrow_schema::{DataType, Field, Schema};
use colored::Colorize;
use dora_core::uhlc::{self, HLC, NTP64, Timestamp};
use dora_message::{
    common::{DataMessage, Timestamped},
    daemon_to_node::{DaemonReply, NodeEvent},
    integration_testing_format::{
        IncomingEvent, InputData, IntegrationTestInput, RecordingStatus, TimedIncomingEvent,
    },
    metadata::Metadata,
    node_to_daemon::DaemonRequest,
};
use eyre::{Context, ContextCompat};

use crate::{
    arrow_utils::encode_arrow_ipc,
    daemon_connection::json_to_arrow::read_json_value_as_arrow,
    event_stream::data_to_arrow_array,
    integration_testing::{TestingInput, TestingOptions, TestingOutput},
};

pub struct IntegrationTestingEvents {
    events: std::vec::IntoIter<TimedIncomingEvent>,
    output_writer: OutputWriter,
    start_timestamp: uhlc::Timestamp,
    start_time: Instant,
    options: TestingOptions,
    /// Set by `DoraNode::drop` so a mid-replay sleep in [`Self::next_event`]
    /// can abort and let the testing daemon process CloseOutputs (dora-rs/dora#2855).
    shutdown: Arc<AtomicBool>,
}

impl IntegrationTestingEvents {
    pub fn new(
        input: TestingInput,
        output: TestingOutput,
        options: TestingOptions,
        shutdown: Arc<AtomicBool>,
    ) -> eyre::Result<Self> {
        let mut node_info: IntegrationTestInput = match input {
            TestingInput::FromJsonFile(input_file_path) => serde_json::from_slice(
                &std::fs::read(&input_file_path)
                    .with_context(|| format!("failed to open {}", input_file_path.display()))?,
            )
            .with_context(|| format!("failed to deserialize {}", input_file_path.display()))?,
            TestingInput::Input(input) => input,
        };

        // Refuse to replay poisoned recordings. The `events` array is
        // known incomplete relative to the original run; loading it
        // anyway would defeat the whole point of recording (#1857).
        // Hand-authored fixtures and pre-#1857 recordings have
        // `recording_status: None` and pass through cleanly.
        if let Some(boxed) = &node_info.recording_status
            && let RecordingStatus::Poisoned {
                first_failure_event_index,
                first_failure_time_offset_secs,
                first_failure_error,
                additional_failures,
            } = boxed.as_ref()
        {
            eyre::bail!(
                "refusing to replay poisoned recording for node `{node_id}`: \
                 the original recorder failed at event index {first_failure_event_index} \
                 (~{first_failure_time_offset_secs:.3}s into the run), then \
                 {additional_failures} additional event(s) also failed to record. \
                 The `events` array is incomplete and replay will not reproduce \
                 the original behavior. First failure: {first_failure_error}",
                node_id = node_info.id,
            );
        }

        let output_writer = match output {
            TestingOutput::ToFile(output_file_path) => {
                let file = File::create(&output_file_path)
                    .with_context(|| format!("failed to create {}", output_file_path.display()))?;
                OutputWriter::Writer(Box::new(file))
            }
            TestingOutput::ToWriter(writer) => OutputWriter::Writer(writer),
            TestingOutput::ToChannel(sender) => OutputWriter::Channel(sender),
        };

        node_info
            .events
            .as_mut_slice()
            .sort_by(|a, b| a.time_offset_secs.total_cmp(&b.time_offset_secs));
        let inputs = std::mem::take(&mut node_info.events).into_iter();

        let clock = HLC::default();
        let start_timestamp = clock.new_timestamp();
        let start_time = Instant::now();
        Ok(Self {
            events: inputs,
            output_writer,
            start_timestamp,
            start_time,
            options,
            shutdown,
        })
    }

    pub fn request(&mut self, request: &Timestamped<DaemonRequest>) -> eyre::Result<DaemonReply> {
        let reply = match &request.inner {
            DaemonRequest::Register(_) => DaemonReply::Result(Ok(())),
            DaemonRequest::Subscribe => DaemonReply::Result(Ok(())),
            DaemonRequest::NextEvent => {
                let events = if let Some(event) = self.next_event()? {
                    vec![event]
                } else {
                    vec![]
                };
                DaemonReply::NextEvents(events)
            }
            DaemonRequest::SendMessage {
                output_id,
                metadata,
                data,
            } => self.handle_output(output_id, metadata, data)?,
            DaemonRequest::OutputSent { .. } => DaemonReply::Empty,
            DaemonRequest::CloseOutputs(data_ids) => {
                println!("{} {data_ids:?}", "node reports closed outputs".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::OutputsDone => {
                println!("{}", "node reports OutputsDone".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::EventStreamDropped => {
                println!("{}", "node reports EventStreamDropped".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::ExtensionRequest { namespace, .. } => {
                eyre::bail!("extension {namespace} is not available in integration-testing mode")
            }
            DaemonRequest::NodeConfig { .. } => {
                eyre::bail!("unexpected NodeConfig in interactive mode")
            }
            // `DaemonRequest` is `#[non_exhaustive]`: a request this build
            // predates is unsupported here.
            other => {
                eyre::bail!("unsupported request in integration-testing mode: {other:?}")
            }
        };
        Ok(reply)
    }

    fn handle_output(
        &mut self,
        output_id: &dora_message::id::DataId,
        metadata: &Metadata,
        data: &Option<DataMessage>,
    ) -> Result<DaemonReply, eyre::Error> {
        let start_timestamp = self.start_timestamp;
        let skip_output_time_offsets = self.options.skip_output_time_offsets;

        let arc_data = data.as_ref().map(|d| std::sync::Arc::new(d.clone()));
        let output = convert_output_to_json(
            output_id,
            metadata,
            &arc_data,
            start_timestamp,
            skip_output_time_offsets,
        )?;
        match &mut self.output_writer {
            OutputWriter::Writer(writer) => {
                serde_json::to_writer(writer.as_mut(), &output)
                    .context("failed to write output as JSON")?;
                writeln!(writer.as_mut()).context("failed to write newline to output file")?;
            }
            OutputWriter::Channel(sender) => {
                // Must not block: the node is waiting for this request's reply.
                sender
                    .send(output)
                    .context("failed to send output to channel")?;
            }
        }
        Ok(DaemonReply::Empty)
    }

    fn next_event(&mut self) -> eyre::Result<Option<Timestamped<NodeEvent>>> {
        if self.shutdown.load(Ordering::Relaxed) {
            return Ok(None);
        }

        let Some(event) = self.events.next() else {
            return Ok(None);
        };

        let TimedIncomingEvent {
            time_offset_secs,
            event,
        } = event;
        let time_offset = Duration::from_secs_f64(time_offset_secs);
        let elapsed = self.start_time.elapsed();
        if let Some(wait_time) = time_offset.checked_sub(elapsed) {
            // Sleep in short slices so `DoraNode::drop` can interrupt a
            // scheduled wait and still get a CloseOutputs reply (#2855).
            let deadline = Instant::now() + wait_time;
            while Instant::now() < deadline {
                if self.shutdown.load(Ordering::Relaxed) {
                    return Ok(None);
                }
                let remaining = deadline.saturating_duration_since(Instant::now());
                std::thread::sleep(remaining.min(Duration::from_millis(10)));
            }
        }

        let timestamp = Timestamp::new(
            self.start_timestamp.get_time() + NTP64::from(time_offset),
            *self.start_timestamp.get_id(),
        );

        let converted = match event {
            IncomingEvent::Stop => NodeEvent::Stop,
            IncomingEvent::Input { id, metadata, data } => {
                let data = if let Some(data) = data {
                    let array = read_input_data(*data).with_context(|| {
                        format!("failed to read input event at offset {time_offset_secs}s ")
                    })?;

                    // The receive side decodes a self-describing Arrow IPC
                    // stream, so encode the array into one here.
                    let buf = encode_arrow_ipc(&dora_arrow_convert::internal::from_array_data(
                        array,
                    ))
                    .with_context(|| {
                        format!("failed to IPC-encode input event at offset {time_offset_secs}s ")
                    })?;

                    Some(buf)
                } else {
                    None
                };
                let mut meta = Metadata::new(timestamp);
                meta.parameters = metadata.unwrap_or_default();
                NodeEvent::Input {
                    id,
                    metadata: std::sync::Arc::new(meta),
                    data: data.map(|d| {
                        std::sync::Arc::new(DataMessage::Vec(aligned_vec::AVec::from_slice(1, &d)))
                    }),
                }
            }
            IncomingEvent::InputClosed { id } => NodeEvent::InputClosed { id },
            IncomingEvent::AllInputsClosed => NodeEvent::AllInputsClosed,
        };
        Ok(Some(Timestamped {
            inner: converted,
            timestamp,
        }))
    }
}

enum OutputWriter {
    Writer(Box<dyn Write + Send>),
    Channel(crate::integration_testing::OutputSender),
}

pub fn convert_output_to_json(
    output_id: &dora_message::id::DataId,
    metadata: &Metadata,
    data: &Option<std::sync::Arc<DataMessage>>,
    start_timestamp: Timestamp,
    skip_output_time_offsets: bool,
) -> eyre::Result<serde_json::Map<String, serde_json::Value>> {
    let mut output = json_header(
        output_id,
        metadata,
        start_timestamp,
        skip_output_time_offsets,
    );
    if data.is_some() {
        let data_array = data_to_arrow_array(data.clone().map(std::sync::Arc::unwrap_or_clone))
            .context("failed to convert output to arrow array")?;
        append_arrow_array_json(&mut output, data_array)?;
    }
    Ok(output)
}

/// Serialize an already-decoded Arrow array (e.g. an input received over the
/// zenoh data plane) into the same JSON shape as [`convert_output_to_json`].
///
/// The daemon-path `Input` events reach the recorder as an encoded
/// [`DataMessage`], but zenoh-delivered inputs arrive already decoded as an
/// `ArrayData`, so this is the entry point for recording those.
pub fn convert_arrow_input_to_json(
    input_id: &dora_message::id::DataId,
    metadata: &Metadata,
    data: arrow::array::ArrayRef,
    start_timestamp: Timestamp,
    skip_output_time_offsets: bool,
) -> eyre::Result<serde_json::Map<String, serde_json::Value>> {
    let mut output = json_header(
        input_id,
        metadata,
        start_timestamp,
        skip_output_time_offsets,
    );
    append_arrow_array_json(&mut output, data)?;
    Ok(output)
}

/// Build the `id` (+ optional `time_offset_secs`) prefix shared by every
/// recorded input/output event.
fn json_header(
    id: &dora_message::id::DataId,
    metadata: &Metadata,
    start_timestamp: Timestamp,
    skip_output_time_offsets: bool,
) -> serde_json::Map<String, serde_json::Value> {
    let mut output = serde_json::Map::new();
    output.insert("id".into(), id.to_string().into());
    if !skip_output_time_offsets {
        let input_ts = metadata.timestamp();
        // A zenoh-delivered input can carry a remote HLC timestamp that
        // precedes this node's `start_timestamp` (a remote clock that is
        // behind, or a producer that started earlier). `get_diff_duration` is
        // an unguarded NTP64 (`u64`) subtraction that would underflow — a debug
        // panic (which unwinds past `add_event`'s `Err` guard and kills the
        // event loop) or a release wraparound to a garbage offset. Clamp to
        // zero when the input predates start. The daemon path only ever records
        // locally-timestamped inputs (always >= start), so this is a no-op
        // there.
        let time_offset = if input_ts.get_time() >= start_timestamp.get_time() {
            input_ts.get_diff_duration(&start_timestamp)
        } else {
            std::time::Duration::ZERO
        };
        output.insert("time_offset_secs".into(), time_offset.as_secs_f64().into());
    }
    output
}

/// Encode `data_array` into the `data` / `data_type` fields of a recorded
/// event's JSON object.
fn append_arrow_array_json(
    output: &mut serde_json::Map<String, serde_json::Value>,
    data_array: arrow::array::ArrayRef,
) -> eyre::Result<()> {
    let data_type_json = serde_json::to_value(data_array.data_type())
        .context("failed to serialize data type as JSON")?;

    let source = data_array.clone();
    let batch = RecordBatch::try_from_iter([("inner", data_array)])
        .context("failed to create RecordBatch")?;

    let mut writer = arrow_json::ArrayWriter::new(Vec::new());
    writer
        .write(&batch)
        .context("failed to encode data as JSON")?;
    writer
        .finish()
        .context("failed to finish writing JSON data")?;
    let json_data_encoded = writer.into_inner();

    // Reparse the string using serde_json
    let json_data: Vec<serde_json::Map<String, serde_json::Value>> =
        serde_json::from_reader(json_data_encoded.as_slice())
            .context("failed to parse JSON data again")?;
    // remove `inner` field again
    let mut json_data_flattened: Vec<_> = json_data
        .into_iter()
        .map(|mut m| m.remove("inner"))
        .collect();
    restore_non_finite_floats(&source, &mut json_data_flattened);
    output.insert("data".into(), json_data_flattened.into());
    output.insert("data_type".into(), data_type_json);
    Ok(())
}

/// `arrow_json` writes NaN and the infinities as `null`, because JSON has no
/// literal for either. Put them back as the strings the reader already accepts
/// for a float column (`"NaN"` / `"Infinity"` / `"-Infinity"`), so a recorded
/// non-finite float replays as itself rather than as a null.
fn restore_non_finite_floats(
    source: &arrow::array::ArrayRef,
    json: &mut [Option<serde_json::Value>],
) {
    use arrow::array::{Float32Array, Float64Array};

    fn encode(v: f64) -> Option<serde_json::Value> {
        if v.is_nan() {
            Some("NaN".into())
        } else if v == f64::INFINITY {
            Some("Infinity".into())
        } else if v == f64::NEG_INFINITY {
            Some("-Infinity".into())
        } else {
            None
        }
    }

    let values: Vec<Option<f64>> = if let Some(a) = source.as_any().downcast_ref::<Float64Array>() {
        (0..a.len())
            .map(|i| (!a.is_null(i)).then(|| a.value(i)))
            .collect()
    } else if let Some(a) = source.as_any().downcast_ref::<Float32Array>() {
        (0..a.len())
            .map(|i| (!a.is_null(i)).then(|| a.value(i) as f64))
            .collect()
    } else {
        return;
    };

    for (slot, value) in json.iter_mut().zip(values) {
        if let Some(encoded) = value.and_then(encode) {
            *slot = Some(encoded);
        }
    }
}

fn read_input_data(data: InputData) -> eyre::Result<arrow::array::ArrayData> {
    Ok(match data {
        InputData::JsonObject { data, data_type } => {
            // input is JSON data
            let array = json_value_to_list(data);
            // Resolve the declared element type up front, if the recording
            // carried one. The recorder always writes `data_type`; only
            // hand-authored fixtures may omit it.
            let declared_schema = data_type.map(data_type_to_schema).transpose()?;
            // A recorded zero-length output serializes to `"data": []`, which the
            // JSON reader turns into no record batch at all ("no record batch in
            // JSON"). A zero-length array is a legitimate value, so build an empty
            // array of the declared type directly instead of routing it through
            // the decoder (dora-rs/dora#3427). Handling it before schema inference
            // also avoids inferring from an empty iterator on the untyped path.
            if array.is_empty() {
                let data_type = declared_schema
                    .as_ref()
                    .and_then(|schema| schema.fields().first())
                    .map(|f| f.data_type().clone())
                    .unwrap_or(DataType::Null);
                return Ok(arrow::array::new_empty_array(&data_type).to_data());
            }
            let schema = match declared_schema {
                Some(schema) => schema,
                None => arrow_json::reader::infer_json_schema_from_iterator(array.iter().map(Ok))?,
            };
            let schema = Arc::new(schema);
            read_json_value_as_arrow(&array, schema.clone()).with_context(|| {
                format!(
                    "failed to decode JSON value for data type {}",
                    schema
                        .fields()
                        .first()
                        .map(|f| f.data_type())
                        .unwrap_or(&DataType::Null)
                )
            })?
        }
        InputData::ArrowFile {
            path,
            batch_index,
            column,
        } => {
            let file = std::fs::File::open(&path)
                .with_context(|| format!("failed to open arrow file {}", path.display()))?;
            let mut reader = arrow::ipc::reader::FileReader::try_new(file, None)
                .context("failed to create arrow file reader")?;
            reader.set_index(batch_index).with_context(|| {
                format!(
                    "failed to seek to batch index {} in arrow file {}",
                    batch_index,
                    path.display()
                )
            })?;
            let batch = reader
                .next()
                .context("no batch at given index")?
                .context("failed to read batch from arrow file")?;
            match column {
                Some(name) => batch
                    .column_by_name(&name)
                    .with_context(|| {
                        format!(
                            "failed to find column '{}' in batch at index {} of arrow file {}",
                            name,
                            batch_index,
                            path.display()
                        )
                    })?
                    .to_data(),
                None => StructArray::from(batch).to_data(),
            }
        }
    })
}

fn json_value_to_list(value: serde_json::Value) -> Vec<serde_json::Value> {
    match value {
        serde_json::Value::Array(inner) => inner.into_iter().map(wrap_value_into_object).collect(),
        _ => {
            // wrap into object to allow bare values
            let object = wrap_value_into_object(value);
            vec![object]
        }
    }
}

fn data_type_to_schema(ty: serde_json::Value) -> eyre::Result<Schema> {
    let ty = serde_json::from_value::<DataType>(ty)
        .context("failed to deserialize `type` field of input data")?;
    Ok(Schema::new([Arc::new(Field::new("inner", ty, true))]))
}

fn wrap_value_into_object(value: serde_json::Value) -> serde_json::Value {
    let mut map = serde_json::Map::new();
    map.insert("inner".into(), value);

    serde_json::Value::Object(map)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{ArrayRef, Float32Array, Float64Array, Int32Array, make_array};

    /// Record an array via the recorder's encoder, then replay it back through
    /// the reader — the property record/replay rests on.
    fn roundtrip(array: ArrayRef) -> eyre::Result<arrow::array::ArrayData> {
        let mut json = serde_json::Map::new();
        append_arrow_array_json(&mut json, array)?;
        read_input_data(InputData::JsonObject {
            data: json.remove("data").expect("encoder always writes `data`"),
            data_type: Some(
                json.remove("data_type")
                    .expect("encoder always writes `data_type`"),
            ),
        })
    }

    #[test]
    fn empty_array_round_trips() {
        // A zero-length output serializes to `"data": []`. Previously the reader
        // failed with "no record batch in JSON" and the value could not be
        // replayed at all (dora-rs/dora#3427).
        let array: ArrayRef = Arc::new(Int32Array::from(Vec::<i32>::new()));
        let back = make_array(roundtrip(array).expect("empty array should replay"));
        assert_eq!(back.len(), 0);
        assert_eq!(back.data_type(), &DataType::Int32);
    }

    #[test]
    fn non_finite_floats_round_trip() {
        // JSON has no NaN or infinity literal, so `arrow_json` writes all three
        // as `null`. They are now re-encoded as the strings the reader already
        // accepts, so they replay as themselves instead of as nulls
        // (dora-rs/dora#3427).
        let array: ArrayRef = Arc::new(Float64Array::from(vec![
            Some(1.5),
            Some(f64::NAN),
            Some(f64::INFINITY),
            Some(f64::NEG_INFINITY),
            None,
        ]));
        let back = Float64Array::from(roundtrip(array).expect("should replay"));

        assert_eq!(back.value(0), 1.5);
        assert!(back.value(1).is_nan());
        assert_eq!(back.value(2), f64::INFINITY);
        assert_eq!(back.value(3), f64::NEG_INFINITY);
        // The genuine null is still the only null: a NaN must not be
        // indistinguishable from a missing value.
        assert_eq!(back.null_count(), 1);
        assert!(back.is_null(4));
    }

    #[test]
    fn non_finite_f32_round_trips() {
        let array: ArrayRef = Arc::new(Float32Array::from(vec![
            Some(1.5f32),
            Some(f32::NAN),
            Some(f32::NEG_INFINITY),
        ]));
        let back = Float32Array::from(roundtrip(array).expect("should replay"));

        assert_eq!(back.value(0), 1.5);
        assert!(back.value(1).is_nan());
        assert_eq!(back.value(2), f32::NEG_INFINITY);
        assert_eq!(back.null_count(), 0);
    }

    #[test]
    fn recorded_non_finite_floats_use_string_encoding() {
        // Pins the on-disk shape. This is the recording format, so a change
        // here is a format change and should be deliberate.
        let array: ArrayRef = Arc::new(Float64Array::from(vec![
            Some(1.5),
            Some(f64::NAN),
            Some(f64::INFINITY),
            Some(f64::NEG_INFINITY),
            None,
        ]));
        let mut json = serde_json::Map::new();
        append_arrow_array_json(&mut json, array).expect("should encode");

        assert_eq!(
            json["data"],
            serde_json::json!([1.5, "NaN", "Infinity", "-Infinity", null]),
        );
    }

    #[test]
    fn recordings_written_before_the_string_encoding_still_read() {
        // A recording produced by the previous writer has `null` where a
        // non-finite value used to be. Those must keep reading as nulls rather
        // than failing, so existing recordings stay replayable.
        let decoded = read_input_data(InputData::JsonObject {
            data: serde_json::json!([1.5, null]),
            data_type: Some(serde_json::json!("Float64")),
        })
        .expect("an older recording should still read");
        let decoded = Float64Array::from(decoded);

        assert_eq!(decoded.value(0), 1.5);
        assert!(decoded.is_null(1));
    }

    #[test]
    fn non_finite_floats_nested_in_a_list_are_still_lost() {
        // The fixup walks the top-level array only, so a non-finite float
        // inside a List or Struct is still written as `null`. Pinning the
        // current limitation rather than endorsing it; extending the walk to
        // nested types is follow-up work on dora-rs/dora#3427.
        use arrow::array::ListArray;
        use arrow::datatypes::Float64Type;

        let list = ListArray::from_iter_primitive::<Float64Type, _, _>(vec![Some(vec![
            Some(1.5),
            Some(f64::NAN),
        ])]);
        let mut json = serde_json::Map::new();
        append_arrow_array_json(&mut json, Arc::new(list)).expect("should encode");

        assert_eq!(json["data"], serde_json::json!([[1.5, null]]));
    }

    #[test]
    fn non_empty_array_still_round_trips() {
        // Guard against the empty-array short-circuit affecting the normal path.
        let array: ArrayRef = Arc::new(Int32Array::from(vec![1, 2, 3]));
        let back = make_array(roundtrip(array).expect("array should replay"));
        let back = back
            .as_any()
            .downcast_ref::<Int32Array>()
            .expect("expected an Int32 array");
        assert_eq!(back.values(), &[1, 2, 3]);
    }
}
