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
            DaemonRequest::RegisterPinnedMemory { .. }
            | DaemonRequest::ReadPinnedMemory { .. }
            | DaemonRequest::FreePinnedMemory { .. }
            | DaemonRequest::WritePinnedMemory { .. } => DaemonReply::Result(Ok(())),
            DaemonRequest::RegisterCrossMachinePool { .. } => {
                eyre::bail!(
                    "cross-machine pool registration is not supported in integration-testing mode"
                )
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
    Channel(tokio::sync::mpsc::UnboundedSender<serde_json::Map<String, serde_json::Value>>),
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
    let json_data_flattened: Vec<_> = json_data
        .into_iter()
        .map(|mut m| m.remove("inner"))
        .collect();
    output.insert("data".into(), json_data_flattened.into());
    output.insert("data_type".into(), data_type_json);
    Ok(())
}

fn read_input_data(data: InputData) -> eyre::Result<arrow::array::ArrayData> {
    Ok(match data {
        InputData::JsonObject { data, data_type } => {
            // input is JSON data
            let array = json_value_to_list(data);
            let schema = match data_type {
                Some(ty) => data_type_to_schema(ty)?,
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
