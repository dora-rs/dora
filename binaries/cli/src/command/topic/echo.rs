use std::{collections::HashMap, ptr::NonNull, sync::Arc, time::SystemTime};

use arrow::{buffer::OffsetBuffer, datatypes::Field};
use clap::Args;
use colored::Colorize;
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent, metadata::Parameter};
use eyre::{Context, eyre};

use crate::{
    command::{
        Executable, default_tracing,
        topic::selector::{TopicSelector, public_topic_output_id},
    },
    common::CoordinatorOptions,
    formatting::OutputFormat,
};

/// Echo topic data in terminal.
///
/// If no `DATA` is provided, all outputs from the selected dataflow will be
/// echoed.
///
/// Topic inspection requires debug mode on the dataflow:
///
/// ```yaml
/// debug:
///   enable_debug_inspection: true
/// ```
///
/// Examples:
///
/// Echo a single topic:
///   dora topic echo -d my-dataflow robot1/pose
///
/// Echo multiple topics:
///   dora topic echo -d my-dataflow robot1/pose robot2/vel
///
/// Emit JSON lines:
///   dora topic echo -d my-dataflow robot1/pose --format json
///
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Echo {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Output format
    ///
    /// `json` emits JSON Lines (one object per decoded message);
    /// diagnostics go to stderr.
    #[clap(long, value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,

    /// Exit after this many messages (default: stream until interrupted).
    /// Must be at least 1.
    #[clap(long, value_name = "N", value_parser = clap::value_parser!(u64).range(1..))]
    pub count: Option<u64>,

    /// Exit after this many seconds (default: stream until interrupted).
    /// Must be at least 1.
    #[clap(long, value_name = "SECONDS", value_parser = clap::value_parser!(u64).range(1..))]
    pub duration: Option<u64>,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Echo {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        inspect(
            self.coordinator,
            self.selector,
            self.format,
            self.count,
            self.duration,
        )
    }
}

/// Compute the `recv_timeout` for one iteration of the echo loop, or `None`
/// when the `--duration` window has elapsed and the loop should stop.
///
/// Taking the already-computed `elapsed` (rather than an absolute deadline)
/// keeps the caller off `Instant + Duration`, which overflows and panics for a
/// large-but-valid `--duration`. A remaining wait is capped at `hint` so the
/// loop still wakes periodically to show the "enable debug inspection" hint.
fn echo_recv_timeout(
    duration: Option<std::time::Duration>,
    elapsed: std::time::Duration,
    hint: std::time::Duration,
) -> Option<std::time::Duration> {
    match duration {
        Some(d) => {
            let remaining = d.saturating_sub(elapsed);
            if remaining.is_zero() {
                None
            } else {
                Some(remaining.min(hint))
            }
        }
        None => Some(hint),
    }
}

fn inspect(
    coordinator: CoordinatorOptions,
    selector: TopicSelector,
    format: OutputFormat,
    count: Option<u64>,
    duration: Option<u64>,
) -> eyre::Result<()> {
    let session = coordinator.connect()?;
    let (dataflow_id, topics, descriptor) = selector.resolve_with_descriptor(&session)?;
    // Frames report the daemon's wire output id; label them with the public id
    // the user typed and `topic list`/`info` display (dora-rs/dora#2893).
    let nodes: HashMap<_, _> = descriptor.nodes.iter().map(|n| (&n.id, n)).collect();

    let ws_topics: Vec<_> = topics
        .iter()
        .map(|t| (t.node_id.clone(), t.data_id.clone()))
        .collect();

    let (_subscription_id, data_rx) = session.subscribe_topics(dataflow_id, ws_topics)?;

    // If no data arrives within this timeout, hint that debug mode may be needed.
    const HINT_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);
    let mut hint_shown = false;
    let mut buf = Vec::with_capacity(1024);
    let mut emitted: u64 = 0;
    // Track the run window as a start instant plus `elapsed()` rather than an
    // absolute `Instant::now() + Duration::from_secs(seconds)`: `--duration` is
    // bounded only from below (`range(1..)`), so a large-but-valid `u64` would
    // make `Instant + Duration` overflow the monotonic clock and panic. See
    // `echo_recv_timeout`, and the sibling `topic hz` sampler.
    let start = std::time::Instant::now();
    let duration = duration.map(std::time::Duration::from_secs);
    loop {
        // Stop conditions: --count reached or --duration elapsed.
        if let Some(max) = count
            && emitted >= max
        {
            break;
        }
        let recv_timeout = match echo_recv_timeout(duration, start.elapsed(), HINT_TIMEOUT) {
            Some(timeout) => timeout,
            None => break,
        };
        let result = match data_rx.recv_timeout(recv_timeout) {
            Ok(result) => result,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                if let Some(d) = duration
                    && start.elapsed() >= d
                {
                    break;
                }
                if !hint_shown {
                    eprintln!(
                        "{}: no topic data received during the wait window. Ensure `debug.enable_debug_inspection: true` is enabled on the dataflow.",
                        "hint".yellow().bold(),
                    );
                    hint_shown = true;
                }
                continue;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        };
        buf.clear();
        let payload = match result {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Error receiving topic data: {e}");
                continue;
            }
        };

        let event = match Timestamped::deserialize_inter_daemon_event(&payload) {
            Ok(event) => event,
            Err(e) => {
                eprintln!("Received invalid event ({} bytes): {e}", payload.len());
                continue;
            }
        };

        match event.inner {
            InterDaemonEvent::Output {
                metadata,
                data,
                node_id,
                output_id,
                ..
            } => {
                use std::fmt::Write;

                let display_output = nodes
                    .get(&node_id)
                    .map(|node| public_topic_output_id(node, &output_id))
                    .unwrap_or_else(|| output_id.clone());
                let output_name = format!("{node_id}/{display_output}");

                let data_str = if let Some(data) = data {
                    // Every data-plane payload is a self-describing Arrow IPC
                    // stream; decode and render it (zero-copy when aligned).
                    match decode_and_render(data, &mut buf) {
                        // `render_array_json` guarantees the `{"":` prefix and
                        // `}\n` suffix, so this slice cannot go out of bounds.
                        Ok(()) => std::str::from_utf8(&buf[4..buf.len() - 2]).ok(),
                        Err(e) => {
                            eprintln!("invalid data on {output_name}: {e}");
                            continue;
                        }
                    }
                } else {
                    None
                };

                let metadata_str = if !metadata.parameters.is_empty() {
                    let mut output = "{".to_string();
                    for (i, (k, v)) in metadata.parameters.iter().enumerate() {
                        if i > 0 {
                            write!(output, ",").unwrap();
                        }
                        let value = match v {
                            Parameter::Bool(value) => value.to_string(),
                            Parameter::Integer(value) => value.to_string(),
                            Parameter::String(value) => serde_json::to_string(value).unwrap(),
                            Parameter::ListInt(value) => serde_json::to_string(value).unwrap(),
                            Parameter::Float(value) => serde_json::to_string(value).unwrap(),
                            Parameter::ListFloat(value) => serde_json::to_string(value).unwrap(),
                            Parameter::ListString(value) => serde_json::to_string(value).unwrap(),
                            Parameter::Timestamp(dt) => serde_json::to_string(dt).unwrap(),
                        };
                        write!(output, "{}:{value}", serde_json::Value::String(k.clone()),)
                            .unwrap();
                    }
                    write!(output, "}}").unwrap();
                    Some(output)
                } else {
                    None
                };

                let display_name = match format {
                    OutputFormat::Table => output_name.green().to_string(),
                    OutputFormat::Json => serde_json::to_string(&output_name).unwrap(),
                };

                match format {
                    OutputFormat::Table => {
                        let mut output = format!("{display_name}\t");
                        if let Some(s) = data_str {
                            write!(output, " {}={s}", "data".bold()).unwrap();
                        }
                        if let Some(s) = metadata_str {
                            write!(output, " {}={s}", "metadata".bold()).unwrap();
                        }
                        println!("{output}");
                    }
                    OutputFormat::Json => {
                        // Computed only on the JSON path: the default `table`
                        // format discards this, so a high-rate topic should not
                        // pay a `SystemTime::now()` syscall per frame for a value
                        // it never emits.
                        //
                        // `duration_since(UNIX_EPOCH)` errors when the wall clock
                        // is set before 1970 (e.g. an embedded target booting
                        // with an unset RTC before NTP sync). Fall back to a zero
                        // timestamp rather than panicking a live `dora topic
                        // echo`, mirroring the daemon's `current_millis()` helper.
                        let timestamp = SystemTime::now()
                            .duration_since(SystemTime::UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_millis();
                        println!(
                            r#"{{"timestamp":{},"name":{},"data":{},"metadata":{}}}"#,
                            timestamp,
                            display_name,
                            data_str.unwrap_or("null"),
                            metadata_str.as_deref().unwrap_or("null")
                        );
                    }
                }
                emitted += 1;
            }
            InterDaemonEvent::OutputClosed {
                node_id, output_id, ..
            } => {
                eprintln!("Output {node_id}/{output_id} closed");
            }
            // `InterDaemonEvent` is `#[non_exhaustive]`: skip events this build predates.
            _ => {}
        }
    }

    Ok(())
}

/// Decode the self-describing Arrow IPC payload and render it into `buf`.
fn decode_and_render(
    data: dora_message::aligned_vec::AVec<u8, dora_message::aligned_vec::ConstAlign<128>>,
    buf: &mut Vec<u8>,
) -> eyre::Result<()> {
    let ptr =
        NonNull::new(data.as_ptr() as *mut u8).ok_or_else(|| eyre!("payload pointer is null"))?;
    let len = data.len();
    let buffer = unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data)) };
    let array = decode_arrow_ipc_zero_copy(buffer)?;
    render_array_json(array, buf)
}

/// Render a decoded Arrow array into `buf` as `{"":[...]}\n`.
///
/// The payload is peer-controlled, and the Arrow JSON writer does not support
/// every Arrow type (e.g. `Float16`), so every step must surface an error
/// instead of panicking the CLI mid-stream.
///
/// On success, `buf` is guaranteed to start with `{"":` and end with `}\n`,
/// so callers can slice off those delimiters to obtain the bare JSON value.
fn render_array_json(array: arrow::array::ArrayData, buf: &mut Vec<u8>) -> eyre::Result<()> {
    // The array length is peer-controlled (e.g. a `NullArray` carries an
    // arbitrary `len` with no backing buffers); a plain `as` cast would wrap
    // to a negative offset and panic inside `OffsetBuffer::new`.
    let len = i32::try_from(array.len())
        .map_err(|_| eyre!("array length {} exceeds i32::MAX", array.len()))?;
    let offsets = OffsetBuffer::new(vec![0, len].into());
    let field = Arc::new(Field::new_list_field(array.data_type().clone(), true));
    let list_array =
        arrow::array::ListArray::new(field, offsets, arrow::array::make_array(array), None);
    let batch = arrow::array::RecordBatch::try_from_iter([("", Arc::new(list_array) as _)])
        .map_err(|e| eyre!("failed to build record batch: {e}"))?;
    let mut writer = arrow_json::LineDelimitedWriter::new(&mut *buf);
    writer
        .write(&batch)
        .and_then(|()| writer.finish())
        .map_err(|e| eyre!("cannot encode as JSON: {e}"))?;
    // The output looks like {"":[...]}\n
    if buf.len() < 6 || !buf.starts_with(b"{\"\":") || !buf.ends_with(b"}\n") {
        return Err(eyre!(
            "unexpected JSON writer output: {:?}",
            String::from_utf8_lossy(buf)
        ));
    }
    Ok(())
}

/// Decode an Arrow IPC stream from an Arrow [`Buffer`](arrow::buffer::Buffer)
/// without copying the body buffers when the input is aligned.
///
/// Mirrors `dora_node_api::arrow_utils::decode_arrow_ipc_zero_copy`: the data
/// plane is Arrow-IPC-only, so `dora topic echo` decodes the same self-describing
/// stream every node receives. The default `require_alignment = false` decoder
/// realigns under-aligned input rather than erroring.
///
/// Like that decoder, this rejects a multi-batch stream and trailing bytes
/// after the end-of-stream marker rather than silently returning only the first
/// batch — `dora topic echo` must not misrepresent a malformed payload as a
/// clean single value.
fn decode_arrow_ipc_zero_copy(
    mut buffer: arrow::buffer::Buffer,
) -> eyre::Result<arrow::array::ArrayData> {
    use arrow::ipc::reader::StreamDecoder;

    let mut decoder = StreamDecoder::new();
    let mut batch = None;
    while !buffer.is_empty() {
        let before = buffer.len();
        let decoded = match decoder.decode(&mut buffer) {
            Ok(decoded) => decoded,
            Err(e) => {
                // `decode` consumes the end-of-stream marker and then errors on
                // any byte after it, so once we already hold the batch a decode
                // error means the stream carried trailing bytes after a complete
                // single-batch stream. Reject them explicitly rather than
                // truncating to the first batch (matching the node-api decoder).
                let context = if batch.is_some() {
                    "unexpected trailing bytes after the record batch in IPC stream"
                } else {
                    "failed to decode Arrow IPC stream"
                };
                return Err::<arrow::array::ArrayData, _>(e).context(context);
            }
        };
        match decoded {
            Some(b) => {
                // A second batch means the stream is malformed; reject it rather
                // than silently dropping everything after the first batch.
                if batch.replace(b).is_some() {
                    return Err(eyre!(
                        "expected exactly one record batch in IPC stream, but found more than one"
                    ));
                }
            }
            // No batch and no progress: a crafted/truncated payload. Stop so the
            // loop cannot spin forever on a partial/corrupt stream.
            None if buffer.len() == before => break,
            // Progress without a batch: the schema message before the batch, or
            // the end-of-stream marker after it. Keep going; the loop ends when
            // the buffer is drained.
            None => {}
        }
    }

    let batch = batch.ok_or_else(|| eyre!("Arrow IPC stream contained no record batches"))?;
    if batch.num_columns() != 1 {
        return Err(eyre!(
            "expected 1 column in IPC record batch, got {}",
            batch.num_columns()
        ));
    }
    Ok(batch.column(0).to_data())
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{Array, Int32Array, NullArray, StringArray};

    /// Encode an array as a single-column IPC stream (matching the wire format),
    /// then decode it back via the echo path.
    fn encode_ipc(array: &dyn Array) -> Vec<u8> {
        use arrow::datatypes::{Field, Schema};
        use arrow::ipc::writer::StreamWriter;
        use arrow::record_batch::RecordBatch;
        use std::sync::Arc;

        let schema = Arc::new(Schema::new(vec![Field::new(
            "data",
            array.data_type().clone(),
            true,
        )]));
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![arrow::array::make_array(array.to_data())],
        )
        .unwrap();
        let mut buf = Vec::new();
        {
            let mut writer = StreamWriter::try_new(&mut buf, &schema).unwrap();
            writer.write(&batch).unwrap();
            writer.finish().unwrap();
        }
        buf
    }

    #[test]
    fn ipc_roundtrip_primitive() {
        let array = Int32Array::from(vec![10, 20, 30]);
        let encoded = encode_ipc(&array);
        let decoded = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(encoded)).unwrap();
        assert_eq!(decoded, array.to_data());
    }

    #[test]
    fn ipc_roundtrip_string() {
        let array = StringArray::from(vec![Some("a"), None, Some("ccc")]);
        let encoded = encode_ipc(&array);
        let decoded = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(encoded)).unwrap();
        assert_eq!(decoded, array.to_data());
    }

    #[test]
    fn render_strips_json_delimiters() {
        let array = arrow::array::Int64Array::from(vec![1, 2, 3]).into_data();
        let mut buf = Vec::new();
        render_array_json(array, &mut buf).unwrap();
        assert_eq!(
            std::str::from_utf8(&buf[4..buf.len() - 2]).unwrap(),
            "[1,2,3]"
        );
    }

    #[test]
    fn unsupported_json_type_is_rejected_not_panicked() {
        // The Arrow JSON writer cannot encode every Arrow type a node may
        // legitimately send (e.g. union arrays). That must surface as an
        // error on the affected message, not panic the whole echo stream.
        use arrow::datatypes::Int32Type;
        let mut builder = arrow::array::UnionBuilder::new_dense();
        builder.append::<Int32Type>("a", 1).unwrap();
        let array = builder.build().unwrap().into_data();
        let mut buf = Vec::new();
        let err = render_array_json(array, &mut buf).unwrap_err();
        assert!(
            err.to_string().contains("cannot encode as JSON"),
            "got: {err}"
        );
    }

    #[test]
    fn oversized_array_length_is_rejected_not_panicked() {
        // `array.len()` is peer-controlled and a `NullArray` carries it
        // without any backing buffers. A length above `i32::MAX` used to wrap
        // negative in the `as` cast and panic inside `OffsetBuffer::new`.
        let array = NullArray::new(i32::MAX as usize + 1).into_data();
        let mut buf = Vec::new();
        let err = render_array_json(array, &mut buf).unwrap_err();
        assert!(err.to_string().contains("exceeds i32::MAX"), "got: {err}");
    }

    #[test]
    fn invalid_stream_is_rejected_not_panicked() {
        let err =
            decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(vec![0u8; 16])).unwrap_err();
        assert!(!err.to_string().is_empty());
    }

    /// Encode the same single-column batch twice into one stream, to exercise
    /// the multi-batch rejection.
    fn encode_two_batch_ipc(array: &dyn Array) -> Vec<u8> {
        use arrow::datatypes::{Field, Schema};
        use arrow::ipc::writer::StreamWriter;
        use arrow::record_batch::RecordBatch;
        use std::sync::Arc;

        let schema = Arc::new(Schema::new(vec![Field::new(
            "data",
            array.data_type().clone(),
            true,
        )]));
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![arrow::array::make_array(array.to_data())],
        )
        .unwrap();
        let mut buf = Vec::new();
        {
            let mut writer = StreamWriter::try_new(&mut buf, &schema).unwrap();
            writer.write(&batch).unwrap();
            writer.write(&batch).unwrap();
            writer.finish().unwrap();
        }
        buf
    }

    /// A multi-batch stream must be rejected rather than silently truncated to
    /// its first batch — otherwise `dora topic echo` would render a malformed
    /// payload as a clean single value.
    #[test]
    fn multi_batch_stream_is_rejected() {
        let array = Int32Array::from(vec![1, 2, 3]);
        let encoded = encode_two_batch_ipc(&array);
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(encoded)).unwrap_err();
        assert!(err.to_string().contains("more than one"), "got: {err}");
    }

    /// A valid single-batch stream with extra bytes after its end-of-stream
    /// marker must be rejected too, matching the node-api decoder.
    #[test]
    fn trailing_bytes_are_rejected() {
        let array = Int32Array::from(vec![1, 2, 3]);
        let mut encoded = encode_ipc(&array);
        encoded.extend_from_slice(&[0u8; 16]);
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(encoded)).unwrap_err();
        assert!(err.to_string().contains("trailing bytes"), "got: {err}");
    }

    #[test]
    fn echo_recv_timeout_does_not_panic_on_oversized_duration() {
        use std::time::Duration;
        let hint = Duration::from_secs(5);

        // No `--duration`: always wait a hint-length window.
        assert_eq!(echo_recv_timeout(None, Duration::ZERO, hint), Some(hint));

        // A huge-but-valid `--duration` (10^19 s) must not panic: previously the
        // loop formed `Instant::now() + Duration::from_secs(seconds)`, which
        // overflowed the monotonic clock. The remaining wait is capped at `hint`.
        let huge = Duration::from_secs(10_000_000_000_000_000_000);
        assert_eq!(
            echo_recv_timeout(Some(huge), Duration::from_secs(1), hint),
            Some(hint),
        );

        // Once the window has elapsed, the loop is told to stop (`None`).
        assert_eq!(
            echo_recv_timeout(Some(Duration::from_secs(2)), Duration::from_secs(2), hint),
            None,
        );

        // A remaining window shorter than the hint is returned as-is.
        assert_eq!(
            echo_recv_timeout(Some(Duration::from_secs(3)), Duration::from_secs(1), hint),
            Some(Duration::from_secs(2)),
        );
    }
}
