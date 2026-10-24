use std::{collections::HashMap, ptr::NonNull, sync::Arc, time::SystemTime};

use arrow::{buffer::OffsetBuffer, datatypes::Field};
use clap::Args;
use colored::Colorize;
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    metadata::{Parameter, strip_internal_parameters},
};
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
                mut metadata,
                data,
                node_id,
                output_id,
                ..
            } => {
                use std::fmt::Write;

                // The daemon stamps every debug frame with its on-wire size for
                // `dora topic info`; it is not part of the node's metadata.
                strip_internal_parameters(&mut metadata.parameters);

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
    // arrow can panic on data that passed its own validation (a corrupted
    // run-end-encoded array), so printing is guarded too.
    catch_arrow_panic("failed to print the payload: arrow panicked", || {
        render_array_json(array, buf)
    })
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
/// Like that decoder, this rejects a multi-batch stream, a truncated stream and
/// trailing bytes after the end-of-stream marker rather than silently returning
/// only the first batch — `dora topic echo` must not misrepresent a malformed
/// payload as a clean single value — and it turns a panic inside arrow-ipc on a
/// malformed message into an error.
pub(super) fn decode_arrow_ipc_zero_copy(
    buffer: arrow::buffer::Buffer,
) -> eyre::Result<arrow::array::ArrayData> {
    check_row_counts(buffer.as_slice())?;
    catch_arrow_panic(
        "failed to decode Arrow IPC stream: the decoder panicked",
        || decode_single_batch(buffer),
    )
}

/// Run `f`, turning a panic inside arrow into an error `"{what}: {reason}"`.
fn catch_arrow_panic<T>(what: &str, f: impl FnOnce() -> eyre::Result<T>) -> eyre::Result<T> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or_else(|panic| {
        let reason = panic
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
            .unwrap_or("unknown reason");
        Err(eyre!("{what}: {reason}"))
    })
}

/// Read an IPC message header, refusing one that grows to more than 4 times
/// its size when read (flatbuffers lets one string or table be referenced many
/// times, and arrow copies it for each). Mirrors `dora_node_api::arrow_utils`.
fn read_message_header(metadata: &[u8]) -> eyre::Result<arrow::ipc::Message<'_>> {
    let options = flatbuffers::VerifierOptions {
        max_apparent_size: metadata.len().saturating_mul(4),
        ..Default::default()
    };
    arrow::ipc::root_as_message_with_opts(&options, metadata)
        .map_err(|e| eyre!("invalid Arrow IPC message header: {e}"))
}

/// Most rows an array in a payload can have: one per bit of dora's largest
/// payload (256 MB). Mirrors `dora_node_api::arrow_utils`.
const MAX_IPC_ROWS: i64 = 256 * 1024 * 1024 * 8;

/// Reject a record batch (or dictionary batch) claiming more than
/// [`MAX_IPC_ROWS`] rows, overall, in any column or in all columns together,
/// or a compressed one, before arrow sees it: arrow
/// validates some arrays by allocating per claimed row, and a failed allocation
/// aborts the process. Mirrors `dora_node_api::arrow_utils::check_row_counts`;
/// anything else malformed is left to the decoder.
fn check_row_counts(stream: &[u8]) -> eyre::Result<()> {
    let read_i32 = |at: usize| -> Option<i32> {
        Some(i32::from_le_bytes(stream.get(at..at + 4)?.try_into().ok()?))
    };
    let mut at = 0usize;
    loop {
        let Some(mut metadata_len) = read_i32(at) else {
            return Ok(());
        };
        if metadata_len == -1 {
            at += 4;
            let Some(len) = read_i32(at) else {
                return Ok(());
            };
            metadata_len = len;
        }
        let Ok(metadata_len) = usize::try_from(metadata_len) else {
            return Ok(());
        };
        if metadata_len == 0 {
            return Ok(());
        }
        let start = at + 4;
        let Some(metadata) = start
            .checked_add(metadata_len)
            .and_then(|end| stream.get(start..end))
        else {
            return Ok(());
        };
        // A header arrow would also refuse is an error either way.
        let message = read_message_header(metadata)?;
        let dictionary = message.header_as_dictionary_batch();
        if dictionary.is_some_and(|dictionary| dictionary.isDelta()) {
            return Err(eyre!(
                "Arrow IPC delta dictionary batches are not supported"
            ));
        }
        let batch = message
            .header_as_record_batch()
            .or_else(|| dictionary.and_then(|dictionary| dictionary.data()));
        if let Some(batch) = batch {
            if batch.compression().is_some() {
                // arrow reserves the size a compressed buffer claims before
                // reading it (when a crate enables arrow's compression), and
                // dora never compresses.
                return Err(eyre!("compressed Arrow IPC messages are not supported"));
            }
            // arrow casts these to `usize`, so a negative one is as bad as a
            // huge one.
            let columns = batch.nodes().into_iter().flatten();
            let counts = std::iter::once(batch.length())
                .chain(columns.flat_map(|node| [node.length(), node.null_count()]));
            for rows in counts {
                if !(0..=MAX_IPC_ROWS).contains(&rows) {
                    return Err(eyre!(
                        "Arrow IPC message claims {rows} rows, outside the 0..={MAX_IPC_ROWS} \
                         rows a payload can hold"
                    ));
                }
            }
            // arrow can build a bitmap per row for each column (a union keeps
            // one per child), so all columns together get the limit too.
            let rows = batch
                .nodes()
                .into_iter()
                .flatten()
                .fold(0i64, |rows, node| rows.saturating_add(node.length()));
            if rows > MAX_IPC_ROWS {
                return Err(eyre!(
                    "Arrow IPC message claims {rows} rows across its columns, more than the \
                     {MAX_IPC_ROWS} a payload can hold"
                ));
            }
            // Each buffer must lie inside the body, and together they may not be
            // longer than it: arrow copies an unaligned buffer once for every
            // column that points at it, so buffers sharing bytes would make a
            // small message allocate many times its size.
            let body = message.bodyLength();
            let mut total = 0i64;
            for buffer in batch.buffers().into_iter().flatten() {
                let (offset, length) = (buffer.offset(), buffer.length());
                let end = offset.checked_add(length);
                total = total.saturating_add(length);
                if offset < 0 || length < 0 || end.is_none_or(|end| end > body) || total > body {
                    return Err(eyre!(
                        "Arrow IPC message buffers don't fit in its {body}-byte body"
                    ));
                }
            }
        }
        let Ok(body_len) = usize::try_from(message.bodyLength()) else {
            return Ok(());
        };
        match (start + metadata_len).checked_add(body_len) {
            Some(next) if next <= stream.len() => at = next,
            _ => return Ok(()),
        }
    }
}

fn decode_single_batch(mut buffer: arrow::buffer::Buffer) -> eyre::Result<arrow::array::ArrayData> {
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

    // A stream that ends inside a message (for example inside a record batch's
    // body, or halfway through the end-of-stream marker) is truncated.
    decoder
        .finish()
        .context("truncated Arrow IPC stream: it ends inside a message")?;
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
pub(super) mod tests {
    use super::*;
    use arrow::array::{Array, Int32Array, NullArray, StringArray};

    /// Encode an array as a single-column IPC stream (matching the wire format),
    /// then decode it back via the echo path.
    pub(in crate::command::topic) fn encode_ipc(array: &dyn Array) -> Vec<u8> {
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

    pub(in crate::command::topic) fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// Found by fuzzing: makes arrow-ipc panic (`convert.rs`) (#3863).
    const PANICS_ARROW: &str = "ffffffff320000001000000000000a0000000000090004000a00000000000000000100000000000000000000000000000000000000000000000000";
    /// Found by fuzzing: its record batch claims a huge body (#3863).
    pub(in crate::command::topic) const CLAIMS_HUGE_BODY: &str = "ffffffff780000001000000000000a000c000a00090004000a00000010000000000104000800080000000400080000000400000001000000140000001000160010000e000f0004000000080010000000180000001c00000000000102180000000000060008000400060000000800000000000000040000006441746100000000ffffffffb8000000100000000c001a0018001700040008000c00000020000000800000000000400000000000000000000304000a0018000c00080004000a0000002c000000100000000300000000000000000000000100000003000000000000000000000000000000000000000200000000000000000000000100000000000000400000000000000003000000000000000000000000000000ffffff7f0000000000000000000000000000000000000000000000000000000000000000000000";

    /// Corrupted streams make `dora topic echo` report an error and keep going
    /// instead of crashing (#3863).
    #[test]
    fn corrupt_streams_are_rejected_without_panicking() {
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(hex(PANICS_ARROW)))
            .unwrap_err()
            .to_string();
        assert!(err.contains("the decoder panicked: "), "got: {err}");
        assert!(!err.contains("unknown reason"), "got: {err}");

        let result =
            decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(hex(CLAIMS_HUGE_BODY)));
        assert!(result.is_err(), "accepted a corrupt stream");
    }

    /// A stream claiming 2^57 rows, or a negative row count (which arrow casts
    /// to nearly 2^64), is rejected before arrow allocates a bitmap for every
    /// claimed row and aborts the CLI (#3863).
    #[test]
    fn a_huge_or_negative_row_count_is_rejected() {
        for to in [(1u64 << 57).to_le_bytes(), (-1i64).to_le_bytes()] {
            use arrow::array::{ArrayData, make_array};
            let rows = 0x0102_0304_usize;
            let child = unsafe {
                ArrayData::new_unchecked(
                    arrow_schema::DataType::Null,
                    rows,
                    Some(rows),
                    None,
                    0,
                    vec![],
                    vec![],
                )
            };
            let ty = arrow_schema::DataType::Struct(
                vec![Field::new("x", arrow_schema::DataType::Null, false)].into(),
            );
            let parent = unsafe {
                ArrayData::new_unchecked(ty, rows, Some(0), None, 0, vec![], vec![child])
            };
            let mut bytes = encode_ipc(&make_array(parent));
            let from = (rows as u64).to_le_bytes();
            let mut at = 0;
            while at + 8 <= bytes.len() {
                if bytes[at..at + 8] == from {
                    bytes[at..at + 8].copy_from_slice(&to);
                    at += 8;
                } else {
                    at += 1;
                }
            }
            let err =
                decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(bytes)).unwrap_err();
            assert!(err.to_string().contains("rows"), "got: {err}");
        }
    }

    /// A compressed batch is rejected like in the node API decoder: arrow
    /// reserves the size a compressed buffer claims before reading it (here
    /// 2^57 bytes), when a crate in the build enables arrow's compression.
    #[test]
    fn a_compressed_batch_is_rejected() {
        let stream = hex(concat!(
            "ffffffffb80000001000000000000a000c000a00090004000a000000100000000001040008000800",
            "00000400080000000400000001000000140000001000140010000e000f0004000000080010000000",
            "1800000020000000000001021c00000008000c0004000b0008000000200000000000000100000000",
            "04000000646174610000000000000000000000000000000000000000000000000000000000000000",
            "0000000000000000000000000000000000000000000000000000000000000000ffffffffb8000000",
            "100000000c00140012001100040008000c0000001c0000001000000000000000000304000c001e00",
            "10000c00080004000c000000200000003c0000002000000001000000000000000000000000000600",
            "08000700060000000000000101000000010000000000000000000000000000000000000002000000",
            "00000000000000000000000000000000000000000000000010000000000000000000000000000000",
            "00000000000000000000000000000000000000000000000000000000000000020000000000000000",
            "ffffffff00000000",
        ));
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(stream)).unwrap_err();
        assert_eq!(
            err.to_string(),
            "compressed Arrow IPC messages are not supported"
        );
    }

    /// All columns together get the row limit too, like in the node API
    /// decoder: a struct of two `Null` columns of 2^30 rows is 3 * 2^30 rows.
    #[test]
    fn rows_of_all_columns_together_are_limited() {
        use arrow::array::{ArrayData, make_array};
        let nulls = |n: usize| {
            let fields: Vec<Field> = (0..n)
                .map(|i| Field::new(format!("n{i}"), arrow_schema::DataType::Null, true))
                .collect();
            let children = (0..n)
                .map(|_| NullArray::new(1 << 30).into_data())
                .collect();
            ArrayData::builder(arrow_schema::DataType::Struct(fields.into()))
                .len(1 << 30)
                .child_data(children)
                .build()
                .unwrap()
        };
        let bytes = encode_ipc(&make_array(nulls(2)));
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(bytes)).unwrap_err();
        assert!(
            err.to_string()
                .contains("claims 3221225472 rows across its columns"),
            "got: {err}"
        );
        let bytes = encode_ipc(&make_array(nulls(1)));
        let decoded = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(bytes)).unwrap();
        assert_eq!(decoded, nulls(1));
    }

    /// A run-end-encoded array whose run end was changed in transit passes
    /// every check and arrow's validation, and arrow panics while printing it;
    /// echo must report an error instead of crashing (#3863, Codex).
    #[test]
    fn printing_a_corrupted_run_end_array_is_an_error() {
        use arrow::array::RunArray;
        use arrow::datatypes::Int32Type;
        let array =
            RunArray::<Int32Type>::try_new(&Int32Array::from(vec![2]), &Int32Array::from(vec![42]))
                .unwrap();
        let mut bytes = encode_ipc(&array);
        // The body holds the run end (2) and then the value (42).
        let value = bytes.windows(4).rposition(|w| w == [42, 0, 0, 0]).unwrap();
        let run_end = bytes[..value]
            .windows(4)
            .rposition(|w| w == [2, 0, 0, 0])
            .unwrap();
        bytes[run_end] = 1;
        let decoded = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from(bytes.as_slice()));
        assert_eq!(decoded.unwrap().len(), 2);

        let payload = dora_message::aligned_vec::AVec::from_slice(128, &bytes);
        let mut buf = Vec::new();
        let err = decode_and_render(payload, &mut buf)
            .unwrap_err()
            .to_string();
        assert!(err.contains("panicked"), "got: {err}");
    }

    /// A message header that grows to more than 4 times its size when read is
    /// rejected like in the node API decoder: here one 4 KB child field listed
    /// 8 times, which arrow would copy for every reference.
    #[test]
    fn a_message_header_that_grows_when_read_is_rejected() {
        use arrow::ipc::{
            FieldBuilder, MessageBuilder, MessageHeader, MetadataVersion, NullBuilder,
            SchemaBuilder, Struct_Builder, Type,
        };
        let mut fbb = flatbuffers::FlatBufferBuilder::new();
        let name = fbb.create_string(&"x".repeat(4096));
        let null = NullBuilder::new(&mut fbb).finish();
        let child = {
            let mut builder = FieldBuilder::new(&mut fbb);
            builder.add_name(name);
            builder.add_type_type(Type::Null);
            builder.add_type_(null.as_union_value());
            builder.finish()
        };
        let children = fbb.create_vector(&[child; 8]);
        let struct_ = Struct_Builder::new(&mut fbb).finish();
        let column = {
            let mut builder = FieldBuilder::new(&mut fbb);
            builder.add_type_type(Type::Struct_);
            builder.add_type_(struct_.as_union_value());
            builder.add_children(children);
            builder.finish()
        };
        let fields = fbb.create_vector(&[column]);
        let schema = {
            let mut builder = SchemaBuilder::new(&mut fbb);
            builder.add_fields(fields);
            builder.finish()
        };
        let message = {
            let mut builder = MessageBuilder::new(&mut fbb);
            builder.add_version(MetadataVersion::V5);
            builder.add_header_type(MessageHeader::Schema);
            builder.add_header(schema.as_union_value());
            builder.finish()
        };
        fbb.finish(message, None);
        let header = fbb.finished_data();
        let mut stream = vec![0xff, 0xff, 0xff, 0xff];
        stream.extend_from_slice(&(header.len() as i32).to_le_bytes());
        stream.extend_from_slice(header);
        stream.extend_from_slice(&[0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0]);
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(stream)).unwrap_err();
        assert!(
            err.to_string()
                .starts_with("invalid Arrow IPC message header"),
            "got: {err}"
        );
    }

    /// A delta dictionary is rejected like in the node API decoder.
    #[test]
    fn a_delta_dictionary_is_rejected() {
        use arrow::array::{DictionaryArray, UInt8Array};
        use arrow::ipc::writer::{DictionaryHandling, IpcWriteOptions, StreamWriter};

        let ty = arrow_schema::DataType::Dictionary(
            Box::new(arrow_schema::DataType::UInt8),
            Box::new(arrow_schema::DataType::Utf8),
        );
        let schema = Arc::new(arrow_schema::Schema::new(vec![Field::new(
            "data", ty, false,
        )]));
        let options =
            IpcWriteOptions::default().with_dictionary_handling(DictionaryHandling::Delta);
        let mut stream = Vec::new();
        let mut writer = StreamWriter::try_new_with_options(&mut stream, &schema, options).unwrap();
        for values in [vec!["a"], vec!["a", "b"]] {
            let keys = UInt8Array::from(vec![values.len() as u8 - 1]);
            let column = DictionaryArray::new(keys, Arc::new(StringArray::from(values)));
            let batch =
                arrow::array::RecordBatch::try_new(schema.clone(), vec![Arc::new(column)]).unwrap();
            writer.write(&batch).unwrap();
        }
        writer.finish().unwrap();
        drop(writer);
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(stream)).unwrap_err();
        assert!(err.to_string().contains("delta"), "got: {err}");
    }

    /// Buffers outside the message body, or longer than it together, are
    /// rejected like in the node API decoder (#3863).
    #[test]
    fn buffers_that_do_not_fit_in_the_body_are_rejected() {
        let original = hex(
            "fffffffff80000001000000000000a000c000a00090004000a0000001000000000010400080008000000040008000000040000000100000004000000acffffff200000000c0000000000000d88000000020000004c0000000c0000000400040004000000d4ffffff10000000180000000000000214000000c4ffffff400000000000000100000000020000006331000010001400100000000f00040000000800100000001800000020000000000000021c00000008000c0004000b000800000040000000000000010000000002000000633000000400000064617461000000000000000000000000000000000000000000000000000000000000000000000000ffffffffd800000014000000000000000c001600140013000c0004000c0000001000000000000000140000000000000304000a0018000c00080004000a0000001400000068000000010000000000000000000000050000000000000000000000000000000000000000000000000000000000000000000000010000000000000008000000000000000000000000000000000000000000000001000000000000000800000000000000000000000300000001000000000000000000000000000000010000000000000000000000000000000100000000000000000000000000000007070707070707070707070707070707ffffffff00000000",
        );
        let mut needle = 1i64.to_le_bytes().to_vec();
        needle.extend_from_slice(&8i64.to_le_bytes());
        let at = (0..original.len() - 16)
            .filter(|&i| original[i..i + 16] == needle[..])
            .nth(1)
            .unwrap();
        for (offset, length) in [(1i64, 9i64), (9, 8)] {
            let mut bytes = original.clone();
            bytes[at..at + 8].copy_from_slice(&offset.to_le_bytes());
            bytes[at + 8..at + 16].copy_from_slice(&length.to_le_bytes());
            let err =
                decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(bytes)).unwrap_err();
            assert!(err.to_string().contains("body"), "got: {err}");
        }
    }

    /// A stream cut inside its end-of-stream marker is truncated, matching the
    /// node-api decoder.
    #[test]
    fn a_stream_cut_inside_the_end_marker_is_rejected() {
        let mut encoded = encode_ipc(&Int32Array::from(vec![1, 2, 3]));
        encoded.truncate(encoded.len() - 4);
        let err = decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(encoded)).unwrap_err();
        assert!(err.to_string().contains("truncated"), "got: {err}");
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
