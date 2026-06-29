use std::{ptr::NonNull, sync::Arc, time::SystemTime};

use arrow::{buffer::OffsetBuffer, datatypes::Field};
use clap::Args;
use colored::Colorize;
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent, metadata::Parameter};
use eyre::{Context, eyre};

use crate::{
    command::{Executable, default_tracing, topic::selector::TopicSelector},
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
/// _unstable_debug:
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

fn inspect(
    coordinator: CoordinatorOptions,
    selector: TopicSelector,
    format: OutputFormat,
    count: Option<u64>,
    duration: Option<u64>,
) -> eyre::Result<()> {
    let session = coordinator.connect()?;
    let (dataflow_id, topics) = selector.resolve(&session)?;

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
    let deadline = duration.map(|s| std::time::Instant::now() + std::time::Duration::from_secs(s));
    loop {
        // Stop conditions: --count reached or --duration elapsed.
        if let Some(max) = count
            && emitted >= max
        {
            break;
        }
        let recv_timeout = match deadline {
            Some(d) => {
                let remaining = d.saturating_duration_since(std::time::Instant::now());
                if remaining.is_zero() {
                    break;
                }
                remaining.min(HINT_TIMEOUT)
            }
            None => HINT_TIMEOUT,
        };
        let result = match data_rx.recv_timeout(recv_timeout) {
            Ok(result) => result,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                if let Some(d) = deadline
                    && std::time::Instant::now() >= d
                {
                    break;
                }
                if !hint_shown {
                    eprintln!(
                        "{}: no topic data received during the wait window. Ensure `_unstable_debug.enable_debug_inspection: true` is enabled on the dataflow.",
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

                let output_name = format!("{node_id}/{output_id}");

                let timestamp = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_millis();

                let data_str = if let Some(data) = data {
                    // Every data-plane payload is a self-describing Arrow IPC
                    // stream; decode it (zero-copy when the buffer is aligned).
                    let ptr = NonNull::new(data.as_ptr() as *mut u8).unwrap();
                    let len = data.len();
                    let buffer = unsafe {
                        arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data))
                    };
                    let array = match decode_arrow_ipc_zero_copy(buffer) {
                        Ok(array) => array,
                        Err(e) => {
                            eprintln!("invalid data on {output_name}: {e}");
                            continue;
                        }
                    };

                    let offsets = OffsetBuffer::new(vec![0, array.len() as _].into());
                    let field = Arc::new(Field::new_list_field(array.data_type().clone(), true));
                    let list_array = arrow::array::ListArray::new(
                        field,
                        offsets,
                        arrow::array::make_array(array),
                        None,
                    );
                    let batch =
                        arrow::array::RecordBatch::try_from_iter([("", Arc::new(list_array) as _)])
                            .unwrap();
                    let mut writer = arrow_json::LineDelimitedWriter::new(&mut buf);
                    writer.write(&batch).unwrap();
                    writer.finish().unwrap();
                    // The output looks like {"":[...]}\n
                    std::str::from_utf8(&buf[4..buf.len() - 2]).ok()
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
        }
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
fn decode_arrow_ipc_zero_copy(
    mut buffer: arrow::buffer::Buffer,
) -> eyre::Result<arrow::array::ArrayData> {
    use arrow::ipc::reader::StreamDecoder;

    let mut decoder = StreamDecoder::new();
    let mut batch = None;
    while !buffer.is_empty() {
        let before = buffer.len();
        if let Some(b) = decoder
            .decode(&mut buffer)
            .context("failed to decode Arrow IPC stream")?
        {
            batch = Some(b);
            break;
        }
        // Guard against a crafted/truncated payload that yields no batch without
        // consuming bytes — otherwise this loop spins forever.
        if buffer.len() == before {
            return Err(eyre!(
                "Arrow IPC decoder made no progress on a partial/corrupt stream"
            ));
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
    use arrow::array::{Array, Int32Array, StringArray};

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
    fn invalid_stream_is_rejected_not_panicked() {
        let err =
            decode_arrow_ipc_zero_copy(arrow::buffer::Buffer::from_vec(vec![0u8; 16])).unwrap_err();
        assert!(!err.to_string().is_empty());
    }
}
