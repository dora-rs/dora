use std::{ptr::NonNull, sync::Arc, time::SystemTime};

use arrow::{buffer::OffsetBuffer, datatypes::Field};
use clap::Args;
use colored::Colorize;
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    metadata::{ArrowTypeInfo, BufferOffset, Parameter},
};
use eyre::eyre;

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
                    let ptr = NonNull::new(data.as_ptr() as *mut u8).unwrap();
                    let len = data.len();
                    let buffer = unsafe {
                        arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data))
                    };
                    let array = match buffer_into_arrow_array(&buffer, &metadata.type_info) {
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

fn buffer_into_arrow_array(
    raw_buffer: &arrow::buffer::Buffer,
    type_info: &ArrowTypeInfo,
) -> eyre::Result<arrow::array::ArrayData> {
    if raw_buffer.is_empty() {
        return Ok(arrow::array::ArrayData::new_empty(&type_info.data_type));
    }

    let mut buffers = Vec::new();
    for BufferOffset { offset, len } in &type_info.buffer_offsets {
        buffers.push(raw_buffer.slice_with_length(*offset, *len));
    }

    let mut child_data = Vec::new();
    for child_type_info in &type_info.child_data {
        child_data.push(buffer_into_arrow_array(raw_buffer, child_type_info)?)
    }

    arrow::array::ArrayData::try_new(
        type_info.data_type.clone(),
        type_info.len,
        type_info
            .validity
            .clone()
            .map(arrow::buffer::Buffer::from_vec),
        type_info.offset,
        buffers,
        child_data,
    )
    .map_err(|e| eyre!("Error creating Arrow array: {e}"))
}
