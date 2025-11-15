use std::{ptr::NonNull, sync::Arc, time::SystemTime};

use arrow::{buffer::OffsetBuffer, datatypes::Field};
use clap::Args;
use colored::Colorize;
use dora_core::topics::{open_zenoh_session, zenoh_output_publish_topic};
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
    metadata::{ArrowTypeInfo, BufferOffset, Parameter},
};
use eyre::{Context, eyre};
use tokio::{runtime::Builder, task::JoinSet};
use uuid::Uuid;

use crate::{
    command::{
        Executable, default_tracing,
        topic::selector::{TopicIdentifier, TopicSelector},
    },
    common::CoordinatorOptions,
    formatting::OutputFormat,
};

/// Echo topic data in terminal.
///
/// If no `DATA` is provided, all outputs from the selected dataflow will be
/// echoed.
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
/// Note: The dataflow descriptor must include the following snippet so that
/// runtime messages can be inspected:
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Echo {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Output format
    #[clap(long, value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Echo {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        inspect(self.coordinator, self.selector, self.format)
    }
}

fn inspect(
    coordinator: CoordinatorOptions,
    selector: TopicSelector,
    format: OutputFormat,
) -> eyre::Result<()> {
    let mut session = coordinator.connect()?;
    let (dataflow_id, topics) = selector.resolve(session.as_mut())?;

    let rt = Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;
    rt.block_on(async move {
        let zenoh_session = open_zenoh_session(Some(coordinator.coordinator_addr))
            .await
            .context("failed to open zenoh session")?;

        let mut join_set = JoinSet::new();
        for TopicIdentifier { node_id, data_id } in topics {
            join_set.spawn(log_to_terminal(
                zenoh_session.clone(),
                dataflow_id,
                node_id,
                data_id,
                format,
            ));
        }
        while let Some(res) = join_set.join_next().await {
            match res {
                Ok(Ok(())) => {}
                Ok(Err(e)) => {
                    eprintln!("Error while inspecting output: {e}");
                }
                Err(e) => {
                    eprintln!("Join error: {e}");
                }
            }
        }

        Result::<_, eyre::Error>::Ok(())
    })
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
    .context("Error creating Arrow array")
}

async fn log_to_terminal(
    zenoh_session: zenoh::Session,
    dataflow_id: Uuid,
    node_id: NodeId,
    output_id: DataId,
    format: OutputFormat,
) -> eyre::Result<()> {
    let subscribe_topic = zenoh_output_publish_topic(dataflow_id, &node_id, &output_id);
    let output_name = format!("{node_id}/{output_id}");
    let subscriber = zenoh_session
        .declare_subscriber(subscribe_topic)
        .await
        .map_err(|e| eyre!(e))
        .wrap_err_with(|| format!("failed to subscribe to {output_name}"))?;

    let output_name = match format {
        OutputFormat::Table => output_name.green().to_string(),
        OutputFormat::Json => serde_json::to_string(&output_name).unwrap(),
    };
    let mut buf = Vec::with_capacity(1024);
    while let Ok(sample) = subscriber.recv_async().await {
        let event = match Timestamped::deserialize_inter_daemon_event(&sample.payload().to_bytes())
        {
            Ok(event) => event,
            Err(_) => {
                eprintln!("Received invalid event");
                continue;
            }
        };
        match event.inner {
            InterDaemonEvent::Output { metadata, data, .. } => {
                use std::fmt::Write;

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
                            eprintln!("invalid data: {e}");
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
                        };
                        write!(output, "{}:{value}", serde_json::Value::String(k.clone()),)
                            .unwrap();
                    }
                    write!(output, "}}").unwrap();
                    Some(output)
                } else {
                    None
                };

                match format {
                    OutputFormat::Table => {
                        let mut output = format!("{output_name}\t");
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
                            output_name,
                            data_str.unwrap_or("null"),
                            metadata_str.as_deref().unwrap_or("null")
                        );
                    }
                }

                buf.clear();
            }
            InterDaemonEvent::OutputClosed { .. } => {
                eprintln!("Output {node_id}/{output_id} closed");
                break;
            }
        }
    }

    Ok(())
}
