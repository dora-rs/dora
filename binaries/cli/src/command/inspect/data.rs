use std::{ptr::NonNull, sync::Arc};

use clap::Args;
use colored::Colorize;
use dora_core::topics::{open_zenoh_session, zenoh_output_publish_topic};
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
    metadata::{ArrowTypeInfo, BufferOffset},
};
use eyre::{eyre, Context};
use tokio::{runtime::Builder, task::JoinSet};
use uuid::Uuid;

use crate::command::{default_tracing, inspect::selector::InspectSelector, Executable};

/// Inspect data in terminal.
#[derive(Debug, Args)]
pub struct InspectData {
    #[clap(flatten)]
    selector: InspectSelector,
}

impl Executable for InspectData {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        inspect(self.selector)
    }
}

fn inspect(selector: InspectSelector) -> eyre::Result<()> {
    let coordinator_addr = selector.coordinator_addr;
    let outputs = selector.resolve()?;

    let rt = Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;
    rt.block_on(async move {
        let zenoh_session = open_zenoh_session(Some(coordinator_addr))
            .await
            .context("failed to open zenoh session")?;

        let mut join_set = JoinSet::new();
        for (dataflow_id, node_id, output_id) in outputs {
            join_set.spawn(log_to_terminal(
                zenoh_session.clone(),
                dataflow_id,
                node_id,
                output_id,
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
) -> eyre::Result<()> {
    let subscribe_topic = zenoh_output_publish_topic(dataflow_id, &node_id, &output_id);
    let output_name = format!("{node_id}/{output_id}");
    let subscriber = zenoh_session
        .declare_subscriber(subscribe_topic)
        .await
        .map_err(|e| eyre!(e))
        .wrap_err_with(|| format!("failed to subscribe to {output_name}"))?;

    let output_name = output_name.green();
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

                let mut output = format!("{output_name}\t");
                if let Some(data) = data {
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
                    let display = if array.is_empty() {
                        "[]".to_owned()
                    } else {
                        let mut display = format!("{:?}", arrow::array::make_array(array));
                        display = display
                            .split_once('\n')
                            .map(|(_, content)| content)
                            .unwrap_or(&display)
                            .replace("\n  ", " ");
                        if display.ends_with(",\n]") {
                            display.truncate(display.len() - 3);
                            display += " ]";
                        }
                        display
                    };

                    write!(output, " {}={display}", "data".bold()).unwrap();
                }
                if !metadata.parameters.is_empty() {
                    write!(output, " {}={:?}", "metadata".bold(), metadata.parameters).unwrap();
                }
                println!("{output}");
            }
            InterDaemonEvent::OutputClosed { .. } => {
                eprintln!("Output {node_id}/{output_id} closed");
                break;
            }
        }
    }

    Ok(())
}
