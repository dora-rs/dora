use std::{ptr::NonNull, sync::Arc};

use colored::Colorize;
use dora_core::topics::zenoh_output_publish_topic;
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
};
use eyre::{eyre, Context, Result};
use uuid::Uuid;

use crate::buffer_into_arrow_array;

pub async fn log_to_terminal(
    zenoh_session: zenoh::Session,
    dataflow_id: Uuid,
    node_id: NodeId,
    output_id: DataId,
) -> Result<()> {
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
