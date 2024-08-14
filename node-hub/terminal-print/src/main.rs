use dora_node_api::{self, dora_core::config::NodeId, DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let mut printed = false;
    loop {
        if let Ok((node, mut events)) =
            DoraNode::init_from_node_id(NodeId::from("terminal-print".to_string()))
        {
            printed = false;
            println!("🔥 `terminal-print` connected to: {}", node.dataflow_id());

            while let Some(event) = events.recv() {
                match event {
                    Event::Input {
                        id,
                        metadata: _,
                        data,
                    } => match data.data_type() {
                        dora_node_api::arrow::datatypes::DataType::Utf8 => {
                            let received_string: &str =
                                TryFrom::try_from(&data).context("expected string message")?;
                            println!("Received id: {}, data: {}", id, received_string);
                        }
                        _other => {
                            println!("Received id: {}, data: {:#?}", id, data);
                        }
                    },
                    _other => {}
                }
            }
            // Waiting for the daemon to update ending of the dataflow.
            std::thread::sleep(std::time::Duration::from_secs(1));
        } else {
            if !printed {
                println!("🕐 waiting for node `terminal-print` to be available...");
                printed = true;
            }
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
    }
}
