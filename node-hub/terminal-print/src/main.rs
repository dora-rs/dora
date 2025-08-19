use dora_node_api::{self, DoraNode, Event, dora_core::config::NodeId};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let mut printed_error = String::new();
    loop {
        match DoraNode::init_from_node_id(NodeId::from("terminal-print".to_string())) {
            Ok((node, mut events)) => {
                printed_error = String::new();
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
                                println!("Received id: {id}, data: {received_string}");
                            }
                            _other => {
                                println!("Received id: {id}, data: {data:#?}");
                            }
                        },
                        _other => {}
                    }
                }
                // Waiting for the daemon to update ending of the dataflow.
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
            Err(err) => {
                if err.to_string() == printed_error {
                    println!("{err:#?}");
                    println!("🕐 waiting for node `terminal-print` to be available...");
                    printed_error = err.to_string();
                }
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
        }
    }
}
