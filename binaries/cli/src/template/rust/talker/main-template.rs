use dora_node_api::{dora_core::config::DataId, DoraNode, Event, IntoArrow};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata,
                data: _,
            } => match id.as_str() {
                "tick" => {
                    node.send_output(DataId::from("speech".to_owned()), metadata.parameters, String::from("Hello World!").into_arrow())?;
                    println!("Node received `{id}`");
                },
                _ => {}
            },
            _ => {}
        }
    }

    Ok(())
}
