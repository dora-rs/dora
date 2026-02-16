use adora_node_api::{adora_core::config::DataId, AdoraNode, Event, IntoArrow};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

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
