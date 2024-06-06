use dora_node_api::{DoraNode, Event};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata,
                data: _,
                ..
            } => match id.as_str() {
                other => eprintln!("Received input `{other}`"),
            },
            _ => {}
        }
    }

    Ok(())
}
