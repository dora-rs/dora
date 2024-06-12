use dora_node_api::{DoraNode, Event};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata,
                data,
            } => match id.as_str() {
                "speech" => {
                    let message: &str = (&data).try_into()?;
                    println!("I heard: {message} from {id}");
                }
                other => eprintln!("Received input `{other}`"),
            },
            _ => {}
        }
    }

    Ok(())
}