use adora_node_api::{AdoraNode, Event};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

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
                other => println!("Received input `{other}`"),
            },
            _ => {}
        }
    }

    Ok(())
}
