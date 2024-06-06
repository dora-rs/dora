use dora_node_api::{self, DoraNode, Event};
use eyre::{bail, Context};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata: _,
                data,
                ..
            } => match id.as_str() {
                "message" => {
                    let received_string: &str =
                        TryFrom::try_from(&data).context("expected string message")?;
                    println!("sink received message: {}", received_string);
                    if !received_string.starts_with("operator received random value ") {
                        bail!("unexpected message format (should start with 'operator received random value')")
                    }
                    if !received_string.ends_with(" ticks") {
                        bail!("unexpected message format (should end with 'ticks')")
                    }
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop => {
                println!("Received manual stop");
            }
            Event::InputClosed { id } => {
                println!("Input `{id}` was closed");
            }
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    Ok(())
}
