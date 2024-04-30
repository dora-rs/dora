use dora_node_api::{self, dora_core::config::DataId, DoraNode, Event, IntoArrow};
use eyre::Context;

fn main() -> eyre::Result<()> {
    println!("hello");

    let status_output = DataId::from("status".to_owned());
    let (mut node, mut events) = DoraNode::init_from_env()?;

    let mut ticks = 0;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_ref() {
                "tick" => {
                    ticks += 1;
                }
                "random" => {
                    let value = u64::try_from(&data).context("unexpected data type")?;

                    let output = format!(
                        "operator received random value {value:#x} after {} ticks",
                        ticks
                    );
                    node.send_output(
                        status_output.clone(),
                        metadata.parameters,
                        output.into_arrow(),
                    )?;
                }
                other => eprintln!("ignoring unexpected input {other}"),
            },
            Event::Stop => {} // TODO: should we stop the node here?
            Event::InputClosed { id } => {
                println!("input `{id}` was closed");
                if *id == "random" {
                    println!("`random` input was closed -> exiting");
                    break;
                }
            }
            other => {
                println!("received unknown event {other:?}");
            }
        }
    }

    Ok(())
}
