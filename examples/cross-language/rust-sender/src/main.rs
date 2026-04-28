use dora_node_api::{DoraNode, Event, EventStream, IntoArrow, dora_core::config::DataId};
use eyre::Context;

#[cfg(test)]
mod tests;

fn main() -> eyre::Result<()> {
    let (node, events) = DoraNode::init_from_env()?;
    run(node, events)
}

fn run(mut node: DoraNode, mut events: EventStream) -> eyre::Result<()> {
    let output = DataId::from("values".to_owned());

    // Send 10 messages with known values: [0, 10, 20, ..., 90]
    let mut sent: i64 = 0;
    while sent < 10 {
        let event = match events.recv() {
            Some(e) => e,
            None => break,
        };
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                let value = sent * 10;
                node.send_output(output.clone(), metadata.parameters, value.into_arrow())
                    .context("failed to send output")?;
                eprintln!("rust-sender: sent {value}");
                sent += 1;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
