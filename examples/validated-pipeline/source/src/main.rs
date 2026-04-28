use dora_node_api::{DoraNode, Event, EventStream, IntoArrow, dora_core::config::DataId};
use eyre::Context;

#[cfg(test)]
mod tests;

fn main() -> eyre::Result<()> {
    let (node, events) = DoraNode::init_from_env()?;
    run(node, events)
}

fn run(mut node: DoraNode, mut events: EventStream) -> eyre::Result<()> {
    let output = DataId::from("value".to_owned());

    let mut sent: i64 = 0;
    while sent < 10 {
        let event = match events.recv() {
            Some(e) => e,
            None => break,
        };
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                node.send_output(output.clone(), metadata.parameters, sent.into_arrow())
                    .context("failed to send output")?;
                eprintln!("source: sent {sent}");
                sent += 1;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
