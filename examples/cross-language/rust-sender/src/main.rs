use adora_node_api::{AdoraNode, Event, IntoArrow, adora_core::config::DataId};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;
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
