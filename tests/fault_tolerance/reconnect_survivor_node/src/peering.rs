//! Bidirectional traffic fixture for dynamic join peering. Records receipt of
//! both schema-once small messages and SHM-sized messages after initialization.
use std::{io::Write, path::PathBuf};

use dora_node_api::{DoraNode, Event, IntoArrow};

fn main() -> eyre::Result<()> {
    let mut args = std::env::args().skip(1);
    let name = args
        .next()
        .ok_or_else(|| eyre::eyre!("missing node name"))?;
    let directory = PathBuf::from(
        args.next()
            .ok_or_else(|| eyre::eyre!("missing directory"))?,
    );
    let (mut node, mut events) = if args.next().as_deref() == Some("dynamic") {
        DoraNode::init_from_node_id(name.clone().into())?
    } else {
        DoraNode::init_from_env()?
    };
    std::fs::write(directory.join(format!("{name}.ready")), b"ready")?;
    let mut received = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(directory.join(format!("{name}.received")))?;
    let mut sequence = 0_u64;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                if name != "anchor" {
                    let len = if sequence.is_multiple_of(2) {
                        8
                    } else {
                        128 * 1024
                    };
                    let bytes = vec![42_u8; len];
                    node.send_output("value".into(), metadata.parameters, bytes.into_arrow())?;
                    sequence += 1;
                }
            }
            Event::Input { data, .. } => {
                writeln!(received, "{}", data.len())?;
                received.flush()?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
