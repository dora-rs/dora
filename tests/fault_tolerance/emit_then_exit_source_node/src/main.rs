//! Emit one `value` output on the first tick, then exit.
//!
//! Once this node exits the daemon closes its output, so a downstream
//! node's input closes while that downstream is still alive. That is the
//! setup for the finish-straggler watchdog (dora-rs/dora#2270): the
//! dataflow becomes "otherwise finished" except for any node that won't
//! exit, which the watchdog then escalates.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let output = DataId::from("value".to_owned());

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                node.send_output(output.clone(), metadata.parameters, 42i64.into_arrow())
                    .context("failed to send value")?;
                // Exit after the first emit; the daemon closing our output
                // is what drains the downstream node.
                break;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
