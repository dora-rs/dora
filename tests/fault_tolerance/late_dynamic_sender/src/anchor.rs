//! The only non-dynamic node in the late-send fixture: it exits as soon
//! as the dynamic sender's first message reaches it.
//!
//! Its exit is what makes the daemon finish the dataflow — `should_finish`
//! in `binaries/daemon/src/lib.rs` deliberately ignores nodes that are
//! still running but dynamic. Keying that exit on a message *from* the
//! sender is what makes the scenario structural instead of timed: by the
//! time this node goes away the sender is provably attached and mid-send,
//! so its remaining sends are guaranteed to land after `finish_dataflow`.

use dora_node_api::{DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    while let Some(event) = events.recv() {
        match event {
            // Exit cleanly, so the dataflow finishes normally rather
            // than through any failure path.
            Event::Input { id, .. } if id.as_str() == "late" => break,
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}
