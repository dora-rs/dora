//! A dynamic node that keeps producing output after the dataflow it
//! belongs to has already finished.
//!
//! Dynamic nodes never report a `SpawnedNodeResult`, so the daemon
//! finishes a dataflow as soon as every *non*-dynamic node has exited —
//! `should_finish` in `binaries/daemon/src/lib.rs` deliberately ignores
//! the still-running dynamic ones. `finish_dataflow` then drops the
//! dataflow from the daemon's `running` map while this node is still
//! connected and still able to send.
//!
//! Its next `send_output` therefore names a dataflow the daemon no
//! longer knows about. That is the same event the daemon sees when a
//! node's queued outputs outlive its exit — the nightly `smoke-suite`
//! failure in dora-rs/dora#2742 — but reached structurally instead of
//! by winning a race: the companion `emit-then-exit-source-node` is
//! wired to this node's `late` output and exits on receiving it, so
//! send #0 finishes the dataflow and every later send is a post-finish
//! one. No sleeps to tune.
//!
//! Sends are deliberately unchecked: `DaemonRequest::SendMessage`
//! expects no reply, so a node cannot observe the daemon's verdict. The
//! property under test is entirely daemon-side (it must survive), which
//! is why this fixture cannot assert it itself.

use std::{thread, time::Duration};

use dora_node_api::{
    DoraNode, IntoArrow,
    dora_core::config::{DataId, NodeId},
};
use eyre::Context;

/// Only send #1 needs to land after the finish; the rest are margin for
/// a loaded CI runner, and the test kills this process as soon as it has
/// seen what it needs.
const SENDS: u32 = 20;
const SEND_INTERVAL: Duration = Duration::from_millis(250);

fn main() -> eyre::Result<()> {
    let (mut node, _events) = DoraNode::init_from_node_id(NodeId::from("late-sender".to_string()))
        .context("failed to init dynamic node `late-sender`")?;
    let output = DataId::from("late".to_owned());

    // `_events` is never polled, so no `Stop` can cut this short: the
    // point is to still be sending well after the daemon considers the
    // dataflow over.
    for i in 0..SENDS {
        let _ = node.send_output(
            output.clone(),
            Default::default(),
            i64::from(i).into_arrow(),
        );
        thread::sleep(SEND_INTERVAL);
    }

    Ok(())
}
