//! Fire-and-forget producer with no `events.recv()` loop.
//!
//! Sends a `value` output every 100ms in a tight loop and never polls
//! the event stream. Because the cooperative `Event::Stop` is delivered
//! over the event stream, this node CANNOT honor it — the daemon's only
//! way to terminate it is SIGTERM (after the grace period) followed by
//! SIGKILL.
//!
//! Paired with `tests/fault-tolerance-e2e.rs` to prove the daemon
//! reports SIGTERM-induced exits as clean when the dataflow was stopped
//! by the operator (`dora run --stop-after`). See dora-rs/dora#1882.

use dora_node_api::{dora_core::config::DataId, DoraNode, IntoArrow};
use eyre::Context;
use std::{thread, time::Duration};

fn main() -> eyre::Result<()> {
    let (mut node, _events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let output = DataId::from("value".to_owned());

    // 1000 iterations * 100ms = 100s of work. The test runs us with
    // --stop-after well below that, so we'll always be SIGTERMed
    // mid-loop.
    for i in 0..1000_i64 {
        node.send_output(output.clone(), Default::default(), i.into_arrow())
            .context("failed to send value")?;
        thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}
