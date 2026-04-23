//! Test-only node that panics on the first input of every incarnation.
//!
//! Paired with `tests/dataflows/max-restarts-truly-exhausted.yml` and
//! `tests/fault-tolerance-e2e.rs::max_restarts_exhaustion_marks_node_failed`.
//!
//! The shared `rust-dataflow-example-status-node` only crashes on
//! `restart_count() == 0` so it can show off *successful* recovery.
//! That makes it unsuitable for exhaustion testing: `max_restarts` is
//! never reached, and the existing e2e test has to settle for a
//! permissive "both Ok and Err are valid" assertion. This node crashes
//! every time so the restart budget is guaranteed to be consumed.

use dora_node_api::{DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    while let Some(event) = events.recv() {
        if let Event::Input { id, .. } = event
            && id.as_str() == "tick"
        {
            panic!("always-crash-node: simulated failure on tick (every incarnation)");
        }
    }
    Ok(())
}
