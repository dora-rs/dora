//! Records how many data inputs it received, then exits when drained.
//!
//! Exists because asserting "the dataflow terminated" is not enough to
//! show that `dora run --exit-when-nodes-finish` is correct: draining a
//! node at startup, before its upstream ever sent anything, also
//! terminates and also exits 0. The interesting property is that the
//! node finished its work *first*.
//!
//! Writes the received-input count to the path in
//! `DORA_TEST_DRAIN_RECORD` on exit, so the test can assert on it.
//! Unlike `examples/rust-dynamic-add-remove/receiver`, this node exits
//! on `AllInputsClosed` — that is the event under test, so it must not
//! depend on the binding happening to close the event stream
//! (dora-rs/dora#2920).

use dora_node_api::{DoraNode, Event, StopCause};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut inputs = 0_u64;
    let mut drained = false;

    while let Some(event) = events.recv() {
        match event {
            // Count ONLY the data input. Counting every input would
            // include timer ticks, so "received something" would be
            // satisfied by the clock alone and the test could pass on a
            // run where the payload never arrived.
            Event::Input { ref id, .. } if id.as_str() == "value" => inputs += 1,
            Event::Stop(StopCause::AllInputsClosed) => {
                drained = true;
                break;
            }
            Event::Stop(StopCause::Manual) => break,
            _ => {}
        }
    }

    if let Ok(path) = std::env::var("DORA_TEST_DRAIN_RECORD") {
        std::fs::write(&path, format!("{inputs} {drained}"))
            .with_context(|| format!("failed to write drain record to {path}"))?;
    }
    Ok(())
}
