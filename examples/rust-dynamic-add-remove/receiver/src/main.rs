//! Receiver node for the `dora node` lifecycle E2E test fixture
//! (issue #1703). Consumes inputs and logs them; tolerant of both
//! the static `value` input from sender and the dynamic `filtered`
//! input wired in at runtime via `dora node connect`.
//!
//! Only exits on `StopCause::Manual` (operator-issued `dora stop`/
//! `dora node stop`/ctrl-c). The dora-rs Rust binding fabricates an
//! `Event::Stop(StopCause::AllInputsClosed)` when every input has been
//! torn down; treating that as a real stop would let the receiver die
//! when sender is stopped via `dora node stop`, leaving no node for
//! the lifecycle harness to `dora node restart` against. The Python
//! receiver in `examples/dynamic-add-remove/receiver.py` exhibits the
//! same selective behavior (its `event["type"] == "STOP"` arm only
//! matches the operator-Manual stop because the binding uses a
//! different `"type"` string for AllInputsClosed).

use dora_node_api::{DoraNode, Event, StopCause};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut count: u64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, .. } => {
                count += 1;
                if count.is_multiple_of(10) {
                    eprintln!("receiver: {} messages (latest input: {})", count, id);
                }
            }
            Event::Stop(StopCause::Manual) => break,
            // `AllInputsClosed` fires when sender is operator-stopped
            // before us. Keep the receiver alive so the lifecycle test
            // can still exercise `dora node restart` on it.
            _ => {}
        }
    }
    eprintln!("receiver: done ({} messages)", count);
    Ok(())
}
