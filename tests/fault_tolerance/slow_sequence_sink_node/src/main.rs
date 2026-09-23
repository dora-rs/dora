//! Consume numbered inputs after a one-off stall and record what arrived.
//!
//! The stall (`$DORA_TEST_SINK_STALL_MS`, on the first input) models a
//! consumer busy with real work — inference, disk I/O — while an unpaced
//! producer keeps sending. During it the node calls nothing on its event
//! stream, so every buffer between the producer and this thread fills:
//! the ingress channel, the daemon listener's queue, the daemon's per-node
//! channel. What the daemon does when the last of those is full is the
//! property under test.
//!
//! Writes `<received> <gaps>` to `$DORA_TEST_SINK_RECORD` once the event
//! stream ends. It deliberately keeps receiving through `InputClosed` and
//! `Stop`: the scheduler yields lifecycle events ahead of the inputs still
//! queued behind them, so exiting on either would cut the count short for
//! a reason unrelated to delivery. The stream returns `None` once the
//! closed input's backlog is drained. `gaps` counts every input whose
//! sequence number is not the previous one plus one, so a single
//! dropped message anywhere in the stream is visible even when the
//! total looks plausible.

use std::time::Duration;

use dora_node_api::{DoraNode, Event, arrow_v59::array::Int64Array};
use eyre::{Context, ContextCompat};

fn main() -> eyre::Result<()> {
    let stall = Duration::from_millis(
        std::env::var("DORA_TEST_SINK_STALL_MS")
            .context("DORA_TEST_SINK_STALL_MS env var must be set by the fixture")?
            .parse()
            .context("DORA_TEST_SINK_STALL_MS must be a u64")?,
    );
    let record = std::env::var("DORA_TEST_SINK_RECORD")
        .context("DORA_TEST_SINK_RECORD env var must be set by the fixture")?;

    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    let mut received: u64 = 0;
    let mut gaps: u64 = 0;
    let mut expected: i64 = 0;
    while let Some(event) = events.recv() {
        let Event::Input { id, data, .. } = event else {
            continue;
        };
        if id.as_str() != "value" {
            continue;
        }
        let seq = data
            .as_array()
            .as_any()
            .downcast_ref::<Int64Array>()
            .context("expected Int64Array")?
            .value(0);
        if seq != expected {
            gaps += 1;
            eprintln!("slow-sink: gap: expected #{expected}, got #{seq}");
        }
        expected = seq + 1;
        received += 1;
        if received == 1 {
            std::thread::sleep(stall);
        }
    }

    eprintln!("slow-sink: received {received} messages, {gaps} gaps");
    std::fs::write(&record, format!("{received} {gaps}"))
        .with_context(|| format!("failed to write sink record to {record}"))?;
    Ok(())
}
