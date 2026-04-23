//! Emit `value` twice with a silence gap in between.
//!
//! Between the two emits the node is alive but not producing — long
//! enough for the downstream's `input_timeout` to fire, break the
//! input, and deliver `InputClosed`. The second emit re-arms the
//! input, which the daemon surfaces as `InputRecovered` (see
//! binaries/daemon/src/lib.rs:4095).

use std::time::{Duration, Instant};

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let output = DataId::from("value".to_owned());

    let gap_ms: u64 = std::env::var("DORA_TEST_BURST_GAP_MS")
        .context("DORA_TEST_BURST_GAP_MS env var must be set by the fixture")?
        .parse()
        .context("DORA_TEST_BURST_GAP_MS must be a u64")?;

    let mut emitted = 0;
    let mut next_emit_after: Option<Instant> = None;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                if emitted >= 2 {
                    continue;
                }
                let ok = match next_emit_after {
                    None => true,
                    Some(t) => Instant::now() >= t,
                };
                if !ok {
                    continue;
                }
                node.send_output(output.clone(), metadata.parameters, 42i64.into_arrow())
                    .context("failed to send value")?;
                emitted += 1;
                next_emit_after = Some(Instant::now() + Duration::from_millis(gap_ms));
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
