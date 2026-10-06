//! Emit a numbered burst on the first tick (after `DORA_TEST_BURST_AFTER_MS`,
//! if set — the first timer tick fires at once), then exit — or, with
//! `DORA_TEST_BURST_LINGER` set, stay alive and idle until `Stop`.
//!
//! The burst is unpaced on purpose: it is the shape of a
//! `dora replay --speed 0` producer, and the point of the fixture is to
//! push far more messages at the consumer than any daemon-side buffer
//! holds, so a drop anywhere on the daemon path shows up as a gap in
//! the sequence the sink receives. Lingering models a producer that emits
//! and then waits for something else: whatever the daemon is still holding
//! for it must reach the consumer without the producer sending again.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let count: i64 = std::env::var("DORA_TEST_BURST_COUNT")
        .context("DORA_TEST_BURST_COUNT env var must be set by the fixture")?
        .parse()
        .context("DORA_TEST_BURST_COUNT must be an i64")?;

    let linger = std::env::var_os("DORA_TEST_BURST_LINGER").is_some();
    let after = std::time::Duration::from_millis(
        std::env::var("DORA_TEST_BURST_AFTER_MS")
            .unwrap_or_else(|_| "0".to_owned())
            .parse()
            .context("DORA_TEST_BURST_AFTER_MS must be a u64")?,
    );

    let (mut node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let output = DataId::from("value".to_owned());

    // Wait for the first tick so the consumer is subscribed before the
    // burst starts; a message sent before that is dropped for a reason
    // unrelated to what this fixture tests.
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                std::thread::sleep(after);
                for seq in 0..count {
                    node.send_output(
                        output.clone(),
                        metadata.parameters.clone(),
                        seq.into_arrow(),
                    )
                    .with_context(|| format!("failed to send value #{seq}"))?;
                }
                eprintln!("burst-source: sent {count} messages");
                if !linger {
                    return Ok(());
                }
                while let Some(event) = events.recv() {
                    if matches!(event, Event::Stop(_)) {
                        break;
                    }
                }
                return Ok(());
            }
            Event::Stop(_) => return Ok(()),
            _ => {}
        }
    }
    Ok(())
}
