//! Emit a numbered burst on the first tick, then exit.
//!
//! The burst is unpaced on purpose: it is the shape of a
//! `dora replay --speed 0` producer, and the point of the fixture is to
//! push far more messages at the consumer than any daemon-side buffer
//! holds, so a drop anywhere on the daemon path shows up as a gap in
//! the sequence the sink receives.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let count: i64 = std::env::var("DORA_TEST_BURST_COUNT")
        .context("DORA_TEST_BURST_COUNT env var must be set by the fixture")?
        .parse()
        .context("DORA_TEST_BURST_COUNT must be an i64")?;

    let (mut node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let output = DataId::from("value".to_owned());

    // Wait for the first tick so the consumer is subscribed before the
    // burst starts; a message sent before that is dropped for a reason
    // unrelated to what this fixture tests.
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                for seq in 0..count {
                    node.send_output(
                        output.clone(),
                        metadata.parameters.clone(),
                        seq.into_arrow(),
                    )
                    .with_context(|| format!("failed to send value #{seq}"))?;
                }
                eprintln!("burst-source: sent {count} messages");
                return Ok(());
            }
            Event::Stop(_) => return Ok(()),
            _ => {}
        }
    }
    Ok(())
}
