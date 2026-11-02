//! Sends a large `frame` output on every `tick` input.
//!
//! * `DORA_TEST_OUTPUT_BYTES` (required) — size of each `frame` in bytes.
//!
//! Honors `Event::Stop`, so any delay in stopping a dataflow built on it is
//! the control plane's, not this node's.

use dora_node_api::{DoraNode, Event, dora_core::config::DataId};
use eyre::Context;
use std::{thread, time::Duration};

/// Absolute lifetime cap, so an orphaned incarnation (killed daemon,
/// timed-out CI step) does not keep producing forever. Same bound as
/// `stop-delay-node`.
const MAX_LIFETIME: Duration = Duration::from_secs(300);

fn main() -> eyre::Result<()> {
    let output_bytes: usize = std::env::var("DORA_TEST_OUTPUT_BYTES")
        .context("DORA_TEST_OUTPUT_BYTES env var must be set by the fixture")?
        .parse()
        .context("DORA_TEST_OUTPUT_BYTES must be a usize")?;

    let (mut node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let output = DataId::from("frame".to_owned());
    let frame = vec![0u8; output_bytes];

    thread::spawn(|| {
        thread::sleep(MAX_LIFETIME);
        eprintln!("large-output-source-node: lifetime cap reached, exiting");
        std::process::exit(0);
    });

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                node.send_output_bytes(output.clone(), metadata.parameters, frame.len(), &frame)
                    .context("failed to send frame")?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}
