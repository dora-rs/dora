//! Record every `Input` / `InputClosed` the daemon delivers.
//!
//! Appends one line per event to `$DORA_TEST_MARKER_FILE` so the test
//! harness can assert the specific sequence without parsing logs. Exits
//! cleanly as soon as it sees `InputClosed` on the watched input, which
//! keeps the test short when `input_timeout` fires as expected.

use std::io::Write;

use dora_node_api::{DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    let marker_path = std::env::var("DORA_TEST_MARKER_FILE")
        .context("DORA_TEST_MARKER_FILE env var must be set by the fixture")?;
    let mut marker = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&marker_path)
        .with_context(|| format!("failed to open marker file {marker_path:?}"))?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, .. } => {
                writeln!(marker, "Input:{id}").context("failed to log Input event")?;
            }
            Event::InputClosed { id } => {
                writeln!(marker, "InputClosed:{id}").context("failed to log InputClosed event")?;
                // First InputClosed is the signal we're waiting for —
                // exiting keeps the test short.
                break;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
