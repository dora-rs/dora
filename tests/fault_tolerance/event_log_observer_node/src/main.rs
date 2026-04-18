//! Record every notable `Event` variant to `$DORA_TEST_MARKER_FILE`.
//!
//! Unlike `input-closed-observer-node` (which exits on the first
//! `InputClosed`), this observer keeps running until it sees `Stop`.
//! That's required for tests that need to observe events which arrive
//! *after* a prior `InputClosed` — for example `NodeRestarted` on
//! upstream crash+restart, or `InputRecovered` when the upstream
//! starts emitting again after an `input_timeout` break.

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
        let line: String = match &event {
            Event::Input { id, .. } => format!("Input:{id}\n"),
            Event::InputClosed { id } => format!("InputClosed:{id}\n"),
            Event::InputRecovered { id } => format!("InputRecovered:{id}\n"),
            Event::NodeRestarted { id } => format!("NodeRestarted:{id}\n"),
            Event::Stop(_) => "Stop\n".to_string(),
            _ => continue,
        };
        marker
            .write_all(line.as_bytes())
            .context("failed to append event marker")?;
        marker.flush().ok();
        if matches!(event, Event::Stop(_)) {
            break;
        }
    }
    Ok(())
}
