//! Test-only node that appends every param event it receives to
//! `$DORA_TEST_MARKER_FILE`: `pid <pid>` on start, then `set <key> <json>` or
//! `del <key>`, one per line, flushed immediately.

use std::io::Write;

use dora_node_api::{DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let path = std::env::var("DORA_TEST_MARKER_FILE")
        .context("DORA_TEST_MARKER_FILE env var must be set")?;
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .with_context(|| format!("failed to open {path:?}"))?;
    writeln!(file, "pid {}", std::process::id())?;
    file.flush()?;

    while let Some(event) = events.recv() {
        match event {
            Event::ParamUpdate { key, value } => writeln!(file, "set {key} {value}")?,
            Event::ParamDeleted { key } => writeln!(file, "del {key}")?,
            Event::Stop(_) => break,
            _ => continue,
        }
        file.flush()?;
    }
    Ok(())
}
