//! Test-only node that exits cleanly after its first tick, once per
//! incarnation.
//!
//! Writes a single `incarnation\n` line to the file at `$DORA_TEST_MARKER_FILE`
//! *before* exiting, so the test harness can count how many times the node
//! was spawned. With `restart_policy: always` + `max_restarts: N`, the file
//! should grow to `N + 1` lines (initial spawn + N restarts). With
//! `restart_policy: never` or `on-failure`, it would contain exactly one.

use std::io::Write;

use dora_node_api::DoraNode;
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
    marker
        .write_all(b"incarnation\n")
        .context("failed to append incarnation marker")?;

    // Wait for one event (tick), then exit cleanly.
    let _ = events.recv();
    Ok(())
}
