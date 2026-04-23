//! Test-only node: appends an incarnation marker, sleeps
//! `$DORA_TEST_CRASH_AFTER_MS` ms, then panics.
//!
//! Paired with `tests/dataflows/restart-window-reset.yml` and
//! `fault-tolerance-e2e.rs::restart_window_resets_restart_counter` (#1631).
//!
//! Each incarnation lives long enough to exceed the fixture's
//! `restart_window`, so the daemon's window reset path fires on every
//! crash — the node would otherwise exhaust `max_restarts` in a handful
//! of incarnations.

use std::io::Write;
use std::time::Duration;

use dora_node_api::DoraNode;
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, _events) =
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
    drop(marker);

    let delay_ms: u64 = std::env::var("DORA_TEST_CRASH_AFTER_MS")
        .context("DORA_TEST_CRASH_AFTER_MS env var must be set by the fixture")?
        .parse()
        .context("DORA_TEST_CRASH_AFTER_MS must be a u64")?;
    std::thread::sleep(Duration::from_millis(delay_ms));
    panic!("delayed-crash-node: simulated failure after {delay_ms} ms uptime");
}
