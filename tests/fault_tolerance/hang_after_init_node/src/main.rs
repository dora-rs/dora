//! Register as a dora node, pull one event to advance the daemon's
//! `last_activity` past its unarmed-zero state, write an incarnation
//! marker, then block without draining further events.
//!
//! The daemon's health check skips nodes whose `last_activity == 0`
//! ("not yet connected" — binaries/daemon/src/lib.rs:1925), so a node
//! that registers and immediately hangs is invisible to the health
//! check. Pumping one event first guarantees `last_activity` holds a
//! real timestamp, after which the watchdog clock can legitimately
//! fire when we go silent.
//!
//! Each incarnation writes a new marker line, so the test can count
//! how many times the daemon had to kill-and-respawn the node.

use std::io::Write;
use std::time::Duration;

use dora_node_api::DoraNode;
use eyre::Context;

fn main() -> eyre::Result<()> {
    // Write the marker BEFORE `init_from_env` so we can tell whether
    // the node at least got spawned, separate from whether init
    // succeeded. init_from_env is known to take ~500 ms on a cold
    // process spawn (zenoh session setup, subscribe, thread spawn),
    // which can race the 0.5 s health-check window.
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

    let (node, events) = DoraNode::init_from_env().context("failed to init dora node from env")?;

    // Kill the node_api's background event_stream thread by dropping
    // the EventStream + DoraNode. Otherwise that thread keeps sending
    // `NextEvent` requests (apis/rust/node/src/event_stream/thread.rs)
    // which refresh `last_activity` on the daemon and keep the health
    // check from firing. After drop, no daemon requests are in flight,
    // so `last_activity` goes stale and the daemon's check_node_health
    // watchdog can finally SIGKILL us.
    drop(events);
    drop(node);

    std::thread::sleep(Duration::from_secs(60 * 60));
    Ok(())
}
