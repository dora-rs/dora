//! Long-running node for the daemon coordinator-reconnect e2e
//! (dora-rs/dora#2029).
//!
//! On startup it records its OS process id (one line, appended) to the
//! file named by `$DORA_TEST_PID_FILE`, then keeps draining inputs until
//! it receives `Stop`. Staying responsive matters: a node that stopped
//! pulling events would let the daemon's health-check watchdog kill it,
//! which would mask the very behavior under test.
//!
//! The test reads the pid file after killing and restarting the
//! coordinator. The fix is proven by two facts together: the file holds
//! exactly one pid (the node was never killed + respawned) and that pid
//! is still alive (the node survived the reconnect).

use std::io::Write;

use dora_node_api::{DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    let pid_path = std::env::var("DORA_TEST_PID_FILE")
        .context("DORA_TEST_PID_FILE env var must be set by the fixture")?;
    let mut pid_file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&pid_path)
        .with_context(|| format!("failed to open pid file {pid_path:?}"))?;
    writeln!(pid_file, "{}", std::process::id()).context("failed to write pid")?;
    pid_file.flush().ok();
    drop(pid_file);

    while let Some(event) = events.recv() {
        if matches!(event, Event::Stop(_)) {
            break;
        }
    }
    Ok(())
}
