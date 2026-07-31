//! Test-only node that honors `Event::Stop` but controls *when* and
//! *how* it exits.
//!
//! A `dora node remove` immediately followed by a `dora node add` of the
//! same id (dora-rs/dora#2916) races the removed process's exit against
//! the new incarnation's spawn. Which side of the re-add that exit lands
//! on decides which daemon code path accounts it, so a test that wants
//! to pin one ordering needs a node whose shutdown latency it controls:
//!
//! * `DORA_TEST_STOP_DELAY_MS` (default 0) — how long to linger after
//!   `Stop` before exiting. Large enough and the exit lands *after* the
//!   re-add (the shape the issue reported, where a Python node's
//!   interpreter teardown outlived the CLI round trip); zero and it
//!   lands *before*.
//! * `DORA_TEST_STOP_EXIT_CODE` (default 0) — the exit status, so a test
//!   can distinguish a clean shutdown from a node that fails on its way
//!   out.
//! * `DORA_TEST_MARKER_FILE` (optional) — path to write the resolved
//!   exit code to, immediately before exiting. Lets a test assert that
//!   this fixture really did exit the way the test's premise assumes,
//!   rather than inferring it from daemon-side state that a removed
//!   node no longer has (`dora node list` drops the row on removal).
//!   Same marker-file convention as `clean_exit_node` and
//!   `delayed_crash_node`.
//!
//! Declares no inputs, so `events.recv()` blocks until the daemon sends
//! `Stop` — the node stays alive until the test removes it.

use dora_node_api::{DoraNode, Event};
use eyre::Context;
use std::{io::Write, thread, time::Duration};

/// Absolute lifetime cap. Nothing in the test path should reach this —
/// the node exits on `Stop` — but an orphaned incarnation (killed
/// daemon, timed-out CI step, severed connection that never yields
/// `None`) would otherwise sit resident forever holding its
/// shared-memory pool. `fire-and-forget-source-node` bounds itself the
/// same way, at 100s of wall-clock work.
const MAX_LIFETIME: Duration = Duration::from_secs(300);

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    // Unset (or set to non-UTF8) means "default"; a set-but-unparseable
    // value is a fixture bug and fails loudly rather than silently
    // falling back.
    let delay_ms: u64 = std::env::var("DORA_TEST_STOP_DELAY_MS")
        .map_or(Ok(0), |raw| raw.parse())
        .context("DORA_TEST_STOP_DELAY_MS must be a u64")?;
    let exit_code: i32 = std::env::var("DORA_TEST_STOP_EXIT_CODE")
        .map_or(Ok(0), |raw| raw.parse())
        .context("DORA_TEST_STOP_EXIT_CODE must be an i32")?;
    let marker = std::env::var("DORA_TEST_MARKER_FILE").ok();

    thread::spawn(|| {
        thread::sleep(MAX_LIFETIME);
        eprintln!("stop-delay-node: lifetime cap reached, exiting");
        std::process::exit(0);
    });

    while let Some(event) = events.recv() {
        if matches!(event, Event::Stop(_)) {
            break;
        }
    }

    thread::sleep(Duration::from_millis(delay_ms));

    if let Some(path) = marker {
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
            .with_context(|| format!("failed to open marker file {path:?}"))?;
        writeln!(file, "{exit_code}").context("failed to write exit marker")?;
        file.flush().context("failed to flush exit marker")?;
    }

    // `process::exit` rather than returning: the exit status has to be
    // exactly what the fixture asked for, with no destructor-driven
    // handshake changing the timing the test is pinning.
    std::process::exit(exit_code);
}
