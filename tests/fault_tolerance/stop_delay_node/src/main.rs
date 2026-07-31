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
//!
//! Declares no inputs, so `events.recv()` blocks until the daemon sends
//! `Stop` — the node stays alive until the test removes it.

use dora_node_api::{DoraNode, Event};
use eyre::Context;
use std::{thread, time::Duration};

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    // Unset means "default"; set-but-unparseable is a fixture bug and
    // must fail loudly rather than silently fall back to the default.
    let delay_ms: u64 = std::env::var("DORA_TEST_STOP_DELAY_MS")
        .map_or(Ok(0), |raw| raw.parse())
        .context("DORA_TEST_STOP_DELAY_MS must be a u64")?;
    let exit_code: i32 = std::env::var("DORA_TEST_STOP_EXIT_CODE")
        .map_or(Ok(0), |raw| raw.parse())
        .context("DORA_TEST_STOP_EXIT_CODE must be an i32")?;

    while let Some(event) = events.recv() {
        if matches!(event, Event::Stop(_)) {
            break;
        }
    }

    thread::sleep(Duration::from_millis(delay_ms));
    // `process::exit` rather than returning: the exit status has to be
    // exactly what the fixture asked for, with no destructor-driven
    // handshake changing the timing the test is pinning.
    std::process::exit(exit_code);
}
