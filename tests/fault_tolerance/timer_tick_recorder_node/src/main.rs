//! Test-only node that records every input event it receives, so a test
//! can assert that an input actually *fires* rather than that the node
//! merely came up.
//!
//! Motivating case (dora-rs/dora#2585): a node added to an
//! already-running dataflow via `dora node add` whose timer input uses an
//! interval no existing node subscribes to used to be starved of ticks
//! forever, because per-interval tick-emitting tasks are only ever
//! spawned in `RunningDataflow::start()`. Such a node spawns fine and
//! `dora node list` reports it as `Running`, so the silence is invisible
//! from the daemon side — the only observable is a delivery the node
//! itself reports.
//!
//! * `DORA_TEST_MARKER_FILE` (required) — path appended with one line per
//!   received input, holding that input's id. Created (empty) at startup
//!   so a test can tell "node came up but no input arrived" apart from
//!   "node never came up". Same marker-file convention as
//!   `stop_delay_node` / `clean_exit_node`.
//!
//! Exits on `Event::Stop`.

use dora_node_api::{DoraNode, Event};
use eyre::Context;
use std::{io::Write, thread, time::Duration};

/// Absolute lifetime cap. Nothing in the test path should reach this —
/// the node exits on `Stop` — but an orphaned incarnation (killed daemon,
/// timed-out CI step, severed connection that never yields `None`) would
/// otherwise sit resident forever holding its shared-memory pool.
/// `stop_delay_node` bounds itself the same way.
const MAX_LIFETIME: Duration = Duration::from_secs(300);

fn main() -> eyre::Result<()> {
    let marker =
        std::env::var("DORA_TEST_MARKER_FILE").context("DORA_TEST_MARKER_FILE must be set")?;

    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    // Opened once, before any input can arrive, so the file's existence
    // means "this node connected" and its emptiness means "connected but
    // nothing was delivered" — the exact distinction #2585 turns on.
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&marker)
        .with_context(|| format!("failed to open marker file {marker:?}"))?;

    thread::spawn(|| {
        thread::sleep(MAX_LIFETIME);
        eprintln!("timer-tick-recorder-node: lifetime cap reached, exiting");
        std::process::exit(0);
    });

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, .. } => {
                writeln!(file, "{id}").context("failed to record input")?;
                // Flushed per line: the test polls this file while the
                // node is still running, so a buffered write it never
                // sees is indistinguishable from an input that never
                // arrived.
                file.flush().context("failed to flush marker file")?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}
