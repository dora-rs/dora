//! A node that can only be stopped by SIGKILL.
//!
//! It ignores SIGTERM and never polls its event stream, so neither the
//! cooperative `Stop` event nor the daemon's SIGTERM rung can end it.
//! That makes it the only way to observe whether the stop ladder
//! actually reaches its final SIGKILL rung: a node that dies to SIGTERM
//! would be cleaned up either way and cannot distinguish the two
//! (dora-rs/dora#2920).
//!
//! Unix-only in substance — the property it fakes is a signal
//! disposition, which Windows has no equivalent for, and its only
//! caller (`run_killed_by_sigterm_terminates_nodes_and_exits` in
//! `tests/node-lifecycle-e2e.rs`) is `#[cfg(unix)]`. Cargo has no
//! cfg-conditional `members`, so it stays a workspace member on every
//! platform and must still *compile* on Windows for the nightly's
//! `cargo test --all` to resolve — hence the stub `main` below rather
//! than a `#![cfg(unix)]` on the crate, which would leave a bin target
//! with no `main` at all.

#[cfg(unix)]
fn main() -> eyre::Result<()> {
    use dora_node_api::DoraNode;
    use eyre::ensure;
    use std::{thread, time::Duration};

    // Installed BEFORE `init_from_env`, which spawns the event-stream
    // threads, the node's tokio runtime and zenoh's threads: `signal` is
    // only well specified in a single-threaded process.
    //
    // SAFETY: setting a disposition to `SIG_IGN` runs no handler, and no
    // other thread exists yet to observe the change.
    let previous = unsafe { libc::signal(libc::SIGTERM, libc::SIG_IGN) };
    // Checked, because a silent failure here would leave a node that
    // dies to SIGTERM — which still passes the test, but via the wrong
    // rung, quietly costing the fixture the only property it has.
    ensure!(
        previous != libc::SIG_ERR,
        "failed to ignore SIGTERM: {}",
        std::io::Error::last_os_error()
    );

    let (_node, _events) = DoraNode::init_from_env()?;

    // Publish our pid so the test can assert on THIS process rather than
    // pattern-matching the process table, which would also catch a
    // leftover from an earlier run and report a false failure.
    //
    // Written to a temp path and renamed, so a reader polling for the
    // file can never observe a half-written pid and parse it as a
    // different, valid-looking process.
    if let Ok(path) = std::env::var("DORA_TEST_PID_FILE") {
        let tmp = format!("{path}.tmp");
        std::fs::write(&tmp, std::process::id().to_string())?;
        std::fs::rename(&tmp, &path)?;
    }

    // Never read `_events`, so the cooperative Stop cannot be honored
    // either.
    //
    // This MUST outlast the test's teardown deadline. Shortening it so a
    // stranded fixture self-limits looks tidy and silently guts the
    // test: if the node exits on its own inside the deadline, a wedged
    // teardown becomes indistinguishable from a working one, and the
    // mutation that should fail passes. Cleanup is the harness's job —
    // the test kills this process group on every exit path.
    for _ in 0..600 {
        thread::sleep(Duration::from_millis(500));
    }
    Ok(())
}

/// Compiles, never runs: nothing spawns this fixture on Windows.
///
/// It exits non-zero rather than sleeping like the Unix build, so a
/// future Windows caller fails loudly instead of silently observing a
/// node that ignores nothing and calling the stop ladder proven.
#[cfg(not(unix))]
fn main() {
    eprintln!(
        "sigterm-ignoring-node is a Unix-only test fixture: it fakes a SIGTERM \
         disposition, which Windows has no equivalent for (dora-rs/dora#2920)"
    );
    std::process::exit(1);
}
