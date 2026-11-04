//! `dora daemon` must not run the daemon on the main thread's stack, which is
//! only 1 MiB on Windows (dora-rs/dora#4011; see `DAEMON_THREAD_STACK_SIZE` in
//! `src/command/daemon.rs`). `cargo test` builds the CLI in debug, the build
//! that overflowed, so on Windows this runs exactly the binary that crashed.
//! Elsewhere it checks that `--run-dataflow` runs a dataflow to the end.

use std::{
    fs::{self, File},
    process::Command,
    thread,
    time::{Duration, Instant},
};

/// Generous: the run below takes about a second.
const TIMEOUT: Duration = Duration::from_secs(120);

#[test]
fn run_dataflow_does_not_overflow_the_main_thread_stack() {
    let dir = tempfile::tempdir().unwrap();
    let dora = env!("CARGO_BIN_EXE_dora");

    // A single node that needs nothing built: the `dora` binary itself, which
    // prints its version and exits without connecting. The daemon sets up its
    // zenoh session and event loop -- where the overflow happened -- before it
    // spawns the node, and the dataflow then finishes on its own.
    let dataflow = dir.path().join("dataflow.yml");
    fs::write(
        &dataflow,
        format!(
            "nodes:\n  - id: version\n    path: '{}'\n    args: --version\n",
            dora.replace('\'', "''")
        ),
    )
    .unwrap();

    // Output goes to a file, not a pipe: nothing drains a pipe while the loop
    // below polls, and a full one would block the daemon.
    let log_path = dir.path().join("daemon.log");
    let log = File::create(&log_path).unwrap();
    let mut daemon = Command::new(dora)
        .current_dir(dir.path())
        .args(["daemon", "--run-dataflow"])
        .arg(&dataflow)
        .stdout(log.try_clone().unwrap())
        .stderr(log)
        .spawn()
        .unwrap();

    let deadline = Instant::now() + TIMEOUT;
    let status = loop {
        if let Some(status) = daemon.try_wait().unwrap() {
            break Some(status);
        }
        if Instant::now() >= deadline {
            let _ = daemon.kill();
            let _ = daemon.wait();
            break None;
        }
        thread::sleep(Duration::from_millis(50));
    };

    let log = fs::read_to_string(&log_path).unwrap_or_default();
    let status = status.unwrap_or_else(|| {
        panic!("`dora daemon --run-dataflow` did not exit within {TIMEOUT:?}\n{log}")
    });
    assert!(
        status.success(),
        "`dora daemon --run-dataflow` failed with {status} \
         (0xc00000fd is STATUS_STACK_OVERFLOW on Windows)\n{log}"
    );
}
