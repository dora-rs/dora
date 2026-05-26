use std::io::{IsTerminal, Write};

use clap::Args;
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

use crate::{
    command::{Executable, default_tracing},
    common::{
        CoordinatorOptions, connect_to_coordinator, expect_reply, query_running_dataflows,
        send_control_request,
    },
    ws_client::WsSession,
};
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::DataflowStatus};
use eyre::Context;

/// Run comprehensive system diagnostics.
///
/// Checks coordinator, daemon, connected machines, running dataflows,
/// and per-node health. Returns non-zero exit code if any check fails.
///
/// Examples:
///
/// Run diagnostics:
///   dora doctor
///
/// Include dataflow validation:
///   dora doctor --dataflow dataflow.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Doctor {
    /// Path to a dataflow YAML to validate
    #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: Option<std::path::PathBuf>,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Doctor {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        doctor(self)
    }
}

fn doctor(args: Doctor) -> eyre::Result<()> {
    let color_choice = if std::io::stdout().is_terminal() {
        ColorChoice::Auto
    } else {
        ColorChoice::Never
    };
    let mut stdout = termcolor::StandardStream::stdout(color_choice);
    let mut failures = 0u32;

    // 1. CLI version
    pass(
        &mut stdout,
        &format!("CLI version: {}", env!("CARGO_PKG_VERSION")),
    )?;

    // Shared memory permissions (Linux only). dora uses /dev/shm for the
    // zero-copy SHM fast path on large payloads, but the node runtime
    // gracefully falls back to heap-buffered zenoh publishes when SHM is
    // unavailable, so a misconfigured /dev/shm is a perf concern, not a
    // correctness one — surface it as a WARN.
    #[cfg(target_os = "linux")]
    check_shared_memory(&mut stdout)?;

    // `uv` availability. `dora build --uv` and the managed Python env flow
    // depend on the `uv` binary being on PATH. Pure Rust/C++ users do not
    // need it, so surface as WARN, not FAIL.
    check_uv(&mut stdout)?;

    // 2. Coordinator connectivity
    let addr = args.coordinator.socket_addr();
    let session = match connect_to_coordinator(addr) {
        Ok(session) => {
            pass(&mut stdout, &format!("Coordinator: reachable at {addr}"))?;
            Some(session)
        }
        Err(_) => {
            fail(
                &mut stdout,
                &format!("Coordinator: not reachable at {addr}"),
            )?;
            failures += 1;
            None
        }
    };

    // 3. Daemon connectivity
    if let Some(ref session) = session {
        match check_daemon(session) {
            Ok(true) => pass(&mut stdout, "Daemon: connected")?,
            Ok(false) => {
                fail(&mut stdout, "Daemon: not connected")?;
                failures += 1;
            }
            Err(e) => {
                fail(&mut stdout, &format!("Daemon: check failed ({e})"))?;
                failures += 1;
            }
        }
    }

    // 4. Connected machines
    if let Some(ref session) = session {
        match check_connected_machines(session) {
            Ok(machines) => {
                if machines.is_empty() {
                    warn(&mut stdout, "Connected machines: none")?;
                } else {
                    pass(
                        &mut stdout,
                        &format!("Connected machines: {}", machines.len()),
                    )?;
                    for m in &machines {
                        let heartbeat_str = format!("{}ms ago", m.last_heartbeat_ago_ms);
                        writeln!(stdout, "    {} (heartbeat: {heartbeat_str})", m.daemon_id)?;
                    }
                }
            }
            Err(e) => {
                fail(
                    &mut stdout,
                    &format!("Connected machines: check failed ({e})"),
                )?;
                failures += 1;
            }
        }
    }

    // 5. Active dataflows
    if let Some(ref session) = session {
        match query_running_dataflows(session) {
            Ok(list) => {
                let running = list
                    .0
                    .iter()
                    .filter(|d| d.status == DataflowStatus::Running)
                    .count();
                let failed = list
                    .0
                    .iter()
                    .filter(|d| d.status == DataflowStatus::Failed)
                    .count();
                let total = list.0.len();
                pass(
                    &mut stdout,
                    &format!("Dataflows: {running} running, {failed} failed, {total} total"),
                )?;
            }
            Err(e) => {
                fail(&mut stdout, &format!("Dataflows: query failed ({e})"))?;
                failures += 1;
            }
        }
    }

    // 6. Per-node health
    if let Some(ref session) = session {
        match check_node_health(session) {
            Ok((total, healthy, degraded, failed_nodes)) => {
                if failed_nodes > 0 {
                    fail(
                        &mut stdout,
                        &format!(
                            "Nodes: {total} total, {healthy} healthy, {degraded} degraded, {failed_nodes} failed"
                        ),
                    )?;
                    failures += 1;
                } else if degraded > 0 {
                    warn(
                        &mut stdout,
                        &format!("Nodes: {total} total, {healthy} healthy, {degraded} degraded"),
                    )?;
                } else if total > 0 {
                    pass(&mut stdout, &format!("Nodes: {total} total, all healthy"))?;
                }
            }
            Err(e) => {
                fail(&mut stdout, &format!("Nodes: check failed ({e})"))?;
                failures += 1;
            }
        }
    }

    // 7. Optional dataflow validation
    if let Some(dataflow_path) = &args.dataflow {
        match validate_dataflow(dataflow_path) {
            Ok(()) => pass(
                &mut stdout,
                &format!("Dataflow validation: {} OK", dataflow_path.display()),
            )?,
            Err(e) => {
                fail(
                    &mut stdout,
                    &format!(
                        "Dataflow validation: {} FAILED: {e}",
                        dataflow_path.display()
                    ),
                )?;
                failures += 1;
            }
        }
    }

    writeln!(stdout)?;
    if failures > 0 {
        eyre::bail!("{failures} check(s) failed");
    }
    pass(&mut stdout, "All checks passed")?;
    Ok(())
}

fn pass(stdout: &mut termcolor::StandardStream, msg: &str) -> eyre::Result<()> {
    let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
    write!(stdout, "  PASS ")?;
    let _ = stdout.reset();
    writeln!(stdout, " {msg}")?;
    Ok(())
}

fn fail(stdout: &mut termcolor::StandardStream, msg: &str) -> eyre::Result<()> {
    let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
    write!(stdout, "  FAIL ")?;
    let _ = stdout.reset();
    writeln!(stdout, " {msg}")?;
    Ok(())
}

fn warn(stdout: &mut termcolor::StandardStream, msg: &str) -> eyre::Result<()> {
    let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Yellow)));
    write!(stdout, "  WARN ")?;
    let _ = stdout.reset();
    writeln!(stdout, " {msg}")?;
    Ok(())
}

/// Pure evaluation of a directory's suitability as a POSIX shared-memory mount.
///
/// Returns one of four states based on `path`'s existence, type, and mode.
/// Separated from the I/O wrapper so it can be unit-tested against a tempdir.
#[cfg(target_os = "linux")]
#[derive(Debug)]
enum ShmState {
    Inaccessible(std::io::Error),
    NotADirectory,
    WrongMode { actual: u32, expected: u32 },
    Ok,
}

#[cfg(target_os = "linux")]
fn evaluate_shm(path: &std::path::Path) -> ShmState {
    use std::os::unix::fs::PermissionsExt;
    let metadata = match std::fs::metadata(path) {
        Ok(metadata) => metadata,
        Err(e) => return ShmState::Inaccessible(e),
    };
    if !metadata.is_dir() {
        return ShmState::NotADirectory;
    }
    let actual = metadata.permissions().mode() & 0o7777;
    let expected = 0o1777;
    if actual != expected {
        return ShmState::WrongMode { actual, expected };
    }
    ShmState::Ok
}

/// Check `/dev/shm` is a directory with mode `1777` (sticky, world-rwx).
///
/// dora's zero-copy fast path for large payloads uses zenoh SHM, which
/// allocates segments in `/dev/shm`. The node runtime treats SHM as
/// best-effort: if the provider can't be created or allocation fails,
/// it falls back to heap-buffered publishes (see
/// `apis/rust/node/src/node/mod.rs`). So a misconfigured `/dev/shm` is
/// a perf concern (heap copy on every large message), not a correctness
/// one — surface it as `WARN`, not `FAIL`. See dora-rs/dora#1352.
#[cfg(target_os = "linux")]
fn check_shared_memory(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    use std::path::Path;
    match evaluate_shm(Path::new("/dev/shm")) {
        ShmState::Inaccessible(e) => warn(
            stdout,
            &format!(
                "Shared memory: /dev/shm not accessible ({e}) — dora will run via heap fallback; \
                 for the zero-copy fast path run \
                 `sudo mkdir -p /dev/shm && sudo chmod 1777 /dev/shm` \
                 (in a container, pass `--shm-size=...` or mount a tmpfs at /dev/shm)"
            ),
        )?,
        ShmState::NotADirectory => warn(
            stdout,
            "Shared memory: /dev/shm exists but is not a directory — dora will run via heap fallback; \
             for the zero-copy fast path run `sudo chmod 1777 /dev/shm`",
        )?,
        ShmState::WrongMode { actual, expected } => warn(
            stdout,
            &format!(
                "Shared memory: /dev/shm has mode {actual:#o}, expected {expected:#o} — \
                 dora will run via heap fallback if SHM allocation is denied; \
                 for the zero-copy fast path run `sudo chmod 1777 /dev/shm`"
            ),
        )?,
        ShmState::Ok => pass(stdout, "Shared memory: /dev/shm mode is 1777")?,
    }
    Ok(())
}

/// Check that `uv` is on PATH.
///
/// `dora build --uv` and the managed Python env flow (`<working-dir>/.dora/python-envs/<node-id>/`)
/// depend on the `uv` binary being available. Pure Rust/C++ users do not need it,
/// so report as `WARN`, not `FAIL` — a system without `uv` is still healthy if no
/// Python dataflows are involved.
fn check_uv(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    match std::process::Command::new("uv").arg("--version").output() {
        Ok(output) if output.status.success() => {
            let version = String::from_utf8_lossy(&output.stdout).trim().to_string();
            // `uv --version` prints `uv 0.4.18 (foo bar)`. Show whatever it gave us.
            let label = if version.is_empty() {
                "uv: available".to_string()
            } else {
                format!("uv: {version}")
            };
            pass(stdout, &label)?;
        }
        Ok(output) => {
            warn(
                stdout,
                &format!(
                    "uv: `uv --version` exited with {} — `dora build --uv` may misbehave",
                    output.status
                ),
            )?;
        }
        Err(_) => {
            warn(
                stdout,
                "uv: not installed — `dora build --uv` (managed Python envs) will fail. \
                 Install via `curl -LsSf https://astral.sh/uv/install.sh | sh`",
            )?;
        }
    }
    Ok(())
}

#[cfg(all(test, target_os = "linux"))]
mod shm_tests {
    use super::{ShmState, evaluate_shm};
    use std::fs;
    use std::os::unix::fs::PermissionsExt;
    use tempfile::tempdir;

    #[test]
    fn inaccessible_when_path_is_missing() {
        let dir = tempdir().unwrap();
        let missing = dir.path().join("nope");
        assert!(matches!(evaluate_shm(&missing), ShmState::Inaccessible(_)));
    }

    #[test]
    fn not_a_directory_when_path_is_file() {
        let dir = tempdir().unwrap();
        let file = dir.path().join("not-a-dir");
        fs::write(&file, b"").unwrap();
        assert!(matches!(evaluate_shm(&file), ShmState::NotADirectory));
    }

    #[test]
    fn wrong_mode_when_directory_is_0755() {
        let dir = tempdir().unwrap();
        let shm = dir.path().join("shm");
        fs::create_dir(&shm).unwrap();
        fs::set_permissions(&shm, fs::Permissions::from_mode(0o755)).unwrap();
        match evaluate_shm(&shm) {
            ShmState::WrongMode { actual, expected } => {
                assert_eq!(actual, 0o755);
                assert_eq!(expected, 0o1777);
            }
            other => panic!("expected WrongMode, got {other:?}"),
        }
    }

    #[test]
    fn ok_when_directory_is_1777() {
        let dir = tempdir().unwrap();
        let shm = dir.path().join("shm");
        fs::create_dir(&shm).unwrap();
        fs::set_permissions(&shm, fs::Permissions::from_mode(0o1777)).unwrap();
        assert!(matches!(evaluate_shm(&shm), ShmState::Ok));
    }
}

fn check_daemon(session: &WsSession) -> eyre::Result<bool> {
    let reply = send_control_request(session, &ControlRequest::DaemonConnected)?;
    Ok(expect_reply!(reply, DaemonConnected(running))?)
}

fn check_connected_machines(
    session: &WsSession,
) -> eyre::Result<Vec<dora_message::coordinator_to_cli::DaemonInfo>> {
    let reply = send_control_request(session, &ControlRequest::ConnectedMachines)?;
    Ok(expect_reply!(reply, ConnectedDaemons(daemons))?)
}

fn check_node_health(session: &WsSession) -> eyre::Result<(usize, usize, usize, usize)> {
    let reply = send_control_request(session, &ControlRequest::GetNodeInfo)?;
    let nodes = expect_reply!(reply, NodeInfoList(infos))?;

    let total = nodes.len();
    let mut healthy = 0usize;
    let mut degraded = 0usize;
    let mut failed = 0usize;

    for node in &nodes {
        if let Some(metrics) = &node.metrics {
            use dora_message::daemon_to_coordinator::NodeStatus;
            match metrics.status {
                NodeStatus::Running => healthy += 1,
                NodeStatus::Restarting => healthy += 1,
                NodeStatus::Degraded => degraded += 1,
                NodeStatus::Failed => failed += 1,
                // A cleanly-stopped node is neither healthy nor a fault:
                // skip it so doctor counts reflect the live surface.
                NodeStatus::Stopped => {}
            }
        } else {
            healthy += 1; // No metrics yet = assumed healthy
        }
    }

    Ok((total, healthy, degraded, failed))
}

fn validate_dataflow(path: &std::path::Path) -> eyre::Result<()> {
    use dora_core::descriptor::{Descriptor, DescriptorExt};
    let working_dir = path
        .canonicalize()
        .context("failed to canonicalize dataflow path")?
        .parent()
        .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
        .to_owned();
    Descriptor::blocking_read(path)?.check(&working_dir)?;
    Ok(())
}
