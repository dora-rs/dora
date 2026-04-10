use std::io::{IsTerminal, Write};

use clap::Args;
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, connect_to_coordinator, query_running_dataflows},
    ws_client::WsSession,
};
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowStatus},
};
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

fn check_daemon(session: &WsSession) -> eyre::Result<bool> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::DaemonConnected).unwrap())
        .wrap_err("failed to send DaemonConnected")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match reply {
        ControlRequestReply::DaemonConnected(running) => Ok(running),
        other => eyre::bail!("unexpected reply: {other:?}"),
    }
}

fn check_connected_machines(
    session: &WsSession,
) -> eyre::Result<Vec<dora_message::coordinator_to_cli::DaemonInfo>> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::ConnectedMachines).unwrap())
        .wrap_err("failed to send ConnectedMachines")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match reply {
        ControlRequestReply::ConnectedDaemons(daemons) => Ok(daemons),
        other => eyre::bail!("unexpected reply: {other:?}"),
    }
}

fn check_node_health(session: &WsSession) -> eyre::Result<(usize, usize, usize, usize)> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::GetNodeInfo).unwrap())
        .wrap_err("failed to send GetNodeInfo")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    let nodes = match reply {
        ControlRequestReply::NodeInfoList(infos) => infos,
        other => eyre::bail!("unexpected reply: {other:?}"),
    };

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
