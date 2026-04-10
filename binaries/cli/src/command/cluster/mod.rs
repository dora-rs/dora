use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DaemonInfo},
};
use eyre::{Context, bail};
use std::collections::BTreeMap;

use crate::{command::Executable, ws_client::WsSession};

use self::config::MachineConfig;

pub mod config;
mod down;
mod install;
mod restart;
mod status;
mod uninstall;
mod up;
mod upgrade;

/// Manage a multi-machine cluster.
#[derive(Debug, clap::Subcommand)]
pub enum Cluster {
    Up(up::Up),
    Status(status::Status),
    Down(down::Down),
    Install(install::Install),
    Uninstall(uninstall::Uninstall),
    Upgrade(upgrade::Upgrade),
    Restart(restart::Restart),
}

impl Executable for Cluster {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Cluster::Up(cmd) => cmd.execute(),
            Cluster::Status(cmd) => cmd.execute(),
            Cluster::Down(cmd) => cmd.execute(),
            Cluster::Install(cmd) => cmd.execute(),
            Cluster::Uninstall(cmd) => cmd.execute(),
            Cluster::Upgrade(cmd) => cmd.execute(),
            Cluster::Restart(cmd) => cmd.execute(),
        }
    }
}

pub(crate) fn query_connected_daemons(session: &WsSession) -> eyre::Result<Vec<DaemonInfo>> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::ConnectedMachines).unwrap())
        .wrap_err("failed to send ConnectedMachines request")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match reply {
        ControlRequestReply::ConnectedDaemons(daemons) => Ok(daemons),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected reply: {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Shared SSH helpers
// ---------------------------------------------------------------------------

/// Format the SSH target string from a machine config.
pub(super) fn ssh_target(machine: &MachineConfig) -> String {
    match &machine.user {
        Some(user) => format!("{user}@{}", machine.host),
        None => machine.host.clone(),
    }
}

/// Format the `--labels key=val,key=val` argument string.
pub(super) fn format_labels_arg(labels: &BTreeMap<String, String>) -> String {
    if labels.is_empty() {
        String::new()
    } else {
        let pairs: Vec<String> = labels.iter().map(|(k, v)| format!("{k}={v}")).collect();
        format!(" --labels {}", pairs.join(","))
    }
}

/// Run a command on a remote machine via SSH. Returns whether it succeeded.
pub(super) fn run_ssh(target: &str, cmd: &str) -> eyre::Result<bool> {
    let status = std::process::Command::new("ssh")
        .args([
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "StrictHostKeyChecking=accept-new",
            target,
            cmd,
        ])
        .status()
        .with_context(|| format!("failed to run ssh to {target}"))?;
    Ok(status.success())
}

/// Record an SSH result into a failure list. Prints OK on success, FAILED on error.
pub(super) fn record_ssh_result(
    failures: &mut Vec<(String, String)>,
    machine_id: &str,
    result: eyre::Result<bool>,
    ok_msg: &str,
) {
    match result {
        Ok(true) => println!("  OK: {ok_msg}"),
        Ok(false) => {
            let msg = "ssh command failed".to_string();
            eprintln!("  FAILED: {msg}");
            failures.push((machine_id.to_owned(), msg));
        }
        Err(err) => {
            let msg = format!("{err}");
            eprintln!("  FAILED: {msg}");
            failures.push((machine_id.to_owned(), msg));
        }
    }
}

/// Print a summary of successes and failures after a batch SSH operation.
pub(super) fn print_summary(action: &str, total: usize, failures: &[(String, String)]) {
    if failures.is_empty() {
        println!("All {total} {action}");
    } else {
        println!("{}/{total} {action}", total - failures.len());
        for (id, reason) in failures {
            eprintln!("  {id}: {reason}");
        }
    }
}
