use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::DaemonInfo};
use eyre::Context;
use std::collections::BTreeMap;

use crate::{
    command::Executable,
    common::{expect_reply, send_control_request},
    ws_client::WsSession,
};

use self::config::{ClusterConfig, MachineConfig, ZenohMesh};

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
    let reply = send_control_request(session, &ControlRequest::ConnectedMachines)?;
    Ok(expect_reply!(reply, ConnectedDaemons(daemons))?)
}

// ---------------------------------------------------------------------------
// Shared SSH helpers
// ---------------------------------------------------------------------------

/// Format the SSH target string from a machine config.
pub(super) fn ssh_target(machine: &MachineConfig) -> String {
    with_user(machine, &machine.host)
}

/// Format the `scp` destination for `path` on a machine.
///
/// scp splits host from path at the first `:` outside brackets, so an IPv6
/// literal host must be bracketed: `2001:db8::1:/path` would otherwise name
/// host `2001`. A hostname or IPv4 address never contains `:`.
pub(super) fn scp_target(machine: &MachineConfig, path: &str) -> String {
    let host = &machine.host;
    if host.contains(':') && !host.starts_with('[') {
        format!("{}:{path}", with_user(machine, &format!("[{host}]")))
    } else {
        format!("{}:{path}", with_user(machine, host))
    }
}

/// `host`, prefixed with the machine's `user@` if it has one.
fn with_user(machine: &MachineConfig, host: &str) -> String {
    match &machine.user {
        Some(user) => format!("{user}@{host}"),
        None => host.to_owned(),
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

/// Format the `--local-listen-port <p>` argument string.
pub(super) fn format_daemon_port_arg(daemon_port: Option<u16>) -> String {
    match daemon_port {
        Some(p) => format!(" --local-listen-port {p}"),
        None => String::new(),
    }
}

/// Format the `--zenoh-peer <ep>` argument string.
pub(super) fn format_zenoh_peer_arg(zenoh_peer: Option<&str>) -> String {
    match zenoh_peer {
        Some(ep) => format!(" --zenoh-peer {ep}"),
        None => String::new(),
    }
}

/// Resolve each machine's zenoh mesh argument fragment
/// (` --zenoh-listen … --zenoh-connect …`) from the cluster config, warning once
/// when a mesh is configured but cannot be derived.
///
/// Shared by `dora cluster up` (which starts daemons over SSH) and
/// `dora cluster install` (which installs them as systemd services) so both wire
/// the daemons into the same explicit clique. Falling back to multicast is
/// deliberate: a partial mesh is worse than none, since explicit connect
/// endpoints turn multicast scouting off for the daemons that have them while
/// the rest still depend on it.
pub(super) fn resolve_zenoh_mesh_args(config: &ClusterConfig) -> Option<BTreeMap<&str, String>> {
    match config.zenoh_mesh_args() {
        ZenohMesh::Derived(args) => Some(args),
        ZenohMesh::NotNeeded => None,
        ZenohMesh::Unavailable(reason) => {
            eprintln!(
                "WARNING: {reason}, so the daemons are left to discover each other \
                 by multicast. On a network without multicast — a mesh VPN carries \
                 none — they will not find each other. Fix the field named above, \
                 or configure a shared `zenoh_peer` rendezvous."
            );
            None
        }
    }
}

/// Run a command on a remote machine via SSH. Returns whether it succeeded.
pub(super) fn run_ssh(target: &str, port: Option<u16>, cmd: &str) -> eyre::Result<bool> {
    let mut command = std::process::Command::new("ssh");
    command.args([
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=10",
        "-o",
        "StrictHostKeyChecking=accept-new",
    ]);
    if let Some(p) = port {
        command.args(["-p", &p.to_string()]);
    }
    // `--` marks the end of options so a `target` beginning with `-` (e.g. a
    // malicious `-oProxyCommand=...`) is treated as the hostname, not an ssh
    // option. Defense-in-depth: `ClusterConfig::validate` already rejects a
    // leading dash on `host`/`user`.
    command.args(["--", target, cmd]);
    let status = command
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

#[cfg(test)]
mod tests {
    use super::*;

    fn machine(host: &str, user: Option<&str>) -> MachineConfig {
        let mut yaml = format!("id: a\nhost: \"{host}\"\n");
        if let Some(user) = user {
            yaml.push_str(&format!("user: {user}\n"));
        }
        serde_yaml::from_str(&yaml).expect("valid machine config")
    }

    #[test]
    fn scp_target_brackets_ipv6_hosts() {
        let path = "/usr/local/bin/dora";
        assert_eq!(
            scp_target(&machine("2001:db8::1", Some("u")), path),
            "u@[2001:db8::1]:/usr/local/bin/dora"
        );
        assert_eq!(
            scp_target(&machine("::1", None), path),
            "[::1]:/usr/local/bin/dora"
        );
        // Already bracketed: left as is.
        assert_eq!(
            scp_target(&machine("[::1]", None), path),
            "[::1]:/usr/local/bin/dora"
        );
        assert_eq!(
            scp_target(&machine("192.168.1.10", Some("u")), path),
            "u@192.168.1.10:/usr/local/bin/dora"
        );
        assert_eq!(
            scp_target(&machine("robot.local", None), path),
            "robot.local:/usr/local/bin/dora"
        );
    }
}
