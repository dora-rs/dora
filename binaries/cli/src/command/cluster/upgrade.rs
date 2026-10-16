use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

use clap::Args;

use crate::{
    command::{Executable, default_tracing, up::dora_executable_path},
    common::connect_to_coordinator,
};

use super::config::ClusterConfig;
use super::{query_connected_daemons, run_ssh, scp_target, ssh_target};

/// Rolling upgrade: SCP the local dora binary to each machine and restart daemons.
///
/// For each machine sequentially:
///   1. SCP the local dora binary to a staging file in the SSH user's home
///   2. `sudo install` it as `/usr/local/bin/dora` and restart the systemd
///      service
///   3. Wait for the daemon to reconnect
///
/// Examples:
///
///   dora cluster upgrade cluster.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Upgrade {
    /// Path to the cluster configuration file
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: PathBuf,
}

impl Executable for Upgrade {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let config = ClusterConfig::load(&self.config)?;
        let local_binary = dora_executable_path()?;
        let coordinator_addr =
            std::net::SocketAddr::from((config.coordinator.addr, config.coordinator.port));
        let session = connect_to_coordinator(coordinator_addr)?;

        let mut failures = Vec::new();

        for machine in &config.machines {
            let target = ssh_target(machine);
            let service_name = format!("dora-daemon-{}", machine.id);

            println!("Upgrading {} ({target})...", machine.id);

            // 1. SCP the binary to a staging file. Copying straight onto
            // `/usr/local/bin/dora` fails: the directory is root-owned for the
            // non-root user `cluster install` assumes, and the running daemon
            // holds the file open, so writing it fails with ETXTBSY.
            let staged = staging_path(&machine.id);
            let local_binary_str = local_binary
                .to_str()
                .ok_or_else(|| eyre::eyre!("local binary path is not valid UTF-8"))?;
            let mut scp = std::process::Command::new("scp");
            scp.args(["-o", "BatchMode=yes", "-o", "ConnectTimeout=10"]);
            if let Some(p) = machine.port {
                // scp uses `-P` (capital) for the port; `-p` means "preserve mtimes"
                scp.args(["-P", &p.to_string()]);
            }
            scp.args([local_binary_str, &scp_target(machine, &staged)]);
            let scp_status = scp.status();

            match scp_status {
                Ok(s) if s.success() => {}
                Ok(s) => {
                    let msg = format!("scp failed with {s}");
                    eprintln!("  FAILED: {msg}");
                    failures.push((machine.id.clone(), msg));
                    continue;
                }
                Err(e) => {
                    let msg = format!("scp error: {e}");
                    eprintln!("  FAILED: {msg}");
                    failures.push((machine.id.clone(), msg));
                    continue;
                }
            }

            // 2. Install the staged binary and restart the systemd service
            let restart_cmd = install_and_restart_command(&staged, &service_name);
            match run_ssh(&target, machine.port, &restart_cmd) {
                Ok(true) => {}
                _ => {
                    let msg = "installing the binary or restarting the service failed".to_string();
                    eprintln!("  FAILED: {msg}");
                    failures.push((machine.id.clone(), msg));
                    continue;
                }
            }

            // 3. Wait for daemon to reconnect (30s timeout)
            let deadline = Instant::now() + Duration::from_secs(30);
            let mut reconnected = false;
            while Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(500));
                if let Ok(connected) = query_connected_daemons(&session)
                    && connected
                        .iter()
                        .any(|d| d.daemon_id.matches_machine_id(&machine.id))
                {
                    reconnected = true;
                    break;
                }
            }

            if reconnected {
                println!("  OK: {} upgraded and reconnected", machine.id);
            } else {
                let msg = "daemon did not reconnect within 30s".to_string();
                eprintln!("  WARNING: {msg}");
                failures.push((machine.id.clone(), msg));
            }
        }

        if failures.is_empty() {
            println!("All {} machine(s) upgraded", config.machines.len());
            Ok(())
        } else {
            println!(
                "Upgraded {}/{} machine(s)",
                config.machines.len() - failures.len(),
                config.machines.len()
            );
            for (id, reason) in &failures {
                eprintln!("  {id}: {reason}");
            }
            eyre::bail!(
                "upgrade failed on {}/{} machine(s)",
                failures.len(),
                config.machines.len()
            )
        }
    }
}

/// Staging file for the uploaded binary, relative to the SSH user's home.
///
/// `machine_id` is shell-safe (`ClusterConfig::validate`), so the path can be
/// interpolated into the remote command unquoted.
fn staging_path(machine_id: &str) -> String {
    format!(".dora-upgrade-{machine_id}")
}

/// Remote command that moves the staged binary into place and restarts the
/// daemon.
///
/// `install` unlinks the destination before writing it, so it replaces the
/// executable the running daemon was started from instead of writing into it.
fn install_and_restart_command(staged: &str, service_name: &str) -> String {
    format!(
        "sudo install -m 755 {staged} /usr/local/bin/dora && rm -f {staged} && \
         sudo systemctl restart {service_name}"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn upgrade_installs_the_staged_binary_before_restarting() {
        let staged = staging_path("robot-1");
        assert_eq!(staged, ".dora-upgrade-robot-1");
        let cmd = install_and_restart_command(&staged, "dora-daemon-robot-1");
        assert_eq!(
            cmd,
            "sudo install -m 755 .dora-upgrade-robot-1 /usr/local/bin/dora && \
             rm -f .dora-upgrade-robot-1 && sudo systemctl restart dora-daemon-robot-1"
        );
    }
}
