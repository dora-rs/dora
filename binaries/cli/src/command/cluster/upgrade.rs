use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

use clap::Args;

use crate::{
    command::{Executable, default_tracing, up::adora_executable_path},
    common::connect_to_coordinator,
};

use super::config::ClusterConfig;
use super::{query_connected_daemons, run_ssh, ssh_target};

/// Rolling upgrade: SCP the local adora binary to each machine and restart daemons.
///
/// For each machine sequentially:
///   1. SCP the local adora binary to `/usr/local/bin/adora`
///   2. Restart the systemd service
///   3. Wait for the daemon to reconnect
///
/// Examples:
///
///   adora cluster upgrade cluster.yml
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
        let local_binary = adora_executable_path()?;
        let coordinator_addr =
            std::net::SocketAddr::from((config.coordinator.addr, config.coordinator.port));
        let session = connect_to_coordinator(coordinator_addr)?;

        let mut failures = Vec::new();

        for machine in &config.machines {
            let target = ssh_target(machine);
            let service_name = format!("adora-daemon-{}", machine.id);

            println!("Upgrading {} ({target})...", machine.id);

            // 1. SCP binary
            let scp_status = std::process::Command::new("scp")
                .args([
                    "-o",
                    "BatchMode=yes",
                    "-o",
                    "ConnectTimeout=10",
                    local_binary.to_str().unwrap_or("adora"),
                    &format!("{target}:/usr/local/bin/adora"),
                ])
                .status();

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

            // 2. Restart systemd service
            let restart_cmd = format!("sudo systemctl restart {service_name}");
            match run_ssh(&target, &restart_cmd) {
                Ok(true) => {}
                _ => {
                    let msg = "systemctl restart failed".to_string();
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
                if let Ok(connected) = query_connected_daemons(&session) {
                    if connected
                        .iter()
                        .any(|d| d.daemon_id.matches_machine_id(&machine.id))
                    {
                        reconnected = true;
                        break;
                    }
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
        } else {
            println!(
                "Upgraded {}/{} machine(s)",
                config.machines.len() - failures.len(),
                config.machines.len()
            );
            for (id, reason) in &failures {
                eprintln!("  {id}: {reason}");
            }
        }

        Ok(())
    }
}
