use std::{
    net::SocketAddr,
    path::PathBuf,
    process::Command,
    time::{Duration, Instant},
};

use clap::Args;
use eyre::{Context, bail};

use crate::{
    command::{Executable, default_tracing, up::adora_executable_path},
    common::connect_to_coordinator,
};

use super::config::ClusterConfig;
use super::{format_labels_arg, query_connected_daemons, run_ssh, ssh_target};

/// Bring up a multi-machine cluster from a cluster.yml file.
///
/// Starts the coordinator locally, then SSH-es into each machine to
/// start a daemon.
///
/// Examples:
///
///   adora cluster up cluster.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Up {
    /// Path to the cluster configuration file
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: PathBuf,
}

impl Executable for Up {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let config = ClusterConfig::load(&self.config)?;
        let coordinator_addr: SocketAddr =
            (config.coordinator.addr, config.coordinator.port).into();

        // 1. Connect to existing coordinator or start a new one
        let session = match connect_to_coordinator(coordinator_addr) {
            Ok(s) => {
                println!("Coordinator already running at {coordinator_addr}");
                s
            }
            Err(_) => {
                start_coordinator(config.coordinator.port)?;
                let deadline = Instant::now() + Duration::from_secs(10);
                loop {
                    match connect_to_coordinator(coordinator_addr) {
                        Ok(s) => break s,
                        Err(_) if Instant::now() < deadline => {
                            std::thread::sleep(Duration::from_millis(50));
                        }
                        Err(err) => {
                            bail!("timed out waiting for coordinator at {coordinator_addr}: {err}");
                        }
                    }
                }
            }
        };

        // 2. SSH into each machine to start a daemon
        let mut ssh_failures: Vec<(String, String)> = Vec::new();
        for machine in &config.machines {
            let target = ssh_target(machine);
            let labels_arg = format_labels_arg(&machine.labels);
            let remote_cmd = format!(
                "nohup adora daemon --machine-id {id} --coordinator-addr {addr} --coordinator-port {port}{labels} --quiet > /tmp/adora-daemon-{id}.log 2>&1 &",
                id = machine.id,
                addr = config.coordinator.addr,
                port = config.coordinator.port,
                labels = labels_arg,
            );

            println!("Starting daemon on {} ({})", machine.id, target);
            match run_ssh(&target, &remote_cmd) {
                Ok(true) => {}
                Ok(false) => {
                    let msg = "ssh command failed".to_string();
                    eprintln!(
                        "  WARNING: failed to start daemon on `{}`: {msg}",
                        machine.id
                    );
                    ssh_failures.push((machine.id.clone(), msg));
                }
                Err(err) => {
                    let msg = format!("{err}");
                    eprintln!(
                        "  WARNING: failed to start daemon on `{}`: {msg}",
                        machine.id
                    );
                    ssh_failures.push((machine.id.clone(), msg));
                }
            }
        }

        // 3. Poll until all (successful) daemons have registered
        let expected: Vec<&str> = config
            .machines
            .iter()
            .filter(|m| !ssh_failures.iter().any(|(id, _)| id == &m.id))
            .map(|m| m.id.as_str())
            .collect();

        if !expected.is_empty() {
            println!("Waiting for {} daemon(s) to connect...", expected.len());
            let deadline = Instant::now() + Duration::from_secs(30);
            loop {
                let connected = query_connected_daemons(&session)?;
                let all_present = expected.iter().all(|machine_id| {
                    connected
                        .iter()
                        .any(|d| d.daemon_id.matches_machine_id(machine_id))
                });
                if all_present {
                    break;
                }
                if Instant::now() >= deadline {
                    let missing: Vec<&str> = expected
                        .iter()
                        .copied()
                        .filter(|machine_id| {
                            !connected
                                .iter()
                                .any(|d| d.daemon_id.matches_machine_id(machine_id))
                        })
                        .collect();
                    eprintln!(
                        "WARNING: timed out waiting for daemon(s): {}",
                        missing.join(", ")
                    );
                    break;
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        }

        // 4. Report
        let ok_count = config.machines.len() - ssh_failures.len();
        if ssh_failures.is_empty() {
            println!(
                "Cluster is up: coordinator + {} daemon(s)",
                config.machines.len()
            );
        } else {
            println!(
                "Cluster partially up: coordinator + {ok_count}/{} daemon(s)",
                config.machines.len()
            );
            for (id, reason) in &ssh_failures {
                eprintln!("  {id}: {reason}");
            }
        }

        Ok(())
    }
}

fn start_coordinator(port: u16) -> eyre::Result<()> {
    let path = adora_executable_path()?;
    let mut cmd = Command::new(path);
    cmd.args([
        "coordinator",
        "--interface",
        "0.0.0.0",
        "--port",
        &port.to_string(),
        "--quiet",
    ]);
    cmd.spawn().wrap_err("failed to start adora coordinator")?;
    println!("Started coordinator on 0.0.0.0:{port}");
    Ok(())
}
