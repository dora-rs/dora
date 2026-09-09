use std::{
    net::SocketAddr,
    path::PathBuf,
    time::{Duration, Instant},
};

use clap::Args;
use eyre::Context;

use crate::{
    command::{Executable, default_tracing, up},
    common::connect_to_coordinator,
};

use super::config::{ClusterConfig, ZenohMesh};
use super::{
    format_daemon_port_arg, format_labels_arg, format_zenoh_peer_arg, query_connected_daemons,
    run_ssh, ssh_target,
};

/// Bring up a multi-machine cluster from a cluster.yml file.
///
/// Starts the coordinator locally, then SSH-es into each machine to
/// start a daemon.
///
/// Examples:
///
///   dora cluster up cluster.yml
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
                // Shared with `dora up` so the two cannot drift apart again;
                // the wildcard bind is what makes the coordinator reachable
                // from the machines we are about to ssh into.
                let startup = up::spawn_coordinator(up::CoordinatorSpawn {
                    interface: Some(std::net::Ipv4Addr::UNSPECIFIED.into()),
                    port: Some(config.coordinator.port),
                    auth: false,
                })
                .wrap_err("failed to start dora coordinator")?;
                up::wait_for_coordinator_start(coordinator_addr, config.coordinator.port, startup)?
            }
        };

        // 2. SSH into each machine to start a daemon
        let zenoh_peer_arg = format_zenoh_peer_arg(config.zenoh_peer.as_deref());
        // Wire the daemons into an explicit zenoh mesh where the config allows
        // it. Falling back is deliberate: a partial mesh is worse than none,
        // since explicit connect endpoints turn multicast scouting off for the
        // daemons that have them while the rest still depend on it.
        let zenoh_mesh_args = match config.zenoh_mesh_args() {
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
        };
        let mut ssh_failures: Vec<(String, String)> = Vec::new();
        for machine in &config.machines {
            let target = ssh_target(machine);
            let labels_arg = format_labels_arg(&machine.labels);
            let daemon_port_arg = format_daemon_port_arg(machine.daemon_port);
            let mesh_arg = zenoh_mesh_args
                .as_ref()
                .and_then(|args| args.get(machine.id.as_str()))
                .map(String::as_str)
                .unwrap_or_default();
            let remote_cmd = format!(
                "nohup dora daemon --machine-id {id} --coordinator-addr {addr} --coordinator-port {port}{daemon_port_arg}{zenoh_peer_arg}{mesh_arg}{labels} --quiet > /tmp/dora-daemon-{id}.log 2>&1 &",
                id = machine.id,
                addr = config.coordinator.addr,
                port = config.coordinator.port,
                labels = labels_arg,
            );

            println!("Starting daemon on {} ({})", machine.id, target);
            match run_ssh(&target, machine.port, &remote_cmd) {
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

        let mut missing_daemons: Vec<String> = Vec::new();
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
                    missing_daemons = expected
                        .iter()
                        .copied()
                        .filter(|machine_id| {
                            !connected
                                .iter()
                                .any(|d| d.daemon_id.matches_machine_id(machine_id))
                        })
                        .map(String::from)
                        .collect();
                    eprintln!(
                        "WARNING: timed out waiting for daemon(s): {}",
                        missing_daemons.join(", ")
                    );
                    report_missing_daemon_logs(&config, &missing_daemons);
                    break;
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        }

        // 4. Report. A partial-up state (some daemons unreachable or never
        // registered) must exit non-zero so callers — scripts, CI, the
        // cluster-e2e job — can react instead of silently treating
        // "Cluster partially up" as success.
        let ok_count = config.machines.len() - ssh_failures.len() - missing_daemons.len();
        if ssh_failures.is_empty() && missing_daemons.is_empty() {
            println!(
                "Cluster is up: coordinator + {} daemon(s)",
                config.machines.len()
            );
            Ok(())
        } else {
            println!(
                "Cluster partially up: coordinator + {ok_count}/{} daemon(s)",
                config.machines.len()
            );
            for (id, reason) in &ssh_failures {
                eprintln!("  {id}: {reason}");
            }
            eyre::bail!(
                "cluster up incomplete: {} ssh failure(s), {} daemon(s) did not register",
                ssh_failures.len(),
                missing_daemons.len()
            )
        }
    }
}

/// Print the tail of each unreachable daemon's log.
///
/// A daemon that dies during startup writes the reason to
/// `/tmp/dora-daemon-<id>.log` on its own host and never registers, so without
/// this the operator sees only "timed out waiting for daemon(s)" and has to go
/// find the file themselves. The common causes are diagnosable from the log and
/// nowhere else: a `--zenoh-listen` address that is not on the machine (a NAT
/// or VIP `host:` in cluster.yml), or its port already held by a stale daemon —
/// both fatal by design for an explicitly named listener, since a daemon that
/// silently fails to bind is undialable.
///
/// Best-effort: a machine we cannot ssh back into is skipped quietly, because
/// this runs on a path that has already failed and must not fail differently.
fn report_missing_daemon_logs(config: &ClusterConfig, missing: &[String]) {
    for machine_id in missing {
        let Some(machine) = config.machines.iter().find(|m| &m.id == machine_id) else {
            continue;
        };
        // `machine.id` is charset-validated by `ClusterConfig::validate`, so it
        // cannot break out of the remote command.
        let cmd = format!("tail -n 20 /tmp/dora-daemon-{machine_id}.log 2>/dev/null");
        eprintln!("  --- last log lines from `{machine_id}` ---");
        if run_ssh(&ssh_target(machine), machine.port, &cmd).is_err() {
            eprintln!("  (could not read the log — ssh to `{machine_id}` failed)");
        }
    }
}
