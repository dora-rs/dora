use std::path::PathBuf;

use clap::Args;

use crate::command::{Executable, default_tracing};

use super::config::{ClusterConfig, MachineConfig};
use super::{
    format_daemon_port_arg, format_labels_arg, format_zenoh_peer_arg, print_summary,
    record_ssh_result, resolve_zenoh_mesh_args, run_ssh, ssh_target,
};

/// Build the systemd unit file for one machine's daemon.
///
/// `mesh_arg` is the pre-formatted per-machine zenoh mesh fragment (e.g.
/// ` --zenoh-listen … --zenoh-connect …`), or empty when no mesh is derived.
/// It must be threaded into `ExecStart` here — a systemd-installed daemon needs
/// the same `--zenoh-listen`/`--zenoh-connect` clique that `dora cluster up`
/// passes over SSH, or the cluster silently falls back to multicast discovery.
fn daemon_unit_file(config: &ClusterConfig, machine: &MachineConfig, mesh_arg: &str) -> String {
    let labels_arg = format_labels_arg(&machine.labels);
    let daemon_port_arg = format_daemon_port_arg(machine.daemon_port);
    let zenoh_peer_arg = format_zenoh_peer_arg(config.zenoh_peer.as_deref());
    format!(
        r#"[Unit]
Description=Dora Daemon ({id})
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=dora daemon --machine-id {id} --coordinator-addr {addr} --coordinator-port {port}{daemon_port_arg}{zenoh_peer_arg}{mesh_arg}{labels} --quiet
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
"#,
        id = machine.id,
        addr = config.coordinator.addr,
        port = config.coordinator.port,
        labels = labels_arg,
    )
}

/// Install dora-daemon as a systemd service on each machine.
///
/// SSH-es into each machine, writes a systemd unit file, and enables the service.
///
/// Examples:
///
///   dora cluster install cluster.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Install {
    /// Path to the cluster configuration file
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: PathBuf,
}

impl Executable for Install {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let config = ClusterConfig::load(&self.config)?;

        let mut failures = Vec::new();

        // Wire the installed daemons into an explicit zenoh mesh where the
        // config allows it, exactly as `dora cluster up` does. A daemon started
        // as a systemd service needs the `--zenoh-listen`/`--zenoh-connect`
        // clique just as much as one started over SSH — without it, a cluster on
        // a network without multicast (a mesh VPN carries none) never forms,
        // which is the failure mode this mesh feature exists to prevent.
        let zenoh_mesh_args = resolve_zenoh_mesh_args(&config);
        for machine in &config.machines {
            let target = ssh_target(machine);
            let mesh_arg = zenoh_mesh_args
                .as_ref()
                .and_then(|args| args.get(machine.id.as_str()))
                .map(String::as_str)
                .unwrap_or_default();
            let service_name = format!("dora-daemon-{}", machine.id);

            let unit = daemon_unit_file(&config, machine, mesh_arg);

            // Escape single quotes for shell
            let escaped_unit = unit.replace('\'', "'\\''");
            let cmd = format!(
                "echo '{escaped_unit}' | sudo tee /etc/systemd/system/{service_name}.service > /dev/null && sudo systemctl daemon-reload && sudo systemctl enable --now {service_name}"
            );

            println!("Installing {service_name} on {} ({target})", machine.id);
            let result = run_ssh(&target, machine.port, &cmd);
            record_ssh_result(
                &mut failures,
                &machine.id,
                result,
                &format!("{service_name} installed and started"),
            );
        }

        print_summary(
            "daemon(s) installed as systemd services",
            config.machines.len(),
            &failures,
        );

        if failures.is_empty() {
            Ok(())
        } else {
            eyre::bail!(
                "install failed on {}/{} machine(s)",
                failures.len(),
                config.machines.len()
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    fn write_yaml(content: &str) -> NamedTempFile {
        let mut f = NamedTempFile::new().unwrap();
        f.write_all(content.as_bytes()).unwrap();
        f
    }

    /// The systemd unit installed by `dora cluster install` must carry the same
    /// zenoh mesh wiring (`--zenoh-listen`/`--zenoh-connect`) that
    /// `dora cluster up` passes over SSH. Regression guard: dropping `{mesh_arg}`
    /// from `ExecStart` — the original bug — silently falls a mesh cluster back
    /// to multicast discovery, which fails on a network without multicast.
    #[test]
    fn install_unit_carries_derived_zenoh_mesh_args() {
        let f = write_yaml(
            "coordinator:\n  addr: 100.64.0.1\nmachines:\n  \
             - id: a\n    host: 100.64.0.2\n  \
             - id: b\n    host: 100.64.0.3\n",
        );
        let config = ClusterConfig::load(f.path()).unwrap();
        let args =
            resolve_zenoh_mesh_args(&config).expect("both hosts are IPs, so the mesh must derive");

        let machine_a = &config.machines[0];
        let unit = daemon_unit_file(&config, machine_a, &args["a"]);

        let exec_start = unit
            .lines()
            .find(|l| l.starts_with("ExecStart="))
            .expect("unit has an ExecStart line");
        assert!(
            exec_start.contains("--zenoh-listen 100.64.0.2:5456"),
            "ExecStart must listen on the machine's own endpoint: {exec_start}"
        );
        assert!(
            exec_start.contains("--zenoh-connect tcp/100.64.0.3:5456"),
            "ExecStart must dial the other machine: {exec_start}"
        );
    }

    /// A single-machine cluster derives no mesh, so its unit must not carry any
    /// zenoh mesh flags — the empty `mesh_arg` must leave `ExecStart` clean
    /// rather than emitting a dangling flag.
    #[test]
    fn install_unit_omits_mesh_args_when_not_needed() {
        let f = write_yaml(
            "coordinator:\n  addr: 100.64.0.1\nmachines:\n  - id: solo\n    host: 100.64.0.2\n",
        );
        let config = ClusterConfig::load(f.path()).unwrap();
        assert!(resolve_zenoh_mesh_args(&config).is_none());

        let unit = daemon_unit_file(&config, &config.machines[0], "");
        let exec_start = unit
            .lines()
            .find(|l| l.starts_with("ExecStart="))
            .expect("unit has an ExecStart line");
        assert!(
            !exec_start.contains("--zenoh-listen") && !exec_start.contains("--zenoh-connect"),
            "single-machine unit must not carry mesh flags: {exec_start}"
        );
    }
}
