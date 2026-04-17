use std::path::PathBuf;

use clap::Args;

use crate::command::{Executable, default_tracing};

use super::config::ClusterConfig;
use super::{format_labels_arg, print_summary, record_ssh_result, run_ssh, ssh_target};

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

        for machine in &config.machines {
            let target = ssh_target(machine);
            let labels_arg = format_labels_arg(&machine.labels);
            let service_name = format!("dora-daemon-{}", machine.id);

            let unit = format!(
                r#"[Unit]
Description=Dora Daemon ({id})
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=dora daemon --machine-id {id} --coordinator-addr {addr} --coordinator-port {port}{labels} --quiet
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
"#,
                id = machine.id,
                addr = config.coordinator.addr,
                port = config.coordinator.port,
                labels = labels_arg,
            );

            // Escape single quotes for shell
            let escaped_unit = unit.replace('\'', "'\\''");
            let cmd = format!(
                "echo '{escaped_unit}' | sudo tee /etc/systemd/system/{service_name}.service > /dev/null && sudo systemctl daemon-reload && sudo systemctl enable --now {service_name}"
            );

            println!("Installing {service_name} on {} ({target})", machine.id);
            let result = run_ssh(&target, &cmd);
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

        Ok(())
    }
}
