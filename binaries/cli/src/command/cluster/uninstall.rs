use std::path::PathBuf;

use clap::Args;

use crate::command::{Executable, default_tracing};

use super::config::ClusterConfig;
use super::{print_summary, record_ssh_result, run_ssh, ssh_target};

/// Uninstall adora-daemon systemd services from each machine.
///
/// SSH-es into each machine, stops, disables, and removes the systemd unit.
///
/// Examples:
///
///   adora cluster uninstall cluster.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Uninstall {
    /// Path to the cluster configuration file
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: PathBuf,
}

impl Executable for Uninstall {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let config = ClusterConfig::load(&self.config)?;

        let mut failures = Vec::new();

        for machine in &config.machines {
            let target = ssh_target(machine);
            let service_name = format!("adora-daemon-{}", machine.id);

            let cmd = format!(
                "sudo systemctl stop {service_name} 2>/dev/null; \
                 sudo systemctl disable {service_name} 2>/dev/null; \
                 sudo rm -f /etc/systemd/system/{service_name}.service; \
                 sudo systemctl daemon-reload"
            );

            println!("Uninstalling {service_name} from {} ({target})", machine.id);
            let result = run_ssh(&target, &cmd);
            record_ssh_result(
                &mut failures,
                &machine.id,
                result,
                &format!("{service_name} removed"),
            );
        }

        print_summary("daemon(s) uninstalled", config.machines.len(), &failures);

        Ok(())
    }
}
