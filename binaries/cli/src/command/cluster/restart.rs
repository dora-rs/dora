use std::{net::SocketAddr, path::PathBuf};

use clap::Args;
use dora_message::cli_to_coordinator::ControlRequest;

use crate::{
    command::{Executable, default_tracing},
    common::{connect_to_coordinator, expect_reply, send_control_request},
};

use super::config::ClusterConfig;

/// Restart a running dataflow by name.
///
/// Stops the dataflow and immediately re-starts it with the stored descriptor.
///
/// Examples:
///
///   dora cluster restart cluster.yml my-dataflow
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Restart {
    /// Path to the cluster configuration file
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: PathBuf,

    /// Name of the dataflow to restart (must match the `--name` passed
    /// to `dora start`). UUID lookup is not supported via this subcommand:
    /// the request is sent as `RestartByName`, which the coordinator
    /// matches against the dataflow's `name` field only, not its UUID.
    #[clap(value_name = "NAME")]
    dataflow: String,
}

impl Executable for Restart {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let config = ClusterConfig::load(&self.config)?;
        let coordinator_addr: SocketAddr =
            (config.coordinator.addr, config.coordinator.port).into();
        let session = connect_to_coordinator(coordinator_addr)?;

        println!("Restarting dataflow `{}`", self.dataflow);

        let request = ControlRequest::RestartByName {
            name: self.dataflow.clone(),
            grace_duration: None,
            force: false,
        };
        let reply = send_control_request(&session, &request)?;
        let (old_uuid, new_uuid) = expect_reply!(reply, DataflowRestarted { old_uuid, new_uuid })?;
        println!("dataflow restarted: {old_uuid} -> {new_uuid}");

        Ok(())
    }
}
