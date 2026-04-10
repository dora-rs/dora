use std::{net::SocketAddr, path::PathBuf};

use clap::Args;
use eyre::{Context, bail};

use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};

use crate::{
    command::{Executable, default_tracing},
    common::connect_to_coordinator,
};

use super::config::ClusterConfig;

/// Restart a running dataflow by name or UUID.
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

    /// Name or UUID of the dataflow to restart
    #[clap(value_name = "DATAFLOW")]
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
        let reply_raw = session
            .request(&serde_json::to_vec(&request).unwrap())
            .wrap_err("failed to send Restart request")?;
        let reply: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse Restart reply")?;

        match reply {
            ControlRequestReply::DataflowRestarted { old_uuid, new_uuid } => {
                println!("dataflow restarted: {old_uuid} -> {new_uuid}");
            }
            ControlRequestReply::Error(err) => bail!("failed to restart dataflow: {err}"),
            other => bail!("unexpected reply: {other:?}"),
        }

        Ok(())
    }
}
