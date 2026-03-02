use std::{net::SocketAddr, path::PathBuf};

use clap::Args;
use eyre::{Context, bail};

use adora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};

use crate::{
    command::{Executable, default_tracing},
    common::connect_to_coordinator,
};

use super::config::ClusterConfig;

/// Stop a running dataflow by name or UUID.
///
/// Connects to the coordinator, resolves the dataflow, and sends a Stop.
/// Use `adora start` afterwards to restart the dataflow.
///
/// Examples:
///
///   adora cluster restart cluster.yml my-dataflow
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

        // Resolve dataflow by name/UUID via List
        let list_raw = session
            .request(&serde_json::to_vec(&ControlRequest::List).unwrap())
            .wrap_err("failed to send List request")?;
        let list_reply: ControlRequestReply =
            serde_json::from_slice(&list_raw).wrap_err("failed to parse List reply")?;

        let dataflow_uuid = match list_reply {
            ControlRequestReply::DataflowList(list) => {
                let matching: Vec<_> = list
                    .0
                    .iter()
                    .filter(|e| {
                        e.id.name.as_deref() == Some(&*self.dataflow)
                            || e.id.uuid.to_string() == self.dataflow
                    })
                    .collect();
                match matching.as_slice() {
                    [entry] => entry.id.uuid,
                    [] => bail!("no running dataflow matching `{}`", self.dataflow),
                    _ => bail!(
                        "multiple running dataflows matching `{}`, use UUID",
                        self.dataflow
                    ),
                }
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            _ => bail!("unexpected reply to List request"),
        };

        println!(
            "Restarting dataflow `{}` ({})",
            self.dataflow, dataflow_uuid
        );

        // Stop the dataflow. A full per-daemon rolling reload would require
        // the coordinator to expose the node-to-daemon mapping to CLI.
        let request = ControlRequest::StopByName {
            name: self.dataflow.clone(),
            grace_duration: None,
            force: false,
        };
        let stop_raw = session
            .request(&serde_json::to_vec(&request).unwrap())
            .wrap_err("failed to send Stop request")?;
        let stop_reply: ControlRequestReply =
            serde_json::from_slice(&stop_raw).wrap_err("failed to parse Stop reply")?;

        match stop_reply {
            ControlRequestReply::DataflowStopped { uuid, .. } => {
                println!("Dataflow {uuid} stopped. Use `adora start` to restart it.");
            }
            ControlRequestReply::Error(err) => bail!("failed to stop dataflow: {err}"),
            other => bail!("unexpected reply: {other:?}"),
        }

        Ok(())
    }
}
