use clap::Args;

use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use adora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::bail;

/// Restart a specific node without stopping the entire dataflow.
///
/// The node process is stopped (with optional grace period), then the
/// daemon's restart loop re-spawns it. The restart counter is incremented.
///
/// Examples:
///
/// Restart a node:
///   adora node restart camera_node
///
/// Restart with a custom grace period:
///   adora node restart camera_node --grace 30s
///
/// Restart a node in a specific dataflow:
///   adora node restart camera_node --dataflow my-dataflow
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Restart {
    /// Node ID to restart
    #[clap(value_name = "NODE")]
    pub node: String,

    /// Filter by dataflow name or UUID
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    pub dataflow: Option<String>,

    /// Grace period before force-killing the node
    #[clap(long, value_name = "DURATION", value_parser = duration_str::parse)]
    pub grace: Option<std::time::Duration>,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Restart {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
        let node_id: NodeId = self.node.clone().into();

        let reply = send_control_request(
            &session,
            &ControlRequest::RestartNode {
                dataflow_id,
                node_id,
                grace_duration: self.grace,
            },
        )?;

        match reply {
            ControlRequestReply::NodeRestarted { node_id, .. } => {
                println!("Node `{node_id}` restart initiated");
                Ok(())
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
    }
}
