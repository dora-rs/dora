use clap::Args;

use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use adora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::bail;

/// Stop a specific node without stopping the entire dataflow.
///
/// The node process is stopped and will NOT be restarted (regardless of
/// restart policy). If all nodes in a dataflow are stopped individually,
/// the dataflow is marked as finished.
///
/// Examples:
///
/// Stop a node:
///   adora node stop camera_node
///
/// Stop with a custom grace period:
///   adora node stop camera_node --grace 30s
///
/// Stop a node in a specific dataflow:
///   adora node stop camera_node --dataflow my-dataflow
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Stop {
    /// Node ID to stop
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

impl Executable for Stop {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
        let node_id: NodeId = self.node.parse::<NodeId>().map_err(|e| eyre::eyre!("invalid node ID: {e}"))?;

        let reply = send_control_request(
            &session,
            &ControlRequest::StopNode {
                dataflow_id,
                node_id,
                grace_duration: self.grace,
            },
        )?;

        match reply {
            ControlRequestReply::NodeStopped { node_id, .. } => {
                println!("Node `{node_id}` stopped");
                Ok(())
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
    }
}
