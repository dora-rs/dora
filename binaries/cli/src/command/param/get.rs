use clap::Args;

use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use adora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::bail;

/// Get a single runtime parameter value.
///
/// Examples:
///
/// Get a parameter:
///   adora param get camera_node fps
///
/// Get from a specific dataflow:
///   adora param get camera_node fps -d my-dataflow
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Get {
    /// Node ID
    #[clap(value_name = "NODE")]
    pub node: String,

    /// Parameter key
    #[clap(value_name = "KEY")]
    pub key: String,

    /// Filter by dataflow name or UUID
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    pub dataflow: Option<String>,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Get {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
        let node_id: NodeId = self.node.clone().into();

        let reply = send_control_request(
            &session,
            &ControlRequest::GetParam {
                dataflow_id,
                node_id,
                key: self.key.clone(),
            },
        )?;

        match reply {
            ControlRequestReply::ParamValue { value, .. } => {
                println!("{value}");
                Ok(())
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
    }
}
