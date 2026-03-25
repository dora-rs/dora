use clap::Args;

use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use adora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::{Context, bail};

/// Set a runtime parameter on a node.
///
/// The value is a JSON value (string, number, boolean, object, array).
/// The node receives a ParamUpdate event it can react to.
///
/// Examples:
///
/// Set a numeric parameter:
///   adora param set camera_node fps 60
///
/// Set a string parameter:
///   adora param set camera_node mode '"high-res"'
///
/// Set a JSON object:
///   adora param set camera_node config '{"width": 1920, "height": 1080}'
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Set {
    /// Node ID
    #[clap(value_name = "NODE")]
    pub node: String,

    /// Parameter key
    #[clap(value_name = "KEY")]
    pub key: String,

    /// Parameter value (JSON)
    #[clap(value_name = "VALUE")]
    pub value: String,

    /// Filter by dataflow name or UUID
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    pub dataflow: Option<String>,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Set {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
        let node_id: NodeId = self
            .node
            .parse::<NodeId>()
            .map_err(|e| eyre::eyre!("invalid node ID: {e}"))?;

        let value: serde_json::Value =
            serde_json::from_str(&self.value).wrap_err("invalid JSON value")?;

        let reply = send_control_request(
            &session,
            &ControlRequest::SetParam {
                dataflow_id,
                node_id,
                key: self.key.clone(),
                value: value.clone(),
            },
        )?;

        match reply {
            ControlRequestReply::ParamSet => {
                println!("Set `{}` = {} on node `{}`", self.key, value, self.node);
                Ok(())
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
    }
}
