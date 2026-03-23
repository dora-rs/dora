use clap::Args;

use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
    formatting::OutputFormat,
};
use adora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::bail;

/// List all runtime parameters for a node.
///
/// Examples:
///
/// List parameters for a node:
///   adora param list camera_node
///
/// List for a specific dataflow:
///   adora param list camera_node -d my-dataflow
///
/// Output as JSON:
///   adora param list camera_node --format json
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct List {
    /// Node ID
    #[clap(value_name = "NODE")]
    pub node: String,

    /// Filter by dataflow name or UUID
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    pub dataflow: Option<String>,

    /// Output format: table or json
    #[clap(long, default_value = "table")]
    pub format: OutputFormat,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for List {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
        let node_id: NodeId = self.node.parse::<NodeId>().map_err(|e| eyre::eyre!("invalid node ID: {e}"))?;

        let reply = send_control_request(
            &session,
            &ControlRequest::GetParams {
                dataflow_id,
                node_id,
            },
        )?;

        match reply {
            ControlRequestReply::ParamList { params } => {
                if self.format == OutputFormat::Json {
                    let map: serde_json::Map<String, serde_json::Value> =
                        params.into_iter().collect();
                    println!("{}", serde_json::to_string_pretty(&map)?);
                } else if params.is_empty() {
                    println!("No parameters set for node `{}`", self.node);
                } else {
                    println!("Parameters for node `{}`:", self.node);
                    for (key, value) in &params {
                        println!("  {key} = {value}");
                    }
                }
                Ok(())
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
    }
}
