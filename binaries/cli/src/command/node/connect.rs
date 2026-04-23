use super::parse_node_port;
use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::bail;

/// Connect two nodes in a running dataflow (add a mapping).
///
/// Example:
///   dora node connect demo sender/value filter/input
#[derive(Debug, clap::Args)]
pub struct Connect {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    /// Dataflow name or UUID to add the mapping in
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    dataflow: Option<String>,
    /// Source in "node/output" format.
    source: String,
    /// Target in "node/input" format.
    target: String,
}

impl Executable for Connect {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;

        let (source_node, source_output) = parse_node_port(&self.source)?;
        let (target_node, target_input) = parse_node_port(&self.target)?;

        let reply = send_control_request(
            &session,
            &ControlRequest::AddMapping {
                dataflow_id,
                source_node: source_node.clone(),
                source_output: source_output.clone(),
                target_node: target_node.clone(),
                target_input: target_input.clone(),
            },
        )?;

        match reply {
            ControlRequestReply::MappingAdded { .. } => {
                println!("{source_node}/{source_output} -> {target_node}/{target_input}");
            }
            ControlRequestReply::Error(err) => bail!("failed to connect: {err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
        Ok(())
    }
}
