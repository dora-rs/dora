use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use adora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::ControlRequestReply,
    id::{DataId, NodeId},
};
use eyre::bail;

/// Disconnect two nodes in a running dataflow (remove a mapping).
///
/// Example:
///   adora node disconnect demo sender/value filter/input
#[derive(Debug, clap::Args)]
pub struct Disconnect {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    #[clap(long)]
    dataflow: Option<String>,
    /// Source in "node/output" format.
    source: String,
    /// Target in "node/input" format.
    target: String,
}

fn parse_node_port(s: &str) -> eyre::Result<(NodeId, DataId)> {
    let parts: Vec<&str> = s.splitn(2, '/').collect();
    if parts.len() != 2 || parts[0].is_empty() || parts[1].is_empty() {
        bail!("expected 'node_id/port_id', got '{s}'");
    }
    let node: NodeId = parts[0].parse().map_err(|e| eyre::eyre!("invalid node ID: {e}"))?;
    let port: DataId = parts[1].parse().map_err(|e| eyre::eyre!("invalid port ID: {e}"))?;
    Ok((node, port))
}

impl Executable for Disconnect {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;

        let (source_node, source_output) = parse_node_port(&self.source)?;
        let (target_node, target_input) = parse_node_port(&self.target)?;

        let reply = send_control_request(
            &session,
            &ControlRequest::RemoveMapping {
                dataflow_id,
                source_node: source_node.clone(),
                source_output: source_output.clone(),
                target_node: target_node.clone(),
                target_input: target_input.clone(),
            },
        )?;

        match reply {
            ControlRequestReply::MappingRemoved { .. } => {
                println!("{source_node}/{source_output} -x- {target_node}/{target_input}");
            }
            ControlRequestReply::Error(err) => bail!("failed to disconnect: {err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
        Ok(())
    }
}
