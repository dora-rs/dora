use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{Context, bail};
use std::path::PathBuf;

/// Atomically replace a running node with a new definition under the same id.
///
/// One command instead of `dora node remove` + `dora node add`: the daemon
/// spawns the replacement first (a failure leaves the current incarnation
/// running), then swaps it in and stops the outgoing incarnation. The
/// replacement must keep the node's edges — the same input mappings, and
/// outputs covering everything downstream nodes consume; use remove/add or
/// the mapping commands for topology changes. Inputs arriving during the
/// brief swap window are dropped, not queued.
///
/// Examples:
///
/// Replace a node with a new build of the same definition:
///   dora node replace --dataflow my-dataflow filter --from-yaml filter-node.yml
#[derive(Debug, clap::Args)]
#[clap(verbatim_doc_comment)]
pub struct Replace {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    /// Dataflow name or UUID containing the node
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    dataflow: Option<String>,
    /// Node id to replace (must match the id in the YAML definition)
    node: String,
    /// YAML file with the replacement node definition
    #[clap(long = "from-yaml")]
    from_yaml: PathBuf,
    /// Grace period before force-killing the outgoing incarnation
    /// (e.g. `30s`, `500ms`, `1m`).
    #[clap(long, value_name = "DURATION", value_parser = crate::common::parse_duration)]
    grace: Option<std::time::Duration>,
}

impl Executable for Replace {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;

        let yaml_content = std::fs::read_to_string(&self.from_yaml)
            .wrap_err_with(|| format!("failed to read {}", self.from_yaml.display()))?;
        let node: dora_message::descriptor::Node =
            serde_yaml::from_str(&yaml_content).wrap_err("failed to parse node YAML")?;
        if node.id.to_string() != self.node {
            bail!(
                "node id mismatch: the command names `{}` but the YAML defines `{}`; \
                 `dora node replace` swaps a node under the SAME id",
                self.node,
                node.id
            );
        }

        let reply = send_control_request(
            &session,
            &ControlRequest::ReplaceNode {
                dataflow_id,
                node,
                grace_duration: self.grace,
            },
        )?;

        match reply {
            ControlRequestReply::NodeReplaced { node_id, .. } => {
                println!("Node `{node_id}` replaced in dataflow {dataflow_id}");
            }
            ControlRequestReply::Error(err) => bail!("failed to replace node: {err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
        Ok(())
    }
}
