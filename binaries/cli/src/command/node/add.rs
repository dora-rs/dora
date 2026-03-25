use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use adora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{Context, bail};
use std::path::PathBuf;

/// Add a node to a running dataflow.
#[derive(Debug, clap::Args)]
pub struct Add {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    #[clap(long)]
    dataflow: Option<String>,
    #[clap(long = "from-yaml")]
    from_yaml: PathBuf,
}

impl Executable for Add {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;

        let yaml_content = std::fs::read_to_string(&self.from_yaml)
            .wrap_err_with(|| format!("failed to read {}", self.from_yaml.display()))?;
        let node: adora_message::descriptor::Node =
            serde_yaml::from_str(&yaml_content).wrap_err("failed to parse node YAML")?;

        let reply = send_control_request(&session, &ControlRequest::AddNode { dataflow_id, node })?;

        match reply {
            ControlRequestReply::NodeAdded { node_id, .. } => {
                println!("Node `{node_id}` added to dataflow {dataflow_id}");
            }
            ControlRequestReply::Error(err) => bail!("failed to add node: {err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
        Ok(())
    }
}
