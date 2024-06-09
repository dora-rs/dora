use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use dora_core::config::NodeId;
use eyre::{Context, Report};
use tokio::task::JoinHandle;
use uuid::Uuid;

pub fn log_path(working_dir: &Path, dataflow_id: &Uuid, node_id: &NodeId) -> PathBuf {
    let dataflow_dir = working_dir.join("out").join(dataflow_id.to_string());
    dataflow_dir.join(format!("log_{node_id}.txt"))
}

pub async fn pretty_string_dataflow_errors(
    node_errors: BTreeMap<NodeId, JoinHandle<Result<Option<Report>, Report>>>,
) -> eyre::Result<String> {
    let mut output = "".to_owned();
    for (_node, error) in node_errors {
        use std::fmt::Write;
        if let Some(error) = error
            .await
            .context("Could not join error")?
            .context("Could not get error logs")?
        {
            write!(&mut output, "- {error}").context("could not write error")?;
        }
    }
    Ok(output)
}
