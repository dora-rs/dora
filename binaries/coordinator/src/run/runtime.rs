use super::command_init_common_env;
use dora_core::descriptor;
use dora_node_api::config::NodeId;
use eyre::{eyre, WrapErr};
use std::path::Path;

#[tracing::instrument(skip(node))]
pub fn spawn_runtime_node(
    runtime: &Path,
    node_id: NodeId,
    node: &descriptor::RuntimeNode,
    communication: &dora_node_api::config::CommunicationConfig,
    working_dir: &Path,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut command = tokio::process::Command::new(runtime);
    command_init_common_env(&mut command, &node_id, communication)?;
    command.env(
        "DORA_OPERATORS",
        serde_yaml::to_string(&node.operators)
            .wrap_err("failed to serialize custom node run config")?,
    );
    command.current_dir(working_dir);

    let mut child = command
        .spawn()
        .wrap_err_with(|| format!("failed to run runtime at `{}`", runtime.display()))?;
    let result = tokio::spawn(async move {
        let status = child.wait().await.context("child process failed")?;
        if status.success() {
            tracing::info!("runtime node {node_id} finished");
            Ok(())
        } else if let Some(code) = status.code() {
            Err(eyre!(
                "runtime node {node_id} failed with exit code: {code}"
            ))
        } else {
            Err(eyre!("runtime node {node_id} failed (unknown exit code)"))
        }
    });
    Ok(result)
}
