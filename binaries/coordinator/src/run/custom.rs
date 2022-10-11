use super::command_init_common_env;
use dora_core::descriptor;
use dora_node_api::config::NodeId;
use eyre::{eyre, WrapErr};
use std::{env::consts::EXE_EXTENSION, path::Path};

#[tracing::instrument]
pub(super) fn spawn_custom_node(
    node_id: NodeId,
    node: &descriptor::CustomNode,
    communication: &dora_node_api::config::CommunicationConfig,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut args = node.run.split_ascii_whitespace();
    let cmd = {
        let raw = Path::new(
            args.next()
                .ok_or_else(|| eyre!("`run` field must not be empty"))?,
        );
        let path = if raw.extension().is_none() {
            raw.with_extension(EXE_EXTENSION)
        } else {
            raw.to_owned()
        };
        path.canonicalize()
            .wrap_err_with(|| format!("no node exists at `{}`", path.display()))?
    };

    let mut command = tokio::process::Command::new(cmd);
    command.args(args);
    command_init_common_env(&mut command, &node_id, communication)?;
    command.env(
        "DORA_NODE_RUN_CONFIG",
        serde_yaml::to_string(&node.run_config)
            .wrap_err("failed to serialize custom node run config")?,
    );

    // Injecting the env variable defined in the `yaml` into
    // the node runtime.
    if let Some(envs) = &node.env {
        for (key, value) in envs {
            command.env(key, value.to_string());
        }
    }

    let mut child = command
        .spawn()
        .wrap_err_with(|| format!("failed to run command `{}`", &node.run))?;
    let result = tokio::spawn(async move {
        let status = child.wait().await.context("child process failed")?;
        if status.success() {
            tracing::info!("node {node_id} finished");
            Ok(())
        } else if let Some(code) = status.code() {
            Err(eyre!("node {node_id} failed with exit code: {code}"))
        } else {
            Err(eyre!("node {node_id} failed (unknown exit code)"))
        }
    });
    Ok(result)
}
