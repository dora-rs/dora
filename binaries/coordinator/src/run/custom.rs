use super::command_init_common_env;
use dora_core::{
    config::NodeId,
    descriptor::{self, source_is_url},
};
use dora_download::download_file;
use eyre::{eyre, WrapErr};
use std::{env::consts::EXE_EXTENSION, path::Path};

#[tracing::instrument]
pub(super) fn spawn_custom_node(
    node_id: NodeId,
    node: &descriptor::CustomNode,
    communication: &dora_core::config::CommunicationConfig,
    working_dir: &Path,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut temp_file = None;
    let path = if source_is_url(&node.source) {
        // try to download the shared library
        let tmp = download_file(&node.source).wrap_err("failed to download custom node")?;
        let path = tmp.path().to_owned();
        temp_file = Some(tmp);
        path
    } else {
        let raw = Path::new(&node.source);
        if raw.extension().is_none() {
            raw.with_extension(EXE_EXTENSION)
        } else {
            raw.to_owned()
        }
    };

    let cmd = working_dir
        .join(&path)
        .canonicalize()
        .wrap_err_with(|| format!("no node exists at `{}`", path.display()))?;

    let mut command = tokio::process::Command::new(cmd);
    if let Some(args) = &node.args {
        command.args(args.split_ascii_whitespace());
    }
    command_init_common_env(&mut command, &node_id, communication)?;
    command.env(
        "DORA_NODE_RUN_CONFIG",
        serde_yaml::to_string(&node.run_config)
            .wrap_err("failed to serialize custom node run config")?,
    );
    command.current_dir(working_dir);

    // Injecting the env variable defined in the `yaml` into
    // the node runtime.
    if let Some(envs) = &node.env {
        for (key, value) in envs {
            command.env(key, value.to_string());
        }
    }

    let mut child = command.spawn().wrap_err_with(|| {
        format!(
            "failed to run executable `{}` with args `{}`",
            path.display(),
            node.args.as_deref().unwrap_or_default()
        )
    })?;
    let result = tokio::spawn(async move {
        let status = child.wait().await.context("child process failed")?;
        std::mem::drop(temp_file);
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
