use super::command_init_common_env;
use dora_core::{
    config::NodeId,
    descriptor::{self, source_is_path, source_is_url, EnvValue},
};
use dora_download::download_file;
use eyre::{eyre, WrapErr};
use std::{collections::BTreeMap, env::consts::EXE_EXTENSION, path::Path};

#[tracing::instrument]
pub(super) async fn spawn_custom_node(
    node_id: NodeId,
    node: &descriptor::CustomNode,
    envs: &Option<BTreeMap<String, EnvValue>>,
    communication: &dora_core::config::CommunicationConfig,
    working_dir: &Path,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut is_path = source_is_path(&node.source, working_dir);

    // Use path if node.source correspond to a path
    let mut command = if let Ok(path) = &is_path {
        tokio::process::Command::new(path)
    // Use url if node.source is a url
    } else if source_is_url(&node.source) {
        // try to download the shared library
        let target_path = Path::new("build")
            .join(node_id.to_string())
            .with_extension(EXE_EXTENSION);
        download_file(&node.source, &target_path)
            .await
            .wrap_err("failed to download custom node")?;
        is_path = Ok(target_path.clone());
        tokio::process::Command::new(target_path)
    // Use node.source as a command if it is not in path and is not a url
    } else if cfg!(target_os = "windows") {
        let mut cmd = tokio::process::Command::new("cmd");
        cmd.args(["/C", &node.source]);
        cmd
    } else {
        let mut cmd = tokio::process::Command::new("sh");

        // Argument are passed in the command string.
        // Arguments passed after string enclosure will be ignored.
        cmd.args([
            "-c",
            &format!("{} {}", node.source, &node.args.clone().unwrap_or_default()),
        ]);
        cmd
    };
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
    if let Some(envs) = envs {
        for (key, value) in envs {
            command.env(key, value.to_string());
        }
    }

    let mut child = command.spawn().wrap_err_with(|| {
        if let Ok(path) = is_path {
            format!(
                "failed to run source path: `{}` with args `{}`",
                path.display(),
                node.args.as_deref().unwrap_or_default()
            )
        } else {
            format!(
                "failed to run command: `{}` with args `{}`",
                node.source,
                node.args.as_deref().unwrap_or_default()
            )
        }
    })?;
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
