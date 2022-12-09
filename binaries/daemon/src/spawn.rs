use dora_core::{
    daemon_messages::{NodeConfig, SpawnNodeParams},
    descriptor::{resolve_path, source_is_url},
};
use dora_download::download_file;
use eyre::{eyre, WrapErr};
use std::{env::consts::EXE_EXTENSION, path::Path};

#[tracing::instrument]
pub async fn spawn_node(
    params: SpawnNodeParams,
    daemon_port: u16,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<()>>> {
    let SpawnNodeParams {
        node_id,
        node,
        working_dir,
    } = params;

    let resolved_path = if source_is_url(&node.source) {
        // try to download the shared library
        let target_path = Path::new("build")
            .join(node_id.to_string())
            .with_extension(EXE_EXTENSION);
        download_file(&node.source, &target_path)
            .await
            .wrap_err("failed to download custom node")?;
        target_path.clone()
    } else {
        resolve_path(&node.source, &working_dir)
            .wrap_err_with(|| format!("failed to resolve node source `{}`", node.source))?
    };
    let node_config = NodeConfig {
        node_id: node_id.clone(),
        run_config: node.run_config.clone(),
        daemon_port,
    };

    let mut command = tokio::process::Command::new(&resolved_path);
    if let Some(args) = &node.args {
        command.args(args.split_ascii_whitespace());
    }
    command.env(
        "DORA_NODE_CONFIG",
        serde_yaml::to_string(&node_config).wrap_err("failed to serialize node config")?,
    );
    command.current_dir(working_dir);

    // Injecting the env variable defined in the `yaml` into
    // the node runtime.
    if let Some(envs) = node.envs {
        for (key, value) in envs {
            command.env(key, value.to_string());
        }
    }

    let mut child = command.spawn().wrap_err_with(move || {
        format!(
            "failed to run source path: `{}` with args `{}`",
            resolved_path.display(),
            node.args.as_deref().unwrap_or_default()
        )
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
