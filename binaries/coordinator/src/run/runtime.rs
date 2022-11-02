use super::command_init_common_env;
use dora_core::{
    config::NodeId,
    descriptor::{self, EnvValue},
};
use eyre::{eyre, WrapErr};
use std::{collections::BTreeMap, path::Path};

#[tracing::instrument(skip(node))]
pub fn spawn_runtime_node(
    runtime: &Path,
    node_id: NodeId,
    node: &descriptor::RuntimeNode,
    envs: &Option<BTreeMap<String, EnvValue>>,
    communication: &dora_core::config::CommunicationConfig,
    working_dir: &Path,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut command = tokio::process::Command::new(runtime);
    command_init_common_env(&mut command, &node_id, communication)?;
    command.env(
        "DORA_OPERATORS",
        serde_yaml::to_string(&node.operators)
            .wrap_err("failed to serialize custom node run config")?,
    );

    // Injecting the env variable defined in the `yaml` into
    // the node runtime.
    if let Some(envs) = &envs {
        for (key, value) in envs {
            command.env(key, value.to_string());
        }
    }

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
            if let Some(meaning) = exit_code_meaning(code) {
                Err(eyre!(
                    "runtime node {node_id} failed with exit code: {code}, meaning: {meaning}"
                ))
            } else {
                Err(eyre!(
                    "runtime node {node_id} failed with exit code: {code} with unknwon meaning."
                ))
            }
        } else {
            Err(eyre!("runtime node {node_id} failed (unknown exit code)"))
        }
    });
    Ok(result)
}

fn exit_code_meaning(code: i32) -> Option<String> {
    if cfg!(unix) {
        let meaning = match code {
            0 => "Success",
            1 => "Catchall for general errors",
            2 => "Misuse of shell built-ins",
            64 => "Usage Error",
            65 => "Data Error",
            66 => "No Input",
            67 => "No User",
            68 => "No Host",
            69 => "Service Unavailable",
            70 => "Software Error",
            71 => "OS Error",
            72 => "OS File Error",
            73 => "Cannot Create",
            74 => "IO Error",
            75 => "Temporary Failure",
            76 => "Protocol Error",
            77 => "No Permission",
            78 => "Config Error",
            126 => "Command invoked cannot execute",
            127 => "Command not found",
            128 => "Invalid argument to `exit`",
            256.. => "Exit status out of range",
            _ => "Unknown Error code.",
        }
        .to_string();
        Some(meaning)
    } else {
        None
    }
}
