use dora_common::descriptor::{self, Descriptor, NodeKind};
use dora_node_api::config::NodeId;
use eyre::{bail, eyre, WrapErr};
use futures::{stream::FuturesUnordered, StreamExt};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Dora coordinator")]
enum Command {
    #[clap(about = "Print Graph")]
    Visualize { dataflow: PathBuf },
    #[clap(about = "Run dataflow pipeline")]
    Run {
        dataflow: PathBuf,
        runtime: Option<PathBuf>,
    },
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let command = clap::Parser::parse();
    match command {
        Command::Visualize { dataflow: file } => {
            let descriptor = read_descriptor(&file).await?;
            let visualized = descriptor
                .visualize_as_mermaid()
                .context("failed to visualize descriptor")?;
            println!("{visualized}");
            println!(
                "Paste the above output on https://mermaid.live/ or in a \
        ```mermaid code block on GitHub to display it."
            );
        }
        Command::Run { dataflow, runtime } => {
            let runtime_path = runtime.unwrap_or_else(|| {
                std::env::args()
                    .next()
                    .map(PathBuf::from)
                    .unwrap_or_default()
                    .with_file_name("dora-runtime")
            });
            run_dataflow(dataflow.clone(), &runtime_path)
                .await
                .wrap_err_with(|| format!("failed to run dataflow at {}", dataflow.display()))?
        }
    }

    Ok(())
}

async fn run_dataflow(dataflow_path: PathBuf, runtime: &Path) -> eyre::Result<()> {
    let Descriptor {
        mut communication,
        nodes,
    } = read_descriptor(&dataflow_path).await.wrap_err_with(|| {
        format!(
            "failed to read dataflow descriptor at {}",
            dataflow_path.display()
        )
    })?;

    if nodes.iter().any(|n| matches!(n.kind, NodeKind::Runtime(_))) && !runtime.is_file() {
        bail!(
            "There is no runtime at {}, or it is not a file",
            runtime.display()
        );
    }

    // add uuid as prefix to ensure isolation
    communication.add_topic_prefix(&uuid::Uuid::new_v4().to_string());

    let mut tasks = FuturesUnordered::new();
    for node in nodes {
        let node_id = node.id.clone();

        match node.kind {
            descriptor::NodeKind::Custom(node) => {
                let result = spawn_custom_node(node_id.clone(), &node, &communication)
                    .wrap_err_with(|| format!("failed to spawn custom node {node_id}"))?;
                tasks.push(result);
            }
            descriptor::NodeKind::Runtime(node) => {
                if !node.operators.is_empty() {
                    let result =
                        spawn_runtime_node(runtime, node_id.clone(), &node, &communication)
                            .wrap_err_with(|| format!("failed to spawn runtime node {node_id}"))?;
                    tasks.push(result);
                }
            }
        }
    }

    while let Some(task_result) = tasks.next().await {
        task_result
            .wrap_err("failed to join async task")?
            .wrap_err("custom node failed")?;
    }

    Ok(())
}

fn spawn_custom_node(
    node_id: NodeId,
    node: &descriptor::CustomNode,
    communication: &dora_node_api::config::CommunicationConfig,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut args = node.run.split_ascii_whitespace();
    let cmd = args
        .next()
        .ok_or_else(|| eyre!("`run` field must not be empty"))?;
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
            println!("node {node_id} finished");
            Ok(())
        } else if let Some(code) = status.code() {
            Err(eyre!("node {node_id} failed with exit code: {code}"))
        } else {
            Err(eyre!("node {node_id} failed (unknown exit code)"))
        }
    });
    Ok(result)
}

fn spawn_runtime_node(
    runtime: &Path,
    node_id: NodeId,
    node: &descriptor::RuntimeNode,
    communication: &dora_node_api::config::CommunicationConfig,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut command = tokio::process::Command::new(runtime);
    command_init_common_env(&mut command, &node_id, communication)?;
    command.env(
        "DORA_OPERATORS",
        serde_yaml::to_string(&node.operators)
            .wrap_err("failed to serialize custom node run config")?,
    );

    let mut child = command
        .spawn()
        .wrap_err_with(|| format!("failed to run runtime at `{}`", runtime.display()))?;
    let result = tokio::spawn(async move {
        let status = child.wait().await.context("child process failed")?;
        if status.success() {
            println!("runtime node {node_id} finished");
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

fn command_init_common_env(
    command: &mut tokio::process::Command,
    node_id: &NodeId,
    communication: &dora_node_api::config::CommunicationConfig,
) -> Result<(), eyre::Error> {
    command.env(
        "DORA_NODE_ID",
        serde_yaml::to_string(&node_id).wrap_err("failed to serialize custom node ID")?,
    );
    command.env(
        "DORA_COMMUNICATION_CONFIG",
        serde_yaml::to_string(communication)
            .wrap_err("failed to serialize communication config")?,
    );
    Ok(())
}

async fn read_descriptor(file: &Path) -> Result<Descriptor, eyre::Error> {
    let descriptor_file = tokio::fs::read(file)
        .await
        .context("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}
