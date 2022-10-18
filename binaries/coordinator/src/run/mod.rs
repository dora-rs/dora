use self::{custom::spawn_custom_node, runtime::spawn_runtime_node};
use dora_core::descriptor::{self, collect_dora_timers, CoreNodeKind, Descriptor, NodeId};
use dora_node_api::{communication, config::format_duration};
use eyre::{bail, eyre, WrapErr};
use futures::{stream::FuturesUnordered, StreamExt};
use std::{
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
};
use tokio_stream::wrappers::IntervalStream;

mod custom;
mod runtime;

pub async fn run_dataflow(dataflow_path: PathBuf, runtime: &Path) -> eyre::Result<()> {
    let mut runtime = runtime.with_extension(EXE_EXTENSION);
    let descriptor = read_descriptor(&dataflow_path).await.wrap_err_with(|| {
        format!(
            "failed to read dataflow descriptor at {}",
            dataflow_path.display()
        )
    })?;

    let working_dir = dataflow_path
        .canonicalize()
        .context("failed to canoncialize dataflow path")?
        .parent()
        .ok_or_else(|| eyre!("canonicalized dataflow path has no parent"))?
        .to_owned();

    let nodes = descriptor.resolve_aliases();
    let dora_timers = collect_dora_timers(&nodes);
    let communication_config = {
        let mut config = descriptor.communication;
        // add uuid as prefix to ensure isolation
        config.add_topic_prefix(&uuid::Uuid::new_v4().to_string());
        config
    };

    if nodes
        .iter()
        .any(|n| matches!(n.kind, CoreNodeKind::Runtime(_)))
    {
        match runtime.canonicalize() {
            Ok(path) => {
                runtime = path;
            }
            Err(err) => {
                let err = eyre!(err).wrap_err(format!(
                    "There is no runtime at {}, or it is not a file",
                    runtime.display()
                ));
                bail!("{err:?}")
            }
        }
    }

    let mut tasks = FuturesUnordered::new();
    for node in nodes {
        let node_id = node.id.clone();

        match node.kind {
            descriptor::CoreNodeKind::Custom(node) => {
                let result =
                    spawn_custom_node(node_id.clone(), &node, &communication_config, &working_dir)
                        .wrap_err_with(|| format!("failed to spawn custom node {node_id}"))?;
                tasks.push(result);
            }
            descriptor::CoreNodeKind::Runtime(node) => {
                if !node.operators.is_empty() {
                    let result = spawn_runtime_node(
                        &runtime,
                        node_id.clone(),
                        &node,
                        &communication_config,
                        &working_dir,
                    )
                    .wrap_err_with(|| format!("failed to spawn runtime node {node_id}"))?;
                    tasks.push(result);
                }
            }
        }
    }

    for interval in dora_timers {
        let communication_config = communication_config.clone();
        let mut communication =
            tokio::task::spawn_blocking(move || communication::init(&communication_config))
                .await
                .wrap_err("failed to join communication layer init task")?
                .wrap_err("failed to init communication layer")?;
        tokio::spawn(async move {
            let topic = {
                let duration = format_duration(interval);
                format!("dora/timer/{duration}")
            };
            let hlc = dora_message::uhlc::HLC::default();
            let mut stream = IntervalStream::new(tokio::time::interval(interval));
            while (stream.next().await).is_some() {
                let metadata = dora_message::Metadata::new(hlc.new_timestamp());
                let data = metadata.serialize().unwrap();
                communication
                    .publisher(&topic)
                    .unwrap()
                    .publish(&data)
                    .expect("failed to publish timer tick message");
            }
        });
    }

    while let Some(task_result) = tasks.next().await {
        task_result
            .wrap_err("failed to join async task")?
            .wrap_err("custom node failed")?;
    }

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
