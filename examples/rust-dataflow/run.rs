use dora_core::{
    daemon_messages::{SpawnDataflowNodes, SpawnNodeParams},
    descriptor::{CoreNodeKind, Descriptor},
};
use eyre::{bail, Context};
use std::{collections::BTreeMap, path::Path};
use tokio::fs;
use uuid::Uuid;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    let working_dir = dataflow
        .canonicalize()
        .context("failed to canoncialize dataflow path")?
        .parent()
        .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
        .to_owned();

    let nodes = read_descriptor(dataflow).await?.resolve_aliases();
    let mut custom_nodes = BTreeMap::new();
    for node in nodes {
        match node.kind {
            CoreNodeKind::Runtime(_) => todo!(),
            CoreNodeKind::Custom(n) => {
                custom_nodes.insert(
                    node.id.clone(),
                    SpawnNodeParams {
                        node_id: node.id,
                        node: n,
                        working_dir: working_dir.clone(),
                    },
                );
            }
        }
    }

    let spawn_command = SpawnDataflowNodes {
        dataflow_id: Uuid::new_v4(),
        nodes: custom_nodes,
    };

    dora_daemon::Daemon::run_dataflow(spawn_command).await?;

    Ok(())
}

async fn build_dataflow(dataflow: &Path) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--").arg("build").arg(dataflow);
    if !cmd.status().await?.success() {
        bail!("failed to build dataflow");
    };
    Ok(())
}

pub async fn read_descriptor(file: &Path) -> eyre::Result<Descriptor> {
    let descriptor_file = fs::read(file).await.context("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
