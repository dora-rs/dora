use eyre::{bail, Context};
use futures::stream;
use std::{
    net::{Ipv4Addr, SocketAddr},
    path::Path,
};
use tokio::task::JoinSet;
use tracing::metadata::LevelFilter;
use tracing_subscriber::Layer;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    build_package("dora-runtime").await?;
    let dora_runtime_path = Some(root.join("target").join("debug").join("dora-runtime"));

    let (coordinator_port, coordinator) = dora_coordinator::start(
        dora_coordinator::Args {
            port: Some(0),
            run_dataflow: Some(dataflow.to_path_buf()),
            dora_runtime_path: dora_runtime_path.clone(),
        },
        stream::empty(),
    )
    .await?;
    let coordinator_addr = SocketAddr::new(Ipv4Addr::LOCALHOST.into(), coordinator_port);
    let daemon_a = dora_daemon::Daemon::run(
        coordinator_addr,
        "A".into(),
        dora_runtime_path.clone(),
        stream::empty(),
    );
    let daemon_b = dora_daemon::Daemon::run(
        coordinator_addr,
        "B".into(),
        dora_runtime_path,
        stream::empty(),
    );

    let mut tasks = JoinSet::new();
    tasks.spawn(coordinator);
    tasks.spawn(daemon_a);
    tasks.spawn(daemon_b);

    while let Some(res) = tasks.join_next().await {
        res.unwrap()?;
    }

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

async fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(package);
    if !cmd.status().await?.success() {
        bail!("failed to build {package}");
    };
    Ok(())
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(LevelFilter::DEBUG);
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
