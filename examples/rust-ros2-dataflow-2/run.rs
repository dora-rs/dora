use eyre::WrapErr;
use std::path::Path;
use tracing_subscriber::{
    filter::{FilterExt, LevelFilter},
    prelude::*,
    EnvFilter, Registry,
};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing()?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build_package("rust-ros2-dataflow-example-node-2")
        .await
        .wrap_err("failed to build rust-ros2-dataflow-example-node-2")?;

    let dataflow = Path::new("dataflow.yml");
    dora_daemon::Daemon::run_dataflow(dataflow).await?;

    Ok(())
}

async fn build_package(name: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(name);
    cmd.arg("--features").arg("ros2");
    if !cmd.status().await?.success() {
        eyre::bail!("failed to build dataflow");
    };
    Ok(())
}

pub fn set_up_tracing() -> eyre::Result<()> {
    // Filter log using `RUST_LOG`. More useful for CLI.
    let filter = EnvFilter::from_default_env().or(LevelFilter::DEBUG);
    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(filter);

    let registry = Registry::default().with(stdout_log);

    tracing::subscriber::set_global_default(registry)
        .context("failed to set tracing global subscriber")
}
