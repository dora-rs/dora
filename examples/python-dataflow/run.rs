use eyre::{bail, Context};
use std::{env, path::Path};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build_package("dora-daemon").await?;

    run(root).await?;

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

async fn run(_root: &Path) -> eyre::Result<()> {
    let mut run = tokio::process::Command::new("sh");
    run.arg("./run.sh");
    if !run.status().await?.success() {
        bail!("failed to run python example.");
    };
    Ok(())
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
