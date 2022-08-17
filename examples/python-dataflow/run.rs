use eyre::{bail, Context};
use std::{env, path::Path};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build_package("dora-runtime").await?;

    run(root).await?;

    Ok(())
}

async fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("build").arg("--release");
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
