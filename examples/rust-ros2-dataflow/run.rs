use eyre::WrapErr;
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build_package("rust-ros2-dataflow-example-node")
        .await
        .wrap_err("failed to build rust-ros2-dataflow-example-node")?;

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
