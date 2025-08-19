use dora_core::{get_uv_path, run};
use dora_tracing::set_up_tracing;
use eyre::{bail, WrapErr};
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("python-dataflow-runner")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let uv = get_uv_path().context("Could not get uv binary")?;

    run(&uv, &["venv", "-p", "3.10", "--seed"], None)
        .await
        .context("failed to create venv")?;

    run(
        &uv,
        &[
            "pip",
            "install",
            "-e",
            "../../apis/python/node",
            "--reinstall",
        ],
        None,
    )
    .await
    .context("Unable to install develop dora-rs API")?;

    let dataflow = Path::new("dataflow.yml");
    run_dataflow(dataflow).await?;

    Ok(())
}

async fn run_dataflow(dataflow: &Path) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();

    // First build the dataflow (install requirements)
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--release");
    cmd.arg("--").arg("build").arg(dataflow).arg("--uv");
    if !cmd.status().await?.success() {
        bail!("failed to run dataflow");
    };

    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--release");
    cmd.arg("--").arg("run").arg(dataflow).arg("--uv");
    if !cmd.status().await?.success() {
        bail!("failed to run dataflow");
    };
    Ok(())
}
