use eyre::WrapErr;
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");

    run_dataflow(dataflow).await?;

    Ok(())
}

async fn run_dataflow(dataflow: &Path) -> eyre::Result<()> {
    let mut cmd = tokio::process::Command::new("dora-daemon");
    cmd.arg("--run-dataflow").arg(dataflow);
    if !cmd.status().await?.success() {
        eyre::bail!("failed to build dataflow");
    };
    Ok(())
}
