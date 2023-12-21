use dora_tracing::set_up_tracing;
use eyre::{bail, Context};
use std::path::Path;

#[derive(Debug, Clone, clap::Parser)]
pub struct Args {
    #[clap(long)]
    pub run_dora_runtime: bool,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let Args { run_dora_runtime } = clap::Parser::parse();

    if run_dora_runtime {
        return tokio::task::block_in_place(dora_daemon::run_dora_runtime);
    }

    set_up_tracing("rust-dataflow-runner").wrap_err("failed to set up tracing subscriber")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    dora_daemon::Daemon::run_dataflow(dataflow).await?;

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
