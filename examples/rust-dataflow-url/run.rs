use eyre::{bail, Context};
use std::path::Path;
use tracing::metadata::LevelFilter;
use tracing_subscriber::Layer;

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

    set_up_tracing().wrap_err("failed to set up tracing")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    dora_daemon::Daemon::run_dataflow(dataflow, false).await?;

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

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(LevelFilter::DEBUG);
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
