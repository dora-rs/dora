use eyre::{ContextCompat, WrapErr};
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

    run(&["python3", "-m", "venv", "../.env"], None).await?;
    let venv = &root.join("examples").join(".env");
    std::env::set_var(
        "VIRTUAL_ENV",
        venv.to_str()
            .context("venv path not valid unicode")?
            .to_owned(),
    );
    let orig_path = std::env::var("PATH")?;
    let venv_bin = venv.join("bin");
    std::env::set_var(
        "PATH",
        format!(
            "{}:{orig_path}",
            venv_bin.to_str().context("venv path not valid unicode")?
        ),
    );

    run(&["pip", "install", "--upgrade", "pip"], None).await?;
    run(&["pip", "install", "-r", "requirements.txt"], None).await?;

    run(&["maturin", "develop"], Some(&root.join("python"))).await?;

    let dataflow = Path::new("dataflow.yml");
    dora_daemon::Daemon::run_dataflow(dataflow).await?;

    Ok(())
}

async fn run(cmd: &[&str], pwd: Option<&Path>) -> eyre::Result<()> {
    let mut run = tokio::process::Command::new(cmd[0]);
    run.args(&cmd[1..]);

    if let Some(pwd) = pwd {
        run.current_dir(pwd);
    }
    if !run.status().await?.success() {
        eyre::bail!("failed to run {cmd:?}");
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
