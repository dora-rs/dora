use dora_tracing::set_up_tracing;
use eyre::{ContextCompat, WrapErr};
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("python-dataflow-runner")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    run(
        &[
            get_python_path()
                .context("Could not get python binary")?
                .to_str()
                .context("Could not convert python path to string")?,
            "-m",
            "venv",
            ".env",
        ],
        None,
    )
    .await
    .context("failed to create venv")?;
    let venv = &root.join("examples").join(".env");
    std::env::set_var(
        "VIRTUAL_ENV",
        venv.to_str().context("venv path not valid unicode")?,
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

    run(&["pip", "install", "--upgrade", "pip"], None)
        .await
        .context("failed to install pip")?;
    run(&["pip", "install", "-r", "requirements.txt"], None)
        .await
        .context("pip install failed")?;

    run(
        &["maturin", "develop"],
        Some(&root.join("apis").join("python").join("node")),
    )
    .await
    .context("maturin develop failed")?;

    let dataflow = Path::new("dataflow.yml");
    dora_daemon::Daemon::run_dataflow(dataflow).await?;

    Ok(())
}
