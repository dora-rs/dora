use dora_core::{get_python_path, run};
use dora_tracing::set_up_tracing;
use eyre::{bail, ContextCompat, WrapErr};
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("python-dataflow-runner")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    run(
        get_python_path().context("Could not get python binary")?,
        &["-m", "venv", "../.env"],
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
    // bin folder is named Scripts on windows.
    // 🤦‍♂️ See: https://github.com/pypa/virtualenv/commit/993ba1316a83b760370f5a3872b3f5ef4dd904c1
    let venv_bin = if cfg!(windows) {
        venv.join("Scripts")
    } else {
        venv.join("bin")
    };

    if cfg!(windows) {
        std::env::set_var(
            "PATH",
            format!(
                "{};{orig_path}",
                venv_bin.to_str().context("venv path not valid unicode")?
            ),
        );
    } else {
        std::env::set_var(
            "PATH",
            format!(
                "{}:{orig_path}",
                venv_bin.to_str().context("venv path not valid unicode")?
            ),
        );
    }

    run(
        "pip",
        &["install", "maturin"],
        Some (venv)
    )
    .await
    .context("pip install maturin failed")?;

    run(
        "maturin",
        &["develop"],
        Some(&root.join("apis").join("python").join("node")),
    )
    .await
    .context("maturin develop failed")?;

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
    cmd.arg("--").arg("build").arg(dataflow);
    if !cmd.status().await?.success() {
        bail!("failed to run dataflow");
    };

    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--")
        .arg("daemon")
        .arg("--run-dataflow")
        .arg(dataflow);
    if !cmd.status().await?.success() {
        bail!("failed to run dataflow");
    };
    Ok(())
}
