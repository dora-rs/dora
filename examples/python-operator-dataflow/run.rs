use dora_core::{get_pip_path, get_python_path, run};
use dora_tracing::set_up_tracing;
use eyre::{ContextCompat, WrapErr};
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("python-operator-dataflow-runner")?;

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
        get_python_path().context("Could not get pip binary")?,
        &["-m", "pip", "install", "--upgrade", "pip"],
        None,
    )
    .await
    .context("failed to install pip")?;
    run(
        get_pip_path().context("Could not get pip binary")?,
        &["install", "-r", "requirements.txt"],
        None,
    )
    .await
    .context("pip install failed")?;

    run(
        "maturin",
        &["develop"],
        Some(&root.join("apis").join("python").join("node")),
    )
    .await
    .context("maturin develop failed")?;

    let dataflow = Path::new("dataflow.yml");
    dora_daemon::Daemon::run_dataflow(dataflow).await?;

    Ok(())
}
