use dora_cli::{build, run as dora_run};
use dora_core::{get_uv_path, run};
use dora_tracing::set_up_tracing;
use eyre::WrapErr;
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("python-dataflow-runner").wrap_err("failed to set up tracing subscriber")?;

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

    build("dataflow.yml".to_string(), None, None, true, true)?;

    dora_run("dataflow.yml".to_string(), true)?;

    Ok(())
}
