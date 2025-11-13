use dora_cli::{build, run as dora_run};
use dora_core::get_uv_path;
use eyre::WrapErr;
use std::path::Path;

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let uv = get_uv_path()?;
    let mut uv = std::process::Command::new(&uv);
    uv.arg("venv").arg("-p").arg("3.10").arg("--seed");
    uv.spawn()
        .wrap_err("failed to create venv")?
        .wait()
        .wrap_err("failed to wait for venv creation")?;

    let uv = get_uv_path()?;
    let mut uv = std::process::Command::new(&uv);
    uv.arg("pip")
        .arg("install")
        .arg("-e")
        .arg("../../apis/python/node");
    uv.spawn()
        .wrap_err("Unable to install develop dora-rs API")?
        .wait()
        .wrap_err("failed to wait for pip install")?;

    build("dataflow.yml".to_string(), None, None, true, true)?;

    dora_run("dataflow.yml".to_string(), true)?;

    Ok(())
}
