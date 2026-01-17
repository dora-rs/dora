use eyre::{Context, bail};
use std::path::Path;

fn main() -> eyre::Result<()> {
    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
        return Ok(());
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    std::fs::create_dir_all("build")?;
    let mut cmd = std::process::Command::new("cmake");
    cmd.arg(format!("-DDORA_ROOT_DIR={}", root.display()));
    cmd.arg("-B").arg("build");
    cmd.arg(".");
    if !cmd.status()?.success() {
        bail!("failed to generating make file");
    }

    let mut cmd = std::process::Command::new("cmake");
    cmd.arg("--build").arg("build");
    if !cmd.status()?.success() {
        bail!("failed to build a cmake-generated project binary tree");
    }

    let mut cmd = std::process::Command::new("cmake");
    cmd.arg("--install").arg("build");
    if !cmd.status()?.success() {
        bail!("failed to build a cmake-generated project binary tree");
    }

    build_package("dora-runtime")?;

    dora_cli::run("dataflow.yml".to_string(), false, None)?;

    Ok(())
}

fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = std::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(package);
    if !cmd.status()?.success() {
        bail!("failed to build {package}");
    }
    Ok(())
}
