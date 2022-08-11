use eyre::{bail, Context};
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    tokio::fs::create_dir_all(root.join("build")).await?;

    build_package("cxx-dataflow-example-node-rust-api").await?;
    build_package("cxx-dataflow-example-operator-rust-api").await?;

    build_package("dora-node-api-c").await?;
    build_cxx_node(
        root,
        &Path::new("node-c-api").join("main.cc").canonicalize()?,
        "node_c_api",
    )
    .await?;
    build_package("dora-runtime").await?;

    dora_coordinator::run(dora_coordinator::Command::Run {
        dataflow: Path::new("dataflow.yml").to_owned(),
        runtime: Some(root.join("target").join("release").join("dora-runtime")),
    })
    .await?;

    Ok(())
}

async fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("build").arg("--release");
    cmd.arg("--package").arg(package);
    if !cmd.status().await?.success() {
        bail!("failed to build {package}");
    };
    Ok(())
}

async fn build_cxx_node(root: &Path, path: &Path, out_name: &str) -> eyre::Result<()> {
    let mut clang = tokio::process::Command::new("clang++");
    clang.arg(path);
    clang.arg("-l").arg("dora_node_api_c");
    clang.arg("-L").arg(root.join("target").join("release"));
    clang
        .arg("--output")
        .arg(Path::new("../build").join(out_name));
    if let Some(parent) = path.parent() {
        clang.current_dir(parent);
    }

    if !clang.status().await?.success() {
        bail!("failed to compile c++ node");
    };
    Ok(())
}
