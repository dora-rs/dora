use eyre::{bail, Context};
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    tokio::fs::create_dir_all("build").await?;

    build_package("dora-runtime").await?;
    build_package("dora-node-api-c").await?;
    build_package("dora-operator-api-c").await?;
    build_c_node(root, "node.c", "c_node").await?;
    build_c_node(root, "sink.c", "c_sink").await?;
    build_c_operator().await?;

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

async fn build_c_node(root: &Path, name: &str, out_name: &str) -> eyre::Result<()> {
    let mut clang = tokio::process::Command::new("clang");
    clang.arg(name);
    clang.arg("-l").arg("dora_node_api_c");
    clang.arg("-l").arg("m");
    clang.arg("-l").arg("rt");
    clang.arg("-l").arg("dl");
    clang.arg("-pthread");
    clang.arg("-L").arg(root.join("target").join("release"));
    clang.arg("--output").arg(Path::new("build").join(out_name));
    if !clang.status().await?.success() {
        bail!("failed to compile c node");
    };
    Ok(())
}

async fn build_c_operator() -> eyre::Result<()> {
    let mut compile = tokio::process::Command::new("clang");
    compile.arg("-c").arg("operator.c");
    compile.arg("-o").arg("build/operator.o");
    compile.arg("-fPIC");
    if !compile.status().await?.success() {
        bail!("failed to compile c operator");
    };

    let mut link = tokio::process::Command::new("clang");
    link.arg("-shared").arg("build/operator.o");
    link.arg("-o").arg("build/operator.so");
    if !link.status().await?.success() {
        bail!("failed to link c operator");
    };

    Ok(())
}
