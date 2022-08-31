use eyre::{bail, Context};
use std::path::Path;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    tokio::fs::create_dir_all("build").await?;

    build_package("cxx-dataflow-example-node-rust-api").await?;
    build_package("cxx-dataflow-example-operator-rust-api").await?;

    build_package("dora-node-api-c").await?;
    build_package("dora-operator-api-c").await?;
    build_cxx_node(
        root,
        &Path::new("node-c-api").join("main.cc").canonicalize()?,
        "node_c_api",
    )
    .await?;
    build_cxx_operator(
        &Path::new("operator-c-api")
            .join("operator.cc")
            .canonicalize()?,
        "operator_c_api",
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
    clang.arg("-l").arg("m");
    clang.arg("-l").arg("rt");
    clang.arg("-l").arg("dl");
    clang.arg("-pthread");
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

async fn build_cxx_operator(path: &Path, out_name: &str) -> eyre::Result<()> {
    let object_file_path = Path::new("../build").join(out_name).with_extension("o");

    let mut compile = tokio::process::Command::new("clang++");
    compile.arg("-c").arg(path);
    compile.arg("-o").arg(&object_file_path);
    compile.arg("-fPIC");
    if let Some(parent) = path.parent() {
        compile.current_dir(parent);
    }
    if !compile.status().await?.success() {
        bail!("failed to compile cxx operator");
    };

    let mut link = tokio::process::Command::new("clang++");
    link.arg("-shared").arg(&object_file_path);
    link.arg("-o")
        .arg(Path::new("../build").join(out_name).with_extension("so"));
    if let Some(parent) = path.parent() {
        link.current_dir(parent);
    }
    if !link.status().await?.success() {
        bail!("failed to create shared library from cxx operator (c api)");
    };

    Ok(())
}
