use eyre::{Context, bail};
use std::{env::consts::EXE_SUFFIX, path::Path};

fn main() -> eyre::Result<()> {
    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
        return Ok(());
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../");
    let target = root.join("target");
    let example_dir = root.join(file!()).parent().unwrap().to_path_buf();
    std::env::set_current_dir(&example_dir).wrap_err("failed to set working dir")?;

    std::fs::create_dir_all("build")?;

    build_package("dora-node-api-cxx", &["ros2-bridge"])?;
    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("install");

    let shared = |main: &Path| -> eyre::Result<Vec<std::path::PathBuf>> {
        Ok(vec![
            dunce::canonicalize(main)?,
            dunce::canonicalize(node_cxxbridge.join("dora-node-api.cc"))?,
            dunce::canonicalize(node_cxxbridge.join("ros2-bridge/impl.cc"))?,
        ])
    };
    let link_args = [
        "-I",
        node_cxxbridge.as_os_str().to_str().unwrap(),
        "-l",
        "dora_node_api_cxx",
    ];

    let host_main = dunce::canonicalize(Path::new("./").join("host.cc"))?;
    let client_main = dunce::canonicalize(Path::new("./").join("client.cc"))?;
    let host_srcs = shared(&host_main)?;
    let client_srcs = shared(&client_main)?;
    build_cxx_node(
        &root,
        &host_srcs.iter().map(|p| p.as_path()).collect::<Vec<_>>(),
        "param_host",
        &link_args,
    )?;
    build_cxx_node(
        &root,
        &client_srcs.iter().map(|p| p.as_path()).collect::<Vec<_>>(),
        "param_client",
        &link_args,
    )?;

    dora_cli::run("dataflow.yml".to_string(), false)
}

fn build_package(package: &str, features: &[&str]) -> eyre::Result<()> {
    let mut cargo = std::process::Command::new("cargo");
    cargo.arg("build").arg("-p").arg(package);
    if !features.is_empty() {
        cargo.arg("--features").arg(features.join(","));
    }
    let status = cargo.status().wrap_err("failed to run cargo build")?;
    if !status.success() {
        bail!("cargo build failed for {package}");
    }
    Ok(())
}

fn build_cxx_node(
    root: &Path,
    sources: &[&Path],
    output: &str,
    link_args: &[&str],
) -> eyre::Result<()> {
    let target = root.join("target");
    let mut clang = std::process::Command::new("clang++");
    clang
        .arg("-std=c++20")
        .arg("-g")
        .arg("-o")
        .arg(format!("build/{output}{EXE_SUFFIX}"))
        .args(sources)
        .arg("-L")
        .arg(target.join("debug"))
        .args(link_args);
    let status = clang.status().wrap_err("failed to run clang++")?;
    if !status.success() {
        bail!("compilation failed for {output}");
    }
    Ok(())
}
