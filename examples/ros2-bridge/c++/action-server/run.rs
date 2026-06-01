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
    let example_dir = root.join(file!()).parent().unwrap().to_owned();
    std::env::set_current_dir(&example_dir).wrap_err("failed to set working dir")?;

    std::fs::create_dir_all("build")?;

    // The dora node is the Fibonacci action server. Its peer is the dora C++
    // action client (the existing c++/action-client example), run as a second
    // dataflow node -- a real rmw client cannot discover a ros2-client-hosted
    // server (ros2-client#4), so both nodes use ros2-client and discover each
    // other over DDS. Both C++ binaries are built here.
    build_package("dora-node-api-cxx", &["ros2-bridge"])?;
    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("install");

    // C++ sources shared by both nodes (action needs the action_msgs / Time /
    // UUID message glue in addition to the example_interfaces + bridge impl).
    let shared = |main: &Path| -> eyre::Result<Vec<std::path::PathBuf>> {
        Ok(vec![
            dunce::canonicalize(main)?,
            dunce::canonicalize(node_cxxbridge.join("dora-node-api.cc"))?,
            dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/example_interfaces.cc"))?,
            dunce::canonicalize(node_cxxbridge.join("ros2-bridge/impl.cc"))?,
            dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/action_msgs.cc"))?,
            dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/builtin_interfaces.cc"))?,
            dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/unique_identifier_msgs.cc"))?,
        ])
    };
    let link_args = [
        "-I",
        node_cxxbridge.as_os_str().to_str().unwrap(),
        "-l",
        "dora_node_api_cxx",
    ];

    let server_main = dunce::canonicalize(Path::new("./").join("main.cc"))?;
    let client_main = dunce::canonicalize(example_dir.join("../action-client/main.cc"))?;
    let server_srcs = shared(&server_main)?;
    let client_srcs = shared(&client_main)?;
    build_cxx_node(
        &root,
        &server_srcs.iter().map(|p| p.as_path()).collect::<Vec<_>>(),
        "action_server",
        &link_args,
    )?;
    build_cxx_node(
        &root,
        &client_srcs.iter().map(|p| p.as_path()).collect::<Vec<_>>(),
        "action_client",
        &link_args,
    )?;

    // Runs both nodes (the prebuilt C++ server + the prebuilt C++ client).
    dora_cli::run("dataflow.yml".to_string(), false)?;

    Ok(())
}

fn build_package(package: &str, features: &[&str]) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = std::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(package);
    if !features.is_empty() {
        cmd.arg("--features").arg(features.join(","));
    }
    if !cmd.status()?.success() {
        bail!("failed to compile {package}");
    };
    Ok(())
}

fn build_cxx_node(root: &Path, paths: &[&Path], out_name: &str, args: &[&str]) -> eyre::Result<()> {
    let mut clang = std::process::Command::new("clang++");
    clang.args(paths);
    clang.arg("-std=c++17");
    #[cfg(target_os = "linux")]
    {
        clang.arg("-l").arg("m");
        clang.arg("-l").arg("rt");
        clang.arg("-l").arg("dl");
        clang.arg("-l").arg("z");
        clang.arg("-pthread");
    }
    #[cfg(target_os = "macos")]
    {
        clang.arg("-framework").arg("CoreServices");
        clang.arg("-framework").arg("Security");
        clang.arg("-l").arg("System");
        clang.arg("-l").arg("resolv");
        clang.arg("-l").arg("pthread");
        clang.arg("-l").arg("c");
        clang.arg("-l").arg("m");
    }
    clang.args(args);
    clang.arg("-L").arg(root.join("target").join("debug"));
    clang
        .arg("--output")
        .arg(Path::new("./build").join(format!("{out_name}{EXE_SUFFIX}")));
    if !clang.status()?.success() {
        bail!("failed to compile c++ node {out_name}");
    };
    Ok(())
}
