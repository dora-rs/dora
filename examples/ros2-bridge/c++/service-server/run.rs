use eyre::{Context, bail};
use std::{env::consts::EXE_SUFFIX, path::Path};

use process_wrap::std::{ChildWrapper, CommandWrap, ProcessGroup};

fn main() -> eyre::Result<()> {
    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
        return Ok(());
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../");
    let target = root.join("target");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    std::fs::create_dir_all("build")?;

    build_package("dora-node-api-cxx", &["ros2-bridge"])?;
    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("install");

    build_cxx_node(
        &root,
        &[
            &dunce::canonicalize(Path::new("./").join("main.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("dora-node-api.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/example_interfaces.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/impl.cc"))?,
        ],
        "service_server",
        &[
            "-I",
            node_cxxbridge.as_os_str().to_str().unwrap(),
            "-l",
            "dora_node_api_cxx",
        ],
    )?;

    let dataflow_task = std::thread::spawn(|| {
        dora_cli::run("dataflow.yml".to_string(), false).unwrap();
    });

    // The dora node hosts the `add_two_ints` service server; drive it with the
    // rclcpp minimal client (same peer the Rust service-server example uses).
    let mut ros_task = run_ros_node("examples_rclcpp_minimal_client", "client_main")?;

    let dataflow_task_res = dataflow_task.join();

    ros_task.kill()?;

    dataflow_task_res.map_err(|e| eyre::eyre!("the dataflow thread failed: {:?}", e))?;
    Ok(())
}

fn run_ros_node(package: &str, node: &str) -> eyre::Result<Box<dyn ChildWrapper>> {
    let mut command = CommandWrap::with_new("ros2", |cmd| {
        cmd.arg("run");
        cmd.arg(package).arg(node);
    });
    command.wrap(ProcessGroup::leader());
    command
        .spawn()
        .map_err(|e| eyre::eyre!("failed to spawn ros node: {}", e))
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
    if let Some(parent) = paths[0].parent() {
        clang.current_dir(parent);
    }

    if !clang.status()?.success() {
        bail!("failed to compile c++ node");
    };
    Ok(())
}
