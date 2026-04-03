use eyre::{Context, bail};
use std::{env::consts::EXE_SUFFIX, path::Path};

use process_wrap::std::{StdChildWrapper as ChildWrapper, StdCommandWrap as CommandWrap};

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
            &dunce::canonicalize(Path::new("node-rust-api").join("main.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("dora-node-api.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/sensor_msgs.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/geometry_msgs.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/example_interfaces.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/turtlesim.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/impl.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/action_msgs.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/builtin_interfaces.cc"))?,
            &dunce::canonicalize(node_cxxbridge.join("ros2-bridge/msg/unique_identifier_msgs.cc"))?,
        ],
        "node_rust_api",
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

    let mut add_service_task = run_ros_node("examples_rclcpp_minimal_service", "service_main")?;
    let mut turtle_task = run_ros_node("turtlesim", "turtlesim_node")?;

    let dataflow_result = dataflow_task
        .join()
        .map_err(|_| eyre::eyre!("Failed to run dataflow"));

    add_service_task.kill()?;
    turtle_task.kill()?;

    dataflow_result
}

fn run_ros_node(package: &str, node: &str) -> eyre::Result<Box<dyn ChildWrapper>> {
    let mut command = CommandWrap::with_new("ros2", |cmd| {
        cmd.arg("run");
        cmd.arg(package).arg(node);
    });
    #[cfg(unix)]
    command.wrap(process_wrap::std::ProcessGroup::leader());
    #[cfg(windows)]
    command.wrap(process_wrap::std::JobObject);
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
    clang.arg("-std=c++20");
    #[cfg(target_os = "linux")]
    {
        clang.arg("-l").arg("m");
        clang.arg("-l").arg("rt");
        clang.arg("-l").arg("dl");
        clang.arg("-l").arg("z");
        clang.arg("-pthread");
    }
    #[cfg(target_os = "windows")]
    {
        clang.arg("-ladvapi32");
        clang.arg("-luserenv");
        clang.arg("-lkernel32");
        clang.arg("-lws2_32");
        clang.arg("-lbcrypt");
        clang.arg("-lncrypt");
        clang.arg("-lschannel");
        clang.arg("-lntdll");
        clang.arg("-liphlpapi");

        clang.arg("-lcfgmgr32");
        clang.arg("-lcredui");
        clang.arg("-lcrypt32");
        clang.arg("-lcryptnet");
        clang.arg("-lfwpuclnt");
        clang.arg("-lgdi32");
        clang.arg("-lmsimg32");
        clang.arg("-lmswsock");
        clang.arg("-lole32");
        clang.arg("-lopengl32");
        clang.arg("-lsecur32");
        clang.arg("-lshell32");
        clang.arg("-lsynchronization");
        clang.arg("-luser32");
        clang.arg("-lwinspool");

        clang.arg("-Wl,-nodefaultlib:libcmt");
        clang.arg("-D_DLL");
        clang.arg("-lmsvcrt");
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
        .arg(Path::new("../build").join(format!("{out_name}{EXE_SUFFIX}")));
    if let Some(parent) = paths[0].parent() {
        clang.current_dir(parent);
    }

    if !clang.status()?.success() {
        bail!("failed to compile c++ node");
    };
    Ok(())
}
