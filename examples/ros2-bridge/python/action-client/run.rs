use dora_cli::{BuildConfig, build, run as dora_run};
use eyre::WrapErr;
use std::path::Path;

use process_wrap::std::{StdChildWrapper as ChildWrapper, StdCommandWrap as CommandWrap};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build(BuildConfig {
        dataflow: "dataflow.yml".to_string(),
        uv: true,
        force_local: true,
        ..Default::default()
    })?;

    let dataflow_task = std::thread::spawn(|| {
        dora_run("dataflow.yml".to_string(), true).unwrap();
    });

    // Real ROS2 fibonacci action server (same node the Rust action-client uses).
    let mut action_server_task = run_ros_node(
        "examples_rclcpp_minimal_action_server",
        "action_server_member_functions",
    )?;

    let dataflow_result = dataflow_task
        .join()
        .map_err(|_| eyre::eyre!("Failed to run dataflow"));

    action_server_task.kill()?;

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
