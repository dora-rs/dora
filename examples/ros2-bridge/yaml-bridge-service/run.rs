use std::{path::Path, time::Duration};

use dora_cli::{Executable, RunCommand};
use eyre::Context;
use process_wrap::std::{ChildWrapper, CommandWrap};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join("../../../").join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let mut service_task = run_ros_node("examples_rclcpp_minimal_service", "service_main")?;
    std::thread::sleep(Duration::from_secs(2));

    let mut run = RunCommand::new("dataflow-client.yml".to_string());
    run.stop_after = Some(Duration::from_secs(12));
    let dataflow_result = run.execute();

    let kill_result = service_task.kill().wrap_err("failed to stop ROS2 service");

    dataflow_result?;
    kill_result?;
    Ok(())
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
