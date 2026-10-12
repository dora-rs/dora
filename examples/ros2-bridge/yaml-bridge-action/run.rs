use std::{path::Path, time::Duration};

use dora_cli::{Executable, RunCommand};
use eyre::Context;
use process_wrap::std::{ChildWrapper, CommandWrap, ProcessGroup};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join("../../../").join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let mut action_task = run_ros_node("examples_rclcpp_action_server", "fibonacci_action_server")?;
    std::thread::sleep(Duration::from_secs(3));

    let mut run = RunCommand::new("dataflow.yml".to_string());
    run.stop_after = Some(Duration::from_secs(20));
    let dataflow_result = run.execute();

    let kill_result = action_task
        .kill()
        .wrap_err("failed to stop ROS2 action server");

    dataflow_result?;
    kill_result?;
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
