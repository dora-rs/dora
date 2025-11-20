use dora_cli::{build, run as dora_run};
use eyre::WrapErr;
use std::path::Path;

use process_wrap::std::{ChildWrapper, CommandWrap, ProcessGroup};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build("dataflow.yml".to_string(), None, None, true, true)?;

    let dataflow_task = std::thread::spawn(|| {
        dora_run("dataflow.yml".to_string(), true).unwrap();
    });

    // let mut add_service_task = run_ros_node("examples_rclcpp_minimal_service", "service_main")?;
    let mut turtle_task = run_ros_node("turtlesim", "turtlesim_node")?;

    while !dataflow_task.is_finished() {}

    // add_service_task.kill()?;
    turtle_task.kill()?;
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
