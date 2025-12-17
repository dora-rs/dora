use dora_cli::{build, run};
use eyre::Context;
use std::path::Path;

use process_wrap::std::{StdChildWrapper as ChildWrapper, StdCommandWrap as CommandWrap};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join("../../../").join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build("dataflow.yml".to_string(), None, None, false, true)?;

    let dataflow_task = std::thread::spawn(|| {
        run("dataflow.yml".to_string(), false).unwrap();
    });

    let mut pub_task = run_ros_node("examples_rclcpp_minimal_publisher", "publisher_lambda")?;

    let dataflow_result = dataflow_task
        .join()
        .map_err(|_| eyre::eyre!("Failed to run dataflow"));

    pub_task.kill()?;

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
