use dora_cli::{BuildConfig, build, run};
use eyre::Context;
use std::{path::Path, process::Command, time::Duration};

use process_wrap::std::{ChildWrapper, CommandWrap};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join("../../../").join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build(BuildConfig {
        dataflow: "dataflow.yml".to_string(),
        force_local: true,
        ..Default::default()
    })?;

    let mut add_service_task = run_ros_node("examples_rclcpp_minimal_service", "service_main")?;
    let mut turtle_task = run_ros_node("turtlesim", "turtlesim_node")?;

    let service_wait_result = wait_for_ros_service("/add_two_ints");

    let dataflow_result = if service_wait_result.is_ok() {
        let dataflow_task = std::thread::spawn(|| {
            run("dataflow.yml".to_string(), false).unwrap();
        });
        dataflow_task
            .join()
            .map_err(|_| eyre::eyre!("Failed to run dataflow"))
    } else {
        Ok(())
    };

    let service_kill_result = add_service_task
        .kill()
        .wrap_err("failed to stop ROS2 add_two_ints service");
    let turtle_kill_result = turtle_task.kill().wrap_err("failed to stop turtlesim node");

    service_wait_result?;
    dataflow_result?;
    service_kill_result?;
    turtle_kill_result?;

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

fn wait_for_ros_service(service: &str) -> eyre::Result<()> {
    for _ in 0..30 {
        let output = Command::new("ros2")
            .arg("service")
            .arg("list")
            .output()
            .wrap_err("failed to list ROS2 services")?;
        if !output.status.success() {
            return Err(eyre::eyre!(
                "ros2 service list failed with status {}",
                output.status
            ));
        }
        if String::from_utf8_lossy(&output.stdout)
            .lines()
            .any(|line| line == service)
        {
            return Ok(());
        }
        std::thread::sleep(Duration::from_secs(1));
    }

    Err(eyre::eyre!("ROS2 service {service} not available"))
}
