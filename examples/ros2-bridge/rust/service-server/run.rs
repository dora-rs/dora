use dora_cli::{build, run};
use eyre::Context;
use std::{path::Path, sync::mpsc};
use tokio;

use process_wrap::tokio::{TokioChildWrapper as ChildWrapper, TokioCommandWrap as CommandWrap};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join("../../../").join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build("dataflow.yml".to_string(), None, None, false, true)?;

    let (finish_tx, finish_rx) = mpsc::channel();
    let dataflow_task = std::thread::spawn(move || {
        let dataflow_result = run("dataflow.yml".to_string(), false);
        finish_tx.send(())?;
        dataflow_result
    });

    let rt = tokio::runtime::Runtime::new()?;
    let (finish_async_tx, mut finish_async_rx) = tokio::sync::mpsc::channel(1);
    // the task for converting std::sync::mpsc to tokio::sync::mpsc
    rt.spawn(async move {
        finish_rx.recv().unwrap();
        finish_async_tx.send(()).await.unwrap();
    });

    // Each client_main only sends one request, and the server node will terminate after 3 times requirement
    rt.block_on(async move {
        let mut client_task = run_ros_node("examples_rclcpp_minimal_client", "client_main")?;
        loop {
            tokio::select! {
                _ = finish_async_rx.recv() => {
                    break;
                },
                _ret = Box::into_pin(client_task.wait()) => {
                    client_task = run_ros_node("examples_rclcpp_minimal_client", "client_main")?;
                }
            }
        }
        Box::into_pin(client_task.kill()).await?;
        Ok::<(), eyre::Report>(())
    })?;

    dataflow_task.join().unwrap()
}

fn run_ros_node(package: &str, node: &str) -> eyre::Result<Box<dyn ChildWrapper>> {
    let mut command = CommandWrap::with_new("ros2", |cmd| {
        cmd.arg("run");
        cmd.arg(package).arg(node);
    });
    #[cfg(unix)]
    command.wrap(process_wrap::tokio::ProcessGroup::leader());
    #[cfg(windows)]
    command.wrap(process_wrap::tokio::JobObject);
    command
        .spawn()
        .map_err(|e| eyre::eyre!("failed to spawn ros node: {}", e))
}
