use dora_tracing::set_up_tracing;
use eyre::{Context, bail};
use process_wrap::tokio::{ChildWrapper, CommandWrap, ProcessGroup};
use std::path::Path;
use tokio::process::Child;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("rust-ros2-pub-dataflow-runner")
        .wrap_err("failed to set up tracing subscriber")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    let mut ros_node =
        run_ros_node("examples_rclcpp_minimal_publisher", "publisher_lambda").await?;
    let mut dataflow = run_dataflow(dataflow).await?;

    tokio::select! {
        _ = ros_node.wait() => (),
        _ = dataflow.wait() => (),
        _ = tokio::signal::ctrl_c() => (),
    }
    println!("start to kill the process");
    Box::into_pin(ros_node.kill())
        .await
        .map_err(|e| eyre::eyre!("failed to kill ros node: {}", e))?;

    dataflow.kill().await?;

    Ok(())
}

async fn run_ros_node(package: &str, node: &str) -> eyre::Result<Box<dyn ChildWrapper>> {
    let mut command = CommandWrap::with_new("ros2", |cmd| {
        cmd.arg("run");
        cmd.arg(package).arg(node);
    });
    command.wrap(ProcessGroup::leader());
    command
        .spawn()
        .map_err(|e| eyre::eyre!("failed to spawn ros node: {}", e))
}

async fn build_dataflow(dataflow: &Path) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--release");
    cmd.arg("--").arg("build").arg(dataflow);
    if !cmd.status().await?.success() {
        bail!("failed to build dataflow");
    };
    Ok(())
}

async fn run_dataflow(dataflow: &Path) -> eyre::Result<Child> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--release");
    cmd.arg("--")
        .arg("daemon")
        .arg("--run-dataflow")
        .arg(dataflow);
    cmd.spawn()
        .map_err(|e| eyre::eyre!("Failed to spawn dataflow: {}", e))
}
