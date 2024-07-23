use dora_tracing::set_up_tracing;
use eyre::{bail, Context};
use std::path::Path;
use tokio::{fs, io::AsyncWriteExt};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("multi-node-benchmark-runner")
        .wrap_err("failed to set up tracing subscriber")?;

    let args: Vec<String> = std::env::args().collect();
    let nodes_num = if args.len() > 1 {
        args[1]
            .parse::<usize>()
            .wrap_err("failed to parse num_nodes")?
    } else {
        bail!("cargo run --release --example multi-node-benchmark -- <num_nodes>")
    };

    build_benchmakr().await?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("__dataflow.yml");
    create_dataflow_yaml(dataflow, nodes_num).await?;

    run_dataflow(dataflow).await?;
    Ok(())
}

async fn create_dataflow_yaml(dataflow: &Path, num: usize) -> eyre::Result<()> {
    const NODE_TEMPLATE: &str = include_str!("node.template");
    let _ = fs::File::create(dataflow).await?;
    let mut dataflow_file = fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open(dataflow)
        .await?;

    dataflow_file.write_all(b"nodes:\n").await?;
    if num == 0 {
        let node_description = NODE_TEMPLATE.replace("#", "0");
        let node_description = node_description.replace("__INPUTS__", "");
        dataflow_file.write_all(node_description.as_bytes()).await?;
        dataflow_file.write_all(b"\n").await?;
    } else {
        for i in 0..num {
            let node_description = NODE_TEMPLATE.replace("#", &i.to_string());
            let node_description = if i == 0 {
                node_description.replace("__INPUTS__", "")
            } else {
                node_description.replace("__INPUTS__", 
                &format!("inputs:\n        latency: rust-node-{}/latency{}\n        throughput: rust-node-{}/throughput{}", i - 1, i - 1, i - 1, i - 1)
                )
            };
            dataflow_file.write_all(node_description.as_bytes()).await?;
            dataflow_file.write_all(b"\n").await?;
        }
    }

    const SINK_TEMPLATE: &str = include_str!("sink.template");
    if num == 0 {
        let sink_description = SINK_TEMPLATE.replace("#", "0");
        dataflow_file.write_all(sink_description.as_bytes()).await?;
    } else {
        let sink_description = SINK_TEMPLATE.replace("#", &(num - 1).to_string());
        dataflow_file.write_all(sink_description.as_bytes()).await?;
    }
    dataflow_file.write_all(b"\n").await?;

    Ok(())
}

async fn build_benchmakr() -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut build = tokio::process::Command::new(&cargo);
    build
        .arg("build")
        .arg("--release")
        .arg("-p")
        .arg("benchmark-example-node");
    if !build.status().await?.success() {
        bail!("failed to build benchmark-example-node");
    };
    let mut build = tokio::process::Command::new(&cargo);
    build
        .arg("build")
        .arg("--release")
        .arg("-p")
        .arg("benchmark-example-sink");
    if !build.status().await?.success() {
        bail!("failed to build benchmark-example-sinik");
    };
    Ok(())
}

async fn run_dataflow(dataflow: &Path) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--release");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--")
        .arg("daemon")
        .arg("--run-dataflow")
        .arg(dataflow);
    if !cmd.status().await?.success() {
        bail!("failed to run dataflow");
    };
    Ok(())
}
