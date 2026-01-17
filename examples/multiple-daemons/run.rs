use dora_cli::session::DataflowSession;
use dora_coordinator::Event;
use dora_core::{
    descriptor::{DescriptorExt, read_as_descriptor},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, DORA_COORDINATOR_PORT_DEFAULT},
};
use dora_message::{
    cli_to_coordinator::{CliToCoordinatorAsyncClient, StartReq},
    coordinator_to_cli::{DataflowIdAndName, DataflowList},
};
use dora_tracing::TracingBuilder;
use eyre::{Context, bail};

use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    path::Path,
    time::Duration,
};
use tokio::task::JoinSet;
use uuid::Uuid;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    TracingBuilder::new("multiple-daemon-runner")
        .with_stdout("debug", false)
        .build()?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    let coordinator_bind = SocketAddr::new(
        IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0)),
        DORA_COORDINATOR_PORT_DEFAULT,
    );
    let coordinator_control_bind = SocketAddr::new(
        IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0)),
        DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
    );
    let (coordinator_port, coordinator) =
        dora_coordinator::start(coordinator_bind, coordinator_control_bind).await?;

    tracing::info!("coordinator running on {coordinator_port}");

    let coordinator_addr = Ipv4Addr::LOCALHOST;
    let daemon_a = run_daemon(coordinator_addr.to_string(), "A");
    let daemon_b = run_daemon(coordinator_addr.to_string(), "B");

    tracing::info!("Spawning coordinator and daemons");
    let mut tasks = JoinSet::new();
    tasks.spawn(coordinator);
    tasks.spawn(daemon_b);
    tasks.spawn(daemon_a);

    tokio::time::sleep(Duration::from_secs(1)).await;
    let mut client = CliToCoordinatorAsyncClient::new_tcp(SocketAddr::new(
        coordinator_addr.into(),
        DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
    ))
    .await?;

    tracing::info!("waiting until daemons are connected to coordinator");
    let mut retries = 0;
    loop {
        let connected_machines = client.connected_machines().await?;
        if connected_machines
            .iter()
            .any(|id| id.matches_machine_id("A"))
            && connected_machines
                .iter()
                .any(|id| id.matches_machine_id("B"))
        {
            break;
        } else if retries > 20 {
            bail!("daemon not connected after {retries} retries");
        } else {
            std::thread::sleep(Duration::from_millis(500));
            retries += 1
        }
    }

    tracing::info!("starting dataflow");
    let uuid = start_dataflow(dataflow, &mut client).await?;
    tracing::info!("started dataflow under ID `{uuid}`");

    let running = running_dataflows(&mut client).await?;
    if !running.iter().map(|d| d.uuid).any(|id| id == uuid) {
        bail!("dataflow `{uuid}` is not running");
    }

    tracing::info!("waiting for dataflow `{uuid}` to finish");
    let mut retries = 0;
    loop {
        let running = running_dataflows(&mut client).await?;
        if running.is_empty() {
            break;
        } else if retries > 100 {
            bail!("dataflow not finished after {retries} retries");
        } else {
            tracing::debug!("not done yet");
            std::thread::sleep(Duration::from_millis(500));
            retries += 1
        }
    }
    tracing::info!("dataflow `{uuid}` finished, destroying coordinator");
    client.destroy().await?;

    tracing::info!("joining tasks");
    while let Some(res) = tasks.join_next().await {
        res.unwrap()?;
    }

    tracing::info!("done");
    Ok(())
}

async fn start_dataflow(
    dataflow: &Path,
    client: &mut CliToCoordinatorAsyncClient,
) -> eyre::Result<Uuid> {
    let dataflow_descriptor = read_as_descriptor(dataflow)
        .await
        .wrap_err("failed to read yaml dataflow")?;
    let working_dir = dataflow
        .canonicalize()
        .context("failed to canonicalize dataflow path")?
        .parent()
        .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
        .to_owned();
    dataflow_descriptor
        .check(&working_dir)
        .wrap_err("could not validate yaml")?;

    let dataflow_session =
        DataflowSession::read_session(dataflow).context("failed to read DataflowSession")?;

    let uuid = client
        .start(StartReq {
            build_id: dataflow_session.build_id,
            session_id: dataflow_session.session_id,
            dataflow: dataflow_descriptor,
            name: None,
            local_working_dir: Some(working_dir),
            uv: false,
            write_events_to: None,
        })
        .await?;

    client.wait_for_spawn(uuid).await
}

async fn running_dataflows(
    client: &mut CliToCoordinatorAsyncClient,
) -> eyre::Result<Vec<DataflowIdAndName>> {
    Ok(DataflowList(client.list().await?).get_active())
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

async fn run_daemon(coordinator: String, machine_id: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--release");
    cmd.arg("--")
        .arg("daemon")
        .arg("--machine-id")
        .arg(machine_id)
        .arg("--coordinator-addr")
        .arg(coordinator)
        .arg("--local-listen-port")
        .arg("9843"); // random port
    if !cmd.status().await?.success() {
        bail!("failed to run dataflow");
    };
    Ok(())
}
