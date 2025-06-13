use dora_cli::session::DataflowSession;
use dora_coordinator::{ControlEvent, Event};
use dora_core::{
    descriptor::{read_as_descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, DORA_COORDINATOR_PORT_DEFAULT},
};
use dora_message::{
    cli_to_coordinator::ControlRequest,
    common::DaemonId,
    coordinator_to_cli::{ControlRequestReply, DataflowIdAndName},
};
use dora_tracing::TracingBuilder;
use eyre::{bail, Context};

use std::{
    collections::BTreeSet,
    net::{IpAddr, Ipv4Addr, SocketAddr},
    path::Path,
    time::Duration,
};
use tokio::{
    sync::{
        mpsc::{self, Sender},
        oneshot,
    },
    task::JoinSet,
};
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    TracingBuilder::new("multiple-daemon-runner")
        .with_stdout("debug")
        .build()?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    let (coordinator_events_tx, coordinator_events_rx) = mpsc::channel(1);
    let coordinator_bind = SocketAddr::new(
        IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0)),
        DORA_COORDINATOR_PORT_DEFAULT,
    );
    let coordinator_control_bind = SocketAddr::new(
        IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0)),
        DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
    );
    let (coordinator_port, coordinator) = dora_coordinator::start(
        coordinator_bind,
        coordinator_control_bind,
        ReceiverStream::new(coordinator_events_rx),
    )
    .await?;

    tracing::info!("coordinator running on {coordinator_port}");

    let coordinator_addr = Ipv4Addr::LOCALHOST;
    let daemon_a = run_daemon(coordinator_addr.to_string(), "A");
    let daemon_b = run_daemon(coordinator_addr.to_string(), "B");

    tracing::info!("Spawning coordinator and daemons");
    let mut tasks = JoinSet::new();
    tasks.spawn(coordinator);
    tasks.spawn(daemon_b);
    tasks.spawn(daemon_a);

    tracing::info!("waiting until daemons are connected to coordinator");
    let mut retries = 0;
    loop {
        let connected_machines = connected_machines(&coordinator_events_tx).await?;
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
    let uuid = start_dataflow(dataflow, &coordinator_events_tx).await?;
    tracing::info!("started dataflow under ID `{uuid}`");

    let running = running_dataflows(&coordinator_events_tx).await?;
    if !running.iter().map(|d| d.uuid).any(|id| id == uuid) {
        bail!("dataflow `{uuid}` is not running");
    }

    tracing::info!("waiting for dataflow `{uuid}` to finish");
    let mut retries = 0;
    loop {
        let running = running_dataflows(&coordinator_events_tx).await?;
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
    destroy(&coordinator_events_tx).await?;

    tracing::info!("joining tasks");
    while let Some(res) = tasks.join_next().await {
        res.unwrap()?;
    }

    tracing::info!("done");
    Ok(())
}

async fn start_dataflow(
    dataflow: &Path,
    coordinator_events_tx: &Sender<Event>,
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

    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::Start {
                build_id: dataflow_session.build_id,
                session_id: dataflow_session.session_id,
                dataflow: dataflow_descriptor,
                local_working_dir: Some(working_dir),
                name: None,
                uv: false,
            },
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    let uuid = match result {
        ControlRequestReply::DataflowStartTriggered { uuid } => uuid,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    };

    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::WaitForSpawn { dataflow_id: uuid },
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    let uuid = match result {
        ControlRequestReply::DataflowSpawned { uuid } => uuid,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    };
    Ok(uuid)
}

async fn connected_machines(
    coordinator_events_tx: &Sender<Event>,
) -> eyre::Result<BTreeSet<DaemonId>> {
    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::ConnectedMachines,
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    let machines = match result {
        ControlRequestReply::ConnectedDaemons(machines) => machines,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    };
    Ok(machines)
}

async fn running_dataflows(
    coordinator_events_tx: &Sender<Event>,
) -> eyre::Result<Vec<DataflowIdAndName>> {
    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::List,
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    let dataflows = match result {
        ControlRequestReply::DataflowList(list) => list.get_active(),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    };
    Ok(dataflows)
}

async fn destroy(coordinator_events_tx: &Sender<Event>) -> eyre::Result<()> {
    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::Destroy,
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    match result {
        ControlRequestReply::DestroyOk => Ok(()),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
}

async fn build_dataflow(dataflow: &Path) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
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
