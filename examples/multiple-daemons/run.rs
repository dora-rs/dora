use dora_coordinator::{ControlEvent, Event};
use dora_core::{
    descriptor::Descriptor,
    topics::{ControlRequest, ControlRequestReply, DataflowId},
};
use eyre::{bail, Context};
use futures::stream;
use std::{
    collections::BTreeSet,
    net::{Ipv4Addr, SocketAddr},
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
use tracing::metadata::LevelFilter;
use tracing_subscriber::Layer;
use uuid::Uuid;

#[derive(Debug, Clone, clap::Parser)]
pub struct Args {
    #[clap(long)]
    pub run_dora_runtime: bool,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let Args { run_dora_runtime } = clap::Parser::parse();
    if run_dora_runtime {
        return tokio::task::block_in_place(dora_daemon::run_dora_runtime);
    }

    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let dataflow = Path::new("dataflow.yml");
    build_dataflow(dataflow).await?;

    let (coordinator_events_tx, coordinator_events_rx) = mpsc::channel(1);
    let (coordinator_port, coordinator) = dora_coordinator::start(
        dora_coordinator::Args { port: Some(0) },
        ReceiverStream::new(coordinator_events_rx),
    )
    .await?;
    let coordinator_addr = SocketAddr::new(Ipv4Addr::LOCALHOST.into(), coordinator_port);
    let daemon_a = dora_daemon::Daemon::run(coordinator_addr, "A".into(), stream::empty());
    let daemon_b = dora_daemon::Daemon::run(coordinator_addr, "B".into(), stream::empty());

    tracing::info!("Spawning coordinator and daemons");
    let mut tasks = JoinSet::new();
    tasks.spawn(coordinator);
    tasks.spawn(daemon_a);
    tasks.spawn(daemon_b);

    // wait until both daemons are connected
    tracing::info!("waiting until daemons are connected to coordinator");
    let mut retries = 0;
    loop {
        let connected_machines = connected_machines(&coordinator_events_tx).await?;
        if connected_machines.contains("A") && connected_machines.contains("B") {
            break;
        } else if retries > 20 {
            bail!("daemon not connected after {retries} retries");
        } else {
            std::thread::sleep(Duration::from_millis(100));
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
    let dataflow_descriptor = Descriptor::read(dataflow)
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

    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::Start {
                dataflow: dataflow_descriptor,
                local_working_dir: working_dir,
                name: None,
            },
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    let uuid = match result {
        ControlRequestReply::DataflowStarted { uuid } => uuid,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    };
    Ok(uuid)
}

async fn connected_machines(
    coordinator_events_tx: &Sender<Event>,
) -> eyre::Result<BTreeSet<String>> {
    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::ConnectedMachines,
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    let machines = match result {
        ControlRequestReply::ConnectedMachines(machines) => machines,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    };
    Ok(machines)
}

async fn running_dataflows(coordinator_events_tx: &Sender<Event>) -> eyre::Result<Vec<DataflowId>> {
    let (reply_sender, reply) = oneshot::channel();
    coordinator_events_tx
        .send(Event::Control(ControlEvent::IncomingRequest {
            request: ControlRequest::List,
            reply_sender,
        }))
        .await?;
    let result = reply.await??;
    let dataflows = match result {
        ControlRequestReply::DataflowList { dataflows } => dataflows,
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

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(LevelFilter::DEBUG);
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
