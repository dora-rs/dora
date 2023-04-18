use crate::{
    run::spawn_dataflow,
    tcp_utils::{tcp_receive, tcp_send},
};
pub use control::ControlEvent;
use dora_core::{
    config::{CommunicationConfig, DataId, NodeId, OperatorId},
    coordinator_messages::RegisterResult,
    daemon_messages::{DaemonCoordinatorEvent, DaemonCoordinatorReply},
    message::Metadata,
    topics::{
        control_socket_addr, ControlRequest, ControlRequestReply, DataflowId,
        DORA_COORDINATOR_PORT_DEFAULT,
    },
};
use eyre::{bail, eyre, ContextCompat, WrapErr};
use futures::{stream::FuturesUnordered, Future, Stream, StreamExt};
use futures_concurrency::stream::Merge;
use run::SpawnedDataflow;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    path::{Path, PathBuf},
    time::Duration,
};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
    task::JoinHandle,
};
use tokio_stream::wrappers::{ReceiverStream, TcpListenerStream};
use uuid::Uuid;

mod control;
mod listener;
mod run;
mod tcp_utils;

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Dora coordinator")]
pub struct Args {
    #[clap(long)]
    pub port: Option<u16>,

    #[clap(long)]
    pub dora_runtime_path: Option<PathBuf>,
}

pub async fn run(args: Args) -> eyre::Result<()> {
    let ctrlc_events = set_up_ctrlc_handler()?;

    let (_, task) = start(args, ctrlc_events).await?;

    task.await?;

    Ok(())
}

pub async fn start(
    args: Args,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let port = args.port.unwrap_or(DORA_COORDINATOR_PORT_DEFAULT);
    let listener = listener::create_listener(port).await?;
    let port = listener
        .local_addr()
        .wrap_err("failed to get local addr of listener")?
        .port();
    let mut tasks = FuturesUnordered::new();
    let future = async move {
        start_inner(listener, &tasks, external_events, args.dora_runtime_path).await?;

        tracing::debug!("coordinator main loop finished, waiting on spawned tasks");
        while let Some(join_result) = tasks.next().await {
            if let Err(err) = join_result {
                tracing::error!("task panicked: {err}");
            }
        }
        tracing::debug!("all spawned tasks finished, exiting..");
        Ok(())
    };
    Ok((port, future))
}

async fn start_inner(
    listener: TcpListener,
    tasks: &FuturesUnordered<JoinHandle<()>>,
    external_events: impl Stream<Item = Event> + Unpin,
    runtime_path: Option<PathBuf>,
) -> eyre::Result<()> {
    let new_daemon_connections = TcpListenerStream::new(listener).map(|c| {
        c.map(Event::NewDaemonConnection)
            .wrap_err("failed to open connection")
            .unwrap_or_else(Event::DaemonConnectError)
    });

    let (daemon_events_tx, daemon_events) = tokio::sync::mpsc::channel(2);
    let mut daemon_events_tx = Some(daemon_events_tx);
    let daemon_events = ReceiverStream::new(daemon_events);

    let control_events = control::control_events(control_socket_addr(), tasks)
        .await
        .wrap_err("failed to create control events")?;

    let daemon_watchdog_interval =
        tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(Duration::from_secs(1)))
            .map(|_| Event::DaemonWatchdogInterval);

    // events that should be aborted on `dora destroy`
    let (abortable_events, abort_handle) = futures::stream::abortable(
        (
            control_events,
            new_daemon_connections,
            external_events,
            daemon_watchdog_interval,
        )
            .merge(),
    );

    let mut events = (abortable_events, daemon_events).merge();

    let mut running_dataflows: HashMap<Uuid, RunningDataflow> = HashMap::new();
    let mut daemon_connections: HashMap<_, TcpStream> = HashMap::new();

    while let Some(event) = events.next().await {
        if event.log() {
            tracing::trace!("Handling event {event:?}");
        }
        match event {
            Event::NewDaemonConnection(connection) => {
                connection.set_nodelay(true)?;
                let events_tx = daemon_events_tx.clone();
                if let Some(events_tx) = events_tx {
                    let task = tokio::spawn(listener::handle_connection(connection, events_tx));
                    tasks.push(task);
                } else {
                    tracing::warn!(
                        "ignoring new daemon connection because events_tx was closed already"
                    );
                }
            }
            Event::DaemonConnectError(err) => {
                tracing::warn!("{:?}", err.wrap_err("failed to connect to dora-daemon"));
            }
            Event::Daemon(event) => match event {
                DaemonEvent::Register {
                    machine_id,
                    mut connection,
                    dora_version: daemon_version,
                } => {
                    let coordinator_version = &env!("CARGO_PKG_VERSION");
                    let reply = if &daemon_version == coordinator_version {
                        RegisterResult::Ok
                    } else {
                        RegisterResult::Err(format!(
                            "version mismatch: daemon v{daemon_version} is \
                            not compatible with coordinator v{coordinator_version}"
                        ))
                    };
                    let send_result = tcp_send(&mut connection, &serde_json::to_vec(&reply)?).await;
                    match (reply, send_result) {
                        (RegisterResult::Ok, Ok(())) => {
                            let previous =
                                daemon_connections.insert(machine_id.clone(), connection);
                            if let Some(_previous) = previous {
                                tracing::info!(
                                    "closing previous connection `{machine_id}` on new register"
                                );
                            }
                        }
                        (RegisterResult::Err(err), _) => {
                            tracing::warn!("failed to register daemon connection for machine `{machine_id}`: {err}");
                        }
                        (RegisterResult::Ok, Err(err)) => {
                            tracing::warn!("failed to confirm daemon connection for machine `{machine_id}`: {err}");
                        }
                    }
                }
            },
            Event::Dataflow { uuid, event } => match event {
                DataflowEvent::ReadyOnMachine { machine_id } => {
                    match running_dataflows.entry(uuid) {
                        std::collections::hash_map::Entry::Occupied(mut entry) => {
                            let dataflow = entry.get_mut();
                            dataflow.pending_machines.remove(&machine_id);
                            if dataflow.pending_machines.is_empty() {
                                let message =
                                    serde_json::to_vec(&DaemonCoordinatorEvent::AllNodesReady {
                                        dataflow_id: uuid,
                                    })
                                    .wrap_err("failed to serialize AllNodesReady message")?;

                                // notify all machines that run parts of the dataflow
                                for machine_id in &dataflow.machines {
                                    let Some(connection) = daemon_connections.get_mut(machine_id) else {
                                        tracing::warn!("no daemon connection found for machine `{machine_id}`");
                                        continue;
                                    };
                                    tcp_send(connection, &message).await.wrap_err_with(|| {
                                        format!(
                                            "failed to send AllNodesReady({uuid}) message \
                                            to machine {machine_id}"
                                        )
                                    })?;
                                }
                            }
                        }
                        std::collections::hash_map::Entry::Vacant(_) => {
                            tracing::warn!("dataflow not running on ReadyOnMachine");
                        }
                    }
                }
                DataflowEvent::DataflowFinishedOnMachine { machine_id, result } => {
                    match running_dataflows.entry(uuid) {
                        std::collections::hash_map::Entry::Occupied(mut entry) => {
                            entry.get_mut().machines.remove(&machine_id);
                            match result {
                                Ok(()) => {
                                    tracing::info!("dataflow `{uuid}` finished successfully on machine `{machine_id}`");
                                }
                                Err(err) => {
                                    let err =
                                        err.wrap_err(format!("error occured in dataflow `{uuid}` on machine `{machine_id}`"));
                                    tracing::error!("{err:?}");
                                }
                            }
                            if entry.get_mut().machines.is_empty() {
                                entry.remove();
                                tracing::info!("dataflow `{uuid}` finished");
                            }
                        }
                        std::collections::hash_map::Entry::Vacant(_) => {
                            tracing::warn!("dataflow not running on DataflowFinishedOnMachine");
                        }
                    }
                }
                DataflowEvent::Output {
                    machine_id,
                    source_node,
                    output_id,
                    metadata,
                    data,
                    target_machines,
                } => {
                    for target_machine in target_machines {
                        match daemon_connections.get_mut(&target_machine) {
                            Some(connection) => {
                                tracing::trace!(
                                    "forwarding output `{uuid}/{source_node}/{output_id}` \
                                    from machine `{machine_id}` to machine `{target_machine}`"
                                );
                                forward_output(connection, uuid, source_node.clone(), output_id.clone(), metadata.clone(), data.clone()).await?;
                            },
                            None => tracing::warn!("received output event for unknown target machine `{target_machine}`"),
                        }
                    }
                }
                DataflowEvent::InputsClosed {
                    source_machine,
                    inputs,
                } => {
                    for (target_machine, inputs) in inputs {
                        match daemon_connections.get_mut(&target_machine) {
                            Some(connection) => {
                                tracing::trace!(
                                    "forwarding InputsClosed event for dataflow `{uuid}` \
                                    from machine `{source_machine}` to machine `{target_machine}`"
                                );
                                forward_inputs_closed(connection, uuid, inputs).await?;
                            },
                            None => tracing::warn!("received InputsClosed event for unknown target machine `{target_machine}`"),
                        }
                    }
                }
            },

            Event::Control(event) => match event {
                ControlEvent::IncomingRequest {
                    request,
                    reply_sender,
                } => {
                    let reply = match request {
                        ControlRequest::Start {
                            dataflow_path,
                            name,
                        } => {
                            let inner = async {
                                if let Some(name) = name.as_deref() {
                                    // check that name is unique
                                    if running_dataflows
                                        .values()
                                        .any(|d: &RunningDataflow| d.name.as_deref() == Some(name))
                                    {
                                        bail!("there is already a running dataflow with name `{name}`");
                                    }
                                }
                                let dataflow = start_dataflow(
                                    &dataflow_path,
                                    name,
                                    runtime_path.clone(),
                                    &mut daemon_connections,
                                )
                                .await?;
                                Ok(dataflow)
                            };
                            inner.await.map(|dataflow| {
                                let uuid = dataflow.uuid;
                                running_dataflows.insert(uuid, dataflow);
                                ControlRequestReply::DataflowStarted { uuid }
                            })
                        }
                        ControlRequest::Check { dataflow_uuid } => {
                            let status = match &running_dataflows.get(&dataflow_uuid) {
                                Some(_) => ControlRequestReply::DataflowStarted {
                                    uuid: dataflow_uuid,
                                },
                                None => ControlRequestReply::DataflowStopped {
                                    uuid: dataflow_uuid,
                                },
                            };
                            Ok(status)
                        }
                        ControlRequest::Reload {
                            dataflow_id,
                            node_id,
                            operator_id,
                        } => {
                            let reload = async {
                                reload_dataflow(
                                    &running_dataflows,
                                    dataflow_id,
                                    node_id,
                                    operator_id,
                                    &mut daemon_connections,
                                )
                                .await?;
                                Result::<_, eyre::Report>::Ok(())
                            };
                            reload
                                .await
                                .map(|()| ControlRequestReply::DataflowReloaded {
                                    uuid: dataflow_id,
                                })
                        }
                        ControlRequest::Stop { dataflow_uuid } => {
                            let stop = async {
                                stop_dataflow(
                                    &running_dataflows,
                                    dataflow_uuid,
                                    &mut daemon_connections,
                                )
                                .await?;
                                Result::<_, eyre::Report>::Ok(())
                            };
                            stop.await.map(|()| ControlRequestReply::DataflowStopped {
                                uuid: dataflow_uuid,
                            })
                        }
                        ControlRequest::StopByName { name } => {
                            let stop = async {
                                let uuids: Vec<_> = running_dataflows
                                    .iter()
                                    .filter(|(_, v)| v.name.as_deref() == Some(name.as_str()))
                                    .map(|(k, _)| k)
                                    .copied()
                                    .collect();
                                let dataflow_uuid = if uuids.is_empty() {
                                    bail!("no running dataflow with name `{name}`");
                                } else if let [uuid] = uuids.as_slice() {
                                    *uuid
                                } else {
                                    bail!("multiple dataflows found with name `{name}`");
                                };

                                stop_dataflow(
                                    &running_dataflows,
                                    dataflow_uuid,
                                    &mut daemon_connections,
                                )
                                .await?;
                                Result::<_, eyre::Report>::Ok(dataflow_uuid)
                            };
                            stop.await
                                .map(|uuid| ControlRequestReply::DataflowStopped { uuid })
                        }
                        ControlRequest::Destroy => {
                            tracing::info!("Received destroy command");

                            handle_destroy(
                                &running_dataflows,
                                &mut daemon_connections,
                                &abort_handle,
                                &mut daemon_events_tx,
                            )
                            .await
                            .map(|()| ControlRequestReply::DestroyOk)
                        }
                        ControlRequest::List => {
                            let mut dataflows: Vec<_> = running_dataflows.values().collect();
                            dataflows.sort_by_key(|d| (&d.name, d.uuid));

                            Ok(ControlRequestReply::DataflowList {
                                dataflows: dataflows
                                    .into_iter()
                                    .map(|d| DataflowId {
                                        uuid: d.uuid,
                                        name: d.name.clone(),
                                    })
                                    .collect(),
                            })
                        }
                        ControlRequest::DaemonConnected => {
                            let running = !daemon_connections.is_empty();
                            Ok(ControlRequestReply::DaemonConnected(running))
                        }
                    };
                    let _ = reply_sender.send(reply);
                }
                ControlEvent::Error(err) => tracing::error!("{err:?}"),
            },
            Event::DaemonWatchdogInterval => {
                let mut disconnected = BTreeSet::new();
                for (machine_id, connection) in &mut daemon_connections {
                    let result: eyre::Result<()> =
                        tokio::time::timeout(Duration::from_millis(100), send_watchdog_message(connection))
                            .await
                            .wrap_err("timeout")
                            .and_then(|r| r).wrap_err_with(||
                                format!("daemon at `{machine_id}` did not react as expected to watchdog message"),
                            );
                    if let Err(err) = result {
                        tracing::warn!("{err:?}");
                        disconnected.insert(machine_id.clone());
                    }
                }
                if !disconnected.is_empty() {
                    tracing::info!("Disconnecting daemons that failed watchdog: {disconnected:?}");
                    for machine_id in disconnected {
                        daemon_connections.remove(&machine_id);
                    }
                }
            }
            Event::CtrlC => {
                tracing::info!("Destroying coordinator after receiving Ctrl-C signal");
                handle_destroy(
                    &running_dataflows,
                    &mut daemon_connections,
                    &abort_handle,
                    &mut daemon_events_tx,
                )
                .await?;
            }
        }
    }

    tracing::info!("stopped");

    Ok(())
}

async fn forward_output(
    connection: &mut TcpStream,
    uuid: Uuid,
    source_node: NodeId,
    output_id: DataId,
    metadata: Metadata<'static>,
    data: Option<Vec<u8>>,
) -> eyre::Result<()> {
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::Output {
        dataflow_id: uuid,
        node_id: source_node,
        output_id,
        metadata,
        data,
    })
    .wrap_err("failed to serialize output message")?;

    tcp_send(connection, &message)
        .await
        .wrap_err("failed to send output message to daemon")?;

    Ok(())
}

async fn forward_inputs_closed(
    connection: &mut TcpStream,
    uuid: Uuid,
    inputs: BTreeSet<(NodeId, DataId)>,
) -> eyre::Result<()> {
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::InputsClosed {
        dataflow_id: uuid,
        inputs,
    })
    .wrap_err("failed to serialize InputsClosed message")?;

    tcp_send(connection, &message)
        .await
        .wrap_err("failed to send InputsClosed message to daemon")?;

    Ok(())
}

fn set_up_ctrlc_handler() -> Result<impl Stream<Item = Event>, eyre::ErrReport> {
    let (ctrlc_tx, ctrlc_rx) = mpsc::channel(1);

    let mut ctrlc_sent = false;
    ctrlc::set_handler(move || {
        if ctrlc_sent {
            tracing::warn!("received second ctrlc signal -> aborting immediately");
            std::process::abort();
        } else {
            tracing::info!("received ctrlc signal");
            if ctrlc_tx.blocking_send(Event::CtrlC).is_err() {
                tracing::error!("failed to report ctrl-c event to dora-coordinator");
            }

            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    Ok(ReceiverStream::new(ctrlc_rx))
}

async fn handle_destroy(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    daemon_connections: &mut HashMap<String, TcpStream>,
    abortable_events: &futures::stream::AbortHandle,
    daemon_events_tx: &mut Option<mpsc::Sender<Event>>,
) -> Result<(), eyre::ErrReport> {
    abortable_events.abort();
    for &uuid in running_dataflows.keys() {
        stop_dataflow(running_dataflows, uuid, daemon_connections).await?;
    }
    destroy_daemons(daemon_connections).await?;
    *daemon_events_tx = None;
    Ok(())
}

async fn send_watchdog_message(connection: &mut TcpStream) -> eyre::Result<()> {
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::Watchdog).unwrap();

    tcp_send(connection, &message)
        .await
        .wrap_err("failed to send watchdog message to daemon")?;
    let reply_raw = tcp_receive(connection)
        .await
        .wrap_err("failed to receive stop reply from daemon")?;

    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize stop reply from daemon")?
    {
        DaemonCoordinatorReply::WatchdogAck => Ok(()),
        other => bail!("unexpected reply after sending `watchdog`: {other:?}"),
    }
}

#[allow(dead_code)] // Keeping the communication layer for later use.
struct RunningDataflow {
    name: Option<String>,
    uuid: Uuid,
    communication_config: Option<CommunicationConfig>,
    /// The IDs of the machines that the dataflow is running on.
    machines: BTreeSet<String>,
    /// IDs of machines that are waiting until all nodes are started.
    pending_machines: BTreeSet<String>,
}

impl PartialEq for RunningDataflow {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.uuid == other.uuid && self.machines == other.machines
    }
}

impl Eq for RunningDataflow {}

async fn stop_dataflow(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    uuid: Uuid,
    daemon_connections: &mut HashMap<String, TcpStream>,
) -> eyre::Result<()> {
    let Some(dataflow) = running_dataflows.get(&uuid) else {
        bail!("No running dataflow found with UUID `{uuid}`")
    };
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::StopDataflow { dataflow_id: uuid })?;

    for machine_id in &dataflow.machines {
        let daemon_connection = daemon_connections
            .get_mut(machine_id)
            .wrap_err("no daemon connection")?; // TODO: take from dataflow spec
        tcp_send(daemon_connection, &message)
            .await
            .wrap_err("failed to send stop message to daemon")?;

        // wait for reply
        let reply_raw = tcp_receive(daemon_connection)
            .await
            .wrap_err("failed to receive stop reply from daemon")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize stop reply from daemon")?
        {
            DaemonCoordinatorReply::StopResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to stop dataflow")?,
            other => bail!("unexpected reply after sending stop: {other:?}"),
        }
    }
    tracing::info!("successfully stopped dataflow `{uuid}`");

    Ok(())
}

async fn reload_dataflow(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    operator_id: Option<OperatorId>,
    daemon_connections: &mut HashMap<String, TcpStream>,
) -> eyre::Result<()> {
    let Some(dataflow) = running_dataflows.get(&dataflow_id) else {
        bail!("No running dataflow found with UUID `{dataflow_id}`")
    };
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::ReloadDataflow {
        dataflow_id,
        node_id,
        operator_id,
    })?;

    for machine_id in &dataflow.machines {
        let daemon_connection = daemon_connections
            .get_mut(machine_id)
            .wrap_err("no daemon connection")?; // TODO: take from dataflow spec
        tcp_send(daemon_connection, &message)
            .await
            .wrap_err("failed to send reload message to daemon")?;

        // wait for reply
        let reply_raw = tcp_receive(daemon_connection)
            .await
            .wrap_err("failed to receive reload reply from daemon")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize reload reply from daemon")?
        {
            DaemonCoordinatorReply::ReloadResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to reload dataflow")?,
            other => bail!("unexpected reply after sending reload: {other:?}"),
        }
    }
    tracing::info!("successfully reloaded dataflow `{dataflow_id}`");

    Ok(())
}

async fn start_dataflow(
    path: &Path,
    name: Option<String>,
    runtime_path: Option<PathBuf>,
    daemon_connections: &mut HashMap<String, TcpStream>,
) -> eyre::Result<RunningDataflow> {
    let SpawnedDataflow {
        uuid,
        communication_config,
        machines,
    } = spawn_dataflow(path, runtime_path, daemon_connections).await?;
    Ok(RunningDataflow {
        uuid,
        name,
        communication_config,
        pending_machines: if machines.len() > 1 {
            machines.clone()
        } else {
            BTreeSet::new()
        },
        machines,
    })
}

async fn destroy_daemons(daemon_connections: &mut HashMap<String, TcpStream>) -> eyre::Result<()> {
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::Destroy)?;

    for (machine_id, mut daemon_connection) in daemon_connections.drain() {
        tcp_send(&mut daemon_connection, &message)
            .await
            .wrap_err("failed to send destroy message to daemon")?;

        // wait for reply
        let reply_raw = tcp_receive(&mut daemon_connection)
            .await
            .wrap_err("failed to receive destroy reply from daemon")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize destroy reply from daemon")?
        {
            DaemonCoordinatorReply::DestroyResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to destroy dataflow")?,
            other => bail!("unexpected reply after sending `destroy`: {other:?}"),
        }

        tracing::info!("successfully destroyed daemon `{machine_id}`");
    }

    Ok(())
}

#[derive(Debug)]
pub enum Event {
    NewDaemonConnection(TcpStream),
    DaemonConnectError(eyre::Report),
    Dataflow { uuid: Uuid, event: DataflowEvent },
    Control(ControlEvent),
    Daemon(DaemonEvent),
    DaemonWatchdogInterval,
    CtrlC,
}

impl Event {
    /// Whether this event should be logged.
    #[allow(clippy::match_like_matches_macro)]
    pub fn log(&self) -> bool {
        match self {
            Event::DaemonWatchdogInterval => false,
            _ => true,
        }
    }
}

#[derive(Debug)]
pub enum DataflowEvent {
    DataflowFinishedOnMachine {
        machine_id: String,
        result: eyre::Result<()>,
    },
    Output {
        machine_id: String,
        source_node: NodeId,
        output_id: DataId,

        metadata: Metadata<'static>,
        data: Option<Vec<u8>>,

        target_machines: BTreeSet<String>,
    },
    InputsClosed {
        source_machine: String,
        inputs: BTreeMap<String, BTreeSet<(NodeId, DataId)>>,
    },
    ReadyOnMachine {
        machine_id: String,
    },
}

#[derive(Debug)]
pub enum DaemonEvent {
    Register {
        machine_id: String,
        connection: TcpStream,
        dora_version: String,
    },
}
