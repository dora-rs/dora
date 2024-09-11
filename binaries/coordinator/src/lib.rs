use crate::{
    run::spawn_dataflow,
    tcp_utils::{tcp_receive, tcp_send},
};
pub use control::ControlEvent;
use dora_core::{
    config::{NodeId, OperatorId},
    descriptor::{Descriptor, ResolvedNode},
    uhlc::{self, HLC},
};
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{
        ControlRequestReply, DataflowIdAndName, DataflowList, DataflowListEntry, DataflowResult,
        DataflowStatus, LogMessage,
    },
    coordinator_to_daemon::{DaemonCoordinatorEvent, RegisterResult, Timestamped},
    daemon_to_coordinator::{DaemonCoordinatorReply, DataflowDaemonResult},
};
use eyre::{bail, eyre, ContextCompat, WrapErr};
use futures::{stream::FuturesUnordered, Future, Stream, StreamExt};
use futures_concurrency::stream::Merge;
use log_subscriber::LogSubscriber;
use run::SpawnedDataflow;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::SocketAddr,
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::{net::TcpStream, sync::mpsc, task::JoinHandle};
use tokio_stream::wrappers::{ReceiverStream, TcpListenerStream};
use uuid::Uuid;

mod control;
mod listener;
mod log_subscriber;
mod run;
mod tcp_utils;

pub async fn start(
    bind: SocketAddr,
    bind_control: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let listener = listener::create_listener(bind).await?;
    let port = listener
        .local_addr()
        .wrap_err("failed to get local addr of listener")?
        .port();
    let new_daemon_connections = TcpListenerStream::new(listener).map(|c| {
        c.map(Event::NewDaemonConnection)
            .wrap_err("failed to open connection")
            .unwrap_or_else(Event::DaemonConnectError)
    });

    let mut tasks = FuturesUnordered::new();
    let control_events = control::control_events(bind_control, &tasks)
        .await
        .wrap_err("failed to create control events")?;

    // Setup ctrl-c handler
    let ctrlc_events = set_up_ctrlc_handler()?;

    let events = (
        external_events,
        new_daemon_connections,
        control_events,
        ctrlc_events,
    )
        .merge();

    let future = async move {
        start_inner(events, &tasks).await?;

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

// Resolve the dataflow name.
fn resolve_name(
    name: String,
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    archived_dataflows: &HashMap<Uuid, ArchivedDataflow>,
) -> eyre::Result<Uuid> {
    let uuids: Vec<_> = running_dataflows
        .iter()
        .filter(|(_, v)| v.name.as_deref() == Some(name.as_str()))
        .map(|(k, _)| k)
        .copied()
        .collect();
    let archived_uuids: Vec<_> = archived_dataflows
        .iter()
        .filter(|(_, v)| v.name.as_deref() == Some(name.as_str()))
        .map(|(k, _)| k)
        .copied()
        .collect();

    if uuids.is_empty() {
        if archived_uuids.is_empty() {
            bail!("no dataflow with name `{name}`");
        } else if let [uuid] = archived_uuids.as_slice() {
            Ok(*uuid)
        } else {
            // TODO: Index the archived dataflows in order to return logs based on the index.
            bail!("multiple archived dataflows found with name `{name}`, Please provide the UUID instead.");
        }
    } else if let [uuid] = uuids.as_slice() {
        Ok(*uuid)
    } else {
        bail!("multiple dataflows found with name `{name}`");
    }
}

async fn start_inner(
    events: impl Stream<Item = Event> + Unpin,
    tasks: &FuturesUnordered<JoinHandle<()>>,
) -> eyre::Result<()> {
    let clock = Arc::new(HLC::default());

    let (daemon_events_tx, daemon_events) = tokio::sync::mpsc::channel(2);
    let mut daemon_events_tx = Some(daemon_events_tx);
    let daemon_events = ReceiverStream::new(daemon_events);

    let daemon_heartbeat_interval =
        tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(Duration::from_secs(3)))
            .map(|_| Event::DaemonHeartbeatInterval);

    // events that should be aborted on `dora destroy`
    let (abortable_events, abort_handle) =
        futures::stream::abortable((events, daemon_heartbeat_interval).merge());

    let mut events = (abortable_events, daemon_events).merge();

    let mut running_dataflows: HashMap<Uuid, RunningDataflow> = HashMap::new();
    let mut dataflow_results: HashMap<Uuid, BTreeMap<String, DataflowDaemonResult>> =
        HashMap::new();
    let mut archived_dataflows: HashMap<Uuid, ArchivedDataflow> = HashMap::new();
    let mut daemon_connections: HashMap<_, DaemonConnection> = HashMap::new();

    while let Some(event) = events.next().await {
        if event.log() {
            tracing::trace!("Handling event {event:?}");
        }
        match event {
            Event::NewDaemonConnection(connection) => {
                connection.set_nodelay(true)?;
                let events_tx = daemon_events_tx.clone();
                if let Some(events_tx) = events_tx {
                    let task = tokio::spawn(listener::handle_connection(
                        connection,
                        events_tx,
                        clock.clone(),
                    ));
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
                DaemonRequest::Register {
                    machine_id,
                    mut connection,
                    version_check_result,
                    listen_port,
                } => {
                    let peer_ip = connection
                        .peer_addr()
                        .map(|addr| addr.ip())
                        .map_err(|err| format!("failed to get peer addr of connection: {err}"));
                    let register_result = version_check_result.and(peer_ip);

                    let reply: Timestamped<RegisterResult> = Timestamped {
                        inner: match &register_result {
                            Ok(_) => RegisterResult::Ok,
                            Err(err) => RegisterResult::Err(err.clone()),
                        },
                        timestamp: clock.new_timestamp(),
                    };
                    let send_result = tcp_send(&mut connection, &serde_json::to_vec(&reply)?).await;
                    match (register_result, send_result) {
                        (Ok(ip), Ok(())) => {
                            let previous = daemon_connections.insert(
                                machine_id.clone(),
                                DaemonConnection {
                                    stream: connection,
                                    listen_socket: (ip, listen_port).into(),
                                    last_heartbeat: Instant::now(),
                                },
                            );
                            if let Some(_previous) = previous {
                                tracing::info!(
                                    "closing previous connection `{machine_id}` on new register"
                                );
                            }
                        }
                        (Err(err), _) => {
                            tracing::warn!("failed to register daemon connection for machine `{machine_id}`: {err}");
                        }
                        (Ok(_), Err(err)) => {
                            tracing::warn!("failed to confirm daemon connection for machine `{machine_id}`: {err}");
                        }
                    }
                }
            },
            Event::Dataflow { uuid, event } => match event {
                DataflowEvent::ReadyOnMachine {
                    machine_id,
                    exited_before_subscribe,
                } => {
                    match running_dataflows.entry(uuid) {
                        std::collections::hash_map::Entry::Occupied(mut entry) => {
                            let dataflow = entry.get_mut();
                            dataflow.pending_machines.remove(&machine_id);
                            dataflow
                                .exited_before_subscribe
                                .extend(exited_before_subscribe);
                            if dataflow.pending_machines.is_empty() {
                                let message = serde_json::to_vec(&Timestamped {
                                    inner: DaemonCoordinatorEvent::AllNodesReady {
                                        dataflow_id: uuid,
                                        exited_before_subscribe: dataflow
                                            .exited_before_subscribe
                                            .clone(),
                                    },
                                    timestamp: clock.new_timestamp(),
                                })
                                .wrap_err("failed to serialize AllNodesReady message")?;

                                // notify all machines that run parts of the dataflow
                                for machine_id in &dataflow.machines {
                                    let Some(connection) = daemon_connections.get_mut(machine_id)
                                    else {
                                        tracing::warn!(
                                            "no daemon connection found for machine `{machine_id}`"
                                        );
                                        continue;
                                    };
                                    tcp_send(&mut connection.stream, &message)
                                        .await
                                        .wrap_err_with(|| {
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
                            // Archive finished dataflow
                            archived_dataflows
                                .entry(uuid)
                                .or_insert_with(|| ArchivedDataflow::from(entry.get()));
                            entry.get_mut().machines.remove(&machine_id);
                            dataflow_results
                                .entry(uuid)
                                .or_default()
                                .insert(machine_id, result);
                            if entry.get_mut().machines.is_empty() {
                                let finished_dataflow = entry.remove();
                                let reply = ControlRequestReply::DataflowStopped {
                                    uuid,
                                    result: dataflow_results
                                        .get(&uuid)
                                        .map(|r| dataflow_result(r, uuid, &clock))
                                        .unwrap_or_else(|| {
                                            DataflowResult::ok_empty(uuid, clock.new_timestamp())
                                        }),
                                };
                                for sender in finished_dataflow.reply_senders {
                                    let _ = sender.send(Ok(reply.clone()));
                                }
                            }
                        }
                        std::collections::hash_map::Entry::Vacant(_) => {
                            tracing::warn!("dataflow not running on DataflowFinishedOnMachine");
                        }
                    }
                }
            },

            Event::Control(event) => match event {
                ControlEvent::IncomingRequest {
                    request,
                    reply_sender,
                } => {
                    match request {
                        ControlRequest::Start {
                            dataflow,
                            name,
                            local_working_dir,
                        } => {
                            let name = name.or_else(|| names::Generator::default().next());

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
                                    dataflow,
                                    local_working_dir,
                                    name,
                                    &mut daemon_connections,
                                    &clock,
                                )
                                .await?;
                                Ok(dataflow)
                            };
                            let reply = inner.await.map(|dataflow| {
                                let uuid = dataflow.uuid;
                                running_dataflows.insert(uuid, dataflow);
                                ControlRequestReply::DataflowStarted { uuid }
                            });
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::Check { dataflow_uuid } => {
                            let status = match &running_dataflows.get(&dataflow_uuid) {
                                Some(_) => ControlRequestReply::DataflowStarted {
                                    uuid: dataflow_uuid,
                                },
                                None => ControlRequestReply::DataflowStopped {
                                    uuid: dataflow_uuid,
                                    result: dataflow_results
                                        .get(&dataflow_uuid)
                                        .map(|r| dataflow_result(r, dataflow_uuid, &clock))
                                        .unwrap_or_else(|| {
                                            DataflowResult::ok_empty(
                                                dataflow_uuid,
                                                clock.new_timestamp(),
                                            )
                                        }),
                                },
                            };
                            let _ = reply_sender.send(Ok(status));
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
                                    clock.new_timestamp(),
                                )
                                .await?;
                                Result::<_, eyre::Report>::Ok(())
                            };
                            let reply =
                                reload
                                    .await
                                    .map(|()| ControlRequestReply::DataflowReloaded {
                                        uuid: dataflow_id,
                                    });
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::Stop {
                            dataflow_uuid,
                            grace_duration,
                        } => {
                            if let Some(result) = dataflow_results.get(&dataflow_uuid) {
                                let reply = ControlRequestReply::DataflowStopped {
                                    uuid: dataflow_uuid,
                                    result: dataflow_result(result, dataflow_uuid, &clock),
                                };
                                let _ = reply_sender.send(Ok(reply));

                                continue;
                            }

                            let dataflow = stop_dataflow(
                                &mut running_dataflows,
                                dataflow_uuid,
                                &mut daemon_connections,
                                clock.new_timestamp(),
                                grace_duration,
                            )
                            .await;

                            match dataflow {
                                Ok(dataflow) => {
                                    dataflow.reply_senders.push(reply_sender);
                                }
                                Err(err) => {
                                    let _ = reply_sender.send(Err(err));
                                }
                            }
                        }
                        ControlRequest::StopByName {
                            name,
                            grace_duration,
                        } => match resolve_name(name, &running_dataflows, &archived_dataflows) {
                            Ok(dataflow_uuid) => {
                                if let Some(result) = dataflow_results.get(&dataflow_uuid) {
                                    let reply = ControlRequestReply::DataflowStopped {
                                        uuid: dataflow_uuid,
                                        result: dataflow_result(result, dataflow_uuid, &clock),
                                    };
                                    let _ = reply_sender.send(Ok(reply));

                                    continue;
                                }

                                let dataflow = stop_dataflow(
                                    &mut running_dataflows,
                                    dataflow_uuid,
                                    &mut daemon_connections,
                                    clock.new_timestamp(),
                                    grace_duration,
                                )
                                .await;

                                match dataflow {
                                    Ok(dataflow) => {
                                        dataflow.reply_senders.push(reply_sender);
                                    }
                                    Err(err) => {
                                        let _ = reply_sender.send(Err(err));
                                    }
                                }
                            }
                            Err(err) => {
                                let _ = reply_sender.send(Err(err));
                            }
                        },
                        ControlRequest::Logs { uuid, name, node } => {
                            let dataflow_uuid = if let Some(uuid) = uuid {
                                Ok(uuid)
                            } else if let Some(name) = name {
                                resolve_name(name, &running_dataflows, &archived_dataflows)
                            } else {
                                Err(eyre!("No uuid"))
                            };

                            match dataflow_uuid {
                                Ok(uuid) => {
                                    let reply = retrieve_logs(
                                        &running_dataflows,
                                        &archived_dataflows,
                                        uuid,
                                        node.into(),
                                        &mut daemon_connections,
                                        clock.new_timestamp(),
                                    )
                                    .await
                                    .map(ControlRequestReply::Logs);
                                    let _ = reply_sender.send(reply);
                                }
                                Err(err) => {
                                    let _ = reply_sender.send(Err(err));
                                }
                            }
                        }
                        ControlRequest::Destroy => {
                            tracing::info!("Received destroy command");

                            let reply = handle_destroy(
                                &mut running_dataflows,
                                &mut daemon_connections,
                                &abort_handle,
                                &mut daemon_events_tx,
                                &clock,
                            )
                            .await
                            .map(|()| ControlRequestReply::DestroyOk);
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::List => {
                            let mut dataflows: Vec<_> = running_dataflows.values().collect();
                            dataflows.sort_by_key(|d| (&d.name, d.uuid));

                            let running = dataflows.into_iter().map(|d| DataflowListEntry {
                                id: DataflowIdAndName {
                                    uuid: d.uuid,
                                    name: d.name.clone(),
                                },
                                status: DataflowStatus::Running,
                            });
                            let finished_failed =
                                dataflow_results.iter().map(|(&uuid, results)| {
                                    let name =
                                        archived_dataflows.get(&uuid).and_then(|d| d.name.clone());
                                    let id = DataflowIdAndName { uuid, name };
                                    let status = if results.values().all(|r| r.is_ok()) {
                                        DataflowStatus::Finished
                                    } else {
                                        DataflowStatus::Failed
                                    };
                                    DataflowListEntry { id, status }
                                });

                            let reply = Ok(ControlRequestReply::DataflowList(DataflowList(
                                running.chain(finished_failed).collect(),
                            )));
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::DaemonConnected => {
                            let running = !daemon_connections.is_empty();
                            let _ = reply_sender
                                .send(Ok(ControlRequestReply::DaemonConnected(running)));
                        }
                        ControlRequest::ConnectedMachines => {
                            let reply = Ok(ControlRequestReply::ConnectedMachines(
                                daemon_connections.keys().cloned().collect(),
                            ));
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::LogSubscribe { .. } => {
                            let _ = reply_sender.send(Err(eyre::eyre!(
                                "LogSubscribe request should be handled separately"
                            )));
                        }
                    }
                }
                ControlEvent::Error(err) => tracing::error!("{err:?}"),
                ControlEvent::LogSubscribe {
                    dataflow_id,
                    level,
                    connection,
                } => {
                    if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                        dataflow
                            .log_subscribers
                            .push(LogSubscriber::new(level, connection));
                    }
                }
            },
            Event::DaemonHeartbeatInterval => {
                let mut disconnected = BTreeSet::new();
                for (machine_id, connection) in &mut daemon_connections {
                    if connection.last_heartbeat.elapsed() > Duration::from_secs(15) {
                        tracing::warn!(
                            "no heartbeat message from machine `{machine_id}` since {:?}",
                            connection.last_heartbeat.elapsed()
                        )
                    }
                    if connection.last_heartbeat.elapsed() > Duration::from_secs(30) {
                        disconnected.insert(machine_id.clone());
                        continue;
                    }
                    let result: eyre::Result<()> = tokio::time::timeout(
                        Duration::from_millis(500),
                        send_heartbeat_message(&mut connection.stream, clock.new_timestamp()),
                    )
                    .await
                    .wrap_err("timeout")
                    .and_then(|r| r)
                    .wrap_err_with(|| {
                        format!("failed to send heartbeat message to daemon at `{machine_id}`")
                    });
                    if let Err(err) = result {
                        tracing::warn!("{err:?}");
                        disconnected.insert(machine_id.clone());
                    }
                }
                if !disconnected.is_empty() {
                    tracing::error!("Disconnecting daemons that failed watchdog: {disconnected:?}");
                    for machine_id in disconnected {
                        daemon_connections.remove(&machine_id);
                    }
                }
            }
            Event::CtrlC => {
                tracing::info!("Destroying coordinator after receiving Ctrl-C signal");
                handle_destroy(
                    &mut running_dataflows,
                    &mut daemon_connections,
                    &abort_handle,
                    &mut daemon_events_tx,
                    &clock,
                )
                .await?;
            }
            Event::DaemonHeartbeat { machine_id } => {
                if let Some(connection) = daemon_connections.get_mut(&machine_id) {
                    connection.last_heartbeat = Instant::now();
                }
            }
            Event::Log(message) => {
                if let Some(dataflow) = running_dataflows.get_mut(&message.dataflow_id) {
                    for subscriber in &mut dataflow.log_subscribers {
                        let send_result = tokio::time::timeout(
                            Duration::from_millis(100),
                            subscriber.send_message(&message),
                        );

                        if send_result.await.is_err() {
                            subscriber.close();
                        }
                    }
                    dataflow.log_subscribers.retain(|s| !s.is_closed());
                }
            }
        }
    }

    tracing::info!("stopped");

    Ok(())
}

fn dataflow_result(
    results: &BTreeMap<String, DataflowDaemonResult>,
    dataflow_uuid: Uuid,
    clock: &uhlc::HLC,
) -> DataflowResult {
    let mut node_results = BTreeMap::new();
    for result in results.values() {
        node_results.extend(result.node_results.clone());
        if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
            tracing::warn!("failed to update HLC: {err}");
        }
    }

    DataflowResult {
        uuid: dataflow_uuid,
        timestamp: clock.new_timestamp(),
        node_results,
    }
}

struct DaemonConnection {
    stream: TcpStream,
    listen_socket: SocketAddr,
    last_heartbeat: Instant,
}

async fn handle_destroy(
    running_dataflows: &mut HashMap<Uuid, RunningDataflow>,
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    abortable_events: &futures::stream::AbortHandle,
    daemon_events_tx: &mut Option<mpsc::Sender<Event>>,
    clock: &HLC,
) -> Result<(), eyre::ErrReport> {
    abortable_events.abort();
    for dataflow_uuid in running_dataflows.keys().cloned().collect::<Vec<_>>() {
        let _ = stop_dataflow(
            running_dataflows,
            dataflow_uuid,
            daemon_connections,
            clock.new_timestamp(),
            None,
        )
        .await?;
    }
    destroy_daemons(daemon_connections, clock.new_timestamp()).await?;
    *daemon_events_tx = None;
    Ok(())
}

async fn send_heartbeat_message(
    connection: &mut TcpStream,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Heartbeat,
        timestamp,
    })
    .context("Could not serialize heartbeat message")?;

    tcp_send(connection, &message)
        .await
        .wrap_err("failed to send heartbeat message to daemon")
}

struct RunningDataflow {
    name: Option<String>,
    uuid: Uuid,
    /// The IDs of the machines that the dataflow is running on.
    machines: BTreeSet<String>,
    /// IDs of machines that are waiting until all nodes are started.
    pending_machines: BTreeSet<String>,
    exited_before_subscribe: Vec<NodeId>,
    nodes: Vec<ResolvedNode>,

    reply_senders: Vec<tokio::sync::oneshot::Sender<eyre::Result<ControlRequestReply>>>,

    log_subscribers: Vec<LogSubscriber>,
}

struct ArchivedDataflow {
    name: Option<String>,
    nodes: Vec<ResolvedNode>,
}

impl From<&RunningDataflow> for ArchivedDataflow {
    fn from(dataflow: &RunningDataflow) -> ArchivedDataflow {
        ArchivedDataflow {
            name: dataflow.name.clone(),
            nodes: dataflow.nodes.clone(),
        }
    }
}

impl PartialEq for RunningDataflow {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.uuid == other.uuid && self.machines == other.machines
    }
}

impl Eq for RunningDataflow {}

async fn stop_dataflow<'a>(
    running_dataflows: &'a mut HashMap<Uuid, RunningDataflow>,
    dataflow_uuid: Uuid,
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    timestamp: uhlc::Timestamp,
    grace_duration: Option<Duration>,
) -> eyre::Result<&'a mut RunningDataflow> {
    let Some(dataflow) = running_dataflows.get_mut(&dataflow_uuid) else {
        bail!("no known running dataflow found with UUID `{dataflow_uuid}`")
    };

    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id: dataflow_uuid,
            grace_duration,
        },
        timestamp,
    })?;

    for machine_id in &dataflow.machines {
        let daemon_connection = daemon_connections
            .get_mut(machine_id)
            .wrap_err("no daemon connection")?; // TODO: take from dataflow spec
        tcp_send(&mut daemon_connection.stream, &message)
            .await
            .wrap_err("failed to send stop message to daemon")?;

        // wait for reply
        let reply_raw = tcp_receive(&mut daemon_connection.stream)
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

    tracing::info!("successfully send stop dataflow `{dataflow_uuid}` to all daemons");

    Ok(dataflow)
}

async fn reload_dataflow(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    operator_id: Option<OperatorId>,
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let Some(dataflow) = running_dataflows.get(&dataflow_id) else {
        bail!("No running dataflow found with UUID `{dataflow_id}`")
    };
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::ReloadDataflow {
            dataflow_id,
            node_id,
            operator_id,
        },
        timestamp,
    })?;

    for machine_id in &dataflow.machines {
        let daemon_connection = daemon_connections
            .get_mut(machine_id)
            .wrap_err("no daemon connection")?; // TODO: take from dataflow spec
        tcp_send(&mut daemon_connection.stream, &message)
            .await
            .wrap_err("failed to send reload message to daemon")?;

        // wait for reply
        let reply_raw = tcp_receive(&mut daemon_connection.stream)
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

async fn retrieve_logs(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    archived_dataflows: &HashMap<Uuid, ArchivedDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<Vec<u8>> {
    let nodes = if let Some(dataflow) = archived_dataflows.get(&dataflow_id) {
        dataflow.nodes.clone()
    } else if let Some(dataflow) = running_dataflows.get(&dataflow_id) {
        dataflow.nodes.clone()
    } else {
        bail!("No dataflow found with UUID `{dataflow_id}`")
    };

    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Logs {
            dataflow_id,
            node_id: node_id.clone(),
        },
        timestamp,
    })?;

    let machine_ids: Vec<String> = nodes
        .iter()
        .filter(|node| node.id == node_id)
        .map(|node| node.deploy.machine.clone())
        .collect();

    let machine_id = if let [machine_id] = &machine_ids[..] {
        machine_id
    } else if machine_ids.is_empty() {
        bail!("No machine contains {}/{}", dataflow_id, node_id)
    } else {
        bail!(
            "More than one machine contains {}/{}. However, it should only be present on one.",
            dataflow_id,
            node_id
        )
    };

    let daemon_connection = daemon_connections
        .get_mut(machine_id.as_str())
        .wrap_err("no daemon connection")?;
    tcp_send(&mut daemon_connection.stream, &message)
        .await
        .wrap_err("failed to send logs message to daemon")?;

    // wait for reply
    let reply_raw = tcp_receive(&mut daemon_connection.stream)
        .await
        .wrap_err("failed to retrieve logs reply from daemon")?;
    let reply_logs = match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize logs reply from daemon")?
    {
        DaemonCoordinatorReply::Logs(logs) => logs,
        other => bail!("unexpected reply after sending logs: {other:?}"),
    };
    tracing::info!("successfully retrieved logs for `{dataflow_id}/{node_id}`");

    reply_logs.map_err(|err| eyre!(err))
}

async fn start_dataflow(
    dataflow: Descriptor,
    working_dir: PathBuf,
    name: Option<String>,
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    clock: &HLC,
) -> eyre::Result<RunningDataflow> {
    let SpawnedDataflow {
        uuid,
        machines,
        nodes,
    } = spawn_dataflow(dataflow, working_dir, daemon_connections, clock).await?;
    Ok(RunningDataflow {
        uuid,
        name,
        pending_machines: if machines.len() > 1 {
            machines.clone()
        } else {
            BTreeSet::new()
        },
        exited_before_subscribe: Default::default(),
        machines,
        nodes,
        reply_senders: Vec::new(),
        log_subscribers: Vec::new(),
    })
}

async fn destroy_daemons(
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Destroy,
        timestamp,
    })?;

    for (machine_id, mut daemon_connection) in daemon_connections.drain() {
        tcp_send(&mut daemon_connection.stream, &message)
            .await
            .wrap_err("failed to send destroy message to daemon")?;

        // wait for reply
        let reply_raw = tcp_receive(&mut daemon_connection.stream)
            .await
            .wrap_err("failed to receive destroy reply from daemon")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize destroy reply from daemon")?
        {
            DaemonCoordinatorReply::DestroyResult { result, .. } => result
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
    DaemonHeartbeat { machine_id: String },
    Dataflow { uuid: Uuid, event: DataflowEvent },
    Control(ControlEvent),
    Daemon(DaemonRequest),
    DaemonHeartbeatInterval,
    CtrlC,
    Log(LogMessage),
}

impl Event {
    /// Whether this event should be logged.
    #[allow(clippy::match_like_matches_macro)]
    pub fn log(&self) -> bool {
        match self {
            Event::DaemonHeartbeatInterval => false,
            _ => true,
        }
    }
}

#[derive(Debug)]
pub enum DataflowEvent {
    DataflowFinishedOnMachine {
        machine_id: String,
        result: DataflowDaemonResult,
    },
    ReadyOnMachine {
        machine_id: String,
        exited_before_subscribe: Vec<NodeId>,
    },
}

#[derive(Debug)]
pub enum DaemonRequest {
    Register {
        version_check_result: Result<(), String>,
        machine_id: String,
        connection: TcpStream,
        listen_port: u16,
    },
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
