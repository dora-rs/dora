use crate::{
    run::spawn_dataflow,
    tcp_utils::{tcp_receive, tcp_send},
};
pub use control::ControlEvent;
use dora_core::{
    config::{NodeId, OperatorId},
    descriptor::DescriptorExt,
    uhlc::{self, HLC},
};
use dora_message::{
    cli_to_coordinator::ControlRequest,
    common::{DaemonId, GitSource},
    coordinator_to_cli::{
        ControlRequestReply, DataflowIdAndName, DataflowList, DataflowListEntry, DataflowResult,
        DataflowStatus, LogLevel, LogMessage,
    },
    coordinator_to_daemon::{
        BuildDataflowNodes, DaemonCoordinatorEvent, RegisterResult, Timestamped,
    },
    daemon_to_coordinator::{DaemonCoordinatorReply, DataflowDaemonResult},
    descriptor::{Descriptor, ResolvedNode},
    BuildId, DataflowId, SessionId,
};
use eyre::{bail, eyre, ContextCompat, Result, WrapErr};
use futures::{future::join_all, stream::FuturesUnordered, Future, Stream, StreamExt};
use futures_concurrency::stream::Merge;
use itertools::Itertools;
use log_subscriber::LogSubscriber;
use run::SpawnedDataflow;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::SocketAddr,
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::{
    net::TcpStream,
    sync::{mpsc, oneshot},
    task::JoinHandle,
};
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

#[derive(Default)]
struct DaemonConnections {
    daemons: BTreeMap<DaemonId, DaemonConnection>,
}

impl DaemonConnections {
    fn add(&mut self, daemon_id: DaemonId, connection: DaemonConnection) {
        let previous = self.daemons.insert(daemon_id.clone(), connection);
        if previous.is_some() {
            tracing::info!("closing previous connection `{daemon_id}` on new register");
        }
    }

    fn get(&self, id: &DaemonId) -> Option<&DaemonConnection> {
        self.daemons.get(id)
    }

    fn get_mut(&mut self, id: &DaemonId) -> Option<&mut DaemonConnection> {
        self.daemons.get_mut(id)
    }

    fn get_matching_daemon_id(&self, machine_id: &str) -> Option<&DaemonId> {
        self.daemons
            .keys()
            .find(|id| id.matches_machine_id(machine_id))
    }

    fn drain(&mut self) -> impl Iterator<Item = (DaemonId, DaemonConnection)> {
        std::mem::take(&mut self.daemons).into_iter()
    }

    fn is_empty(&self) -> bool {
        self.daemons.is_empty()
    }

    fn keys(&self) -> impl Iterator<Item = &DaemonId> {
        self.daemons.keys()
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = (&DaemonId, &mut DaemonConnection)> {
        self.daemons.iter_mut()
    }

    fn remove(&mut self, daemon_id: &DaemonId) -> Option<DaemonConnection> {
        self.daemons.remove(daemon_id)
    }

    fn unnamed(&self) -> impl Iterator<Item = &DaemonId> {
        self.daemons.keys().filter(|id| id.machine_id().is_none())
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

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    let mut finished_builds: HashMap<BuildId, CachedResult> = HashMap::new();

    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    let mut dataflow_results: HashMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        HashMap::new();
    let mut archived_dataflows: HashMap<DataflowId, ArchivedDataflow> = HashMap::new();
    let mut daemon_connections = DaemonConnections::default();

    while let Some(event) = events.next().await {
        // used below for measuring the event handling duration
        let start = Instant::now();
        let event_kind = event.kind();

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
                } => {
                    let existing = match &machine_id {
                        Some(id) => daemon_connections.get_matching_daemon_id(id),
                        None => daemon_connections.unnamed().next(),
                    };
                    let existing_result = if existing.is_some() {
                        Err(format!(
                            "There is already a connected daemon with machine ID `{machine_id:?}`"
                        ))
                    } else {
                        Ok(())
                    };

                    // assign a unique ID to the daemon
                    let daemon_id = DaemonId::new(machine_id);

                    let reply: Timestamped<RegisterResult> = Timestamped {
                        inner: match version_check_result.as_ref().and(existing_result.as_ref()) {
                            Ok(_) => RegisterResult::Ok {
                                daemon_id: daemon_id.clone(),
                            },
                            Err(err) => RegisterResult::Err(err.clone()),
                        },
                        timestamp: clock.new_timestamp(),
                    };

                    let send_result = tcp_send(&mut connection, &serde_json::to_vec(&reply)?)
                        .await
                        .context("tcp send failed");
                    match version_check_result.map_err(|e| eyre!(e)).and(send_result) {
                        Ok(()) => {
                            daemon_connections.add(
                                daemon_id.clone(),
                                DaemonConnection {
                                    stream: connection,
                                    last_heartbeat: Instant::now(),
                                },
                            );
                        }
                        Err(err) => {
                            tracing::warn!("failed to register daemon connection for daemon `{daemon_id}`: {err}");
                        }
                    }
                }
            },
            Event::Dataflow { uuid, event } => match event {
                DataflowEvent::ReadyOnDaemon {
                    daemon_id,
                    exited_before_subscribe,
                } => {
                    match running_dataflows.entry(uuid) {
                        std::collections::hash_map::Entry::Occupied(mut entry) => {
                            let dataflow = entry.get_mut();
                            dataflow.pending_daemons.remove(&daemon_id);
                            dataflow
                                .exited_before_subscribe
                                .extend(exited_before_subscribe);
                            if dataflow.pending_daemons.is_empty() {
                                tracing::debug!("sending all nodes ready message to daemons");
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
                                for daemon_id in &dataflow.daemons {
                                    let Some(connection) = daemon_connections.get_mut(daemon_id)
                                    else {
                                        tracing::warn!(
                                            "no daemon connection found for machine `{daemon_id}`"
                                        );
                                        continue;
                                    };
                                    tcp_send(&mut connection.stream, &message)
                                        .await
                                        .wrap_err_with(|| {
                                            format!(
                                                "failed to send AllNodesReady({uuid}) message \
                                            to machine {daemon_id}"
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
                DataflowEvent::DataflowFinishedOnDaemon { daemon_id, result } => {
                    tracing::debug!("coordinator received DataflowFinishedOnDaemon ({daemon_id:?}, result: {result:?})");
                    match running_dataflows.entry(uuid) {
                        std::collections::hash_map::Entry::Occupied(mut entry) => {
                            let dataflow = entry.get_mut();
                            dataflow.daemons.remove(&daemon_id);
                            tracing::info!(
                                "removed machine id: {daemon_id} from dataflow: {:#?}",
                                dataflow.uuid
                            );
                            dataflow_results
                                .entry(uuid)
                                .or_default()
                                .insert(daemon_id, result);

                            if dataflow.daemons.is_empty() {
                                // Archive finished dataflow
                                archived_dataflows
                                    .entry(uuid)
                                    .or_insert_with(|| ArchivedDataflow::from(entry.get()));
                                let mut finished_dataflow = entry.remove();
                                let dataflow_id = finished_dataflow.uuid;
                                send_log_message(
                                    &mut finished_dataflow.log_subscribers,
                                    &LogMessage {
                                        build_id: None,
                                        dataflow_id: Some(dataflow_id),
                                        node_id: None,
                                        daemon_id: None,
                                        level: LogLevel::Info.into(),
                                        target: Some("coordinator".into()),
                                        module_path: None,
                                        file: None,
                                        line: None,
                                        message: "dataflow finished".into(),
                                    },
                                )
                                .await;

                                let reply = ControlRequestReply::DataflowStopped {
                                    uuid,
                                    result: dataflow_results
                                        .get(&uuid)
                                        .map(|r| dataflow_result(r, uuid, &clock))
                                        .unwrap_or_else(|| {
                                            DataflowResult::ok_empty(uuid, clock.new_timestamp())
                                        }),
                                };
                                for sender in finished_dataflow.stop_reply_senders {
                                    let _ = sender.send(Ok(reply.clone()));
                                }
                                if !matches!(
                                    finished_dataflow.spawn_result,
                                    CachedResult::Cached { .. }
                                ) {
                                    log::error!("pending spawn result on dataflow finish");
                                }
                            }
                        }
                        std::collections::hash_map::Entry::Vacant(_) => {
                            tracing::warn!("dataflow not running on DataflowFinishedOnDaemon");
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
                        ControlRequest::Build {
                            session_id,
                            dataflow,
                            git_sources,
                            prev_git_sources,
                            local_working_dir,
                            uv,
                        } => {
                            // assign a random build id
                            let build_id = BuildId::generate();

                            let result = build_dataflow(
                                build_id,
                                session_id,
                                dataflow,
                                git_sources,
                                prev_git_sources,
                                local_working_dir,
                                &clock,
                                uv,
                                &mut daemon_connections,
                            )
                            .await;
                            match result {
                                Ok(build) => {
                                    running_builds.insert(build_id, build);
                                    let _ = reply_sender.send(Ok(
                                        ControlRequestReply::DataflowBuildTriggered { build_id },
                                    ));
                                }
                                Err(err) => {
                                    let _ = reply_sender.send(Err(err));
                                }
                            }
                        }
                        ControlRequest::WaitForBuild { build_id } => {
                            if let Some(build) = running_builds.get_mut(&build_id) {
                                build.build_result.register(reply_sender);
                            } else if let Some(result) = finished_builds.get_mut(&build_id) {
                                result.register(reply_sender);
                            } else {
                                let _ =
                                    reply_sender.send(Err(eyre!("unknown build id {build_id}")));
                            }
                        }
                        ControlRequest::Start {
                            build_id,
                            session_id,
                            dataflow,
                            name,
                            local_working_dir,
                            uv,
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
                                    build_id,
                                    session_id,
                                    dataflow,
                                    local_working_dir,
                                    name,
                                    &mut daemon_connections,
                                    &clock,
                                    uv,
                                )
                                .await?;
                                Ok(dataflow)
                            };
                            match inner.await {
                                Ok(dataflow) => {
                                    let uuid = dataflow.uuid;
                                    running_dataflows.insert(uuid, dataflow);
                                    let _ = reply_sender.send(Ok(
                                        ControlRequestReply::DataflowStartTriggered { uuid },
                                    ));
                                }
                                Err(err) => {
                                    let _ = reply_sender.send(Err(err));
                                }
                            }
                        }
                        ControlRequest::WaitForSpawn { dataflow_id } => {
                            if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                                dataflow.spawn_result.register(reply_sender);
                            } else {
                                let _ =
                                    reply_sender.send(Err(eyre!("unknown dataflow {dataflow_id}")));
                            }
                        }
                        ControlRequest::Check { dataflow_uuid } => {
                            let status = match &running_dataflows.get(&dataflow_uuid) {
                                Some(_) => ControlRequestReply::DataflowSpawned {
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
                                    dataflow.stop_reply_senders.push(reply_sender);
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
                                        dataflow.stop_reply_senders.push(reply_sender);
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
                            let reply = Ok(ControlRequestReply::ConnectedDaemons(
                                daemon_connections.keys().cloned().collect(),
                            ));
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::LogSubscribe { .. } => {
                            let _ = reply_sender.send(Err(eyre::eyre!(
                                "LogSubscribe request should be handled separately"
                            )));
                        }
                        ControlRequest::BuildLogSubscribe { .. } => {
                            let _ = reply_sender.send(Err(eyre::eyre!(
                                "BuildLogSubscribe request should be handled separately"
                            )));
                        }
                        ControlRequest::CliAndDefaultDaemonOnSameMachine => {
                            let mut default_daemon_ip = None;
                            if let Some(default_id) = daemon_connections.unnamed().next() {
                                if let Some(connection) = daemon_connections.get(default_id) {
                                    if let Ok(addr) = connection.stream.peer_addr() {
                                        default_daemon_ip = Some(addr.ip());
                                    }
                                }
                            }
                            let _ = reply_sender.send(Ok(
                                ControlRequestReply::CliAndDefaultDaemonIps {
                                    default_daemon: default_daemon_ip,
                                    cli: None, // filled later
                                },
                            ));
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
                ControlEvent::BuildLogSubscribe {
                    build_id,
                    level,
                    connection,
                } => {
                    if let Some(build) = running_builds.get_mut(&build_id) {
                        build
                            .log_subscribers
                            .push(LogSubscriber::new(level, connection));
                    }
                }
            },
            Event::DaemonHeartbeatInterval => {
                let mut disconnected = BTreeSet::new();
                for (machine_id, connection) in daemon_connections.iter_mut() {
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
            Event::DaemonHeartbeat {
                daemon_id: machine_id,
            } => {
                if let Some(connection) = daemon_connections.get_mut(&machine_id) {
                    connection.last_heartbeat = Instant::now();
                }
            }
            Event::Log(message) => {
                if let Some(dataflow_id) = &message.dataflow_id {
                    if let Some(dataflow) = running_dataflows.get_mut(dataflow_id) {
                        send_log_message(&mut dataflow.log_subscribers, &message).await;
                    }
                }
                if let Some(build_id) = message.build_id {
                    if let Some(build) = running_builds.get_mut(&build_id) {
                        send_log_message(&mut build.log_subscribers, &message).await;
                    }
                }
            }
            Event::DaemonExit { daemon_id } => {
                tracing::info!("Daemon `{daemon_id}` exited");
                daemon_connections.remove(&daemon_id);
            }
            Event::DataflowBuildResult {
                build_id,
                daemon_id,
                result,
            } => match running_builds.get_mut(&build_id) {
                Some(build) => {
                    build.pending_build_results.remove(&daemon_id);
                    match result {
                        Ok(()) => {}
                        Err(err) => {
                            build.errors.push(format!("{err:?}"));
                        }
                    };
                    if build.pending_build_results.is_empty() {
                        tracing::info!("dataflow build finished: `{build_id}`");
                        let mut build = running_builds.remove(&build_id).unwrap();
                        let result = if build.errors.is_empty() {
                            Ok(())
                        } else {
                            Err(format!("build failed: {}", build.errors.join("\n\n")))
                        };

                        build.build_result.set_result(Ok(
                            ControlRequestReply::DataflowBuildFinished { build_id, result },
                        ));

                        finished_builds.insert(build_id, build.build_result);
                    }
                }
                None => {
                    tracing::warn!("received DataflowSpawnResult, but no matching dataflow in `running_dataflows` map");
                }
            },
            Event::DataflowSpawnResult {
                dataflow_id,
                daemon_id,
                result,
            } => match running_dataflows.get_mut(&dataflow_id) {
                Some(dataflow) => {
                    dataflow.pending_spawn_results.remove(&daemon_id);
                    match result {
                        Ok(()) => {
                            if dataflow.pending_spawn_results.is_empty() {
                                tracing::info!("successfully spawned dataflow `{dataflow_id}`",);
                                dataflow.spawn_result.set_result(Ok(
                                    ControlRequestReply::DataflowSpawned { uuid: dataflow_id },
                                ));
                            }
                        }
                        Err(err) => {
                            tracing::warn!("error while spawning dataflow `{dataflow_id}`");
                            dataflow.spawn_result.set_result(Err(err));
                        }
                    };
                }
                None => {
                    tracing::warn!("received DataflowSpawnResult, but no matching dataflow in `running_dataflows` map");
                }
            },
        }

        // warn if event handling took too long -> the main loop should never be blocked for too long
        let elapsed = start.elapsed();
        if elapsed > Duration::from_millis(100) {
            tracing::warn!(
                "Coordinator took {}ms for handling event: {event_kind}",
                elapsed.as_millis()
            );
        }
    }

    tracing::info!("stopped");

    Ok(())
}

async fn send_log_message(log_subscribers: &mut Vec<LogSubscriber>, message: &LogMessage) {
    for subscriber in log_subscribers.iter_mut() {
        let send_result =
            tokio::time::timeout(Duration::from_millis(100), subscriber.send_message(message));

        if send_result.await.is_err() {
            subscriber.close();
        }
    }
    log_subscribers.retain(|s| !s.is_closed());
}

fn dataflow_result(
    results: &BTreeMap<DaemonId, DataflowDaemonResult>,
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
    last_heartbeat: Instant,
}

async fn handle_destroy(
    running_dataflows: &mut HashMap<Uuid, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
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

    let result = destroy_daemons(daemon_connections, clock.new_timestamp()).await;
    *daemon_events_tx = None;
    result
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

struct RunningBuild {
    errors: Vec<String>,
    build_result: CachedResult,

    log_subscribers: Vec<LogSubscriber>,

    pending_build_results: BTreeSet<DaemonId>,
}

struct RunningDataflow {
    name: Option<String>,
    uuid: Uuid,
    /// The IDs of the daemons that the dataflow is running on.
    daemons: BTreeSet<DaemonId>,
    /// IDs of daemons that are waiting until all nodes are started.
    pending_daemons: BTreeSet<DaemonId>,
    exited_before_subscribe: Vec<NodeId>,
    nodes: BTreeMap<NodeId, ResolvedNode>,

    spawn_result: CachedResult,
    stop_reply_senders: Vec<tokio::sync::oneshot::Sender<eyre::Result<ControlRequestReply>>>,

    log_subscribers: Vec<LogSubscriber>,

    pending_spawn_results: BTreeSet<DaemonId>,
}

pub enum CachedResult {
    Pending {
        result_senders: Vec<tokio::sync::oneshot::Sender<eyre::Result<ControlRequestReply>>>,
    },
    Cached {
        result: eyre::Result<ControlRequestReply>,
    },
}

impl Default for CachedResult {
    fn default() -> Self {
        Self::Pending {
            result_senders: Vec::new(),
        }
    }
}

impl CachedResult {
    fn register(
        &mut self,
        reply_sender: tokio::sync::oneshot::Sender<eyre::Result<ControlRequestReply>>,
    ) {
        match self {
            CachedResult::Pending { result_senders } => result_senders.push(reply_sender),
            CachedResult::Cached { result } => {
                Self::send_result_to(result, reply_sender);
            }
        }
    }

    fn set_result(&mut self, result: eyre::Result<ControlRequestReply>) {
        match self {
            CachedResult::Pending { result_senders } => {
                for sender in result_senders.drain(..) {
                    Self::send_result_to(&result, sender);
                }
                *self = CachedResult::Cached { result };
            }
            CachedResult::Cached { .. } => {}
        }
    }

    fn send_result_to(
        result: &eyre::Result<ControlRequestReply>,
        sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
    ) {
        let result = match result {
            Ok(r) => Ok(r.clone()),
            Err(err) => Err(eyre!("{err:?}")),
        };
        let _ = sender.send(result);
    }
}

struct ArchivedDataflow {
    name: Option<String>,
    nodes: BTreeMap<NodeId, ResolvedNode>,
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
        self.name == other.name && self.uuid == other.uuid && self.daemons == other.daemons
    }
}

impl Eq for RunningDataflow {}

async fn stop_dataflow<'a>(
    running_dataflows: &'a mut HashMap<Uuid, RunningDataflow>,
    dataflow_uuid: Uuid,
    daemon_connections: &mut DaemonConnections,
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

    for daemon_id in &dataflow.daemons {
        let daemon_connection = daemon_connections
            .get_mut(daemon_id)
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
    daemon_connections: &mut DaemonConnections,
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

    for machine_id in &dataflow.daemons {
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
    daemon_connections: &mut DaemonConnections,
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

    let machine_ids: Vec<Option<String>> = nodes
        .values()
        .filter(|node| node.id == node_id)
        .map(|node| node.deploy.as_ref().and_then(|d| d.machine.clone()))
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

    let daemon_ids: Vec<_> = match machine_id {
        None => daemon_connections.unnamed().collect(),
        Some(machine_id) => daemon_connections
            .get_matching_daemon_id(machine_id)
            .into_iter()
            .collect(),
    };
    let daemon_id = match &daemon_ids[..] {
        [id] => (*id).clone(),
        [] => eyre::bail!("no matching daemon connections for machine ID `{machine_id:?}`"),
        _ => eyre::bail!("multiple matching daemon connections for machine ID `{machine_id:?}`"),
    };
    let daemon_connection = daemon_connections
        .get_mut(&daemon_id)
        .wrap_err_with(|| format!("no daemon connection to `{daemon_id}`"))?;
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

#[allow(clippy::too_many_arguments)]
#[tracing::instrument(skip(daemon_connections, clock))]
async fn build_dataflow(
    build_id: BuildId,
    session_id: SessionId,
    dataflow: Descriptor,
    git_sources: BTreeMap<NodeId, GitSource>,
    prev_git_sources: BTreeMap<NodeId, GitSource>,
    local_working_dir: Option<PathBuf>,
    clock: &HLC,
    uv: bool,
    daemon_connections: &mut DaemonConnections,
) -> eyre::Result<RunningBuild> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

    let mut git_sources_by_daemon = git_sources
        .into_iter()
        .into_grouping_map_by(|(id, _)| {
            nodes
                .get(id)
                .and_then(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()))
        })
        .collect();
    let mut prev_git_sources_by_daemon = prev_git_sources
        .into_iter()
        .into_grouping_map_by(|(id, _)| {
            nodes
                .get(id)
                .and_then(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()))
        })
        .collect();

    let nodes_by_daemon = nodes
        .values()
        .into_group_map_by(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()));

    let mut daemons = BTreeSet::new();
    for (machine, nodes_on_machine) in &nodes_by_daemon {
        let nodes_on_machine = nodes_on_machine.iter().map(|n| n.id.clone()).collect();
        tracing::debug!(
            "Running dataflow build `{build_id}` on machine `{machine:?}` (nodes: {nodes_on_machine:?})"
        );

        let build_command = BuildDataflowNodes {
            build_id,
            session_id,
            local_working_dir: local_working_dir.clone(),
            git_sources: git_sources_by_daemon.remove(machine).unwrap_or_default(),
            prev_git_sources: prev_git_sources_by_daemon
                .remove(machine)
                .unwrap_or_default(),
            dataflow_descriptor: dataflow.clone(),
            nodes_on_machine,
            uv,
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::Build(build_command),
            timestamp: clock.new_timestamp(),
        })?;

        let daemon_id =
            build_dataflow_on_machine(daemon_connections, machine.map(|s| s.as_str()), &message)
                .await
                .wrap_err_with(|| format!("failed to build dataflow on machine `{machine:?}`"))?;
        daemons.insert(daemon_id);
    }

    tracing::info!("successfully triggered dataflow build `{build_id}`",);

    Ok(RunningBuild {
        errors: Vec::new(),
        build_result: CachedResult::default(),
        log_subscribers: Vec::new(),
        pending_build_results: daemons,
    })
}

async fn build_dataflow_on_machine(
    daemon_connections: &mut DaemonConnections,
    machine: Option<&str>,
    message: &[u8],
) -> Result<DaemonId, eyre::ErrReport> {
    let daemon_id = match machine {
        Some(machine) => daemon_connections
            .get_matching_daemon_id(machine)
            .wrap_err_with(|| format!("no matching daemon for machine id {machine:?}"))?
            .clone(),
        None => daemon_connections
            .unnamed()
            .next()
            .wrap_err("no unnamed daemon connections")?
            .clone(),
    };

    let daemon_connection = daemon_connections
        .get_mut(&daemon_id)
        .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?;
    tcp_send(&mut daemon_connection.stream, message)
        .await
        .wrap_err("failed to send build message to daemon")?;

    let reply_raw = tcp_receive(&mut daemon_connection.stream)
        .await
        .wrap_err("failed to receive build reply from daemon")?;
    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize build reply from daemon")?
    {
        DaemonCoordinatorReply::TriggerBuildResult(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("daemon returned an error")?,
        _ => bail!("unexpected reply"),
    }
    Ok(daemon_id)
}

#[allow(clippy::too_many_arguments)]
async fn start_dataflow(
    build_id: Option<BuildId>,
    session_id: SessionId,
    dataflow: Descriptor,
    local_working_dir: Option<PathBuf>,
    name: Option<String>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
    uv: bool,
) -> eyre::Result<RunningDataflow> {
    let SpawnedDataflow {
        uuid,
        daemons,
        nodes,
    } = spawn_dataflow(
        build_id,
        session_id,
        dataflow,
        local_working_dir,
        daemon_connections,
        clock,
        uv,
    )
    .await?;
    Ok(RunningDataflow {
        uuid,
        name,
        pending_daemons: if daemons.len() > 1 {
            daemons.clone()
        } else {
            BTreeSet::new()
        },
        exited_before_subscribe: Default::default(),
        daemons: daemons.clone(),
        nodes,
        spawn_result: CachedResult::default(),
        stop_reply_senders: Vec::new(),
        log_subscribers: Vec::new(),
        pending_spawn_results: daemons,
    })
}

async fn destroy_daemon(
    daemon_id: DaemonId,
    mut daemon_connection: DaemonConnection,

    timestamp: uhlc::Timestamp,
) -> Result<()> {
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Destroy,
        timestamp,
    })?;

    tcp_send(&mut daemon_connection.stream, &message)
        .await
        .wrap_err(format!(
            "failed to send destroy message to daemon `{daemon_id}`"
        ))?;

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

    tracing::info!("successfully destroyed daemon `{daemon_id}`");
    Ok(())
}

async fn destroy_daemons(
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let futures = daemon_connections
        .drain()
        .map(|(daemon_id, daemon_connection)| {
            destroy_daemon(daemon_id, daemon_connection, timestamp)
        })
        .collect::<Vec<_>>();
    let results: Vec<std::result::Result<(), eyre::Error>> =
        join_all(futures).await.into_iter().collect::<Vec<_>>();
    for result in results {
        result?;
    }
    Ok(())
}

#[derive(Debug)]
pub enum Event {
    NewDaemonConnection(TcpStream),
    DaemonConnectError(eyre::Report),
    DaemonHeartbeat {
        daemon_id: DaemonId,
    },
    Dataflow {
        uuid: Uuid,
        event: DataflowEvent,
    },
    Control(ControlEvent),
    Daemon(DaemonRequest),
    DaemonHeartbeatInterval,
    CtrlC,
    Log(LogMessage),
    DaemonExit {
        daemon_id: dora_message::common::DaemonId,
    },
    DataflowBuildResult {
        build_id: BuildId,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
    DataflowSpawnResult {
        dataflow_id: uuid::Uuid,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
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

    fn kind(&self) -> &'static str {
        match self {
            Event::NewDaemonConnection(_) => "NewDaemonConnection",
            Event::DaemonConnectError(_) => "DaemonConnectError",
            Event::DaemonHeartbeat { .. } => "DaemonHeartbeat",
            Event::Dataflow { .. } => "Dataflow",
            Event::Control(_) => "Control",
            Event::Daemon(_) => "Daemon",
            Event::DaemonHeartbeatInterval => "DaemonHeartbeatInterval",
            Event::CtrlC => "CtrlC",
            Event::Log(_) => "Log",
            Event::DaemonExit { .. } => "DaemonExit",
            Event::DataflowBuildResult { .. } => "DataflowBuildResult",
            Event::DataflowSpawnResult { .. } => "DataflowSpawnResult",
        }
    }
}

#[derive(Debug)]
pub enum DataflowEvent {
    DataflowFinishedOnDaemon {
        daemon_id: DaemonId,
        result: DataflowDaemonResult,
    },
    ReadyOnDaemon {
        daemon_id: DaemonId,
        exited_before_subscribe: Vec<NodeId>,
    },
}

#[derive(Debug)]
pub enum DaemonRequest {
    Register {
        version_check_result: Result<(), String>,
        machine_id: Option<String>,
        connection: TcpStream,
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
