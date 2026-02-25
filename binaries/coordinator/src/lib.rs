use crate::{
    events::set_up_ctrlc_handler,
    handlers::{
        build_dataflow, dataflow_result, handle_destroy, reload_dataflow, resolve_name,
        retrieve_logs, send_heartbeat_message, send_log_message, start_dataflow, stop_dataflow,
    },
    state::{ArchivedDataflow, CachedResult, RunningBuild, RunningDataflow},
};
use adora_core::uhlc::HLC;
use adora_message::{
    BuildId, DataflowId,
    cli_to_coordinator::ControlRequest,
    common::DaemonId,
    coordinator_to_cli::{
        ControlRequestReply, DataflowIdAndName, DataflowList, DataflowListEntry, DataflowResult,
        DataflowStatus, LogLevel, LogMessage,
    },
    coordinator_to_daemon::{DaemonCoordinatorEvent, RegisterResult, Timestamped},
    daemon_to_coordinator::DataflowDaemonResult,
};
pub use control::ControlEvent;
pub use events::{DaemonRequest, DataflowEvent, Event};
use eyre::{Result, WrapErr, bail, eyre};
use futures::{Future, Stream, StreamExt, stream::FuturesUnordered};
use futures_concurrency::stream::Merge;
use log_subscriber::LogSubscriber;
use petname::petname;
pub(crate) use state::DaemonConnections;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::SocketAddr,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio_stream::wrappers::ReceiverStream;

mod control;
mod events;
mod handlers;
mod log_subscriber;
mod run;
mod state;
mod ws_control;
mod ws_daemon;
mod ws_server;

pub async fn start(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let ctrlc_events = set_up_ctrlc_handler()?;
    start_with_events(bind, external_events, ctrlc_events).await
}

/// Like [`start`] but without registering a ctrl-c handler.
/// Useful for tests that run multiple coordinators in the same process.
#[doc(hidden)]
pub async fn start_testing(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    start_with_events(bind, external_events, futures::stream::empty()).await
}

async fn start_with_events(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
    extra_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let clock = Arc::new(HLC::default());

    let mut tasks = FuturesUnordered::new();

    // Setup WS event channel (used by axum WS handlers)
    let (ws_event_tx, ws_event_rx) = tokio::sync::mpsc::channel::<Event>(64);
    let ws_events = ReceiverStream::new(ws_event_rx);

    // Start WS server
    let (port, ws_shutdown, ws_future) = ws_server::serve(bind, ws_event_tx.clone(), clock.clone())
        .await
        .wrap_err("failed to start WS server")?;
    tracing::info!("WS server listening on port {port}");
    tasks.push(tokio::spawn(async move {
        if let Err(e) = ws_future.await {
            tracing::error!("WS server error: {e:?}");
        }
    }));

    let events = (external_events, extra_events, ws_events).merge();

    let future = async move {
        start_inner(events, clock).await?;

        tracing::debug!("coordinator main loop finished, shutting down WS server");
        ws_shutdown.shutdown();

        tracing::debug!("waiting on spawned tasks");
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
    events: impl Stream<Item = Event> + Unpin,
    clock: Arc<HLC>,
) -> eyre::Result<()> {
    let daemon_heartbeat_interval =
        tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(Duration::from_secs(3)))
            .map(|_| Event::DaemonHeartbeatInterval);

    // events that should be aborted on `adora destroy`
    let (abortable_events, abort_handle) =
        futures::stream::abortable((events, daemon_heartbeat_interval).merge());

    let mut events = abortable_events;

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
            Event::Daemon(event) => match event {
                DaemonRequest::Register {
                    machine_id,
                    mut connection,
                    version_check_result,
                    daemon_id_tx,
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

                    let send_result = connection
                        .send(&serde_json::to_vec(&reply)?)
                        .await
                        .context("failed to send register reply");
                    match version_check_result
                        .map_err(|e| eyre!(e))
                        .and(existing_result.map_err(|e| eyre!(e)))
                        .and(send_result)
                    {
                        Ok(()) => {
                            let _ = daemon_id_tx.send(daemon_id.clone());
                            daemon_connections.add(daemon_id.clone(), connection);
                        }
                        Err(err) => {
                            tracing::warn!(
                                "failed to register daemon connection for daemon `{daemon_id}`: {err}"
                            );
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
                                    connection.send(&message).await.wrap_err_with(|| {
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
                    tracing::debug!(
                        "coordinator received DataflowFinishedOnDaemon ({daemon_id:?}, result: {result:?})"
                    );
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
                                        timestamp: clock
                                            .new_timestamp()
                                            .get_time()
                                            .to_system_time()
                                            .into(),
                                        fields: None,
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
                            write_events_to,
                        } => {
                            let name = name.or_else(|| petname(2, "-"));

                            let inner = async {
                                if let Some(name) = name.as_deref() {
                                    // check that name is unique
                                    if running_dataflows
                                        .values()
                                        .any(|d: &RunningDataflow| d.name.as_deref() == Some(name))
                                    {
                                        bail!(
                                            "there is already a running dataflow with name `{name}`"
                                        );
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
                                    write_events_to,
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
                            force,
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
                                force,
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
                            force,
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
                                    force,
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
                        ControlRequest::Logs {
                            uuid,
                            name,
                            node,
                            tail,
                        } => {
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
                                        tail,
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
                        ControlRequest::Info { dataflow_uuid } => {
                            if let Some(dataflow) = running_dataflows.get(&dataflow_uuid) {
                                let _ = reply_sender.send(Ok(ControlRequestReply::DataflowInfo {
                                    uuid: dataflow.uuid,
                                    name: dataflow.name.clone(),
                                    descriptor: dataflow.descriptor.clone(),
                                }));
                            } else {
                                let _ = reply_sender.send(Err(eyre!(
                                    "No running dataflow with uuid `{dataflow_uuid}`"
                                )));
                            }
                        }
                        ControlRequest::Destroy => {
                            tracing::info!("Received destroy command");

                            let reply = handle_destroy(
                                &mut running_dataflows,
                                &mut daemon_connections,
                                &abort_handle,
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
                            let daemon_infos: Vec<_> = daemon_connections
                                .iter_mut()
                                .map(|(id, conn)| adora_message::coordinator_to_cli::DaemonInfo {
                                    daemon_id: id.clone(),
                                    last_heartbeat_ago_ms: conn.last_heartbeat.elapsed().as_millis()
                                        as u64,
                                })
                                .collect();
                            let reply = Ok(ControlRequestReply::ConnectedDaemons(daemon_infos));
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
                            // With WS we can't inspect peer addresses.
                            // If an unnamed (local) daemon is connected, assume same
                            // machine by returning localhost for both.
                            let has_unnamed = daemon_connections.unnamed().next().is_some();
                            let ip = if has_unnamed {
                                Some(std::net::IpAddr::V4(std::net::Ipv4Addr::LOCALHOST))
                            } else {
                                None
                            };
                            let _ = reply_sender.send(Ok(
                                ControlRequestReply::CliAndDefaultDaemonIps {
                                    default_daemon: ip,
                                    cli: ip,
                                },
                            ));
                        }
                        ControlRequest::GetNodeInfo => {
                            use adora_message::coordinator_to_cli::{NodeInfo, NodeMetricsInfo};

                            let mut node_infos = Vec::new();
                            for dataflow in running_dataflows.values() {
                                for node_id in dataflow.nodes.keys() {
                                    // Get the specific daemon this node is running on
                                    if let Some(daemon_id) = dataflow.node_to_daemon.get(node_id) {
                                        // Get metrics if available
                                        let metrics = dataflow.node_metrics.get(node_id).map(|m| {
                                            NodeMetricsInfo {
                                                pid: m.pid,
                                                cpu_usage: m.cpu_usage,
                                                // Use 1000 for MB (megabytes) instead of 1024 (mebibytes)
                                                memory_mb: m.memory_bytes as f64 / 1000.0 / 1000.0,
                                                disk_read_mb_s: m
                                                    .disk_read_bytes
                                                    .map(|b| b as f64 / 1000.0 / 1000.0),
                                                disk_write_mb_s: m
                                                    .disk_write_bytes
                                                    .map(|b| b as f64 / 1000.0 / 1000.0),
                                            }
                                        });

                                        node_infos.push(NodeInfo {
                                            dataflow_id: dataflow.uuid,
                                            dataflow_name: dataflow.name.clone(),
                                            node_id: node_id.clone(),
                                            daemon_id: daemon_id.clone(),
                                            metrics,
                                        });
                                    }
                                }
                            }
                            let _ = reply_sender
                                .send(Ok(ControlRequestReply::NodeInfoList(node_infos)));
                        }
                    }
                }
                ControlEvent::Error(err) => tracing::error!("{err:?}"),
                ControlEvent::LogSubscribe {
                    dataflow_id,
                    level,
                    sender,
                    found_tx,
                } => {
                    if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                        dataflow
                            .log_subscribers
                            .push(LogSubscriber::new(level, sender));
                        let buffered = std::mem::take(&mut dataflow.buffered_log_messages);
                        for message in buffered {
                            send_log_message(&mut dataflow.log_subscribers, &message).await;
                        }
                        let _ = found_tx.send(true);
                    } else {
                        let _ = found_tx.send(false);
                    }
                }
                ControlEvent::BuildLogSubscribe {
                    build_id,
                    level,
                    sender,
                    found_tx,
                } => {
                    if let Some(build) = running_builds.get_mut(&build_id) {
                        build
                            .log_subscribers
                            .push(LogSubscriber::new(level, sender));
                        let buffered = std::mem::take(&mut build.buffered_log_messages);
                        for message in buffered {
                            send_log_message(&mut build.log_subscribers, &message).await;
                        }
                        let _ = found_tx.send(true);
                    } else {
                        let _ = found_tx.send(false);
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
                        send_heartbeat_message(connection, clock.new_timestamp()),
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
                    for machine_id in &disconnected {
                        daemon_connections.remove(machine_id);
                    }
                    // Notify remaining daemons about disconnected peers
                    for disconnected_id in &disconnected {
                        let msg = serde_json::to_vec(&Timestamped {
                            inner: DaemonCoordinatorEvent::PeerDaemonDisconnected {
                                daemon_id: disconnected_id.clone(),
                            },
                            timestamp: clock.new_timestamp(),
                        })
                        .wrap_err("failed to serialize PeerDaemonDisconnected")?;
                        for (_id, conn) in daemon_connections.iter_mut() {
                            if let Err(err) = conn.send(&msg).await {
                                tracing::warn!("failed to notify daemon of peer disconnect: {err}");
                            }
                        }
                    }
                }
            }
            Event::CtrlC => {
                tracing::info!("Destroying coordinator after receiving Ctrl-C signal");
                handle_destroy(
                    &mut running_dataflows,
                    &mut daemon_connections,
                    &abort_handle,
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
                const MAX_BUFFERED_LOG_MESSAGES: usize = 10_000;
                if let Some(dataflow_id) = &message.dataflow_id {
                    if let Some(dataflow) = running_dataflows.get_mut(dataflow_id) {
                        if dataflow.log_subscribers.is_empty() {
                            if dataflow.buffered_log_messages.len() < MAX_BUFFERED_LOG_MESSAGES {
                                dataflow.buffered_log_messages.push(message);
                            } else if dataflow.buffered_log_messages.len()
                                == MAX_BUFFERED_LOG_MESSAGES
                            {
                                tracing::warn!(
                                    "log buffer full for dataflow {dataflow_id}, dropping new messages"
                                );
                                dataflow.buffered_log_messages.push(message);
                            }
                        } else {
                            send_log_message(&mut dataflow.log_subscribers, &message).await;
                        }
                    }
                } else if let Some(build_id) = &message.build_id {
                    if let Some(build) = running_builds.get_mut(build_id) {
                        if build.log_subscribers.is_empty() {
                            if build.buffered_log_messages.len() < MAX_BUFFERED_LOG_MESSAGES {
                                build.buffered_log_messages.push(message);
                            } else if build.buffered_log_messages.len() == MAX_BUFFERED_LOG_MESSAGES
                            {
                                tracing::warn!(
                                    "log buffer full for build {build_id}, dropping new messages"
                                );
                                build.buffered_log_messages.push(message);
                            }
                        } else {
                            send_log_message(&mut build.log_subscribers, &message).await;
                        }
                    }
                }
            }
            Event::DaemonExit { daemon_id } => {
                tracing::info!("Daemon `{daemon_id}` exited");
                daemon_connections.remove(&daemon_id);
            }
            Event::NodeMetrics {
                dataflow_id,
                metrics,
            } => {
                // Store metrics for this dataflow
                if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                    for (node_id, node_metrics) in metrics {
                        dataflow.node_metrics.insert(node_id, node_metrics);
                    }
                }
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
                    tracing::warn!(
                        "received DataflowSpawnResult, but no matching dataflow in `running_dataflows` map"
                    );
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
                    tracing::warn!(
                        "received DataflowSpawnResult, but no matching dataflow in `running_dataflows` map"
                    );
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
