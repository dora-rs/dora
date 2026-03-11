use crate::{
    events::set_up_ctrlc_handler,
    handlers::{
        build_dataflow, dataflow_result, handle_destroy, reload_dataflow, resolve_name,
        restart_node, retrieve_logs, send_heartbeat_message, send_log_message, start_dataflow,
        stop_dataflow, stop_node,
    },
    state::{ArchivedDataflow, CachedResult, RunningBuild, RunningDataflow},
};
use adora_coordinator_store::DataflowStatus as StoreDataflowStatus;
pub use adora_coordinator_store::{self, CoordinatorStore, InMemoryStore};
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

pub(crate) mod artifacts;
mod control;
mod events;
mod handlers;
mod log_subscriber;
#[cfg(feature = "prometheus")]
mod prometheus_metrics;
mod run;
mod state;
mod ws_control;
mod ws_daemon;
mod ws_server;

/// Type alias for the optional in-memory span store.
///
/// When `Some`, the coordinator will serve `GetTraces` / `GetTraceSpans`
/// requests by reading captured spans from this store.
#[cfg(feature = "tracing")]
pub type SpanStore = Option<adora_tracing::span_store::SharedSpanStore>;
#[cfg(not(feature = "tracing"))]
pub type SpanStore = ();

/// Start the coordinator without authentication (default).
pub async fn start(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
    store: Arc<dyn CoordinatorStore>,
    span_store: SpanStore,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    start_with_auth(bind, external_events, store, span_store, false).await
}

/// Like [`start`] but allows enabling token authentication.
///
/// When `auth` is `true`, the coordinator generates a random token on startup,
/// writes it to `~/.config/adora/.adora-token`, and requires all clients to
/// present it via the `Authorization: Bearer <token>` header.
pub async fn start_with_auth(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
    store: Arc<dyn CoordinatorStore>,
    span_store: SpanStore,
    auth: bool,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let ctrlc_events = set_up_ctrlc_handler()?;

    let token = if auth {
        let token = adora_message::auth::generate_token();
        if let Ok(cwd) = std::env::current_dir() {
            if let Err(e) = adora_message::auth::write_token(&cwd, &token) {
                tracing::warn!("failed to write auth token: {e}");
            } else {
                tracing::info!(
                    "auth token written to {}",
                    adora_message::auth::token_path(&cwd).display()
                );
            }
        }
        Some(token)
    } else {
        None
    };

    start_with_events(
        bind,
        external_events,
        ctrlc_events,
        store,
        token,
        span_store,
    )
    .await
}

/// Like [`start`] but without registering a ctrl-c handler.
/// For embedding the coordinator in another process (e.g., `adora run`).
pub async fn start_embedded(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
    store: Arc<dyn CoordinatorStore>,
    span_store: SpanStore,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    start_with_events(
        bind,
        external_events,
        futures::stream::empty(),
        store,
        None,
        span_store,
    )
    .await
}

/// Like [`start`] but without registering a ctrl-c handler.
/// Useful for tests that run multiple coordinators in the same process.
/// Testing-only entry point. Starts coordinator without auth.
/// Do NOT use in production — stripped from release builds.
#[doc(hidden)]
#[cfg(debug_assertions)]
pub async fn start_testing(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    // Tests run without auth by default
    #[cfg(feature = "tracing")]
    let span_store: SpanStore = None;
    #[cfg(not(feature = "tracing"))]
    let span_store: SpanStore = ();
    start_with_events(
        bind,
        external_events,
        futures::stream::empty(),
        store,
        None,
        span_store,
    )
    .await
}

async fn start_with_events(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
    extra_events: impl Stream<Item = Event> + Unpin,
    store: Arc<dyn CoordinatorStore>,
    auth_token: Option<adora_message::auth::AuthToken>,
    span_store: SpanStore,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let clock = Arc::new(HLC::default());
    let artifact_store =
        Arc::new(artifacts::ArtifactStore::new().wrap_err("failed to create artifact store")?);

    let mut tasks = FuturesUnordered::new();

    // Setup WS event channel (used by axum WS handlers)
    let (ws_event_tx, ws_event_rx) = tokio::sync::mpsc::channel::<Event>(64);
    let ws_events = ReceiverStream::new(ws_event_rx);

    // Start WS server
    #[cfg(feature = "prometheus")]
    let prom_metrics = prometheus_metrics::new_shared();

    let (port, ws_shutdown, ws_future) = ws_server::serve(
        bind,
        ws_event_tx.clone(),
        clock.clone(),
        auth_token,
        artifact_store,
        #[cfg(feature = "prometheus")]
        prom_metrics.clone(),
    )
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
        start_inner(
            events,
            clock,
            store,
            span_store,
            #[cfg(feature = "prometheus")]
            prom_metrics,
        )
        .await?;

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
    store: Arc<dyn CoordinatorStore>,
    span_store: SpanStore,
    #[cfg(feature = "prometheus")] prom_metrics: prometheus_metrics::SharedMetrics,
) -> eyre::Result<()> {
    let daemon_heartbeat_interval =
        tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(Duration::from_secs(3)))
            .map(|_| Event::DaemonHeartbeatInterval);

    // events that should be aborted on `adora down`
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
    let mut persist_failure_count: u64 = 0;

    // Clear stale daemon records -- connections cannot survive a coordinator restart.
    match store.list_daemons() {
        Ok(daemons) => {
            for info in &daemons {
                if let Err(e) = store.unregister_daemon(&info.daemon_id) {
                    tracing::warn!("failed to clear stale daemon record: {e}");
                }
            }
            if !daemons.is_empty() {
                tracing::info!("cleared {} stale daemon records", daemons.len());
            }
        }
        Err(e) => {
            tracing::warn!("failed to read persisted daemons on startup: {e}");
        }
    }

    // Recover persisted state: mark any previously-running dataflows as failed
    // (full reconciliation with daemons is Phase 2 work).
    match store.list_dataflows() {
        Ok(records) => {
            for mut record in records {
                match record.status {
                    StoreDataflowStatus::Pending
                    | StoreDataflowStatus::Running
                    | StoreDataflowStatus::Stopping => {
                        tracing::info!(
                            "recovering stale dataflow {} ({:?}) -> marking as Failed",
                            record.uuid,
                            record.name
                        );
                        record.status = StoreDataflowStatus::Failed {
                            error: "coordinator restarted".into(),
                        };
                        record.generation += 1;
                        record.updated_at = state::now_millis();
                        if let Err(e) = store.put_dataflow(&record) {
                            persist_failure_count += 1;
                            tracing::warn!("failed to update stale dataflow record: {e}");
                        }
                    }
                    StoreDataflowStatus::Succeeded | StoreDataflowStatus::Failed { .. } => {}
                }
            }
        }
        Err(e) => {
            tracing::warn!("failed to read persisted dataflows on startup: {e}");
        }
    }

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
                    labels,
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
                            if let Err(e) =
                                store.register_daemon(adora_coordinator_store::DaemonInfo {
                                    daemon_id: daemon_id.clone(),
                                    machine_id: daemon_id.machine_id().map(|s| s.to_owned()),
                                    labels,
                                })
                            {
                                persist_failure_count += 1;
                                tracing::warn!("failed to persist daemon registration: {e}");
                            }
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
                                // Persist: dataflow finished
                                let final_status =
                                    if let Some(results) = dataflow_results.get(&uuid) {
                                        let errors: Vec<String> = results
                                            .values()
                                            .flat_map(|dr| dr.node_results.iter())
                                            .filter_map(|(node_id, r)| {
                                                r.as_ref().err().map(|e| format!("{node_id}: {e}"))
                                            })
                                            .collect();
                                        if errors.is_empty() {
                                            StoreDataflowStatus::Succeeded
                                        } else {
                                            StoreDataflowStatus::Failed {
                                                error: errors.join("; "),
                                            }
                                        }
                                    } else {
                                        StoreDataflowStatus::Succeeded
                                    };
                                if let Err(e) = finished_dataflow
                                    .make_record(final_status)
                                    .and_then(|r| store.put_dataflow(&r))
                                {
                                    persist_failure_count += 1;
                                    tracing::warn!("failed to persist dataflow finish: {e}");
                                }

                                for sender in finished_dataflow.stop_reply_senders {
                                    let _ = sender.send(Ok(reply.clone()));
                                }
                                // If WaitForSpawn waiters are still pending, notify them
                                // that the dataflow finished before spawn completed (e.g.,
                                // a node crashed at startup or the build failed).
                                if !matches!(
                                    finished_dataflow.spawn_result,
                                    CachedResult::Cached { .. }
                                ) {
                                    let node_errors: Vec<String> = dataflow_results
                                        .get(&uuid)
                                        .into_iter()
                                        .flat_map(|r| r.values())
                                        .flat_map(|dr| dr.node_results.iter())
                                        .filter_map(|(node_id, r)| {
                                            r.as_ref().err().map(|e| format!("{node_id}: {e}"))
                                        })
                                        .collect();
                                    let msg = if node_errors.is_empty() {
                                        "dataflow exited before spawn completed".to_string()
                                    } else {
                                        format!(
                                            "dataflow failed to start:\n  {}",
                                            node_errors.join("\n  ")
                                        )
                                    };
                                    finished_dataflow.spawn_result.set_result(Err(eyre!(msg)));
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
                                Ok(mut dataflow) => {
                                    let uuid = dataflow.uuid;
                                    // Persist: dataflow started
                                    if let Err(e) = dataflow
                                        .make_record(StoreDataflowStatus::Pending)
                                        .and_then(|r| store.put_dataflow(&r))
                                    {
                                        persist_failure_count += 1;
                                        tracing::warn!("failed to persist dataflow start: {e}");
                                    }
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
                        ControlRequest::RestartNode {
                            dataflow_id,
                            node_id,
                            grace_duration,
                        } => {
                            let result = restart_node(
                                &running_dataflows,
                                dataflow_id,
                                node_id.clone(),
                                grace_duration,
                                &mut daemon_connections,
                                clock.new_timestamp(),
                            )
                            .await;
                            let reply = result.map(|()| ControlRequestReply::NodeRestarted {
                                dataflow_id,
                                node_id,
                            });
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::StopNode {
                            dataflow_id,
                            node_id,
                            grace_duration,
                        } => {
                            let result = stop_node(
                                &running_dataflows,
                                dataflow_id,
                                node_id.clone(),
                                grace_duration,
                                &mut daemon_connections,
                                clock.new_timestamp(),
                            )
                            .await;
                            let reply = result.map(|()| ControlRequestReply::NodeStopped {
                                dataflow_id,
                                node_id,
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
                                    // Persist: dataflow stopping
                                    if let Err(e) = dataflow
                                        .make_record(StoreDataflowStatus::Stopping)
                                        .and_then(|r| store.put_dataflow(&r))
                                    {
                                        persist_failure_count += 1;
                                        tracing::warn!("failed to persist dataflow stopping: {e}");
                                    }
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
                                        // Persist: dataflow stopping
                                        if let Err(e) = dataflow
                                            .make_record(StoreDataflowStatus::Stopping)
                                            .and_then(|r| store.put_dataflow(&r))
                                        {
                                            persist_failure_count += 1;
                                            tracing::warn!(
                                                "failed to persist dataflow stopping: {e}"
                                            );
                                        }
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
                        ControlRequest::Restart {
                            dataflow_uuid,
                            grace_duration,
                            force,
                        } => {
                            let result = restart_dataflow(
                                dataflow_uuid,
                                grace_duration,
                                force,
                                &mut running_dataflows,
                                &mut daemon_connections,
                                &clock,
                                store.as_ref(),
                                &mut persist_failure_count,
                            )
                            .await;
                            let _ = reply_sender.send(result);
                        }
                        ControlRequest::RestartByName {
                            name,
                            grace_duration,
                            force,
                        } => match resolve_name(name, &running_dataflows, &archived_dataflows) {
                            Ok(dataflow_uuid) => {
                                let result = restart_dataflow(
                                    dataflow_uuid,
                                    grace_duration,
                                    force,
                                    &mut running_dataflows,
                                    &mut daemon_connections,
                                    &clock,
                                    store.as_ref(),
                                    &mut persist_failure_count,
                                )
                                .await;
                                let _ = reply_sender.send(result);
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
                                store.as_ref(),
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
                                    ft_stats: conn.ft_stats.clone(),
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
                        ControlRequest::TopicSubscribe { .. } => {
                            let _ = reply_sender.send(Err(eyre::eyre!(
                                "TopicSubscribe request should be handled separately"
                            )));
                        }
                        ControlRequest::TopicUnsubscribe { .. } => {
                            let _ = reply_sender.send(Err(eyre::eyre!(
                                "TopicUnsubscribe request should be handled separately"
                            )));
                        }
                        ControlRequest::TopicPublish { .. } => {
                            let _ = reply_sender.send(Err(eyre::eyre!(
                                "TopicPublish request should be handled separately"
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
                                                restart_count: m.restart_count,
                                                broken_inputs: m.broken_inputs.clone(),
                                                status: m.status.clone(),
                                                pending_messages: m.pending_messages,
                                            }
                                        });

                                        node_infos.push(NodeInfo {
                                            dataflow_id: dataflow.uuid,
                                            dataflow_name: dataflow.name.clone(),
                                            node_id: node_id.clone(),
                                            daemon_id: daemon_id.clone(),
                                            metrics,
                                            network: dataflow.network_metrics.clone(),
                                        });
                                    }
                                }
                            }
                            let _ = reply_sender
                                .send(Ok(ControlRequestReply::NodeInfoList(node_infos)));
                        }
                        ControlRequest::GetTraces => {
                            let reply = handle_get_traces(&span_store);
                            let _ = reply_sender.send(Ok(reply));
                        }
                        ControlRequest::GetTraceSpans { trace_id } => {
                            let reply = if trace_id.len() <= 36 && trace_id.is_ascii() {
                                handle_get_trace_spans(&span_store, &trace_id)
                            } else {
                                ControlRequestReply::Error("invalid trace_id format".to_string())
                            };
                            let _ = reply_sender.send(Ok(reply));
                        }
                        ControlRequest::GetParams {
                            dataflow_id,
                            node_id,
                        } => {
                            let reply = match store.list_node_params(&dataflow_id, &node_id) {
                                Ok(params) => {
                                    let params: Vec<_> = params
                                        .into_iter()
                                        .filter_map(|(k, v)| {
                                            serde_json::from_slice(&v).ok().map(|val| (k, val))
                                        })
                                        .collect();
                                    Ok(ControlRequestReply::ParamList { params })
                                }
                                Err(e) => Err(e),
                            };
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::GetParam {
                            dataflow_id,
                            node_id,
                            key,
                        } => {
                            let reply = match store.get_node_param(&dataflow_id, &node_id, &key) {
                                Ok(Some(bytes)) => match serde_json::from_slice(&bytes) {
                                    Ok(value) => Ok(ControlRequestReply::ParamValue { key, value }),
                                    Err(e) => Err(eyre::eyre!("corrupt param value: {e}")),
                                },
                                Ok(None) => Err(eyre::eyre!("param not found: {key}")),
                                Err(e) => Err(e),
                            };
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::SetParam {
                            dataflow_id,
                            node_id,
                            key,
                            value,
                        } => {
                            let reply = match serde_json::to_vec(&value) {
                                Ok(bytes) => {
                                    match store.put_node_param(&dataflow_id, &node_id, &key, &bytes)
                                    {
                                        Ok(()) => {
                                            // Best-effort forward to daemon if the node is running
                                            if let Some(daemon_id) = running_dataflows
                                                .get(&dataflow_id)
                                                .and_then(|df| df.node_to_daemon.get(&node_id))
                                            {
                                                if let Ok(msg) = serde_json::to_vec(&Timestamped {
                                                    inner: DaemonCoordinatorEvent::SetParam {
                                                        dataflow_id,
                                                        node_id: node_id.clone(),
                                                        key: key.clone(),
                                                        value: value.clone(),
                                                    },
                                                    timestamp: clock.new_timestamp(),
                                                }) {
                                                    if let Some(conn) =
                                                        daemon_connections.get_mut(daemon_id)
                                                    {
                                                        if let Err(e) =
                                                            conn.send_and_receive(&msg).await
                                                        {
                                                            tracing::warn!(
                                                                %node_id,
                                                                "param stored but daemon delivery failed: {e}"
                                                            );
                                                        }
                                                    }
                                                }
                                            }
                                            Ok(ControlRequestReply::ParamSet)
                                        }
                                        Err(e) => Err(e),
                                    }
                                }
                                Err(e) => Err(eyre!("failed to serialize param value: {e}")),
                            };
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::DeleteParam {
                            dataflow_id,
                            node_id,
                            key,
                        } => {
                            let reply = store
                                .delete_node_param(&dataflow_id, &node_id, &key)
                                .map(|()| ControlRequestReply::ParamDeleted);
                            let _ = reply_sender.send(reply);
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
                ControlEvent::TopicSubscribe {
                    dataflow_id,
                    topics,
                    found_tx,
                } => {
                    let found = running_dataflows.get(&dataflow_id).is_some_and(|df| {
                        if !df.descriptor.debug.publish_all_messages_to_zenoh {
                            return false;
                        }
                        // Validate each requested topic exists in the descriptor
                        topics.iter().all(|(node_id, data_id)| {
                            df.descriptor
                                .nodes
                                .iter()
                                .any(|node| node.id == *node_id && node.outputs.contains(data_id))
                        })
                    });
                    let _ = found_tx.send(found);
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
                        if let Err(e) = store.unregister_daemon(machine_id) {
                            persist_failure_count += 1;
                            tracing::warn!("failed to persist daemon unregistration: {e}");
                        }
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
                if persist_failure_count > 0 {
                    tracing::warn!(
                        persist_failures = persist_failure_count,
                        "store persistence failures since startup"
                    );
                }
            }
            Event::CtrlC => {
                tracing::info!("Destroying coordinator after receiving Ctrl-C signal");
                handle_destroy(
                    &mut running_dataflows,
                    &mut daemon_connections,
                    &abort_handle,
                    &clock,
                    store.as_ref(),
                )
                .await?;
            }
            Event::DaemonHeartbeat {
                daemon_id: machine_id,
                ft_stats,
            } => {
                if let Some(connection) = daemon_connections.get_mut(&machine_id) {
                    connection.last_heartbeat = Instant::now();
                    if let Some(stats) = ft_stats {
                        connection.ft_stats = Some(stats);
                    }
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
                if let Err(e) = store.unregister_daemon(&daemon_id) {
                    persist_failure_count += 1;
                    tracing::warn!("failed to persist daemon unregistration: {e}");
                }
            }
            Event::NodeMetrics {
                dataflow_id,
                metrics,
                network,
            } => {
                // Store metrics for this dataflow
                if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                    #[cfg(feature = "prometheus")]
                    let df_name = dataflow.name.as_deref().unwrap_or("");
                    #[cfg(feature = "prometheus")]
                    let df_id = dataflow_id.to_string();
                    for (node_id, node_metrics) in &metrics {
                        dataflow
                            .node_metrics
                            .insert(node_id.clone(), node_metrics.clone());
                    }
                    if let Some(net) = network {
                        dataflow.network_metrics = Some(net);
                    }

                    #[cfg(feature = "prometheus")]
                    {
                        use crate::prometheus_metrics::sanitize_prom_label;
                        let m = prom_metrics.lock().unwrap_or_else(|e| e.into_inner());
                        for (node_id, node_metrics) in &metrics {
                            let nid = sanitize_prom_label(node_id.as_ref());
                            let daemon = sanitize_prom_label(
                                &dataflow
                                    .node_to_daemon
                                    .get(node_id)
                                    .map(|d| d.to_string())
                                    .unwrap_or_default(),
                            );
                            m.node_cpu
                                .with_label_values(&[&df_id, &nid, &daemon])
                                .set(node_metrics.cpu_usage as f64);
                            m.node_memory
                                .with_label_values(&[&df_id, &nid, &daemon])
                                .set(node_metrics.memory_bytes as i64);
                            m.node_pending
                                .with_label_values(&[&df_id, &nid, &daemon])
                                .set(node_metrics.pending_messages as i64);
                            m.node_restarts
                                .with_label_values(&[&df_id, &nid, &daemon])
                                .set(node_metrics.restart_count as i64);
                        }
                        let df_name_s = sanitize_prom_label(df_name);
                        m.dataflow_nodes
                            .with_label_values(&[&df_id, &df_name_s])
                            .set(dataflow.nodes.len() as i64);
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
                            tracing::error!("build error for {build_id}: {err:?}");
                            build.errors.push(format!("{err}"));
                        }
                    };
                    if build.pending_build_results.is_empty() {
                        tracing::info!("dataflow build finished: `{build_id}`");
                        let Some(mut build) = running_builds.remove(&build_id) else {
                            tracing::error!("build {build_id} disappeared from running_builds");
                            continue;
                        };
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
                                // Persist: dataflow now running
                                if let Err(e) = dataflow
                                    .make_record(StoreDataflowStatus::Running)
                                    .and_then(|r| store.put_dataflow(&r))
                                {
                                    persist_failure_count += 1;
                                    tracing::warn!("failed to persist dataflow running: {e}");
                                }
                            }
                        }
                        Err(err) => {
                            tracing::warn!("error while spawning dataflow `{dataflow_id}`");
                            // Persist: spawn failed
                            if let Err(e) = dataflow
                                .make_record(StoreDataflowStatus::Failed {
                                    error: format!("spawn failed: {err}"),
                                })
                                .and_then(|r| store.put_dataflow(&r))
                            {
                                persist_failure_count += 1;
                                tracing::warn!("failed to persist dataflow spawn failure: {e}");
                            }
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
            Event::DaemonStatusReport {
                daemon_id,
                running_dataflows: reported_dataflows,
            } => {
                tracing::info!(
                    "daemon {daemon_id} reports {} running dataflow(s)",
                    reported_dataflows.len()
                );
                // Reconcile: if daemon reports a dataflow as running and it exists in
                // the store as Pending/Failed, update it to Running.
                for df_id in &reported_dataflows {
                    match store.get_dataflow(df_id) {
                        Ok(Some(mut record)) => match record.status {
                            StoreDataflowStatus::Failed { .. } | StoreDataflowStatus::Pending => {
                                tracing::info!(
                                    "reconciling dataflow {df_id}: {:?} -> Running (daemon reports active)",
                                    record.status
                                );
                                record.status = StoreDataflowStatus::Running;
                                record.generation += 1;
                                record.updated_at = state::now_millis();
                                if let Err(e) = store.put_dataflow(&record) {
                                    persist_failure_count += 1;
                                    tracing::warn!("failed to reconcile dataflow {df_id}: {e}");
                                }
                            }
                            _ => {}
                        },
                        Ok(None) => {
                            tracing::warn!(
                                "daemon reports dataflow {df_id} running, but not found in store"
                            );
                        }
                        Err(e) => {
                            tracing::warn!(
                                "failed to look up dataflow {df_id} for reconciliation: {e}"
                            );
                        }
                    }
                }

                // Auto-recovery: find dataflows that should be running on this daemon
                // but aren't reported. Uses 30s backoff per daemon per dataflow to
                // avoid infinite re-spawn loops for crash-looping nodes.
                const RECOVERY_BACKOFF: Duration = Duration::from_secs(30);
                let reported_set: BTreeSet<DataflowId> =
                    reported_dataflows.iter().copied().collect();
                let now = Instant::now();
                for (uuid, df) in &mut running_dataflows {
                    if !df.daemons.contains(&daemon_id) {
                        continue;
                    }
                    if reported_set.contains(uuid) {
                        // Dataflow is running — clear any previous recovery timestamp
                        df.last_recovery_attempt.remove(&daemon_id);
                        continue;
                    }
                    // Backoff: skip if we attempted recovery recently
                    if let Some(last) = df.last_recovery_attempt.get(&daemon_id) {
                        if now.duration_since(*last) < RECOVERY_BACKOFF {
                            continue;
                        }
                    }
                    // Collect nodes assigned to this daemon
                    let spawn_nodes: BTreeSet<_> = df
                        .node_to_daemon
                        .iter()
                        .filter(|(_, did)| **did == daemon_id)
                        .map(|(nid, _)| nid.clone())
                        .collect();
                    if spawn_nodes.is_empty() {
                        continue;
                    }
                    df.last_recovery_attempt.insert(daemon_id.clone(), now);
                    tracing::info!(
                        "auto-recovery: re-spawning {} node(s) for dataflow {uuid} on daemon {daemon_id}",
                        spawn_nodes.len()
                    );
                    let spawn_command = adora_message::coordinator_to_daemon::SpawnDataflowNodes {
                        build_id: None,
                        session_id: adora_message::SessionId::generate(),
                        dataflow_id: *uuid,
                        local_working_dir: None,
                        nodes: df.nodes.clone(),
                        dataflow_descriptor: df.descriptor.clone(),
                        spawn_nodes,
                        uv: false,
                        write_events_to: None,
                        artifact_base_url: None,
                    };
                    let message = match serde_json::to_vec(&Timestamped {
                        inner: DaemonCoordinatorEvent::Spawn(spawn_command),
                        timestamp: clock.new_timestamp(),
                    }) {
                        Ok(m) => m,
                        Err(e) => {
                            tracing::warn!("failed to serialize re-spawn command: {e}");
                            continue;
                        }
                    };
                    if let Some(conn) = daemon_connections.get_mut(&daemon_id) {
                        if let Err(e) = conn.send(&message).await {
                            tracing::warn!("failed to send re-spawn to daemon {daemon_id}: {e}");
                        }
                    }
                }
            }
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

#[cfg(feature = "tracing")]
fn handle_get_traces(span_store: &SpanStore) -> ControlRequestReply {
    use adora_message::coordinator_to_cli::TraceSummary;
    use std::collections::HashMap;

    let Some(store) = span_store else {
        return ControlRequestReply::TraceList(Vec::new());
    };

    // Snapshot spans under the lock, then release immediately.
    let records: Vec<_> = match store.lock() {
        Ok(store) => store.spans().iter().cloned().collect(),
        Err(e) => {
            tracing::warn!("span store mutex poisoned: {e}");
            return ControlRequestReply::TraceList(Vec::new());
        }
    };

    // Group spans by trace_id.
    let mut groups: HashMap<&str, Vec<&adora_tracing::span_store::SpanRecord>> = HashMap::new();
    for span in &records {
        groups.entry(&span.trace_id).or_default().push(span);
    }

    let mut summaries: Vec<TraceSummary> = groups
        .into_iter()
        .map(|(trace_id, spans)| {
            let root = spans
                .iter()
                .find(|s| s.parent_span_id.is_none())
                .unwrap_or(&spans[0]);
            let start_time = spans.iter().map(|s| s.start_time).min().unwrap_or(0);
            TraceSummary {
                trace_id: trace_id.to_string(),
                root_span_name: root.name.clone(),
                span_count: spans.len(),
                start_time,
                total_duration_us: root.duration_us,
            }
        })
        .collect();

    // Newest first.
    summaries.sort_by(|a, b| b.start_time.cmp(&a.start_time));
    ControlRequestReply::TraceList(summaries)
}

#[cfg(not(feature = "tracing"))]
fn handle_get_traces(_span_store: &SpanStore) -> ControlRequestReply {
    ControlRequestReply::TraceList(Vec::new())
}

#[cfg(feature = "tracing")]
fn handle_get_trace_spans(span_store: &SpanStore, trace_id: &str) -> ControlRequestReply {
    use adora_message::coordinator_to_cli::TraceSpan;

    let Some(store) = span_store else {
        return ControlRequestReply::TraceSpans(Vec::new());
    };

    // Snapshot matching spans under the lock, then release immediately.
    let records: Vec<_> = match store.lock() {
        Ok(store) => store
            .spans()
            .iter()
            .filter(|s| s.trace_id == trace_id)
            .cloned()
            .collect(),
        Err(e) => {
            tracing::warn!("span store mutex poisoned: {e}");
            return ControlRequestReply::TraceSpans(Vec::new());
        }
    };

    let spans: Vec<TraceSpan> = records
        .into_iter()
        .map(|s| TraceSpan {
            trace_id: s.trace_id,
            span_id: s.span_id,
            parent_span_id: s.parent_span_id,
            name: s.name,
            target: s.target,
            level: s.level,
            start_time: s.start_time,
            duration_us: s.duration_us,
            fields: s.fields,
        })
        .collect();

    ControlRequestReply::TraceSpans(spans)
}

#[cfg(not(feature = "tracing"))]
fn handle_get_trace_spans(_span_store: &SpanStore, _trace_id: &str) -> ControlRequestReply {
    ControlRequestReply::TraceSpans(Vec::new())
}

/// Restart a running dataflow: stop it, then re-start with the stored descriptor.
#[allow(clippy::too_many_arguments)]
async fn restart_dataflow(
    dataflow_uuid: uuid::Uuid,
    grace_duration: Option<Duration>,
    force: bool,
    running_dataflows: &mut HashMap<uuid::Uuid, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
    store: &dyn CoordinatorStore,
    persist_failure_count: &mut u64,
) -> eyre::Result<ControlRequestReply> {
    // 1. Extract descriptor, name, and uv from the running dataflow
    let (descriptor, name, uv) = {
        let df = running_dataflows
            .get(&dataflow_uuid)
            .ok_or_else(|| eyre!("no running dataflow with UUID `{dataflow_uuid}`"))?;
        (df.descriptor.clone(), df.name.clone(), df.uv)
    };

    // 2. Stop the old dataflow (scoped borrow so it drops before start_dataflow)
    {
        let dataflow = stop_dataflow(
            running_dataflows,
            dataflow_uuid,
            daemon_connections,
            clock.new_timestamp(),
            grace_duration,
            force,
        )
        .await?;

        // Persist: dataflow stopping
        if let Err(e) = dataflow
            .make_record(StoreDataflowStatus::Stopping)
            .and_then(|r| store.put_dataflow(&r))
        {
            *persist_failure_count += 1;
            tracing::warn!("failed to persist dataflow stopping: {e}");
        }
    }

    // 3. Start a new dataflow with the stored descriptor
    let new_dataflow = start_dataflow(
        None, // no build_id for restart
        adora_message::SessionId::generate(),
        descriptor,
        None, // no local_working_dir for restart
        name,
        daemon_connections,
        clock,
        uv,
        None, // no write_events_to
    )
    .await?;

    let new_uuid = new_dataflow.uuid;

    // 4. Only now remove old dataflow (after new one started successfully)
    running_dataflows.remove(&dataflow_uuid);

    // Persist: new dataflow started
    let mut new_df = new_dataflow;
    if let Err(e) = new_df
        .make_record(StoreDataflowStatus::Pending)
        .and_then(|r| store.put_dataflow(&r))
    {
        *persist_failure_count += 1;
        tracing::warn!("failed to persist restarted dataflow: {e}");
    }
    running_dataflows.insert(new_uuid, new_df);

    Ok(ControlRequestReply::DataflowRestarted {
        old_uuid: dataflow_uuid,
        new_uuid,
    })
}
