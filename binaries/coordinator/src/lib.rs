use crate::{
    events::set_up_ctrlc_handler,
    handlers::{
        build_dataflow, dataflow_result, handle_destroy, reload_dataflow, resolve_name,
        restart_node, retrieve_logs, send_heartbeat_message, send_log_message, send_topic_frames,
        start_dataflow, stop_dataflow, stop_node,
    },
    state::{ArchivedDataflow, CachedResult, ParamTarget, RunningBuild, RunningDataflow},
};
pub use control::ControlEvent;
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
pub use dora_coordinator_store::{self, CoordinatorStore, InMemoryStore};
use dora_core::{descriptor::DescriptorExt, uhlc::HLC};
use dora_message::{
    BuildId, DataflowId,
    cli_to_coordinator::ControlRequest,
    common::DaemonId,
    coordinator_to_cli::{
        ControlRequestReply, DataflowIdAndName, DataflowList, DataflowListEntry, DataflowResult,
        DataflowStatus, LogLevel, LogMessage,
    },
    coordinator_to_daemon::{
        DaemonCoordinatorEvent, RegisterResult, StateCatchUpOperation, Timestamped,
    },
    daemon_to_coordinator::{DaemonCoordinatorReply, DataflowDaemonResult},
};
pub use events::{DaemonRequest, DataflowEvent, Event};
use eyre::{ContextCompat, Result, WrapErr, bail, eyre};
use futures::{Future, Stream, StreamExt, future::join_all, stream::FuturesUnordered};
use futures_concurrency::stream::Merge;
use indexmap::IndexMap;
use log_subscriber::LogSubscriber;
use petname::petname;
use serde::Serialize;
use serde_json::value::RawValue;
pub(crate) use state::{DaemonConnections, resolve_param_target};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::SocketAddr,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;

const FALLBACK_REPLAY_BACKOFF: Duration = Duration::from_secs(5);

pub(crate) mod artifacts;
mod control;
mod events;
mod handlers;
mod log_subscriber;
#[cfg(feature = "metrics")]
mod otel_metrics;
mod run;
mod state;
mod topic_subscriber;
mod ws_control;
mod ws_daemon;
mod ws_server;

/// Type alias for the optional in-memory span store.
///
/// When `Some`, the coordinator will serve `GetTraces` / `GetTraceSpans`
/// requests by reading captured spans from this store.
#[cfg(feature = "tracing")]
pub type SpanStore = Option<dora_tracing::span_store::SharedSpanStore>;
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
/// writes it to `~/.config/dora/.dora-token`, and requires all clients to
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
        let token = dora_message::auth::generate_token();
        if let Ok(cwd) = std::env::current_dir() {
            if let Err(e) = dora_message::auth::write_token(&cwd, &token) {
                tracing::warn!("failed to write auth token: {e}");
            } else {
                tracing::info!(
                    "auth token written to {}",
                    dora_message::auth::token_path(&cwd).display()
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
/// For embedding the coordinator in another process (e.g., `dora run`).
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
/// Do NOT use in production.
#[doc(hidden)]
pub async fn start_testing(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    start_testing_with_store(bind, external_events, store).await
}

/// Like [`start_testing`], but allows injecting a custom store.
#[doc(hidden)]
pub async fn start_testing_with_store(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
    store: Arc<dyn CoordinatorStore>,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
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
    auth_token: Option<dora_message::auth::AuthToken>,
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
    #[cfg(feature = "metrics")]
    let _meter_provider = {
        let provider = dora_metrics::init_metrics();
        opentelemetry::global::set_meter_provider(provider.clone());
        provider
    };
    #[cfg(feature = "metrics")]
    let otel_metrics = otel_metrics::new_shared();

    let (port, ws_shutdown, ws_future) = ws_server::serve(
        bind,
        ws_event_tx.clone(),
        clock.clone(),
        auth_token,
        artifact_store,
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
            #[cfg(feature = "metrics")]
            otel_metrics,
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
    #[cfg(feature = "metrics")] otel_metrics: otel_metrics::SharedMetrics,
) -> eyre::Result<()> {
    let daemon_heartbeat_interval =
        tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(Duration::from_secs(3)))
            .map(|_| Event::DaemonHeartbeatInterval);

    // events that should be aborted on `dora down`
    let (abortable_events, abort_handle) =
        futures::stream::abortable((events, daemon_heartbeat_interval).merge());

    let mut events = abortable_events;

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    let mut dataflow_results: HashMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        HashMap::new();
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut daemon_connections = DaemonConnections::default();

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
                    | StoreDataflowStatus::Stopping
                    | StoreDataflowStatus::Recovering => {
                        // Mark as Recovering instead of Failed — give daemons
                        // 60s to reconnect and report their running state.
                        // Dataflows that are not reclaimed transition to Failed
                        // via the recovery timeout in the event loop.
                        tracing::info!(
                            "coordinator restarted: dataflow {} ({:?}) -> Recovering \
                             (waiting for daemon reconnect)",
                            record.uuid,
                            record.name
                        );
                        record.status = StoreDataflowStatus::Recovering;
                        record.generation += 1;
                        record.updated_at = state::now_millis();
                        if let Err(e) = store.put_dataflow(&record) {
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
                    connection,
                    version_check_result,
                    daemon_id_tx,
                } => {
                    let existing = match &machine_id {
                        Some(id) => daemon_connections.get_matching_daemon_id(id),
                        None => daemon_connections.unnamed().next(),
                    };
                    // Allow re-registration: if a daemon with the same machine_id
                    // reconnects (e.g., after coordinator restart), replace the old
                    // connection. DaemonConnections::add() handles this.
                    if existing.is_some() {
                        tracing::info!(
                            ?machine_id,
                            "daemon re-registering (replacing stale connection)"
                        );
                    }
                    // Reuse existing DaemonId if daemon is re-registering (#5 fix)
                    let daemon_id = match existing {
                        Some(existing_id) => existing_id.clone(),
                        None => DaemonId::new(machine_id),
                    };

                    let reply: Timestamped<RegisterResult> = Timestamped {
                        inner: match version_check_result.as_ref() {
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
                    match version_check_result.map_err(|e| eyre!(e)).and(send_result) {
                        Ok(()) => {
                            let _ = daemon_id_tx.send(daemon_id.clone());
                            daemon_connections.add(daemon_id.clone(), connection);
                            if let Err(e) =
                                store.register_daemon(dora_coordinator_store::DaemonInfo {
                                    daemon_id: daemon_id.clone(),
                                    machine_id: daemon_id.machine_id().map(|s| s.to_owned()),
                                    labels,
                                })
                            {
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

                                schedule_param_replay_for_ready_dataflow(
                                    uuid,
                                    dataflow,
                                    &mut daemon_connections,
                                    store.clone(),
                                    clock.clone(),
                                );
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
                                // Archive finished dataflow (cap at 200 to prevent unbounded growth)
                                archived_dataflows
                                    .entry(uuid)
                                    .or_insert_with(|| ArchivedDataflow::from(entry.get()));
                                const MAX_ARCHIVED_DATAFLOWS: usize = 200;
                                while archived_dataflows.len() > MAX_ARCHIVED_DATAFLOWS {
                                    archived_dataflows.shift_remove_index(0);
                                }
                                let mut finished_dataflow = entry.remove();
                                close_topic_subscribers_on_finish(&mut finished_dataflow);
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
                    match *request {
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
                                .map(|(id, conn)| dora_message::coordinator_to_cli::DaemonInfo {
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
                            use dora_message::coordinator_to_cli::{NodeInfo, NodeMetricsInfo};

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
                            let reply = match resolve_param_target(
                                &running_dataflows,
                                store.as_ref(),
                                &dataflow_id,
                                &node_id,
                            ) {
                                Err(e) => Err(e),
                                Ok(_) => match store.list_node_params(&dataflow_id, &node_id) {
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
                                },
                            };
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::GetParam {
                            dataflow_id,
                            node_id,
                            key,
                        } => {
                            let reply = match resolve_param_target(
                                &running_dataflows,
                                store.as_ref(),
                                &dataflow_id,
                                &node_id,
                            ) {
                                Err(e) => Err(e),
                                Ok(_) => match store.get_node_param(&dataflow_id, &node_id, &key) {
                                    Ok(Some(bytes)) => match serde_json::from_slice(&bytes) {
                                        Ok(value) => {
                                            Ok(ControlRequestReply::ParamValue { key, value })
                                        }
                                        Err(e) => Err(eyre::eyre!("corrupt param value: {e}")),
                                    },
                                    Ok(None) => Err(eyre::eyre!("param not found: {key}")),
                                    Err(e) => Err(e),
                                },
                            };
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::SetParam {
                            dataflow_id,
                            node_id,
                            key,
                            value,
                        } => {
                            let reply = match resolve_param_target(
                                &running_dataflows,
                                store.as_ref(),
                                &dataflow_id,
                                &node_id,
                            ) {
                                Ok(target) => match serde_json::to_vec(&value) {
                                    Ok(bytes) => {
                                        match store.put_node_param(
                                            &dataflow_id,
                                            &node_id,
                                            &key,
                                            &bytes,
                                        ) {
                                            Ok(()) => {
                                                if let ParamTarget::Running { daemon_id } = target {
                                                    if let Some(df) =
                                                        running_dataflows.get_mut(&dataflow_id)
                                                    {
                                                        df.append_state_log(
                                                            StateCatchUpOperation::SetParam {
                                                                node_id: node_id.clone(),
                                                                key: key.clone(),
                                                                value: value.clone(),
                                                            },
                                                        );

                                                        if let Ok(msg) =
                                                            serde_json::to_vec(&Timestamped {
                                                                inner: DaemonCoordinatorEvent::SetParam {
                                                                    dataflow_id,
                                                                    node_id: node_id.clone(),
                                                                    key: key.clone(),
                                                                    value: value.clone(),
                                                                },
                                                                timestamp: clock.new_timestamp(),
                                                            })
                                                            && let Some(conn) =
                                                                daemon_connections.get_mut(&daemon_id)
                                                                && let Err(e) =
                                                                    conn.send_and_receive(&msg).await
                                                                {
                                                                    tracing::warn!(
                                                                        %node_id,
                                                                        %daemon_id,
                                                                        "param persisted in store; runtime forwarding is best-effort and failed: {e}"
                                                                    );
                                                                }
                                                    } else {
                                                        // Dataflow may have been removed after target resolution.
                                                        tracing::warn!(
                                                            %dataflow_id,
                                                            %node_id,
                                                            "param persisted in store; running dataflow disappeared before runtime forwarding"
                                                        );
                                                    }
                                                }
                                                Ok(ControlRequestReply::ParamSet)
                                            }
                                            Err(e) => Err(e),
                                        }
                                    }
                                    Err(e) => Err(eyre!("failed to serialize param value: {e}")),
                                },
                                Err(e) => Err(e),
                            };
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::DeleteParam {
                            dataflow_id,
                            node_id,
                            key,
                        } => {
                            let reply = match resolve_param_target(
                                &running_dataflows,
                                store.as_ref(),
                                &dataflow_id,
                                &node_id,
                            ) {
                                Ok(target) => {
                                    match store.delete_node_param(&dataflow_id, &node_id, &key) {
                                        Ok(()) => {
                                            if let ParamTarget::Running { daemon_id } = target {
                                                if let Some(df) =
                                                    running_dataflows.get_mut(&dataflow_id)
                                                {
                                                    df.append_state_log(
                                                        StateCatchUpOperation::DeleteParam {
                                                            node_id: node_id.clone(),
                                                            key: key.clone(),
                                                        },
                                                    );

                                                    if let Ok(msg) =
                                                        serde_json::to_vec(&Timestamped {
                                                            inner: DaemonCoordinatorEvent::DeleteParam {
                                                                dataflow_id,
                                                                node_id: node_id.clone(),
                                                                key: key.clone(),
                                                            },
                                                            timestamp: clock.new_timestamp(),
                                                        })
                                                        && let Some(conn) =
                                                            daemon_connections.get_mut(&daemon_id)
                                                            && let Err(e) =
                                                                conn.send_and_receive(&msg).await
                                                            {
                                                                tracing::warn!(
                                                                    %node_id,
                                                                    %daemon_id,
                                                                    "param deleted in store; runtime forwarding is best-effort and failed: {e}"
                                                                );
                                                            }
                                                } else {
                                                    // Dataflow may have been removed after target resolution.
                                                    tracing::warn!(
                                                        %dataflow_id,
                                                        %node_id,
                                                        "param deleted in store; running dataflow disappeared before runtime forwarding"
                                                    );
                                                }
                                            }
                                            Ok(ControlRequestReply::ParamDeleted)
                                        }
                                        Err(e) => Err(e),
                                    }
                                }
                                Err(e) => Err(e),
                            };
                            let _ = reply_sender.send(reply);
                        }
                        // --- Dynamic Topology ---
                        ControlRequest::AddNode { dataflow_id, node } => {
                            let result = match running_dataflows.get_mut(&dataflow_id) {
                                Some(dataflow) => {
                                    if dataflow.node_to_daemon.contains_key(&node.id) {
                                        Err(eyre!(
                                            "node '{}' already exists in dataflow {dataflow_id}",
                                            node.id
                                        ))
                                    } else {
                                        // Keep a clone of the original Node so
                                        // we can push it into the stored
                                        // descriptor after a successful spawn
                                        // (so `dora info` reflects the new
                                        // node).
                                        let original_node = node.clone();

                                        // Resolve the Node into a ResolvedNode via a
                                        // temporary single-node descriptor.
                                        let tmp_desc = dora_message::descriptor::Descriptor {
                                            nodes: vec![node],
                                            communication: Default::default(),
                                            deploy: None,
                                            debug: Default::default(),
                                            health_check_interval: None,
                                            strict_types: None,
                                            type_rules: Vec::new(),
                                            env: None,
                                        };
                                        match tmp_desc.resolve_aliases_and_set_defaults() {
                                            Ok(mut resolved_map) => {
                                                let (node_id, resolved_node) =
                                                    resolved_map.pop_first().ok_or_else(|| {
                                                        eyre!(
                                                            "node descriptor resolved to empty map"
                                                        )
                                                    })?;
                                                // Pick the first daemon (single-daemon case)
                                                // TODO: use machine label or load balancing for multi-daemon
                                                let daemon_id =
                                                    dataflow.daemons.iter().next().cloned();
                                                match daemon_id {
                                                    Some(did) => {
                                                        let msg =
                                                            serde_json::to_vec(&Timestamped {
                                                                inner:
                                                                    DaemonCoordinatorEvent::AddNode {
                                                                        dataflow_id,
                                                                        node: resolved_node.clone(),
                                                                        uv: dataflow.uv,
                                                                    },
                                                                timestamp: clock.new_timestamp(),
                                                            })?;
                                                        match daemon_connections.get_mut(&did) {
                                                            Some(conn) => {
                                                                match conn
                                                                    .send_and_receive(&msg)
                                                                    .await
                                                                {
                                                                    Ok(_) => {
                                                                        dataflow
                                                                            .node_to_daemon
                                                                            .insert(
                                                                                node_id.clone(),
                                                                                did,
                                                                            );
                                                                        // Update the stored descriptor
                                                                        // and resolved nodes so
                                                                        // `dora info` reflects the
                                                                        // new node.
                                                                        dataflow
                                                                            .descriptor
                                                                            .nodes
                                                                            .push(original_node);
                                                                        dataflow.nodes.insert(
                                                                            node_id.clone(),
                                                                            resolved_node,
                                                                        );
                                                                        Ok(
                                                                            ControlRequestReply::NodeAdded {
                                                                                dataflow_id,
                                                                                node_id,
                                                                            },
                                                                        )
                                                                    }
                                                                    Err(e) => Err(eyre!(
                                                                        "daemon dispatch failed: {e}"
                                                                    )),
                                                                }
                                                            }
                                                            None => Err(eyre!(
                                                                "no connection for daemon {did}"
                                                            )),
                                                        }
                                                    }
                                                    None => Err(eyre!(
                                                        "no daemons registered for dataflow {dataflow_id}"
                                                    )),
                                                }
                                            }
                                            Err(e) => Err(eyre!("failed to resolve node: {e}")),
                                        }
                                    }
                                }
                                None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
                            };
                            let _ = reply_sender.send(result);
                        }
                        ControlRequest::RemoveNode {
                            dataflow_id,
                            node_id,
                            grace_duration,
                        } => {
                            let result = match running_dataflows.get(&dataflow_id) {
                                Some(dataflow) => {
                                    match dataflow.node_to_daemon.get(&node_id) {
                                        Some(daemon_id) => {
                                            let msg = serde_json::to_vec(&Timestamped {
                                                inner: DaemonCoordinatorEvent::RemoveNode {
                                                    dataflow_id,
                                                    node_id: node_id.clone(),
                                                    grace_duration,
                                                },
                                                timestamp: clock.new_timestamp(),
                                            })?;
                                            match daemon_connections.get_mut(daemon_id) {
                                                Some(conn) => {
                                                    match conn.send_and_receive(&msg).await {
                                                        Ok(_) => {
                                                            // Clean up coordinator state
                                                            // (inverse of AddNode inserts)
                                                            if let Some(dataflow) =
                                                                running_dataflows
                                                                    .get_mut(&dataflow_id)
                                                            {
                                                                dataflow
                                                                    .node_to_daemon
                                                                    .remove(&node_id);
                                                                dataflow
                                                                    .descriptor
                                                                    .nodes
                                                                    .retain(|n| n.id != node_id);
                                                                dataflow.nodes.remove(&node_id);
                                                            }
                                                            Ok(ControlRequestReply::NodeRemoved {
                                                                dataflow_id,
                                                                node_id,
                                                            })
                                                        }
                                                        Err(e) => Err(eyre!(
                                                            "daemon dispatch failed: {e}"
                                                        )),
                                                    }
                                                }
                                                None => Err(eyre!(
                                                    "no connection for daemon {daemon_id}"
                                                )),
                                            }
                                        }
                                        None => Err(eyre!(
                                            "node '{node_id}' not found in dataflow {dataflow_id}"
                                        )),
                                    }
                                }
                                None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
                            };
                            let _ = reply_sender.send(result);
                        }
                        ControlRequest::AddMapping {
                            dataflow_id,
                            source_node,
                            source_output,
                            target_node,
                            target_input,
                        } => {
                            let result = match running_dataflows.get(&dataflow_id) {
                                Some(dataflow) => match dataflow.node_to_daemon.get(&target_node) {
                                    Some(daemon_id) => {
                                        let msg = serde_json::to_vec(&Timestamped {
                                            inner: DaemonCoordinatorEvent::AddMapping {
                                                dataflow_id,
                                                source_node: source_node.clone(),
                                                source_output: source_output.clone(),
                                                target_node: target_node.clone(),
                                                target_input: target_input.clone(),
                                            },
                                            timestamp: clock.new_timestamp(),
                                        })?;
                                        match daemon_connections.get_mut(daemon_id) {
                                            Some(conn) => match conn.send_and_receive(&msg).await {
                                                Ok(_) => Ok(ControlRequestReply::MappingAdded {
                                                    dataflow_id,
                                                    source_node,
                                                    source_output,
                                                    target_node,
                                                    target_input,
                                                }),
                                                Err(e) => Err(eyre!("daemon dispatch failed: {e}")),
                                            },
                                            None => {
                                                Err(eyre!("no connection for daemon {daemon_id}"))
                                            }
                                        }
                                    }
                                    None => Err(eyre!(
                                        "target node '{target_node}' not found in dataflow {dataflow_id}"
                                    )),
                                },
                                None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
                            };
                            let _ = reply_sender.send(result);
                        }
                        ControlRequest::RemoveMapping {
                            dataflow_id,
                            source_node,
                            source_output,
                            target_node,
                            target_input,
                        } => {
                            let result = match running_dataflows.get(&dataflow_id) {
                                Some(dataflow) => match dataflow.node_to_daemon.get(&target_node) {
                                    Some(daemon_id) => {
                                        let msg = serde_json::to_vec(&Timestamped {
                                            inner: DaemonCoordinatorEvent::RemoveMapping {
                                                dataflow_id,
                                                source_node: source_node.clone(),
                                                source_output: source_output.clone(),
                                                target_node: target_node.clone(),
                                                target_input: target_input.clone(),
                                            },
                                            timestamp: clock.new_timestamp(),
                                        })?;
                                        match daemon_connections.get_mut(daemon_id) {
                                            Some(conn) => match conn.send_and_receive(&msg).await {
                                                Ok(_) => Ok(ControlRequestReply::MappingRemoved {
                                                    dataflow_id,
                                                    source_node,
                                                    source_output,
                                                    target_node,
                                                    target_input,
                                                }),
                                                Err(e) => Err(eyre!("daemon dispatch failed: {e}")),
                                            },
                                            None => {
                                                Err(eyre!("no connection for daemon {daemon_id}"))
                                            }
                                        }
                                    }
                                    None => Err(eyre!(
                                        "target node '{target_node}' not found in dataflow {dataflow_id}"
                                    )),
                                },
                                None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
                            };
                            let _ = reply_sender.send(result);
                        }
                        ControlRequest::Hello { .. } => {
                            // Handled directly in ws_control.rs; never
                            // forwarded to the event loop. This arm exists
                            // only to keep the match exhaustive
                            // (dora-rs/adora#151).
                            let _ = reply_sender.send(Err(eyre!(
                                "Hello must be handled at the WS layer; \
                                 reaching the event loop is a bug"
                            )));
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
                    sender,
                    done_tx,
                } => {
                    let result = start_topic_debug_stream(
                        &mut running_dataflows,
                        &mut daemon_connections,
                        dataflow_id,
                        topics,
                        sender,
                        &clock,
                    )
                    .await
                    .map_err(|err| format!("{err:?}"));
                    let _ = done_tx.send(result);
                }
                ControlEvent::TopicCheck {
                    dataflow_id,
                    topics,
                    found_tx,
                } => {
                    let found = topic_outputs_by_daemon(&running_dataflows, dataflow_id, &topics)
                        .is_ok()
                        && topic_debug_enabled(&running_dataflows, dataflow_id).unwrap_or(false);
                    let _ = found_tx.send(found);
                }
                ControlEvent::TopicUnsubscribe {
                    subscription_id,
                    done_tx,
                } => {
                    if let Err(err) = stop_topic_debug_stream(
                        &mut running_dataflows,
                        &mut daemon_connections,
                        subscription_id,
                        &clock,
                    )
                    .await
                    {
                        tracing::warn!("failed to unsubscribe topic debug stream: {err:?}");
                    }
                    let _ = done_tx.send(());
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
                            tracing::warn!("failed to persist daemon unregistration: {e}");
                        }
                    }
                    // Clean up running_dataflows: remove disconnected daemon references
                    for (_uuid, df) in running_dataflows.iter_mut() {
                        for machine_id in &disconnected {
                            df.daemons.remove(machine_id);
                            df.pending_daemons.remove(machine_id);
                            df.pending_spawn_results.remove(machine_id);
                        }
                        if df.daemons.is_empty() {
                            tracing::error!(
                                dataflow = %df.uuid,
                                "all daemons disconnected — dataflow has no live daemons"
                            );
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
                // Recovery timeout: transition stale Recovering dataflows to Failed.
                // Dataflows are marked Recovering on coordinator startup and should
                // be reclaimed by reconnecting daemons within 60 seconds.
                const RECOVERY_TIMEOUT_SECS: u64 = 60;
                let now_ms = state::now_millis();
                match store.list_dataflows() {
                    Ok(records) => {
                        for mut record in records {
                            if record.status == StoreDataflowStatus::Recovering {
                                let age_ms = now_ms.saturating_sub(record.updated_at);
                                if age_ms > RECOVERY_TIMEOUT_SECS * 1000 {
                                    tracing::warn!(
                                        uuid = %record.uuid,
                                        age_secs = age_ms / 1000,
                                        "recovery timeout: Recovering -> Failed"
                                    );
                                    record.status = StoreDataflowStatus::Failed {
                                        error: format!(
                                            "recovery timeout ({RECOVERY_TIMEOUT_SECS}s): \
                                             no daemon reconnected"
                                        ),
                                    };
                                    record.generation += 1;
                                    record.updated_at = now_ms;
                                    if let Err(e) = store.put_dataflow(&record) {
                                        tracing::warn!("failed to mark dataflow as Failed: {e}");
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => tracing::warn!("failed to list dataflows for recovery check: {e}"),
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
                } else if let Some(build_id) = &message.build_id
                    && let Some(build) = running_builds.get_mut(build_id)
                {
                    if build.log_subscribers.is_empty() {
                        if build.buffered_log_messages.len() < MAX_BUFFERED_LOG_MESSAGES {
                            build.buffered_log_messages.push(message);
                        } else if build.buffered_log_messages.len() == MAX_BUFFERED_LOG_MESSAGES {
                            tracing::warn!(
                                "log buffer full for build {build_id}, dropping new messages"
                            );
                        }
                    } else {
                        send_log_message(&mut build.log_subscribers, &message).await;
                    }
                }
            }
            Event::TopicDebugData {
                dataflow_id,
                subscription_ids,
                payload,
            } => {
                tracing::trace!(
                    %dataflow_id,
                    subscriptions = subscription_ids.len(),
                    bytes = payload.len(),
                    "received topic debug frame from daemon"
                );
                if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                    send_topic_frames(&mut dataflow.topic_subscribers, subscription_ids, payload)
                        .await;
                }
            }
            Event::DaemonExit { daemon_id } => {
                tracing::info!("Daemon `{daemon_id}` exited");
                daemon_connections.remove(&daemon_id);
                if let Err(e) = store.unregister_daemon(&daemon_id) {
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
                    for (node_id, node_metrics) in &metrics {
                        dataflow
                            .node_metrics
                            .insert(node_id.clone(), node_metrics.clone());
                    }
                    if let Some(net) = network {
                        dataflow.network_metrics = Some(net);
                    }

                    #[cfg(feature = "metrics")]
                    {
                        use crate::otel_metrics::node_attrs;
                        use opentelemetry::KeyValue;
                        let df_id = dataflow_id.to_string();
                        for (node_id, node_metrics) in &metrics {
                            let daemon = dataflow
                                .node_to_daemon
                                .get(node_id)
                                .map(|d| d.to_string())
                                .unwrap_or_default();
                            let attrs = node_attrs(df_id.clone(), node_id.to_string(), daemon);
                            otel_metrics
                                .node_cpu
                                .record(node_metrics.cpu_usage as f64, &attrs);
                            otel_metrics
                                .node_memory
                                .record(node_metrics.memory_bytes as i64, &attrs);
                            otel_metrics
                                .node_pending
                                .record(node_metrics.pending_messages as i64, &attrs);
                            otel_metrics
                                .node_restarts
                                .record(node_metrics.restart_count as i64, &attrs);
                        }
                        otel_metrics.dataflow_nodes.record(
                            dataflow.nodes.len() as i64,
                            &[
                                KeyValue::new("dataflow", df_id),
                                KeyValue::new("name", dataflow.name.clone().unwrap_or_default()),
                            ],
                        );
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
                        // Cap cached builds — evict oldest (FIFO via IndexMap)
                        const MAX_FINISHED_BUILDS: usize = 100;
                        while finished_builds.len() > MAX_FINISHED_BUILDS {
                            finished_builds.shift_remove_index(0);
                        }
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
                // the store as Pending/Failed/Recovering, update it to Running.
                for entry in &reported_dataflows {
                    let df_id = &entry.dataflow_id;
                    match store.get_dataflow(df_id) {
                        Ok(Some(mut record)) => match record.status {
                            StoreDataflowStatus::Failed { .. }
                            | StoreDataflowStatus::Pending
                            | StoreDataflowStatus::Recovering => {
                                tracing::info!(
                                    "reconciling dataflow {df_id}: {:?} -> Running \
                                     (daemon reports {} active node(s))",
                                    record.status,
                                    entry.running_nodes.len(),
                                );
                                record.status = StoreDataflowStatus::Running;
                                record.generation += 1;
                                record.updated_at = state::now_millis();
                                if let Err(e) = store.put_dataflow(&record) {
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
                    reported_dataflows.iter().map(|e| e.dataflow_id).collect();
                restore_topic_debug_streams_for_daemon(
                    &running_dataflows,
                    &mut daemon_connections,
                    &daemon_id,
                    &reported_set,
                    &clock,
                )
                .await;
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
                    if let Some(last) = df.last_recovery_attempt.get(&daemon_id)
                        && now.duration_since(*last) < RECOVERY_BACKOFF
                    {
                        continue;
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
                    let spawn_command = dora_message::coordinator_to_daemon::SpawnDataflowNodes {
                        build_id: None,
                        session_id: dora_message::SessionId::generate(),
                        dataflow_id: *uuid,
                        local_working_dir: None,
                        nodes: df.nodes.clone(),
                        dataflow_descriptor: df.descriptor.clone(),
                        spawn_nodes,
                        uv: df.uv,
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
                    if let Some(conn) = daemon_connections.get_mut(&daemon_id)
                        && let Err(e) = conn.send(&message).await
                    {
                        tracing::warn!("failed to send re-spawn to daemon {daemon_id}: {e}");
                    }
                }

                // State catch-up: for dataflows the daemon reports as running,
                // send any state mutations it missed while disconnected.
                for (uuid, df) in &mut running_dataflows {
                    if !df.daemons.contains(&daemon_id) || !reported_set.contains(uuid) {
                        continue;
                    }
                    let last_ack = df.daemon_ack_sequence.get(&daemon_id).copied().unwrap_or(0);
                    if last_ack >= df.state_log_sequence {
                        continue; // already up to date
                    }
                    match df.state_log_delta(last_ack) {
                        Some(entries) if entries.is_empty() => {}
                        Some(entries) => {
                            tracing::info!(
                                "state catch-up: sending {} entry(ies) for dataflow {uuid} \
                                 to daemon {daemon_id} (seq {last_ack}..{})",
                                entries.len(),
                                df.state_log_sequence,
                            );
                            let event = DaemonCoordinatorEvent::StateCatchUp {
                                dataflow_id: *uuid,
                                entries,
                            };
                            if let Ok(msg) = serde_json::to_vec(&Timestamped {
                                inner: event,
                                timestamp: clock.new_timestamp(),
                            }) && let Some(conn) = daemon_connections.get_mut(&daemon_id)
                                && let Err(e) = conn.send(&msg).await
                            {
                                tracing::warn!(
                                    "failed to send state catch-up to daemon {daemon_id}: {e}"
                                );
                            }
                        }
                        None => {
                            // Log was pruned past this daemon's ack — fall back to full
                            // param replay from the store.
                            tracing::info!(
                                "state catch-up: log pruned for dataflow {uuid}, \
                                 falling back to full param replay for daemon {daemon_id}"
                            );
                            handle_pruned_state_catchup_fallback(
                                *uuid,
                                df,
                                &daemon_id,
                                store.clone(),
                                &mut daemon_connections,
                                clock.clone(),
                                now,
                            )
                            .await;
                        }
                    }
                }
            }
            Event::DaemonStateCatchUpAck {
                daemon_id,
                dataflow_id,
                ack_sequence,
            } => {
                if let Some(df) = running_dataflows.get_mut(&dataflow_id) {
                    if !df.daemons.contains(&daemon_id) {
                        tracing::warn!(
                            "ignoring StateCatchUpAck from daemon {daemon_id} \
                             not in dataflow {dataflow_id}"
                        );
                    } else {
                        // Clamp: monotonically increasing, bounded by log sequence.
                        let current = df.daemon_ack_sequence.get(&daemon_id).copied().unwrap_or(0);
                        let clamped = ack_sequence.min(df.state_log_sequence).max(current);
                        df.daemon_ack_sequence.insert(daemon_id, clamped);
                        df.prune_state_log();
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

struct ParamReplayItem {
    node_id: dora_core::config::NodeId,
    key: String,
    value_json: Vec<u8>,
}

#[derive(Debug, Default)]
struct ParamReplaySummary {
    attempted: usize,
    failed: usize,
}

fn topic_outputs_by_daemon(
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
    dataflow_id: DataflowId,
    topics: &[(dora_message::id::NodeId, dora_message::id::DataId)],
) -> eyre::Result<BTreeMap<DaemonId, Vec<(dora_message::id::NodeId, dora_message::id::DataId)>>> {
    let dataflow = running_dataflows
        .get(&dataflow_id)
        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;

    let mut outputs_by_daemon: BTreeMap<
        DaemonId,
        Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    > = BTreeMap::new();
    for (node_id, data_id) in topics {
        let output_exists = dataflow
            .descriptor
            .nodes
            .iter()
            .any(|node| node.id == *node_id && node.outputs.contains(data_id));
        if !output_exists {
            eyre::bail!("no output `{node_id}/{data_id}` in dataflow `{dataflow_id}`");
        }
        let daemon_id = dataflow
            .node_to_daemon
            .get(node_id)
            .wrap_err_with(|| format!("no daemon mapping found for node `{node_id}`"))?;
        outputs_by_daemon
            .entry(daemon_id.clone())
            .or_default()
            .push((node_id.clone(), data_id.clone()));
    }

    Ok(outputs_by_daemon)
}

fn topic_debug_enabled(
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
    dataflow_id: DataflowId,
) -> eyre::Result<bool> {
    let dataflow = running_dataflows
        .get(&dataflow_id)
        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
    Ok(dataflow.descriptor.debug.enable_debug_inspection)
}

async fn start_topic_debug_stream(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    dataflow_id: DataflowId,
    topics: Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    sender: tokio::sync::mpsc::Sender<crate::topic_subscriber::TopicFrame>,
    clock: &HLC,
) -> eyre::Result<Uuid> {
    let outputs_by_daemon = topic_outputs_by_daemon(running_dataflows, dataflow_id, &topics)?;
    if !topic_debug_enabled(running_dataflows, dataflow_id)? {
        eyre::bail!(
            "topic inspection requires `_unstable_debug.enable_debug_inspection: true` \
             (the flag was previously named `publish_all_messages_to_zenoh`; the old name is still accepted)"
        );
    }
    let subscription_id = Uuid::new_v4();
    running_dataflows
        .get_mut(&dataflow_id)
        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?
        .topic_subscribers
        .insert(
            subscription_id,
            topic_subscriber::TopicSubscriber::new(outputs_by_daemon.clone(), sender),
        );

    let mut start_requests = Vec::new();
    for (daemon_id, outputs) in outputs_by_daemon {
        let connection = daemon_connections
            .get_mut(&daemon_id)
            .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?
            .clone();
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::StartTopicDebugStream {
                dataflow_id,
                outputs,
                subscription_id,
            },
            timestamp: clock.new_timestamp(),
        })?;
        start_requests.push(async move {
            let result = async {
                let reply_raw = connection
                    .send_and_receive(&message)
                    .await
                    .wrap_err("failed to send start-topic-debug-stream message")?;
                let reply: DaemonCoordinatorReply = serde_json::from_slice(&reply_raw)
                    .wrap_err("failed to deserialize start-topic-debug-stream reply")?;
                match reply {
                    DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())) => Ok(()),
                    DaemonCoordinatorReply::StartTopicDebugStreamResult(Err(err)) => {
                        Err(eyre!(err))
                    }
                    other => Err(eyre!(
                        "unexpected start-topic-debug-stream reply: {other:?}"
                    )),
                }
            }
            .await;
            (daemon_id, result)
        });
    }

    let mut started_daemons = Vec::new();
    let mut first_error = None;
    for (daemon_id, result) in join_all(start_requests).await {
        match result {
            Ok(()) => started_daemons.push(daemon_id),
            Err(err) => {
                if first_error.is_none() {
                    first_error = Some(err);
                }
            }
        }
    }

    if let Some(err) = first_error {
        rollback_topic_debug_stream(
            running_dataflows,
            daemon_connections,
            dataflow_id,
            subscription_id,
            &started_daemons,
            clock,
        )
        .await?;
        return Err(err);
    }

    Ok(subscription_id)
}

async fn rollback_topic_debug_stream(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    dataflow_id: DataflowId,
    subscription_id: Uuid,
    started_daemons: &[DaemonId],
    clock: &HLC,
) -> eyre::Result<()> {
    let Some(_) = running_dataflows
        .get_mut(&dataflow_id)
        .and_then(|dataflow| dataflow.topic_subscribers.remove(&subscription_id))
    else {
        return Ok(());
    };

    for daemon_id in started_daemons {
        let Some(connection) = daemon_connections.get_mut(daemon_id).cloned() else {
            continue;
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::StopTopicDebugStream {
                dataflow_id,
                subscription_id,
            },
            timestamp: clock.new_timestamp(),
        })?;
        let reply_raw = connection
            .send_and_receive(&message)
            .await
            .wrap_err("failed to roll back start-topic-debug-stream message")?;
        let reply: DaemonCoordinatorReply = serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize rollback stop-topic-debug-stream reply")?;
        match reply {
            DaemonCoordinatorReply::StopTopicDebugStreamResult(Ok(())) => {}
            DaemonCoordinatorReply::StopTopicDebugStreamResult(Err(err)) => {
                tracing::warn!(%daemon_id, %subscription_id, "failed to roll back topic debug stream: {err}");
            }
            other => {
                tracing::warn!(%daemon_id, %subscription_id, "unexpected rollback reply: {other:?}");
            }
        }
    }
    Ok(())
}

async fn stop_topic_debug_stream(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    subscription_id: Uuid,
    clock: &HLC,
) -> eyre::Result<()> {
    let Some((dataflow_id, outputs_by_daemon)) =
        running_dataflows.iter_mut().find_map(|(id, df)| {
            df.topic_subscribers
                .remove(&subscription_id)
                .map(|subscriber| (*id, subscriber.outputs_by_daemon().clone()))
        })
    else {
        return Ok(());
    };

    let mut stop_requests = Vec::new();
    for (daemon_id, _) in outputs_by_daemon {
        let Some(connection) = daemon_connections.get_mut(&daemon_id).cloned() else {
            tracing::warn!(
                %daemon_id,
                %subscription_id,
                "skipping topic debug stream teardown for missing daemon connection"
            );
            continue;
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::StopTopicDebugStream {
                dataflow_id,
                subscription_id,
            },
            timestamp: clock.new_timestamp(),
        })?;
        stop_requests.push(async move {
            let result = async {
                let reply_raw = connection
                    .send_and_receive(&message)
                    .await
                    .wrap_err("failed to send stop-topic-debug-stream message")?;
                let reply: DaemonCoordinatorReply = serde_json::from_slice(&reply_raw)
                    .wrap_err("failed to deserialize stop-topic-debug-stream reply")?;
                match reply {
                    DaemonCoordinatorReply::StopTopicDebugStreamResult(Ok(())) => Ok(()),
                    DaemonCoordinatorReply::StopTopicDebugStreamResult(Err(err)) => Err(eyre!(err)),
                    other => Err(eyre!("unexpected stop-topic-debug-stream reply: {other:?}")),
                }
            }
            .await;
            (daemon_id, result)
        });
    }

    let mut first_error = None;
    for (daemon_id, result) in join_all(stop_requests).await {
        if let Err(err) = result {
            tracing::warn!(
                %daemon_id,
                %subscription_id,
                "failed to stop topic debug stream on daemon: {err}"
            );
            if first_error.is_none() {
                first_error = Some(err);
            }
        }
    }

    if let Some(err) = first_error {
        return Err(err);
    }

    Ok(())
}

/// Close every CLI topic-subscriber channel on a finished dataflow so the
/// corresponding `dora topic echo/hz/info` client sees EOF on its receiver
/// instead of hanging forever.
///
/// Called from the `DataflowFinishedOnDaemon` arm of the event loop when the
/// last daemon has finished. Pulling this out of the inline match arm gives
/// the behavior a named call site: removing it from the event loop shows up
/// in a code review as "no more callers of close_topic_subscribers_on_finish".
fn close_topic_subscribers_on_finish(dataflow: &mut RunningDataflow) {
    for subscriber in dataflow.topic_subscribers.values_mut() {
        subscriber.close();
    }
}

async fn restore_topic_debug_streams_for_daemon(
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    daemon_id: &DaemonId,
    reported_dataflows: &BTreeSet<DataflowId>,
    clock: &HLC,
) {
    let Some(connection) = daemon_connections.get_mut(daemon_id).cloned() else {
        return;
    };

    for (dataflow_id, dataflow) in running_dataflows {
        if !reported_dataflows.contains(dataflow_id) {
            continue;
        }
        for (subscription_id, subscriber) in &dataflow.topic_subscribers {
            let Some(outputs) = subscriber.outputs_by_daemon().get(daemon_id).cloned() else {
                continue;
            };
            let message = match serde_json::to_vec(&Timestamped {
                inner: DaemonCoordinatorEvent::StartTopicDebugStream {
                    dataflow_id: *dataflow_id,
                    outputs,
                    subscription_id: *subscription_id,
                },
                timestamp: clock.new_timestamp(),
            }) {
                Ok(message) => message,
                Err(err) => {
                    tracing::warn!(
                        %daemon_id,
                        %dataflow_id,
                        %subscription_id,
                        "failed to serialize topic debug stream restore message: {err}"
                    );
                    continue;
                }
            };

            match connection.send_and_receive(&message).await {
                Ok(reply_raw) => {
                    match serde_json::from_slice::<DaemonCoordinatorReply>(&reply_raw) {
                        Ok(DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(()))) => {}
                        Ok(DaemonCoordinatorReply::StartTopicDebugStreamResult(Err(err))) => {
                            tracing::warn!(
                                %daemon_id,
                                %dataflow_id,
                                %subscription_id,
                                "daemon rejected restored topic debug stream: {err}"
                            );
                        }
                        Ok(other) => {
                            tracing::warn!(
                                %daemon_id,
                                %dataflow_id,
                                %subscription_id,
                                "unexpected restore-topic-debug-stream reply: {other:?}"
                            );
                        }
                        Err(err) => {
                            tracing::warn!(
                                %daemon_id,
                                %dataflow_id,
                                %subscription_id,
                                "failed to deserialize restore-topic-debug-stream reply: {err}"
                            );
                        }
                    }
                }
                Err(err) => {
                    tracing::warn!(
                        %daemon_id,
                        %dataflow_id,
                        %subscription_id,
                        "failed to restore topic debug stream after daemon reconnect: {err}"
                    );
                }
            }
        }
    }
}

async fn handle_pruned_state_catchup_fallback(
    dataflow_id: DataflowId,
    dataflow: &mut RunningDataflow,
    daemon_id: &DaemonId,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    daemon_connections: &mut DaemonConnections,
    clock: Arc<HLC>,
    now: Instant,
) {
    if let Some(last_replay_attempt) = dataflow.last_replay_attempt.get(daemon_id)
        && now.duration_since(*last_replay_attempt) < FALLBACK_REPLAY_BACKOFF
    {
        tracing::debug!(
            "skipping fallback replay for dataflow {dataflow_id} on daemon {daemon_id}: \
                 backoff active"
        );
        return;
    }

    let Some(connection) = daemon_connections.get_mut(daemon_id).cloned() else {
        tracing::warn!(
            "failed to run fallback replay for dataflow {dataflow_id}: \
             daemon {daemon_id} is not connected"
        );
        return;
    };

    let node_ids_on_daemon: Vec<_> = dataflow
        .node_to_daemon
        .iter()
        .filter(|(_, did)| *did == daemon_id)
        .map(|(node_id, _)| node_id.clone())
        .collect();
    dataflow.last_replay_attempt.insert(daemon_id.clone(), now);

    let replay_summary = replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id.clone(),
        node_ids_on_daemon,
        store,
        connection,
        clock,
    )
    .await;

    let last_ack = dataflow
        .daemon_ack_sequence
        .get(daemon_id)
        .copied()
        .unwrap_or(0);
    if replay_summary.failed == 0 {
        // Mark daemon as caught up only when full replay succeeds:
        // replay is authoritative for pruned history, and advancing
        // ack on partial failure can silently diverge runtime/store state.
        // Individual SetParam events don't trigger StateCatchUpAck, so
        // we set ack here for successful full replay to avoid repeated
        // fallback replays on every status-report cycle.
        dataflow
            .daemon_ack_sequence
            .insert(daemon_id.clone(), dataflow.state_log_sequence);
    } else {
        tracing::warn!(
            "fallback replay incomplete for dataflow {dataflow_id} on daemon \
             {daemon_id}: attempted={}, failed={}; leaving ack at {}",
            replay_summary.attempted,
            replay_summary.failed,
            last_ack
        );
    }
}

fn collect_param_replay_items(
    dataflow_id: DataflowId,
    node_ids_on_daemon: &[dora_core::config::NodeId],
    store: &dyn dora_coordinator_store::CoordinatorStore,
) -> Vec<ParamReplayItem> {
    let mut items = Vec::new();
    for node_id in node_ids_on_daemon {
        let params = match store.list_node_params(&dataflow_id, node_id) {
            Ok(params) => params,
            Err(err) => {
                tracing::warn!(
                    "failed to load persisted params for {dataflow_id}/{node_id}: {err}"
                );
                continue;
            }
        };
        for (key, bytes) in params {
            items.push(ParamReplayItem {
                node_id: node_id.clone(),
                key,
                value_json: bytes,
            });
        }
    }
    items
}

fn build_set_param_message_from_raw_json(
    dataflow_id: DataflowId,
    node_id: &dora_core::config::NodeId,
    key: &str,
    value_json: &[u8],
    timestamp: dora_core::uhlc::Timestamp,
) -> eyre::Result<Vec<u8>> {
    #[derive(Serialize)]
    struct SetParamPayloadRaw<'a> {
        dataflow_id: DataflowId,
        node_id: &'a dora_core::config::NodeId,
        key: &'a str,
        value: &'a RawValue,
    }
    #[derive(Serialize)]
    enum DaemonCoordinatorEventRaw<'a> {
        SetParam(SetParamPayloadRaw<'a>),
    }
    #[derive(Serialize)]
    struct TimestampedDaemonEventRaw<'a> {
        inner: DaemonCoordinatorEventRaw<'a>,
        timestamp: dora_core::uhlc::Timestamp,
    }

    // Parse persisted bytes as raw JSON once, then rely on serde for
    // envelope serialization instead of manual string JSON assembly.
    let value = serde_json::from_slice::<Box<RawValue>>(value_json)
        .map_err(|e| eyre!("invalid persisted param JSON: {e}"))?;

    serde_json::to_vec(&TimestampedDaemonEventRaw {
        inner: DaemonCoordinatorEventRaw::SetParam(SetParamPayloadRaw {
            dataflow_id,
            node_id,
            key,
            value: value.as_ref(),
        }),
        timestamp,
    })
    .map_err(Into::into)
}

fn schedule_param_replay_for_ready_dataflow(
    dataflow_id: DataflowId,
    dataflow: &RunningDataflow,
    daemon_connections: &mut DaemonConnections,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: Arc<HLC>,
) {
    // Replay persisted runtime parameters once nodes are ready.
    // This restores desired node state after restart/recovery.
    let daemon_ids: Vec<_> = dataflow.daemons.iter().cloned().collect();
    for daemon_id in daemon_ids {
        let Some(connection) = daemon_connections.get_mut(&daemon_id).cloned() else {
            tracing::warn!(
                "cannot replay params for dataflow {dataflow_id}: no connection for daemon {daemon_id}"
            );
            continue;
        };
        let node_ids_on_daemon: Vec<_> = dataflow
            .node_to_daemon
            .iter()
            .filter(|(_, node_daemon_id)| *node_daemon_id == &daemon_id)
            .map(|(node_id, _)| node_id.clone())
            .collect();
        let store = store.clone();
        let clock = clock.clone();
        tokio::spawn(async move {
            replay_persisted_params_for_daemon(
                dataflow_id,
                daemon_id,
                node_ids_on_daemon,
                store,
                connection,
                clock,
            )
            .await;
        });
    }
}

async fn replay_persisted_params_for_daemon(
    dataflow_id: DataflowId,
    daemon_id: DaemonId,
    node_ids_on_daemon: Vec<dora_core::config::NodeId>,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    connection: crate::state::DaemonConnection,
    clock: Arc<HLC>,
) -> ParamReplaySummary {
    let replay_items = collect_param_replay_items(dataflow_id, &node_ids_on_daemon, store.as_ref());
    let mut summary = ParamReplaySummary::default();
    if replay_items.is_empty() {
        return summary;
    }

    tracing::debug!(
        "replaying {} persisted params for dataflow {} on daemon {}",
        replay_items.len(),
        dataflow_id,
        daemon_id
    );

    for item in replay_items {
        summary.attempted += 1;
        let message = match build_set_param_message_from_raw_json(
            dataflow_id,
            &item.node_id,
            &item.key,
            &item.value_json,
            clock.new_timestamp(),
        ) {
            Ok(msg) => msg,
            Err(err) => {
                tracing::warn!(
                    "skipping corrupt persisted param {dataflow_id}/{}/{}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
                continue;
            }
        };

        let reply_raw = match connection.send_and_receive(&message).await {
            Ok(reply) => reply,
            Err(err) => {
                tracing::warn!(
                    "failed to replay param {dataflow_id}/{}/{} to daemon {daemon_id}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
                continue;
            }
        };

        match serde_json::from_slice(&reply_raw) {
            Ok(DaemonCoordinatorReply::SetParamResult(Ok(()))) => {}
            Ok(DaemonCoordinatorReply::SetParamResult(Err(err))) => {
                tracing::warn!(
                    "daemon rejected replayed param {dataflow_id}/{}/{}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
            }
            Ok(other) => {
                tracing::warn!(
                    "unexpected daemon reply while replaying param {dataflow_id}/{}/{}: {other:?}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
            }
            Err(err) => {
                tracing::warn!(
                    "failed to deserialize daemon reply while replaying param {dataflow_id}/{}/{}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
            }
        }
    }

    summary
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_message::descriptor::Descriptor;
    use std::collections::HashMap;
    use tokio::time::{Duration as TokioDuration, timeout};
    use uuid::Uuid;

    fn test_running_dataflow(
        dataflow_id: DataflowId,
        daemon_id: DaemonId,
        node_id: dora_core::config::NodeId,
    ) -> RunningDataflow {
        let mut daemons = BTreeSet::new();
        daemons.insert(daemon_id.clone());

        let mut node_to_daemon = BTreeMap::new();
        node_to_daemon.insert(node_id, daemon_id);
        let descriptor: Descriptor = serde_json::from_value(serde_json::json!({
            "nodes": [{
                "id": "sender",
                "outputs": ["message"],
            }]
        }))
        .expect("valid test descriptor");

        RunningDataflow {
            name: None,
            uuid: dataflow_id,
            descriptor,
            daemons,
            pending_daemons: BTreeSet::new(),
            exited_before_subscribe: vec![],
            nodes: BTreeMap::new(),
            node_to_daemon,
            node_metrics: BTreeMap::new(),
            network_metrics: None,
            spawn_result: CachedResult::default(),
            stop_reply_senders: vec![],
            buffered_log_messages: vec![],
            log_subscribers: vec![],
            topic_subscribers: BTreeMap::new(),
            pending_spawn_results: BTreeSet::new(),
            created_at: 0,
            store_generation: 0,
            last_recovery_attempt: BTreeMap::new(),
            last_replay_attempt: BTreeMap::new(),
            uv: false,
            state_log_sequence: 0,
            state_log: Vec::new(),
            daemon_ack_sequence: BTreeMap::new(),
        }
    }

    #[test]
    fn resolve_param_target_returns_running_daemon_for_active_node() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(
            dataflow_id,
            test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone()),
        );

        let target =
            resolve_param_target(&running_dataflows, store.as_ref(), &dataflow_id, &node_id)
                .unwrap();
        match target {
            ParamTarget::Running { daemon_id: got } => assert_eq!(got, daemon_id),
            ParamTarget::PersistedOnly => panic!("expected running target"),
        }
    }

    #[test]
    fn resolve_param_target_returns_persisted_only_for_known_stopped_dataflow() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let node_id: dora_core::config::NodeId = "camera".to_string().into();
        let running_dataflows = HashMap::new();

        let record = dora_coordinator_store::DataflowRecord {
            uuid: dataflow_id,
            name: Some("df".to_string()),
            descriptor_json: serde_json::json!({
                "nodes": [
                    {"id": "camera"}
                ]
            })
            .to_string(),
            status: dora_coordinator_store::DataflowStatus::Succeeded,
            daemon_ids: Vec::new(),
            node_to_daemon: BTreeMap::new(),
            uv: false,
            generation: 1,
            created_at: 0,
            updated_at: 0,
        };
        store.put_dataflow(&record).unwrap();

        let target =
            resolve_param_target(&running_dataflows, store.as_ref(), &dataflow_id, &node_id)
                .unwrap();
        match target {
            ParamTarget::PersistedOnly => {}
            ParamTarget::Running { .. } => panic!("expected persisted-only target"),
        }
    }

    #[test]
    fn resolve_param_target_errors_for_unknown_dataflow() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let node_id: dora_core::config::NodeId = "camera".to_string().into();
        let running_dataflows = HashMap::new();

        let err = resolve_param_target(&running_dataflows, store.as_ref(), &dataflow_id, &node_id)
            .expect_err("unknown dataflow should fail validation");
        assert!(err.to_string().contains("dataflow"));
        assert!(err.to_string().contains("node"));
    }

    #[test]
    fn resolve_param_target_errors_for_unknown_node() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let existing_node: dora_core::config::NodeId = "camera".to_string().into();
        let missing_node: dora_core::config::NodeId = "ghost".to_string().into();

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(
            dataflow_id,
            test_running_dataflow(dataflow_id, daemon_id, existing_node),
        );

        let err = resolve_param_target(
            &running_dataflows,
            store.as_ref(),
            &dataflow_id,
            &missing_node,
        )
        .expect_err("unknown node should fail validation");
        assert!(err.to_string().contains("not found in dataflow"));
    }

    #[tokio::test]
    async fn replay_replays_persisted_param_to_daemon_connection() {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
            method: String,
            params: Timestamped<DaemonCoordinatorEvent>,
        }

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let value_bytes = serde_json::to_vec(&serde_json::json!(42)).unwrap();
        store
            .put_node_param(&dataflow_id, &node_id, "threshold", &value_bytes)
            .unwrap();

        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());

        let node_id_for_assert = node_id.clone();
        let daemon_task = tokio::spawn(async move {
            let outbound = rx
                .recv()
                .await
                .expect("daemon should receive replay command");
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            assert_eq!(outbound_raw.method, "daemon_command");

            let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");

            match outbound_raw.params.inner {
                DaemonCoordinatorEvent::SetParam {
                    dataflow_id: replay_df,
                    node_id: replay_node,
                    key,
                    value,
                } => {
                    assert_eq!(replay_df, dataflow_id);
                    assert_eq!(replay_node, node_id_for_assert);
                    assert_eq!(key, "threshold");
                    assert_eq!(value, serde_json::json!(42));
                }
                other => panic!("unexpected replay event: {other:?}"),
            }

            let reply =
                serde_json::to_string(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
            let reply_tx = pending_replies
                .lock()
                .await
                .remove(&request_id)
                .expect("pending reply sender should exist");
            let _ = reply_tx.send(reply);
        });

        let summary = replay_persisted_params_for_daemon(
            dataflow_id,
            daemon_id,
            vec![node_id],
            store,
            connection,
            Arc::new(HLC::default()),
        )
        .await;
        assert_eq!(summary.attempted, 1);
        assert_eq!(summary.failed, 0);

        daemon_task.await.unwrap();
    }

    #[tokio::test]
    async fn replay_skips_when_no_persisted_params() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection = crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new());

        let summary = replay_persisted_params_for_daemon(
            dataflow_id,
            daemon_id,
            vec![node_id],
            store,
            connection,
            Arc::new(HLC::default()),
        )
        .await;
        assert_eq!(summary.attempted, 0);
        assert_eq!(summary.failed, 0);

        let recv = timeout(TokioDuration::from_millis(50), rx.recv()).await;
        assert!(
            matches!(recv, Err(_) | Ok(None)),
            "no replay command should be sent for empty persisted params"
        );
    }

    #[tokio::test]
    async fn replay_reports_failure_when_daemon_rejects_param() {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
        }

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let value_bytes = serde_json::to_vec(&serde_json::json!(7)).unwrap();
        store
            .put_node_param(&dataflow_id, &node_id, "threshold", &value_bytes)
            .unwrap();

        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());

        let daemon_task = tokio::spawn(async move {
            let outbound = rx
                .recv()
                .await
                .expect("daemon should receive replay command");
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");

            let reply = serde_json::to_string(&DaemonCoordinatorReply::SetParamResult(Err(
                "rejected".to_string(),
            )))
            .unwrap();
            let reply_tx = pending_replies
                .lock()
                .await
                .remove(&request_id)
                .expect("pending reply sender should exist");
            let _ = reply_tx.send(reply);
        });

        let summary = replay_persisted_params_for_daemon(
            dataflow_id,
            daemon_id,
            vec![node_id],
            store,
            connection,
            Arc::new(HLC::default()),
        )
        .await;

        assert_eq!(summary.attempted, 1);
        assert_eq!(summary.failed, 1);
        assert!(summary.failed > 0);
        daemon_task.await.unwrap();
    }

    #[tokio::test]
    async fn fallback_replay_keeps_ack_unchanged_when_daemon_is_disconnected() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let value_bytes = serde_json::to_vec(&serde_json::json!(1)).unwrap();
        store
            .put_node_param(&dataflow_id, &node_id, "threshold", &value_bytes)
            .unwrap();

        let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        dataflow.state_log_sequence = 10;
        dataflow.daemon_ack_sequence.insert(daemon_id.clone(), 3);

        let mut daemon_connections = DaemonConnections::default();
        handle_pruned_state_catchup_fallback(
            dataflow_id,
            &mut dataflow,
            &daemon_id,
            store,
            &mut daemon_connections,
            Arc::new(HLC::default()),
            Instant::now(),
        )
        .await;

        assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&3));
        assert!(!dataflow.last_replay_attempt.contains_key(&daemon_id));
    }

    #[tokio::test]
    async fn fallback_replay_respects_backoff_window() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        dataflow.state_log_sequence = 8;
        dataflow.daemon_ack_sequence.insert(daemon_id.clone(), 2);
        dataflow
            .last_replay_attempt
            .insert(daemon_id.clone(), Instant::now());

        let mut daemon_connections = DaemonConnections::default();
        handle_pruned_state_catchup_fallback(
            dataflow_id,
            &mut dataflow,
            &daemon_id,
            store,
            &mut daemon_connections,
            Arc::new(HLC::default()),
            Instant::now(),
        )
        .await;

        // No replay attempted while backoff is active, so ack remains unchanged.
        assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&2));
    }

    #[tokio::test]
    async fn ready_boundary_schedules_replay_for_daemon_nodes() {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
            method: String,
            params: Timestamped<DaemonCoordinatorEvent>,
        }

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let value_bytes = serde_json::to_vec(&serde_json::json!(123)).unwrap();
        store
            .put_node_param(&dataflow_id, &node_id, "gain", &value_bytes)
            .unwrap();

        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
        let mut daemon_connections = DaemonConnections::default();
        daemon_connections.add(daemon_id.clone(), connection);

        let running_dataflow =
            test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());

        let daemon_task = tokio::spawn(async move {
            let outbound = rx
                .recv()
                .await
                .expect("daemon should receive replay command from ready-boundary scheduler");
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            assert_eq!(outbound_raw.method, "daemon_command");
            match outbound_raw.params.inner {
                DaemonCoordinatorEvent::SetParam {
                    dataflow_id: replay_df,
                    node_id: replay_node,
                    key,
                    value,
                } => {
                    assert_eq!(replay_df, dataflow_id);
                    assert_eq!(replay_node, node_id);
                    assert_eq!(key, "gain");
                    assert_eq!(value, serde_json::json!(123));
                }
                other => panic!("unexpected replay event: {other:?}"),
            }

            let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
            let reply =
                serde_json::to_string(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
            let reply_tx = pending_replies
                .lock()
                .await
                .remove(&request_id)
                .expect("pending reply sender should exist");
            let _ = reply_tx.send(reply);
        });

        schedule_param_replay_for_ready_dataflow(
            dataflow_id,
            &running_dataflow,
            &mut daemon_connections,
            store,
            Arc::new(HLC::default()),
        );

        daemon_task.await.unwrap();
    }

    #[tokio::test]
    async fn start_topic_debug_stream_targets_source_daemon() {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
            method: String,
            params: Timestamped<DaemonCoordinatorEvent>,
        }

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let data_id: dora_core::config::DataId = "message".to_string().into();
        let (frame_tx, _frame_rx) =
            tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);

        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
        let mut daemon_connections = DaemonConnections::default();
        daemon_connections.add(daemon_id.clone(), connection);

        let mut running_dataflows = HashMap::new();
        let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
        dataflow.descriptor.debug.enable_debug_inspection = true;
        running_dataflows.insert(dataflow_id, dataflow);
        let expected_node_id = node_id.clone();
        let expected_data_id = data_id.clone();
        let seen_subscription = Arc::new(tokio::sync::Mutex::new(None::<Uuid>));
        let seen_subscription_task = seen_subscription.clone();

        let daemon_task = tokio::spawn(async move {
            let outbound = rx
                .recv()
                .await
                .expect("daemon should receive topic stream command");
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            assert_eq!(outbound_raw.method, "daemon_command");
            match outbound_raw.params.inner {
                DaemonCoordinatorEvent::StartTopicDebugStream {
                    dataflow_id: start_df,
                    outputs,
                    subscription_id,
                } => {
                    assert_eq!(start_df, dataflow_id);
                    assert_eq!(outputs, vec![(expected_node_id, expected_data_id)]);
                    *seen_subscription_task.lock().await = Some(subscription_id);
                }
                other => panic!("unexpected topic stream event: {other:?}"),
            }

            let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
            let reply =
                serde_json::to_string(&DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())))
                    .unwrap();
            let reply_tx = pending_replies
                .lock()
                .await
                .remove(&request_id)
                .expect("pending reply sender should exist");
            let _ = reply_tx.send(reply);
        });

        let subscription_id = start_topic_debug_stream(
            &mut running_dataflows,
            &mut daemon_connections,
            dataflow_id,
            vec![(node_id.clone(), data_id.clone())],
            frame_tx,
            &HLC::default(),
        )
        .await
        .expect("subscription should succeed");

        daemon_task.await.unwrap();
        assert_eq!(Some(subscription_id), *seen_subscription.lock().await);
        assert_eq!(
            running_dataflows[&dataflow_id]
                .topic_subscribers
                .get(&subscription_id)
                .expect("subscription should be stored")
                .outputs_by_daemon()
                .values()
                .next()
                .expect("daemon mapping should exist"),
            &vec![(node_id, data_id)]
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn start_topic_debug_stream_rolls_back_on_daemon_error() {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
            params: Timestamped<DaemonCoordinatorEvent>,
        }

        let dataflow_id = Uuid::new_v4();
        let daemon_id_a = DaemonId::new(Some("daemon-a".to_string()));
        let daemon_id_b = DaemonId::new(Some("daemon-b".to_string()));
        let node_id_a: dora_message::id::NodeId = "sender".to_string().into();
        let node_id_b: dora_message::id::NodeId = "sink".to_string().into();
        let data_id: dora_core::config::DataId = "message".to_string().into();
        let (frame_tx, _frame_rx) =
            tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);

        let mut daemon_connections = DaemonConnections::default();
        let mut daemon_tasks = Vec::new();
        for (daemon_id, should_fail) in [(daemon_id_a.clone(), false), (daemon_id_b.clone(), true)]
        {
            let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
            let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
            let connection =
                crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
            daemon_connections.add(daemon_id, connection);

            daemon_tasks.push(tokio::spawn(async move {
                while let Some(outbound) = rx.recv().await {
                    let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
                    let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
                    let reply = match outbound_raw.params.inner {
                        DaemonCoordinatorEvent::StartTopicDebugStream { .. } if should_fail => {
                            serde_json::to_string(
                                &DaemonCoordinatorReply::StartTopicDebugStreamResult(Err(
                                    "daemon rejected debug stream".to_string(),
                                )),
                            )
                            .unwrap()
                        }
                        DaemonCoordinatorEvent::StartTopicDebugStream { .. } => {
                            serde_json::to_string(
                                &DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())),
                            )
                            .unwrap()
                        }
                        DaemonCoordinatorEvent::StopTopicDebugStream { .. } => {
                            serde_json::to_string(
                                &DaemonCoordinatorReply::StopTopicDebugStreamResult(Ok(())),
                            )
                            .unwrap()
                        }
                        other => panic!("unexpected daemon event during rollback test: {other:?}"),
                    };
                    let reply_tx = pending_replies
                        .lock()
                        .await
                        .remove(&request_id)
                        .expect("pending reply sender should exist");
                    let _ = reply_tx.send(reply);
                }
            }));
        }

        let mut running_dataflows = HashMap::new();
        let mut dataflow =
            test_running_dataflow(dataflow_id, daemon_id_a.clone(), node_id_a.clone());
        dataflow.descriptor.debug.enable_debug_inspection = true;
        dataflow
            .node_to_daemon
            .insert(node_id_b.clone(), daemon_id_b.clone());
        let mut descriptor_json = serde_json::to_value(&dataflow.descriptor).unwrap();
        descriptor_json
            .get_mut("nodes")
            .and_then(serde_json::Value::as_array_mut)
            .expect("descriptor nodes array")
            .push(serde_json::json!({
                "id": node_id_b,
                "outputs": [data_id.clone()],
            }));
        dataflow.descriptor = serde_json::from_value(descriptor_json).unwrap();
        running_dataflows.insert(dataflow_id, dataflow);

        let err = start_topic_debug_stream(
            &mut running_dataflows,
            &mut daemon_connections,
            dataflow_id,
            vec![(node_id_a, data_id.clone()), (node_id_b, data_id)],
            frame_tx,
            &HLC::default(),
        )
        .await
        .expect_err("subscription should fail");

        assert!(format!("{err:#}").contains("daemon rejected debug stream"));
        assert!(
            running_dataflows[&dataflow_id].topic_subscribers.is_empty(),
            "failed subscription should be rolled back"
        );

        for task in daemon_tasks {
            task.abort();
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn restore_topic_debug_streams_re_issues_start_after_reconnect() {
        // Regression test for the daemon-reconnect lifecycle fix (#238 / #242):
        // when a daemon reconnects, every active subscriber with outputs on
        // that daemon must receive a fresh StartTopicDebugStream.
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
            params: Timestamped<DaemonCoordinatorEvent>,
        }

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let data_id: dora_core::config::DataId = "message".to_string().into();
        let subscription_id = Uuid::new_v4();

        // Stand up a connection whose rx we can inspect after the reconnect path runs.
        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
        let mut daemon_connections = DaemonConnections::default();
        daemon_connections.add(daemon_id.clone(), connection);

        // Pre-populate a dataflow with an already-registered subscriber. This
        // simulates a subscriber that was set up before the daemon dropped.
        let mut running_dataflows = HashMap::new();
        let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
        let (frame_tx, _frame_rx) =
            tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);
        let mut outputs_by_daemon = BTreeMap::new();
        outputs_by_daemon.insert(daemon_id.clone(), vec![(node_id.clone(), data_id.clone())]);
        dataflow.topic_subscribers.insert(
            subscription_id,
            crate::topic_subscriber::TopicSubscriber::new(outputs_by_daemon, frame_tx),
        );
        running_dataflows.insert(dataflow_id, dataflow);

        // Task that responds as the reconnected daemon would.
        let seen = Arc::new(tokio::sync::Mutex::new(None::<(Uuid, DataflowId)>));
        let seen_task = seen.clone();
        let daemon_task = tokio::spawn(async move {
            let outbound = rx
                .recv()
                .await
                .expect("reconnected daemon should receive restore message");
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            if let DaemonCoordinatorEvent::StartTopicDebugStream {
                dataflow_id: restore_df,
                subscription_id: restore_sub,
                ..
            } = outbound_raw.params.inner
            {
                *seen_task.lock().await = Some((restore_sub, restore_df));
            } else {
                panic!(
                    "unexpected event on reconnect: {:?}",
                    outbound_raw.params.inner
                );
            }
            let reply =
                serde_json::to_string(&DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())))
                    .unwrap();
            let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
            let reply_tx = pending_replies
                .lock()
                .await
                .remove(&request_id)
                .expect("pending reply sender should exist");
            let _ = reply_tx.send(reply);
        });

        let mut reported = BTreeSet::new();
        reported.insert(dataflow_id);

        restore_topic_debug_streams_for_daemon(
            &running_dataflows,
            &mut daemon_connections,
            &daemon_id,
            &reported,
            &HLC::default(),
        )
        .await;

        daemon_task.await.unwrap();
        assert_eq!(
            *seen.lock().await,
            Some((subscription_id, dataflow_id)),
            "restore should re-issue StartTopicDebugStream for the existing subscription"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn restore_topic_debug_streams_skips_unreported_dataflows() {
        // If the daemon did not report this dataflow on reconnect, we must
        // not re-issue subscriptions for it (the dataflow is no longer on
        // that daemon).
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let data_id: dora_core::config::DataId = "message".to_string().into();

        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
        let mut daemon_connections = DaemonConnections::default();
        daemon_connections.add(daemon_id.clone(), connection);

        let mut running_dataflows = HashMap::new();
        let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
        let (frame_tx, _frame_rx) =
            tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);
        let mut outputs_by_daemon = BTreeMap::new();
        outputs_by_daemon.insert(daemon_id.clone(), vec![(node_id, data_id)]);
        dataflow.topic_subscribers.insert(
            Uuid::new_v4(),
            crate::topic_subscriber::TopicSubscriber::new(outputs_by_daemon, frame_tx),
        );
        running_dataflows.insert(dataflow_id, dataflow);

        // Empty reported set: daemon did not acknowledge this dataflow.
        let reported = BTreeSet::new();
        restore_topic_debug_streams_for_daemon(
            &running_dataflows,
            &mut daemon_connections,
            &daemon_id,
            &reported,
            &HLC::default(),
        )
        .await;

        // No message should have been sent.
        assert!(
            rx.try_recv().is_err(),
            "restore must not message the daemon for unreported dataflows"
        );
    }

    #[tokio::test]
    async fn close_topic_subscribers_on_finish_drains_all_subscribers() {
        // Regression test for the CRITICAL dataflow-finish leak (#242 root cause).
        // Directly exercises the helper called from the DataflowFinishedOnDaemon
        // arm of the event loop (lib.rs: `close_topic_subscribers_on_finish`).
        // CLI must observe EOF on its data_rx (no silent hang).
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();

        let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id);
        let (tx1, mut rx1) = tokio::sync::mpsc::channel(4);
        let (tx2, mut rx2) = tokio::sync::mpsc::channel(4);
        dataflow.topic_subscribers.insert(
            Uuid::new_v4(),
            crate::topic_subscriber::TopicSubscriber::new(BTreeMap::new(), tx1),
        );
        dataflow.topic_subscribers.insert(
            Uuid::new_v4(),
            crate::topic_subscriber::TopicSubscriber::new(BTreeMap::new(), tx2),
        );

        // Call the real helper, not a mirror of it. If the helper is renamed
        // or its semantics change, this test must be updated or fails loudly.
        super::close_topic_subscribers_on_finish(&mut dataflow);

        // Wrap recv in a short timeout so a future regression where close()
        // silently drops the sender without closing fails loudly instead of
        // hanging CI indefinitely.
        let timeout = std::time::Duration::from_secs(1);
        let got1 = tokio::time::timeout(timeout, rx1.recv())
            .await
            .expect("subscriber 1 must not hang after dataflow finish");
        assert!(got1.is_none(), "subscriber 1 must see EOF");
        let got2 = tokio::time::timeout(timeout, rx2.recv())
            .await
            .expect("subscriber 2 must not hang after dataflow finish");
        assert!(got2.is_none(), "subscriber 2 must see EOF");
    }

    /// Source-level guard that the DataflowFinishedOnDaemon dispatch still
    /// calls the cleanup helper. A refactor that moves the branch but forgets
    /// to keep the call wired up would leave the helper unreferenced from
    /// `lib.rs` and fail this check.
    ///
    /// This is a second-line guard — the primary protection is that
    /// `close_topic_subscribers_on_finish` has no other callers, so removal
    /// also produces a `dead_code` lint. This test hard-fails the case a
    /// reviewer might waive.
    #[test]
    fn dataflow_finish_dispatch_calls_close_helper() {
        // Runtime read (not include_str!) so we don't embed ~100KB of source
        // into every test binary. The file is always present when `cargo test`
        // runs from the crate root.
        let src = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/lib.rs"),
        )
        .expect("lib.rs must be readable at CARGO_MANIFEST_DIR/src/lib.rs");
        assert!(
            src.contains("close_topic_subscribers_on_finish(&mut finished_dataflow)"),
            "DataflowFinishedOnDaemon arm must still call close_topic_subscribers_on_finish; \
             if you moved the cleanup, update this guard to point at the new call site"
        );
    }

    #[test]
    fn state_log_append_and_sequence() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
        assert_eq!(df.state_log_sequence, 0);
        assert!(df.state_log.is_empty());

        df.append_state_log(StateCatchUpOperation::SetParam {
            node_id: node_id.clone(),
            key: "threshold".to_string(),
            value: serde_json::json!(42),
        });
        assert_eq!(df.state_log_sequence, 1);
        assert_eq!(df.state_log.len(), 1);
        assert_eq!(df.state_log[0].sequence, 1);

        df.append_state_log(StateCatchUpOperation::DeleteParam {
            node_id,
            key: "threshold".to_string(),
        });
        assert_eq!(df.state_log_sequence, 2);
        assert_eq!(df.state_log.len(), 2);
    }

    #[test]
    fn state_log_delta_returns_missed_entries() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
        for i in 0..5 {
            df.append_state_log(StateCatchUpOperation::SetParam {
                node_id: node_id.clone(),
                key: format!("key_{i}"),
                value: serde_json::json!(i),
            });
        }

        // Daemon acked up to seq 2 — should get entries 3, 4, 5
        let delta = df.state_log_delta(2).expect("delta should be available");
        assert_eq!(delta.len(), 3);
        assert_eq!(delta[0].sequence, 3);
        assert_eq!(delta[2].sequence, 5);

        // Daemon fully caught up — empty delta
        let delta = df.state_log_delta(5).expect("delta should be available");
        assert!(delta.is_empty());
    }

    #[test]
    fn state_log_prune_removes_acked_entries() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let d1 = DaemonId::new(Some("m1".to_string()));
        let d2 = DaemonId::new(Some("m2".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, d1.clone(), node_id.clone());
        df.daemons.insert(d2.clone());
        for i in 0..5 {
            df.append_state_log(StateCatchUpOperation::SetParam {
                node_id: node_id.clone(),
                key: format!("key_{i}"),
                value: serde_json::json!(i),
            });
        }

        // d1 acked 3, d2 acked 5 — min is 3, so entries 1-3 should be pruned
        df.daemon_ack_sequence.insert(d1, 3);
        df.daemon_ack_sequence.insert(d2, 5);
        df.prune_state_log();
        assert_eq!(df.state_log.len(), 2);
        assert_eq!(df.state_log[0].sequence, 4);
    }

    #[test]
    fn state_log_delta_returns_none_when_pruned() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let d1 = DaemonId::new(Some("m1".to_string()));
        let d2 = DaemonId::new(Some("m2".to_string()));
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, d1.clone(), node_id.clone());
        df.daemons.insert(d2.clone());
        for i in 0..10 {
            df.append_state_log(StateCatchUpOperation::SetParam {
                node_id: node_id.clone(),
                key: format!("key_{i}"),
                value: serde_json::json!(i),
            });
        }

        // Simulate pruning: both acked up to 7
        df.daemon_ack_sequence.insert(d1, 7);
        df.daemon_ack_sequence.insert(d2, 7);
        df.prune_state_log();
        // Log now starts at seq 8

        // A daemon that was at seq 2 cannot catch up incrementally
        assert!(df.state_log_delta(2).is_none());
        // But a daemon at seq 7 can
        let delta = df.state_log_delta(7).expect("should succeed");
        assert_eq!(delta.len(), 3); // entries 8, 9, 10
    }
}

#[cfg(feature = "tracing")]
fn handle_get_traces(span_store: &SpanStore) -> ControlRequestReply {
    use dora_message::coordinator_to_cli::TraceSummary;
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
    let mut groups: HashMap<&str, Vec<&dora_tracing::span_store::SpanRecord>> = HashMap::new();
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
    use dora_message::coordinator_to_cli::TraceSpan;

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
            tracing::warn!("failed to persist dataflow stopping: {e}");
        }
    }

    // 3. Start a new dataflow with the stored descriptor
    let new_dataflow = start_dataflow(
        None, // no build_id for restart
        dora_message::SessionId::generate(),
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
        tracing::warn!("failed to persist restarted dataflow: {e}");
    }
    running_dataflows.insert(new_uuid, new_df);

    Ok(ControlRequestReply::DataflowRestarted {
        old_uuid: dataflow_uuid,
        new_uuid,
    })
}
