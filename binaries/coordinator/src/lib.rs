//! **Internal to dora — not a public API.**
//!
//! This crate is published to crates.io only because cargo requires every
//! dependency of a published crate to be published; `dora-node-api` and
//! `dora-cli` depend on it. It is not covered by dora's 1.0 stability
//! guarantee and may change in any release, including a patch.
//!
//! Depend on it directly at your own risk. See the "Stability scope at 1.0"
//! section of `docs/api-rust.md`.
//!
use crate::{
    events::set_up_ctrlc_handler,
    handlers::{
        build_dataflow, dataflow_result, handle_destroy, parse_logs_node_id, reload_dataflow,
        resolve_name, restart_node, retrieve_logs, send_log_message, send_topic_frames,
        start_dataflow, stop_dataflow, stop_node,
    },
    state::{
        ArchivedDataflow, CachedResult, ParamTarget, PendingRestart, RunningBuild, RunningDataflow,
    },
};
pub use control::ControlEvent;
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
pub use dora_coordinator_store::{self, CoordinatorStore, InMemoryStore};
use dora_core::uhlc::HLC;
use dora_message::{
    BuildId, DataflowId,
    cli_to_coordinator::ControlRequest,
    common::DaemonId,
    coordinator_to_cli::{
        CleanFailure, ControlRequestReply, DataflowIdAndName, DataflowList, DataflowListEntry,
        DataflowResult, DataflowStatus, LogLevel, LogMessage,
    },
    coordinator_to_daemon::{
        DaemonCoordinatorEvent, RegisterResult, StateCatchUpOperation, Timestamped,
    },
    daemon_to_coordinator::DataflowDaemonResult,
};
pub use events::{DaemonRequest, DataflowEvent, Event};
use eyre::{Result, WrapErr, bail, eyre};
use futures::{Future, Stream, StreamExt, future::join_all, stream::FuturesUnordered};
use futures_concurrency::stream::Merge;
use indexmap::IndexMap;
use log_subscriber::LogSubscriber;
use petname::petname;
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

/// Cap on the in-memory archived-dataflow history. Mirrors the local constant
/// used by the `DataflowFinishedOnDaemon` teardown path; lifted to module
/// scope so the spawn-timeout watchdog can use the same cap for archived
/// failed dataflows. Keep the two in sync.
const MAX_ARCHIVED_DATAFLOWS: usize = 200;

/// Cap on the in-memory per-dataflow result history (FIFO via `IndexMap`).
/// Finished dataflows normally have their result entry removed when archived,
/// but synthetic entries (reconnect / spawn-timeout watchdog) and results for
/// dataflows that never reach archival would otherwise accumulate without
/// bound. Capped at the same size as the archived history (dora-rs/dora#2027).
const MAX_DATAFLOW_RESULTS: usize = 200;

/// Cap on the in-memory finished-builds history (FIFO via `IndexMap`).
/// Lifted to module scope so the build-timeout watchdog and the
/// `DataflowBuildResult` handler share the same eviction policy (#1465).
const MAX_FINISHED_BUILDS: usize = 100;

/// Default deadline after which a distributed spawn that has not received
/// every daemon's `spawn_result` is considered stuck. The heartbeat-driven
/// watchdog fails such spawns and rolls back any daemons that already
/// reported success.
///
/// Chosen well above realistic per-daemon spawn time (even with `--uv`
/// Python venv preparation) so the watchdog only fires on genuine hangs.
/// Overridable at process startup via the `DORA_SPAWN_RESULT_TIMEOUT_SECS`
/// environment variable (parsed once into [`spawn_result_timeout`]; invalid
/// or zero values fall back to this default).
///
/// Rescue of [#1593](https://github.com/dora-rs/dora/pull/1593).
const SPAWN_RESULT_TIMEOUT_DEFAULT: Duration = Duration::from_secs(60);

/// Reads `DORA_SPAWN_RESULT_TIMEOUT_SECS` once and caches the result so
/// each heartbeat tick is not repeatedly parsing the env var. Invalid or
/// non-positive values fall back to [`SPAWN_RESULT_TIMEOUT_DEFAULT`].
fn spawn_result_timeout() -> Duration {
    use std::sync::OnceLock;
    static CACHED: OnceLock<Duration> = OnceLock::new();
    *CACHED.get_or_init(|| {
        std::env::var("DORA_SPAWN_RESULT_TIMEOUT_SECS")
            .ok()
            .and_then(|s| s.parse::<u64>().ok())
            .filter(|&n| n > 0)
            .map(Duration::from_secs)
            .unwrap_or(SPAWN_RESULT_TIMEOUT_DEFAULT)
    })
}

/// Default deadline after which a distributed build that has not received
/// a `build_result` from every assigned daemon is treated as terminally
/// failed by the watchdog. Mirrors [`SPAWN_RESULT_TIMEOUT_DEFAULT`] but is
/// far larger because cold `--uv` Rust builds legitimately take 5–10 min;
/// 20 min is a safety net for genuine hangs, not for slow-but-progressing
/// builds. Overridable via `DORA_BUILD_RESULT_TIMEOUT_SECS` (#1465).
const BUILD_RESULT_TIMEOUT_DEFAULT: Duration = Duration::from_secs(20 * 60);

/// Reads `DORA_BUILD_RESULT_TIMEOUT_SECS` once and caches the result, same
/// pattern as [`spawn_result_timeout`]. Invalid or non-positive values fall
/// back to [`BUILD_RESULT_TIMEOUT_DEFAULT`].
fn build_result_timeout() -> Duration {
    use std::sync::OnceLock;
    static CACHED: OnceLock<Duration> = OnceLock::new();
    *CACHED.get_or_init(|| {
        std::env::var("DORA_BUILD_RESULT_TIMEOUT_SECS")
            .ok()
            .and_then(|s| s.parse::<u64>().ok())
            .filter(|&n| n > 0)
            .map(Duration::from_secs)
            .unwrap_or(BUILD_RESULT_TIMEOUT_DEFAULT)
    })
}

pub(crate) mod artifacts;
mod control;
mod daemon_liveness;
mod events;
mod handlers;
mod log_subscriber;
#[cfg(feature = "metrics")]
mod otel_metrics;
mod params;
mod ready_barrier;
mod run;
mod spawn_build;
mod state;
#[cfg(test)]
mod tests;
mod timeout_streak;
mod topic_debug;
mod topic_subscriber;
mod topology;
mod traces;
mod ws_control;
mod ws_daemon;
mod ws_server;

pub(crate) use daemon_liveness::{
    apply_disconnect_actions, cancel_pending_restart,
    cleanup_disconnected_daemons_from_running_builds,
    cleanup_disconnected_daemons_from_running_dataflows, notify_daemons_about_disconnected_peers,
    reestablish_running_dataflow, send_heartbeat_with_timeout, status_report_should_stop_orphan,
    stop_orphaned_dataflow_on_daemon,
};
pub(crate) use params::{
    ensure_delete_param_forward_applied, ensure_set_param_forward_applied,
    handle_pruned_state_catchup_fallback, replay_persisted_params_for_daemon,
    schedule_param_replay_for_ready_dataflow,
};
pub(crate) use ready_barrier::{
    broadcast_all_nodes_ready, nodes_on_daemon, replay_all_nodes_ready,
};
pub(crate) use spawn_build::{
    MAX_BUFFERED_LOG_MESSAGES, buffer_log_message, cap_dataflow_results, check_build_timeouts,
    check_spawn_timeouts, finalize_build, handle_dataflow_spawn_result,
};
pub(crate) use topic_debug::{
    close_topic_subscribers_on_finish, restore_topic_debug_streams_for_daemon,
    start_topic_debug_stream, stop_topic_debug_stream, topic_debug_enabled,
    topic_outputs_by_daemon,
};
pub(crate) use topology::{
    ensure_add_mapping_applied, ensure_add_node_applied, ensure_remove_mapping_applied,
    ensure_remove_node_applied, ensure_replace_node_applied, expire_stopped_nodes,
    resolve_single_node,
};
pub(crate) use traces::{handle_get_trace_spans, handle_get_traces};

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

/// Testing-only entry point. Starts the coordinator without auth and without
/// registering a ctrl-c handler, allowing a custom store to be injected.
/// Useful for tests that run multiple coordinators in the same process.
/// Do NOT use in production.
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
        // Pin the exporter to `DORA_OTLP_ENDPOINT` when set; when unset, pass
        // `None` so the previous (endpoint-less) behaviour is preserved — the
        // exporter then resolves its target from the OTel-standard
        // `OTEL_EXPORTER_OTLP_ENDPOINT` env vars (defaulting to localhost).
        let endpoint = std::env::var("DORA_OTLP_ENDPOINT").ok();
        let provider = dora_metrics::init_metrics(endpoint.as_deref())?;
        opentelemetry::global::set_meter_provider(provider.clone());
        provider
    };
    #[cfg(feature = "metrics")]
    let otel_metrics = otel_metrics::new_shared();

    // DaemonId -> WS peer address, shared with the WS server so the
    // ResolveMachine reply can carry the target daemon's direct-TCP data
    // listener address.
    let daemon_peer_addrs: Arc<
        std::sync::RwLock<std::collections::HashMap<String, std::net::SocketAddr>>,
    > = Arc::new(std::sync::RwLock::new(std::collections::HashMap::new()));
    let (port, ws_shutdown, ws_future) = ws_server::serve(
        bind,
        ws_event_tx.clone(),
        clock.clone(),
        auth_token,
        artifact_store,
        store.clone(),
        daemon_peer_addrs.clone(),
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
            daemon_peer_addrs,
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
    daemon_peer_addrs: Arc<
        std::sync::RwLock<std::collections::HashMap<String, std::net::SocketAddr>>,
    >,
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
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
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
                            // Gathered here rather than before the match, so a
                            // version-mismatched daemon retrying in a loop does
                            // not make the coordinator walk its whole daemon map
                            // and clone every endpoint per attempt — on the
                            // serial event loop that is time no one gets back.
                            //
                            // Gathered before `add` below, so the joining daemon
                            // is not in the map yet and receives exactly the
                            // peers that preceded it; see
                            // `RegisterResult::Ok::peer_zenoh_endpoints`.
                            Ok(_) => RegisterResult::ok(
                                daemon_id.clone(),
                                daemon_connections.zenoh_endpoints_for(&daemon_id),
                            ),
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
                            if let Some(peer_addr) = connection.peer_addr {
                                daemon_peer_addrs
                                    .write()
                                    .unwrap_or_else(|e| e.into_inner())
                                    .insert(daemon_id.to_string(), peer_addr);
                            }
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
                } => match running_dataflows.entry(uuid) {
                    std::collections::hash_map::Entry::Occupied(mut entry) => {
                        let dataflow = entry.get_mut();
                        dataflow.pending_daemons.remove(&daemon_id);
                        dataflow
                            .exited_before_subscribe
                            .extend(exited_before_subscribe);
                        if dataflow.pending_daemons.is_empty() {
                            broadcast_all_nodes_ready(
                                uuid,
                                dataflow,
                                &mut daemon_connections,
                                &store,
                                &clock,
                            )
                            .await?;
                        }
                    }
                    std::collections::hash_map::Entry::Vacant(_) => {
                        tracing::warn!("dataflow not running on ReadyOnMachine");
                    }
                },
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
                                while archived_dataflows.len() > MAX_ARCHIVED_DATAFLOWS {
                                    archived_dataflows.shift_remove_index(0);
                                }
                                let mut finished_dataflow = entry.remove();

                                // Complete any pending restart on this dataflow.
                                // The restart was deferred in `initiate_restart` so that
                                // old nodes' Zenoh subscribers/declarations are fully
                                // torn down before new nodes spawn, avoiding the race where
                                // new nodes start before old nodes exit (dora-rs/dora#2082).
                                //
                                // This runs *before* close_topic_subscribers_on_finish
                                // intentionally: the restart block only touches the
                                // coordinator's `running_dataflows` map and `pending_restarts`
                                // map, it does not interact with `finished_dataflow` or its
                                // Zenoh-side state (which has already been cleaned up by the
                                // daemons — that's why we're in this handler).
                                if let Some(restart) = pending_restarts.remove(&uuid) {
                                    let name = restart.name.clone();
                                    match start_dataflow(
                                        None,
                                        dora_message::SessionId::generate(),
                                        restart.descriptor,
                                        None,
                                        restart.name,
                                        &mut daemon_connections,
                                        &clock,
                                        restart.uv,
                                        None,
                                    )
                                    .await
                                    {
                                        Ok(new_dataflow) => {
                                            let new_uuid = new_dataflow.uuid;
                                            // Persist new dataflow as Pending
                                            let mut new_df = new_dataflow;
                                            if let Err(e) = new_df
                                                .make_record(StoreDataflowStatus::Pending)
                                                .and_then(|r| store.put_dataflow(&r))
                                            {
                                                tracing::warn!(
                                                    "failed to persist restarted dataflow: {e}"
                                                );
                                            }
                                            running_dataflows.insert(new_uuid, new_df);
                                            let _ = restart.reply_sender.send(Ok(
                                                ControlRequestReply::DataflowRestarted {
                                                    old_uuid: uuid,
                                                    new_uuid,
                                                },
                                            ));
                                        }
                                        Err(err) => {
                                            tracing::error!(
                                                "failed to start dataflow during deferred restart of `{name:?}` ({uuid}): {err:#}"
                                            );
                                            let _ = restart.reply_sender.send(Err(eyre!(
                                                "failed to start restarted dataflow: {err:#}"
                                            )));
                                        }
                                    }
                                }

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
                                                // Normal end-of-life failure: not flagged terminal
                                                // because there is no concurrent path that could
                                                // resurrect a properly-finished dataflow.
                                                terminal: false,
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
                            // If the dataflow was previously archived by the
                            // spawn-timeout watchdog (round-6 Finding 2), merge
                            // the daemon's late-arriving completion result into
                            // the synthetic `dataflow_results` entry so the
                            // per-node details are surfaced via `dora list` /
                            // `dora check` instead of being silently dropped.
                            // Per-node `Err(FailedToSpawn(..))` entries
                            // (synthesized at watchdog time) are overwritten
                            // by the daemon's real per-node results where they
                            // overlap, giving the user the best available
                            // post-mortem info.
                            if archived_dataflows.contains_key(&uuid) {
                                let entry = dataflow_results.entry(uuid).or_default();
                                let existing =
                                    entry.entry(daemon_id.clone()).or_insert_with(|| {
                                        DataflowDaemonResult {
                                            timestamp: result.timestamp,
                                            node_results: BTreeMap::new(),
                                        }
                                    });
                                existing.timestamp = result.timestamp;
                                existing.node_results.extend(result.node_results);
                            } else {
                                tracing::warn!("dataflow not running on DataflowFinishedOnDaemon",);
                            }
                        }
                    }
                    // Bound finished-history growth (active multi-daemon entries
                    // are preserved). Done after the match so `running_dataflows`
                    // is no longer borrowed by the `entry(uuid)` scrutinee.
                    cap_dataflow_results(&mut dataflow_results, &running_dataflows);
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
                            // A pending restart already sent `StopDataflow` to
                            // the daemon(s) and is waiting for
                            // `DataflowFinishedOnDaemon` to spawn the new
                            // incarnation under a fresh UUID; `running_dataflows`
                            // still contains the old UUID in the meantime. An
                            // explicit `Stop` for that UUID means the caller
                            // wants the dataflow gone, not restarted — cancel
                            // the pending restart (erroring its caller) rather
                            // than letting it silently spawn a new incarnation
                            // after this stop reports success. Last-writer-wins,
                            // and unlike an outright rejection this doesn't
                            // block `--force` from ever landing while a
                            // long-`--grace-duration` restart is in flight.
                            cancel_pending_restart(
                                &mut pending_restarts,
                                dataflow_uuid,
                                format!(
                                    "dataflow `{dataflow_uuid}` was stopped before the restart could complete"
                                ),
                            );

                            // `dataflow_results` is filled incrementally, one
                            // entry per daemon, while a multi-daemon dataflow is
                            // still running on the others (see
                            // `DataflowFinishedOnDaemon`). Only take the
                            // already-stopped fast path when the dataflow is
                            // truly gone from `running_dataflows`; otherwise fall
                            // through to `stop_dataflow` so the daemons that are
                            // still running actually get told to stop. Mirrors
                            // the `Clean` handler's guard.
                            if !running_dataflows.contains_key(&dataflow_uuid)
                                && let Some(result) = dataflow_results.get(&dataflow_uuid)
                            {
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
                                // Same pending-restart cancellation as `Stop`
                                // — see the comment there for why.
                                cancel_pending_restart(
                                    &mut pending_restarts,
                                    dataflow_uuid,
                                    format!(
                                        "dataflow `{dataflow_uuid}` was stopped before the restart could complete"
                                    ),
                                );

                                // Same partial-completion guard as `Stop`: a
                                // still-running multi-daemon dataflow has a
                                // partial `dataflow_results` entry, but must
                                // still be stopped rather than reported done.
                                if !running_dataflows.contains_key(&dataflow_uuid)
                                    && let Some(result) = dataflow_results.get(&dataflow_uuid)
                                {
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
                            initiate_restart(
                                dataflow_uuid,
                                grace_duration,
                                force,
                                &mut running_dataflows,
                                &mut pending_restarts,
                                &mut daemon_connections,
                                &clock,
                                store.as_ref(),
                                reply_sender,
                            )
                            .await;
                        }
                        ControlRequest::RestartByName {
                            name,
                            grace_duration,
                            force,
                        } => match resolve_name(name, &running_dataflows, &archived_dataflows) {
                            Ok(dataflow_uuid) => {
                                initiate_restart(
                                    dataflow_uuid,
                                    grace_duration,
                                    force,
                                    &mut running_dataflows,
                                    &mut pending_restarts,
                                    &mut daemon_connections,
                                    &clock,
                                    store.as_ref(),
                                    reply_sender,
                                )
                                .await;
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
                                    // `node` arrives as a raw wire `String`, so it may be an
                                    // invalid node id. Validate it instead of using the panicking
                                    // `String -> NodeId` conversion, which would unwind the
                                    // coordinator's single event loop and take down every
                                    // daemon/CLI connection (control-plane DoS). See #3450 — the
                                    // node-id sub-case that #650's fix for #648 missed.
                                    let reply = match parse_logs_node_id(&node) {
                                        Ok(node_id) => retrieve_logs(
                                            &running_dataflows,
                                            &archived_dataflows,
                                            uuid,
                                            node_id,
                                            &mut daemon_connections,
                                            clock.new_timestamp(),
                                            tail,
                                        )
                                        .await
                                        .map(ControlRequestReply::Logs),
                                        Err(err) => Err(err),
                                    };
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
                            // Skip uuids still in `running_dataflows`: a
                            // partially-finished multi-daemon dataflow has a
                            // partial `dataflow_results` entry while it keeps
                            // running, and would otherwise be listed twice (once
                            // Running, once Finished/Failed) with contradictory
                            // statuses. It is already yielded above as Running.
                            let finished_failed = dataflow_results
                                .iter()
                                .filter(|(uuid, _)| !running_dataflows.contains_key(uuid))
                                .map(|(&uuid, results)| {
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
                        ControlRequest::Clean => {
                            // `dora clean` semantics (see #1835):
                            //
                            // * Only FULLY completed dataflows are eligible. For
                            //   multi-daemon dataflows `dataflow_results` is
                            //   populated incrementally as each daemon finishes,
                            //   while the dataflow stays in `running_dataflows`
                            //   until ALL daemons are gone. Cleaning a partial
                            //   entry would corrupt the final status: when the
                            //   last daemon finishes the reply is computed from
                            //   the (now-missing) entry and can default to
                            //   Succeeded even if an earlier daemon reported a
                            //   node failure.
                            //
                            // * Each cleaned entry is removed from the persisted
                            //   store so the on-disk state file doesn't grow
                            //   unboundedly. The persisted-store delete cascades
                            //   to associated `dora param` rows.
                            //
                            // * `finished_builds` is intentionally NOT touched —
                            //   clearing it would break concurrent `dora build`
                            //   calls with "unknown build id" errors.
                            //
                            // Phase A: enumerate completed candidates from BOTH
                            // `dataflow_results` AND `store.list_dataflows()` so
                            // a restarted coordinator can still reap historical
                            // Succeeded/Failed rows that only exist on disk
                            // (startup recovery intentionally does NOT reload
                            // them into memory — see the empty match arm at
                            // `StoreDataflowStatus::Succeeded | Failed` in the
                            // startup loop). Per-candidate tuple:
                            // (uuid, name, cli-facing status, in_memory).
                            let mut candidates: Vec<(Uuid, Option<String>, DataflowStatus, bool)> =
                                Vec::new();

                            for (uuid, results) in dataflow_results.iter() {
                                if running_dataflows.contains_key(uuid) {
                                    // Multi-daemon dataflow still completing —
                                    // keep partial results so the final status
                                    // is computed correctly when the last daemon
                                    // completes.
                                    continue;
                                }
                                let name =
                                    archived_dataflows.get(uuid).and_then(|d| d.name.clone());
                                let status = if results.values().all(|r| r.is_ok()) {
                                    DataflowStatus::Finished
                                } else {
                                    DataflowStatus::Failed
                                };
                                candidates.push((*uuid, name, status, true));
                            }

                            // Hard-fail if we can't enumerate the persisted
                            // store. With a partial view we cannot honor the
                            // "trim disk state" contract, and silently
                            // processing only the in-memory subset would let
                            // the CLI claim "nothing to clean" while
                            // historical rows still sit on disk untouched.
                            // The in-memory entries we would have processed
                            // stay in `dataflow_results`, so a subsequent
                            // `dora clean` (after the operator fixes the
                            // underlying store issue) reaps them on the next
                            // call. No state is mutated on this path.
                            let records = match store.list_dataflows() {
                                Ok(records) => records,
                                Err(e) => {
                                    let _ = reply_sender.send(Err(eyre!(
                                        "dora clean: failed to enumerate persisted \
                                         dataflows: {e}. No state was modified; the \
                                         next `dora clean` will retry once the \
                                         coordinator's store is healthy again."
                                    )));
                                    continue;
                                }
                            };
                            for record in records {
                                if running_dataflows.contains_key(&record.uuid) {
                                    continue;
                                }
                                if dataflow_results.contains_key(&record.uuid) {
                                    // Already covered by the in-memory pass;
                                    // skip to avoid double-counting.
                                    continue;
                                }
                                let status = match record.status {
                                    StoreDataflowStatus::Succeeded => DataflowStatus::Finished,
                                    StoreDataflowStatus::Failed { .. } => DataflowStatus::Failed,
                                    _ => continue,
                                };
                                candidates.push((record.uuid, record.name, status, false));
                            }

                            // Phase B: per-candidate, persist-first, then mutate
                            // in-memory state on success. Collect-then-mutate
                            // avoids borrow friction with two sources and makes
                            // the success/failure split obvious.
                            let mut cleaned: Vec<DataflowListEntry> = Vec::new();
                            let mut failed: Vec<CleanFailure> = Vec::new();
                            for (uuid, name, status, in_memory) in candidates {
                                let id = DataflowIdAndName { uuid, name };
                                if let Err(e) = store.delete_dataflow(&uuid) {
                                    tracing::warn!(
                                        "skipping clean for dataflow {uuid}: \
                                         persisted-store delete failed: {e}. \
                                         {state} preserved so a later `dora clean` \
                                         can retry.",
                                        state = if in_memory {
                                            "In-memory entry"
                                        } else {
                                            "Persisted record"
                                        }
                                    );
                                    failed.push(CleanFailure {
                                        id,
                                        error: e.to_string(),
                                    });
                                    continue;
                                }
                                if in_memory {
                                    dataflow_results.shift_remove(&uuid);
                                }
                                archived_dataflows.shift_remove(&uuid);
                                cleaned.push(DataflowListEntry { id, status });
                            }

                            cleaned.sort_by(|a, b| {
                                (a.id.name.as_deref(), a.id.uuid)
                                    .cmp(&(b.id.name.as_deref(), b.id.uuid))
                            });
                            failed.sort_by(|a, b| {
                                (a.id.name.as_deref(), a.id.uuid)
                                    .cmp(&(b.id.name.as_deref(), b.id.uuid))
                            });

                            let reply = Ok(ControlRequestReply::CleanResult {
                                cleaned: DataflowList(cleaned),
                                failed,
                            });
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
                            let reply: eyre::Result<ControlRequestReply> = async {
                                let target = resolve_param_target(
                                    &running_dataflows,
                                    store.as_ref(),
                                    &dataflow_id,
                                    &node_id,
                                )?;
                                let bytes = serde_json::to_vec(&value)
                                    .map_err(|e| eyre!("failed to serialize param value: {e}"))?;
                                // Persist first (source of truth), then attempt synchronous
                                // runtime forwarding. If forwarding fails, caller gets Error(...)
                                // but persisted value will be replayed on catch-up/reconnect.
                                store.put_node_param(&dataflow_id, &node_id, &key, &bytes)?;

                                if let ParamTarget::Running { daemon_id } = target {
                                    let df = running_dataflows.get_mut(&dataflow_id).ok_or_else(
                                        || {
                                            eyre!(
                                                "param persisted in store but running dataflow `{dataflow_id}` disappeared before runtime forwarding for node `{node_id}`"
                                            )
                                        },
                                    )?;
                                    df.append_state_log(StateCatchUpOperation::SetParam {
                                        node_id: node_id.clone(),
                                        key: key.clone(),
                                        value: value.clone(),
                                    });

                                    let msg = serde_json::to_vec(&Timestamped {
                                        inner: DaemonCoordinatorEvent::SetParam {
                                            dataflow_id,
                                            node_id: node_id.clone(),
                                            key: key.clone(),
                                            value: value.clone(),
                                        },
                                        timestamp: clock.new_timestamp(),
                                    })
                                    .map_err(|e| {
                                        eyre!("failed to serialize SetParam event for node `{node_id}`: {e}")
                                    })?;

                                    let conn =
                                        daemon_connections.get_mut(&daemon_id).ok_or_else(|| {
                                            eyre!(
                                                "param persisted in store but daemon `{daemon_id}` is not connected"
                                            )
                                        })?;
                                    let reply_raw = conn.send_and_receive(&msg).await.map_err(|e| {
                                        eyre!(
                                            "failed to forward SetParam to daemon `{daemon_id}` for node `{node_id}`: {e}"
                                        )
                                    })?;
                                    ensure_set_param_forward_applied(&reply_raw, &node_id)?;
                                }
                                Ok(ControlRequestReply::ParamSet)
                            }
                            .await;
                            let _ = reply_sender.send(reply);
                        }
                        ControlRequest::DeleteParam {
                            dataflow_id,
                            node_id,
                            key,
                        } => {
                            let reply: eyre::Result<ControlRequestReply> = async {
                                let target = resolve_param_target(
                                    &running_dataflows,
                                    store.as_ref(),
                                    &dataflow_id,
                                    &node_id,
                                )?;
                                // Persist first (source of truth), then attempt synchronous
                                // runtime forwarding. If forwarding fails, caller gets Error(...)
                                // but delete is still reflected in persisted state/catch-up log.
                                store.delete_node_param(&dataflow_id, &node_id, &key)?;

                                if let ParamTarget::Running { daemon_id } = target {
                                    let df = running_dataflows.get_mut(&dataflow_id).ok_or_else(
                                        || {
                                            eyre!(
                                                "param deleted in store but running dataflow `{dataflow_id}` disappeared before runtime forwarding for node `{node_id}`"
                                            )
                                        },
                                    )?;
                                    df.append_state_log(StateCatchUpOperation::DeleteParam {
                                        node_id: node_id.clone(),
                                        key: key.clone(),
                                    });

                                    let msg = serde_json::to_vec(&Timestamped {
                                        inner: DaemonCoordinatorEvent::DeleteParam {
                                            dataflow_id,
                                            node_id: node_id.clone(),
                                            key: key.clone(),
                                        },
                                        timestamp: clock.new_timestamp(),
                                    })
                                    .map_err(|e| {
                                        eyre!(
                                            "failed to serialize DeleteParam event for node `{node_id}`: {e}"
                                        )
                                    })?;

                                    let conn =
                                        daemon_connections.get_mut(&daemon_id).ok_or_else(|| {
                                            eyre!(
                                                "param deleted in store but daemon `{daemon_id}` is not connected"
                                            )
                                        })?;
                                    let reply_raw = conn.send_and_receive(&msg).await.map_err(|e| {
                                        eyre!(
                                            "failed to forward DeleteParam to daemon `{daemon_id}` for node `{node_id}`: {e}"
                                        )
                                    })?;
                                    ensure_delete_param_forward_applied(&reply_raw, &node_id)?;
                                }
                                Ok(ControlRequestReply::ParamDeleted)
                            }
                            .await;
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

                                        // See `resolve_single_node` for what
                                        // the running descriptor contributes
                                        // (env carry-through #2919,
                                        // single-operator output prefixing
                                        // #2877).
                                        match resolve_single_node(node, &dataflow.descriptor) {
                                            Ok((node_id, resolved_node)) => {
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
                                                                    Ok(reply_raw) => {
                                                                        // Validate the daemon reply is
                                                                        // specifically an `AddNodeResult`
                                                                        // (not just any non-error reply)
                                                                        // before committing state. Without
                                                                        // this, a `SetParamResult` or an
                                                                        // explicit `AddNodeResult(Err)`
                                                                        // would still be reported as
                                                                        // applied and corrupt the dataflow
                                                                        // state (#1682, rescue of #1757).
                                                                        // The validator's error is folded
                                                                        // into the `Err` arm of `result`
                                                                        // (via explicit `Err(e) => Err(e)`
                                                                        // below — no `?`), which the
                                                                        // coordinator's main loop sends
                                                                        // back to the CLI as
                                                                        // `ControlRequestReply::Error`.
                                                                        // Addresses phil-opp's review of
                                                                        // #1757 (do not tear down the
                                                                        // event loop on a recoverable
                                                                        // per-request failure).
                                                                        match ensure_add_node_applied(
                                                                            &reply_raw, &node_id,
                                                                        ) {
                                                                            Ok(()) => {
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
                                                                                // Clear any stale Stopped/
                                                                                // Finalized state for this
                                                                                // node id so the new
                                                                                // incarnation's metrics push
                                                                                // isn't blocked by the prior
                                                                                // stop's `node_stopped_at` /
                                                                                // `node_finalized` entries.
                                                                                dataflow
                                                                                    .node_stopped_at
                                                                                    .remove(&node_id);
                                                                                dataflow
                                                                                    .node_finalized
                                                                                    .remove(&node_id);
                                                                                dataflow
                                                                                    .node_metrics
                                                                                    .remove(&node_id);
                                                                                Ok(
                                                                                    ControlRequestReply::NodeAdded {
                                                                                        dataflow_id,
                                                                                        node_id,
                                                                                    },
                                                                                )
                                                                            }
                                                                            Err(e) => Err(e),
                                                                        }
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
                                            // `resolve_single_node` already
                                            // prefixes the resolve context.
                                            Err(e) => Err(e),
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
                                                        Ok(reply_raw) => {
                                                            match ensure_remove_node_applied(
                                                                &reply_raw, &node_id,
                                                            ) {
                                                                Ok(()) => {
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
                                                                            .retain(|n| {
                                                                                n.id != node_id
                                                                            });
                                                                        dataflow
                                                                            .nodes
                                                                            .remove(&node_id);
                                                                    }
                                                                    Ok(
                                                                        ControlRequestReply::NodeRemoved {
                                                                            dataflow_id,
                                                                            node_id,
                                                                        },
                                                                    )
                                                                }
                                                                Err(e) => Err(e),
                                                            }
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
                        ControlRequest::ReplaceNode {
                            dataflow_id,
                            node,
                            grace_duration,
                        } => {
                            let result = async {
                                // Route to the daemon that OWNS the id — a
                                // replace targets an existing node, unlike
                                // AddNode's first-daemon placement.
                                let original_node = node.clone();
                                // Resolve inside the borrow so the running
                                // descriptor can be passed by reference; the
                                // borrow ends before `daemon_connections` is
                                // taken mutably below.
                                let (daemon_id, uv, node_id, resolved_node) = {
                                    let dataflow =
                                        running_dataflows.get(&dataflow_id).ok_or_else(|| {
                                            eyre!("no running dataflow with ID {dataflow_id}")
                                        })?;
                                    let daemon_id = dataflow
                                        .node_to_daemon
                                        .get(&node.id)
                                        .cloned()
                                        .ok_or_else(|| {
                                            eyre!(
                                                "node '{}' not found in dataflow {dataflow_id}; \
                                                 use `dora node add` to add a new node",
                                                node.id
                                            )
                                        })?;
                                    let (node_id, resolved_node) =
                                        resolve_single_node(node, &dataflow.descriptor)?;
                                    (daemon_id, dataflow.uv, node_id, resolved_node)
                                };
                                let msg = serde_json::to_vec(&Timestamped {
                                    inner: DaemonCoordinatorEvent::ReplaceNode {
                                        dataflow_id,
                                        node: resolved_node.clone(),
                                        // Ship the original YAML-shape node so
                                        // the daemon can assign the descriptor
                                        // entry wholesale, mirroring the
                                        // `*existing = original_node` commit
                                        // this arm does below.
                                        unresolved_node: original_node.clone(),
                                        uv,
                                        grace_duration,
                                    },
                                    timestamp: clock.new_timestamp(),
                                })?;
                                let conn = daemon_connections
                                    .get_mut(&daemon_id)
                                    .ok_or_else(|| eyre!("no connection for daemon {daemon_id}"))?;
                                let reply_raw = conn
                                    .send_and_receive(&msg)
                                    .await
                                    .map_err(|e| eyre!("daemon dispatch failed: {e}"))?;
                                // Commit coordinator state only after the
                                // daemon confirms with the specific reply
                                // variant (#1682 contract).
                                ensure_replace_node_applied(&reply_raw, &node_id)?;
                                if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                                    if let Some(existing) = dataflow
                                        .descriptor
                                        .nodes
                                        .iter_mut()
                                        .find(|n| n.id == node_id)
                                    {
                                        *existing = original_node;
                                    } else {
                                        dataflow.descriptor.nodes.push(original_node);
                                    }
                                    dataflow.nodes.insert(node_id.clone(), resolved_node);
                                    // Clear stale lifecycle markers so the new
                                    // incarnation's metrics are not suppressed
                                    // (same set AddNode clears).
                                    dataflow.node_stopped_at.remove(&node_id);
                                    dataflow.node_finalized.remove(&node_id);
                                    dataflow.node_metrics.remove(&node_id);
                                }
                                Ok(ControlRequestReply::NodeReplaced {
                                    dataflow_id,
                                    node_id,
                                })
                            }
                            .await;
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
                                                Ok(reply_raw) => {
                                                    // Validate the daemon reply is specifically an
                                                    // `AddMappingResult` before reporting success,
                                                    // mirroring the #1682 / #1873 rescue for AddNode.
                                                    let src =
                                                        format!("{source_node}/{source_output}");
                                                    let tgt =
                                                        format!("{target_node}/{target_input}");
                                                    match ensure_add_mapping_applied(
                                                        &reply_raw, &src, &tgt,
                                                    ) {
                                                        Ok(()) => {
                                                            Ok(ControlRequestReply::MappingAdded {
                                                                dataflow_id,
                                                                source_node,
                                                                source_output,
                                                                target_node,
                                                                target_input,
                                                            })
                                                        }
                                                        Err(e) => Err(e),
                                                    }
                                                }
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
                                                Ok(reply_raw) => {
                                                    let src =
                                                        format!("{source_node}/{source_output}");
                                                    let tgt =
                                                        format!("{target_node}/{target_input}");
                                                    match ensure_remove_mapping_applied(
                                                        &reply_raw, &src, &tgt,
                                                    ) {
                                                        Ok(()) => Ok(
                                                            ControlRequestReply::MappingRemoved {
                                                                dataflow_id,
                                                                source_node,
                                                                source_output,
                                                                target_node,
                                                                target_input,
                                                            },
                                                        ),
                                                        Err(e) => Err(e),
                                                    }
                                                }
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
                    } else if archived_dataflows.contains_key(&dataflow_id) {
                        // Dataflow already finished before the CLI could subscribe.
                        // Acknowledge the subscription so the CLI doesn't error, then
                        // drop `sender` immediately — the closed channel signals EOF.
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
                // Also drives expired-stopped-node cleanup so dataflows
                // with no live nodes (and therefore no NodeMetrics push)
                // still eventually shed their zombie rows.
                for dataflow in running_dataflows.values_mut() {
                    expire_stopped_nodes(dataflow);
                }
                let mut disconnected = BTreeSet::new();
                // Send the per-daemon heartbeats concurrently rather than
                // awaiting each 500 ms-bounded send in turn. Each send awaits a
                // bounded WS mpsc, so a single backpressured (slow/half-dead but
                // not yet 30 s-stale) daemon would otherwise stall this control
                // loop for up to 500 ms before the next daemon is even tried —
                // and N such daemons stall it for N × 500 ms, delaying every
                // spawn/stop/logs request. `join_all` bounds the whole tick at
                // ~500 ms regardless of daemon count. Mirrors `destroy_daemons`.
                let mut heartbeats = Vec::new();
                for (machine_id, connection) in daemon_connections.iter_mut() {
                    let elapsed = connection.last_heartbeat.elapsed();
                    if elapsed > Duration::from_secs(15) {
                        tracing::warn!(
                            "no heartbeat message from machine `{machine_id}` since {elapsed:?}"
                        )
                    }
                    if elapsed > Duration::from_secs(30) {
                        disconnected.insert(machine_id.clone());
                        continue;
                    }
                    heartbeats.push(send_heartbeat_with_timeout(
                        machine_id.clone(),
                        connection,
                        clock.new_timestamp(),
                    ));
                }
                for (machine_id, disconnect) in join_all(heartbeats).await {
                    if disconnect {
                        disconnected.insert(machine_id);
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
                    let disconnect_actions = cleanup_disconnected_daemons_from_running_dataflows(
                        &mut running_dataflows,
                        &disconnected,
                        &mut pending_restarts,
                    );
                    apply_disconnect_actions(
                        disconnect_actions,
                        &mut running_dataflows,
                        &mut daemon_connections,
                        &store,
                        &clock,
                    )
                    .await?;
                    cleanup_disconnected_daemons_from_running_builds(
                        &mut running_builds,
                        &mut finished_builds,
                        &disconnected,
                    );
                    notify_daemons_about_disconnected_peers(
                        &disconnected,
                        &mut daemon_connections,
                        &clock,
                    )
                    .await?;
                }
                // Spawn timeout watchdog: detect distributed starts that are
                // stuck waiting for one or more `spawn_result` reports and
                // release `wait_for_spawn` waiters with a clear error rather
                // than letting them hang on the client-side RPC deadline.
                // Triggers on either failure mode:
                //   1. A daemon accepted the spawn RPC but its internal flow
                //      is hung (still heartbeating, never reports back).
                //   2. Pending daemons all disconnected; the disconnect path
                //      above cleared their entries from `pending_spawn_results`
                //      but does not itself fire `spawn_result`.
                // Rescue of #1593 (issue #1592).
                //
                // SAFETY net only: 60 s is well above realistic per-daemon
                // spawn time even with `--uv` Python env preparation.
                check_spawn_timeouts(
                    &mut running_dataflows,
                    &mut archived_dataflows,
                    &mut dataflow_results,
                    &mut daemon_connections,
                    &mut pending_restarts,
                    &clock,
                    store.as_ref(),
                )
                .await;

                // Build timeout watchdog — mirror of `check_spawn_timeouts`
                // for `running_builds`. Releases `wait_for_build` waiters
                // that would otherwise hang on the client-side RPC deadline
                // when a daemon participating in `dora build` disconnects
                // or otherwise never reports its `build_result`. #1465.
                check_build_timeouts(
                    &mut running_builds,
                    &mut finished_builds,
                    &clock,
                    build_result_timeout(),
                )
                .await;

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
                                        // Coordinator gave up waiting for daemon reconnect;
                                        // mark terminal so a daemon that eventually returns
                                        // doesn't promote this back to Running via reconcile.
                                        terminal: true,
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
            Event::DaemonZenohEndpoint {
                daemon_id,
                connection_id,
                endpoint,
            } => {
                // Same guard as `DaemonExit` below (#2392): a report still in
                // flight from a connection that has since been replaced would
                // otherwise overwrite the live endpoint with a dead one, and
                // every daemon registering afterwards would be handed it.
                if daemon_connections.connection_id_of(&daemon_id) == Some(connection_id) {
                    match &endpoint {
                        Some(ep) => {
                            tracing::debug!("daemon `{daemon_id}` confirmed zenoh endpoint `{ep}`")
                        }
                        // The daemon advertised an endpoint when it registered
                        // and then failed to bind it. Withdrawing stops the
                        // coordinator handing a dead endpoint to every daemon
                        // that registers from here on.
                        None => tracing::warn!(
                            "daemon `{daemon_id}` withdrew its zenoh endpoint: its listener \
                             did not bind, so other daemons cannot reach it directly"
                        ),
                    }
                    daemon_connections.set_zenoh_endpoint(&daemon_id, endpoint);
                } else {
                    tracing::debug!(
                        "ignoring zenoh endpoint {endpoint:?} from a superseded \
                         connection of daemon `{daemon_id}`"
                    );
                }
            }
            Event::Log(message) => {
                // `dataflow_id`/`build_id` are `Copy`, so match them by value to
                // leave `message` free to move into `buffer_log_message`.
                if let Some(dataflow_id) = message.dataflow_id {
                    if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                        if dataflow.log_subscribers.is_empty() {
                            if buffer_log_message(&mut dataflow.buffered_log_messages, message) {
                                tracing::warn!(
                                    "log buffer full for dataflow {dataflow_id} \
                                     ({MAX_BUFFERED_LOG_MESSAGES} messages); dropping further \
                                     messages until a log subscriber attaches"
                                );
                            }
                        } else {
                            send_log_message(&mut dataflow.log_subscribers, &message).await;
                        }
                    }
                } else if let Some(build_id) = message.build_id
                    && let Some(build) = running_builds.get_mut(&build_id)
                {
                    if build.log_subscribers.is_empty() {
                        if buffer_log_message(&mut build.buffered_log_messages, message) {
                            tracing::warn!(
                                "log buffer full for build {build_id} \
                                 ({MAX_BUFFERED_LOG_MESSAGES} messages); dropping further \
                                 messages until a log subscriber attaches"
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
            Event::DaemonExit {
                daemon_id,
                connection_id,
            } => {
                // Only act on the exit if the connection_id matches the
                // currently-registered connection. A named daemon that
                // reconnects before the old connection's handler task
                // delivers its DaemonExit must NOT evict the new connection
                // or trigger spurious dataflow cleanup (#2392).
                if daemon_connections.connection_id_of(&daemon_id) == Some(connection_id) {
                    tracing::info!("Daemon `{daemon_id}` exited");
                    daemon_connections.remove(&daemon_id);
                    if let Err(e) = store.unregister_daemon(&daemon_id) {
                        tracing::warn!("failed to persist daemon unregistration: {e}");
                    }
                    let disconnected = BTreeSet::from([daemon_id]);
                    let disconnect_actions = cleanup_disconnected_daemons_from_running_dataflows(
                        &mut running_dataflows,
                        &disconnected,
                        &mut pending_restarts,
                    );
                    apply_disconnect_actions(
                        disconnect_actions,
                        &mut running_dataflows,
                        &mut daemon_connections,
                        &store,
                        &clock,
                    )
                    .await?;
                    // Mirror the watchdog disconnect path: fail any in-flight
                    // build the exited daemon was still part of. Without this, a
                    // multi-daemon `dora build` where one daemon exits cleanly
                    // mid-build never sees its pending set resolve (the exited
                    // daemon's entry lingers), so the build is not finalized by
                    // the `DataflowBuildResult` handler and instead hangs until
                    // `check_build_timeouts` fires the 20-minute deadline. #1465.
                    cleanup_disconnected_daemons_from_running_builds(
                        &mut running_builds,
                        &mut finished_builds,
                        &disconnected,
                    );
                    notify_daemons_about_disconnected_peers(
                        &disconnected,
                        &mut daemon_connections,
                        &clock,
                    )
                    .await?;
                } else {
                    tracing::debug!(
                        "ignoring stale DaemonExit for `{daemon_id}` \
                         (connection replaced by reconnect)"
                    );
                }
            }
            Event::NodeMetrics {
                dataflow_id,
                metrics,
                network,
            } => {
                // Store metrics for this dataflow
                if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                    // Sweep expired stopped-node entries before applying
                    // fresh metrics, so `dora node list` eventually stops
                    // showing rows for nodes the daemon last reported as
                    // stopped > NODE_STOPPED_GRACE ago.
                    expire_stopped_nodes(dataflow);
                    for (node_id, node_metrics) in &metrics {
                        // A NodeStopped event is authoritative: skip any
                        // in-flight metrics row for the same node that was
                        // captured by the daemon's pre-stop snapshot. The
                        // check uses `node_finalized` (covers Stopped AND
                        // Failed) rather than `node_stopped_at` (Stopped
                        // only) so a delayed metrics push cannot revive a
                        // crashed-Failed row back to a stale Running.
                        if dataflow.node_finalized.contains(node_id) {
                            continue;
                        }
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
                            // Same authority as the in-memory table above: a
                            // finalized node's delayed metrics push is a stale
                            // pre-exit snapshot. Skip it here too, so OTEL does
                            // not record a fresh data point that resurrects the
                            // node's CPU/memory time series one push past its
                            // death.
                            if dataflow.node_finalized.contains(node_id) {
                                continue;
                            }
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
                        let Some(build) = running_builds.remove(&build_id) else {
                            tracing::error!("build {build_id} disappeared from running_builds");
                            continue;
                        };
                        finalize_build(build_id, build, &mut finished_builds);
                    }
                }
                None => {
                    // Build no longer in `running_builds` — usually means
                    // the watchdog (`check_build_timeouts`) already marked
                    // it as terminally failed and moved it to
                    // `finished_builds`. Late replies are expected in that
                    // case; warn but do not resurrect (the cached failure
                    // result is the authoritative one). #1465.
                    tracing::warn!(
                        build_id = %build_id,
                        daemon_id = %daemon_id,
                        "received DataflowBuildResult for a build no longer in `running_builds` (already finalized or timed out — ignoring)"
                    );
                }
            },
            Event::DataflowSpawnResult {
                dataflow_id,
                daemon_id,
                result,
            } => {
                handle_dataflow_spawn_result(
                    dataflow_id,
                    daemon_id,
                    result,
                    &mut running_dataflows,
                    &mut archived_dataflows,
                    &mut dataflow_results,
                    &mut daemon_connections,
                    &mut pending_restarts,
                    &clock,
                    store.as_ref(),
                )
                .await;
            }
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
                //
                // Exception: dataflows that are present in `archived_dataflows`
                // have been declared terminally failed by the spawn-timeout
                // watchdog (or any other archive-on-failure path). Promoting
                // their store status back to Running would contradict the
                // terminal verdict the user already received via
                // `wait_for_spawn`. Round-7 Finding 1.
                for entry in &reported_dataflows {
                    let df_id = &entry.dataflow_id;
                    if archived_dataflows.contains_key(df_id) {
                        tracing::warn!(
                            "daemon {daemon_id} reports archived (terminally-failed) \
                             dataflow {df_id} as running; skipping reconcile so the \
                             watchdog's Failed verdict is preserved"
                        );
                        // The daemon kept these nodes alive but the coordinator
                        // has terminally failed the dataflow; stop the orphans
                        // rather than leave them unmanageable (#2029 P3).
                        stop_orphaned_dataflow_on_daemon(
                            *df_id,
                            &daemon_id,
                            &mut daemon_connections,
                            &clock,
                        )
                        .await;
                        continue;
                    }
                    match store.get_dataflow(df_id) {
                        Ok(Some(mut record)) => match record.status {
                            // Failed records: only promote to Running if NOT
                            // terminal. The `terminal: true` marker (set by
                            // the spawn-timeout watchdog and the recovery
                            // timeout) survives coordinator restarts in the
                            // store, so a wedged daemon that reconnects
                            // post-restart cannot resurrect a terminally-
                            // failed dataflow (round-8 Finding 1).
                            // Non-terminal Failed records preserve the
                            // pre-#1854 behaviour where a daemon's report
                            // could override a coordinator-side Failed
                            // (e.g. the multi-daemon partial-failure case
                            // where another daemon is still running).
                            StoreDataflowStatus::Failed {
                                terminal: false, ..
                            }
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
                                // Rebuild the live in-memory entry so the
                                // surviving nodes are visible + manageable again
                                // (#2029 P1) — store status alone doesn't drive
                                // `dora list` / `stop` / `logs`.
                                if reestablish_running_dataflow(
                                    &mut running_dataflows,
                                    &record,
                                    &daemon_id,
                                    &entry.running_nodes,
                                ) && let Some(df) = running_dataflows.get(&record.uuid)
                                {
                                    // Barrier released while this daemon was
                                    // gone; nothing else will tell it (#2998).
                                    // Best-effort, like every other step in
                                    // this reconciliation: a failed send here
                                    // means one daemon's nodes stay parked,
                                    // which must not take down the coordinator
                                    // and every other dataflow with it.
                                    if let Err(e) = replay_all_nodes_ready(
                                        record.uuid,
                                        df,
                                        &daemon_id,
                                        &mut daemon_connections,
                                        &store,
                                        &clock,
                                    )
                                    .await
                                    {
                                        tracing::warn!(
                                            "failed to replay ready barrier to daemon \
                                             `{daemon_id}` for dataflow {}: {e:#}",
                                            record.uuid
                                        );
                                    }
                                }
                            }
                            StoreDataflowStatus::Running => {
                                // Already `Running` in the store but possibly
                                // missing from the live map (e.g. a later report,
                                // or a coordinator restart that loaded the record
                                // but not the in-memory entry). Idempotent.
                                if reestablish_running_dataflow(
                                    &mut running_dataflows,
                                    &record,
                                    &daemon_id,
                                    &entry.running_nodes,
                                ) && let Some(df) = running_dataflows.get(&record.uuid)
                                {
                                    // Barrier released while this daemon was
                                    // gone; nothing else will tell it (#2998).
                                    // Best-effort, like every other step in
                                    // this reconciliation: a failed send here
                                    // means one daemon's nodes stay parked,
                                    // which must not take down the coordinator
                                    // and every other dataflow with it.
                                    if let Err(e) = replay_all_nodes_ready(
                                        record.uuid,
                                        df,
                                        &daemon_id,
                                        &mut daemon_connections,
                                        &store,
                                        &clock,
                                    )
                                    .await
                                    {
                                        tracing::warn!(
                                            "failed to replay ready barrier to daemon \
                                             `{daemon_id}` for dataflow {}: {e:#}",
                                            record.uuid
                                        );
                                    }
                                }
                            }
                            status if status_report_should_stop_orphan(&status) => {
                                // Terminal state (success, watchdog failure, or
                                // equivalent coordinator-side verdict). Daemon's
                                // report is ignored to preserve the verdict the
                                // user already received.
                                tracing::warn!(
                                    "daemon {daemon_id} reports terminal \
                                     dataflow {df_id} as running; skipping reconcile",
                                );
                                // Stop the orphaned nodes the daemon kept alive
                                // past the coordinator's terminal verdict, so
                                // they don't run unmanageable forever (#2029 P3).
                                stop_orphaned_dataflow_on_daemon(
                                    *df_id,
                                    &daemon_id,
                                    &mut daemon_connections,
                                    &clock,
                                )
                                .await;
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
                    // Skip if a spawn is already in-flight for this daemon
                    if df.pending_spawn_results.contains(&daemon_id) {
                        continue;
                    }
                    // Skip dataflows that the spawn-timeout watchdog (or
                    // any other terminal-failure path) has already marked
                    // as failed. Without this, a daemon whose
                    // `DaemonStatusReport` lacks a watchdog-failed
                    // dataflow would have the dataflow re-spawned here,
                    // resurrecting a terminally-failed dataflow in memory
                    // even though `spawn_result` is `Cached(Err)` and the
                    // store says Failed. See PR #1854 round-4 Finding 1.
                    if df.spawn_result.is_terminal_error() {
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
            Event::DaemonNodeStopped {
                daemon_id,
                dataflow_id,
                node_id,
                clean_stop,
            } => {
                // Daemon reports a node has stopped and will not be
                // restarted. Mark the cached metrics so `dora node list`
                // shows `Stopped` (clean exit) or `Failed` (final-failure
                // exit) instead of the frozen-Running snapshot, and arm
                // the expiry side-band so the row eventually disappears
                // (see `expire_stopped_nodes`).
                //
                // Ownership check: the daemon that owns this node per
                // `node_to_daemon` is the only one allowed to declare it
                // stopped. Drops stale events from a previous incarnation
                // (e.g. stop → remove → re-add on a different daemon) and
                // foreign-daemon spoofing.
                if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                    match dataflow.node_to_daemon.get(&node_id) {
                        Some(owner) if owner == &daemon_id => {}
                        Some(other) => {
                            tracing::warn!(
                                %dataflow_id, %node_id,
                                "ignoring NodeStopped from daemon `{daemon_id}`: node \
                                 is owned by `{other}`"
                            );
                            continue;
                        }
                        None => {
                            tracing::debug!(
                                %dataflow_id, %node_id,
                                "ignoring NodeStopped: node no longer in dataflow"
                            );
                            continue;
                        }
                    }
                    let status = if clean_stop {
                        dora_message::daemon_to_coordinator::NodeStatus::Stopped
                    } else {
                        // Crash / restart-policy exhaustion: surface as
                        // `Failed` so `dora doctor` still counts it. A
                        // clean `Stopped` would be invisible to the
                        // doctor's healthy/degraded/failed bucketing.
                        dora_message::daemon_to_coordinator::NodeStatus::Failed
                    };
                    let entry = dataflow
                        .node_metrics
                        .entry(node_id.clone())
                        .or_insert_with(|| dora_message::daemon_to_coordinator::NodeMetrics {
                            pid: 0,
                            cpu_usage: 0.0,
                            memory_bytes: 0,
                            disk_read_bytes: None,
                            disk_write_bytes: None,
                            restart_count: 0,
                            broken_inputs: Vec::new(),
                            status: status.clone(),
                            pending_messages: 0,
                        });
                    entry.status = status;
                    entry.pid = 0;
                    entry.cpu_usage = 0.0;
                    entry.memory_bytes = 0;
                    entry.disk_read_bytes = None;
                    entry.disk_write_bytes = None;
                    // Authoritative finalize marker: protects against late
                    // in-flight metrics pushes overwriting the row, for
                    // BOTH Stopped and Failed.
                    dataflow.node_finalized.insert(node_id.clone());
                    // Only arm the auto-expire side-band for clean Stopped
                    // rows. Failed rows must stay visible until the dataflow
                    // is stopped/destroyed — otherwise a crashed node would
                    // disappear from `dora node list` / `dora doctor` after
                    // the 60s grace, hiding the failure this PR is meant
                    // to surface.
                    if clean_stop {
                        dataflow.node_stopped_at.insert(node_id, Instant::now());
                    } else {
                        // Defensive: if the same node id was previously
                        // marked Stopped (then re-spawned and now Failed),
                        // clear the stale stopped_at so the Failed row
                        // isn't swept on the next tick.
                        dataflow.node_stopped_at.remove(&node_id);
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

/// Restart a running dataflow: stop it, then re-start with the stored descriptor.
#[allow(clippy::too_many_arguments)]
/// Phase-1 of a two-phase restart: sends `StopDataflow` to all daemons
/// and registers a `PendingRestart`. The actual start of the replacement
/// dataflow is deferred until all daemons report `DataflowFinishedOnDaemon`
/// (processed in the event loop), guaranteeing old nodes' Zenoh subscribers
/// and declarations are torn down before new nodes spawn (dora-rs/dora#2082).
async fn initiate_restart(
    dataflow_uuid: uuid::Uuid,
    grace_duration: Option<Duration>,
    force: bool,
    running_dataflows: &mut HashMap<uuid::Uuid, RunningDataflow>,
    pending_restarts: &mut HashMap<uuid::Uuid, PendingRestart>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
    store: &dyn CoordinatorStore,
    reply_sender: tokio::sync::oneshot::Sender<eyre::Result<ControlRequestReply>>,
) {
    // Guard against double-restart FIRST: reject duplicate requests before
    // touching any daemon state or sending StopDataflow side effects.
    if pending_restarts.contains_key(&dataflow_uuid) {
        let _ = reply_sender.send(Err(eyre!(
            "dataflow `{dataflow_uuid}` is already being restarted – wait for it to finish"
        )));
        return;
    }
    // 1. Extract descriptor, name, and uv from the running dataflow
    let (descriptor, name, uv) = {
        let Some(df) = running_dataflows.get(&dataflow_uuid) else {
            let _ = reply_sender.send(Err(eyre!(
                "no running dataflow with UUID `{dataflow_uuid}`"
            )));
            return;
        };
        (df.descriptor.clone(), df.name.clone(), df.uv)
    };

    // 2. Stop the old dataflow
    match stop_dataflow(
        running_dataflows,
        dataflow_uuid,
        daemon_connections,
        clock.new_timestamp(),
        grace_duration,
        force,
    )
    .await
    {
        Ok(dataflow) => {
            if let Err(e) = dataflow
                .make_record(StoreDataflowStatus::Stopping)
                .and_then(|r| store.put_dataflow(&r))
            {
                tracing::warn!("failed to persist dataflow stopping: {e}");
            }
        }
        Err(err) => {
            let _ = reply_sender.send(Err(err));
            return;
        }
    }

    // 3. Register the deferred restart — the event loop will complete it
    //    when all daemons report `DataflowFinishedOnDaemon`.
    //    Stale entries are drained in the daemon-disconnect cleanup path
    //    so a daemon crash after StopDataflow does not hang the caller.
    pending_restarts.insert(
        dataflow_uuid,
        PendingRestart {
            descriptor,
            name,
            uv,
            reply_sender,
        },
    );
    // Logged (rather than left silent) so a test can poll for the exact
    // moment the restart becomes cancellable via a `Stop`, instead of
    // guessing a fixed delay.
    tracing::info!(dataflow = %dataflow_uuid, "restart pending; waiting for dataflow to finish stopping");
}
