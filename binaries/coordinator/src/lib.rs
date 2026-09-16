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
    handlers::{handle_destroy, stop_dataflow},
    state::{ArchivedDataflow, CachedResult, PendingRestart, RunningBuild, RunningDataflow},
};
pub use control::ControlEvent;
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
pub use dora_coordinator_store::{self, CoordinatorStore, InMemoryStore};
use dora_core::uhlc::HLC;
use dora_message::{
    BuildId, DataflowId, common::DaemonId, coordinator_to_cli::ControlRequestReply,
    daemon_to_coordinator::DataflowDaemonResult,
};
pub use events::{DaemonRequest, DataflowEvent, Event};
use eyre::{Result, WrapErr, eyre};
use futures::{Future, Stream, StreamExt, stream::FuturesUnordered};
use futures_concurrency::stream::Merge;
use indexmap::IndexMap;
pub(crate) use state::{DaemonConnections, resolve_param_target};
use std::{
    collections::{BTreeMap, HashMap},
    net::SocketAddr,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio_stream::wrappers::ReceiverStream;

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
mod control_requests;
mod daemon_events;
mod daemon_liveness;
mod dataflow_events;
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

/// The coordinator's event-loop state.
///
/// `start_inner` builds one of these and then dispatches every event to a
/// method on it; the methods live in `control_requests`, `daemon_events` and
/// `dataflow_events`. Fields are `pub(crate)` so those `impl` blocks can
/// borrow them independently.
pub(crate) struct Coordinator {
    pub(crate) running_builds: HashMap<BuildId, RunningBuild>,
    pub(crate) finished_builds: IndexMap<BuildId, CachedResult>,
    pub(crate) running_dataflows: HashMap<DataflowId, RunningDataflow>,
    pub(crate) pending_restarts: HashMap<DataflowId, PendingRestart>,
    pub(crate) dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    pub(crate) archived_dataflows: IndexMap<DataflowId, ArchivedDataflow>,
    pub(crate) daemon_connections: DaemonConnections,
    pub(crate) clock: Arc<HLC>,
    pub(crate) store: Arc<dyn CoordinatorStore>,
    pub(crate) span_store: SpanStore,
    pub(crate) daemon_peer_addrs: Arc<std::sync::RwLock<HashMap<String, SocketAddr>>>,
    #[cfg(feature = "metrics")]
    pub(crate) otel_metrics: otel_metrics::SharedMetrics,
    /// Aborts the event stream on `dora down` / Ctrl-C.
    pub(crate) abort_handle: futures::stream::AbortHandle,
}

impl Coordinator {
    pub(crate) async fn handle_ctrl_c(&mut self) -> eyre::Result<()> {
        tracing::info!("Destroying coordinator after receiving Ctrl-C signal");
        handle_destroy(
            &mut self.running_dataflows,
            &mut self.daemon_connections,
            &self.abort_handle,
            &self.clock,
            self.store.as_ref(),
        )
        .await?;
        Ok(())
    }
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

    let mut coordinator = Coordinator {
        running_builds: HashMap::new(),
        finished_builds: IndexMap::new(),
        running_dataflows: HashMap::new(),
        pending_restarts: HashMap::new(),
        dataflow_results: IndexMap::new(),
        archived_dataflows: IndexMap::new(),
        daemon_connections: DaemonConnections::default(),
        clock,
        store,
        span_store,
        daemon_peer_addrs,
        #[cfg(feature = "metrics")]
        otel_metrics,
        abort_handle,
    };

    while let Some(event) = events.next().await {
        // used below for measuring the event handling duration
        let start = Instant::now();
        let event_kind = event.kind();

        if event.log() {
            tracing::trace!("Handling event {event:?}");
        }
        match event {
            Event::Daemon(event) => coordinator.handle_daemon_request(event).await?,
            Event::Dataflow { uuid, event } => {
                coordinator.handle_dataflow_event(uuid, event).await?
            }
            Event::Control(event) => coordinator.handle_control_event(event).await?,
            Event::DaemonHeartbeatInterval => coordinator.handle_heartbeat_interval().await?,
            Event::CtrlC => coordinator.handle_ctrl_c().await?,
            Event::DaemonHeartbeat {
                daemon_id,
                ft_stats,
            } => {
                coordinator
                    .handle_daemon_heartbeat(daemon_id, ft_stats)
                    .await?
            }
            Event::DaemonZenohEndpoint {
                daemon_id,
                connection_id,
                endpoint,
            } => {
                coordinator
                    .handle_daemon_zenoh_endpoint(daemon_id, connection_id, endpoint)
                    .await?
            }
            Event::Log(message) => coordinator.handle_log(message).await?,
            Event::TopicDebugData {
                dataflow_id,
                subscription_ids,
                payload,
            } => {
                coordinator
                    .handle_topic_debug_data(dataflow_id, subscription_ids, payload)
                    .await?
            }
            Event::DaemonExit {
                daemon_id,
                connection_id,
            } => {
                coordinator
                    .handle_daemon_exit(daemon_id, connection_id)
                    .await?
            }
            Event::NodeMetrics {
                dataflow_id,
                metrics,
                network,
            } => {
                coordinator
                    .handle_node_metrics(dataflow_id, metrics, network)
                    .await?
            }
            Event::DataflowBuildResult {
                build_id,
                daemon_id,
                result,
            } => {
                coordinator
                    .handle_build_result(build_id, daemon_id, result)
                    .await?
            }
            Event::DataflowSpawnResult {
                dataflow_id,
                daemon_id,
                result,
            } => {
                coordinator
                    .handle_spawn_result(dataflow_id, daemon_id, result)
                    .await?
            }
            Event::DaemonStatusReport {
                daemon_id,
                running_dataflows,
            } => {
                coordinator
                    .handle_daemon_status_report(daemon_id, running_dataflows)
                    .await?
            }
            Event::DaemonStateCatchUpAck {
                daemon_id,
                dataflow_id,
                ack_sequence,
            } => {
                coordinator
                    .handle_state_catch_up_ack(daemon_id, dataflow_id, ack_sequence)
                    .await?
            }
            Event::DaemonNodeStopped {
                daemon_id,
                dataflow_id,
                node_id,
                clean_stop,
            } => {
                coordinator
                    .handle_daemon_node_stopped(daemon_id, dataflow_id, node_id, clean_stop)
                    .await?
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
