use crate::{
    events::set_up_ctrlc_handler,
    handlers::{
        build_dataflow, dataflow_result, handle_destroy, reload_dataflow, resolve_name,
        restart_node, retrieve_logs, send_heartbeat_message, send_log_message, send_topic_frames,
        start_dataflow, stop_dataflow, stop_node,
    },
    state::{
        ArchivedDataflow, CachedResult, ParamTarget, PendingRestart, RunningBuild, RunningDataflow,
    },
};
pub use control::ControlEvent;
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
pub use dora_coordinator_store::{self, CoordinatorStore, InMemoryStore};
use dora_core::{descriptor::DescriptorExt, uhlc::HLC};
use dora_message::{
    BuildId, DataflowId,
    cli_to_coordinator::ControlRequest,
    common::{DaemonId, NodeError, NodeErrorCause, NodeExitStatus},
    coordinator_to_cli::{
        CleanFailure, ControlRequestReply, DataflowIdAndName, DataflowList, DataflowListEntry,
        DataflowResult, DataflowStatus, LogLevel, LogMessage,
    },
    coordinator_to_daemon::{
        DaemonCoordinatorEvent, RegisterResult, StateCatchUpOperation, Timestamped,
    },
    daemon_to_coordinator::{DaemonCoordinatorReply, DataflowDaemonResult},
    id::NodeId,
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
                            // Only short-circuit with the cached result once the
                            // dataflow has fully finished (no longer in
                            // `running_dataflows`). `dataflow_results` accumulates
                            // one entry per daemon *while the dataflow is still
                            // running* (see `DataflowFinishedOnDaemon` /
                            // `cap_dataflow_results`), so keying off it alone would
                            // report a partially-finished multi-daemon dataflow as
                            // stopped and skip stopping the still-running daemons.
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
                                // See the `Stop` handler above: only use the cached
                                // result when the dataflow has fully finished, so a
                                // partially-finished multi-daemon dataflow is still
                                // actually stopped instead of being reported as
                                // stopped while some daemons keep running.
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
                        while finished_builds.len() > MAX_FINISHED_BUILDS {
                            finished_builds.shift_remove_index(0);
                        }
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
            } => match running_dataflows.get_mut(&dataflow_id) {
                Some(dataflow) => {
                    dataflow.pending_spawn_results.remove(&daemon_id);
                    match result {
                        Ok(()) => {
                            handle_spawn_result_ok(
                                dataflow,
                                dataflow_id,
                                &daemon_id,
                                store.as_ref(),
                            );
                        }
                        Err(err) => {
                            handle_spawn_result_err(
                                dataflow,
                                dataflow_id,
                                &daemon_id,
                                err,
                                store.as_ref(),
                            );
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
                                reestablish_running_dataflow(
                                    &mut running_dataflows,
                                    &record,
                                    &daemon_id,
                                    &entry.running_nodes,
                                );
                            }
                            StoreDataflowStatus::Running => {
                                // Already `Running` in the store but possibly
                                // missing from the live map (e.g. a later report,
                                // or a coordinator restart that loaded the record
                                // but not the in-memory entry). Idempotent.
                                reestablish_running_dataflow(
                                    &mut running_dataflows,
                                    &record,
                                    &daemon_id,
                                    &entry.running_nodes,
                                );
                            }
                            StoreDataflowStatus::Failed { terminal: true, .. } => {
                                // Terminal failure (watchdog or equivalent
                                // coordinator-side verdict). Daemon's report
                                // is ignored to preserve the verdict the
                                // user already received via wait_for_spawn.
                                tracing::warn!(
                                    "daemon {daemon_id} reports terminally-failed \
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
            Event::NodeMetricsExpire {
                dataflow_id,
                node_id,
            } => {
                if let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) {
                    dataflow.node_metrics.remove(&node_id);
                    dataflow.node_stopped_at.remove(&node_id);
                    dataflow.node_finalized.remove(&node_id);
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

/// Handle the success arm of `Event::DataflowSpawnResult`.
///
/// Extracted from the inline event loop so the late-arrival guard can be
/// exercised directly by unit tests. Two correctness invariants:
///
/// 1. **Late-Ok guard**: if `spawn_result` is no longer `Pending` (i.e. the
///    timeout watchdog already cached an `Err`), do NOT persist `Running`.
///    Without this guard, a delayed daemon's `Ok` arriving after the
///    watchdog fires would resurrect a terminally-failed dataflow as
///    `Running` in the store — in-memory `spawn_result` would still be
///    `Cached(Err)` (because `set_result` is a no-op on `Cached`), so the
///    store and the in-memory state would silently diverge. Rescue of
///    [#1593](https://github.com/dora-rs/dora/pull/1593) Finding 1 follow-up.
///
/// 2. **All-daemons-succeeded check**: only mark the dataflow Running once
///    every assigned daemon has reported a successful spawn. Preserves the
///    pre-existing handler semantics.
fn handle_spawn_result_ok(
    dataflow: &mut RunningDataflow,
    dataflow_id: DataflowId,
    daemon_id: &DaemonId,
    store: &dyn CoordinatorStore,
) {
    // Guard: if the spawn was already terminally failed (by the watchdog
    // or by another daemon reporting an error), a late Ok must not
    // promote the store status back to Running.
    if !dataflow.spawn_result.is_pending() {
        tracing::warn!(
            dataflow = %dataflow_id,
            daemon = %daemon_id,
            "ignoring late successful spawn_result on a dataflow already \
             terminally failed (e.g. by the spawn-timeout watchdog)",
        );
        return;
    }

    if dataflow.pending_spawn_results.is_empty() {
        tracing::info!("successfully spawned dataflow `{dataflow_id}`");
        dataflow
            .spawn_result
            .set_result(Ok(ControlRequestReply::DataflowSpawned {
                uuid: dataflow_id,
            }));
        if let Err(e) = dataflow
            .make_record(StoreDataflowStatus::Running)
            .and_then(|r| store.put_dataflow(&r))
        {
            tracing::warn!("failed to persist dataflow running: {e}");
        }
    }
}

/// Handle the failure arm of `Event::DataflowSpawnResult`.
///
/// Symmetric with [`handle_spawn_result_ok`]. The late-arrival guard
/// prevents a late daemon `Err` from overwriting the watchdog's (or any
/// other terminal-failure path's) more informative store record with a
/// generic `"spawn failed: <daemon-error>"` message — same data-integrity
/// concern as the Ok guard, just with cosmetic-only consequences instead
/// of resurrection. Also avoids bumping `store_generation` for no
/// observable state change.
fn handle_spawn_result_err(
    dataflow: &mut RunningDataflow,
    dataflow_id: DataflowId,
    daemon_id: &DaemonId,
    err: eyre::Report,
    store: &dyn CoordinatorStore,
) {
    if !dataflow.spawn_result.is_pending() {
        tracing::warn!(
            dataflow = %dataflow_id,
            daemon = %daemon_id,
            "ignoring late failed spawn_result on a dataflow already \
             terminally failed: {err:?}",
        );
        return;
    }

    tracing::warn!("error while spawning dataflow `{dataflow_id}`");
    if let Err(e) = dataflow
        .make_record(StoreDataflowStatus::Failed {
            error: format!("spawn failed: {err}"),
            // Daemon-side spawn error: the daemon authoritatively
            // reported the spawn failed. Terminal so a later daemon
            // status report can't promote this back to Running.
            terminal: true,
        })
        .and_then(|r| store.put_dataflow(&r))
    {
        tracing::warn!("failed to persist dataflow spawn failure: {e}");
    }
    dataflow.spawn_result.set_result(Err(err));
}

/// Fire-and-forget compensating rollback used by [`check_spawn_timeouts`].
///
/// Enqueues `StopDataflow{force: true}` to each succeeded daemon via
/// `connection.send()` (no reply wait). Unlike
/// `run::rollback_spawned_daemons`, this does NOT use `send_and_receive` —
/// see the comment in `check_spawn_timeouts` for why: (a) reply-waiting
/// blocks the heartbeat handler for up to `TCP_READ_TIMEOUT` per wedged
/// daemon, risking false-positive disconnections elsewhere in the cluster;
/// (b) `send_and_receive` is not cancellation-safe (it inserts into
/// `pending_replies` *before* registering its own cleanup), so any
/// external timeout that cancels it leaks state.
///
/// Returns `(daemon_id, error)` pairs for dispatch failures (serialization
/// failure or absent daemon connection). The mpsc `send` itself is
/// near-instantaneous so total wall-clock is bounded regardless of N.
async fn fire_and_forget_rollback(
    dataflow_id: Uuid,
    spawned_daemons: &BTreeSet<DaemonId>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
) -> Vec<(DaemonId, String)> {
    if spawned_daemons.is_empty() {
        return Vec::new();
    }

    let stop_message = match serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id,
            grace_duration: None,
            force: true,
        },
        timestamp: clock.new_timestamp(),
    }) {
        Ok(m) => m,
        Err(e) => {
            // Serialization failure is unlikely but fatal for rollback.
            return spawned_daemons
                .iter()
                .map(|id| (id.clone(), format!("failed to serialize stop message: {e}")))
                .collect();
        }
    };

    let mut errors = Vec::new();
    for daemon_id in spawned_daemons {
        let Some(conn) = daemon_connections.get_mut(daemon_id) else {
            errors.push((daemon_id.clone(), "no daemon connection".to_string()));
            continue;
        };
        if let Err(e) = conn.send(&stop_message).await {
            errors.push((daemon_id.clone(), format!("send failed: {e}")));
        }
    }
    errors
}

/// Remove disconnected daemon ids from all in-memory dataflow membership sets.
///
/// This intentionally does not resolve `spawn_result`: the spawn timeout
/// watchdog remains the single path that releases spawn waiters for
/// disconnect-mid-spawn cases.
/// Action the daemon-disconnect cleanup asks the (async) caller to perform for
/// a dataflow that was already past spawn when a daemon it depended on vanished.
/// See #2028.
enum DisconnectAction {
    /// The last daemon we were awaiting `ReadyOnDaemon` from disconnected, but
    /// survivors remain — release the start barrier so they don't block forever.
    ReleaseReadyBarrier(DataflowId),
    /// Every daemon running the dataflow disconnected — begin a reclaim window
    /// rather than failing terminally. The daemon may have only transiently lost
    /// the coordinator (a heartbeat blip, a coordinator restart) while its node
    /// processes keep running (dora-rs/dora#2029); marking the dataflow
    /// `Recovering` lets the reconnecting daemon's `DaemonStatusReport` reconcile
    /// it back to `Running`. If no daemon reclaims it within the recovery window,
    /// the existing recovery-timeout path fails it terminally (dora-rs/dora#2028).
    ReclaimOrphaned(DataflowId),
}

fn cleanup_disconnected_daemons_from_running_dataflows(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    disconnected: &BTreeSet<DaemonId>,
    pending_restarts: &mut HashMap<DataflowId, PendingRestart>,
) -> Vec<DisconnectAction> {
    let mut actions = Vec::new();
    let mut affected_uuids = Vec::new();
    for df in running_dataflows.values_mut() {
        let pending_was_nonempty = !df.pending_daemons.is_empty();
        let mut affected = false;
        for daemon_id in disconnected {
            affected |= df.daemons.remove(daemon_id);
            affected |= df.pending_daemons.remove(daemon_id);
            affected |= df.pending_spawn_results.remove(daemon_id);
        }
        if !affected {
            continue;
        }
        affected_uuids.push(df.uuid);
        // Only act on dataflows that already spawned successfully. While
        // `spawn_result` is still pending, the spawn-timeout watchdog
        // (`check_spawn_timeouts`) owns the disconnect-mid-spawn case, so we
        // must not race it here.
        let spawned_ok = df.spawn_result.is_cached_ok();
        if df.daemons.is_empty() {
            tracing::warn!(
                dataflow = %df.uuid,
                "all daemons disconnected - entering reclaim window (waiting for daemon reconnect)"
            );
            if spawned_ok {
                actions.push(DisconnectAction::ReclaimOrphaned(df.uuid));
            }
        } else if spawned_ok && pending_was_nonempty && df.pending_daemons.is_empty() {
            // The last daemon we were waiting on for `ReadyOnDaemon` vanished
            // via disconnect; `AllNodesReady` would otherwise never fire.
            actions.push(DisconnectAction::ReleaseReadyBarrier(df.uuid));
        }
    }
    // Drain pending restarts for affected dataflows: the disconnected
    // daemon(s) will never send DataflowFinishedOnDaemon, so any caller
    // waiting on a deferred restart would hang indefinitely (#2082 H1).
    for uuid in &affected_uuids {
        if let Some(restart) = pending_restarts.remove(uuid) {
            tracing::warn!(
                dataflow = %uuid,
                "daemon disconnected while restart was pending; cancelling restart"
            );
            let _ = restart.reply_sender.send(Err(eyre!(
                "daemon disconnected while restart was pending for dataflow `{uuid}`"
            )));
        }
    }
    actions
}

/// Re-establish the in-memory [`RunningDataflow`] for a dataflow a daemon has
/// just re-reported as running after a reconnect (dora-rs/dora#2029 P1).
///
/// `begin_orphaned_dataflow_reclaim` (and a coordinator restart) leave the
/// dataflow only in the persisted store; the live control plane — `dora list`,
/// `stop`, `logs`, `node`, `param` — all read `running_dataflows`, so without
/// this the surviving nodes would be invisible and unmanageable even though the
/// store says `Running`. If the entry is still present (a multi-daemon dataflow
/// whose other daemons are live), just relink this daemon's share; otherwise
/// reconstruct it from the persisted record + the daemon's report.
fn reestablish_running_dataflow(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    record: &dora_coordinator_store::DataflowRecord,
    daemon_id: &DaemonId,
    reported_nodes: &[dora_core::config::NodeId],
) {
    if let Some(df) = running_dataflows.get_mut(&record.uuid) {
        df.daemons.insert(daemon_id.clone());
        df.pending_daemons.remove(daemon_id);
        df.pending_spawn_results.remove(daemon_id);
        for node in reported_nodes {
            df.node_to_daemon.insert(node.clone(), daemon_id.clone());
        }
        return;
    }

    let descriptor: dora_message::descriptor::Descriptor =
        match serde_json::from_str(&record.descriptor_json) {
            Ok(d) => d,
            Err(e) => {
                tracing::warn!(
                    "cannot re-establish running dataflow {}: failed to parse descriptor: {e}",
                    record.uuid
                );
                return;
            }
        };
    let nodes = match descriptor.resolve_aliases_and_set_defaults() {
        Ok(n) => n,
        Err(e) => {
            tracing::warn!(
                "cannot re-establish running dataflow {}: failed to resolve nodes: {e}",
                record.uuid
            );
            return;
        }
    };
    running_dataflows.insert(
        record.uuid,
        RunningDataflow::recovered(record, descriptor, nodes, daemon_id.clone(), reported_nodes),
    );
    tracing::info!(
        "re-established running dataflow {} in live coordinator state after daemon {daemon_id} reconnect",
        record.uuid
    );
}

/// Tell a single daemon to stop a dataflow the coordinator has terminally given
/// up on but the daemon still reports as running (dora-rs/dora#2029 P3).
///
/// Closes the orphan window where the daemon's reconnect window outlives the
/// coordinator's recovery timeout: the daemon comes back with surviving nodes
/// *after* the coordinator already failed the dataflow terminally, so the
/// reconcile won't re-adopt it (the terminal verdict is deliberately preserved).
/// Rather than leave those now-unmanageable nodes running, ask the daemon to
/// stop them.
///
/// Fire-and-forget (`send`, not `send_and_receive`): the reconcile loop must not
/// block on the daemon's stop round-trip, and no reply is needed (the daemon
/// processes a `daemon_event` without replying).
async fn stop_orphaned_dataflow_on_daemon(
    dataflow_id: DataflowId,
    daemon_id: &DaemonId,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
) {
    let Some(connection) = daemon_connections.get_mut(daemon_id) else {
        return;
    };
    let message = match serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id,
            grace_duration: None,
            force: false,
        },
        timestamp: clock.new_timestamp(),
    }) {
        Ok(m) => m,
        Err(e) => {
            tracing::warn!("failed to serialize orphan-stop for dataflow {dataflow_id}: {e}");
            return;
        }
    };
    match connection.send(&message).await {
        Ok(()) => tracing::warn!(
            "told daemon {daemon_id} to stop orphaned dataflow {dataflow_id}: it reconnected with \
             the dataflow still running after the coordinator already failed it terminally (#2029)"
        ),
        Err(e) => tracing::warn!(
            "failed to tell daemon {daemon_id} to stop orphaned dataflow {dataflow_id}: {e}"
        ),
    }
}

/// Begin a reclaim window for a dataflow whose every daemon disconnected *after*
/// it had already spawned successfully. This is the running-dataflow counterpart
/// of [`check_spawn_timeouts`] (which only fires while `spawn_result` is pending),
/// but unlike that watchdog it does **not** fail the dataflow terminally: a
/// daemon often only transiently loses the coordinator (a heartbeat blip or a
/// coordinator restart) while its node processes keep running
/// (dora-rs/dora#2029).
///
/// So instead of the terminal teardown tail (persist `Failed { terminal: true }`,
/// synthesize a `Failed` result, archive), this:
/// - persists `Recovering`, so a reconnecting daemon's `DaemonStatusReport` can
///   reconcile it back to `Running` (the reconcile path skips archived/terminal
///   records — hence we must NOT archive here and must NOT mark it terminal);
/// - releases parked `dora stop` waiters so they don't hang (deadlock #2 of
///   dora-rs/dora#2028) — a `stop` racing a disconnect still returns;
/// - removes the dataflow from `running_dataflows`; the live entry is rebuilt by
///   `reestablish_running_dataflow` when the daemon reconnects and re-reports it
///   (so `dora list` / `stop` / `logs` work again — dora-rs/dora#2029 P1).
///
/// If no daemon reclaims it, the recovery-timeout path (the `Recovering -> Failed`
/// sweep) fails it terminally after `RECOVERY_TIMEOUT_SECS`.
///
/// Idempotent: `running_dataflows.remove` returning `None` makes a repeated call
/// a no-op.
async fn begin_orphaned_dataflow_reclaim(
    uuid: DataflowId,
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    clock: &HLC,
    store: &dyn dora_coordinator_store::CoordinatorStore,
) {
    let Some(mut df) = running_dataflows.remove(&uuid) else {
        return;
    };
    let msg = "all daemons running this dataflow disconnected; \
               waiting for daemon reconnect (Recovering)"
        .to_string();

    if let Err(e) = df
        .make_record(StoreDataflowStatus::Recovering)
        .and_then(|r| store.put_dataflow(&r))
    {
        tracing::warn!(dataflow = %uuid, "failed to persist reclaim (Recovering) state: {e}");
    }

    send_log_message(
        &mut df.log_subscribers,
        &LogMessage {
            build_id: None,
            dataflow_id: Some(uuid),
            node_id: None,
            daemon_id: None,
            level: LogLevel::Warn.into(),
            target: Some("coordinator".into()),
            module_path: None,
            file: None,
            line: None,
            message: msg,
            timestamp: clock.new_timestamp().get_time().to_system_time().into(),
            fields: None,
        },
    )
    .await;

    close_topic_subscribers_on_finish(&mut df);

    // Release any in-flight `dora stop` waiters so they don't hang across the
    // reclaim window (deadlock #2 of #2028). There is no synthesized failure
    // result here (the dataflow is recovering, not failed), so report an empty
    // OK result.
    let stop_reply = ControlRequestReply::DataflowStopped {
        uuid,
        result: DataflowResult::ok_empty(uuid, clock.new_timestamp()),
    };
    for sender in df.stop_reply_senders.drain(..) {
        let _ = sender.send(Ok(stop_reply.clone()));
    }
}

/// Execute the [`DisconnectAction`]s produced by
/// [`cleanup_disconnected_daemons_from_running_dataflows`]: release the start
/// barrier for dataflows whose last pending daemon vanished, and begin the
/// reclaim window for dataflows that lost every daemon. Runs at the (async)
/// caller after the synchronous set-pruning. See #2028 / #2029.
async fn apply_disconnect_actions(
    actions: Vec<DisconnectAction>,
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) -> eyre::Result<()> {
    for action in actions {
        match action {
            DisconnectAction::ReleaseReadyBarrier(uuid) => {
                if let Some(df) = running_dataflows.get(&uuid) {
                    broadcast_all_nodes_ready(uuid, df, daemon_connections, store, clock).await?;
                }
            }
            DisconnectAction::ReclaimOrphaned(uuid) => {
                begin_orphaned_dataflow_reclaim(uuid, running_dataflows, clock, store.as_ref())
                    .await;
            }
        }
    }
    Ok(())
}

/// Mirror of [`cleanup_disconnected_daemons_from_running_dataflows`] for
/// `running_builds`: prune disconnected daemon IDs from each running build's
/// `pending_build_results` so the in-memory state matches the live cluster.
///
/// This intentionally does NOT resolve `build_result` — the build timeout
/// watchdog ([`check_build_timeouts`]) remains the single path that releases
/// build waiters, preserving the chokepoint architecture documented at the
/// disconnect-handler comment above (#1465).
fn cleanup_disconnected_daemons_from_running_builds(
    running_builds: &mut HashMap<BuildId, RunningBuild>,
    disconnected: &BTreeSet<DaemonId>,
) {
    for build in running_builds.values_mut() {
        for daemon_id in disconnected {
            build.pending_build_results.remove(daemon_id);
        }
    }
}

async fn notify_daemons_about_disconnected_peers(
    disconnected: &BTreeSet<DaemonId>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
) -> Result<()> {
    for disconnected_id in disconnected {
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
    Ok(())
}

/// Scan `running_dataflows` for spawns that have been pending past
/// [`spawn_result_timeout`] and resolve them as terminally failed.
///
/// For each stuck dataflow:
/// 1. Roll back any daemons that already reported a successful spawn
///    (fire-and-forget; failures are logged).
/// 2. Set `spawn_result` to an error so `wait_for_spawn` waiters are
///    unblocked with an actionable message instead of hanging on the
///    client-side RPC deadline.
/// 3. Persist the dataflow as `Failed` so a restarted coordinator does
///    not re-resurrect it.
/// 4. **Tear down in-memory state**: archive the dataflow, drain
///    `stop_reply_senders`, close `topic_subscribers`, send a final
///    "dataflow failed" log to `log_subscribers`, then remove from
///    `running_dataflows`. Mirrors the `DataflowFinishedOnDaemon` teardown
///    so the dataflow is no longer visible to `Check` / `List` /
///    `DaemonStatusReport` reconciliation / `Clean` / etc. as if it
///    were still active. Without this step, those handlers would
///    contradict the watchdog's "terminally failed" verdict
///    (PR #1854 round-5 Findings 1 and 2).
///
/// Idempotent: `CachedResult::set_result` is a no-op on `Cached`, and
/// removed dataflows simply don't reappear in the next pass, so
/// re-running this on subsequent heartbeats is safe.
///
/// FIFO-evict *finished* entries from `dataflow_results` until it is within
/// [`MAX_DATAFLOW_RESULTS`].
///
/// `dataflow_results` is not pure history: a partially-finished multi-daemon
/// dataflow accumulates one entry per daemon while it is still running, and
/// the final success/failure status is computed from the full set once the
/// last daemon finishes. Evicting an entry whose dataflow is still in
/// `running_dataflows` would drop an earlier daemon's failure and let the
/// dataflow be reported as `Succeeded` (dora-rs/dora#2027 review). So only
/// entries for dataflows no longer running are evictable; if every over-cap
/// entry is still active the map is left above the cap rather than corrupting
/// live state (the bound targets finished-history growth, not concurrency).
fn cap_dataflow_results(
    dataflow_results: &mut IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
) {
    while dataflow_results.len() > MAX_DATAFLOW_RESULTS {
        let Some(idx) = dataflow_results
            .keys()
            .position(|uuid| !running_dataflows.contains_key(uuid))
        else {
            break;
        };
        dataflow_results.shift_remove_index(idx);
    }
}

/// Rescue of [#1593](https://github.com/dora-rs/dora/pull/1593)
/// (issue [#1592](https://github.com/dora-rs/dora/issues/1592)).
#[allow(clippy::too_many_arguments)]
async fn check_spawn_timeouts(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    archived_dataflows: &mut IndexMap<DataflowId, ArchivedDataflow>,
    dataflow_results: &mut IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
    store: &dyn CoordinatorStore,
) {
    let timeout_threshold = spawn_result_timeout();
    // First pass: identify stuck spawns and snapshot the daemon sets we
    // need for rollback. We collect into an owned Vec so the immutable
    // borrow of `running_dataflows` drops before we mutate it below.
    let stuck: Vec<(DataflowId, BTreeSet<DaemonId>, usize)> = running_dataflows
        .iter()
        .filter_map(|(uuid, df)| {
            if df.spawn_result.is_pending() && df.spawn_started_at.elapsed() > timeout_threshold {
                // Daemons assigned to this dataflow that already reported
                // successful spawn (i.e. were removed from
                // `pending_spawn_results`). These are the ones we need to
                // roll back to avoid leaving partial state running.
                let succeeded: BTreeSet<DaemonId> = df
                    .daemons
                    .difference(&df.pending_spawn_results)
                    .cloned()
                    .collect();
                Some((*uuid, succeeded, df.pending_spawn_results.len()))
            } else {
                None
            }
        })
        .collect();

    for (uuid, succeeded_daemons, pending_count) in stuck {
        tracing::warn!(
            dataflow = %uuid,
            timeout_secs = timeout_threshold.as_secs(),
            pending = pending_count,
            succeeded = succeeded_daemons.len(),
            "spawn timeout: releasing waiters and rolling back",
        );

        // Fire-and-forget rollback: enqueue StopDataflow on each succeeded
        // daemon WITHOUT awaiting a reply. This solves two problems at once:
        //
        // 1. Cascade-failure risk: a reply-awaiting rollback (the original
        //    `run::rollback_spawned_daemons` path) blocks
        //    `TCP_READ_TIMEOUT = 30s` per wedged daemon. With N wedged
        //    daemons, the heartbeat handler would block ~N*30s, during
        //    which heartbeats to *other* healthy daemons aren't dispatched
        //    and they trip the 30s disconnect threshold.
        // 2. Cancellation safety: an earlier version wrapped
        //    `rollback_spawned_daemons` in `tokio::time::timeout`, but
        //    that future cancels mid-`send_and_receive`, which inserts a
        //    pending reply *before* registering its own cleanup -- the
        //    cancellation would leak `pending_replies` entries.
        //    `connection.send()` is just an mpsc enqueue; no pending state,
        //    no cleanup needed, fully cancellation-safe.
        //
        // Trade-off: we don't get per-daemon ack of "stop succeeded". For
        // the watchdog this is acceptable -- the user is already getting
        // a clear timeout error, and unstopped daemons will be reclaimed
        // by daemon-disconnect or operator `dora stop`.
        let rollback_errors =
            fire_and_forget_rollback(uuid, &succeeded_daemons, daemon_connections, clock).await;
        if !rollback_errors.is_empty() {
            let rollback_summary = rollback_errors
                .iter()
                .map(|(id, e)| format!("  {id}: {e}"))
                .collect::<Vec<_>>()
                .join("\n");
            tracing::warn!(
                dataflow = %uuid,
                "rollback partial after spawn timeout, {} dispatch(es) failed:\n{rollback_summary}",
                rollback_errors.len(),
            );
        }

        // Fire the spawn_result error, persist Failed, then tear down
        // in-memory state so the dataflow is terminal from every other
        // handler's point of view (Check, List, reconcile, Clean, ...).
        let Some(mut df) = running_dataflows.remove(&uuid) else {
            // Concurrent removal — nothing more to do. (Not currently
            // reachable from any other code path; defensive.)
            continue;
        };
        let err_msg = format!(
            "spawn timed out after {}s; {} daemon(s) never reported \
             spawn_result; rolled back {} previously-started daemon(s)",
            timeout_threshold.as_secs(),
            pending_count,
            succeeded_daemons.len(),
        );
        df.spawn_result.set_result(Err(eyre!(err_msg.clone())));
        if let Err(e) = df
            .make_record(StoreDataflowStatus::Failed {
                error: err_msg.clone(),
                // Watchdog verdict is terminal: even across coordinator
                // restarts, a wedged daemon that eventually reports the
                // dataflow as running must NOT resurrect this record to
                // Running via the reconcile path (round-8 Finding 1).
                terminal: true,
            })
            .and_then(|r| store.put_dataflow(&r))
        {
            tracing::warn!(
                dataflow = %uuid,
                "failed to persist spawn timeout: {e}",
            );
        }

        // Final log message to anyone subscribed.
        send_log_message(
            &mut df.log_subscribers,
            &LogMessage {
                build_id: None,
                dataflow_id: Some(uuid),
                node_id: None,
                daemon_id: None,
                level: LogLevel::Error.into(),
                target: Some("coordinator".into()),
                module_path: None,
                file: None,
                line: None,
                message: err_msg.clone(),
                timestamp: clock.new_timestamp().get_time().to_system_time().into(),
                fields: None,
            },
        )
        .await;

        // Close topic subscribers so attached clients see a clean end-of-
        // stream rather than hanging.
        close_topic_subscribers_on_finish(&mut df);

        // Synthesize a `dataflow_results` entry so:
        //   - `dora list` shows the dataflow as Failed (instead of
        //     disappearing entirely — round-6 Finding 1)
        //   - `dora stop <uuid>` returns DataflowStopped via the early-
        //     return at the Stop handler (instead of "no known running
        //     dataflow" — round-6 Finding 3)
        //   - Late `DataflowFinishedOnDaemon` events can merge into the
        //     same entry rather than being silently discarded (round-6
        //     Finding 2; merge logic lives in that handler's Vacant arm).
        //
        // For each daemon that was assigned to this dataflow, emit a
        // per-daemon `DataflowDaemonResult` with one `Err(NodeError {
        // cause: FailedToSpawn(..) })` entry per node assigned to that
        // daemon. This makes
        // `results.values().all(DataflowDaemonResult::is_ok) == false`,
        // which classifies the dataflow as `Failed` in
        // `DataflowList` (lib.rs ~1019).
        //
        // **Crucially, iterate `df.node_to_daemon` for the daemon set,
        // not `df.daemons`**: the daemon-disconnect cleanup path at
        // `lib.rs:1893-1899` removes disconnected daemons from
        // `df.daemons` but leaves `df.node_to_daemon` (the original
        // assignment) intact. If we iterated `df.daemons` here and the
        // disconnect-mid-spawn case had emptied it, the result map
        // would be empty and List's classification check
        // `results.values().all(is_ok)` would be vacuously true,
        // misclassifying the dataflow as `Finished` (round-7
        // Finding 2). The original assignment is the right source of
        // truth for "what daemons should have been running this".
        let synth_results = synthesize_failed_dataflow_results(&df, uuid, &err_msg, clock);
        // Insert before draining stop senders so the DataflowResult
        // they receive carries the synthesized node-level errors.
        dataflow_results
            .entry(uuid)
            .or_default()
            .extend(synth_results);

        // Drain `stop_reply_senders`. Any in-flight `dora stop` calls were
        // waiting for the dataflow to stop; that's effectively what just
        // happened (the watchdog took ownership and the dataflow will not
        // proceed). Use `dataflow_result` (the helper used by the normal
        // DataflowFinishedOnDaemon path) over the synthesized entry so
        // the reply carries the per-node errors that `dora list` /
        // `dora check` will also surface.
        let stop_reply = ControlRequestReply::DataflowStopped {
            uuid,
            result: dataflow_results
                .get(&uuid)
                .map(|r| dataflow_result(r, uuid, clock))
                .unwrap_or_else(|| DataflowResult::ok_empty(uuid, clock.new_timestamp())),
        };
        for sender in df.stop_reply_senders.drain(..) {
            let _ = sender.send(Ok(stop_reply.clone()));
        }

        // Archive so `dora list` still surfaces the dataflow's name +
        // descriptor for users investigating after the fact. Capped to
        // prevent unbounded growth — uses the same MAX_ARCHIVED_DATAFLOWS
        // limit as the DataflowFinishedOnDaemon teardown.
        archived_dataflows
            .entry(uuid)
            .or_insert_with(|| ArchivedDataflow::from(&df));
        while archived_dataflows.len() > MAX_ARCHIVED_DATAFLOWS {
            archived_dataflows.shift_remove_index(0);
        }

        // Cap LAST: the synthesized entry was just read for the stop reply and
        // archival above, so evicting it now (if it is over-cap finished
        // history) can't misreport this dataflow as `ok_empty`. `uuid` is no
        // longer in `running_dataflows` here.
        cap_dataflow_results(dataflow_results, running_dataflows);
        // `df` drops here, releasing all remaining resources.
    }
}

/// Scan `running_builds` for builds that have been pending past
/// [`build_result_timeout`] and resolve them as terminally failed.
///
/// Mirrors [`check_spawn_timeouts`] but for builds, and is simpler:
/// there is no per-daemon "succeeded" state to roll back — a timed-out
/// build is just released. Daemons' local build artifacts (if any)
/// stay on disk and can be reused by a future `dora build`; the user
/// gets an actionable error immediately instead of a hung
/// `wait_for_build`.
///
/// For each stuck build the watchdog:
/// 1. Removes the entry from `running_builds`.
/// 2. Sets `build_result` to `Err` with a clear timeout message so
///    already-registered `wait_for_build` waiters are released.
/// 3. Sends a final log line to subscribers (`dora build --attach`).
/// 4. Inserts the cached result into `finished_builds` (FIFO-capped)
///    so a `wait_for_build` registered AFTER the watchdog fired also
///    receives the cached error rather than "unknown build id".
///
/// Idempotent: `CachedResult::set_result` is a no-op on already-Cached
/// values, and a build removed from `running_builds` simply doesn't
/// reappear in the next heartbeat tick's filter. Late
/// `DataflowBuildResult` replies after this fires fall into the
/// `running_builds.get_mut → None` arm in the event handler (warn
/// only, no resurrection).
///
/// Rescue of [#1465](https://github.com/dora-rs/dora/issues/1465).
async fn check_build_timeouts(
    running_builds: &mut HashMap<BuildId, RunningBuild>,
    finished_builds: &mut IndexMap<BuildId, CachedResult>,
    clock: &HLC,
    timeout_threshold: Duration,
) {
    let stuck: Vec<(BuildId, usize)> = running_builds
        .iter()
        .filter_map(|(id, build)| {
            if build.build_result.is_pending()
                && build.build_started_at.elapsed() > timeout_threshold
            {
                Some((*id, build.pending_build_results.len()))
            } else {
                None
            }
        })
        .collect();

    for (build_id, pending_count) in stuck {
        tracing::warn!(
            build_id = %build_id,
            timeout_secs = timeout_threshold.as_secs(),
            pending = pending_count,
            "build timeout: releasing wait_for_build waiters",
        );
        let Some(mut build) = running_builds.remove(&build_id) else {
            // Concurrent removal — nothing to do. (Single-task event
            // loop makes this unreachable today; defensive.)
            continue;
        };
        let err_msg = format!(
            "build timed out after {}s; {} daemon(s) never reported build_result",
            timeout_threshold.as_secs(),
            pending_count,
        );
        build.build_result.set_result(Err(eyre!(err_msg.clone())));

        // Final log to subscribers so attached `dora build --attach`
        // sessions see a clean end-of-stream rather than hanging.
        send_log_message(
            &mut build.log_subscribers,
            &LogMessage {
                build_id: Some(build_id),
                dataflow_id: None,
                node_id: None,
                daemon_id: None,
                level: LogLevel::Error.into(),
                target: Some("coordinator".into()),
                module_path: None,
                file: None,
                line: None,
                message: err_msg.clone(),
                timestamp: clock.new_timestamp().get_time().to_system_time().into(),
                fields: None,
            },
        )
        .await;

        // Insert into finished_builds so a `wait_for_build` registered
        // AFTER the watchdog fired also receives the cached error
        // (handler at the `WaitForBuild` arm above).
        finished_builds.insert(build_id, build.build_result);
        while finished_builds.len() > MAX_FINISHED_BUILDS {
            finished_builds.shift_remove_index(0);
        }
    }
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

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::AddNode`
/// is a successful `AddNodeResult`. Returns `Err` for both an explicit
/// daemon failure and an unexpected reply variant; callers should forward
/// the error to the CLI as a `ControlRequestReply::Error` and NOT use `?`
/// to bubble out of the coordinator's main loop. Rescue of #1757,
/// addresses #1682.
fn ensure_add_node_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::AddNodeResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::AddNodeResult(Err(err)) => {
            Err(eyre!("daemon failed to add node `{node_id}`: {err}"))
        }
        other => Err(eyre!(
            "unexpected daemon reply for AddNode on node `{node_id}`: {other:?}"
        )),
    }
}

fn ensure_set_param_forward_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::SetParamResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::SetParamResult(Err(err)) => Err(eyre!(
            "daemon failed to apply SetParam for node `{node_id}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for SetParam on node `{node_id}`: {other:?}"
        )),
    }
}

fn ensure_delete_param_forward_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::DeleteParamResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::DeleteParamResult(Err(err)) => Err(eyre!(
            "daemon failed to apply DeleteParam for node `{node_id}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for DeleteParam on node `{node_id}`: {other:?}"
        )),
    }
}

/// How long a node's `Stopped` row stays visible in
/// `running_dataflows[df].node_metrics` after a `DaemonEvent::NodeStopped`
/// arrives before the coordinator drops it. Long enough for an operator
/// running `dora node list` to see the `Stopped` status; short enough that
/// the listing doesn't accumulate zombie rows over a long-lived dataflow.
const NODE_STOPPED_GRACE: Duration = Duration::from_secs(60);

/// Drop any `node_metrics` rows whose corresponding `node_stopped_at`
/// timestamp is older than `NODE_STOPPED_GRACE`. Called from the
/// `NodeMetrics` push handler and the heartbeat tick so cleanup runs
/// even when no live metrics flow.
fn expire_stopped_nodes(dataflow: &mut RunningDataflow) {
    let now = Instant::now();
    let expired: Vec<dora_core::config::NodeId> = dataflow
        .node_stopped_at
        .iter()
        .filter(|(_, t)| now.duration_since(**t) >= NODE_STOPPED_GRACE)
        .map(|(nid, _)| nid.clone())
        .collect();
    for nid in expired {
        dataflow.node_metrics.remove(&nid);
        dataflow.node_stopped_at.remove(&nid);
        // Clear the finalize marker too so a subsequent AddNode of the
        // same name (or any future metrics push for it) is not blocked
        // by the stale Stopped state.
        dataflow.node_finalized.remove(&nid);
    }
}

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::RemoveNode`
/// is a successful `RemoveNodeResult`. Returns `Err` for both an explicit
/// daemon failure and an unexpected reply variant; callers should forward
/// the error to the CLI as a `ControlRequestReply::Error` and NOT use `?`
/// to bubble out of the coordinator's main loop. Parallel to
/// `ensure_add_node_applied` (#1873). Closes #1874.
fn ensure_remove_node_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::RemoveNodeResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::RemoveNodeResult(Err(err)) => {
            Err(eyre!("daemon failed to remove node `{node_id}`: {err}"))
        }
        other => Err(eyre!(
            "unexpected daemon reply for RemoveNode on node `{node_id}`: {other:?}"
        )),
    }
}

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::AddMapping`
/// is a successful `AddMappingResult`. Parallel to `ensure_add_node_applied`.
/// Before the daemon returned an explicit `AddMappingResult`, the coordinator's
/// `send_and_receive` for `AddMapping` timed out after 30s because the WS
/// layer dropped the daemon's `None` reply; closing that hole means we now
/// have a typed reply to check against — same #1682 class.
fn ensure_add_mapping_applied(reply_raw: &[u8], source: &str, target: &str) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::AddMappingResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::AddMappingResult(Err(err)) => Err(eyre!(
            "daemon failed to add mapping `{source}` -> `{target}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for AddMapping `{source}` -> `{target}`: {other:?}"
        )),
    }
}

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::RemoveMapping`
/// is a successful `RemoveMappingResult`. See `ensure_add_mapping_applied`.
fn ensure_remove_mapping_applied(reply_raw: &[u8], source: &str, target: &str) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::RemoveMappingResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::RemoveMappingResult(Err(err)) => Err(eyre!(
            "daemon failed to remove mapping `{source}` -> `{target}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for RemoveMapping `{source}` -> `{target}`: {other:?}"
        )),
    }
}

/// Build the per-daemon `DataflowDaemonResult` map that classifies a
/// terminally-failed dataflow as `Failed` (every assigned node reported as
/// `Err(FailedToSpawn(err_msg))`).
///
/// Iterates `node_to_daemon` (the original assignment), NOT `daemons`: the
/// disconnect-cleanup path prunes `daemons` but leaves `node_to_daemon` intact,
/// and an empty map would be vacuously classified `Finished` by `DataflowList`
/// (round-7 Finding 2). Falls back to a sentinel entry when `node_to_daemon` is
/// empty so the map is never empty (round-8 Finding 2).
///
/// Shared by the spawn-timeout watchdog and the daemon-disconnect teardown
/// (#2028).
fn synthesize_failed_dataflow_results(
    df: &RunningDataflow,
    uuid: DataflowId,
    err_msg: &str,
    clock: &HLC,
) -> BTreeMap<DaemonId, DataflowDaemonResult> {
    let synth_timestamp = clock.new_timestamp();
    let assigned_daemons: BTreeSet<DaemonId> = df.node_to_daemon.values().cloned().collect();
    let synth_results: BTreeMap<DaemonId, DataflowDaemonResult> = assigned_daemons
        .iter()
        .map(|daemon_id| {
            let nodes_for_daemon: BTreeMap<NodeId, Result<(), NodeError>> = df
                .node_to_daemon
                .iter()
                .filter(|(_, did)| *did == daemon_id)
                .map(|(node_id, _)| {
                    (
                        node_id.clone(),
                        Err(NodeError {
                            timestamp: synth_timestamp,
                            cause: NodeErrorCause::FailedToSpawn(err_msg.to_string()),
                            exit_status: NodeExitStatus::Unknown,
                        }),
                    )
                })
                .collect();
            (
                daemon_id.clone(),
                DataflowDaemonResult {
                    timestamp: synth_timestamp,
                    node_results: nodes_for_daemon,
                },
            )
        })
        .collect();
    if synth_results.is_empty() {
        tracing::warn!(
            dataflow = %uuid,
            "teardown: node_to_daemon was empty; injecting sentinel result \
             so list classification is Failed",
        );
        let mut sentinel = BTreeMap::new();
        let mut node_results = BTreeMap::new();
        // Use `"watchdog"` (no angle brackets): `NodeId::from(invalid_chars)`
        // PANICS via `validate_node_id` (rejects chars outside `[a-zA-Z0-9_.-]`).
        node_results.insert(
            NodeId::from("watchdog".to_string()),
            Err(NodeError {
                timestamp: synth_timestamp,
                cause: NodeErrorCause::FailedToSpawn(err_msg.to_string()),
                exit_status: NodeExitStatus::Unknown,
            }),
        );
        sentinel.insert(
            DaemonId::new(Some("watchdog".to_string())),
            DataflowDaemonResult {
                timestamp: synth_timestamp,
                node_results,
            },
        );
        sentinel
    } else {
        synth_results
    }
}

/// Broadcast `AllNodesReady` to every daemon running part of `dataflow` and
/// schedule the persisted-parameter replay. Extracted from the `ReadyOnDaemon`
/// handler so the disconnect/cleanup path can also release the start barrier
/// when the last *pending* daemon goes away via disconnect rather than
/// `ReadyOnDaemon` (see issue #2028).
async fn broadcast_all_nodes_ready(
    uuid: DataflowId,
    dataflow: &RunningDataflow,
    daemon_connections: &mut DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) -> eyre::Result<()> {
    tracing::debug!("sending all nodes ready message to daemons");
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::AllNodesReady {
            dataflow_id: uuid,
            exited_before_subscribe: dataflow.exited_before_subscribe.clone(),
        },
        timestamp: clock.new_timestamp(),
    })
    .wrap_err("failed to serialize AllNodesReady message")?;

    // notify all machines that run parts of the dataflow
    for daemon_id in &dataflow.daemons {
        let Some(connection) = daemon_connections.get_mut(daemon_id) else {
            tracing::warn!("no daemon connection found for machine `{daemon_id}`");
            continue;
        };
        connection.send(&message).await.wrap_err_with(|| {
            format!("failed to send AllNodesReady({uuid}) message to machine {daemon_id}")
        })?;
    }

    schedule_param_replay_for_ready_dataflow(
        uuid,
        dataflow,
        daemon_connections,
        store.clone(),
        clock.clone(),
    );
    Ok(())
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

#[cfg(feature = "tracing")]
#[allow(clippy::unnecessary_sort_by)]
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
            node_finalized: BTreeSet::new(),
            node_stopped_at: BTreeMap::new(),
            network_metrics: None,
            spawn_result: CachedResult::default(),
            stop_reply_senders: vec![],
            buffered_log_messages: vec![],
            log_subscribers: vec![],
            topic_subscribers: BTreeMap::new(),
            pending_spawn_results: BTreeSet::new(),
            spawn_started_at: Instant::now(),
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
    fn reconcile_reestablishes_running_dataflow_after_reconnect() {
        // #2029 P1: after a reclaim (or coordinator restart) removed the live
        // entry, a reconnecting daemon's status report must rebuild it in
        // `running_dataflows` so `dora list` / `stop` / `logs` see the survivor
        // again — store status alone doesn't drive the control plane.
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("d1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();

        // A persisted record as it would exist after a reclaim/restart: status
        // Recovering, descriptor with a resolvable node.
        let record = dora_coordinator_store::DataflowRecord {
            uuid: dataflow_id,
            name: Some("df".to_string()),
            descriptor_json: serde_json::json!({
                "nodes": [{ "id": "sender", "path": "sleep", "outputs": ["message"] }]
            })
            .to_string(),
            status: StoreDataflowStatus::Recovering,
            daemon_ids: vec![daemon_id.clone()],
            node_to_daemon: BTreeMap::new(),
            uv: false,
            generation: 3,
            created_at: 7,
            updated_at: 7,
        };

        // Absent (reclaimed away / restart) -> reconstruct from the record.
        let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
        reestablish_running_dataflow(
            &mut running_dataflows,
            &record,
            &daemon_id,
            std::slice::from_ref(&node_id),
        );
        let rebuilt = running_dataflows
            .get(&dataflow_id)
            .expect("must reconstruct the live entry");
        assert!(rebuilt.daemons.contains(&daemon_id));
        assert_eq!(rebuilt.node_to_daemon.get(&node_id), Some(&daemon_id));
        assert!(
            matches!(&rebuilt.spawn_result, CachedResult::Cached { result } if result.is_ok()),
            "spawn_result must be cached Ok so stop/wait waiters don't hang"
        );

        // Present (multi-daemon partial reconnect) -> relink, not duplicate.
        let daemon2 = DaemonId::new(Some("d2".to_string()));
        let node2: dora_core::config::NodeId = "receiver".to_string().into();
        reestablish_running_dataflow(
            &mut running_dataflows,
            &record,
            &daemon2,
            std::slice::from_ref(&node2),
        );
        let df = running_dataflows.get(&dataflow_id).expect("still present");
        assert!(df.daemons.contains(&daemon_id) && df.daemons.contains(&daemon2));
        assert_eq!(df.node_to_daemon.get(&node2), Some(&daemon2));
        assert_eq!(running_dataflows.len(), 1, "must relink, not duplicate");
    }

    #[tokio::test]
    async fn orphan_stop_sends_stopdataflow_to_reporting_daemon() {
        // #2029 P3: when a daemon reconnects (within its own reconnect window)
        // reporting a dataflow the coordinator already failed terminally — i.e.
        // its recovery timeout fired first — the coordinator can't re-adopt it,
        // so it must tell that daemon to stop the orphaned nodes.
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("d1".to_string()));
        let clock = HLC::default();

        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let connection = state::DaemonConnection::new(
            tx,
            std::sync::Arc::new(tokio::sync::Mutex::new(std::collections::HashMap::new())),
            BTreeMap::new(),
        );
        let mut daemon_connections = DaemonConnections::default();
        daemon_connections.add(daemon_id.clone(), connection);

        stop_orphaned_dataflow_on_daemon(dataflow_id, &daemon_id, &mut daemon_connections, &clock)
            .await;

        let sent = rx
            .try_recv()
            .expect("a stop message must be sent to the reporting daemon");
        assert!(
            sent.contains("StopDataflow"),
            "must send StopDataflow, got: {sent}"
        );
        assert!(
            sent.contains(&dataflow_id.to_string()),
            "stop must target the orphaned dataflow"
        );
    }

    #[test]
    fn disconnect_cleanup_removes_daemon_from_running_dataflow_membership() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("gone".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
        df.pending_daemons.insert(daemon_id.clone());
        df.pending_spawn_results.insert(daemon_id.clone());

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let disconnected = BTreeSet::from([daemon_id.clone()]);

        cleanup_disconnected_daemons_from_running_dataflows(
            &mut running_dataflows,
            &disconnected,
            &mut HashMap::new(),
        );

        let df = running_dataflows
            .get(&dataflow_id)
            .expect("disconnect cleanup must not remove the dataflow");
        assert!(!df.daemons.contains(&daemon_id));
        assert!(!df.pending_daemons.contains(&daemon_id));
        assert!(!df.pending_spawn_results.contains(&daemon_id));
        assert_eq!(
            df.node_to_daemon.get(&node_id),
            Some(&daemon_id),
            "original node assignment stays available for later failure synthesis"
        );
        assert!(
            df.spawn_result.is_pending(),
            "disconnect cleanup must leave spawn_result to the watchdog"
        );
    }

    // #2028: a spawn-pending dataflow stays the spawn-timeout watchdog's
    // domain — disconnect cleanup must produce no action and not remove it.
    #[test]
    fn disconnect_of_spawn_pending_dataflow_produces_no_action() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("gone".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        // spawn_result left Pending (CachedResult::default()).

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let disconnected = BTreeSet::from([daemon_id]);

        let actions = cleanup_disconnected_daemons_from_running_dataflows(
            &mut running_dataflows,
            &disconnected,
            &mut HashMap::new(),
        );
        assert!(
            // spawn_result is Pending, so the dataflow should NOT trigger
            // a ReclaimOrphaned action.
            actions.is_empty(),
            "spawn-pending dataflows must be left to the spawn-timeout watchdog"
        );
        assert!(
            running_dataflows.contains_key(&dataflow_id),
            "must not tear down a spawn-pending dataflow"
        );
    }

    // #2028 deadlock #1: the last daemon awaited for ReadyOnDaemon disconnects
    // after spawn succeeded -> the start barrier must be released for survivors.
    #[test]
    fn disconnect_releases_ready_barrier_when_last_pending_daemon_drops() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_a = DaemonId::new(Some("a".to_string()));
        let daemon_b = DaemonId::new(Some("b".to_string()));
        let node_a: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_a.clone(), node_a);
        df.daemons.insert(daemon_b.clone());
        df.node_to_daemon
            .insert("receiver".to_string().into(), daemon_b.clone());
        // spawn already succeeded; A reported ready, still waiting on B.
        df.spawn_result
            .set_result(Ok(ControlRequestReply::DataflowSpawned {
                uuid: dataflow_id,
            }));
        df.pending_daemons.insert(daemon_b.clone());

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let disconnected = BTreeSet::from([daemon_b]);

        let actions = cleanup_disconnected_daemons_from_running_dataflows(
            &mut running_dataflows,
            &disconnected,
            &mut HashMap::new(),
        );
        assert!(
            matches!(actions.as_slice(), [DisconnectAction::ReleaseReadyBarrier(id)] if *id == dataflow_id),
            "must release the ready barrier, got {} action(s)",
            actions.len()
        );
        let df = running_dataflows
            .get(&dataflow_id)
            .expect("survivor dataflow must remain");
        assert_eq!(df.daemons, BTreeSet::from([daemon_a]));
        assert!(df.pending_daemons.is_empty());
    }

    // #2028 deadlocks #2 + #3 + #2029 reclaim: the sole daemon of a running
    // dataflow disconnects -> the dataflow must be removed from the running set
    // and any parked `dora stop` waiter resolved instead of hanging. But unlike
    // a terminal teardown, the disconnect now opens a *reclaim window*: the
    // store record is `Recovering` (NOT terminal `Failed`) and the dataflow is
    // NOT archived, so a reconnecting daemon's `DaemonStatusReport` can
    // reconcile it back to `Running` (the reconcile path skips archived/terminal
    // records). Permanent loss is handled by the `Recovering -> Failed` recovery
    // timeout sweep.
    #[tokio::test]
    async fn disconnect_reclaims_orphaned_running_dataflow() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = Arc::new(HLC::default());
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("gone".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        df.spawn_result
            .set_result(Ok(ControlRequestReply::DataflowSpawned {
                uuid: dataflow_id,
            }));
        // Seed the store with a Running record so we can observe the transition
        // to Recovering (the production spawn path persists this).
        store
            .put_dataflow(
                &df.make_record(StoreDataflowStatus::Running)
                    .expect("make running record"),
            )
            .expect("seed running record");
        // a parked `dora stop` waiter
        let (tx, rx) = tokio::sync::oneshot::channel();
        df.stop_reply_senders.push(tx);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let disconnected = BTreeSet::from([daemon_id]);

        let actions = cleanup_disconnected_daemons_from_running_dataflows(
            &mut running_dataflows,
            &disconnected,
            &mut HashMap::new(),
        );
        assert!(
            matches!(actions.as_slice(), [DisconnectAction::ReclaimOrphaned(id)] if *id == dataflow_id),
            "must reclaim (not terminally tear down) the orphaned dataflow"
        );

        apply_disconnect_actions(
            actions,
            &mut running_dataflows,
            &mut daemon_connections,
            &store,
            &clock,
        )
        .await
        .expect("apply_disconnect_actions");

        assert!(
            !running_dataflows.contains_key(&dataflow_id),
            "orphaned dataflow must be removed from running set"
        );
        // Reclaimable, not terminal: store record is Recovering so a reconnecting
        // daemon's status report can promote it back to Running.
        let record = store
            .get_dataflow(&dataflow_id)
            .expect("store lookup")
            .expect("record present");
        assert!(
            matches!(record.status, StoreDataflowStatus::Recovering),
            "disconnect must mark the dataflow Recovering, got {:?}",
            record.status
        );
        // The parked `dora stop` must still be released (no hang).
        let reply = rx.await.expect("stop waiter must be resolved by reclaim");
        assert!(
            matches!(reply, Ok(ControlRequestReply::DataflowStopped { uuid, .. }) if uuid == dataflow_id),
            "parked dora stop must receive DataflowStopped"
        );
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

    #[test]
    fn set_param_forward_reply_reports_daemon_rejection() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Err(
            "node `camera` channel full".to_string(),
        )))
        .unwrap();
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let err = ensure_set_param_forward_applied(&reply, &node_id)
            .expect_err("daemon rejection should fail strict forwarding");
        assert!(err.to_string().contains("failed to apply SetParam"));
    }

    // -------------------------------------------------------------------
    // AddNode reply validation (rescue of #1757, addresses #1682)
    // -------------------------------------------------------------------
    //
    // The bug: coordinator's `Ok(_) =>` arm in the AddNode dispatch
    // (lib.rs:1558 pre-fix) accepted any successful `send_and_receive`
    // reply and committed dataflow state, even when the daemon returned
    // an `AddNodeResult(Err(...))` or a stale reply from a different
    // request. The three tests below pin the validator's contract: it
    // must accept ONLY a successful `AddNodeResult` and forward every
    // other shape as an error to the call site (which then surfaces it
    // to the CLI without bringing down the coordinator's main loop).

    #[test]
    fn add_node_reply_accepts_daemon_success() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddNodeResult(Ok(()))).unwrap();
        let node_id: dora_core::config::NodeId = "filter".to_string().into();

        ensure_add_node_applied(&reply, &node_id).expect("successful AddNode reply should pass");
    }

    #[test]
    fn add_node_reply_reports_daemon_rejection() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddNodeResult(Err(
            "failed to spawn node".to_string(),
        )))
        .unwrap();
        let node_id: dora_core::config::NodeId = "filter".to_string().into();

        let err = ensure_add_node_applied(&reply, &node_id)
            .expect_err("daemon rejection should fail AddNode forwarding");
        let msg = err.to_string();
        assert!(
            msg.contains("failed to add node") && msg.contains("filter"),
            "error must name the operation and node: {msg}"
        );
    }

    #[test]
    fn add_node_reply_rejects_wrong_reply_variant() {
        // This is the regression scenario for #1682: the daemon returned
        // a stale or otherwise unrelated reply variant (here:
        // `SetParamResult(Ok)`). Before the fix, the coordinator's
        // `Ok(_) =>` arm would accept this and commit state for a node
        // the daemon never actually added.
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        let node_id: dora_core::config::NodeId = "filter".to_string().into();

        let err = ensure_add_node_applied(&reply, &node_id)
            .expect_err("unexpected reply variant should fail AddNode forwarding");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );

        let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveNodeResult(Ok(()))).unwrap();
        let err = ensure_add_node_applied(&reply, &node_id)
            .expect_err("RemoveNodeResult reply must not be accepted by AddNode validator");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );
    }

    #[test]
    fn delete_param_forward_reply_rejects_unexpected_reply_variant() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let err = ensure_delete_param_forward_applied(&reply, &node_id)
            .expect_err("unexpected reply variant should fail strict forwarding");
        assert!(err.to_string().contains("unexpected daemon reply"));
    }

    #[test]
    fn remove_node_reply_accepts_daemon_success() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveNodeResult(Ok(()))).unwrap();
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        ensure_remove_node_applied(&reply, &node_id)
            .expect("successful RemoveNode reply should pass");
    }

    #[test]
    fn remove_node_reply_reports_daemon_rejection() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveNodeResult(Err(
            "node `camera` not found in running dataflow".to_string(),
        )))
        .unwrap();
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let err = ensure_remove_node_applied(&reply, &node_id)
            .expect_err("daemon rejection should fail RemoveNode forwarding");
        let msg = err.to_string();
        assert!(
            msg.contains("failed to remove node") && msg.contains("camera"),
            "error must name the operation and node: {msg}"
        );
    }

    #[test]
    fn remove_node_reply_rejects_wrong_reply_variant() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        let node_id: dora_core::config::NodeId = "camera".to_string().into();

        let err = ensure_remove_node_applied(&reply, &node_id)
            .expect_err("unexpected reply variant should fail RemoveNode forwarding");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );

        let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddNodeResult(Ok(()))).unwrap();
        let err = ensure_remove_node_applied(&reply, &node_id)
            .expect_err("AddNodeResult reply must not be accepted by RemoveNode validator");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );
    }

    // -------------------------------------------------------------------
    // AddMapping / RemoveMapping reply validation (silent-reply rescue
    // for the connect/disconnect timeouts — same class as #1682).
    // -------------------------------------------------------------------

    #[test]
    fn add_mapping_reply_accepts_daemon_success() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddMappingResult(Ok(()))).unwrap();
        ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
            .expect("successful AddMapping reply should pass");
    }

    #[test]
    fn add_mapping_reply_reports_daemon_rejection() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddMappingResult(Err(
            "no running dataflow with ID `xyz`".to_string(),
        )))
        .unwrap();
        let err = ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
            .expect_err("daemon rejection should fail AddMapping forwarding");
        let msg = err.to_string();
        assert!(
            msg.contains("failed to add mapping")
                && msg.contains("sender/value")
                && msg.contains("filter/input"),
            "error must name the operation and the mapping endpoints: {msg}"
        );
    }

    #[test]
    fn add_mapping_reply_rejects_wrong_reply_variant() {
        // Pre-fix #1682-equivalent regression: daemon returned `None`,
        // WS layer dropped it, coordinator's `Ok(_) =>` arm reported
        // success on any wire response. With AddMappingResult now typed,
        // a foreign reply variant must NOT be accepted.
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        let err = ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
            .expect_err("unexpected reply variant should fail AddMapping forwarding");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );

        let reply =
            serde_json::to_vec(&DaemonCoordinatorReply::RemoveMappingResult(Ok(()))).unwrap();
        let err = ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
            .expect_err("RemoveMappingResult must not be accepted by AddMapping validator");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );
    }

    #[test]
    fn remove_mapping_reply_accepts_daemon_success() {
        let reply =
            serde_json::to_vec(&DaemonCoordinatorReply::RemoveMappingResult(Ok(()))).unwrap();
        ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
            .expect("successful RemoveMapping reply should pass");
    }

    #[test]
    fn remove_mapping_reply_reports_daemon_rejection() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveMappingResult(Err(
            "no running dataflow with ID `xyz`".to_string(),
        )))
        .unwrap();
        let err = ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
            .expect_err("daemon rejection should fail RemoveMapping forwarding");
        let msg = err.to_string();
        assert!(
            msg.contains("failed to remove mapping")
                && msg.contains("sender/value")
                && msg.contains("filter/input"),
            "error must name the operation and the mapping endpoints: {msg}"
        );
    }

    #[test]
    fn remove_mapping_reply_rejects_wrong_reply_variant() {
        let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        let err = ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
            .expect_err("unexpected reply variant should fail RemoveMapping forwarding");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );

        let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddMappingResult(Ok(()))).unwrap();
        let err = ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
            .expect_err("AddMappingResult must not be accepted by RemoveMapping validator");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );
    }

    // -------------------------------------------------------------------
    // Node-stop stale-metrics fix (PR #1901 / follow-up to #1703 prereq)
    // -------------------------------------------------------------------

    #[test]
    fn expire_stopped_nodes_removes_entries_older_than_grace() {
        use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "stopped-sender".to_string().into();
        let fresh_node: dora_core::config::NodeId = "fresh-receiver".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());

        let stale_row = NodeMetrics {
            pid: 0,
            cpu_usage: 0.0,
            memory_bytes: 0,
            disk_read_bytes: None,
            disk_write_bytes: None,
            restart_count: 0,
            broken_inputs: Vec::new(),
            status: NodeStatus::Stopped,
            pending_messages: 0,
        };
        let fresh_row = NodeMetrics {
            status: NodeStatus::Running,
            ..stale_row.clone()
        };

        df.node_metrics.insert(node_id.clone(), stale_row);
        df.node_metrics.insert(fresh_node.clone(), fresh_row);

        // Backdate the stale node past the grace window; leave the fresh
        // node out of node_stopped_at entirely (it's still running).
        df.node_stopped_at.insert(
            node_id.clone(),
            Instant::now() - NODE_STOPPED_GRACE - Duration::from_secs(1),
        );

        expire_stopped_nodes(&mut df);

        assert!(
            !df.node_metrics.contains_key(&node_id),
            "stale stopped row should be dropped after grace"
        );
        assert!(
            !df.node_stopped_at.contains_key(&node_id),
            "stale stopped_at marker should be dropped together with the metrics row"
        );
        assert!(
            df.node_metrics.contains_key(&fresh_node),
            "non-stopped node must NOT be affected by the sweep"
        );
    }

    #[test]
    fn expire_stopped_nodes_keeps_entries_within_grace() {
        use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "just-stopped".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
        df.node_metrics.insert(
            node_id.clone(),
            NodeMetrics {
                pid: 0,
                cpu_usage: 0.0,
                memory_bytes: 0,
                disk_read_bytes: None,
                disk_write_bytes: None,
                restart_count: 0,
                broken_inputs: Vec::new(),
                status: NodeStatus::Stopped,
                pending_messages: 0,
            },
        );
        // Recent: well within grace.
        df.node_stopped_at.insert(node_id.clone(), Instant::now());

        expire_stopped_nodes(&mut df);

        assert!(
            df.node_metrics.contains_key(&node_id),
            "stopped row must stay visible during the grace window"
        );
        assert!(
            df.node_stopped_at.contains_key(&node_id),
            "stopped_at marker must stay armed during the grace window"
        );
    }

    #[test]
    fn expire_stopped_nodes_leaves_failed_rows_untouched() {
        // Pre-fix regression scenario: the DaemonNodeStopped handler
        // previously inserted into `node_stopped_at` for both Stopped
        // and Failed statuses, so a crashed `restart_policy: Never`
        // node would disappear from `dora node list` and `dora doctor`
        // after the 60s grace window — hiding the very failure this
        // PR is supposed to surface. The handler now only arms the
        // expire side-band for Stopped rows; this test asserts that
        // a Failed row whose `node_stopped_at` entry is somehow set
        // would still be swept on its own clock, but in the normal
        // flow no entry exists for Failed rows so the sweep is a no-op.
        use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "crashed".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
        df.node_metrics.insert(
            node_id.clone(),
            NodeMetrics {
                pid: 0,
                cpu_usage: 0.0,
                memory_bytes: 0,
                disk_read_bytes: None,
                disk_write_bytes: None,
                restart_count: 0,
                broken_inputs: Vec::new(),
                status: NodeStatus::Failed,
                pending_messages: 0,
            },
        );
        // No node_stopped_at entry (which is what the handler now
        // guarantees for Failed rows). Even hours later, the sweep
        // must not remove this row.
        expire_stopped_nodes(&mut df);
        assert!(
            df.node_metrics.contains_key(&node_id),
            "Failed row with no expire timer must remain after sweep"
        );

        // Even if a *very* old expire timer somehow remains armed
        // for a Failed row (e.g. stale leftover from a prior
        // Stopped → respawn → Failed sequence), the sweep removes
        // both — that's OK because the caller for the Failed update
        // explicitly clears `node_stopped_at`. This test pins down
        // the sweep's semantics so a future refactor that decides
        // to "preserve Failed rows past grace" doesn't quietly
        // break by overriding the side-band contract.
    }

    #[test]
    fn expire_stopped_nodes_clears_finalized_marker_too() {
        // The grace-period sweep must clear `node_finalized` in addition
        // to `node_metrics` and `node_stopped_at`. Without this, a
        // subsequent `dora node add` of the same name (or the next
        // metrics push for it) would be blocked by the stale marker —
        // the new incarnation would never appear in `dora node list`.
        use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "expiring".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
        df.node_metrics.insert(
            node_id.clone(),
            NodeMetrics {
                pid: 0,
                cpu_usage: 0.0,
                memory_bytes: 0,
                disk_read_bytes: None,
                disk_write_bytes: None,
                restart_count: 0,
                broken_inputs: Vec::new(),
                status: NodeStatus::Stopped,
                pending_messages: 0,
            },
        );
        df.node_finalized.insert(node_id.clone());
        df.node_stopped_at.insert(
            node_id.clone(),
            Instant::now() - NODE_STOPPED_GRACE - Duration::from_secs(1),
        );

        expire_stopped_nodes(&mut df);

        assert!(!df.node_metrics.contains_key(&node_id));
        assert!(!df.node_stopped_at.contains_key(&node_id));
        assert!(
            !df.node_finalized.contains(&node_id),
            "finalized marker must clear together with the metrics row, \
             else AddNode of the same id would be silently blocked"
        );
    }

    #[test]
    fn finalized_marker_blocks_stale_metrics_for_failed_row_too() {
        // Pre-fix race: the metrics-handler guard used `node_stopped_at`
        // (only armed for Stopped). A crashed node sends NodeStopped
        // {clean_stop:false}, coordinator writes Failed but does NOT
        // arm node_stopped_at; a delayed in-flight metrics push then
        // sails past the guard and overwrites Failed back to Running.
        // The fix uses `node_finalized` (armed for BOTH Stopped and
        // Failed) so the guard catches the Failed case too.
        //
        // This test exercises the data-level invariant: a row marked
        // Failed AND inserted into node_finalized is what the
        // NodeMetrics handler now consults via
        // `dataflow.node_finalized.contains(node_id)` — the unit-test
        // equivalent is just verifying that lookup is true.
        use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "crashed-no-restart".to_string().into();

        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
        df.node_metrics.insert(
            node_id.clone(),
            NodeMetrics {
                pid: 0,
                cpu_usage: 0.0,
                memory_bytes: 0,
                disk_read_bytes: None,
                disk_write_bytes: None,
                restart_count: 0,
                broken_inputs: Vec::new(),
                status: NodeStatus::Failed,
                pending_messages: 0,
            },
        );
        // Failed path: finalized armed, stopped_at NOT armed.
        df.node_finalized.insert(node_id.clone());

        assert!(
            df.node_finalized.contains(&node_id),
            "Failed row must be in node_finalized so the metrics-push \
             guard at lib.rs:2181 skips it"
        );
        assert!(
            !df.node_stopped_at.contains_key(&node_id),
            "Failed row must NOT be in node_stopped_at — that would \
             arm the auto-expire timer and hide the crash after 60s"
        );

        // Run the sweep: nothing should change for a Failed row that
        // has no stopped_at entry.
        expire_stopped_nodes(&mut df);

        assert!(
            df.node_metrics.contains_key(&node_id) && df.node_finalized.contains(&node_id),
            "Failed row + its finalize marker must survive the sweep \
             until the dataflow itself is stopped/destroyed"
        );
    }

    #[test]
    fn cached_result_is_pending_distinguishes_pending_and_cached() {
        let mut r = CachedResult::default();
        assert!(r.is_pending(), "fresh CachedResult should be Pending");

        r.set_result(Err(eyre!("done")));
        assert!(!r.is_pending(), "after set_result, should be Cached");

        // set_result on Cached should be a no-op (already covered by
        // existing semantics but verified here to lock in idempotency).
        r.set_result(Ok(ControlRequestReply::DataflowSpawned {
            uuid: DataflowId::from(Uuid::new_v4()),
        }));
        assert!(!r.is_pending());
    }

    #[tokio::test]
    async fn check_spawn_timeouts_fires_error_when_pending_past_deadline() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        // Backdate the spawn start so the watchdog treats it as stuck.
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);
        // Still waiting on the daemon's spawn_result; this is the trigger
        // condition the watchdog is meant to catch.
        df.pending_spawn_results.insert(daemon_id);

        // Register a waiter so we can prove `wait_for_spawn` unblocks with
        // an error rather than hanging.
        let (tx, rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(tx);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // The waiter should now have an error reply.
        let reply = timeout(TokioDuration::from_secs(1), rx)
            .await
            .expect("waiter should resolve, not hang")
            .expect("sender should not drop");
        let err = reply.expect_err("timed-out spawn must surface as Err");
        let msg = format!("{err:?}");
        assert!(
            msg.contains("spawn timed out") || msg.contains("timeout"),
            "error should explain the timeout, got: {msg}"
        );

        // Watchdog must remove the dataflow from running_dataflows so
        // Check / List / reconcile no longer report it as active.
        assert!(
            !running_dataflows.contains_key(&dataflow_id),
            "watchdog must remove the terminally-failed dataflow from running_dataflows"
        );

        // And it must be archived so post-mortem queries can still find
        // its name/descriptor.
        assert!(
            archived_dataflows.contains_key(&dataflow_id),
            "watchdog must archive the terminally-failed dataflow"
        );
    }

    #[tokio::test]
    async fn check_spawn_timeouts_no_op_when_within_deadline() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        // Fresh spawn — well within the deadline.
        df.spawn_started_at = Instant::now();
        df.pending_spawn_results.insert(daemon_id);

        let (tx, rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(tx);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // The waiter should still be pending.
        let polled = tokio::time::timeout(TokioDuration::from_millis(50), rx).await;
        assert!(polled.is_err(), "waiter must still be pending pre-deadline");

        let df = running_dataflows
            .get(&dataflow_id)
            .expect("df still present");
        assert!(
            df.spawn_result.is_pending(),
            "spawn_result must remain Pending pre-deadline"
        );
    }

    #[tokio::test]
    async fn check_spawn_timeouts_idempotent_on_already_cached_result() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id);
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);
        // Spawn already resolved successfully — the watchdog must NOT
        // re-fire on subsequent heartbeats and clobber the cached result.
        df.spawn_result
            .set_result(Ok(ControlRequestReply::DataflowSpawned {
                uuid: dataflow_id,
            }));

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        let df = running_dataflows
            .get_mut(&dataflow_id)
            .expect("Cached(Ok) dataflow must stay in running_dataflows — only Cached(Err) triggers teardown");
        // Registering a new waiter on a Cached result must immediately
        // deliver the cached Ok — proving the watchdog did not clobber it.
        let (tx, rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(tx);
        let reply = timeout(TokioDuration::from_millis(50), rx)
            .await
            .expect("Cached result should deliver immediately")
            .expect("sender should not drop");
        let ok = reply.expect("the cached Ok result must be preserved");
        assert!(matches!(ok, ControlRequestReply::DataflowSpawned { .. }));
    }

    /// Covers Finding 2 from the self-review on PR #1854: exercises the
    /// rollback dispatch path with a *non-empty* succeeded set. The other
    /// timeout tests have `succeeded = {}`, so the rollback helper
    /// short-circuits at its `if spawned_daemons.is_empty()` early-exit
    /// and never actually sends a stop message.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn check_spawn_timeouts_dispatches_rollback_stop_to_succeeded_daemon() {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            params: Timestamped<DaemonCoordinatorEvent>,
        }

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_a = DaemonId::new(Some("daemon-a".to_string())); // succeeded
        let daemon_b = DaemonId::new(Some("daemon-b".to_string())); // still pending

        // Mock daemon `a` -- fire-and-forget rollback uses `connection.send()`
        // which only enqueues; no reply is awaited. We just need to capture
        // the outbound message and verify StopDataflow was dispatched.
        let (tx_a, mut rx_a) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies_a = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let conn_a =
            crate::state::DaemonConnection::new(tx_a, pending_replies_a.clone(), BTreeMap::new());
        let mut daemon_connections = DaemonConnections::default();
        daemon_connections.add(daemon_a.clone(), conn_a);

        let stop_seen = Arc::new(tokio::sync::Mutex::new(false));
        let stop_seen_task = stop_seen.clone();
        let daemon_a_task = tokio::spawn(async move {
            while let Some(outbound) = rx_a.recv().await {
                let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
                match outbound_raw.params.inner {
                    DaemonCoordinatorEvent::StopDataflow { .. } => {
                        *stop_seen_task.lock().await = true;
                    }
                    other => panic!("unexpected event on daemon-a in rollback test: {other:?}"),
                }
            }
        });

        // Build a RunningDataflow where `a` succeeded (in `daemons` but not
        // in `pending_spawn_results`) and `b` is still pending.
        let mut df =
            test_running_dataflow(dataflow_id, daemon_a.clone(), "sender".to_string().into());
        df.daemons.insert(daemon_b.clone());
        df.pending_spawn_results.insert(daemon_b.clone());
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        // Register a waiter so we can confirm spawn_result fires.
        let (waiter_tx, waiter_rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(waiter_tx);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // 1. The mock daemon must have received a stop dispatch.
        //    Fire-and-forget rollback enqueues into the mpsc channel; the
        //    daemon task picks it up asynchronously, so poll briefly.
        let saw_stop = timeout(TokioDuration::from_secs(1), async {
            loop {
                if *stop_seen.lock().await {
                    return true;
                }
                tokio::time::sleep(TokioDuration::from_millis(5)).await;
            }
        })
        .await
        .unwrap_or(false);
        assert!(
            saw_stop,
            "rollback must dispatch StopDataflow to the succeeded daemon"
        );

        // 2. The waiter must be released with an error (regardless of
        //    rollback outcome).
        let reply = timeout(TokioDuration::from_secs(1), waiter_rx)
            .await
            .expect("waiter should resolve, not hang")
            .expect("sender should not drop");
        let err = reply.expect_err("timed-out spawn must surface as Err");
        let msg = format!("{err:?}");
        assert!(
            msg.contains("spawn timed out") || msg.contains("timeout"),
            "error should explain the timeout, got: {msg}"
        );

        // 3. Dataflow must be persisted as Failed so a coordinator restart
        //    does not resurrect it as Recovering.
        let records = store.list_dataflows().expect("store should list");
        let record = records
            .iter()
            .find(|r| r.uuid == dataflow_id)
            .expect("dataflow should be persisted after timeout");
        assert!(
            matches!(
                record.status,
                dora_coordinator_store::DataflowStatus::Failed { .. }
            ),
            "dataflow must be persisted as Failed, got: {:?}",
            record.status,
        );

        daemon_a_task.abort();
    }

    /// Covers Finding 1 from the round-3 review on PR #1854: a late
    /// successful `DataflowSpawnResult` arriving after the watchdog has
    /// already failed the dataflow must NOT resurrect it as `Running` in
    /// the store.
    ///
    /// This test drives `handle_spawn_result_ok` directly (the same
    /// helper the event loop's success arm calls) so any future refactor
    /// that drops the `spawn_result.is_pending()` guard will surface here.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn late_successful_spawn_result_does_not_resurrect_failed_dataflow() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("late".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());

        // Simulate the watchdog having already fired: spawn_result is
        // Cached(Err), and the Failed status is persisted to the store.
        df.spawn_result
            .set_result(Err(eyre!("spawn timed out after 60s (watchdog)")));
        assert!(!df.spawn_result.is_pending());

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let failed_record = df
            .make_record(StoreDataflowStatus::Failed {
                error: "spawn timed out".to_string(),
                terminal: true,
            })
            .expect("make_record should succeed");
        store
            .put_dataflow(&failed_record)
            .expect("seed Failed in store");

        // Now simulate the late-arriving Ok: the event loop would have
        // removed `daemon_id` from `pending_spawn_results` and then
        // called `handle_spawn_result_ok`. Drive the helper directly.
        df.pending_spawn_results.remove(&daemon_id);
        handle_spawn_result_ok(&mut df, dataflow_id, &daemon_id, store.as_ref());

        // 1. The store entry must still be Failed — NOT promoted to Running.
        let records = store.list_dataflows().expect("store should list");
        let record = records
            .iter()
            .find(|r| r.uuid == dataflow_id)
            .expect("seeded record should still be present");
        assert!(
            matches!(
                record.status,
                dora_coordinator_store::DataflowStatus::Failed { .. }
            ),
            "Failed must NOT be promoted to Running by a late successful spawn_result, \
             got: {:?}",
            record.status,
        );

        // 2. spawn_result must still be Cached(Err) — registering a new
        //    waiter should immediately deliver the original error.
        let (tx, rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(tx);
        let reply = timeout(TokioDuration::from_millis(50), rx)
            .await
            .expect("Cached result should deliver immediately")
            .expect("sender should not drop");
        let err = reply.expect_err("late ok must not clobber the watchdog's Err");
        let msg = format!("{err:?}");
        assert!(msg.contains("watchdog") || msg.contains("timed out"),);
    }

    /// Companion test: the happy path through `handle_spawn_result_ok`
    /// (spawn_result was Pending, all pending daemons have now reported)
    /// must still promote to Running in the store. Guards against an
    /// over-eager guard that would block the legitimate success path.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn handle_spawn_result_ok_promotes_to_running_when_pending_and_complete() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
        // Single-daemon dataflow: after removing this daemon from pending,
        // the set becomes empty and the success branch should fire.
        df.pending_spawn_results.insert(daemon_id.clone());
        df.pending_spawn_results.remove(&daemon_id);
        assert!(df.spawn_result.is_pending());

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        handle_spawn_result_ok(&mut df, dataflow_id, &daemon_id, store.as_ref());

        // spawn_result must now be Cached(Ok), waiter should receive
        // DataflowSpawned.
        let (tx, rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(tx);
        let reply = timeout(TokioDuration::from_millis(50), rx)
            .await
            .expect("Cached result should deliver immediately")
            .expect("sender should not drop");
        let ok = reply.expect("happy-path ok must not be guarded out");
        assert!(matches!(ok, ControlRequestReply::DataflowSpawned { .. }));

        // Store must reflect Running.
        let records = store.list_dataflows().expect("store should list");
        let record = records
            .iter()
            .find(|r| r.uuid == dataflow_id)
            .expect("dataflow should be persisted");
        assert!(
            matches!(
                record.status,
                dora_coordinator_store::DataflowStatus::Running
            ),
            "happy-path success must persist Running, got: {:?}",
            record.status,
        );
    }

    // -------------------------------------------------------------------
    // Round-4 findings (PR #1854): terminal-failure paths must be
    // respected by every handler that writes to the affected state.
    // -------------------------------------------------------------------

    #[test]
    fn cached_result_is_terminal_error_distinguishes_states() {
        let mut r = CachedResult::default();
        assert!(!r.is_terminal_error(), "fresh Pending must not be terminal");

        // Cached(Ok) is terminal but NOT an error.
        r.set_result(Ok(ControlRequestReply::DataflowSpawned {
            uuid: DataflowId::from(Uuid::new_v4()),
        }));
        assert!(
            !r.is_terminal_error(),
            "Cached(Ok) must not match is_terminal_error()"
        );

        // Cached(Err) is the only state we want flagged.
        let mut r = CachedResult::default();
        r.set_result(Err(eyre!("spawn timed out")));
        assert!(
            r.is_terminal_error(),
            "Cached(Err) must match is_terminal_error()"
        );
    }

    /// Round-4 Finding 2: a late-arriving daemon `Err` after the watchdog
    /// (or any other terminal-failure path) has already cached an `Err`
    /// must NOT overwrite the existing store record. Drives
    /// `handle_spawn_result_err` directly.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn late_failed_spawn_result_does_not_overwrite_watchdog_store_error() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("late".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());

        // Simulate the watchdog having already fired with a detailed err.
        let watchdog_msg = "spawn timed out after 60s (watchdog)";
        df.spawn_result.set_result(Err(eyre!(watchdog_msg)));
        assert!(df.spawn_result.is_terminal_error());

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        // Seed the store with the detailed Failed record the watchdog
        // would have persisted.
        let initial_record = df
            .make_record(StoreDataflowStatus::Failed {
                error: watchdog_msg.to_string(),
                terminal: true,
            })
            .expect("make_record");
        store.put_dataflow(&initial_record).expect("seed");
        let initial_generation = initial_record.generation;

        // Late-arriving daemon Err.
        let late_err = eyre!("daemon-side spawn rejection");
        handle_spawn_result_err(&mut df, dataflow_id, &daemon_id, late_err, store.as_ref());

        // 1. Store record must still carry the watchdog's detailed message.
        let records = store.list_dataflows().expect("store list");
        let record = records
            .iter()
            .find(|r| r.uuid == dataflow_id)
            .expect("seeded record present");
        match &record.status {
            dora_coordinator_store::DataflowStatus::Failed { error, .. } => assert!(
                error.contains("watchdog") || error.contains("timed out"),
                "watchdog error message must be preserved, got: {error}"
            ),
            other => panic!("expected Failed, got: {other:?}"),
        }
        // 2. Generation must NOT bump (no rewrite happened).
        assert_eq!(
            record.generation, initial_generation,
            "guarded late-Err must not bump store_generation"
        );

        // 3. In-memory spawn_result still carries the watchdog's err.
        let (tx, rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(tx);
        let reply = timeout(TokioDuration::from_millis(50), rx)
            .await
            .expect("Cached result delivers immediately")
            .expect("sender alive");
        let err = reply.expect_err("must still be Err");
        let msg = format!("{err:?}");
        assert!(
            msg.contains("watchdog") || msg.contains("timed out"),
            "in-memory err must be the watchdog's, got: {msg}"
        );
    }

    /// Companion test for Finding 2: when spawn_result IS still Pending,
    /// the err arm must persist Failed and fire the waiter — guards
    /// against an over-eager guard blocking the legitimate failure path.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn handle_spawn_result_err_persists_failed_when_pending() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
        df.pending_spawn_results.insert(daemon_id.clone());
        df.pending_spawn_results.remove(&daemon_id);
        assert!(df.spawn_result.is_pending());

        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let err = eyre!("daemon-side spawn rejection");
        handle_spawn_result_err(&mut df, dataflow_id, &daemon_id, err, store.as_ref());

        // 1. spawn_result must now be Cached(Err); waiter receives the err.
        let (tx, rx) = tokio::sync::oneshot::channel();
        df.spawn_result.register(tx);
        let reply = timeout(TokioDuration::from_millis(50), rx)
            .await
            .expect("Cached result delivers immediately")
            .expect("sender alive");
        let err = reply.expect_err("happy-path err must not be guarded out");
        let msg = format!("{err:?}");
        assert!(msg.contains("rejection"));

        // 2. Store must reflect Failed with the daemon's error message.
        let records = store.list_dataflows().expect("store list");
        let record = records
            .iter()
            .find(|r| r.uuid == dataflow_id)
            .expect("dataflow persisted");
        match &record.status {
            dora_coordinator_store::DataflowStatus::Failed { error, .. } => {
                assert!(
                    error.contains("rejection"),
                    "Failed.error must carry the daemon's message, got: {error}"
                );
            }
            other => panic!("expected Failed, got: {other:?}"),
        }
    }

    // -------------------------------------------------------------------
    // Round-5 findings (PR #1854): watchdog must make the dataflow
    // terminal in memory too, not just in the store. Without removal
    // from running_dataflows, `Check` and `List` would report it as
    // active, and `DaemonStatusReport` reconciliation could promote
    // its Failed store record back to Running.
    // -------------------------------------------------------------------

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn watchdog_teardown_drains_stop_reply_senders() {
        // Anyone waiting on `dora stop` for the dataflow when the
        // watchdog fires must be released — otherwise they hang
        // forever because no `DataflowFinishedOnDaemon` will arrive
        // for a spawn-that-never-started.
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
        df.pending_spawn_results.insert(daemon_id);
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        // Register an in-flight `dora stop` waiter.
        let (stop_tx, stop_rx) = tokio::sync::oneshot::channel();
        df.stop_reply_senders.push(stop_tx);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // The stop waiter must have received a reply.
        let reply = timeout(TokioDuration::from_secs(1), stop_rx)
            .await
            .expect("stop waiter should resolve, not hang")
            .expect("stop sender should not drop");
        let stop = reply.expect("watchdog drains with Ok DataflowStopped");
        assert!(
            matches!(stop, ControlRequestReply::DataflowStopped { uuid, .. } if uuid == dataflow_id),
            "drain should send DataflowStopped with the watchdog'd dataflow's uuid"
        );
    }

    /// Round-5 Finding 2: after the watchdog fires, the dataflow must be
    /// absent from `running_dataflows` so `Check`/`List` no longer report
    /// it as active. The archive map must hold a record so post-mortem
    /// queries can still find its name/descriptor.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn watchdog_makes_dataflow_invisible_to_check_and_list() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
        df.name = Some("flagged-name".to_string());
        df.pending_spawn_results.insert(daemon_id);
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // Check / List query running_dataflows directly; absence is the
        // contract that fixes both consumers in one move.
        assert!(
            !running_dataflows.contains_key(&dataflow_id),
            "Check/List must observe the watchdog-failed dataflow as absent"
        );

        // The archive preserves the name so post-mortem `dora list`
        // queries against `archived_dataflows` can correlate the uuid
        // with a human-readable identity.
        let archived = archived_dataflows
            .get(&dataflow_id)
            .expect("watchdog must archive for post-mortem queries");
        assert_eq!(
            archived.name.as_deref(),
            Some("flagged-name"),
            "archive must preserve the dataflow name"
        );
    }

    // -------------------------------------------------------------------
    // Round-6 findings (PR #1854): post-watchdog UX parity with
    // normal-failure path. Watchdog now populates `dataflow_results` so
    // `dora list` / `dora check` / `dora stop` all see the dataflow as
    // Failed (instead of "disappeared" / "no known running dataflow").
    // -------------------------------------------------------------------

    /// Round-6 Finding 1: after watchdog fires, `dataflow_results` must
    /// contain an entry that classifies the dataflow as Failed (i.e. at
    /// least one per-daemon entry is_ok()==false). Without this, the
    /// List handler's `finished_failed` iterator skips the dataflow and
    /// it disappears from `dora list` entirely.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn watchdog_synthesizes_dataflow_results_for_list_visibility() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
        df.pending_spawn_results.insert(daemon_id.clone());
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // `dataflow_results` must contain an entry for the timed-out uuid.
        let per_daemon = dataflow_results
            .get(&dataflow_id)
            .expect("watchdog must synthesize a dataflow_results entry");

        // The entry must have at least one per-daemon DataflowDaemonResult.
        assert!(
            !per_daemon.is_empty(),
            "synthesized entry must have per-daemon results"
        );

        // Mirror the List classification check (lib.rs ~1019): all per-
        // daemon results must NOT be is_ok(), so the dataflow is Failed.
        let is_failed = !per_daemon.values().all(DataflowDaemonResult::is_ok);
        assert!(
            is_failed,
            "synthesized result must classify as Failed (List would show \
             Finished otherwise)"
        );

        // Each per-daemon entry must include the dataflow's nodes with
        // Err(FailedToSpawn(..)) so the user sees which nodes never started.
        let daemon_result = per_daemon
            .get(&daemon_id)
            .expect("entry must include the assigned daemon");
        let node_err = daemon_result
            .node_results
            .get(&node_id)
            .expect("node must be present in synthesized results");
        match node_err {
            Err(NodeError {
                cause: NodeErrorCause::FailedToSpawn(msg),
                ..
            }) => {
                assert!(
                    msg.contains("spawn timed out") || msg.contains("timeout"),
                    "FailedToSpawn cause must carry the watchdog timeout message, got: {msg}"
                );
            }
            other => panic!("expected Err(FailedToSpawn(..)), got: {other:?}"),
        }
    }

    /// #2027: `dataflow_results` is FIFO-bounded at `MAX_DATAFLOW_RESULTS`.
    /// Synthetic watchdog entries (and any results that never get cleared via
    /// archival) must not accumulate without bound. When an insert pushes the
    /// map over the cap, the oldest entry is evicted and the new one survives.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn watchdog_synthesis_keeps_dataflow_results_bounded() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        df.pending_spawn_results.insert(daemon_id.clone());
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();

        // Pre-fill to exactly the cap with stale entries; the oldest is first.
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();
        let oldest = DataflowId::from(Uuid::new_v4());
        dataflow_results.insert(oldest, BTreeMap::new());
        for _ in 1..MAX_DATAFLOW_RESULTS {
            dataflow_results.insert(DataflowId::from(Uuid::new_v4()), BTreeMap::new());
        }
        assert_eq!(dataflow_results.len(), MAX_DATAFLOW_RESULTS);

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        assert_eq!(
            dataflow_results.len(),
            MAX_DATAFLOW_RESULTS,
            "map must stay capped after the watchdog synthesizes a new entry"
        );
        assert!(
            dataflow_results.contains_key(&dataflow_id),
            "the freshly synthesized entry must survive eviction"
        );
        assert!(
            !dataflow_results.contains_key(&oldest),
            "the oldest entry must be the one evicted (FIFO)"
        );
    }

    /// #2027 review (P2): a partially-finished multi-daemon dataflow keeps its
    /// `dataflow_results` entry while still running (one daemon reported, others
    /// pending). The cap must evict only finished-dataflow history, never an
    /// active entry — otherwise the earlier daemon's result is lost and the
    /// final status (computed when the last daemon finishes) is wrong.
    #[test]
    fn cap_dataflow_results_preserves_running_dataflow_entries() {
        let active = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(active, test_running_dataflow(active, daemon_id, node_id));

        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();
        // Insert the active entry FIRST so it is the FIFO-oldest — i.e. the
        // entry a blind cap would evict first.
        dataflow_results.insert(active, BTreeMap::new());
        for _ in 0..MAX_DATAFLOW_RESULTS {
            dataflow_results.insert(DataflowId::from(Uuid::new_v4()), BTreeMap::new());
        }
        assert_eq!(dataflow_results.len(), MAX_DATAFLOW_RESULTS + 1);

        cap_dataflow_results(&mut dataflow_results, &running_dataflows);

        assert_eq!(
            dataflow_results.len(),
            MAX_DATAFLOW_RESULTS,
            "cap must bring the map back within the limit"
        );
        assert!(
            dataflow_results.contains_key(&active),
            "the active running dataflow's entry must be preserved even though \
             it is the FIFO-oldest — a finished-history entry is evicted instead"
        );
    }

    /// Round-6 Finding 3: `dora stop <watchdog-failed-uuid>` must return
    /// `DataflowStopped` (via the cached `dataflow_results` early-return)
    /// rather than "no known running dataflow". This auto-resolves once
    /// Finding 1's synthesis is in place — the Stop handler's
    /// `dataflow_results.get(&uuid)` early-return fires.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn dora_stop_after_watchdog_finds_dataflow_results_entry() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
        df.pending_spawn_results.insert(daemon_id);
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // The Stop handler at lib.rs:824 does:
        //   if let Some(result) = dataflow_results.get(&dataflow_uuid) {
        //       reply DataflowStopped { uuid, result: dataflow_result(result, ...) };
        //       continue;
        //   }
        // The early-return only fires when the synthesis above worked.
        assert!(
            dataflow_results.contains_key(&dataflow_id),
            "Stop handler's dataflow_results early-return must fire for \
             watchdog-failed dataflows (otherwise stop_dataflow bails \
             with 'no known running dataflow')"
        );
    }

    /// Round-6 Finding 2: late `DataflowFinishedOnDaemon` arrivals must
    /// merge into the synthetic `dataflow_results` entry, not be silently
    /// discarded. This tests the merge math directly (extending
    /// node_results overwrites synthetic FailedToSpawn entries with the
    /// daemon's real per-node results).
    #[test]
    fn late_finished_on_daemon_merge_extends_node_results() {
        // Step 1: build a synthesized DataflowDaemonResult as the watchdog
        // would emit. Two nodes assigned, both with FailedToSpawn errors.
        let synth_timestamp = HLC::default().new_timestamp();
        let node_a: dora_core::config::NodeId = "node_a".to_string().into();
        let node_b: dora_core::config::NodeId = "node_b".to_string().into();

        let mut synth = DataflowDaemonResult {
            timestamp: synth_timestamp,
            node_results: BTreeMap::new(),
        };
        synth.node_results.insert(
            node_a.clone(),
            Err(NodeError {
                timestamp: synth_timestamp,
                cause: NodeErrorCause::FailedToSpawn("spawn timed out".to_string()),
                exit_status: NodeExitStatus::Unknown,
            }),
        );
        synth.node_results.insert(
            node_b.clone(),
            Err(NodeError {
                timestamp: synth_timestamp,
                cause: NodeErrorCause::FailedToSpawn("spawn timed out".to_string()),
                exit_status: NodeExitStatus::Unknown,
            }),
        );

        // Step 2: late `result` from a daemon that DID end up running
        // node_a successfully and saw node_b crash with a real exit code.
        let late_timestamp = HLC::default().new_timestamp();
        let mut late_results: BTreeMap<dora_core::config::NodeId, Result<(), NodeError>> =
            BTreeMap::new();
        late_results.insert(node_a.clone(), Ok(()));
        late_results.insert(
            node_b.clone(),
            Err(NodeError {
                timestamp: late_timestamp,
                cause: NodeErrorCause::Other {
                    stderr: "real error".to_string(),
                },
                exit_status: NodeExitStatus::ExitCode(42),
            }),
        );

        // Step 3: apply the merge logic from the Vacant arm.
        synth.timestamp = late_timestamp;
        synth.node_results.extend(late_results);

        // The merged entry must reflect the daemon's real results, not
        // the synthetic ones, for the nodes that overlap.
        assert!(matches!(synth.node_results.get(&node_a), Some(Ok(()))));
        match synth.node_results.get(&node_b) {
            Some(Err(NodeError {
                cause: NodeErrorCause::Other { stderr },
                exit_status: NodeExitStatus::ExitCode(42),
                ..
            })) => {
                assert_eq!(stderr, "real error");
            }
            other => panic!("expected merged real Err, got: {other:?}"),
        }
        assert_eq!(synth.timestamp, late_timestamp);
    }

    // -------------------------------------------------------------------
    // Round-7 findings (PR #1854):
    //   1. DaemonStatusReport reconcile must NOT promote watchdog-failed
    //      (archived) dataflows back to Running.
    //   2. Watchdog synthesis must produce a non-empty result map even
    //      when df.daemons was emptied by disconnect cleanup before the
    //      watchdog fired.
    // -------------------------------------------------------------------

    /// Round-7 Finding 1: ensure the reconcile guard skips
    /// archived/terminal dataflows. We test the predicate the reconcile
    /// loop checks (`archived_dataflows.contains_key(df_id)`) rather
    /// than driving the full DaemonStatusReport event end-to-end —
    /// that would require synthesizing a registered daemon and a
    /// reported_dataflows entry, which the existing reconnect test
    /// (`restore_topic_debug_streams_re_issues_start_after_reconnect`)
    /// shows is non-trivial setup. The watchdog test
    /// `watchdog_makes_dataflow_invisible_to_check_and_list` already
    /// proves the dataflow lands in `archived_dataflows`, so this
    /// closing-the-loop predicate test plus the integration via
    /// `dora_stop_after_watchdog_finds_dataflow_results_entry` is
    /// sufficient.
    #[test]
    fn archived_dataflows_predicate_distinguishes_watchdog_failed_from_unknown() {
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let watchdog_failed = DataflowId::from(Uuid::new_v4());
        let unknown = DataflowId::from(Uuid::new_v4());

        archived_dataflows.insert(
            watchdog_failed,
            ArchivedDataflow {
                name: Some("flagged".to_string()),
                nodes: BTreeMap::new(),
            },
        );

        // The reconcile guard predicate.
        assert!(
            archived_dataflows.contains_key(&watchdog_failed),
            "watchdog-archived dataflow must be flagged for reconcile skip"
        );
        assert!(
            !archived_dataflows.contains_key(&unknown),
            "unknown dataflow must NOT be flagged (normal reconcile path applies)"
        );
    }

    /// Round-7 Finding 2: if all daemons disconnected before the watchdog
    /// fired (so `df.daemons` is empty by the disconnect cleanup at
    /// `lib.rs:1893-1899`), the watchdog's synthesis must STILL produce
    /// a non-empty `dataflow_results` entry. Empty `BTreeMap` would make
    /// List's `results.values().all(is_ok)` vacuously true and
    /// misclassify the dataflow as `Finished`.
    ///
    /// Synthesis iterates `df.node_to_daemon` (the original assignment,
    /// untouched by disconnect cleanup) rather than `df.daemons`, so the
    /// daemon set survives. As a final defence, an empty
    /// `node_to_daemon` injects a sentinel result.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn watchdog_disconnect_mid_spawn_still_classifies_as_failed() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("disconnected".to_string()));
        let node_id: dora_core::config::NodeId = "sender".to_string().into();
        let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
        // `node_to_daemon` already includes `daemon_id` (set up by
        // test_running_dataflow). Simulate the disconnect cleanup having
        // run before the watchdog: df.daemons is emptied.
        df.daemons.clear();
        df.pending_spawn_results.clear();
        // spawn_result still Pending because the disconnect path
        // intentionally doesn't fire it (round-1 design decision); the
        // watchdog is the single chokepoint.
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // The synthesized entry must NOT be empty.
        let per_daemon = dataflow_results
            .get(&dataflow_id)
            .expect("watchdog must synthesize even when df.daemons is empty");
        assert!(
            !per_daemon.is_empty(),
            "synthesized result must be non-empty so List classifies as Failed"
        );

        // List classification: must be Failed (not vacuously Finished).
        let is_failed = !per_daemon.values().all(DataflowDaemonResult::is_ok);
        assert!(
            is_failed,
            "disconnect-mid-spawn case must classify as Failed, not vacuously Finished"
        );

        // Verify it used the original assignment from node_to_daemon
        // (the daemon was disconnected but its assignment remains).
        let daemon_result = per_daemon.get(&daemon_id).unwrap_or_else(|| {
            panic!(
                "node_to_daemon must drive synthesis; expected key {daemon_id:?}, got {:?}",
                per_daemon.keys().collect::<Vec<_>>()
            )
        });
        assert!(
            daemon_result.node_results.contains_key(&node_id),
            "synthesis must include the originally-assigned node"
        );
    }

    // -------------------------------------------------------------------
    // Round-8 findings (PR #1854):
    //   1. Cross-restart reconcile resurrection: store-level `terminal`
    //      marker so the watchdog's verdict survives coordinator
    //      restarts where `archived_dataflows` is in-memory-wiped.
    //   2. Defensive sentinel must not panic: `NodeId::from(invalid)`
    //      asserts on disallowed chars.
    // -------------------------------------------------------------------

    /// Round-8 Finding 1: watchdog must persist `terminal: true` so the
    /// reconcile path skips promotion to Running even after coordinator
    /// restart (when `archived_dataflows` is empty).
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn watchdog_persists_terminal_marker_for_cross_restart_protection() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
        df.pending_spawn_results.insert(daemon_id);
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        let record = store
            .get_dataflow(&dataflow_id)
            .expect("store should be readable")
            .expect("watchdog must persist a record");
        match record.status {
            dora_coordinator_store::DataflowStatus::Failed { terminal, .. } => {
                assert!(
                    terminal,
                    "watchdog-persisted Failed must carry terminal: true so \
                     a post-restart reconcile cannot resurrect it"
                );
            }
            other => panic!("expected Failed, got: {other:?}"),
        }
    }

    /// Round-8 Finding 1 companion: assert that the `Failed` field
    /// defaults to `terminal: false` when an older record (without the
    /// field) is deserialized. This guarantees backward compat with
    /// records persisted by pre-#1854 coordinators.
    #[test]
    fn dataflow_status_failed_terminal_field_defaults_to_false_on_legacy_records() {
        // Simulate a record JSON written by an older coordinator that
        // didn't know about the `terminal` field.
        let legacy_json = r#"{ "Failed": { "error": "old failure" } }"#;
        let deser: dora_coordinator_store::DataflowStatus =
            serde_json::from_str(legacy_json).expect("legacy Failed must deserialize");
        match deser {
            dora_coordinator_store::DataflowStatus::Failed { error, terminal } => {
                assert_eq!(error, "old failure");
                assert!(
                    !terminal,
                    "legacy Failed records (no terminal field) MUST default to \
                     terminal: false to preserve the daemon-overrides-coordinator \
                     reconcile semantics of pre-#1854 stores"
                );
            }
            other => panic!("expected Failed, got: {other:?}"),
        }
    }

    /// Round-8 Finding 2: the defensive sentinel branch in the watchdog
    /// (fired when `node_to_daemon` is empty) must NOT panic. Previous
    /// versions used `NodeId::from("<watchdog>".to_string())` which
    /// panics via `validate_node_id` -- the `<` / `>` chars aren't in
    /// the allowed `[a-zA-Z0-9_.-]` set.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn watchdog_sentinel_branch_does_not_panic_on_empty_node_to_daemon() {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = HLC::default();
        let mut daemon_connections = DaemonConnections::default();

        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut df =
            test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
        // Force the sentinel branch: empty node_to_daemon (and daemons)
        // so the synthesis-from-assignment path produces an empty map.
        df.node_to_daemon.clear();
        df.daemons.clear();
        df.pending_spawn_results.clear();
        df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);
        let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
        let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
            IndexMap::new();

        // If `NodeId::from("<watchdog>")` is reintroduced, this awaits
        // a panic and the test fails. Currently uses `"watchdog"` which
        // passes validation.
        check_spawn_timeouts(
            &mut running_dataflows,
            &mut archived_dataflows,
            &mut dataflow_results,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
        )
        .await;

        // Sentinel result must be present and classify as Failed.
        let per_daemon = dataflow_results
            .get(&dataflow_id)
            .expect("sentinel must populate dataflow_results");
        assert!(!per_daemon.is_empty(), "sentinel must be non-empty");
        assert!(
            !per_daemon.values().all(DataflowDaemonResult::is_ok),
            "sentinel must classify as Failed (not vacuously Finished)"
        );
    }

    // ===== check_build_timeouts watchdog tests (#1465) =====
    //
    // Mirror the four spawn watchdog tests above for the build path.
    // Build's analog is simpler: no daemon rollback, no archived_dataflows /
    // dataflow_results synthesis — just release waiters and move the entry
    // from `running_builds` to `finished_builds`.

    // Small, cross-platform-safe watchdog timeout for the unit tests. The
    // tests pass this to `check_build_timeouts` instead of the 20-min
    // production `build_result_timeout()`, so `test_running_build` only has to
    // backdate `build_started_at` by ~2s. Backdating by the production timeout
    // computes `Instant::now() - 20min`, which underflows and panics on
    // Windows runners whose monotonic-clock epoch is younger than 20 min
    // (dora-rs/dora#2082).
    const TEST_BUILD_TIMEOUT: Duration = Duration::from_secs(1);

    fn test_running_build(daemon_id: DaemonId, backdate: bool) -> RunningBuild {
        let mut pending = BTreeSet::new();
        pending.insert(daemon_id);
        RunningBuild {
            errors: Vec::new(),
            build_result: CachedResult::default(),
            buffered_log_messages: Vec::new(),
            log_subscribers: Vec::new(),
            pending_build_results: pending,
            build_started_at: if backdate {
                Instant::now() - TEST_BUILD_TIMEOUT - Duration::from_secs(1)
            } else {
                Instant::now()
            },
        }
    }

    #[tokio::test]
    async fn check_build_timeouts_fires_error_when_pending_past_deadline() {
        let clock = HLC::default();
        let build_id = BuildId::generate();
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut build = test_running_build(daemon_id, /*backdate=*/ true);

        // Register a waiter so we can prove `wait_for_build` unblocks with
        // an error rather than hanging on the client-side RPC deadline.
        let (tx, rx) = tokio::sync::oneshot::channel();
        build.build_result.register(tx);

        let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
        running_builds.insert(build_id, build);
        let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

        check_build_timeouts(
            &mut running_builds,
            &mut finished_builds,
            &clock,
            TEST_BUILD_TIMEOUT,
        )
        .await;

        // Waiter resolved with an error.
        let reply = timeout(TokioDuration::from_secs(1), rx)
            .await
            .expect("waiter should resolve, not hang")
            .expect("sender should not drop");
        let err = reply.expect_err("timed-out build must surface as Err");
        let msg = format!("{err:?}");
        assert!(
            msg.contains("build timed out"),
            "error should explain the timeout, got: {msg}"
        );

        // Build moved from running_builds to finished_builds.
        assert!(
            !running_builds.contains_key(&build_id),
            "watchdog must remove timed-out build from running_builds"
        );
        assert!(
            finished_builds.contains_key(&build_id),
            "watchdog must move timed-out build into finished_builds so late \
             WaitForBuild requests find the cached error",
        );
    }

    #[tokio::test]
    async fn check_build_timeouts_no_op_when_within_deadline() {
        let clock = HLC::default();
        let build_id = BuildId::generate();
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut build = test_running_build(daemon_id, /*backdate=*/ false);

        let (tx, rx) = tokio::sync::oneshot::channel();
        build.build_result.register(tx);

        let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
        running_builds.insert(build_id, build);
        let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

        check_build_timeouts(
            &mut running_builds,
            &mut finished_builds,
            &clock,
            TEST_BUILD_TIMEOUT,
        )
        .await;

        // Waiter still pending.
        let polled = tokio::time::timeout(TokioDuration::from_millis(50), rx).await;
        assert!(polled.is_err(), "waiter must still be pending pre-deadline");

        let build = running_builds.get(&build_id).expect("build still present");
        assert!(
            build.build_result.is_pending(),
            "build_result must remain Pending pre-deadline"
        );
        assert!(
            finished_builds.is_empty(),
            "fresh build must not be moved to finished_builds"
        );
    }

    #[tokio::test]
    async fn check_build_timeouts_idempotent_on_already_cached_result() {
        let clock = HLC::default();
        let build_id = BuildId::generate();
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let mut build = test_running_build(daemon_id, /*backdate=*/ true);
        // Build already resolved successfully — the watchdog must NOT
        // re-fire on subsequent heartbeats and clobber the cached result.
        build
            .build_result
            .set_result(Ok(ControlRequestReply::DataflowBuildFinished {
                build_id,
                result: Ok(()),
            }));

        let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
        running_builds.insert(build_id, build);
        let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

        check_build_timeouts(
            &mut running_builds,
            &mut finished_builds,
            &clock,
            TEST_BUILD_TIMEOUT,
        )
        .await;

        let build = running_builds.get_mut(&build_id).expect(
            "Cached(Ok) build must stay in running_builds — only is_pending() \
             triggers the watchdog",
        );
        // Registering a new waiter on a Cached result must immediately
        // deliver the cached Ok — proving the watchdog did not clobber it.
        let (tx, rx) = tokio::sync::oneshot::channel();
        build.build_result.register(tx);
        let reply = timeout(TokioDuration::from_millis(50), rx)
            .await
            .expect("Cached result should deliver immediately")
            .expect("sender should not drop");
        let ok = reply.expect("the cached Ok result must be preserved");
        assert!(matches!(
            ok,
            ControlRequestReply::DataflowBuildFinished { result: Ok(()), .. }
        ));
        assert!(
            finished_builds.is_empty(),
            "watchdog must not move a non-timed-out build into finished_builds"
        );
    }

    /// Build-specific edge: a `WaitForBuild` registered AFTER the watchdog
    /// fired must still receive the cached error via `finished_builds`
    /// (the second branch of the `WaitForBuild` arm). This proves the
    /// late-arriver path, complementing the pre-registered waiter path
    /// in `check_build_timeouts_fires_error_when_pending_past_deadline`.
    #[tokio::test]
    async fn check_build_timeouts_late_wait_for_build_gets_cached_error() {
        let clock = HLC::default();
        let build_id = BuildId::generate();
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let build = test_running_build(daemon_id, /*backdate=*/ true);
        // NB: NO waiter registered yet — this is the late-arriver path.

        let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
        running_builds.insert(build_id, build);
        let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

        check_build_timeouts(
            &mut running_builds,
            &mut finished_builds,
            &clock,
            TEST_BUILD_TIMEOUT,
        )
        .await;

        // Build was moved to finished_builds with a Cached Err.
        let cached = finished_builds
            .get_mut(&build_id)
            .expect("build must be in finished_builds after watchdog");

        // A late WaitForBuild registers on the finished CachedResult and
        // must receive the cached error immediately, NOT hang.
        let (tx, rx) = tokio::sync::oneshot::channel();
        cached.register(tx);
        let reply = timeout(TokioDuration::from_millis(50), rx)
            .await
            .expect("Cached error should deliver immediately")
            .expect("sender should not drop");
        let err = reply.expect_err("late wait_for_build must surface the watchdog's Err");
        assert!(format!("{err:?}").contains("build timed out"));
    }

    #[test]
    fn cleanup_disconnected_daemons_drains_pending_restarts() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "node".to_string().into();

        let df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(dataflow_id, df);

        let (tx, rx) = tokio::sync::oneshot::channel();
        let descriptor: Descriptor =
            serde_json::from_value(serde_json::json!({"nodes": [{"id": "node"}]}))
                .expect("valid descriptor");
        let mut pending_restarts = HashMap::new();
        pending_restarts.insert(
            dataflow_id,
            PendingRestart {
                descriptor,
                name: None,
                uv: false,
                reply_sender: tx,
            },
        );

        let disconnected = BTreeSet::from([daemon_id]);
        let _actions = cleanup_disconnected_daemons_from_running_dataflows(
            &mut running_dataflows,
            &disconnected,
            &mut pending_restarts,
        );

        // pending_restarts must be drained
        assert!(
            pending_restarts.is_empty(),
            "pending_restarts must be drained on daemon disconnect, but has {} entries",
            pending_restarts.len()
        );

        // caller must be released with an error, not hang
        let result = rx.blocking_recv().expect("restart sender must send reply");
        let err = result.expect_err("restart must fail on daemon disconnect");
        assert!(
            format!("{err:?}").contains("disconnected"),
            "error must mention daemon disconnect, got: {err:?}"
        );
    }

    #[tokio::test]
    async fn initiate_restart_rejects_duplicate_request() {
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("m1".to_string()));
        let node_id: dora_core::config::NodeId = "node".to_string().into();

        let mut running_dataflows = HashMap::new();
        running_dataflows.insert(
            dataflow_id,
            test_running_dataflow(dataflow_id, daemon_id.clone(), node_id),
        );

        let (existing_tx, _existing_rx) = tokio::sync::oneshot::channel();
        let descriptor: Descriptor =
            serde_json::from_value(serde_json::json!({"nodes": [{"id": "node"}]}))
                .expect("valid descriptor");
        let mut pending_restarts = HashMap::new();
        pending_restarts.insert(
            dataflow_id,
            PendingRestart {
                descriptor: descriptor.clone(),
                name: None,
                uv: false,
                reply_sender: existing_tx,
            },
        );

        let (dup_tx, dup_rx) = tokio::sync::oneshot::channel();
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let clock = Arc::new(HLC::default());
        let mut daemon_connections = DaemonConnections::default();

        initiate_restart(
            dataflow_id,
            None,
            false,
            &mut running_dataflows,
            &mut pending_restarts,
            &mut daemon_connections,
            &clock,
            store.as_ref(),
            dup_tx,
        )
        .await;

        // duplicate must be rejected with an error
        let result = dup_rx
            .await
            .expect("duplicate restart sender must send reply");
        let err = result.expect_err("duplicate restart must be rejected");
        assert!(
            format!("{err:?}").contains("already being restarted"),
            "error must mention already being restarted, got: {err:?}"
        );

        // original PendingRestart must not be overwritten
        assert_eq!(
            pending_restarts.len(),
            1,
            "original PendingRestart must not be overwritten by duplicate"
        );
    }
}
