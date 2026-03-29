use aligned_vec::{AVec, ConstAlign};
use crossbeam::queue::ArrayQueue;
use dora_core::{
    build::{self, BuildInfo, PrevGitSource, TracingBuildLogger},
    config::{DataId, Input, InputMapping, NodeId, NodeRunConfig},
    descriptor::{
        CoreNodeKind, DYNAMIC_SOURCE, Descriptor, DescriptorExt, ResolvedNode, RuntimeNode,
        read_as_descriptor,
    },
    topics::{
        DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST, open_zenoh_session,
        open_zenoh_session_as_daemon, zenoh_output_publish_topic,
    },
    uhlc::HLC,
};
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{
        DaemonId, DataMessage, DropToken, GitSource, LogLevel, NodeError, NodeErrorCause,
        NodeExitStatus,
    },
    coordinator_to_cli::DataflowResult,
    coordinator_to_daemon::SpawnDataflowNodes,
    daemon_to_coordinator::DataflowDaemonResult,
    daemon_to_daemon::InterDaemonEvent,
    daemon_to_node::{DaemonReply, NodeConfig, NodeDropEvent, NodeEvent},
    descriptor::{NodeSource, RestartPolicy},
    metadata::{self, ArrowTypeInfo},
    node_to_daemon::{DynamicNodeEvent, Timestamped},
    tarpc,
};
use dora_node_api::{Parameter, arrow::datatypes::DataType};
use eyre::{Context, ContextCompat, Result, bail, eyre};
use futures::{FutureExt, TryFutureExt, future, stream};
use futures_concurrency::stream::Merge;
use local_listener::DynamicNodeEventWrapper;
use log::{DaemonLogger, DataflowLogger, Logger};
use pending::PendingNodes;
use process_wrap::tokio::TokioChildWrapper;
use shared_memory_extended::ShmemConf;
use spawn::Spawner;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap, VecDeque},
    env::current_dir,
    future::Future,
    io,
    net::SocketAddr,
    path::{Path, PathBuf},
    pin::pin,
    sync::{
        Arc,
        atomic::{self, AtomicBool, AtomicU32},
    },
    time::{Duration, Instant},
};
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncSeekExt},
    net::TcpStream,
    sync::{
        broadcast,
        mpsc::{self, UnboundedSender},
        oneshot::{self, Sender},
    },
};
use tokio_stream::{Stream, StreamExt, wrappers::ReceiverStream};
use tracing::{error, warn};
use uuid::{NoContext, Timestamp, Uuid};

pub use flume;
pub use log::LogDestination;

mod coordinator;
mod extract_err_from_stderr;
mod local_listener;
mod log;
mod node_communication;
mod pending;
mod socket_stream_utils;
mod spawn;
pub(crate) mod state;

#[cfg(feature = "telemetry")]
use dora_tracing::telemetry::serialize_context;
#[cfg(feature = "telemetry")]
use tracing_opentelemetry::OpenTelemetrySpanExt;

use crate::{extract_err_from_stderr::extract_err_from_stderr, pending::DataflowStatus};

const STDERR_LOG_LINES_MAX: usize = 500;

static EMPTY_SET: BTreeSet<DataId> = BTreeSet::new();

pub struct Daemon {
    pub(crate) state: Arc<state::DaemonState>,

    /// used for testing and examples
    exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
    /// set on ctrl-c
    exit_when_all_finished: bool,

    logger: DaemonLogger,

    /// System instance for metrics collection (reused across calls)
    metrics_system: sysinfo::System,
}

type DaemonRunResult = BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>;

struct NodeBuildTask<F> {
    node_id: NodeId,
    dynamic_node: bool,
    task: F,
}

impl Daemon {
    pub async fn run(
        coordinator_addr: SocketAddr,
        machine_id: Option<String>,
        local_listen_port: u16,
    ) -> eyre::Result<()> {
        let clock = Arc::new(HLC::default());

        let mut ctrlc_events = set_up_ctrlc_handler(clock.clone())?;
        let (remote_daemon_events_tx, remote_daemon_events_rx) = flume::bounded(10);

        // Use a large channel capacity to prevent deadlock
        let (dora_events_tx, dora_events_rx) = mpsc::channel(1000);

        let zenoh_session = open_zenoh_session_as_daemon(Some(coordinator_addr.ip()))
            .await
            .wrap_err("failed to open zenoh session")?;

        // Build shared state early so the RPC server can use it.
        // `daemon_id` and `coordinator_client` are set via OnceLock after registration.
        let daemon_state = Arc::new(state::DaemonState::new(
            clock.clone(),
            dora_events_tx,
            Some(zenoh_session),
            Some(remote_daemon_events_tx),
        ));

        let ((daemon_id, coordinator_client), incoming_events) = {
            let incoming_events = set_up_event_stream(
                coordinator_addr,
                &machine_id,
                &clock,
                daemon_state.clone(),
                remote_daemon_events_rx,
                local_listen_port,
            );

            // finish early if ctrl-c is pressed during event stream setup
            let ctrl_c = pin!(ctrlc_events.recv());
            match futures::future::select(ctrl_c, pin!(incoming_events)).await {
                future::Either::Left((_ctrl_c, _)) => {
                    tracing::info!("received ctrl-c signal -> stopping daemon");
                    return Ok(());
                }
                future::Either::Right((events, _)) => events?,
            }
        };

        daemon_state.set_daemon_id(daemon_id.clone());
        daemon_state.set_coordinator_client(coordinator_client);

        let log_destination = match daemon_state.zenoh_session.as_ref() {
            Some(session) => LogDestination::Zenoh {
                session: session.clone(),
                publishers: std::collections::HashMap::new(),
            },
            None => {
                tracing::warn!(
                    "no zenoh session available; log messages will only be printed locally"
                );
                LogDestination::Tracing
            }
        };

        Self::run_general(
            (ReceiverStream::new(ctrlc_events), incoming_events).merge(),
            daemon_id,
            None,
            daemon_state,
            dora_events_rx,
            log_destination,
        )
        .await
        .map(|_| ())
    }

    pub async fn run_dataflow(
        dataflow_path: &Path,
        build_id: Option<BuildId>,
        local_build: Option<BuildInfo>,
        session_id: SessionId,
        uv: bool,
        log_destination: LogDestination,
        write_events_to: Option<PathBuf>,
        stop_after: Option<Duration>,
    ) -> eyre::Result<DataflowResult> {
        let working_dir = dataflow_path
            .canonicalize()
            .context("failed to canonicalize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
            .to_owned();

        let descriptor = read_as_descriptor(dataflow_path).await?;
        if let Some(node) = descriptor.nodes.iter().find(|n| n.deploy.is_some()) {
            eyre::bail!(
                "node {} has a `deploy` section, which is not supported in `dora run`\n\n
                Instead, you need to spawn a `dora coordinator` and one or more `dora daemon`
                instances and then use `dora start`.",
                node.id
            )
        }

        descriptor.check(&working_dir)?;
        let nodes = descriptor.resolve_aliases_and_set_defaults()?;

        let (events_tx, events_rx) = flume::bounded(10);
        if nodes
            .iter()
            .find(|(_n, resolved_nodes)| resolved_nodes.kind.dynamic())
            .is_some()
        {
            // Spawn local listener for dynamic nodes
            let _listen_port = local_listener::spawn_listener_loop(
                (LOCALHOST, DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT).into(),
                events_tx,
            )
            .await?;
        }
        let dynamic_node_events = events_rx.into_stream().map(|e| Timestamped {
            inner: Event::DynamicNode(e.inner),
            timestamp: e.timestamp,
        });

        let dataflow_id = Uuid::new_v7(Timestamp::now(NoContext));
        let spawn_command = SpawnDataflowNodes {
            build_id,
            session_id,
            dataflow_id,
            local_working_dir: Some(working_dir),
            spawn_nodes: nodes.keys().cloned().collect(),
            nodes,
            dataflow_descriptor: descriptor,
            uv,
            write_events_to,
        };

        let clock = Arc::new(HLC::default());

        let ctrlc_events = ReceiverStream::new(set_up_ctrlc_handler(clock.clone())?);

        // Set up optional timeout for --stop-after
        let timeout_events = if let Some(duration) = stop_after {
            let clock = clock.clone();
            let (tx, rx) = tokio::sync::mpsc::channel(1);
            tokio::spawn(async move {
                tokio::time::sleep(duration).await;
                tracing::info!("stop-after timeout reached ({duration:?}) -> stopping dataflow");
                let _ = tx
                    .send(Timestamped {
                        inner: Event::StopAfter(duration),
                        timestamp: clock.new_timestamp(),
                    })
                    .await;
            });
            ReceiverStream::new(rx)
        } else {
            // Create an empty stream that never emits events
            ReceiverStream::new(tokio::sync::mpsc::channel(1).1)
        };

        let all_nodes_dynamic = spawn_command.nodes.values().all(|n| n.kind.dynamic());
        let exit_when_done: BTreeSet<(Uuid, NodeId)> = spawn_command
            .nodes
            .values()
            .filter(|n| !n.kind.dynamic())
            .map(|n| (spawn_command.dataflow_id, n.id.clone()))
            .collect();

        // In standalone mode (`dora run`), inject the spawn as a SpawnRequest event
        // instead of going through the coordinator.
        let (reply_tx, reply_rx) = oneshot::channel();
        let spawn_event_clock = clock.clone();
        let spawn_event = stream::once(async move {
            Timestamped {
                inner: Event::SpawnRequest {
                    build_id: spawn_command.build_id,
                    dataflow_id: spawn_command.dataflow_id,
                    local_working_dir: spawn_command.local_working_dir,
                    nodes: spawn_command.nodes,
                    dataflow_descriptor: spawn_command.dataflow_descriptor,
                    spawn_nodes: spawn_command.spawn_nodes,
                    uv: spawn_command.uv,
                    write_events_to: spawn_command.write_events_to,
                    reply_tx,
                },
                timestamp: spawn_event_clock.new_timestamp(),
            }
        });
        let events = (
            spawn_event,
            ctrlc_events,
            timeout_events,
            dynamic_node_events,
        )
            .merge();

        let builds_map: BTreeMap<BuildId, BuildInfo> = if let Some(local_build) = local_build {
            let Some(build_id) = build_id else {
                bail!("no build_id, but local_build set")
            };
            let mut builds = BTreeMap::new();
            builds.insert(build_id, local_build);
            builds
        } else {
            Default::default()
        };

        let zenoh_session = open_zenoh_session(None)
            .await
            .wrap_err("failed to open zenoh session")?;
        let daemon_id = DaemonId::new(None);
        let (dora_events_tx, dora_events_rx) = mpsc::channel(1000);
        let daemon_state = Arc::new(state::DaemonState::new_standalone(
            clock.clone(),
            daemon_id.clone(),
            dora_events_tx,
            zenoh_session,
            builds_map,
        ));

        let run_result = Self::run_general(
            Box::pin(events),
            daemon_id,
            Some(exit_when_done),
            daemon_state,
            dora_events_rx,
            log_destination,
        );

        let spawn_result = reply_rx
            .map_err(|err| eyre!("failed to receive spawn result: {err}"))
            .and_then(|r| async { r.map_err(|err| eyre!(err)) });

        let (mut dataflow_results, ()) = future::try_join(run_result, spawn_result).await?;

        let node_results = match dataflow_results.remove(&dataflow_id) {
            Some(results) => results,
            None if all_nodes_dynamic => {
                // All nodes are dynamic - they don't send SpawnedNodeResult events,
                // so there are no node results to report. This is expected and means success.
                BTreeMap::new()
            }
            None => {
                return Err(eyre::eyre!("no node results for dataflow_id {dataflow_id}"));
            }
        };

        Ok(DataflowResult {
            uuid: dataflow_id,
            timestamp: clock.new_timestamp(),
            node_results,
        })
    }

    /// Shared daemon run logic used by both `run` (with coordinator) and
    /// `run_dataflow` (standalone).
    #[allow(clippy::too_many_arguments)]
    async fn run_general(
        external_events: impl Stream<Item = Timestamped<Event>> + Unpin,
        daemon_id: DaemonId,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        daemon_state: Arc<state::DaemonState>,
        dora_events_rx: mpsc::Receiver<Timestamped<Event>>,
        log_destination: LogDestination,
    ) -> eyre::Result<DaemonRunResult> {
        let clock = daemon_state.clock.clone();
        let daemon = Self {
            logger: Logger {
                destination: log_destination,
                clock: clock.clone(),
            }
            .for_daemon(daemon_id.clone()),
            state: daemon_state,
            exit_when_done,
            exit_when_all_finished: false,
            metrics_system: sysinfo::System::new(),
        };

        let dora_events = ReceiverStream::new(dora_events_rx);
        let watchdog_clock = daemon.state.clock.clone();
        let watchdog_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(5),
        ))
        .map(move |_| Timestamped {
            inner: Event::HeartbeatInterval,
            timestamp: watchdog_clock.new_timestamp(),
        });

        let metrics_clock = daemon.state.clock.clone();
        let metrics_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(2), // Collect metrics every 2 seconds
        ))
        .map(move |_| Timestamped {
            inner: Event::MetricsInterval,
            timestamp: metrics_clock.new_timestamp(),
        });

        let events = (
            external_events,
            dora_events,
            watchdog_interval,
            metrics_interval,
        )
            .merge();
        daemon.run_inner(events).await
    }

    #[tracing::instrument(skip(incoming_events, self), fields(daemon_id = ?self.state.daemon_id()))]
    async fn run_inner(
        mut self,
        incoming_events: impl Stream<Item = Timestamped<Event>> + Unpin,
    ) -> eyre::Result<DaemonRunResult> {
        let mut events = incoming_events;

        while let Some(event) = events.next().await {
            let Timestamped { inner, timestamp } = event;
            if let Err(err) = self.state.clock.update_with_timestamp(&timestamp) {
                tracing::warn!("failed to update HLC with incoming event timestamp: {err}");
            }

            // used below for checking the duration of event handling
            let start = Instant::now();
            let event_kind = inner.kind();

            match inner {
                Event::Daemon(event) => {
                    self.handle_inter_daemon_event(event).await?;
                }
                Event::Node {
                    dataflow_id: dataflow,
                    node_id,
                    event,
                } => self.handle_node_event(event, dataflow, node_id).await?,
                Event::Dora(event) => self.handle_dora_event(event).await?,
                Event::DynamicNode(event) => self.handle_dynamic_node_event(event).await?,
                Event::HeartbeatInterval => {
                    if let Some(client) = self.state.coordinator_client() {
                        // Fire-and-forget: notify the coordinator we're alive.
                        // Don't block the event loop waiting for the RPC response.
                        let client = client.clone();
                        tokio::spawn(async move {
                            let _ = client.heartbeat(tarpc::context::current()).await;
                        });

                        let last_hb = self.state.last_coordinator_heartbeat.lock().await;
                        if last_hb.elapsed() > Duration::from_secs(20) {
                            bail!("lost connection to coordinator")
                        }
                    }
                }
                Event::Destroy => {
                    tracing::info!("received destroy command -> exiting");
                    break;
                }
                Event::SpawnRequest {
                    build_id,
                    dataflow_id,
                    local_working_dir,
                    nodes,
                    dataflow_descriptor,
                    spawn_nodes,
                    uv,
                    write_events_to,
                    reply_tx,
                } => {
                    let result = self
                        .handle_spawn_request(
                            build_id,
                            dataflow_id,
                            local_working_dir,
                            nodes,
                            dataflow_descriptor,
                            spawn_nodes,
                            uv,
                            write_events_to,
                        )
                        .await;
                    let _ = reply_tx.send(result);
                }
                Event::StopDataflowRequest {
                    dataflow_id,
                    grace_duration,
                    force,
                    reply_tx,
                } => {
                    let result = async {
                        // First, handle pending nodes (may do RPC to coordinator)
                        // while holding the DashMap guard briefly for extraction.
                        {
                            let mut logger = self.logger.for_dataflow(dataflow_id);
                            let mut dataflow =
                                self.state.running.get_mut(&dataflow_id).ok_or_else(|| {
                                    format!("no running dataflow with ID `{dataflow_id}`")
                                })?;
                            let df = &mut *dataflow;
                            df.pending_nodes
                                .handle_dataflow_stop(
                                    &self.state.coordinator_client().cloned(),
                                    &self.state.clock,
                                    &mut df.cascading_error_causes,
                                    &df.dynamic_nodes,
                                    &mut logger,
                                )
                                .await
                                .map_err(|err| format!("{err:?}"))?;
                        }
                        // DashMap guard is dropped — safe to re-acquire.
                        let finish_when = {
                            let mut dataflow =
                                self.state.running.get_mut(&dataflow_id).ok_or_else(|| {
                                    format!("no running dataflow with ID `{dataflow_id}`")
                                })?;
                            dataflow
                                .stop_all_after_pending(&self.state, grace_duration, force)
                                .map_err(|err| format!("{err:?}"))?
                        };
                        // DashMap guard is dropped — safe to call finish_dataflow.
                        if matches!(finish_when, FinishDataflowWhen::Now) {
                            self.state
                                .finish_dataflow(dataflow_id)
                                .await
                                .map_err(|err| format!("{err:?}"))?;
                        }
                        Ok(())
                    }
                    .await;
                    let _ = reply_tx.send(result);
                }
                Event::MetricsInterval => {
                    self.collect_and_send_metrics().await?;
                }
                Event::CtrlC => {
                    tracing::info!("received ctrlc signal -> stopping all dataflows");
                    self.trigger_manual_stop().await?;
                    if self.state.running.is_empty() {
                        break;
                    }
                }
                Event::SecondCtrlC => {
                    tracing::warn!("received second ctrlc signal -> exit immediately");
                    bail!("received second ctrl-c signal");
                }
                Event::StopAfter(duration) => {
                    tracing::info!("stopping after {duration:?} as requested");
                    self.trigger_manual_stop().await?;
                    if self.state.running.is_empty() {
                        break;
                    }
                }
                Event::DaemonError(err) => {
                    bail!(err.wrap_err("fatal error from background task"));
                }
                Event::SpawnNodeResult {
                    dataflow_id,
                    node_id,
                    dynamic_node,
                    result,
                } => match result {
                    Ok(running_node) => {
                        if let Some(mut dataflow) = self.state.running.get_mut(&dataflow_id) {
                            if let Some(ref abort_handle) = running_node.listener_abort_handle {
                                dataflow
                                    ._listener_tasks
                                    .push(ListenerTask(abort_handle.clone()));
                            }
                            dataflow.running_nodes.insert(node_id, running_node);
                        } else {
                            tracing::error!(
                                "failed to handle SpawnNodeResult: no running dataflow with ID {dataflow_id}"
                            );
                        }
                    }
                    Err(error) => {
                        self.state
                            .dataflow_node_results
                            .entry(dataflow_id)
                            .or_default()
                            .insert(node_id.clone(), Err(error));
                        self.handle_node_stop(dataflow_id, &node_id, dynamic_node)
                            .await?;
                    }
                },
                Event::SpawnDataflowResult {
                    dataflow_id,
                    result,
                } => {
                    if let Some(client) = self.state.coordinator_client() {
                        let client = client.clone();
                        let result = result.map_err(|err| format!("{err:?}"));
                        tokio::spawn(async move {
                            if let Err(err) = client
                                .spawn_result(tarpc::context::current(), dataflow_id, result)
                                .await
                            {
                                tracing::error!(
                                    ?err,
                                    "failed to send spawn_result notification to coordinator"
                                );
                            }
                        });
                    }
                }
                Event::NodeStopped {
                    dataflow_id,
                    node_id,
                } => {
                    if let Some(exit_when_done) = &mut self.exit_when_done {
                        exit_when_done.remove(&(dataflow_id, node_id));
                        if exit_when_done.is_empty() {
                            tracing::info!(
                                "exiting daemon because all required dataflows are finished"
                            );
                            break;
                        }
                    }
                    // Exit when all dataflows are finished after Ctrl-C
                    if self.exit_when_all_finished && self.state.running.is_empty() {
                        tracing::info!("exiting daemon because all dataflows are finished");
                        break;
                    }
                }
            }

            // warn if event handling took too long -> the main loop should never be blocked for too long
            let elapsed = start.elapsed();
            if elapsed > Duration::from_millis(100) {
                tracing::warn!(
                    "Daemon took {}ms for handling event: {event_kind}",
                    elapsed.as_millis()
                );
            }
        }

        if let Some(client) = self.state.coordinator_client() {
            let ctx = tarpc::context::current();
            if let Err(err) = client.daemon_exit(ctx).await {
                tracing::error!(
                    ?err,
                    "failed to send daemon_exit notification to coordinator"
                );
            }
        }

        Ok(self
            .state
            .dataflow_node_results
            .iter()
            .map(|entry| (*entry.key(), entry.value().clone()))
            .collect())
    }

    async fn trigger_manual_stop(&mut self) -> eyre::Result<()> {
        // Collect dataflow IDs first, then process one at a time to avoid
        // holding DashMap guards across `.await` points.
        let dataflow_ids: Vec<DataflowId> = self
            .state
            .running
            .iter()
            .map(|entry| *entry.key())
            .collect();

        let mut dataflows_to_finish = Vec::new();
        for dataflow_id in dataflow_ids {
            if let Some(mut dataflow) = self.state.running.get_mut(&dataflow_id) {
                let mut logger = self.logger.for_dataflow(dataflow_id);
                let finish_when = dataflow
                    .stop_all(&self.state, None, false, &mut logger)
                    .await?;
                if matches!(finish_when, FinishDataflowWhen::Now) {
                    dataflows_to_finish.push(dataflow_id);
                }
            }
        }

        // DashMap guards are dropped — safe to call finish_dataflow.
        for dataflow_id in dataflows_to_finish {
            self.state.finish_dataflow(dataflow_id).await?;
        }

        self.exit_when_all_finished = true;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    async fn handle_spawn_request(
        &mut self,
        build_id: Option<BuildId>,
        dataflow_id: DataflowId,
        local_working_dir: Option<PathBuf>,
        nodes: BTreeMap<NodeId, ResolvedNode>,
        dataflow_descriptor: Descriptor,
        spawn_nodes: BTreeSet<NodeId>,
        uv: bool,
        write_events_to: Option<PathBuf>,
    ) -> Result<(), String> {
        match dataflow_descriptor.communication.remote {
            dora_core::config::RemoteCommunicationConfig::Tcp => {}
        }

        // Resolve base working dir — for spawn we use the daemon's working dir
        let base_working_dir = match local_working_dir {
            Some(working_dir) => {
                if working_dir.exists() {
                    Ok(working_dir)
                } else {
                    Err(format!(
                        "working directory does not exist: {}",
                        working_dir.display(),
                    ))
                }
            }
            None => {
                let daemon_working_dir =
                    current_dir().map_err(|e| format!("failed to get daemon working dir: {e}"))?;
                Ok(daemon_working_dir)
            }
        }?;

        let result = self
            .spawn_dataflow(
                build_id,
                dataflow_id,
                base_working_dir,
                nodes,
                dataflow_descriptor,
                spawn_nodes,
                uv,
                write_events_to,
            )
            .await;
        let (trigger_result, result_task) = match result {
            Ok(result_task) => (Ok(()), Some(result_task)),
            Err(err) => (Err(format!("{err:?}")), None),
        };

        // Spawn background task to report spawn completion to coordinator
        if let Some(result_task) = result_task {
            let state = self.state.clone();
            tokio::spawn(async move {
                let result = result_task.await;
                let spawn_result = result
                    .as_ref()
                    .map(|_| ())
                    .map_err(|err| format!("{err:?}"));

                if let Some(client) = state.coordinator_client() {
                    let ctx = tarpc::context::current();
                    if let Err(err) = client.spawn_result(ctx, dataflow_id, spawn_result).await {
                        tracing::error!(
                            ?err,
                            "failed to send spawn_result notification to coordinator"
                        );
                    }
                }
            });
        }

        trigger_result
    }

    async fn collect_and_send_metrics(&mut self) -> eyre::Result<()> {
        use dora_message::daemon_to_coordinator::NodeMetrics;
        use sysinfo::{Pid, ProcessRefreshKind, ProcessesToUpdate};

        if self.state.coordinator_client().is_none() {
            return Ok(());
        }

        // Reuse system instance for metrics collection
        let system = &mut self.metrics_system;

        // Metrics are collected every 2 seconds (metrics_interval)
        const METRICS_INTERVAL_SECS: f64 = 2.0;

        // Collect metrics for all running dataflows
        for entry in self.state.running.iter() {
            let (dataflow_id, dataflow) = (entry.key(), entry.value());
            let mut metrics = BTreeMap::new();

            // Collect all PIDs for this dataflow
            let pids: Vec<Pid> = dataflow
                .running_nodes
                .values()
                .filter_map(|node| {
                    node.pid
                        .as_ref()
                        .map(|pid| Pid::from_u32(pid.load(atomic::Ordering::Acquire)))
                })
                .collect();

            if !pids.is_empty() {
                // Refresh process metrics (cpu, memory, disk)
                let refresh_kind = ProcessRefreshKind::nothing()
                    .with_cpu()
                    .with_memory()
                    .with_disk_usage();
                system.refresh_processes_specifics(
                    ProcessesToUpdate::Some(&pids),
                    true,
                    refresh_kind,
                );

                // Collect metrics for each node
                for (node_id, running_node) in &dataflow.running_nodes {
                    if let Some(pid) = running_node.pid.as_ref() {
                        let pid = pid.load(atomic::Ordering::Acquire);
                        let sys_pid = Pid::from_u32(pid);
                        if let Some(process) = system.process(sys_pid) {
                            let disk_usage = process.disk_usage();
                            // Divide by metrics_interval to get per-second averages
                            metrics.insert(
                                node_id.clone(),
                                NodeMetrics {
                                    pid,
                                    cpu_usage: process.cpu_usage(),
                                    memory_bytes: process.memory(),
                                    disk_read_bytes: Some(
                                        (disk_usage.read_bytes as f64 / METRICS_INTERVAL_SECS)
                                            as u64,
                                    ),
                                    disk_write_bytes: Some(
                                        (disk_usage.written_bytes as f64 / METRICS_INTERVAL_SECS)
                                            as u64,
                                    ),
                                },
                            );
                        }
                    }
                }
            }

            // Send metrics to coordinator if we have any (fire-and-forget).
            if !metrics.is_empty() {
                if let Some(client) = self.state.coordinator_client() {
                    let client = client.clone();
                    let dataflow_id = *dataflow_id;
                    tokio::spawn(async move {
                        let _ = client
                            .node_metrics(tarpc::context::current(), dataflow_id, metrics)
                            .await;
                    });
                }
            }
        }

        Ok(())
    }

    async fn handle_inter_daemon_event(&mut self, event: InterDaemonEvent) -> eyre::Result<()> {
        match event {
            InterDaemonEvent::Output {
                dataflow_id,
                node_id,
                output_id,
                metadata,
                data,
            } => {
                let inner = async {
                    let mut dataflow =
                        self.state.running.get_mut(&dataflow_id).wrap_err_with(|| {
                            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                        })?;
                    send_output_to_local_receivers(
                        node_id.clone(),
                        output_id.clone(),
                        &mut *dataflow,
                        &metadata,
                        data.map(DataMessage::Vec),
                        &self.state.clock,
                    )
                    .await?;
                    Result::<_, eyre::Report>::Ok(())
                };
                if let Err(err) = inner
                    .await
                    .wrap_err("failed to forward remote output to local receivers")
                {
                    let mut logger = self.logger.for_dataflow(dataflow_id).for_node(node_id);
                    logger
                        .log(LogLevel::Warn, Some("daemon".into()), format!("{err:?}"))
                        .await;
                }
                Ok(())
            }
            InterDaemonEvent::OutputClosed {
                dataflow_id,
                node_id,
                output_id,
            } => {
                let output_id = OutputId(node_id.clone(), output_id);
                let mut logger = self
                    .logger
                    .for_dataflow(dataflow_id)
                    .for_node(node_id.clone());
                logger
                    .log(
                        LogLevel::Debug,
                        Some("daemon".into()),
                        format!("received OutputClosed event for output {output_id:?}"),
                    )
                    .await;

                let result = match self.state.running.get_mut(&dataflow_id) {
                    Some(mut dataflow) => {
                        if let Some(inputs) = dataflow.mappings.get(&output_id).cloned() {
                            for (receiver_id, input_id) in &inputs {
                                close_input(
                                    &mut *dataflow,
                                    receiver_id,
                                    input_id,
                                    &self.state.clock,
                                );
                            }
                        }
                        Ok(())
                    }
                    None => Err(eyre::eyre!(
                        "send out failed: no running dataflow with ID `{dataflow_id}`"
                    )),
                };
                if let Err(err) =
                    result.wrap_err("failed to handle InputsClosed event sent by coordinator")
                {
                    logger
                        .log(LogLevel::Warn, Some("daemon".into()), format!("{err:?}"))
                        .await;
                }
                Ok(())
            }
            InterDaemonEvent::NodeFailed {
                dataflow_id,
                source_node_id,
                error,
            } => {
                tracing::debug!(
                    "received NodeFailed event from {source_node_id} for dataflow {dataflow_id}"
                );

                let result = match self.state.running.get_mut(&dataflow_id) {
                    Some(mut dataflow) => {
                        // Deduplicate: the sending daemon publishes one NodeFailed per output topic,
                        // so we may receive the same event multiple times. Only notify local receivers
                        // once per source node.
                        if dataflow.handled_node_failed.insert(source_node_id.clone()) {
                            let (_outputs, _remote_receivers) =
                                Self::find_and_notify_local_receivers(
                                    &mut dataflow,
                                    &source_node_id,
                                    &error,
                                    &self.state.clock,
                                );
                        }
                        Ok(())
                    }
                    None => Err(eyre::eyre!(
                        "failed to handle NodeFailed: no running dataflow with ID `{dataflow_id}`"
                    )),
                };
                if let Err(err) =
                    result.wrap_err("failed to handle NodeFailed event sent by remote daemon")
                {
                    tracing::warn!("Failed to handle NodeFailed event: {err:?}");
                }
                Ok(())
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    async fn spawn_dataflow(
        &mut self,
        build_id: Option<BuildId>,
        dataflow_id: DataflowId,
        base_working_dir: PathBuf,
        nodes: BTreeMap<NodeId, ResolvedNode>,
        dataflow_descriptor: Descriptor,
        spawn_nodes: BTreeSet<NodeId>,
        uv: bool,
        write_events_to: Option<PathBuf>,
    ) -> eyre::Result<impl Future<Output = eyre::Result<()>> + use<>> {
        let mut logger = self
            .logger
            .for_dataflow(dataflow_id)
            .try_clone()
            .await
            .context("failed to clone logger")?;
        let dataflow = RunningDataflow::new(
            dataflow_id,
            self.state.daemon_id().clone(),
            dataflow_descriptor.clone(),
            self.state.events_tx.clone(),
            self.state.clock.clone(),
        );
        let mut dataflow = match self.state.running.entry(dataflow_id) {
            dashmap::Entry::Vacant(entry) => {
                self.state
                    .working_dir
                    .insert(dataflow_id, base_working_dir.clone());
                entry.insert(dataflow)
            }
            dashmap::Entry::Occupied(_) => {
                bail!("there is already a running dataflow with ID `{dataflow_id}`")
            }
        };

        let mut stopped = Vec::new();

        let build_info = build_id.and_then(|build_id| self.state.builds.get(&build_id));
        let node_with_git_source = nodes.values().find(|n| n.has_git_source());
        if let Some(git_node) = node_with_git_source {
            if build_info.is_none() {
                eyre::bail!(
                    "node {} has git source, but no `dora build` was run yet\n\n\
                    nodes with a `git` field must be built using `dora build` before starting the \
                    dataflow",
                    git_node.id
                )
            }
        }
        let node_working_dirs = build_info
            .map(|info| info.node_working_dirs.clone())
            .unwrap_or_default();

        // calculate info about mappings
        for node in nodes.values() {
            let local = spawn_nodes.contains(&node.id);

            let inputs = node_inputs(node);
            for (input_id, input) in inputs {
                if local {
                    dataflow
                        .open_inputs
                        .entry(node.id.clone())
                        .or_default()
                        .insert(input_id.clone());
                    match input.mapping {
                        InputMapping::User(mapping) => {
                            dataflow
                                .mappings
                                .entry(OutputId(mapping.source, mapping.output))
                                .or_default()
                                .insert((node.id.clone(), input_id));
                        }
                        InputMapping::Timer { interval } => {
                            dataflow
                                .timers
                                .entry(interval)
                                .or_default()
                                .insert((node.id.clone(), input_id));
                        }
                    }
                } else if let InputMapping::User(mapping) = input.mapping {
                    dataflow
                        .open_external_mappings
                        .insert(OutputId(mapping.source, mapping.output));
                }
            }
        }

        let spawner = Spawner {
            dataflow_id,
            daemon_tx: self.state.events_tx.clone(),
            dataflow_descriptor,
            clock: self.state.clock.clone(),
            uv,
        };

        let mut tasks = Vec::new();

        // spawn nodes and set up subscriptions
        for node in nodes.into_values() {
            let mut logger = logger.reborrow().for_node(node.id.clone());
            let local = spawn_nodes.contains(&node.id);
            if local {
                let dynamic_node = node.kind.dynamic();
                if dynamic_node {
                    dataflow.dynamic_nodes.insert(node.id.clone());
                } else {
                    dataflow.pending_nodes.insert(node.id.clone());
                }

                let node_id = node.id.clone();
                let node_stderr_most_recent = dataflow
                    .node_stderr_most_recent
                    .entry(node.id.clone())
                    .or_insert_with(|| Arc::new(ArrayQueue::new(STDERR_LOG_LINES_MAX)))
                    .clone();

                let configured_node_working_dir = node_working_dirs.get(&node_id).cloned();
                if configured_node_working_dir.is_none() && node.has_git_source() {
                    eyre::bail!(
                        "node {} has git source, but no git clone directory was found for it\n\n\
                        try running `dora build` again",
                        node.id
                    )
                }
                let node_working_dir = configured_node_working_dir
                    .or_else(|| {
                        node.deploy
                            .as_ref()
                            .and_then(|d| d.working_dir.as_ref().map(|d| base_working_dir.join(d)))
                    })
                    .unwrap_or(base_working_dir.clone())
                    .clone();
                let node_write_events_to = write_events_to
                    .as_ref()
                    .map(|p| p.join(format!("inputs-{}.json", node.id)));
                match spawner
                    .clone()
                    .spawn_node(
                        node,
                        node_working_dir,
                        node_stderr_most_recent,
                        node_write_events_to,
                        &mut logger,
                    )
                    .await
                    .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))
                {
                    Ok(result) => {
                        tasks.push(NodeBuildTask {
                            node_id,
                            task: result,
                            dynamic_node,
                        });
                    }
                    Err(err) => {
                        logger
                            .log(LogLevel::Error, Some("daemon".into()), format!("{err:?}"))
                            .await;
                        self.state
                            .dataflow_node_results
                            .entry(dataflow_id)
                            .or_default()
                            .insert(
                                node_id.clone(),
                                Err(NodeError {
                                    timestamp: self.state.clock.new_timestamp(),
                                    cause: NodeErrorCause::FailedToSpawn(format!("{err:?}")),
                                    exit_status: NodeExitStatus::Unknown,
                                }),
                            );
                        stopped.push((node_id.clone(), dynamic_node));
                    }
                }
            } else {
                // wait until node is ready before starting
                dataflow.pending_nodes.set_external_nodes(true);

                // subscribe to all node outputs that are mapped to some local inputs
                for output_id in dataflow.mappings.keys().filter(|o| o.0 == node.id) {
                    let tx = self
                        .state
                        .remote_daemon_events_tx
                        .clone()
                        .wrap_err("no remote_daemon_events_tx channel")?;
                    let mut finished_rx = dataflow.finished_tx.subscribe();
                    let subscribe_topic =
                        zenoh_output_publish_topic(dataflow.id, &output_id.0, &output_id.1);
                    tracing::debug!("declaring subscriber on {subscribe_topic}");
                    let zenoh = self
                        .state
                        .zenoh_session
                        .as_ref()
                        .wrap_err("no zenoh session")?;
                    let subscriber = zenoh
                        .declare_subscriber(subscribe_topic)
                        .await
                        .map_err(|e| eyre!(e))
                        .wrap_err_with(|| format!("failed to subscribe to {output_id:?}"))?;
                    tokio::spawn(async move {
                        let mut finished = pin!(finished_rx.recv());
                        loop {
                            let finished_or_next =
                                futures::future::select(finished, subscriber.recv_async());
                            match finished_or_next.await {
                                future::Either::Left((finished, _)) => match finished {
                                    Err(broadcast::error::RecvError::Closed) => {
                                        tracing::debug!(
                                            "dataflow finished, breaking from zenoh subscribe task"
                                        );
                                        break;
                                    }
                                    other => {
                                        tracing::warn!(
                                            "unexpected return value of dataflow finished_rx channel: {other:?}"
                                        );
                                        break;
                                    }
                                },
                                future::Either::Right((sample, f)) => {
                                    finished = f;
                                    let event = sample.map_err(|e| eyre!(e)).and_then(|s| {
                                        Timestamped::deserialize_inter_daemon_event(
                                            &s.payload().to_bytes(),
                                        )
                                    });
                                    if tx.send_async(event).await.is_err() {
                                        // daemon finished
                                        break;
                                    }
                                }
                            }
                        }
                    });
                }
            }
        }
        drop(dataflow);
        for (node_id, dynamic) in stopped {
            self.handle_node_stop(dataflow_id, &node_id, dynamic)
                .await?;
        }

        let spawn_result = Self::spawn_prepared_nodes(
            dataflow_id,
            logger,
            tasks,
            self.state.events_tx.clone(),
            self.state.clock.clone(),
        );

        Ok(spawn_result)
    }

    async fn spawn_prepared_nodes(
        dataflow_id: Uuid,
        mut logger: DataflowLogger<'_>,
        tasks: Vec<NodeBuildTask<impl Future<Output = eyre::Result<spawn::PreparedNode>>>>,
        events_tx: mpsc::Sender<Timestamped<Event>>,
        clock: Arc<HLC>,
    ) -> eyre::Result<()> {
        let node_result = |node_id, dynamic_node, result| Timestamped {
            inner: Event::SpawnNodeResult {
                dataflow_id,
                node_id,
                dynamic_node,
                result,
            },
            timestamp: clock.new_timestamp(),
        };
        let mut failed_to_prepare = None;
        let mut prepared_nodes = Vec::new();
        for task in tasks {
            let NodeBuildTask {
                node_id,
                dynamic_node,
                task,
            } = task;
            match task.await {
                Ok(node) => prepared_nodes.push(node),
                Err(err) => {
                    if failed_to_prepare.is_none() {
                        failed_to_prepare = Some(node_id.clone());
                    }
                    let node_err: NodeError = NodeError {
                        timestamp: clock.new_timestamp(),
                        cause: NodeErrorCause::FailedToSpawn(format!(
                            "preparing for spawn failed: {err:?}"
                        )),
                        exit_status: NodeExitStatus::Unknown,
                    };
                    let send_result = events_tx
                        .send(node_result(node_id, dynamic_node, Err(node_err)))
                        .await;
                    if send_result.is_err() {
                        tracing::error!("failed to send SpawnNodeResult to main daemon task")
                    }
                }
            }
        }

        // once all nodes are prepared, do the actual spawning
        if let Some(failed_node) = failed_to_prepare {
            // don't spawn any nodes when an error occurred before
            for node in prepared_nodes {
                let err = NodeError {
                    timestamp: clock.new_timestamp(),
                    cause: NodeErrorCause::Cascading {
                        caused_by_node: failed_node.clone(),
                    },
                    exit_status: NodeExitStatus::Unknown,
                };
                let send_result = events_tx
                    .send(node_result(
                        node.node_id().clone(),
                        node.dynamic(),
                        Err(err),
                    ))
                    .await;
                if send_result.is_err() {
                    tracing::error!("failed to send SpawnNodeResult to main daemon task")
                }
            }
            Err(eyre!("failed to prepare node {failed_node}"))
        } else {
            let mut spawn_result = Ok(());

            logger
                .log(
                    LogLevel::Info,
                    None,
                    Some("dora daemon".into()),
                    "finished building nodes, spawning...",
                )
                .await;

            // spawn the nodes
            for node in prepared_nodes {
                let node_id = node.node_id().clone();
                let dynamic_node = node.dynamic();
                let logger = logger
                    .reborrow()
                    .for_node(node_id.clone())
                    .try_clone()
                    .await
                    .context("failed to clone NodeLogger")?;
                let result = node.spawn(logger).await;
                let node_spawn_result = match result {
                    Ok(node) => Ok(node),
                    Err(err) => {
                        let node_err = NodeError {
                            timestamp: clock.new_timestamp(),
                            cause: NodeErrorCause::FailedToSpawn(format!("spawn failed: {err:?}")),
                            exit_status: NodeExitStatus::Unknown,
                        };
                        if spawn_result.is_ok() {
                            spawn_result = Err(err.wrap_err(format!("failed to spawn {node_id}")));
                        }
                        Err(node_err)
                    }
                };
                let send_result = events_tx
                    .send(node_result(node_id, dynamic_node, node_spawn_result))
                    .await;
                if send_result.is_err() {
                    tracing::error!("failed to send SpawnNodeResult to main daemon task")
                }
            }
            spawn_result
        }
    }

    async fn handle_dynamic_node_event(
        &mut self,
        event: DynamicNodeEventWrapper,
    ) -> eyre::Result<()> {
        match event {
            DynamicNodeEventWrapper {
                event: DynamicNodeEvent::NodeConfig { node_id },
                reply_tx,
            } => {
                let number_node_id = self
                    .state
                    .running
                    .iter()
                    .filter(|entry| entry.value().running_nodes.contains_key(&node_id))
                    .count();

                let node_config = match number_node_id {
                    2.. => Err(format!(
                        "multiple dataflows contain dynamic node id {node_id}. \
                        Please only have one running dataflow with the specified \
                        node id if you want to use dynamic node",
                    )),
                    1 => self
                        .state
                        .running
                        .iter()
                        .filter(|entry| entry.value().running_nodes.contains_key(&node_id))
                        .map(|entry| -> Result<NodeConfig> {
                            let (id, dataflow) = (entry.key(), entry.value());
                            let node_config = dataflow
                                .running_nodes
                                .get(&node_id)
                                .with_context(|| {
                                    format!("no node with ID `{node_id}` within the given dataflow")
                                })?
                                .node_config
                                .clone();
                            if !node_config.dynamic {
                                bail!("node with ID `{node_id}` in {id} is not dynamic");
                            }
                            Ok(node_config)
                        })
                        .next()
                        .ok_or_else(|| eyre!("no node with ID `{node_id}`"))
                        .and_then(|r| r)
                        .map_err(|err| {
                            format!(
                                "failed to get dynamic node config within given dataflow: {err}"
                            )
                        }),
                    0 => Err(format!("no node with ID `{node_id}`")),
                };

                let reply = DaemonReply::NodeConfig {
                    result: node_config,
                };
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send node info reply from daemon to coordinator")
                });
                Ok(())
            }
        }
    }

    async fn handle_node_event(
        &mut self,
        event: DaemonNodeEvent,
        dataflow_id: DataflowId,
        node_id: NodeId,
    ) -> eyre::Result<()> {
        let might_restart = || {
            let dataflow = self.state.running.get(&dataflow_id)?;
            let node = dataflow.running_nodes.get(&node_id)?;
            Some(match node.restart_policy {
                RestartPolicy::Never => false,
                _ if node.restarts_disabled() => false,
                RestartPolicy::OnFailure | RestartPolicy::Always => true,
            })
        };
        match event {
            DaemonNodeEvent::Subscribe {
                event_sender,
                reply_sender,
            } => {
                let mut logger = self.logger.for_dataflow(dataflow_id);
                logger
                    .log(
                        LogLevel::Info,
                        Some(node_id.clone()),
                        Some("daemon".into()),
                        "node is ready",
                    )
                    .await;

                let dataflow = self.state.running.get_mut(&dataflow_id).ok_or_else(|| {
                    format!("subscribe failed: no running dataflow with ID `{dataflow_id}`")
                });

                match dataflow {
                    Err(err) => {
                        let _ = reply_sender.send(DaemonReply::Result(Err(err)));
                    }
                    Ok(mut dataflow) => {
                        Self::subscribe(
                            &mut *dataflow,
                            node_id.clone(),
                            event_sender,
                            &self.state.clock,
                        )
                        .await;

                        let df = &mut *dataflow;
                        let status = df
                            .pending_nodes
                            .handle_node_subscription(
                                node_id.clone(),
                                reply_sender,
                                &self.state.coordinator_client().cloned(),
                                &self.state.clock,
                                &mut df.cascading_error_causes,
                                &mut logger,
                            )
                            .await?;
                        match status {
                            DataflowStatus::AllNodesReady if !dataflow.dataflow_started => {
                                logger
                                    .log(
                                        LogLevel::Info,
                                        None,
                                        Some("daemon".into()),
                                        "all nodes are ready, starting dataflow",
                                    )
                                    .await;
                                dataflow
                                    .start(&self.state.events_tx, &self.state.clock)
                                    .await?;
                                dataflow.dataflow_started = true;
                            }
                            _ => {}
                        }
                    }
                }
            }
            DaemonNodeEvent::SubscribeDrop {
                event_sender,
                reply_sender,
            } => {
                let dataflow = self.state.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!("failed to subscribe: no running dataflow with ID `{dataflow_id}`")
                });
                let result = match dataflow {
                    Ok(mut dataflow) => {
                        dataflow.drop_channels.insert(node_id, event_sender);
                        Ok(())
                    }
                    Err(err) => Err(err.to_string()),
                };
                let _ = reply_sender.send(DaemonReply::Result(result));
            }
            DaemonNodeEvent::CloseOutputs {
                outputs,
                reply_sender,
            } => {
                let reply = if might_restart().unwrap_or(false) {
                    self.logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .log(
                            LogLevel::Debug,
                            Some("daemon".into()),
                            "skipping CloseOutputs because node might restart",
                        )
                        .await;
                    Ok(())
                } else {
                    // notify downstream nodes
                    let inner = async {
                        self.send_output_closed_events(dataflow_id, node_id, outputs)
                            .await
                    };

                    inner.await.map_err(|err| format!("{err:?}"))
                };
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
            DaemonNodeEvent::OutputsDone { reply_sender } => {
                let result = self
                    .handle_outputs_done(dataflow_id, &node_id, might_restart().unwrap_or(false))
                    .await;

                let _ = reply_sender.send(DaemonReply::Result(
                    result.map_err(|err| format!("{err:?}")),
                ));
            }
            DaemonNodeEvent::SendOut {
                output_id,
                metadata,
                data,
            } => self
                .send_out(dataflow_id, node_id, output_id, metadata, data)
                .await
                .context("failed to send out")?,
            DaemonNodeEvent::ReportDrop { tokens } => {
                let dataflow = self.state.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!(
                        "failed to get handle drop tokens: \
                        no running dataflow with ID `{dataflow_id}`"
                    )
                });

                match dataflow {
                    Ok(mut dataflow) => {
                        for token in tokens {
                            match dataflow.pending_drop_tokens.get_mut(&token) {
                                Some(info) => {
                                    if info.pending_nodes.remove(&node_id) {
                                        dataflow.check_drop_token(token, &self.state.clock).await?;
                                    } else {
                                        tracing::warn!(
                                            "node `{node_id}` is not pending for drop token `{token:?}`"
                                        );
                                    }
                                }
                                None => tracing::warn!("unknown drop token `{token:?}`"),
                            }
                        }
                    }
                    Err(err) => tracing::warn!("{err:?}"),
                }
            }
            DaemonNodeEvent::EventStreamDropped { reply_sender } => {
                let reply = match self.state.running.get_mut(&dataflow_id) {
                    Some(mut dataflow) => {
                        dataflow.subscribe_channels.remove(&node_id);
                        Ok(())
                    }
                    None => Err(format!("no running dataflow with ID `{dataflow_id}`")),
                };
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
        }
        Ok(())
    }

    async fn send_out(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        metadata: dora_message::metadata::Metadata,
        data: Option<DataMessage>,
    ) -> Result<(), eyre::ErrReport> {
        let mut dataflow = self.state.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        let data_bytes = send_output_to_local_receivers(
            node_id.clone(),
            output_id.clone(),
            &mut *dataflow,
            &metadata,
            data,
            &self.state.clock,
        )
        .await?;

        let output_id = OutputId(node_id, output_id);
        let remote_receivers = dataflow.open_external_mappings.contains(&output_id)
            || dataflow.publish_all_messages_to_zenoh;
        drop(dataflow);
        if remote_receivers {
            let event = InterDaemonEvent::Output {
                dataflow_id,
                node_id: output_id.0.clone(),
                output_id: output_id.1.clone(),
                metadata,
                data: data_bytes,
            };
            self.send_to_remote_receivers(dataflow_id, &output_id, event)
                .await?;
        }

        Ok(())
    }

    /// Find all receivers affected by a node failure and send NodeFailed events to local ones.
    ///
    /// Returns the list of outputs and a set of remote receiver node IDs (nodes not on this daemon).
    fn find_and_notify_local_receivers(
        dataflow: &mut RunningDataflow,
        source_node_id: &NodeId,
        error_message: &str,
        clock: &HLC,
    ) -> (Vec<DataId>, BTreeSet<NodeId>) {
        // Get all outputs of the failed node
        let outputs: Vec<DataId> = dataflow
            .mappings
            .keys()
            .filter(|m| &m.0 == source_node_id)
            .map(|m| m.1.clone())
            .collect();

        // Group affected inputs by receiver node
        let mut affected_by_receiver: BTreeMap<NodeId, Vec<DataId>> = BTreeMap::new();

        for output_id in &outputs {
            let output_key = OutputId(source_node_id.clone(), output_id.clone());
            if let Some(receivers) = dataflow.mappings.get(&output_key) {
                for (receiver_node_id, input_id) in receivers {
                    affected_by_receiver
                        .entry(receiver_node_id.clone())
                        .or_default()
                        .push(input_id.clone());
                }
            }
        }

        // Identify remote receivers (nodes not on this daemon)
        let remote_receivers: BTreeSet<NodeId> = affected_by_receiver
            .keys()
            .filter(|receiver_node_id| !dataflow.subscribe_channels.contains_key(receiver_node_id))
            .cloned()
            .collect();

        // Send NodeFailed event to each local receiver
        for (receiver_node_id, affected_input_ids) in affected_by_receiver {
            if let Some(channel) = dataflow.subscribe_channels.get(&receiver_node_id) {
                // Local receiver - send directly
                let event = NodeEvent::NodeFailed {
                    affected_input_ids,
                    error: error_message.to_string(),
                    source_node_id: source_node_id.clone(),
                };
                let _ = send_with_timestamp(channel, event, clock);
            }
        }

        (outputs, remote_receivers)
    }

    /// Send error events to downstream nodes when a node fails.
    ///
    /// This is called automatically when a node exits with a non-zero exit code.
    /// It sends `NodeFailed` events to all downstream nodes that consume outputs from the failed node.
    /// For nodes on the same daemon, events are sent directly. For nodes on other daemons,
    /// events are routed through zenoh by sending to all outputs (to ensure all subscribing daemons receive it).
    async fn send_node_failed_events(
        &mut self,
        dataflow_id: Uuid,
        source_node_id: NodeId,
        error_message: String,
    ) -> Result<(), eyre::ErrReport> {
        let mut dataflow = self.state.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("propagate error failed: no running dataflow with ID `{dataflow_id}`")
        })?;

        tracing::warn!(
            "Node {}/{} failed: {}. Propagating error to downstream nodes.",
            dataflow_id,
            source_node_id,
            error_message
        );

        let (outputs, remote_receivers) = Self::find_and_notify_local_receivers(
            &mut dataflow,
            &source_node_id,
            &error_message,
            &self.state.clock,
        );

        // Drop the DashMap lock before doing async I/O via send_to_remote_receivers.
        drop(dataflow);

        // If there are remote receivers, send InterDaemonEvent to all outputs
        // This ensures all daemons that subscribe to any of the node's outputs will receive the error
        if !remote_receivers.is_empty() {
            // If node has no outputs, there are no remote receivers to notify
            if outputs.is_empty() {
                return Ok(());
            }

            // Send to all outputs to ensure all subscribing daemons receive the error
            // (daemons subscribe to specific output topics, so we need to send to each one)
            for output_id in &outputs {
                let output_key = OutputId(source_node_id.clone(), output_id.clone());
                let event = InterDaemonEvent::NodeFailed {
                    dataflow_id,
                    source_node_id: source_node_id.clone(),
                    error: error_message.clone(),
                };
                if let Err(err) = self
                    .send_to_remote_receivers(dataflow_id, &output_key, event)
                    .await
                {
                    tracing::warn!(
                        "Failed to send error event to remote receivers via output {}: {err:?}",
                        output_id
                    );
                }
            }
        }

        Ok(())
    }

    async fn send_to_remote_receivers(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        event: InterDaemonEvent,
    ) -> Result<(), eyre::Error> {
        // Ensure the zenoh publisher exists. Creating a publisher involves
        // network I/O, so we do it outside the DashMap lock.
        {
            let has_publisher = self
                .state
                .running
                .get(&dataflow_id)
                .map(|d| d.publishers.contains_key(output_id))
                .unwrap_or(false);
            if !has_publisher {
                let publish_topic = {
                    let dataflow = self.state.running.get(&dataflow_id).wrap_err_with(|| {
                        format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                    })?;
                    zenoh_output_publish_topic(dataflow.id, &output_id.0, &output_id.1)
                };
                // DashMap lock dropped — safe to do async I/O.
                tracing::debug!("declaring publisher on {publish_topic}");
                let zenoh = self
                    .state
                    .zenoh_session
                    .as_ref()
                    .wrap_err("no zenoh session")?;
                let publisher = zenoh
                    .declare_publisher(publish_topic)
                    .await
                    .map_err(|e| eyre!(e))
                    .context("failed to create zenoh publisher")?;
                // Re-acquire lock briefly to insert the publisher.
                let mut dataflow =
                    self.state.running.get_mut(&dataflow_id).wrap_err_with(|| {
                        format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                    })?;
                dataflow.publishers.insert(output_id.clone(), publisher);
            }
        }

        // Now publish via zenoh. The DashMap lock is held during `put()`,
        // which is typically fast (no network round-trip).
        let dataflow = self.state.running.get(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        let publisher = dataflow
            .publishers
            .get(output_id)
            .wrap_err("publisher disappeared")?;
        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.state.clock.new_timestamp(),
        }
        .serialize();
        publisher
            .put(serialized_event)
            .await
            .map_err(|e| eyre!(e))
            .context("zenoh put failed")?;
        Ok(())
    }

    async fn send_output_closed_events(
        &mut self,
        dataflow_id: DataflowId,
        node_id: NodeId,
        outputs: Vec<DataId>,
    ) -> eyre::Result<()> {
        let (local_node_inputs, closed) = {
            let mut dataflow = self
                .state
                .running
                .get_mut(&dataflow_id)
                .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
            let local_node_inputs: BTreeSet<_> = dataflow
                .mappings
                .iter()
                .filter(|(k, _)| k.0 == node_id && outputs.contains(&k.1))
                .flat_map(|(_, v)| v)
                .cloned()
                .collect();
            for (receiver_id, input_id) in &local_node_inputs {
                close_input(&mut *dataflow, receiver_id, input_id, &self.state.clock);
            }

            let mut closed = Vec::new();
            for output_id in &dataflow.open_external_mappings {
                if output_id.0 == node_id && outputs.contains(&output_id.1) {
                    closed.push(output_id.clone());
                }
            }
            (local_node_inputs, closed)
        };

        for output_id in closed {
            let event = InterDaemonEvent::OutputClosed {
                dataflow_id,
                node_id: output_id.0.clone(),
                output_id: output_id.1.clone(),
            };
            self.send_to_remote_receivers(dataflow_id, &output_id, event)
                .await?;
        }

        Ok(())
    }

    async fn subscribe(
        dataflow: &mut RunningDataflow,
        node_id: NodeId,
        event_sender: UnboundedSender<Timestamped<NodeEvent>>,
        clock: &HLC,
    ) {
        // some inputs might have been closed already -> report those events
        let closed_inputs = dataflow
            .mappings
            .values()
            .flatten()
            .filter(|(node, _)| node == &node_id)
            .map(|(_, input)| input)
            .filter(|input| {
                dataflow
                    .open_inputs
                    .get(&node_id)
                    .map(|open_inputs| !open_inputs.contains(*input))
                    .unwrap_or(true)
            });
        for input_id in closed_inputs {
            let _ = send_with_timestamp(
                &event_sender,
                NodeEvent::InputClosed {
                    id: input_id.clone(),
                },
                clock,
            );
        }
        if dataflow.open_inputs(&node_id).is_empty() {
            if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                node.disable_restart();
            }
            if let Some(node) = dataflow.descriptor.nodes.iter().find(|n| n.id == node_id) {
                if node.inputs.is_empty() {
                    // do not send AllInputsClosed for source nodes
                } else {
                    let _ = send_with_timestamp(&event_sender, NodeEvent::AllInputsClosed, clock);
                }
            }
        }

        // if a stop event was already sent for the dataflow, send it to
        // the newly connected node too
        if dataflow.stop_sent {
            if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                node.disable_restart();
            }
            let _ = send_with_timestamp(&event_sender, NodeEvent::Stop, clock);
        }

        dataflow.subscribe_channels.insert(node_id, event_sender);
    }

    #[tracing::instrument(skip(self), level = "trace")]
    async fn handle_outputs_done(
        &mut self,
        dataflow_id: DataflowId,
        node_id: &NodeId,
        might_restart: bool,
    ) -> eyre::Result<()> {
        let outputs = {
            let dataflow = self
                .state
                .running
                .get(&dataflow_id)
                .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
            dataflow
                .mappings
                .keys()
                .filter(|m| &m.0 == node_id)
                .map(|m| &m.1)
                .cloned()
                .collect()
        };

        if might_restart {
            self.logger
                .for_dataflow(dataflow_id)
                .for_node(node_id.clone())
                .log(
                    LogLevel::Debug,
                    Some("daemon".into()),
                    "keeping outputs open because node might restart",
                )
                .await;
        } else {
            self.send_output_closed_events(dataflow_id, node_id.clone(), outputs)
                .await?;
        }

        if let Some(mut dataflow) = self.state.running.get_mut(&dataflow_id) {
            dataflow.drop_channels.remove(node_id);
        }
        Ok(())
    }

    async fn handle_node_stop(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
        dynamic_node: bool,
    ) -> eyre::Result<()> {
        let result = self
            .handle_node_stop_inner(dataflow_id, node_id, dynamic_node)
            .await;

        // Always send NodeStopped event, even if handle_node_stop_inner failed
        // This is critical for daemon shutdown - the event must be sent
        if let Err(err) = self
            .state
            .events_tx
            .send(Timestamped {
                inner: Event::NodeStopped {
                    dataflow_id,
                    node_id: node_id.clone(),
                },
                timestamp: self.state.clock.new_timestamp(),
            })
            .await
        {
            tracing::error!(
                "Failed to send NodeStopped event for {}/{}: {err:?}. Daemon may not exit properly.",
                dataflow_id,
                node_id
            );
        }

        result
    }

    async fn handle_node_stop_inner(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
        dynamic_node: bool,
    ) -> eyre::Result<()> {
        let mut logger = self.logger.for_dataflow(dataflow_id);
        let exited_before_subscribe = {
            let mut dataflow = match self.state.running.get_mut(&dataflow_id) {
                Some(dataflow) => dataflow,
                None if dynamic_node => {
                    // The dataflow might be done already as we don't wait for dynamic nodes. In this
                    // case, we don't need to do anything to handle the node stop.
                    tracing::debug!(
                        "dynamic node {dataflow_id}/{node_id} stopped after dataflow was done"
                    );
                    return Ok(());
                }
                None => eyre::bail!(
                    "failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`"
                ),
            };

            let df = &mut *dataflow;
            df.pending_nodes
                .handle_node_stop_sync(node_id, &mut df.cascading_error_causes)
        };
        // DashMap guard is dropped — safe to do async I/O.
        if exited_before_subscribe {
            logger
                .log(
                    LogLevel::Warn,
                    Some(node_id.clone()),
                    Some("daemon::pending".into()),
                    "node exited before initializing dora connection",
                )
                .await;
        }
        // DashMap guard is dropped — safe to do async I/O (RPC to coordinator).
        // Check if all local nodes are ready and report to coordinator if needed.
        // We must not hold the DashMap guard during the RPC call.
        let needs_report = self
            .state
            .running
            .get_mut(&dataflow_id)
            .map(|mut dataflow| {
                let df = &mut *dataflow;
                df.pending_nodes
                    .check_and_answer_subscribers(&mut df.cascading_error_causes)
            })
            .unwrap_or(false);
        if needs_report {
            // DashMap guard is dropped — safe to do RPC.
            if let Some(client) = self.state.coordinator_client() {
                if let Some(dataflow) = self.state.running.get(&dataflow_id) {
                    let exited = dataflow.pending_nodes.exited_before_subscribe().to_vec();
                    drop(dataflow);
                    // DashMap guard is dropped — safe to do async I/O.
                    logger
                        .log(
                            LogLevel::Info,
                            None,
                            Some("daemon".into()),
                            format!(
                                "all local nodes are ready (exit before subscribe: {exited:?}), \
                                 waiting for remote nodes",
                            ),
                        )
                        .await;
                    let client = client.clone();
                    let events_tx = self.state.events_tx.clone();
                    let clock = self.state.clock.clone();
                    tokio::spawn(async move {
                        if let Err(err) = client
                            .all_nodes_ready(tarpc::context::current(), dataflow_id, exited)
                            .await
                        {
                            let _ = events_tx
                                .send(Timestamped {
                                    inner: Event::DaemonError(
                                        eyre::eyre!(err)
                                            .wrap_err("failed to send AllNodesReady RPC"),
                                    ),
                                    timestamp: clock.new_timestamp(),
                                })
                                .await;
                        }
                    });
                }
            }
        }

        // node only reaches here if it will not be restarted
        let might_restart = false;

        self.handle_outputs_done(dataflow_id, node_id, might_restart)
            .await?;

        let should_finish = {
            let mut dataflow = self.state.running.get_mut(&dataflow_id).wrap_err_with(|| {
                format!(
                    "failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`"
                )
            })?;
            dataflow.running_nodes.remove(node_id);
            // Check if all remaining nodes are dynamic (won't send SpawnedNodeResult)
            !dataflow.pending_nodes.local_nodes_pending()
                && dataflow
                    .running_nodes
                    .iter()
                    .all(|(_id, n)| n.node_config.dynamic)
        };

        if should_finish {
            self.finish_dataflow(dataflow_id).await?;
        }

        Ok(())
    }

    /// Mark a dataflow as finished and perform cleanup.
    /// This should be called when:
    /// - `stop_all()` returns `FinishDataflowWhen::Now`, or
    /// - All non-dynamic nodes have sent `SpawnedNodeResult` events
    async fn finish_dataflow(&mut self, dataflow_id: Uuid) -> eyre::Result<()> {
        let mut logger = self.logger.for_dataflow(dataflow_id);

        // Dynamic nodes don't send SpawnedNodeResult events, so there may be no entry
        // in dataflow_node_results. An empty map means all dynamic nodes handled stop successfully.
        let result = DataflowDaemonResult {
            timestamp: self.state.clock.new_timestamp(),
            node_results: self
                .state
                .dataflow_node_results
                .get(&dataflow_id)
                .map(|entry| entry.value().clone())
                .unwrap_or_default(),
        };

        {
            let mut git_manager = self.state.git_manager.lock().await;
            git_manager
                .clones_in_use
                .values_mut()
                .for_each(|dataflows| {
                    dataflows.remove(&dataflow_id);
                });
        }

        logger
            .log(
                LogLevel::Info,
                None,
                Some("daemon".into()),
                format!("dataflow finished on machine `{}`", self.state.daemon_id()),
            )
            .await;

        if let Some(client) = self.state.coordinator_client() {
            let client = client.clone();
            tokio::spawn(async move {
                if let Err(err) = client
                    .all_nodes_finished(tarpc::context::current(), dataflow_id, result)
                    .await
                {
                    tracing::error!(
                        ?err,
                        "failed to send all_nodes_finished notification to coordinator"
                    );
                }
            });
        }
        self.state.running.remove(&dataflow_id);

        Ok(())
    }

    async fn handle_dora_event(&mut self, event: DoraEvent) -> eyre::Result<()> {
        match event {
            DoraEvent::Timer {
                dataflow_id,
                interval,
                metadata,
            } => {
                let Some(mut dataflow) = self.state.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Timer event for unknown dataflow `{dataflow_id}`");
                    return Ok(());
                };

                let Some(subscribers) = dataflow.timers.get(&interval).cloned() else {
                    return Ok(());
                };

                let mut closed = Vec::new();
                for (receiver_id, input_id) in &subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        continue;
                    };

                    let send_result = send_with_timestamp(
                        channel,
                        NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: metadata.clone(),
                            data: None,
                        },
                        &self.state.clock,
                    );
                    match send_result {
                        Ok(()) => {}
                        Err(_) => {
                            closed.push(receiver_id.clone());
                        }
                    }
                }
                for id in &closed {
                    dataflow.subscribe_channels.remove(id);
                }
            }
            DoraEvent::Logs {
                dataflow_id,
                output_id,
                message,
                metadata,
            } => {
                let Some(mut dataflow) = self.state.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Logs event for unknown dataflow `{dataflow_id}`");
                    return Ok(());
                };

                let Some(subscribers) = dataflow.mappings.get(&output_id).cloned() else {
                    tracing::warn!(
                        "No subscribers found for {:?} in {:?}",
                        output_id,
                        dataflow.mappings
                    );
                    return Ok(());
                };

                let mut closed = Vec::new();
                for (receiver_id, input_id) in &subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        tracing::warn!("No subscriber channel found for {:?}", output_id);
                        continue;
                    };

                    let send_result = send_with_timestamp(
                        channel,
                        NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: metadata.clone(),
                            data: Some(message.clone()),
                        },
                        &self.state.clock,
                    );
                    match send_result {
                        Ok(()) => {}
                        Err(_) => {
                            closed.push(receiver_id.clone());
                        }
                    }
                }
                for id in &closed {
                    dataflow.subscribe_channels.remove(id);
                }
            }
            DoraEvent::SpawnedNodeResult {
                dataflow_id,
                node_id,
                dynamic_node,
                exit_status,
                restart,
            } => {
                let mut logger = self
                    .logger
                    .for_dataflow(dataflow_id)
                    .for_node(node_id.clone());
                logger
                    .log(
                        LogLevel::Debug,
                        Some("daemon".into()),
                        format!("handling node stop with exit status {exit_status:?} (restart: {restart})"),
                    )
                    .await;

                let stop_sent = self
                    .state
                    .running
                    .get(&dataflow_id)
                    .map_or(false, |d| d.stop_sent);

                let node_result = match exit_status {
                    NodeExitStatus::Success => Ok(()),
                    // Exit code 143 (128 + SIGTERM) is the conventional exit code used by
                    // processes that catch SIGTERM and exit explicitly (e.g. Python nodes).
                    // When a stop was already requested for the dataflow, treat this as success.
                    NodeExitStatus::ExitCode(143) if stop_sent => Ok(()),
                    exit_status => {
                        // Extract all needed info from the dataflow Ref in one shot
                        let (caused_by_node, grace_duration_kill, stderr_lines) =
                            if let Some(dataflow) = self.state.running.get(&dataflow_id) {
                                let caused_by = dataflow
                                    .cascading_error_causes
                                    .error_caused_by(&node_id)
                                    .cloned();

                                let grace_kill = dataflow.grace_duration_kills.contains(&node_id);
                                let stderr =
                                    dataflow.node_stderr_most_recent.get(&node_id).map(|queue| {
                                        let mut lines = Vec::new();
                                        if queue.is_full() {
                                            lines.push("[...]".into());
                                        }
                                        while let Some(line) = queue.pop() {
                                            lines.push(line);
                                        }
                                        lines
                                    });
                                (caused_by, grace_kill, stderr)
                            } else {
                                (None, false, None)
                            };

                        let cause = match caused_by_node {
                            Some(caused_by_node) => {
                                logger
                                .log(
                                    LogLevel::Info,
                                    Some("daemon".into()),
                                    format!("marking `{node_id}` as cascading error caused by `{caused_by_node}`")
                                )
                                .await;

                                NodeErrorCause::Cascading { caused_by_node }
                            }
                            None if grace_duration_kill => NodeErrorCause::GraceDuration,
                            None => {
                                let cause = stderr_lines
                                    .map(extract_err_from_stderr)
                                    .unwrap_or_default();

                                NodeErrorCause::Other { stderr: cause }
                            }
                        };
                        Err(NodeError {
                            timestamp: self.state.clock.new_timestamp(),
                            cause,
                            exit_status,
                        })
                    }
                };

                logger
                    .log(
                        if node_result.is_ok() {
                            LogLevel::Info
                        } else {
                            LogLevel::Error
                        },
                        Some("daemon".into()),
                        match &node_result {
                            Ok(()) => format!("{node_id} finished successfully"),
                            Err(err) => format!("{err}"),
                        },
                    )
                    .await;

                drop(logger);

                // Propagate error to downstream nodes only for genuine user-code failures,
                // even if the node will be restarted (downstream should know about failures).
                // Cascading errors (caused by a previously failed node), grace-duration
                // kills (dora's own timeout), and spawn failures are excluded:
                // - Cascading: downstream nodes already received a NodeFailed for the root cause.
                // - GraceDuration: this is dora operational behaviour, not a user-code error.
                // - FailedToSpawn: the node never ran, so its outputs were never open.
                if let Err(node_error) = &node_result {
                    if matches!(node_error.cause, NodeErrorCause::Other { .. }) {
                        let error_message = format!("{node_error}");
                        if let Err(err) = self
                            .send_node_failed_events(dataflow_id, node_id.clone(), error_message)
                            .await
                        {
                            tracing::warn!(
                                "Failed to propagate error for node {}/{}: {err:?}",
                                dataflow_id,
                                node_id
                            );
                        }
                    }
                }

                if restart {
                    self.logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .log(
                            LogLevel::Info,
                            Some("daemon".into()),
                            "node will be restarted",
                        )
                        .await;
                } else {
                    self.state
                        .dataflow_node_results
                        .entry(dataflow_id)
                        .or_default()
                        .insert(node_id.clone(), node_result);

                    self.handle_node_stop(dataflow_id, &node_id, dynamic_node)
                        .await?;
                }
            }
        }
        Ok(())
    }

    /// Static version of `base_working_dir`, usable without a `Daemon` instance.
    pub(crate) fn base_working_dir_static(
        local_working_dir: Option<PathBuf>,
        session_id: SessionId,
    ) -> eyre::Result<PathBuf> {
        match local_working_dir {
            Some(working_dir) => {
                // check that working directory exists
                if working_dir.exists() {
                    Ok(working_dir)
                } else {
                    bail!(
                        "working directory does not exist: {}",
                        working_dir.display(),
                    )
                }
            }
            None => {
                // use subfolder of daemon working dir
                let daemon_working_dir =
                    current_dir().context("failed to get daemon working dir")?;
                Ok(daemon_working_dir
                    .join("_work")
                    .join(session_id.uuid().to_string()))
            }
        }
    }

    /// Static version of `build_dataflow`, usable from the RPC server without
    /// a `Daemon` instance. Uses `DaemonState` directly.
    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn build_dataflow_static(
        state: &state::DaemonState,
        build_id: BuildId,
        session_id: SessionId,
        base_working_dir: PathBuf,
        git_sources: BTreeMap<NodeId, GitSource>,
        prev_git_sources: BTreeMap<NodeId, GitSource>,
        dataflow_descriptor: Descriptor,
        local_nodes: BTreeSet<NodeId>,
        uv: bool,
    ) -> eyre::Result<impl Future<Output = eyre::Result<BuildInfo>> + use<>> {
        let builder = build::Builder {
            session_id,
            base_working_dir,
            uv,
        };
        {
            let mut git_manager = state.git_manager.lock().await;
            git_manager.clear_planned_builds(session_id);
        }

        let nodes = dataflow_descriptor.resolve_aliases_and_set_defaults()?;

        let mut tasks = Vec::new();

        // build nodes
        for node in nodes.into_values().filter(|n| local_nodes.contains(&n.id)) {
            let node_id = node.id.clone();
            let git_source = git_sources.get(&node_id).cloned();
            let prev_git_source = prev_git_sources.get(&node_id).cloned();
            let prev_git = prev_git_source.map(|prev_source| PrevGitSource {
                still_needed_for_this_build: git_sources.values().any(|s| s == &prev_source),
                git_source: prev_source,
            });

            let mut builder = builder.clone();
            if let Some(node_working_dir) =
                node.deploy.as_ref().and_then(|d| d.working_dir.as_deref())
            {
                builder.base_working_dir = builder.base_working_dir.join(node_working_dir);
            }

            // Use a zenoh-backed logger when available so build output is
            // streamed to the CLI subscriber; fall back to tracing-only.
            let logger = if let (Some(session), Some(daemon_id)) =
                (state.zenoh_session.as_ref(), state.try_daemon_id())
            {
                let dest = log::LogDestination::Zenoh {
                    session: session.clone(),
                    publishers: std::collections::HashMap::new(),
                };
                let inner_logger = log::Logger {
                    destination: dest,
                    clock: state.clock.clone(),
                };
                let mut daemon_logger = inner_logger.for_daemon(daemon_id.clone());
                log::RpcBuildLogger::Zenoh(
                    daemon_logger
                        .for_node_build(build_id, node_id.clone())
                        .try_clone_impl()
                        .await?,
                )
            } else {
                log::RpcBuildLogger::Tracing(build::TracingBuildLogger::new(
                    build_id,
                    node_id.clone(),
                ))
            };

            let mut git_manager = state.git_manager.lock().await;
            match builder
                .build_node(node, git_source, prev_git, logger, &mut git_manager)
                .await
                .wrap_err_with(|| format!("failed to build node `{node_id}`"))
            {
                Ok(result) => {
                    tasks.push(NodeBuildTask {
                        node_id,
                        task: result,
                        dynamic_node: false,
                    });
                }
                Err(err) => {
                    tracing::error!("build failed for node {node_id}: {err:?}");
                    return Err(err);
                }
            }
        }

        let task = async move {
            let mut info = BuildInfo {
                node_working_dirs: Default::default(),
            };
            for task in tasks {
                let NodeBuildTask {
                    node_id,
                    dynamic_node: _,
                    task,
                } = task;
                let node = task
                    .await
                    .with_context(|| format!("failed to build node `{node_id}`"))?;
                info.node_working_dirs
                    .insert(node_id, node.node_working_dir);
            }
            Ok(info)
        };

        Ok(task)
    }
}

async fn read_last_n_lines(file: &mut File, mut tail: usize) -> io::Result<Vec<u8>> {
    let mut pos = file.seek(io::SeekFrom::End(0)).await?;

    let mut output = VecDeque::<u8>::new();
    let mut extend_slice_to_start = |slice: &[u8]| {
        output.extend(slice);
        output.rotate_right(slice.len());
    };

    let mut buffer = vec![0; 2048];
    let mut estimated_line_length = 0;
    let mut at_end = true;
    'main: while tail > 0 && pos > 0 {
        let new_pos = pos.saturating_sub(buffer.len() as u64);
        file.seek(io::SeekFrom::Start(new_pos)).await?;
        let read_len = (pos - new_pos) as usize;
        pos = new_pos;

        file.read_exact(&mut buffer[..read_len]).await?;
        let read_buf = if at_end {
            at_end = false;
            &buffer[..read_len].trim_ascii_end()
        } else {
            &buffer[..read_len]
        };

        let mut iter = memchr::memrchr_iter(b'\n', read_buf);
        let mut lines = 1;
        loop {
            let Some(pos) = iter.next() else {
                extend_slice_to_start(read_buf);
                break;
            };
            lines += 1;
            tail -= 1;
            if tail == 0 {
                extend_slice_to_start(&read_buf[(pos + 1)..]);
                break 'main;
            }
        }

        estimated_line_length = estimated_line_length.max((read_buf.len() + 1).div_ceil(lines));
        let estimated_buffer_length = estimated_line_length * tail;
        if estimated_buffer_length >= buffer.len() * 2 {
            buffer.resize(buffer.len() * 2, 0);
        }
    }

    Ok(output.into())
}

/// Set up the event stream for the daemon.
///
/// Returns `((daemon_id, coordinator_client), event_stream)`.
async fn set_up_event_stream(
    coordinator_addr: SocketAddr,
    machine_id: &Option<String>,
    clock: &Arc<HLC>,
    state: Arc<state::DaemonState>,
    remote_daemon_events_rx: flume::Receiver<eyre::Result<Timestamped<InterDaemonEvent>>>,
    // used for dynamic nodes
    local_listen_port: u16,
) -> eyre::Result<(
    (
        DaemonId,
        dora_message::daemon_to_coordinator::CoordinatorNotifyClient,
    ),
    impl Stream<Item = Timestamped<Event>> + Unpin,
)> {
    let clock_cloned = clock.clone();
    let remote_daemon_events = remote_daemon_events_rx.into_stream().map(move |e| match e {
        Ok(e) => Timestamped {
            inner: Event::Daemon(e.inner),
            timestamp: e.timestamp,
        },
        Err(err) => Timestamped {
            inner: Event::DaemonError(err),
            timestamp: clock_cloned.new_timestamp(),
        },
    });

    let register_result = coordinator::register(coordinator_addr, machine_id.clone(), clock, state)
        .await
        .wrap_err("failed to connect to dora-coordinator")?;
    // Monitor the RPC server task so panics are logged instead of silently lost.
    let rpc_server_handle = register_result.rpc_server_handle;
    let daemon_id = register_result.daemon_id;
    let coordinator_client = register_result.coordinator_client;
    tokio::spawn(async move {
        if let Err(err) = rpc_server_handle.await {
            tracing::error!("coordinator RPC server task panicked: {err}");
        }
    });

    let (events_tx, events_rx) = flume::bounded(10);
    let _listen_port =
        local_listener::spawn_listener_loop((LOCALHOST, local_listen_port).into(), events_tx)
            .await?;
    let dynamic_node_events = events_rx.into_stream().map(|e| Timestamped {
        inner: Event::DynamicNode(e.inner),
        timestamp: e.timestamp,
    });
    let incoming = (remote_daemon_events, dynamic_node_events).merge();
    Ok(((daemon_id, coordinator_client), incoming))
}

async fn send_output_to_local_receivers(
    node_id: NodeId,
    output_id: DataId,
    dataflow: &mut RunningDataflow,
    metadata: &metadata::Metadata,
    data: Option<DataMessage>,
    clock: &HLC,
) -> Result<Option<AVec<u8, ConstAlign<128>>>, eyre::ErrReport> {
    let timestamp = metadata.timestamp();
    let empty_set = BTreeSet::new();
    let output_id = OutputId(node_id, output_id);
    let local_receivers = dataflow.mappings.get(&output_id).unwrap_or(&empty_set);
    let OutputId(node_id, _) = output_id;
    let mut closed = Vec::new();
    for (receiver_id, input_id) in local_receivers {
        if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
            let item = NodeEvent::Input {
                id: input_id.clone(),
                metadata: metadata.clone(),
                data: data.clone(),
            };
            match channel.send(Timestamped {
                inner: item,
                timestamp,
            }) {
                Ok(()) => {
                    if let Some(token) = data.as_ref().and_then(|d| d.drop_token()) {
                        dataflow
                            .pending_drop_tokens
                            .entry(token)
                            .or_insert_with(|| DropTokenInformation {
                                owner: node_id.clone(),
                                pending_nodes: Default::default(),
                            })
                            .pending_nodes
                            .insert(receiver_id.clone());
                    }
                }
                Err(_) => {
                    closed.push(receiver_id);
                }
            }
        }
    }
    for id in closed {
        dataflow.subscribe_channels.remove(id);
    }
    let (data_bytes, drop_token) = match data {
        None => (None, None),
        Some(DataMessage::SharedMemory {
            shared_memory_id,
            len,
            drop_token,
        }) => {
            let memory = ShmemConf::new()
                .os_id(shared_memory_id)
                .open()
                .wrap_err("failed to map shared memory output")?;
            let data = Some(AVec::from_slice(1, &unsafe { memory.as_slice() }[..len]));
            (data, Some(drop_token))
        }
        Some(DataMessage::Vec(v)) => (Some(v), None),
    };
    if let Some(token) = drop_token {
        // insert token into `pending_drop_tokens` even if there are no local subscribers
        dataflow
            .pending_drop_tokens
            .entry(token)
            .or_insert_with(|| DropTokenInformation {
                owner: node_id.clone(),
                pending_nodes: Default::default(),
            });
        // check if all local subscribers are finished with the token
        dataflow.check_drop_token(token, clock).await?;
    }
    Ok(data_bytes)
}

fn node_inputs(node: &ResolvedNode) -> BTreeMap<DataId, Input> {
    match &node.kind {
        CoreNodeKind::Custom(n) => n.run_config.inputs.clone(),
        CoreNodeKind::Runtime(n) => runtime_node_inputs(n),
    }
}

fn close_input(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    input_id: &DataId,
    clock: &HLC,
) {
    if let Some(open_inputs) = dataflow.open_inputs.get_mut(receiver_id) {
        if !open_inputs.remove(input_id) {
            return;
        }
    }
    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
        let _ = send_with_timestamp(
            channel,
            NodeEvent::InputClosed {
                id: input_id.clone(),
            },
            clock,
        );

        if dataflow.open_inputs(receiver_id).is_empty() {
            if let Some(node) = dataflow.running_nodes.get_mut(receiver_id) {
                node.disable_restart();
            }
            let _ = send_with_timestamp(channel, NodeEvent::AllInputsClosed, clock);
        }
    }
}

#[derive(Debug)]
pub struct RunningNode {
    process: Option<ProcessHandle>,
    node_config: NodeConfig,
    pid: Option<Arc<AtomicU32>>,
    restart_policy: RestartPolicy,
    /// Don't restart the node even if the restart policy says so.
    ///
    /// This flag is set when all inputs of the node were closed and when a manual stop command
    /// was sent.
    disable_restart: Arc<AtomicBool>,
    /// Abort handle for this node's listener task, carried here until the dataflow
    /// collects it into `RunningDataflow::_listener_tasks`.
    listener_abort_handle: Option<tokio::task::AbortHandle>,
}

impl RunningNode {
    pub fn restarts_disabled(&self) -> bool {
        self.disable_restart.load(atomic::Ordering::Acquire)
    }

    pub fn disable_restart(&mut self) {
        self.disable_restart.store(true, atomic::Ordering::Release);
    }
}

#[derive(Debug)]
enum ProcessOperation {
    SoftKill,
    Kill,
}

impl ProcessOperation {
    pub fn execute(&self, child: &mut dyn TokioChildWrapper) {
        match self {
            Self::SoftKill => {
                #[cfg(unix)]
                {
                    // Send SIGTERM
                    if let Err(err) = child.signal(15) {
                        warn!("failed to send SIGTERM to process {:?}: {err}", child.id());
                    }
                }

                #[cfg(windows)]
                unsafe {
                    let Some(pid) = child.id() else {
                        warn!("failed to get child process id");
                        return;
                    };
                    if let Err(err) = windows::Win32::System::Console::GenerateConsoleCtrlEvent(
                        windows::Win32::System::Console::CTRL_BREAK_EVENT,
                        pid,
                    ) {
                        warn!("failed to send CTRL_BREAK_EVENT to process {pid}: {err}");
                    }
                }

                #[cfg(not(any(unix, windows)))]
                {
                    warn!("killing process is not implemented on this platform");
                }
            }
            Self::Kill => {
                if let Err(err) = child.start_kill() {
                    warn!("failed to kill child process: {err}");
                }
            }
        }
    }
}

#[derive(Debug)]
struct ProcessHandle {
    op_tx: flume::Sender<ProcessOperation>,
}

impl ProcessHandle {
    pub fn new(op_tx: flume::Sender<ProcessOperation>) -> Self {
        Self { op_tx }
    }

    /// Returns true if the process is not finished yet and the operation is
    /// delivered.
    pub fn submit(&self, operation: ProcessOperation) -> bool {
        self.op_tx.send(operation).is_ok()
    }
}

impl Drop for ProcessHandle {
    fn drop(&mut self) {
        if self.submit(ProcessOperation::Kill) {
            warn!("process was killed on drop because it was still running");
        }
    }
}

/// Aborts the associated tokio task when dropped.
struct ListenerTask(tokio::task::AbortHandle);

impl Drop for ListenerTask {
    fn drop(&mut self) {
        self.0.abort();
    }
}

pub struct RunningDataflow {
    id: Uuid,

    descriptor: Descriptor,

    /// Local nodes that are not started yet
    pending_nodes: PendingNodes,

    dataflow_started: bool,

    subscribe_channels: HashMap<NodeId, UnboundedSender<Timestamped<NodeEvent>>>,
    drop_channels: HashMap<NodeId, UnboundedSender<Timestamped<NodeDropEvent>>>,
    mappings: HashMap<OutputId, BTreeSet<InputId>>,
    timers: BTreeMap<Duration, BTreeSet<InputId>>,
    open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    running_nodes: BTreeMap<NodeId, RunningNode>,

    /// List of all dynamic node IDs.
    ///
    /// We want to treat dynamic nodes differently in some cases, so we need
    /// to know which nodes are dynamic.
    dynamic_nodes: BTreeSet<NodeId>,

    open_external_mappings: BTreeSet<OutputId>,

    pending_drop_tokens: HashMap<DropToken, DropTokenInformation>,

    /// Keep handles to all timer tasks of this dataflow to cancel them on drop.
    _timer_handles: BTreeMap<Duration, futures::future::RemoteHandle<()>>,
    /// Keep abort handles for all node listener tasks so they are cancelled when
    /// the dataflow finishes and this struct is dropped.
    _listener_tasks: Vec<ListenerTask>,
    stop_sent: bool,

    /// Used in `open_inputs`.

    /// Contains the node that caused the error for nodes that experienced a cascading error.
    cascading_error_causes: CascadingErrorCauses,
    grace_duration_kills: Arc<crossbeam_skiplist::SkipSet<NodeId>>,

    /// Tracks nodes whose NodeFailed event has already been delivered to local receivers.
    ///
    /// Because zenoh is topic-based, a remote daemon may publish one NodeFailed per output
    /// of the failed node. Without deduplication the receiving daemon would call
    /// find_and_notify_local_receivers once per message, delivering duplicate events.
    handled_node_failed: BTreeSet<NodeId>,

    node_stderr_most_recent: BTreeMap<NodeId, Arc<ArrayQueue<String>>>,

    publishers: BTreeMap<OutputId, zenoh::pubsub::Publisher<'static>>,

    finished_tx: broadcast::Sender<()>,

    publish_all_messages_to_zenoh: bool,
}

/// Indicates whether a dataflow should be finished immediately after stop_all()
/// or whether to wait for SpawnedNodeResult events from running nodes.
#[must_use]
pub enum FinishDataflowWhen {
    /// Finish the dataflow immediately (all nodes are dynamic or no nodes running)
    Now,
    /// Wait for SpawnedNodeResult events from non-dynamic nodes
    WaitForNodes,
}

impl RunningDataflow {
    fn new(
        dataflow_id: Uuid,
        daemon_id: DaemonId,
        dataflow_descriptor: Descriptor,
        events_tx: tokio::sync::mpsc::Sender<Timestamped<Event>>,
        clock: Arc<HLC>,
    ) -> RunningDataflow {
        let (finished_tx, _) = broadcast::channel(1);
        Self {
            id: dataflow_id,
            pending_nodes: PendingNodes::new(dataflow_id, daemon_id, events_tx, clock),
            dataflow_started: false,
            subscribe_channels: HashMap::new(),
            drop_channels: HashMap::new(),
            mappings: HashMap::new(),
            timers: BTreeMap::new(),
            open_inputs: BTreeMap::new(),
            running_nodes: BTreeMap::new(),
            dynamic_nodes: BTreeSet::new(),
            open_external_mappings: Default::default(),
            pending_drop_tokens: HashMap::new(),
            _timer_handles: BTreeMap::new(),
            _listener_tasks: Vec::new(),
            stop_sent: false,
            cascading_error_causes: Default::default(),
            grace_duration_kills: Default::default(),
            handled_node_failed: BTreeSet::new(),
            node_stderr_most_recent: BTreeMap::new(),
            publishers: Default::default(),
            finished_tx,
            publish_all_messages_to_zenoh: dataflow_descriptor.debug.publish_all_messages_to_zenoh,
            descriptor: dataflow_descriptor,
        }
    }

    async fn start(
        &mut self,
        events_tx: &mpsc::Sender<Timestamped<Event>>,
        clock: &Arc<HLC>,
    ) -> eyre::Result<()> {
        for interval in self.timers.keys().copied() {
            if self._timer_handles.get(&interval).is_some() {
                continue;
            }
            let events_tx = events_tx.clone();
            let dataflow_id = self.id;
            let clock = clock.clone();
            let task = async move {
                let mut interval_stream = tokio::time::interval(interval);
                let hlc = HLC::default();
                loop {
                    interval_stream.tick().await;

                    let span = tracing::span!(tracing::Level::TRACE, "tick");
                    let _ = span.enter();

                    let mut parameters = BTreeMap::new();
                    parameters.insert(
                        "open_telemetry_context".to_string(),
                        #[cfg(feature = "telemetry")]
                        Parameter::String(serialize_context(&span.context())),
                        #[cfg(not(feature = "telemetry"))]
                        Parameter::String("".into()),
                    );

                    let metadata = metadata::Metadata::from_parameters(
                        hlc.new_timestamp(),
                        empty_type_info(),
                        parameters,
                    );

                    let event = Timestamped {
                        inner: DoraEvent::Timer {
                            dataflow_id,
                            interval,
                            metadata,
                        }
                        .into(),
                        timestamp: clock.new_timestamp(),
                    };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
            };
            let (task, handle) = task.remote_handle();
            tokio::spawn(task);
            self._timer_handles.insert(interval, handle);
        }

        Ok(())
    }

    async fn stop_all(
        &mut self,
        state: &state::DaemonState,
        grace_duration: Option<Duration>,
        force: bool,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<FinishDataflowWhen> {
        self.pending_nodes
            .handle_dataflow_stop(
                &state.coordinator_client().cloned(),
                &state.clock,
                &mut self.cascading_error_causes,
                &self.dynamic_nodes,
                logger,
            )
            .await?;

        for node in self.running_nodes.values_mut() {
            node.disable_restart();
        }

        for (_node_id, channel) in self.subscribe_channels.drain() {
            let _ = send_with_timestamp(&channel, NodeEvent::Stop, &state.clock);
        }

        let running_processes: Vec<_> = self
            .running_nodes
            .iter_mut()
            .map(|(id, n)| (id.clone(), n.process.take()))
            .collect();
        if force {
            for (_, proc) in &running_processes {
                if let Some(proc) = proc {
                    proc.submit(crate::ProcessOperation::Kill);
                }
            }
        } else {
            let grace_duration_kills = self.grace_duration_kills.clone();
            tokio::spawn(async move {
                let duration = grace_duration.unwrap_or(Duration::from_millis(10000));
                tokio::time::sleep(duration).await;

                for (node, proc) in &running_processes {
                    if let Some(proc) = proc {
                        grace_duration_kills.insert(node.clone());
                        proc.submit(crate::ProcessOperation::SoftKill);
                    }
                }

                let kill_duration = duration / 2;
                tokio::time::sleep(kill_duration).await;

                for (node, proc) in &running_processes {
                    if let Some(proc) = proc {
                        grace_duration_kills.insert(node.clone());
                        proc.submit(crate::ProcessOperation::Kill);
                        warn!(
                            "{node} was killed due to not stopping within the {:#?} grace period",
                            duration + kill_duration
                        );
                    }
                }
            });
        }
        self.stop_sent = true;

        // Determine if we should finish immediately or wait for nodes
        Ok(self.should_finish_immediately())
    }

    /// Synchronous part of `stop_all` — sends stop signals to nodes and
    /// schedules grace-period kills. Does NOT do any RPC or async I/O, so
    /// it is safe to call while holding a DashMap guard.
    fn stop_all_after_pending(
        &mut self,
        state: &state::DaemonState,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> eyre::Result<FinishDataflowWhen> {
        for node in self.running_nodes.values_mut() {
            node.disable_restart();
        }

        for (_node_id, channel) in self.subscribe_channels.drain() {
            let _ = send_with_timestamp(&channel, NodeEvent::Stop, &state.clock);
        }

        let running_processes: Vec<_> = self
            .running_nodes
            .iter_mut()
            .map(|(id, n)| (id.clone(), n.process.take()))
            .collect();
        if force {
            for (_, proc) in &running_processes {
                if let Some(proc) = proc {
                    proc.submit(crate::ProcessOperation::Kill);
                }
            }
        } else {
            let grace_duration_kills = self.grace_duration_kills.clone();
            tokio::spawn(async move {
                let duration = grace_duration.unwrap_or(Duration::from_millis(10000));
                tokio::time::sleep(duration).await;

                for (node, proc) in &running_processes {
                    if let Some(proc) = proc {
                        grace_duration_kills.insert(node.clone());
                        proc.submit(crate::ProcessOperation::SoftKill);
                    }
                }

                let kill_duration = duration / 2;
                tokio::time::sleep(kill_duration).await;

                for (node, proc) in &running_processes {
                    if let Some(proc) = proc {
                        grace_duration_kills.insert(node.clone());
                        proc.submit(crate::ProcessOperation::Kill);
                        warn!(
                            "{node} was killed due to not stopping within the {:#?} grace period",
                            duration + kill_duration
                        );
                    }
                }
            });
        }
        self.stop_sent = true;

        Ok(self.should_finish_immediately())
    }

    /// Check if dataflow should finish immediately after stop_all().
    /// Returns `Now` if all running nodes are dynamic (they won't send SpawnedNodeResult).
    /// Returns `WaitForNodes` if there are non-dynamic nodes to wait for.
    fn should_finish_immediately(&self) -> FinishDataflowWhen {
        // Only finish immediately if:
        // 1. No pending nodes
        // 2. All running nodes are dynamic (they won't send SpawnedNodeResult)
        // 3. Stop was sent (stop_all() was called)
        if !self.pending_nodes.local_nodes_pending()
            && self
                .running_nodes
                .iter()
                .all(|(_id, n)| n.node_config.dynamic)
            && self.stop_sent
        {
            FinishDataflowWhen::Now
        } else {
            FinishDataflowWhen::WaitForNodes
        }
    }

    fn open_inputs(&self, node_id: &NodeId) -> &BTreeSet<DataId> {
        self.open_inputs.get(node_id).unwrap_or(&EMPTY_SET)
    }

    async fn check_drop_token(&mut self, token: DropToken, clock: &HLC) -> eyre::Result<()> {
        match self.pending_drop_tokens.entry(token) {
            std::collections::hash_map::Entry::Occupied(entry) => {
                if entry.get().pending_nodes.is_empty() {
                    let (drop_token, info) = entry.remove_entry();
                    let result = match self.drop_channels.get_mut(&info.owner) {
                        Some(channel) => send_with_timestamp(
                            channel,
                            NodeDropEvent::OutputDropped { drop_token },
                            clock,
                        )
                        .wrap_err("send failed"),
                        None => Err(eyre!("no subscribe channel for node `{}`", &info.owner)),
                    };
                    if let Err(err) = result.wrap_err_with(|| {
                        format!(
                            "failed to report drop token `{drop_token:?}` to owner `{}`",
                            &info.owner
                        )
                    }) {
                        tracing::warn!("{err:?}");
                    }
                }
            }
            std::collections::hash_map::Entry::Vacant(_) => {
                tracing::warn!("check_drop_token called with already closed token")
            }
        }

        Ok(())
    }
}

fn empty_type_info() -> ArrowTypeInfo {
    ArrowTypeInfo {
        data_type: DataType::Null,
        len: 0,
        null_count: 0,
        validity: None,
        offset: 0,
        buffer_offsets: Vec::new(),
        child_data: Vec::new(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct OutputId(NodeId, DataId);
type InputId = (NodeId, DataId);

struct DropTokenInformation {
    /// The node that created the associated drop token.
    owner: NodeId,
    /// Contains the set of pending nodes that still have access to the input
    /// associated with a drop token.
    pending_nodes: BTreeSet<NodeId>,
}

#[derive(Debug)]
pub enum Event {
    Node {
        dataflow_id: DataflowId,
        node_id: NodeId,
        event: DaemonNodeEvent,
    },
    Daemon(InterDaemonEvent),
    Dora(DoraEvent),
    DynamicNode(DynamicNodeEventWrapper),
    HeartbeatInterval,
    MetricsInterval,
    CtrlC,
    StopAfter(Duration),
    SecondCtrlC,
    DaemonError(eyre::Report),
    /// Coordinator requested a destroy — shut down the daemon.
    Destroy,
    /// Coordinator requested spawning nodes (routed from RPC server).
    SpawnRequest {
        build_id: Option<BuildId>,
        dataflow_id: DataflowId,
        local_working_dir: Option<PathBuf>,
        nodes: BTreeMap<NodeId, ResolvedNode>,
        dataflow_descriptor: Descriptor,
        spawn_nodes: BTreeSet<NodeId>,
        uv: bool,
        write_events_to: Option<PathBuf>,
        reply_tx: oneshot::Sender<Result<(), String>>,
    },
    /// Coordinator requested stopping a dataflow (routed from RPC server).
    StopDataflowRequest {
        dataflow_id: DataflowId,
        grace_duration: Option<Duration>,
        force: bool,
        reply_tx: oneshot::Sender<Result<(), String>>,
    },
    SpawnNodeResult {
        dataflow_id: DataflowId,
        node_id: NodeId,
        dynamic_node: bool,
        result: Result<RunningNode, NodeError>,
    },
    SpawnDataflowResult {
        dataflow_id: Uuid,
        result: eyre::Result<()>,
    },
    NodeStopped {
        dataflow_id: Uuid,
        node_id: NodeId,
    },
}

impl From<DoraEvent> for Event {
    fn from(event: DoraEvent) -> Self {
        Event::Dora(event)
    }
}

impl Event {
    pub fn kind(&self) -> &'static str {
        match self {
            Event::Node { .. } => "Node",
            Event::Daemon(_) => "Daemon",
            Event::Dora(_) => "Dora",
            Event::DynamicNode(_) => "DynamicNode",
            Event::HeartbeatInterval => "HeartbeatInterval",
            Event::MetricsInterval => "MetricsInterval",
            Event::CtrlC => "CtrlC",
            Event::StopAfter(_) => "StopAfter",
            Event::SecondCtrlC => "SecondCtrlC",
            Event::DaemonError(_) => "DaemonError",
            Event::Destroy => "Destroy",
            Event::SpawnRequest { .. } => "SpawnRequest",
            Event::StopDataflowRequest { .. } => "StopDataflowRequest",
            Event::SpawnNodeResult { .. } => "SpawnNodeResult",
            Event::SpawnDataflowResult { .. } => "SpawnDataflowResult",
            Event::NodeStopped { .. } => "NodeStopped",
        }
    }
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum DaemonNodeEvent {
    OutputsDone {
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    Subscribe {
        event_sender: UnboundedSender<Timestamped<NodeEvent>>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SubscribeDrop {
        event_sender: UnboundedSender<Timestamped<NodeDropEvent>>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    CloseOutputs {
        outputs: Vec<dora_core::config::DataId>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SendOut {
        output_id: DataId,
        metadata: metadata::Metadata,
        data: Option<DataMessage>,
    },
    ReportDrop {
        tokens: Vec<DropToken>,
    },
    EventStreamDropped {
        reply_sender: oneshot::Sender<DaemonReply>,
    },
}

#[derive(Debug)]
pub enum DoraEvent {
    Timer {
        dataflow_id: DataflowId,
        interval: Duration,
        metadata: metadata::Metadata,
    },
    Logs {
        dataflow_id: DataflowId,
        output_id: OutputId,
        message: DataMessage,
        metadata: metadata::Metadata,
    },
    SpawnedNodeResult {
        dataflow_id: DataflowId,
        node_id: NodeId,
        dynamic_node: bool,
        exit_status: NodeExitStatus,
        /// Whether the node will be restarted
        restart: bool,
    },
}

fn send_with_timestamp<T>(
    sender: &UnboundedSender<Timestamped<T>>,
    event: T,
    clock: &HLC,
) -> Result<(), mpsc::error::SendError<Timestamped<T>>> {
    sender.send(Timestamped {
        inner: event,
        timestamp: clock.new_timestamp(),
    })
}

fn set_up_ctrlc_handler(
    clock: Arc<HLC>,
) -> eyre::Result<tokio::sync::mpsc::Receiver<Timestamped<Event>>> {
    let (ctrlc_tx, ctrlc_rx) = mpsc::channel(1);

    let mut ctrlc_sent = 0;
    ctrlc::set_handler(move || {
        let event = match ctrlc_sent {
            0 => Event::CtrlC,
            1 => Event::SecondCtrlC,
            _ => {
                tracing::warn!("received 3rd ctrlc signal -> aborting immediately");
                std::process::abort();
            }
        };
        if ctrlc_tx
            .blocking_send(Timestamped {
                inner: event,
                timestamp: clock.new_timestamp(),
            })
            .is_err()
        {
            tracing::error!("failed to report ctrl-c event to dora-coordinator");
        }

        ctrlc_sent += 1;
    })
    .wrap_err("failed to set ctrl-c handler")?;

    Ok(ctrlc_rx)
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CascadingErrorCauses {
    caused_by: BTreeMap<NodeId, NodeId>,
}

impl CascadingErrorCauses {
    pub fn experienced_cascading_error(&self, node: &NodeId) -> bool {
        self.caused_by.contains_key(node)
    }

    /// Return the ID of the node that caused a cascading error for the given node, if any.
    pub fn error_caused_by(&self, node: &NodeId) -> Option<&NodeId> {
        self.caused_by.get(node)
    }

    pub fn report_cascading_error(&mut self, causing_node: NodeId, affected_node: NodeId) {
        self.caused_by.entry(affected_node).or_insert(causing_node);
    }
}

fn runtime_node_inputs(n: &RuntimeNode) -> BTreeMap<DataId, Input> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator.config.inputs.iter().map(|(input_id, mapping)| {
                (
                    DataId::from(format!("{}/{input_id}", operator.id)),
                    mapping.clone(),
                )
            })
        })
        .collect()
}

fn runtime_node_outputs(n: &RuntimeNode) -> BTreeSet<DataId> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator
                .config
                .outputs
                .iter()
                .map(|output_id| DataId::from(format!("{}/{output_id}", operator.id)))
        })
        .collect()
}

trait CoreNodeKindExt {
    fn run_config(&self) -> NodeRunConfig;
    fn dynamic(&self) -> bool;
}

impl CoreNodeKindExt for CoreNodeKind {
    fn run_config(&self) -> NodeRunConfig {
        match self {
            CoreNodeKind::Runtime(n) => NodeRunConfig {
                inputs: runtime_node_inputs(n),
                outputs: runtime_node_outputs(n),
            },
            CoreNodeKind::Custom(n) => n.run_config.clone(),
        }
    }

    fn dynamic(&self) -> bool {
        match self {
            CoreNodeKind::Runtime(_n) => false,
            CoreNodeKind::Custom(n) => {
                matches!(&n.source, NodeSource::Local) && n.path == DYNAMIC_SOURCE
            }
        }
    }
}
