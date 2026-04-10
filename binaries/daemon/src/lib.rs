use aligned_vec::{AVec, ConstAlign};
use coordinator::CoordinatorEvent;
use crossbeam::queue::ArrayQueue;
use dora_core::{
    build::{self, BuildInfo, GitManager, PrevGitSource},
    config::{DataId, Input, InputMapping, NodeId, NodeRunConfig, OperatorId},
    descriptor::{
        CoreNodeKind, DYNAMIC_SOURCE, Descriptor, DescriptorExt, ResolvedNode, RuntimeNode,
        read_as_descriptor,
    },
    topics::{
        DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST, open_zenoh_session,
        zenoh_output_publish_topic,
    },
    uhlc::{self, HLC},
};
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{
        DaemonId, DataMessage, GitSource, LogLevel, NodeError, NodeErrorCause, NodeExitStatus,
    },
    coordinator_to_cli::DataflowResult,
    coordinator_to_daemon::{
        BuildDataflowNodes, DaemonCoordinatorEvent, SpawnDataflowNodes, StateCatchUpOperation,
    },
    daemon_to_coordinator::{
        CoordinatorRequest, DaemonCoordinatorReply, DaemonEvent, DataflowDaemonResult,
    },
    daemon_to_node::{DaemonReply, NodeConfig, NodeEvent},
    descriptor::{NodeSource, RestartPolicy},
    metadata::{self, ArrowTypeInfo},
    node_to_daemon::{DynamicNodeEvent, Timestamped},
};
use dora_node_api::arrow::datatypes::DataType;
use eyre::{Context, ContextCompat, Result, bail, eyre};
use futures::{TryFutureExt, future, stream};
use futures_concurrency::stream::Merge;
use local_listener::DynamicNodeEventWrapper;
use log::{DaemonLogger, DataflowLogger, Logger};
use shared_memory_server::ShmemConf;
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
        atomic::{self, AtomicU32, AtomicU64},
    },
    time::{Duration, Instant},
};
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncSeekExt},
    sync::{
        broadcast,
        mpsc::{self},
        oneshot::{self, Sender},
    },
};
use tokio_stream::{Stream, StreamExt, wrappers::ReceiverStream};
use tracing::error;
use uuid::{NoContext, Timestamp, Uuid};

pub use flume;
pub use log::LogDestination;

/// Benchmark support: exposes internal routing functions for criterion benchmarks.
/// Not part of the public API.
#[cfg(feature = "bench")]
#[doc(hidden)]
pub mod bench_support {
    use super::*;

    /// Create a minimal `RunningDataflow` with the given sender->receiver mapping.
    /// Returns the dataflow and a vec of receivers (one per subscriber).
    pub fn setup_routing(
        fan_out: usize,
    ) -> (
        RunningDataflow,
        HLC,
        Vec<mpsc::Receiver<Timestamped<NodeEvent>>>,
    ) {
        let descriptor = dora_message::descriptor::Descriptor {
            nodes: vec![],
            communication: dora_message::config::CommunicationConfig::default(),
            deploy: None,
            debug: dora_message::descriptor::Debug::default(),
            health_check_interval: None,
            strict_types: None,
            type_rules: vec![],
        };
        let mut df = RunningDataflow::new(Uuid::nil(), DaemonId::new(None), descriptor);

        let sender_id: NodeId = "sender".to_string().into();
        let output_id: DataId = "output".to_string().into();

        let mut receivers = Vec::new();
        let mut mapping = BTreeSet::new();

        let input_id: DataId = "input".to_string().into();
        for i in 0..fan_out {
            let receiver_id: NodeId = format!("receiver_{i}").into();
            let (tx, rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
            df.subscribe_channels.insert(receiver_id.clone(), tx);
            df.pending_messages
                .insert(receiver_id.clone(), Arc::new(AtomicU64::new(0)));
            mapping.insert((receiver_id, input_id.clone()));
            receivers.push(rx);
        }

        df.mappings.insert(OutputId(sender_id, output_id), mapping);

        let clock = HLC::default();
        (df, clock, receivers)
    }

    /// Pre-built message components for benchmark iterations.
    pub struct RoutingFixture {
        pub sender_id: NodeId,
        pub output_id: DataId,
        pub data_msg: DataMessage,
        pub metadata: metadata::Metadata,
    }

    /// Create a reusable fixture for the routing hot path (call once per config).
    pub fn make_fixture(clock: &HLC, payload_size: usize) -> RoutingFixture {
        let data = vec![0u8; payload_size];
        let type_info = ArrowTypeInfo {
            data_type: dora_node_api::arrow::datatypes::DataType::UInt8,
            len: payload_size,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![],
            child_data: vec![],
            field_names: None,
            schema_hash: None,
        };
        RoutingFixture {
            sender_id: "sender".to_string().into(),
            output_id: "output".to_string().into(),
            data_msg: DataMessage::Vec(AVec::from_slice(128, &data)),
            metadata: metadata::Metadata::new(clock.new_timestamp(), type_info),
        }
    }

    /// Run one iteration of the routing hot path using pre-built fixture.
    pub async fn route_message(df: &mut RunningDataflow, fixture: &RoutingFixture, clock: &HLC) {
        let _ = send_output_to_local_receivers(
            fixture.sender_id.clone(),
            fixture.output_id.clone(),
            df,
            &fixture.metadata,
            Some(fixture.data_msg.clone()),
            clock,
            None,
            false, // bench: no remote receivers
        )
        .await;
    }
}

mod coordinator;
pub(crate) mod event_types;
mod extract_err_from_stderr;
pub(crate) mod fault_tolerance;
mod local_listener;
mod log;
mod node_communication;
mod pending;
pub(crate) mod running_dataflow;
mod socket_stream_utils;
mod spawn;

pub(crate) use event_types::{
    CONTROL_EVENT_HEADROOM, DaemonNodeEvent, DoraEvent, Event, InterDaemonEvent,
    NODE_EVENT_CHANNEL_CAPACITY, OutputId, RunStatus, ZenohOutbound, send_drop_with_timestamp,
    send_with_timestamp,
};
pub(crate) use fault_tolerance::{CascadingErrorCauses, FaultToleranceStats};
pub(crate) use running_dataflow::{
    DropTokenInformation, FinishDataflowWhen, InputDeadline, ProcessHandle, ProcessOperation,
    RunningDataflow, RunningNode,
};

use crate::{extract_err_from_stderr::extract_err_from_stderr, pending::DataflowStatus};

const STDERR_LOG_LINES_MAX: usize = 500;
const METRICS_INTERVAL: Duration = Duration::from_secs(2);
const METRICS_INTERVAL_SECS: f64 = METRICS_INTERVAL.as_secs_f64();
/// Capacity of the Zenoh publish drain channel. Large enough for burst
/// patterns; messages are dropped with a warning when full.
const ZENOH_PUBLISH_CHANNEL_CAPACITY: usize = 256;

/// The Daemon manages running dataflows, node communication, and inter-daemon
/// message routing. Fields are `pub(crate)` to enable `impl Daemon` blocks in
/// submodules (e.g., `node_events.rs`, `dataflow_lifecycle.rs`) as part of the
/// ongoing daemon module split.
pub struct Daemon {
    pub(crate) running: HashMap<DataflowId, RunningDataflow>,
    pub(crate) working_dir: HashMap<DataflowId, PathBuf>,
    pub(crate) events_tx: mpsc::Sender<Timestamped<Event>>,
    pub(crate) coordinator_sender: Option<coordinator::CoordinatorSender>,
    pub(crate) last_coordinator_heartbeat: Instant,
    pub(crate) daemon_id: DaemonId,
    pub(crate) exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
    pub(crate) exit_when_all_finished: bool,
    pub(crate) dataflow_node_results: BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>,
    pub(crate) clock: Arc<uhlc::HLC>,
    pub(crate) ft_stats: Arc<FaultToleranceStats>,
    pub(crate) zenoh_session: zenoh::Session,
    pub(crate) zenoh_publish_tx: mpsc::Sender<ZenohOutbound>,
    pub(crate) remote_daemon_events_tx:
        Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
    pub(crate) logger: DaemonLogger,
    pub(crate) sessions: BTreeMap<SessionId, BuildId>,
    pub(crate) builds: BTreeMap<BuildId, BuildInfo>,
    pub(crate) git_manager: GitManager,
    pub(crate) metrics_system: Arc<std::sync::Mutex<sysinfo::System>>,
}

type DaemonRunResult = BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>;

struct NodeBuildTask<F> {
    node_id: NodeId,
    dynamic_node: bool,
    task: F,
}

/// Snapshot of a single node for background metrics collection.
struct NodeSnapshot {
    node_id: NodeId,
    pid: Option<Arc<AtomicU32>>,
    restart_count: Arc<AtomicU32>,
    restarts_disabled: bool,
    broken_inputs: Vec<String>,
    pending_messages: u64,
}

/// Snapshot of a running dataflow for background metrics collection.
struct DataflowMetricsSnapshot {
    dataflow_id: DataflowId,
    nodes: Vec<NodeSnapshot>,
    net_bytes_sent: Arc<AtomicU64>,
    net_bytes_received: Arc<AtomicU64>,
    net_messages_sent: Arc<AtomicU64>,
    net_messages_received: Arc<AtomicU64>,
    net_publish_failures: Arc<AtomicU64>,
}

/// Collect and send metrics in the background. Errors are returned to the
/// caller (the spawned task logs them).
async fn collect_and_send_metrics_bg(
    dataflows: Vec<DataflowMetricsSnapshot>,
    metrics_system: Arc<std::sync::Mutex<sysinfo::System>>,
    sender: coordinator::CoordinatorSender,
    daemon_id: DaemonId,
    clock: Arc<uhlc::HLC>,
) -> eyre::Result<()> {
    use dora_message::daemon_to_coordinator::NodeMetrics;
    use sysinfo::{Pid, ProcessRefreshKind, ProcessesToUpdate};

    let has_any_running = dataflows
        .iter()
        .any(|df| df.nodes.iter().any(|n| n.pid.is_some()));

    // Refresh sysinfo on a blocking thread if any nodes are running.
    // Use try_lock to skip if a previous collection is still in progress.
    let refreshed_system = if has_any_running {
        let sys = match metrics_system.try_lock() {
            Ok(mut guard) => std::mem::take(&mut *guard),
            Err(_) => {
                tracing::debug!("metrics: skipping, previous collection still running");
                return Ok(());
            }
        };
        let refresh_kind = ProcessRefreshKind::nothing()
            .with_cpu()
            .with_memory()
            .with_disk_usage();
        let sys = tokio::task::spawn_blocking(move || {
            let mut sys = sys;
            sys.refresh_processes_specifics(ProcessesToUpdate::All, true, refresh_kind);
            sys
        })
        .await
        .unwrap_or_else(|e| {
            tracing::error!("sysinfo refresh panicked: {e}");
            sysinfo::System::new()
        });
        Some(sys)
    } else {
        None
    };

    for df in &dataflows {
        let mut metrics = BTreeMap::new();

        if let Some(ref sys) = refreshed_system {
            // Pre-build parent->children map once per refresh.
            let mut children_map: HashMap<sysinfo::Pid, Vec<sysinfo::Pid>> = HashMap::new();
            for (pid, proc_info) in sys.processes() {
                if let Some(parent) = proc_info.parent() {
                    children_map.entry(parent).or_default().push(*pid);
                }
            }

            for node in &df.nodes {
                if let Some(pid_arc) = node.pid.as_ref() {
                    let pid = pid_arc.load(atomic::Ordering::Acquire);
                    let sys_pid = Pid::from_u32(pid);
                    if let Some(process) = sys.process(sys_pid) {
                        let mut cpu_usage = process.cpu_usage();
                        let mut memory_bytes = process.memory();
                        let disk_usage = process.disk_usage();
                        let mut disk_read = disk_usage.read_bytes;
                        let mut disk_written = disk_usage.written_bytes;

                        // Recursively aggregate all descendants.
                        let mut stack = vec![sys_pid];
                        while let Some(parent) = stack.pop() {
                            if let Some(kids) = children_map.get(&parent) {
                                for &child_pid in kids {
                                    if let Some(child) = sys.processes().get(&child_pid) {
                                        cpu_usage += child.cpu_usage();
                                        memory_bytes += child.memory();
                                        let child_disk = child.disk_usage();
                                        disk_read += child_disk.read_bytes;
                                        disk_written += child_disk.written_bytes;
                                    }
                                    stack.push(child_pid);
                                }
                            }
                        }

                        let restart_count = node.restart_count.load(atomic::Ordering::Acquire);
                        let status = if !node.broken_inputs.is_empty() {
                            dora_message::daemon_to_coordinator::NodeStatus::Degraded
                        } else {
                            dora_message::daemon_to_coordinator::NodeStatus::Running
                        };

                        metrics.insert(
                            node.node_id.clone(),
                            NodeMetrics {
                                pid,
                                cpu_usage,
                                memory_bytes,
                                disk_read_bytes: Some(
                                    (disk_read as f64 / METRICS_INTERVAL_SECS) as u64,
                                ),
                                disk_write_bytes: Some(
                                    (disk_written as f64 / METRICS_INTERVAL_SECS) as u64,
                                ),
                                restart_count,
                                broken_inputs: node.broken_inputs.clone(),
                                status,
                                pending_messages: node.pending_messages,
                            },
                        );
                    }
                }
            }
        }

        // Second pass: report nodes without a running process.
        for node in &df.nodes {
            if !metrics.contains_key(&node.node_id) {
                let restart_count = node.restart_count.load(atomic::Ordering::Acquire);
                let status = if node.restarts_disabled {
                    dora_message::daemon_to_coordinator::NodeStatus::Failed
                } else if restart_count > 0 {
                    dora_message::daemon_to_coordinator::NodeStatus::Restarting
                } else {
                    continue;
                };
                metrics.insert(
                    node.node_id.clone(),
                    NodeMetrics {
                        pid: 0,
                        cpu_usage: 0.0,
                        memory_bytes: 0,
                        disk_read_bytes: None,
                        disk_write_bytes: None,
                        restart_count,
                        broken_inputs: Vec::new(),
                        status,
                        pending_messages: node.pending_messages,
                    },
                );
            }
        }

        if !metrics.is_empty() {
            let network = {
                let bs = df.net_bytes_sent.load(atomic::Ordering::Relaxed);
                let br = df.net_bytes_received.load(atomic::Ordering::Relaxed);
                let ms = df.net_messages_sent.load(atomic::Ordering::Relaxed);
                let mr = df.net_messages_received.load(atomic::Ordering::Relaxed);
                let pf = df.net_publish_failures.load(atomic::Ordering::Relaxed);
                if bs > 0 || br > 0 || ms > 0 || mr > 0 || pf > 0 {
                    Some(dora_message::daemon_to_coordinator::NetworkMetrics {
                        bytes_sent: bs,
                        bytes_received: br,
                        messages_sent: ms,
                        messages_received: mr,
                        publish_failures: pf,
                    })
                } else {
                    None
                }
            };
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: daemon_id.clone(),
                    event: DaemonEvent::NodeMetrics {
                        dataflow_id: df.dataflow_id,
                        metrics,
                        network,
                    },
                },
                timestamp: clock.new_timestamp(),
            })?;
            if let Err(e) = sender.send_event(&msg).await {
                tracing::warn!("failed to send metrics for dataflow: {e}");
                continue;
            }
        }
    }

    // Return the refreshed System to the shared mutex for reuse.
    if let Some(sys) = refreshed_system
        && let Ok(mut guard) = metrics_system.lock()
    {
        *guard = sys;
    }

    Ok(())
}

impl Daemon {
    pub async fn run(
        coordinator_ws_addr: SocketAddr,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        local_listen_port: u16,
    ) -> eyre::Result<()> {
        let clock = Arc::new(HLC::default());
        let mut ctrlc_events = set_up_ctrlc_handler(clock.clone())?;
        let mut reconnect_attempt = 0u32;

        loop {
            // Sized for bursts of inter-daemon events
            let (remote_daemon_events_tx, remote_daemon_events_rx) = flume::bounded(100);

            let connect_result = {
                let incoming_events = set_up_event_stream(
                    coordinator_ws_addr,
                    &machine_id,
                    labels.clone(),
                    &clock,
                    remote_daemon_events_rx,
                    local_listen_port,
                );

                let ctrl_c = pin!(ctrlc_events.recv());
                match futures::future::select(ctrl_c, pin!(incoming_events)).await {
                    future::Either::Left((_ctrl_c, _)) => {
                        tracing::info!("received ctrl-c signal -> stopping daemon");
                        return Ok(());
                    }
                    future::Either::Right((events, _)) => events,
                }
            };

            match connect_result {
                Ok((daemon_id, coordinator_sender, incoming_events)) => {
                    reconnect_attempt = 0;
                    let log_destination = LogDestination::Coordinator {
                        sender: coordinator_sender.clone(),
                    };

                    // Don't pass ctrlc_events into run_general — keep it in
                    // the outer loop so Ctrl+C works across reconnect cycles.
                    // (ctrlc::set_handler can only be called once per process)
                    let run_future = Self::run_general(
                        incoming_events,
                        Some(coordinator_sender),
                        daemon_id,
                        None,
                        clock.clone(),
                        Some(remote_daemon_events_tx),
                        Default::default(),
                        log_destination,
                        None,
                    );

                    let result = tokio::select! {
                        r = run_future => r,
                        _ = ctrlc_events.recv() => {
                            tracing::info!("received ctrl-c signal -> stopping daemon");
                            return Ok(());
                        }
                    };

                    match result {
                        Ok(_) => return Ok(()),
                        Err(e) => {
                            tracing::warn!(
                                "daemon disconnected from coordinator: {e:#}. \
                                 Attempting reconnect..."
                            );
                        }
                    }
                }
                Err(e) => {
                    tracing::warn!(
                        attempt = reconnect_attempt,
                        "failed to connect to coordinator: {e:#}"
                    );
                }
            }

            // Exponential backoff: 1s, 2s, 4s, 8s, 16s, 30s, 30s...
            let delay = Duration::from_secs((1u64 << reconnect_attempt.min(5)).min(30));
            reconnect_attempt += 1;
            tracing::info!("reconnecting in {delay:?}...");
            tokio::time::sleep(delay).await;
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub async fn run_dataflow(
        dataflow_path: &Path,
        build_id: Option<BuildId>,
        local_build: Option<BuildInfo>,
        session_id: SessionId,
        uv: bool,
        log_destination: LogDestination,
        write_events_to: Option<PathBuf>,
        stop_after: Option<Duration>,
        debug: bool,
    ) -> eyre::Result<DataflowResult> {
        let working_dir = dataflow_path
            .canonicalize()
            .context("failed to canonicalize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
            .to_owned();

        let mut descriptor = read_as_descriptor(dataflow_path).await?;
        if debug {
            descriptor.debug.publish_all_messages_to_zenoh = true;
        }
        if let Some(node) = descriptor.nodes.iter().find(|n| n.deploy.is_some()) {
            eyre::bail!(
                "node {} has a `deploy` section, which is not supported in `dora run`\n\n
                Instead, you need to spawn a `dora coordinator` and one or more `dora daemon`
                instances and then use `dora start`.",
                node.id
            )
        }

        descriptor.check(&working_dir)?;
        let health_check_interval = descriptor
            .health_check_interval
            .map(Duration::from_secs_f64);
        let nodes = descriptor.resolve_aliases_and_set_defaults()?;

        let (events_tx, events_rx) = flume::bounded(10);
        if nodes
            .iter()
            .any(|(_n, resolved_nodes)| resolved_nodes.kind.dynamic())
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
            artifact_base_url: None,
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
        let exit_when_done = spawn_command
            .nodes
            .values()
            .filter(|n| !n.kind.dynamic())
            .map(|n| (spawn_command.dataflow_id, n.id.clone()))
            .collect();
        let (reply_tx, reply_rx) = oneshot::channel();
        let timestamp = clock.new_timestamp();
        let coordinator_events = stream::once(async move {
            Timestamped {
                inner: Event::Coordinator(CoordinatorEvent {
                    event: DaemonCoordinatorEvent::Spawn(spawn_command),
                    reply_tx,
                }),
                timestamp,
            }
        });
        let events = (
            coordinator_events,
            ctrlc_events,
            timeout_events,
            dynamic_node_events,
        )
            .merge();
        let run_result = Self::run_general(
            Box::pin(events),
            None,
            DaemonId::new(None),
            Some(exit_when_done),
            clock.clone(),
            None,
            if let Some(local_build) = local_build {
                let Some(build_id) = build_id else {
                    bail!("no build_id, but local_build set")
                };
                let mut builds = BTreeMap::new();
                builds.insert(build_id, local_build);
                builds
            } else {
                Default::default()
            },
            log_destination,
            health_check_interval,
        );

        let spawn_result = reply_rx
            .map_err(|err| eyre!("failed to receive spawn result: {err}"))
            .and_then(|r| async {
                match r {
                    Some(DaemonCoordinatorReply::TriggerSpawnResult(result)) => {
                        result.map_err(|err| eyre!(err))
                    }
                    _ => Err(eyre!("unexpected spawn reply")),
                }
            });

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

    #[allow(clippy::too_many_arguments)]
    async fn run_general(
        external_events: impl Stream<Item = Timestamped<Event>> + Unpin,
        coordinator_sender: Option<coordinator::CoordinatorSender>,
        daemon_id: DaemonId,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        clock: Arc<HLC>,
        remote_daemon_events_tx: Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
        builds: BTreeMap<BuildId, BuildInfo>,
        log_destination: LogDestination,
        health_check_interval_duration: Option<Duration>,
    ) -> eyre::Result<DaemonRunResult> {
        let zenoh_session = open_zenoh_session(None)
            .await
            .wrap_err("failed to open zenoh session")?;
        // Use a large channel capacity to prevent deadlock
        let (dora_events_tx, dora_events_rx) = mpsc::channel(1000);

        // Zenoh publish drain task: offloads .put().await from the main event loop.
        // The main loop sends ZenohOutbound messages via try_send; this task
        // performs the actual network I/O without blocking event processing.
        let (zenoh_publish_tx, mut zenoh_publish_rx) =
            mpsc::channel::<ZenohOutbound>(ZENOH_PUBLISH_CHANNEL_CAPACITY);
        let _zenoh_drain_handle = tokio::spawn(async move {
            while let Some(msg) = zenoh_publish_rx.recv().await {
                if let Err(e) = msg.publisher.put(msg.serialized).await {
                    tracing::error!("zenoh publish failed: {e}");
                    msg.net_publish_failures
                        .fetch_add(1, atomic::Ordering::Relaxed);
                    continue;
                }
                // Relaxed ordering is correct: counters are read-only for metrics
                // reporting and never used as synchronization guards.
                msg.net_bytes_sent
                    .fetch_add(msg.payload_len, atomic::Ordering::Relaxed);
                msg.net_messages_sent
                    .fetch_add(1, atomic::Ordering::Relaxed);
            }
            tracing::debug!("zenoh publish drain task exiting");
        });

        let daemon = Self {
            logger: Logger {
                destination: log_destination,
                daemon_id: daemon_id.clone(),
                clock: clock.clone(),
            }
            .for_daemon(daemon_id.clone()),
            running: HashMap::new(),
            working_dir: HashMap::new(),
            events_tx: dora_events_tx,
            coordinator_sender,
            last_coordinator_heartbeat: Instant::now(),
            daemon_id,
            exit_when_done,
            exit_when_all_finished: false,
            dataflow_node_results: BTreeMap::new(),
            clock,
            ft_stats: Default::default(),
            zenoh_session,
            zenoh_publish_tx,
            remote_daemon_events_tx,
            git_manager: Default::default(),
            builds,
            sessions: Default::default(),
            metrics_system: Arc::new(std::sync::Mutex::new(sysinfo::System::new())),
        };

        let dora_events = ReceiverStream::new(dora_events_rx);
        let watchdog_clock = daemon.clock.clone();
        let watchdog_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(5),
        ))
        .map(|_| Timestamped {
            inner: Event::HeartbeatInterval,
            timestamp: watchdog_clock.new_timestamp(),
        });

        let metrics_clock = daemon.clock.clone();
        let metrics_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            METRICS_INTERVAL, // Collect metrics every 2 seconds
        ))
        .map(|_| Timestamped {
            inner: Event::MetricsInterval,
            timestamp: metrics_clock.new_timestamp(),
        });

        let health_check_clock = daemon.clock.clone();
        let health_check_interval = tokio_stream::wrappers::IntervalStream::new(
            tokio::time::interval(health_check_interval_duration.unwrap_or(Duration::from_secs(5))),
        )
        .map(|_| Timestamped {
            inner: Event::NodeHealthCheckInterval,
            timestamp: health_check_clock.new_timestamp(),
        });

        let events = (
            external_events,
            dora_events,
            watchdog_interval,
            metrics_interval,
            health_check_interval,
        )
            .merge();
        daemon.run_inner(events).await
    }

    #[tracing::instrument(skip(incoming_events, self), fields(?self.daemon_id))]
    async fn run_inner(
        mut self,
        incoming_events: impl Stream<Item = Timestamped<Event>> + Unpin,
    ) -> eyre::Result<DaemonRunResult> {
        let mut events = incoming_events;

        // Send status report to coordinator so it can reconcile dataflow state.
        if let Some(sender) = &self.coordinator_sender {
            let running_dataflows: Vec<_> = self
                .running
                .iter()
                .map(
                    |(id, df)| dora_message::daemon_to_coordinator::DataflowStatusEntry {
                        dataflow_id: *id,
                        running_nodes: df.running_nodes.keys().cloned().collect(),
                    },
                )
                .collect();
            let event = DaemonEvent::StatusReport { running_dataflows };
            let stamped = Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: self.daemon_id.clone(),
                    event,
                },
                timestamp: self.clock.new_timestamp(),
            };
            if let Ok(bytes) = serde_json::to_vec(&stamped)
                && let Err(err) = sender.send_event(&bytes).await
            {
                tracing::warn!("failed to send status report to coordinator: {err}");
            }
        }

        while let Some(event) = events.next().await {
            let Timestamped { inner, timestamp } = event;
            if let Err(err) = self.clock.update_with_timestamp(&timestamp) {
                tracing::warn!("failed to update HLC with incoming event timestamp: {err}");
            }

            // used below for checking the duration of event handling
            let start = Instant::now();
            let event_kind = inner.kind();

            match inner {
                Event::Coordinator(CoordinatorEvent { event, reply_tx }) => {
                    let status = self.handle_coordinator_event(event, reply_tx).await?;

                    match status {
                        RunStatus::Continue => {}
                        RunStatus::Exit => break,
                    }
                }
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
                    if let Some(sender) = &self.coordinator_sender {
                        let msg = serde_json::to_vec(&Timestamped {
                            inner: CoordinatorRequest::Event {
                                daemon_id: self.daemon_id.clone(),
                                event: DaemonEvent::Heartbeat {
                                    ft_stats: Some(dora_message::daemon_to_coordinator::FaultToleranceSnapshot {
                                        restarts: self.ft_stats.restarts.load(atomic::Ordering::Relaxed),
                                        health_check_kills: self.ft_stats.health_check_kills.load(atomic::Ordering::Relaxed),
                                        input_timeouts: self.ft_stats.input_timeouts.load(atomic::Ordering::Relaxed),
                                        circuit_breaker_recoveries: self.ft_stats.circuit_breaker_recoveries.load(atomic::Ordering::Relaxed),
                                    }),
                                },
                            },
                            timestamp: self.clock.new_timestamp(),
                        })?;
                        sender
                            .send_event(&msg)
                            .await
                            .wrap_err("failed to send watchdog message to dora-coordinator")?;

                        if self.last_coordinator_heartbeat.elapsed() > Duration::from_secs(20) {
                            // Return error to trigger reconnection loop in run().
                            // Running dataflows continue — nodes are separate processes.
                            bail!("coordinator heartbeat timeout (20s)")
                        }
                    }
                }
                Event::MetricsInterval => {
                    self.spawn_metrics_collection();
                }
                Event::NodeHealthCheckInterval => {
                    self.check_node_health();
                    self.check_input_timeouts();
                    if self.ft_stats.any_nonzero() {
                        tracing::info!(
                            restarts = self.ft_stats.restarts.load(atomic::Ordering::Relaxed),
                            health_kills = self
                                .ft_stats
                                .health_check_kills
                                .load(atomic::Ordering::Relaxed),
                            input_timeouts =
                                self.ft_stats.input_timeouts.load(atomic::Ordering::Relaxed),
                            cb_recoveries = self
                                .ft_stats
                                .circuit_breaker_recoveries
                                .load(atomic::Ordering::Relaxed),
                            "fault tolerance stats",
                        );
                    }
                }
                Event::CtrlC => {
                    tracing::info!("received ctrlc signal -> stopping all dataflows");
                    self.trigger_manual_stop().await?;
                    if self.running.is_empty() {
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
                    if self.running.is_empty() {
                        break;
                    }
                }
                Event::DaemonError(err) => {
                    tracing::error!("Daemon error: {err:?}");
                }
                Event::SpawnNodeResult {
                    dataflow_id,
                    node_id,
                    dynamic_node,
                    result,
                } => match result {
                    Ok(running_node) => {
                        if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                            dataflow.running_nodes.insert(node_id, running_node);
                        } else {
                            tracing::error!(
                                "failed to handle SpawnNodeResult: no running dataflow with ID {dataflow_id}"
                            );
                        }
                    }
                    Err(error) => {
                        self.dataflow_node_results
                            .entry(dataflow_id)
                            .or_default()
                            .insert(node_id.clone(), Err(error));
                        self.handle_node_stop(dataflow_id, &node_id, dynamic_node)
                            .await?;
                    }
                },
                Event::BuildDataflowResult {
                    build_id,
                    session_id,
                    result,
                } => {
                    let (build_info, result) = match result {
                        Ok(build_info) => (Some(build_info), Ok(())),
                        Err(err) => (None, Err(err)),
                    };
                    if let Some(build_info) = build_info {
                        self.builds.insert(build_id, build_info);
                        if let Some(old_build_id) = self.sessions.insert(session_id, build_id) {
                            self.builds.remove(&old_build_id);
                        }
                    }
                    if let Some(sender) = &self.coordinator_sender {
                        let msg = serde_json::to_vec(&Timestamped {
                            inner: CoordinatorRequest::Event {
                                daemon_id: self.daemon_id.clone(),
                                event: DaemonEvent::BuildResult {
                                    build_id,
                                    result: result.map_err(|err| format!("{err:?}")),
                                },
                            },
                            timestamp: self.clock.new_timestamp(),
                        })?;
                        sender.send_event(&msg).await.wrap_err(
                            "failed to send BuildDataflowResult message to dora-coordinator",
                        )?;
                    }
                }
                Event::SpawnDataflowResult {
                    dataflow_id,
                    result,
                } => {
                    if let Some(sender) = &self.coordinator_sender {
                        let msg = serde_json::to_vec(&Timestamped {
                            inner: CoordinatorRequest::Event {
                                daemon_id: self.daemon_id.clone(),
                                event: DaemonEvent::SpawnResult {
                                    dataflow_id,
                                    result: result.map_err(|err| format!("{err:?}")),
                                },
                            },
                            timestamp: self.clock.new_timestamp(),
                        })?;
                        sender.send_event(&msg).await.wrap_err(
                            "failed to send SpawnDataflowResult message to dora-coordinator",
                        )?;
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
                    if self.exit_when_all_finished && self.running.is_empty() {
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

        if let Some(sender) = self.coordinator_sender.take() {
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: self.daemon_id.clone(),
                    event: DaemonEvent::Exit,
                },
                timestamp: self.clock.new_timestamp(),
            })?;
            // Best-effort: the coordinator may have already shut down (e.g. after
            // a Destroy command), so a closed channel is not an error.
            if let Err(e) = sender.send_event(&msg).await {
                tracing::debug!("could not send Exit to coordinator (already gone?): {e}");
            }
        }

        Ok(self.dataflow_node_results)
    }

    async fn trigger_manual_stop(&mut self) -> eyre::Result<()> {
        // Collect dataflow IDs that need immediate finishing
        let mut dataflows_to_finish = Vec::new();

        for dataflow in self.running.values_mut() {
            let mut logger = self.logger.for_dataflow(dataflow.id);
            let finish_when = dataflow
                .stop_all(
                    &mut self.coordinator_sender,
                    &self.clock,
                    None,
                    false,
                    &mut logger,
                )
                .await?;

            // If stop_all returns Now, we need to finish this dataflow
            if matches!(finish_when, FinishDataflowWhen::Now) {
                dataflows_to_finish.push(dataflow.id);
            }
        }

        // Finish dataflows after the loop to avoid borrow checker issues
        for dataflow_id in dataflows_to_finish {
            self.finish_dataflow(dataflow_id).await?;
        }

        self.exit_when_all_finished = true;
        Ok(())
    }

    async fn handle_coordinator_event(
        &mut self,
        event: DaemonCoordinatorEvent,
        reply_tx: Sender<Option<DaemonCoordinatorReply>>,
    ) -> eyre::Result<RunStatus> {
        let status = match event {
            DaemonCoordinatorEvent::Build(BuildDataflowNodes {
                build_id,
                session_id,
                local_working_dir,
                git_sources,
                prev_git_sources,
                dataflow_descriptor,
                nodes_on_machine,
                uv,
            }) => {
                match dataflow_descriptor.communication.remote {
                    dora_core::config::RemoteCommunicationConfig::Tcp => {}
                }

                let base_working_dir = self.base_working_dir(local_working_dir, session_id)?;

                let result = self
                    .build_dataflow(
                        build_id,
                        session_id,
                        base_working_dir,
                        git_sources,
                        prev_git_sources,
                        dataflow_descriptor,
                        nodes_on_machine,
                        uv,
                    )
                    .await;
                let (trigger_result, result_task) = match result {
                    Ok(result_task) => (Ok(()), Some(result_task)),
                    Err(err) => (Err(format!("{err:?}")), None),
                };
                let reply = DaemonCoordinatorReply::TriggerBuildResult(trigger_result);
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send `TriggerBuildResult` reply from daemon to coordinator")
                });

                let result_tx = self.events_tx.clone();
                let clock = self.clock.clone();
                if let Some(result_task) = result_task {
                    tokio::spawn(async move {
                        let message = Timestamped {
                            inner: Event::BuildDataflowResult {
                                build_id,
                                session_id,
                                result: result_task.await,
                            },
                            timestamp: clock.new_timestamp(),
                        };
                        let _ = result_tx
                            .send(message)
                            .map_err(|_| {
                                error!(
                                    "could not send `BuildResult` reply from daemon to coordinator"
                                )
                            })
                            .await;
                    });
                }

                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Spawn(SpawnDataflowNodes {
                build_id,
                session_id,
                dataflow_id,
                local_working_dir,
                nodes,
                dataflow_descriptor,
                spawn_nodes,
                uv,
                write_events_to,
                artifact_base_url: _,
            }) => {
                match dataflow_descriptor.communication.remote {
                    dora_core::config::RemoteCommunicationConfig::Tcp => {}
                }

                let base_working_dir = self.base_working_dir(local_working_dir, session_id)?;

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
                let reply = DaemonCoordinatorReply::TriggerSpawnResult(trigger_result);
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send `TriggerSpawnResult` reply from daemon to coordinator")
                });

                let result_tx = self.events_tx.clone();
                let clock = self.clock.clone();
                if let Some(result_task) = result_task {
                    tokio::spawn(async move {
                        let message = Timestamped {
                            inner: Event::SpawnDataflowResult {
                                dataflow_id,
                                result: result_task.await,
                            },
                            timestamp: clock.new_timestamp(),
                        };
                        let _ = result_tx
                            .send(message)
                            .map_err(|_| {
                                error!(
                                    "could not send `SpawnResult` reply from daemon to coordinator"
                                )
                            })
                            .await;
                    });
                }

                RunStatus::Continue
            }
            DaemonCoordinatorEvent::AllNodesReady {
                dataflow_id,
                exited_before_subscribe,
            } => {
                let mut logger = self.logger.for_dataflow(dataflow_id);
                logger.log(LogLevel::Debug, None,
                    Some("daemon".into()),
                    format!("received AllNodesReady (exited_before_subscribe: {exited_before_subscribe:?})"
                )).await;
                match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        let ready = exited_before_subscribe.is_empty();
                        dataflow
                            .pending_nodes
                            .handle_external_all_nodes_ready(
                                exited_before_subscribe,
                                &mut dataflow.cascading_error_causes,
                            )
                            .await?;
                        if ready {
                            logger.log(LogLevel::Info, None,
                                Some("daemon".into()),
                                "coordinator reported that all nodes are ready, starting dataflow",
                            ).await;
                            dataflow.start(&self.events_tx, &self.clock).await?;
                        }
                    }
                    None => {
                        tracing::warn!(
                            "received AllNodesReady for unknown dataflow (ID `{dataflow_id}`)"
                        );
                    }
                }
                let _ = reply_tx.send(None).map_err(|_| {
                    error!("could not send `AllNodesReady` reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Logs {
                dataflow_id,
                node_id,
                tail,
            } => {
                match self.working_dir.get(&dataflow_id) {
                    Some(working_dir) => {
                        let working_dir = working_dir.clone();
                        tokio::spawn(async move {
                            let logs = async {
                                let mut file =
                                    File::open(log::log_path(&working_dir, &dataflow_id, &node_id))
                                        .await
                                        .wrap_err(format!(
                                            "Could not open log file: {:#?}",
                                            log::log_path(&working_dir, &dataflow_id, &node_id)
                                        ))?;

                                let mut contents = match tail {
                                    None | Some(0) => {
                                        let mut contents = vec![];
                                        file.read_to_end(&mut contents).await.map(|_| contents)
                                    }
                                    Some(tail) => read_last_n_lines(&mut file, tail).await,
                                }
                                .wrap_err("Could not read last n lines of log file")?;
                                if !contents.ends_with(b"\n") {
                                    // Append newline for better readability
                                    contents.push(b'\n');
                                }
                                Result::<Vec<u8>, eyre::Report>::Ok(contents)
                            }
                            .await
                            .map_err(|err| format!("{err:?}"));
                            let _ = reply_tx
                                .send(Some(DaemonCoordinatorReply::Logs(logs)))
                                .map_err(|_| {
                                    error!("could not send logs reply from daemon to coordinator")
                                });
                        });
                    }
                    None => {
                        tracing::warn!("received Logs for unknown dataflow (ID `{dataflow_id}`)");
                        let _ = reply_tx.send(None).map_err(|_| {
                            error!(
                                "could not send `AllNodesReady` reply from daemon to coordinator"
                            )
                        });
                    }
                }
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::ReloadDataflow {
                dataflow_id,
                node_id,
                operator_id,
            } => {
                let result = self.send_reload(dataflow_id, node_id, operator_id).await;
                let reply =
                    DaemonCoordinatorReply::ReloadResult(result.map_err(|err| format!("{err:?}")));
                let _ = reply_tx
                    .send(Some(reply))
                    .map_err(|_| error!("could not send reload reply from daemon to coordinator"));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::RestartNode {
                dataflow_id,
                node_id,
                grace_duration,
            } => {
                let result = match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        dataflow.restart_single_node(&node_id, &self.clock, grace_duration)
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::RestartNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send restart node reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StopNode {
                dataflow_id,
                node_id,
                grace_duration,
            } => {
                let result = match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        dataflow.stop_single_node(&node_id, &self.clock, grace_duration)
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::StopNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send stop node reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::SetParam {
                dataflow_id,
                node_id,
                key,
                value,
            } => {
                let result = match self.running.get(&dataflow_id) {
                    Some(dataflow) => {
                        if let Some(channel) = dataflow.subscribe_channels.get(&node_id) {
                            match send_with_timestamp(
                                channel,
                                NodeEvent::ParamUpdate { key, value },
                                &self.clock,
                            ) {
                                Ok(true) => {
                                    dataflow.inc_pending(&node_id);
                                    Ok(())
                                }
                                Ok(false) => Ok(()), // event dropped (channel full)
                                Err(_) => Err(eyre::eyre!("node `{node_id}` channel closed")),
                            }
                        } else {
                            tracing::debug!(
                                %node_id,
                                "param update not deliverable: node not yet connected"
                            );
                            Ok(())
                        }
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::SetParamResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send set param reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::DeleteParam {
                dataflow_id,
                node_id,
                key,
            } => {
                let result = match self.running.get(&dataflow_id) {
                    Some(dataflow) => {
                        if let Some(channel) = dataflow.subscribe_channels.get(&node_id) {
                            match send_with_timestamp(
                                channel,
                                NodeEvent::ParamDeleted { key },
                                &self.clock,
                            ) {
                                Ok(true) => {
                                    dataflow.inc_pending(&node_id);
                                    Ok(())
                                }
                                Ok(false) => Ok(()), // event dropped (channel full)
                                Err(_) => Err(eyre::eyre!("node `{node_id}` channel closed")),
                            }
                        } else {
                            tracing::debug!(
                                %node_id,
                                "param delete not deliverable: node not yet connected"
                            );
                            Ok(())
                        }
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::DeleteParamResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send delete param reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StopDataflow {
                dataflow_id,
                grace_duration,
                force,
            } => {
                let finish_when = {
                    let mut logger = self.logger.for_dataflow(dataflow_id);
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"));
                    let (reply, future) = match dataflow {
                        Ok(dataflow) => {
                            let future = dataflow.stop_all(
                                &mut self.coordinator_sender,
                                &self.clock,
                                grace_duration,
                                force,
                                &mut logger,
                            );
                            (Ok(()), Some(future))
                        }
                        Err(err) => (Err(err.to_string()), None),
                    };

                    let _ = reply_tx
                        .send(Some(DaemonCoordinatorReply::StopResult(reply)))
                        .map_err(|_| {
                            error!("could not send stop reply from daemon to coordinator")
                        });

                    if let Some(future) = future {
                        Some(future.await?)
                    } else {
                        None
                    }
                };

                // If stop_all returns Now, finish the dataflow immediately
                if matches!(finish_when, Some(FinishDataflowWhen::Now)) {
                    self.finish_dataflow(dataflow_id).await?;
                }

                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Destroy => {
                tracing::info!("received destroy command -> exiting");
                let (notify_tx, notify_rx) = oneshot::channel();
                let reply = DaemonCoordinatorReply::DestroyResult {
                    result: Ok(()),
                    notify: Some(notify_tx),
                };
                let _ = reply_tx
                    .send(Some(reply))
                    .map_err(|_| error!("could not send destroy reply from daemon to coordinator"));
                // wait until the reply is sent out
                if notify_rx.await.is_err() {
                    tracing::warn!("no confirmation received for DestroyReply");
                }
                RunStatus::Exit
            }
            DaemonCoordinatorEvent::Heartbeat => {
                self.last_coordinator_heartbeat = Instant::now();
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::PeerDaemonDisconnected { daemon_id } => {
                tracing::warn!(%daemon_id, "peer daemon disconnected");
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            // --- Dynamic Topology ---
            DaemonCoordinatorEvent::AddNode {
                dataflow_id,
                node,
                uv,
            } => {
                let node_id = node.id.clone();
                tracing::info!(%dataflow_id, %node_id, "adding node to running dataflow");

                let result: eyre::Result<()> = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
                    let base_working_dir = self
                        .working_dir
                        .get(&dataflow_id)
                        .cloned()
                        .unwrap_or_else(|| std::path::PathBuf::from("."));

                    // Collect input metadata for post-spawn registration.
                    // State mutations are DEFERRED until after the spawn
                    // succeeds so a spawn failure doesn't leave stale
                    // routing/deadline/pending state behind.
                    let inputs = node_inputs(&node);
                    let is_dynamic = node.kind.dynamic();

                    // Prepare stderr buffer (harmless — just an empty
                    // ArrayQueue, no routing implications).
                    let node_stderr = dataflow
                        .node_stderr_most_recent
                        .entry(node_id.clone())
                        .or_insert_with(|| Arc::new(ArrayQueue::new(STDERR_LOG_LINES_MAX)))
                        .clone();

                    // --- Spawn the node (before any routing state is touched) ---
                    let descriptor = dataflow.descriptor.clone();
                    let spawner = Spawner {
                        dataflow_id,
                        daemon_tx: self.events_tx.clone(),
                        dataflow_descriptor: descriptor,
                        clock: self.clock.clone(),
                        uv,
                        ft_stats: self.ft_stats.clone(),
                        shutdown: dataflow.listener_shutdown_rx.clone(),
                    };
                    let mut logger = self
                        .logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .try_clone()
                        .await
                        .context("failed to clone logger")?;
                    let task = spawner
                        .spawn_node(
                            node.clone(),
                            base_working_dir,
                            node_stderr,
                            None,
                            &mut logger,
                        )
                        .await
                        .wrap_err("failed to prepare node")?;
                    let prepared = task.await.wrap_err("failed to build node")?;
                    let running_node = prepared
                        .spawn(logger)
                        .await
                        .wrap_err("failed to spawn node")?;

                    // --- Spawn succeeded — now apply state mutations ---
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("dataflow disappeared during spawn"))?;

                    // Register inputs (open_inputs, mappings, timers, deadlines)
                    for (input_id, input) in &inputs {
                        dataflow
                            .open_inputs
                            .entry(node_id.clone())
                            .or_default()
                            .insert(input_id.clone());
                        match &input.mapping {
                            InputMapping::User(mapping) => {
                                if let Some(timeout) = input.input_timeout {
                                    dataflow.input_deadlines.insert(
                                        (node_id.clone(), input_id.clone()),
                                        InputDeadline {
                                            timeout: Duration::from_secs_f64(timeout),
                                            last_received: None,
                                        },
                                    );
                                }
                                dataflow
                                    .mappings
                                    .entry(OutputId(mapping.source.clone(), mapping.output.clone()))
                                    .or_default()
                                    .insert((node_id.clone(), input_id.clone()));
                            }
                            InputMapping::Timer { interval } => {
                                dataflow
                                    .timers
                                    .entry(*interval)
                                    .or_default()
                                    .insert((node_id.clone(), input_id.clone()));
                            }
                            InputMapping::Logs(filter) => {
                                dataflow.log_subscribers.push(
                                    crate::running_dataflow::LogSubscriber {
                                        node_id: node_id.clone(),
                                        input_id: input_id.clone(),
                                        filter: filter.clone(),
                                    },
                                );
                            }
                        }
                    }

                    // Mark as pending
                    if is_dynamic {
                        dataflow.dynamic_nodes.insert(node_id.clone());
                    } else {
                        dataflow.pending_nodes.insert(node_id.clone());
                    }

                    // Insert the running node
                    dataflow.running_nodes.insert(node_id.clone(), running_node);

                    // Update the daemon's stored descriptor so
                    // descriptor-based lookups (e.g. AllInputsClosed
                    // check at handle_outputs_done) find the new node.
                    // Construct a minimal Node from the inputs we
                    // already collected — only `id` and `inputs` are
                    // consulted by the daemon.
                    dataflow
                        .descriptor
                        .nodes
                        .push(dora_message::descriptor::Node {
                            id: node_id.clone(),
                            name: None,
                            description: None,
                            path: None,
                            args: None,
                            env: None,
                            operators: None,
                            operator: None,
                            ros2: None,
                            custom: None,
                            outputs: Default::default(),
                            output_types: Default::default(),
                            output_framing: Default::default(),
                            inputs: inputs.into_iter().collect(),
                            input_types: Default::default(),
                            output_metadata: Default::default(),
                            pattern: None,
                            send_stdout_as: None,
                            send_logs_as: None,
                            min_log_level: None,
                            max_log_size: None,
                            max_rotated_files: None,
                            build: None,
                            git: None,
                            branch: None,
                            tag: None,
                            rev: None,
                            restart_policy: Default::default(),
                            max_restarts: 0,
                            restart_delay: None,
                            max_restart_delay: None,
                            restart_window: None,
                            health_check_timeout: None,
                            module: None,
                            params: Default::default(),
                            cpu_affinity: None,
                            deploy: None,
                        });

                    tracing::info!(
                        %dataflow_id,
                        %node_id,
                        dynamic = is_dynamic,
                        "node added successfully"
                    );
                    Ok(())
                }
                .await;

                if let Err(err) = &result {
                    tracing::error!(%dataflow_id, %node_id, "AddNode failed: {err:?}");
                }
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::RemoveNode {
                dataflow_id,
                node_id,
                grace_duration,
            } => {
                tracing::info!(%dataflow_id, %node_id, "removing node from running dataflow");
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    let _ = dataflow.stop_single_node(&node_id, &self.clock, grace_duration);

                    // Clean up routing tables: remove all mappings where this
                    // node is a source, and close inputs on downstream nodes.
                    let outputs_to_remove: Vec<OutputId> = dataflow
                        .mappings
                        .keys()
                        .filter(|oid| oid.0 == node_id)
                        .cloned()
                        .collect();
                    for output_id in outputs_to_remove {
                        if let Some(receivers) = dataflow.mappings.remove(&output_id) {
                            for (receiver_id, input_id) in receivers {
                                close_input(dataflow, &receiver_id, &input_id, &self.clock);
                            }
                        }
                    }

                    // Remove all mappings where this node is a receiver.
                    for receivers in dataflow.mappings.values_mut() {
                        receivers.retain(|(nid, _)| nid != &node_id);
                    }

                    // Clean up remaining state for this node.
                    dataflow.running_nodes.remove(&node_id);
                    dataflow.open_inputs.remove(&node_id);
                    dataflow.subscribe_channels.remove(&node_id);
                    dataflow.drop_channels.remove(&node_id);
                    dataflow.pending_messages.remove(&node_id);

                    // Remove from stored descriptor (inverse of AddNode
                    // push) so descriptor-based lookups stay consistent.
                    dataflow.descriptor.nodes.retain(|n| n.id != node_id);
                }
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::AddMapping {
                dataflow_id,
                source_node,
                source_output,
                target_node,
                target_input,
            } => {
                tracing::info!(%dataflow_id, "{source_node}/{source_output} -> {target_node}/{target_input}");
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    let output_id = OutputId(source_node, source_output);
                    dataflow
                        .mappings
                        .entry(output_id)
                        .or_default()
                        .insert((target_node.clone(), target_input.clone()));
                    dataflow
                        .open_inputs
                        .entry(target_node)
                        .or_default()
                        .insert(target_input);
                }
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::RemoveMapping {
                dataflow_id,
                source_node,
                source_output,
                target_node,
                target_input,
            } => {
                tracing::info!(%dataflow_id, "{source_node}/{source_output} -x- {target_node}/{target_input}");
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    let output_id = OutputId(source_node, source_output);
                    if let Some(receivers) = dataflow.mappings.get_mut(&output_id)
                        && receivers.remove(&(target_node.clone(), target_input.clone()))
                    {
                        close_input(dataflow, &target_node, &target_input, &self.clock);
                    }
                }
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StateCatchUp {
                dataflow_id,
                entries,
            } => {
                let max_seq = entries.last().map(|e| e.sequence).unwrap_or(0);
                tracing::info!(
                    %dataflow_id,
                    "state catch-up: applying {} entry(ies) (up to seq {max_seq})",
                    entries.len(),
                );
                for entry in &entries {
                    match &entry.operation {
                        StateCatchUpOperation::SetParam {
                            node_id,
                            key,
                            value,
                        } => {
                            if let Some(dataflow) = self.running.get(&dataflow_id)
                                && let Some(channel) = dataflow.subscribe_channels.get(node_id)
                            {
                                match send_with_timestamp(
                                    channel,
                                    NodeEvent::ParamUpdate {
                                        key: key.clone(),
                                        value: value.clone(),
                                    },
                                    &self.clock,
                                ) {
                                    Ok(true) => dataflow.inc_pending(node_id),
                                    Ok(false) => {} // dropped (channel full)
                                    Err(_) => {
                                        tracing::warn!("catch-up: node `{node_id}` channel closed")
                                    }
                                }
                            }
                        }
                        StateCatchUpOperation::DeleteParam { node_id, key } => {
                            if let Some(dataflow) = self.running.get(&dataflow_id)
                                && let Some(channel) = dataflow.subscribe_channels.get(node_id)
                            {
                                match send_with_timestamp(
                                    channel,
                                    NodeEvent::ParamDeleted { key: key.clone() },
                                    &self.clock,
                                ) {
                                    Ok(true) => dataflow.inc_pending(node_id),
                                    Ok(false) => {}
                                    Err(_) => {
                                        tracing::warn!("catch-up: node `{node_id}` channel closed")
                                    }
                                }
                            }
                        }
                    }
                }
                // Send ack back to coordinator so it can prune the log.
                if max_seq > 0
                    && let Some(sender) = &self.coordinator_sender
                {
                    let ack = DaemonEvent::StateCatchUpAck {
                        dataflow_id,
                        ack_sequence: max_seq,
                    };
                    let stamped = Timestamped {
                        inner: CoordinatorRequest::Event {
                            daemon_id: self.daemon_id.clone(),
                            event: ack,
                        },
                        timestamp: self.clock.new_timestamp(),
                    };
                    if let Ok(bytes) = serde_json::to_vec(&stamped)
                        && let Err(err) = sender.send_event(&bytes).await
                    {
                        tracing::warn!("failed to send state catch-up ack to coordinator: {err}");
                    }
                }
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
        };
        Ok(status)
    }

    fn check_node_health(&self) {
        let now_millis = node_communication::current_millis();
        for dataflow in self.running.values() {
            for (node_id, node) in &dataflow.running_nodes {
                let Some(timeout) = node.health_check_timeout else {
                    continue;
                };
                let last = node.last_activity.load(atomic::Ordering::Acquire);
                if last == 0 {
                    continue; // not yet connected
                }
                let elapsed_ms = now_millis.saturating_sub(last);
                let timeout_ms = timeout.as_millis() as u64;
                if elapsed_ms > timeout_ms {
                    tracing::warn!(
                        "node `{node_id}` unresponsive for {}ms (timeout: {timeout:?}), killing",
                        elapsed_ms,
                    );
                    self.ft_stats
                        .health_check_kills
                        .fetch_add(1, atomic::Ordering::Relaxed);
                    if let Some(process) = &node.process {
                        process.submit(ProcessOperation::Kill);
                    }
                }
            }
        }
    }

    fn check_input_timeouts(&mut self) {
        let clock = self.clock.clone();
        for dataflow in self.running.values_mut() {
            let mut timed_out = Vec::new();
            for ((node_id, input_id), deadline) in &dataflow.input_deadlines {
                // Skip inputs already tracked as broken (avoids duplicate warnings)
                if dataflow
                    .broken_inputs
                    .contains_key(&(node_id.clone(), input_id.clone()))
                {
                    continue;
                }
                // Only count elapsed time once the input has actually
                // received a message. Inputs that never saw traffic are
                // considered "not yet armed" — see InputDeadline::is_timed_out
                // (dora-rs/adora#149).
                if deadline.is_timed_out() {
                    timed_out.push((node_id.clone(), input_id.clone(), deadline.timeout));
                }
            }
            for (node_id, input_id, timeout) in &timed_out {
                tracing::warn!(
                    "input `{node_id}/{input_id}` timed out after {timeout:?}, \
                     closing (circuit breaker armed)",
                );
                self.ft_stats
                    .input_timeouts
                    .fetch_add(1, atomic::Ordering::Relaxed);
                dataflow
                    .broken_inputs
                    .insert((node_id.clone(), input_id.clone()), *timeout);
                break_input(dataflow, node_id, input_id, &clock);
            }
            for (node_id, input_id, _) in timed_out {
                dataflow.input_deadlines.remove(&(node_id, input_id));
            }
        }
    }

    /// Snapshot running dataflow state and spawn metrics collection as a
    /// background task so it never blocks the event loop.
    fn spawn_metrics_collection(&self) {
        let sender = match &self.coordinator_sender {
            Some(s) => s.clone(),
            None => return,
        };

        // Snapshot per-dataflow data needed for metrics.
        let dataflow_snapshots: Vec<DataflowMetricsSnapshot> = self
            .running
            .iter()
            .map(|(id, df)| DataflowMetricsSnapshot {
                dataflow_id: *id,
                nodes: df
                    .running_nodes
                    .iter()
                    .map(|(nid, rn)| NodeSnapshot {
                        node_id: nid.clone(),
                        pid: rn.pid.clone(),
                        restart_count: rn.restart_count.clone(),
                        restarts_disabled: rn.restarts_disabled(),
                        broken_inputs: df
                            .broken_inputs
                            .keys()
                            .filter(|(n, _)| n == nid)
                            .map(|(_, d)| d.to_string())
                            .collect(),
                        pending_messages: df
                            .pending_messages
                            .get(nid)
                            .map(|c| c.load(atomic::Ordering::Relaxed))
                            .unwrap_or(0),
                    })
                    .collect(),
                net_bytes_sent: df.net_bytes_sent.clone(),
                net_bytes_received: df.net_bytes_received.clone(),
                net_messages_sent: df.net_messages_sent.clone(),
                net_messages_received: df.net_messages_received.clone(),
                net_publish_failures: df.net_publish_failures.clone(),
            })
            .collect();

        let metrics_system = self.metrics_system.clone();
        let daemon_id = self.daemon_id.clone();
        let clock = self.clock.clone();

        tokio::spawn(async move {
            if let Err(e) = collect_and_send_metrics_bg(
                dataflow_snapshots,
                metrics_system,
                sender,
                daemon_id,
                clock,
            )
            .await
            {
                tracing::warn!("metrics collection failed: {e}");
            }
        });
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
                    let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                        format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                    })?;
                    send_output_to_local_receivers(
                        node_id.clone(),
                        output_id.clone(),
                        dataflow,
                        &metadata,
                        data.map(DataMessage::Vec),
                        &self.clock,
                        Some(&self.ft_stats),
                        false, // WS topic publish: no Zenoh forwarding
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

                let inner = async {
                    let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                        format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                    })?;

                    if let Some(inputs) = dataflow.mappings.get(&output_id).cloned() {
                        for (receiver_id, input_id) in &inputs {
                            close_input(dataflow, receiver_id, input_id, &self.clock);
                        }
                    }
                    Result::<(), eyre::Report>::Ok(())
                };
                if let Err(err) = inner
                    .await
                    .wrap_err("failed to handle InputsClosed event sent by coordinator")
                {
                    logger
                        .log(LogLevel::Warn, Some("daemon".into()), format!("{err:?}"))
                        .await;
                }
                Ok(())
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    async fn build_dataflow(
        &mut self,
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
        self.git_manager.clear_planned_builds(session_id);

        let nodes = dataflow_descriptor.resolve_aliases_and_set_defaults()?;

        let mut tasks = Vec::new();

        // build nodes
        for node in nodes.into_values().filter(|n| local_nodes.contains(&n.id)) {
            let dynamic_node = node.kind.dynamic();

            let node_id = node.id.clone();
            let mut logger = self.logger.for_node_build(build_id, node_id.clone());
            logger.log(LogLevel::Debug, "building").await;
            let git_source = git_sources.get(&node_id).cloned();
            let prev_git_source = prev_git_sources.get(&node_id).cloned();
            let prev_git = prev_git_source.map(|prev_source| PrevGitSource {
                still_needed_for_this_build: git_sources.values().any(|s| s == &prev_source),
                git_source: prev_source,
            });

            let logger_cloned = logger
                .try_clone_impl()
                .await
                .wrap_err("failed to clone logger")?;

            let mut builder = builder.clone();
            if let Some(node_working_dir) =
                node.deploy.as_ref().and_then(|d| d.working_dir.as_deref())
            {
                builder.base_working_dir = builder.base_working_dir.join(node_working_dir);
            }

            match builder
                .build_node(
                    node,
                    git_source,
                    prev_git,
                    logger_cloned,
                    &mut self.git_manager,
                )
                .await
                .wrap_err_with(|| format!("failed to build node `{node_id}`"))
            {
                Ok(result) => {
                    tasks.push(NodeBuildTask {
                        node_id,
                        task: result,
                        dynamic_node,
                    });
                }
                Err(err) => {
                    logger.log(LogLevel::Error, format!("{err:?}")).await;
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
            self.daemon_id.clone(),
            dataflow_descriptor.clone(),
        );
        let dataflow = match self.running.entry(dataflow_id) {
            std::collections::hash_map::Entry::Vacant(entry) => {
                self.working_dir
                    .insert(dataflow_id, base_working_dir.clone());
                entry.insert(dataflow)
            }
            std::collections::hash_map::Entry::Occupied(_) => {
                bail!("there is already a running dataflow with ID `{dataflow_id}`")
            }
        };

        let mut stopped = Vec::new();

        let build_info = build_id.and_then(|build_id| self.builds.get(&build_id));
        let node_with_git_source = nodes.values().find(|n| n.has_git_source());
        if let Some(git_node) = node_with_git_source
            && build_info.is_none()
        {
            eyre::bail!(
                "node {} has git source, but no `dora build` was run yet\n\n\
                    nodes with a `git` field must be built using `dora build` before starting the \
                    dataflow",
                git_node.id
            )
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
                            if let Some(timeout) = input.input_timeout {
                                dataflow.input_deadlines.insert(
                                    (node.id.clone(), input_id.clone()),
                                    InputDeadline {
                                        timeout: Duration::from_secs_f64(timeout),
                                        // Unarmed until the first message
                                        // arrives — see issue #149.
                                        last_received: None,
                                    },
                                );
                            }
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
                        InputMapping::Logs(filter) => {
                            dataflow
                                .log_subscribers
                                .push(crate::running_dataflow::LogSubscriber {
                                    node_id: node.id.clone(),
                                    input_id,
                                    filter,
                                });
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
            daemon_tx: self.events_tx.clone(),
            dataflow_descriptor,
            clock: self.clock.clone(),
            uv,
            ft_stats: self.ft_stats.clone(),
            shutdown: dataflow.listener_shutdown_rx.clone(),
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
                        self.dataflow_node_results
                            .entry(dataflow_id)
                            .or_default()
                            .insert(
                                node_id.clone(),
                                Err(NodeError {
                                    timestamp: self.clock.new_timestamp(),
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
                        .remote_daemon_events_tx
                        .clone()
                        .wrap_err("no remote_daemon_events_tx channel")?;
                    let mut finished_rx = dataflow.finished_tx.subscribe();
                    let subscribe_topic =
                        zenoh_output_publish_topic(dataflow.id, &output_id.0, &output_id.1);
                    tracing::debug!("declaring subscriber on {subscribe_topic}");
                    let subscriber = self
                        .zenoh_session
                        .declare_subscriber(subscribe_topic)
                        .await
                        .map_err(|e| eyre!(e))
                        .wrap_err_with(|| format!("failed to subscribe to {output_id:?}"))?;
                    let net_bytes_rx = dataflow.net_bytes_received.clone();
                    let net_msgs_rx = dataflow.net_messages_received.clone();
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
                                        let bytes = s.payload().to_bytes();
                                        net_bytes_rx.fetch_add(
                                            bytes.len() as u64,
                                            std::sync::atomic::Ordering::Relaxed,
                                        );
                                        net_msgs_rx
                                            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                        Timestamped::deserialize_inter_daemon_event(&bytes)
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
        for (node_id, dynamic) in stopped {
            self.handle_node_stop(dataflow_id, &node_id, dynamic)
                .await?;
        }

        let spawn_result = Self::spawn_prepared_nodes(
            dataflow_id,
            logger,
            tasks,
            self.events_tx.clone(),
            self.clock.clone(),
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
                    .running
                    .iter()
                    .filter(|(_id, dataflow)| dataflow.running_nodes.contains_key(&node_id))
                    .count();

                let node_config = match number_node_id {
                    2.. => Err(format!(
                        "multiple dataflows contain dynamic node id {node_id}. \
                        Please only have one running dataflow with the specified \
                        node id if you want to use dynamic node",
                    )),
                    1 => self
                        .running
                        .iter()
                        .filter(|(_id, dataflow)| dataflow.running_nodes.contains_key(&node_id))
                        .map(|(id, dataflow)| -> Result<NodeConfig> {
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
            let dataflow = self.running.get(&dataflow_id)?;
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
                pending_counter,
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

                let dataflow = self.running.get_mut(&dataflow_id).ok_or_else(|| {
                    format!("subscribe failed: no running dataflow with ID `{dataflow_id}`")
                });

                match dataflow {
                    Err(err) => {
                        let _ = reply_sender.send(DaemonReply::Result(Err(err)));
                    }
                    Ok(dataflow) => {
                        dataflow
                            .pending_messages
                            .insert(node_id.clone(), pending_counter);
                        Self::subscribe(dataflow, node_id.clone(), event_sender, &self.clock).await;

                        let status = dataflow
                            .pending_nodes
                            .handle_node_subscription(
                                node_id.clone(),
                                reply_sender,
                                &mut self.coordinator_sender,
                                &self.clock,
                                &mut dataflow.cascading_error_causes,
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
                                dataflow.start(&self.events_tx, &self.clock).await?;
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
                let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!("failed to subscribe: no running dataflow with ID `{dataflow_id}`")
                });
                let result = match dataflow {
                    Ok(dataflow) => {
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
                let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!(
                        "failed to get handle drop tokens: \
                        no running dataflow with ID `{dataflow_id}`"
                    )
                });

                match dataflow {
                    Ok(dataflow) => {
                        for token in tokens {
                            match dataflow.pending_drop_tokens.get_mut(&token) {
                                Some(info) => {
                                    if info.pending_nodes.remove(&node_id) {
                                        dataflow.check_drop_token(token, &self.clock).await?;
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
                let inner = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
                    dataflow.subscribe_channels.remove(&node_id);
                    Result::<_, eyre::Error>::Ok(())
                };

                let reply = inner.await.map_err(|err| format!("{err:?}"));
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
        }
        Ok(())
    }

    async fn send_reload(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<(), eyre::ErrReport> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("Reload failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        if let Some(channel) = dataflow.subscribe_channels.get(&node_id) {
            match send_with_timestamp(channel, NodeEvent::Reload { operator_id }, &self.clock) {
                Ok(true) => {
                    dataflow.inc_pending(&node_id);
                }
                Ok(false) => { /* event dropped (channel full) */ }
                Err(_) => {
                    dataflow.subscribe_channels.remove(&node_id);
                }
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
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        let output_id_key = OutputId(node_id.clone(), output_id.clone());
        let remote_receivers = dataflow.open_external_mappings.contains(&output_id_key)
            || dataflow.publish_all_messages_to_zenoh;
        let data_bytes = send_output_to_local_receivers(
            node_id.clone(),
            output_id.clone(),
            dataflow,
            &metadata,
            data,
            &self.clock,
            Some(&self.ft_stats),
            remote_receivers,
        )
        .await?;

        let output_id = output_id_key;
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

    async fn send_to_remote_receivers(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        event: InterDaemonEvent,
    ) -> Result<(), eyre::Error> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;

        // Get or create publisher (lazy, cached per output)
        let publisher = match dataflow.publishers.entry(output_id.clone()) {
            std::collections::btree_map::Entry::Occupied(e) => e.get().clone(),
            std::collections::btree_map::Entry::Vacant(e) => {
                let publish_topic =
                    zenoh_output_publish_topic(dataflow.id, &output_id.0, &output_id.1);
                tracing::debug!("declaring publisher on {publish_topic}");
                let publisher = self
                    .zenoh_session
                    .declare_publisher(publish_topic)
                    .await
                    .map_err(|err| eyre!(err))
                    .context("failed to create zenoh publisher")?;
                let arc = Arc::new(publisher);
                e.insert(arc.clone());
                arc
            }
        };

        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        }
        .serialize()
        .wrap_err("failed to serialize inter-daemon event")?;
        let payload_len = serialized_event.len() as u64;

        // Offload Zenoh I/O to the drain task — never blocks the event loop.
        let outbound = ZenohOutbound {
            publisher,
            serialized: serialized_event,
            payload_len,
            net_bytes_sent: dataflow.net_bytes_sent.clone(),
            net_messages_sent: dataflow.net_messages_sent.clone(),
            net_publish_failures: dataflow.net_publish_failures.clone(),
        };
        match self.zenoh_publish_tx.try_send(outbound) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(_)) => {
                tracing::warn!(
                    "zenoh publish channel full ({ZENOH_PUBLISH_CHANNEL_CAPACITY}), \
                     dropping inter-daemon message"
                );
            }
            Err(mpsc::error::TrySendError::Closed(_)) => {
                tracing::error!("zenoh drain task is gone — inter-daemon publish channel closed");
            }
        }

        Ok(())
    }

    async fn send_output_closed_events(
        &mut self,
        dataflow_id: DataflowId,
        node_id: NodeId,
        outputs: Vec<DataId>,
    ) -> eyre::Result<()> {
        let dataflow = self
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
            close_input(dataflow, receiver_id, input_id, &self.clock);
        }

        let mut closed = Vec::new();
        for output_id in &dataflow.open_external_mappings {
            if output_id.0 == node_id && outputs.contains(&output_id.1) {
                closed.push(output_id.clone());
            }
        }

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
        event_sender: mpsc::Sender<Timestamped<NodeEvent>>,
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
            if send_with_timestamp(
                &event_sender,
                NodeEvent::InputClosed {
                    id: input_id.clone(),
                },
                clock,
            )
            .ok()
                == Some(true)
            {
                dataflow.inc_pending(&node_id);
            }
        }
        if dataflow.open_inputs(&node_id).is_empty() {
            if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                node.disable_restart();
            }
            if let Some(node) = dataflow.descriptor.nodes.iter().find(|n| n.id == node_id) {
                if node.inputs.is_empty() {
                    // do not send AllInputsClosed for source nodes
                } else if send_with_timestamp(&event_sender, NodeEvent::AllInputsClosed, clock).ok()
                    == Some(true)
                {
                    dataflow.inc_pending(&node_id);
                }
            }
        }

        // if a stop event was already sent for the dataflow, send it to
        // the newly connected node too
        if dataflow.stop_sent {
            if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                node.disable_restart();
            }
            if send_with_timestamp(&event_sender, NodeEvent::Stop, clock).ok() == Some(true) {
                dataflow.inc_pending(&node_id);
            }
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
        let dataflow = self
            .running
            .get_mut(&dataflow_id)
            .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;

        let outputs = dataflow
            .mappings
            .keys()
            .filter(|m| &m.0 == node_id)
            .map(|m| &m.1)
            .cloned()
            .collect();

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

        let dataflow = self
            .running
            .get_mut(&dataflow_id)
            .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
        dataflow.drop_channels.remove(node_id);
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
        let _ = self
            .events_tx
            .send(Timestamped {
                inner: Event::NodeStopped {
                    dataflow_id,
                    node_id: node_id.clone(),
                },
                timestamp: self.clock.new_timestamp(),
            })
            .await;
        result
    }

    async fn handle_node_stop_inner(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
        dynamic_node: bool,
    ) -> eyre::Result<()> {
        let mut logger = self.logger.for_dataflow(dataflow_id);
        let dataflow = match self.running.get_mut(&dataflow_id) {
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

        dataflow
            .pending_nodes
            .handle_node_stop(
                node_id,
                &mut self.coordinator_sender,
                &self.clock,
                &mut dataflow.cascading_error_causes,
                &mut logger,
            )
            .await?;

        // node only reaches here if it will not be restarted
        let might_restart = false;

        self.handle_outputs_done(dataflow_id, node_id, might_restart)
            .await?;

        let should_finish = {
            let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
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
            timestamp: self.clock.new_timestamp(),
            node_results: self
                .dataflow_node_results
                .get(&dataflow_id)
                .cloned()
                .unwrap_or_default(),
        };

        self.git_manager
            .clones_in_use
            .values_mut()
            .for_each(|dataflows| {
                dataflows.remove(&dataflow_id);
            });

        logger
            .log(
                LogLevel::Info,
                None,
                Some("daemon".into()),
                format!("dataflow finished on machine `{}`", self.daemon_id),
            )
            .await;

        if let Some(sender) = &self.coordinator_sender {
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: self.daemon_id.clone(),
                    event: DaemonEvent::AllNodesFinished {
                        dataflow_id,
                        result,
                    },
                },
                timestamp: self.clock.new_timestamp(),
            })?;
            sender
                .send_event(&msg)
                .await
                .wrap_err("failed to report dataflow finish to dora-coordinator")?;
        }
        // Signal all listener loops for this dataflow to shut down
        if let Some(df) = self.running.get(&dataflow_id) {
            let _ = df.listener_shutdown_tx.send(true);
        }
        self.running.remove(&dataflow_id);

        Ok(())
    }

    async fn handle_dora_event(&mut self, event: DoraEvent) -> eyre::Result<()> {
        match event {
            DoraEvent::Timer {
                dataflow_id,
                interval,
                metadata,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Timer event for unknown dataflow `{dataflow_id}`");
                    return Ok(());
                };

                let Some(subscribers) = dataflow.timers.get(&interval) else {
                    return Ok(());
                };

                let mut closed = Vec::new();
                for (receiver_id, input_id) in subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        continue;
                    };

                    let send_result = send_with_timestamp(
                        channel,
                        NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: Arc::new(metadata.clone()),
                            data: None,
                        },
                        &self.clock,
                    );
                    match send_result {
                        Ok(true) => {
                            dataflow.inc_pending(receiver_id);
                        }
                        Ok(false) => { /* event dropped (channel full) */ }
                        Err(_) => {
                            closed.push(receiver_id);
                        }
                    }
                }
                for id in closed {
                    dataflow.subscribe_channels.remove(id);
                }
            }
            DoraEvent::Logs {
                dataflow_id,
                output_id,
                message,
                metadata,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Logs event for unknown dataflow `{dataflow_id}`");
                    return Ok(());
                };

                let Some(subscribers) = dataflow.mappings.get(&output_id) else {
                    tracing::warn!(
                        "No subscribers found for {:?} in {:?}",
                        output_id,
                        dataflow.mappings
                    );
                    return Ok(());
                };

                let mut closed = Vec::new();
                for (receiver_id, input_id) in subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        tracing::warn!("No subscriber channel found for {:?}", output_id);
                        continue;
                    };

                    let send_result = send_with_timestamp(
                        channel,
                        NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: Arc::new(metadata.clone()),
                            data: Some(Arc::new(message.clone())),
                        },
                        &self.clock,
                    );
                    match send_result {
                        Ok(true) => {
                            dataflow.inc_pending(receiver_id);
                        }
                        Ok(false) => { /* event dropped (channel full) */ }
                        Err(_) => {
                            closed.push(receiver_id);
                        }
                    }
                }
                for id in closed {
                    dataflow.subscribe_channels.remove(id);
                }
            }
            DoraEvent::LogBroadcast {
                dataflow_id,
                log_message,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    return Ok(());
                };

                if dataflow.log_subscribers.is_empty() {
                    return Ok(());
                }

                // Serialize to JSON once (shared across all subscribers)
                let json = match serde_json::to_string(&log_message) {
                    Ok(j) => j,
                    Err(e) => {
                        tracing::warn!("failed to serialize LogMessage: {e}");
                        return Ok(());
                    }
                };

                // Convert to Arrow once, share the sample across subscribers
                use dora_arrow_convert::IntoArrow;
                let array = json.as_str().into_arrow();
                let array: dora_node_api::arrow::array::ArrayData = array.into();
                let total_len = dora_node_api::arrow_utils::required_data_size(&array);
                let mut sample: aligned_vec::AVec<u8, aligned_vec::ConstAlign<128>> =
                    aligned_vec::AVec::__from_elem(128, 0, total_len);
                let type_info =
                    dora_node_api::arrow_utils::copy_array_into_sample(&mut sample, &array);

                let mut closed = Vec::new();
                for sub in &dataflow.log_subscribers {
                    // Apply level filter
                    if let Some(min_level) = &sub.filter.min_level
                        && !log_message.level.passes(min_level)
                    {
                        continue;
                    }
                    // Apply node filter
                    if let Some(node_filter) = &sub.filter.node_filter
                        && log_message.node_id.as_ref() != Some(node_filter)
                    {
                        continue;
                    }
                    // Don't deliver logs to the subscriber itself (avoid loops)
                    if log_message.node_id.as_ref() == Some(&sub.node_id) {
                        continue;
                    }

                    let Some(channel) = dataflow.subscribe_channels.get(&sub.node_id) else {
                        continue;
                    };

                    let metadata =
                        metadata::Metadata::new(self.clock.new_timestamp(), type_info.clone());

                    let send_result = send_with_timestamp(
                        channel,
                        NodeEvent::Input {
                            id: sub.input_id.clone(),
                            metadata: Arc::new(metadata),
                            data: Some(Arc::new(DataMessage::Vec(sample.clone()))),
                        },
                        &self.clock,
                    );
                    match send_result {
                        Ok(true) => {
                            dataflow.inc_pending(&sub.node_id);
                        }
                        Ok(false) => { /* event dropped (channel full) */ }
                        Err(_) => {
                            closed.push(sub.node_id.clone());
                        }
                    }
                }
                for id in &closed {
                    dataflow.subscribe_channels.remove(id);
                }
                // Prune stale log subscribers whose channels were just removed
                dataflow
                    .log_subscribers
                    .retain(|sub| !closed.contains(&sub.node_id));
            }
            DoraEvent::SpawnedNodeResult {
                dataflow_id,
                node_id,
                dynamic_node,
                exit_status,
                restart,
                restart_count,
            } => {
                let mut logger = self
                    .logger
                    .for_dataflow(dataflow_id)
                    .for_node(node_id.clone());
                logger
                    .log(
                        LogLevel::Debug,
                        Some("daemon".into()),
                        format!("handling node stop with exit status {exit_status:?} (restart: {restart}, restart_count: {restart_count})"),
                    )
                    .await;

                let node_result = match exit_status {
                    NodeExitStatus::Success => Ok(()),
                    exit_status => {
                        let dataflow = self.running.get(&dataflow_id);
                        let caused_by_node = dataflow
                            .and_then(|dataflow| {
                                dataflow.cascading_error_causes.error_caused_by(&node_id)
                            })
                            .cloned();
                        let grace_duration_kill = dataflow
                            .map(|d| d.grace_duration_kills.contains(&node_id))
                            .unwrap_or_default();

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
                                let cause = dataflow
                                    .and_then(|d| d.node_stderr_most_recent.get(&node_id))
                                    .map(|queue| {
                                        let mut lines = Vec::new();
                                        if queue.is_full() {
                                            lines.push("[...]".into());
                                        }
                                        while let Some(line) = queue.pop() {
                                            lines.push(line);
                                        }
                                        lines
                                    })
                                    .map(extract_err_from_stderr)
                                    .unwrap_or_default();

                                NodeErrorCause::Other { stderr: cause }
                            }
                        };
                        Err(NodeError {
                            timestamp: self.clock.new_timestamp(),
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

                if restart {
                    logger
                        .log(
                            LogLevel::Info,
                            Some("daemon".into()),
                            format!("node will be restarted (attempt {})", restart_count + 1),
                        )
                        .await;

                    // Notify downstream nodes about the restart
                    if let Some(dataflow) = self.running.get(&dataflow_id) {
                        let downstream: BTreeSet<NodeId> = dataflow
                            .mappings
                            .iter()
                            .filter(|(k, _)| k.0 == node_id)
                            .flat_map(|(_, v)| v)
                            .map(|(receiver_id, _)| receiver_id.clone())
                            .collect();
                        for receiver_id in &downstream {
                            if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
                                // First try the non-blocking path. NodeRestarted is
                                // control-classed inside send_with_timestamp, so it
                                // benefits from the control headroom reservation.
                                match send_with_timestamp(
                                    channel,
                                    NodeEvent::NodeRestarted {
                                        id: node_id.clone(),
                                    },
                                    &self.clock,
                                ) {
                                    Ok(true) => {
                                        dataflow.inc_pending(receiver_id);
                                    }
                                    Ok(false) => {
                                        // Channel full even with control headroom.
                                        // NodeRestarted is a critical lifecycle event:
                                        // dropping it leaves service/action clients
                                        // blocked on pre-crash correlations forever
                                        // (dora-rs/adora#148). Use an unconditional
                                        // backpressure-aware send to guarantee delivery.
                                        //
                                        // This blocks the daemon main loop until the
                                        // receiver drains one slot. Acceptable because:
                                        // - NodeRestarted is rare (only on crash+restart)
                                        // - The receiver (Listener::run_inner) is a tokio
                                        //   task on the same runtime, so it will make
                                        //   progress cooperatively
                                        // - If the receiver is dead the channel is closed
                                        //   and send() returns Err immediately
                                        tracing::warn!(
                                            %dataflow_id,
                                            restarted_node = %node_id,
                                            %receiver_id,
                                            "NodeRestarted try_send failed (channel full); \
                                             awaiting backpressure delivery"
                                        );
                                        let msg = Timestamped {
                                            inner: NodeEvent::NodeRestarted {
                                                id: node_id.clone(),
                                            },
                                            timestamp: self.clock.new_timestamp(),
                                        };
                                        match channel.send(msg).await {
                                            Ok(()) => {
                                                dataflow.inc_pending(receiver_id);
                                            }
                                            Err(_closed) => {
                                                tracing::warn!(
                                                    %dataflow_id,
                                                    restarted_node = %node_id,
                                                    %receiver_id,
                                                    "NodeRestarted delivery failed: \
                                                     receiver channel closed"
                                                );
                                            }
                                        }
                                    }
                                    Err(_) => {
                                        tracing::warn!(
                                            %dataflow_id,
                                            restarted_node = %node_id,
                                            %receiver_id,
                                            "failed to send NodeRestarted: receiver channel closed"
                                        );
                                    }
                                }
                            }
                        }
                    }
                } else {
                    self.dataflow_node_results
                        .entry(dataflow_id)
                        .or_default()
                        .insert(node_id.clone(), node_result);

                    self.handle_node_stop(dataflow_id, &node_id, dynamic_node)
                        .await?;
                }
            }
            DoraEvent::ProcessHandleReplaced {
                dataflow_id,
                node_id,
                new_handle,
            } => {
                // The per-node restart_loop just spawned a replacement
                // process with a fresh op_tx/op_rx pair. Swap it into
                // running_nodes so subsequent stop/kill operations
                // target the new incarnation rather than a dead
                // predecessor's channel (dora-rs/adora#152).
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                        // Overwriting the previous Some(old_handle)
                        // drops it, which currently fires its `Kill`
                        // send on the already-closed old op_rx — a
                        // no-op.
                        node.process = Some(new_handle);
                    } else {
                        tracing::warn!(
                            %dataflow_id,
                            %node_id,
                            "ProcessHandleReplaced for unknown node"
                        );
                    }
                } else {
                    tracing::warn!(
                        %dataflow_id,
                        %node_id,
                        "ProcessHandleReplaced for unknown dataflow"
                    );
                }
            }
        }
        Ok(())
    }

    fn base_working_dir(
        &self,
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
            buffer[..read_len].trim_ascii_end()
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

async fn set_up_event_stream(
    coordinator_ws_addr: SocketAddr,
    machine_id: &Option<String>,
    labels: BTreeMap<String, String>,
    clock: &Arc<HLC>,
    remote_daemon_events_rx: flume::Receiver<eyre::Result<Timestamped<InterDaemonEvent>>>,
    // used for dynamic nodes
    local_listen_port: u16,
) -> eyre::Result<(
    DaemonId,
    coordinator::CoordinatorSender,
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
    let (daemon_id, coordinator_sender, coordinator_events) = coordinator::register(
        coordinator_ws_addr,
        machine_id.clone(),
        labels,
        clock.clone(),
    )
    .await
    .wrap_err("failed to connect to dora-coordinator")?;
    let coordinator_events = coordinator_events.map(
        |Timestamped {
             inner: event,
             timestamp,
         }| Timestamped {
            inner: Event::Coordinator(event),
            timestamp,
        },
    );
    let (events_tx, events_rx) = flume::bounded(10);
    let _listen_port =
        local_listener::spawn_listener_loop((LOCALHOST, local_listen_port).into(), events_tx)
            .await?;
    let dynamic_node_events = events_rx.into_stream().map(|e| Timestamped {
        inner: Event::DynamicNode(e.inner),
        timestamp: e.timestamp,
    });
    let incoming = (
        coordinator_events,
        remote_daemon_events,
        dynamic_node_events,
    )
        .merge();
    Ok((daemon_id, coordinator_sender, incoming))
}

#[allow(clippy::too_many_arguments)]
async fn send_output_to_local_receivers(
    node_id: NodeId,
    output_id: DataId,
    dataflow: &mut RunningDataflow,
    metadata: &metadata::Metadata,
    data: Option<DataMessage>,
    clock: &HLC,
    ft_stats: Option<&FaultToleranceStats>,
    need_data_bytes: bool,
) -> Result<Option<AVec<u8, ConstAlign<128>>>, eyre::ErrReport> {
    let timestamp = metadata.timestamp();
    let empty_set = BTreeSet::new();
    let output_id = OutputId(node_id, output_id);
    let local_receivers = dataflow.mappings.get(&output_id).unwrap_or(&empty_set);
    let OutputId(node_id, _) = output_id;
    // Wrap in Arc once; fan-out clones are O(1) atomic ref bumps instead of O(payload_size) memcpy
    let metadata = Arc::new(metadata.clone());
    let data = data.map(Arc::new);
    let mut closed = Vec::new();
    for (receiver_id, input_id) in local_receivers {
        if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
            // Reserve headroom for control events (Stop, InputClosed, etc.)
            if channel.capacity() < CONTROL_EVENT_HEADROOM {
                tracing::warn!(
                    node = %receiver_id,
                    "event channel low on capacity ({}/{}), dropping data to preserve control headroom",
                    channel.capacity(),
                    NODE_EVENT_CHANNEL_CAPACITY,
                );
                continue;
            }
            let item = NodeEvent::Input {
                id: input_id.clone(),
                metadata: metadata.clone(),
                data: data.clone(),
            };
            match channel.try_send(Timestamped {
                inner: item,
                timestamp,
            }) {
                Ok(()) => {
                    dataflow.inc_pending(receiver_id);
                    if let Some(deadline) = dataflow
                        .input_deadlines
                        .get_mut(&(receiver_id.clone(), input_id.clone()))
                    {
                        deadline.last_received = Some(Instant::now());
                    }
                    // Circuit breaker recovery: re-open broken input
                    if let Some(timeout) = dataflow
                        .broken_inputs
                        .remove(&(receiver_id.clone(), input_id.clone()))
                    {
                        tracing::info!(
                            "input `{receiver_id}/{input_id}` recovered, \
                             re-opening (circuit breaker reset)",
                        );
                        if let Some(stats) = ft_stats {
                            stats
                                .circuit_breaker_recoveries
                                .fetch_add(1, atomic::Ordering::Relaxed);
                        }
                        dataflow
                            .open_inputs
                            .entry(receiver_id.clone())
                            .or_default()
                            .insert(input_id.clone());
                        dataflow.input_deadlines.insert(
                            (receiver_id.clone(), input_id.clone()),
                            InputDeadline {
                                timeout,
                                // A message just arrived — arm immediately.
                                last_received: Some(Instant::now()),
                            },
                        );
                        match send_with_timestamp(
                            channel,
                            NodeEvent::InputRecovered {
                                id: input_id.clone(),
                            },
                            clock,
                        ) {
                            Ok(true) => {
                                dataflow.inc_pending(receiver_id);
                            }
                            Ok(false) => { /* event dropped (channel full) */ }
                            Err(_) => {
                                tracing::warn!(
                                    "failed to send InputRecovered for `{receiver_id}/{input_id}`"
                                );
                            }
                        }
                    }
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
                Err(mpsc::error::TrySendError::Closed(_)) => {
                    closed.push(receiver_id);
                }
                Err(mpsc::error::TrySendError::Full(_)) => {
                    tracing::warn!(
                        node = %receiver_id,
                        "event channel full (capacity {}), dropping message (node is too slow)",
                        NODE_EVENT_CHANNEL_CAPACITY,
                    );
                }
            }
        }
    }
    for id in closed {
        dataflow.subscribe_channels.remove(id);
    }
    // Extract data bytes for remote receivers (Zenoh). Skip the expensive shmem
    // mmap + copy when there are no remote receivers (local-only optimization).
    let (data_bytes, drop_token) = if need_data_bytes {
        let data_owned = data.as_ref().map(|arc| (**arc).clone());
        match data_owned {
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
                let mem_slice = unsafe { memory.as_slice() };
                if len > mem_slice.len() {
                    eyre::bail!(
                        "shared memory length {len} exceeds region size {}",
                        mem_slice.len()
                    );
                }
                let data = Some(AVec::from_slice(1, &mem_slice[..len]));
                (data, Some(drop_token))
            }
            Some(DataMessage::Vec(v)) => (Some(v), None),
        }
    } else {
        // Extract drop token without copying data
        let drop_token = data.as_ref().and_then(|d| d.drop_token());
        (None, drop_token)
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
    // Clean up broken state if this input was circuit-broken
    let was_broken = dataflow
        .broken_inputs
        .remove(&(receiver_id.clone(), input_id.clone()))
        .is_some();

    let was_open = dataflow
        .open_inputs
        .get_mut(receiver_id)
        .map(|inputs| inputs.remove(input_id))
        .unwrap_or(false);

    if !was_open && !was_broken {
        return;
    }

    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
        if was_open
            && send_with_timestamp(
                channel,
                NodeEvent::InputClosed {
                    id: input_id.clone(),
                },
                clock,
            )
            .ok()
                == Some(true)
        {
            dataflow.inc_pending(receiver_id);
        }

        let has_broken = dataflow
            .broken_inputs
            .keys()
            .any(|(nid, _)| nid == receiver_id);
        if dataflow.open_inputs(receiver_id).is_empty() && !has_broken {
            if let Some(node) = dataflow.running_nodes.get_mut(receiver_id) {
                node.disable_restart();
            }
            if send_with_timestamp(channel, NodeEvent::AllInputsClosed, clock).ok() == Some(true) {
                dataflow.inc_pending(receiver_id);
            }
        }
    }
}

/// Circuit-breaker version of close_input: closes the input but keeps it recoverable.
/// The input is moved to `broken_inputs` before calling this function.
fn break_input(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    input_id: &DataId,
    clock: &HLC,
) {
    if let Some(open_inputs) = dataflow.open_inputs.get_mut(receiver_id)
        && !open_inputs.remove(input_id)
    {
        return;
    }
    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
        if send_with_timestamp(
            channel,
            NodeEvent::InputClosed {
                id: input_id.clone(),
            },
            clock,
        )
        .ok()
            == Some(true)
        {
            dataflow.inc_pending(receiver_id);
        }

        let has_broken = dataflow
            .broken_inputs
            .keys()
            .any(|(nid, _)| nid == receiver_id);
        if dataflow.open_inputs(receiver_id).is_empty() && !has_broken {
            if let Some(node) = dataflow.running_nodes.get_mut(receiver_id) {
                node.disable_restart();
            }
            if send_with_timestamp(channel, NodeEvent::AllInputsClosed, clock).ok() == Some(true) {
                dataflow.inc_pending(receiver_id);
            }
        }
    }
}

// RunningDataflow and related types are in running_dataflow.rs
// FaultToleranceStats and CascadingErrorCauses are in fault_tolerance.rs

pub(crate) fn empty_type_info() -> ArrowTypeInfo {
    ArrowTypeInfo {
        data_type: DataType::Null,
        len: 0,
        null_count: 0,
        validity: None,
        offset: 0,
        buffer_offsets: Vec::new(),
        child_data: Vec::new(),
        field_names: None,
        schema_hash: None,
    }
}

fn set_up_ctrlc_handler(
    clock: Arc<HLC>,
) -> eyre::Result<tokio::sync::mpsc::Receiver<Timestamped<Event>>> {
    let (ctrlc_tx, ctrlc_rx) = mpsc::channel(1);

    let mut ctrlc_sent = 0;
    let handler_result = ctrlc::set_handler(move || {
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
    });

    if let Err(e) = handler_result {
        tracing::warn!("ctrl-c handler already registered, skipping: {e}");
        // The closure (and ctrlc_tx) was dropped since the handler wasn't registered.
        // Create a new channel whose receiver pends indefinitely so the daemon
        // doesn't interpret a closed channel as a ctrl-c event.
        let (tx, rx) = mpsc::channel(1);
        std::mem::forget(tx);
        return Ok(rx);
    }

    Ok(ctrlc_rx)
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

#[cfg(test)]
mod fault_tolerance_tests {
    use super::*;
    use std::sync::atomic::AtomicU32;

    use dora_message::{
        config::CommunicationConfig,
        daemon_to_node::NodeEvent,
        descriptor::{Debug as DescriptorDebug, Descriptor},
    };
    use std::sync::atomic::{AtomicBool, AtomicU64};

    fn test_dataflow() -> RunningDataflow {
        let descriptor = Descriptor {
            nodes: vec![],
            communication: CommunicationConfig::default(),
            deploy: None,
            debug: DescriptorDebug::default(),
            health_check_interval: None,
            strict_types: None,
            type_rules: vec![],
        };
        RunningDataflow::new(Uuid::nil(), DaemonId::new(None), descriptor)
    }

    fn test_running_node() -> RunningNode {
        RunningNode {
            process: None,
            node_config: NodeConfig {
                dataflow_id: Uuid::nil().into(),
                node_id: NodeId::from("test".to_string()),
                run_config: NodeRunConfig {
                    inputs: BTreeMap::new(),
                    outputs: BTreeSet::new(),
                    output_types: BTreeMap::new(),
                    input_types: BTreeMap::new(),
                    output_framing: BTreeMap::new(),
                },
                daemon_communication: None,
                dataflow_descriptor: serde_yaml::Value::Null,
                dynamic: false,
                write_events_to: None,
                restart_count: 0,
            },
            pid: None,
            restart_count: Arc::new(AtomicU32::new(0)),
            restart_policy: RestartPolicy::Never,
            disable_restart: Arc::new(AtomicBool::new(false)),
            last_activity: Arc::new(AtomicU64::new(0)),
            health_check_timeout: None,
        }
    }

    fn test_clock() -> HLC {
        HLC::default()
    }

    fn drain_events(rx: &mut mpsc::Receiver<Timestamped<NodeEvent>>) -> Vec<NodeEvent> {
        let mut events = Vec::new();
        while let Ok(timestamped) = rx.try_recv() {
            events.push(timestamped.inner);
        }
        events
    }

    fn matches_event(event: &NodeEvent, expected: &str) -> bool {
        match (event, expected) {
            (NodeEvent::InputClosed { .. }, "InputClosed") => true,
            (NodeEvent::InputRecovered { .. }, "InputRecovered") => true,
            (NodeEvent::AllInputsClosed, "AllInputsClosed") => true,
            (NodeEvent::Input { .. }, "Input") => true,
            _ => false,
        }
    }

    // -- Test 1: close_input removes input, sends InputClosed, no AllInputsClosed with remaining inputs --

    #[test]
    fn close_input_removes_from_open_inputs() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();
        let input_y: DataId = "input_y".to_string().into();

        // Setup: node_a has two open inputs
        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_x.clone());
        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_y.clone());

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        // Act: permanently close input_x
        close_input(&mut df, &node_a, &input_x, &clock);

        // Assert: input_x removed, input_y still open
        let open = df.open_inputs(&node_a);
        assert!(!open.contains(&input_x));
        assert!(open.contains(&input_y));

        // Assert: only InputClosed sent (no AllInputsClosed)
        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 1);
        assert!(matches_event(&events[0], "InputClosed"));
    }

    // -- Test 2: close_input sends AllInputsClosed + disable_restart on last input --

    #[test]
    fn close_input_sends_all_inputs_closed() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();

        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_x.clone());

        let running = test_running_node();
        let disable_restart = running.disable_restart.clone();
        df.running_nodes.insert(node_a.clone(), running);

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        close_input(&mut df, &node_a, &input_x, &clock);

        assert!(df.open_inputs(&node_a).is_empty());

        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 2);
        assert!(matches_event(&events[0], "InputClosed"));
        assert!(matches_event(&events[1], "AllInputsClosed"));
        assert!(disable_restart.load(atomic::Ordering::Acquire));
    }

    // -- Test 3: close_input defers AllInputsClosed when broken_inputs exist --

    #[test]
    fn close_input_deferred_by_broken_inputs() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();
        let input_y: DataId = "input_y".to_string().into();

        // node_a has input_x open, input_y is broken
        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_x.clone());
        df.broken_inputs
            .insert((node_a.clone(), input_y.clone()), Duration::from_secs(10));

        let running = test_running_node();
        let disable_restart = running.disable_restart.clone();
        df.running_nodes.insert(node_a.clone(), running);

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        // Close last open input — but broken input still exists
        close_input(&mut df, &node_a, &input_x, &clock);

        let events = drain_events(&mut rx);
        // Only InputClosed, NO AllInputsClosed (broken input might recover)
        assert_eq!(events.len(), 1);
        assert!(matches_event(&events[0], "InputClosed"));
        assert!(!disable_restart.load(atomic::Ordering::Acquire));
    }

    // -- Test 4: close_input on already-broken input --

    #[test]
    fn close_input_on_already_broken_input() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();

        // input_x is broken (not in open_inputs)
        df.broken_inputs
            .insert((node_a.clone(), input_x.clone()), Duration::from_secs(10));

        let running = test_running_node();
        let disable_restart = running.disable_restart.clone();
        df.running_nodes.insert(node_a.clone(), running);

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        // Permanently close the broken input (upstream exited)
        close_input(&mut df, &node_a, &input_x, &clock);

        // broken_inputs cleaned up
        assert!(
            !df.broken_inputs
                .contains_key(&(node_a.clone(), input_x.clone()))
        );

        let events = drain_events(&mut rx);
        // No InputClosed (was already sent when it broke), just AllInputsClosed
        assert_eq!(events.len(), 1);
        assert!(matches_event(&events[0], "AllInputsClosed"));
        assert!(disable_restart.load(atomic::Ordering::Acquire));
    }

    // -- Test 5: break_input sends InputClosed --

    #[test]
    fn break_input_sends_input_closed() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();

        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_x.clone());

        // Pre-populate broken_inputs (as check_input_timeouts does before calling break_input)
        df.broken_inputs
            .insert((node_a.clone(), input_x.clone()), Duration::from_secs(5));

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        break_input(&mut df, &node_a, &input_x, &clock);

        // Removed from open_inputs
        assert!(!df.open_inputs(&node_a).contains(&input_x));

        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 1);
        assert!(matches_event(&events[0], "InputClosed"));
    }

    // -- Test 6: break_input defers AllInputsClosed when broken_inputs has entry --

    #[test]
    fn break_input_defers_all_inputs_closed() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();

        // Only one input, and it's open
        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_x.clone());

        // Caller inserts into broken_inputs before calling break_input
        df.broken_inputs
            .insert((node_a.clone(), input_x.clone()), Duration::from_secs(5));

        let running = test_running_node();
        let disable_restart = running.disable_restart.clone();
        df.running_nodes.insert(node_a.clone(), running);

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        break_input(&mut df, &node_a, &input_x, &clock);

        let events = drain_events(&mut rx);
        // Only InputClosed — AllInputsClosed deferred because broken_inputs has entry
        assert_eq!(events.len(), 1);
        assert!(matches_event(&events[0], "InputClosed"));
        assert!(!disable_restart.load(atomic::Ordering::Acquire));
    }

    // -- Test 7: Circuit breaker recovery via send_output_to_local_receivers --

    #[tokio::test]
    async fn circuit_breaker_recovery() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let sender: NodeId = "sender".to_string().into();
        let receiver: NodeId = "receiver".to_string().into();
        let output: DataId = "output".to_string().into();
        let input: DataId = "input".to_string().into();

        // Setup mapping: sender/output -> receiver/input
        df.mappings
            .entry(OutputId(sender.clone(), output.clone()))
            .or_default()
            .insert((receiver.clone(), input.clone()));

        // Input is broken (timed out earlier)
        let timeout = Duration::from_secs(5);
        df.broken_inputs
            .insert((receiver.clone(), input.clone()), timeout);

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(receiver.clone(), tx);

        // Send data from upstream
        let metadata = metadata::Metadata::new(
            clock.new_timestamp(),
            ArrowTypeInfo {
                data_type: DataType::UInt8,
                len: 0,
                null_count: 0,
                validity: None,
                offset: 0,
                buffer_offsets: vec![],
                child_data: vec![],
                field_names: None,
                schema_hash: None,
            },
        );

        let result = send_output_to_local_receivers(
            sender, output, &mut df, &metadata, None, &clock, None, false,
        )
        .await;
        assert!(result.is_ok());

        // Assert: broken input recovered
        assert!(
            !df.broken_inputs
                .contains_key(&(receiver.clone(), input.clone()))
        );

        // Assert: re-added to open_inputs
        assert!(df.open_inputs(&receiver).contains(&input));

        // Assert: deadline recreated
        assert!(
            df.input_deadlines
                .contains_key(&(receiver.clone(), input.clone()))
        );
        let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
        assert_eq!(deadline.timeout, timeout);

        // Assert: events — Input + InputRecovered
        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 2);
        assert!(matches_event(&events[0], "Input"));
        assert!(matches_event(&events[1], "InputRecovered"));
    }

    // -- Test 8: Full circuit breaker cycle: open -> break -> recover --

    #[tokio::test]
    async fn full_circuit_breaker_cycle() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let sender: NodeId = "sender".to_string().into();
        let receiver: NodeId = "receiver".to_string().into();
        let output: DataId = "output".to_string().into();
        let input: DataId = "input".to_string().into();

        // Setup: receiver has one open input with timeout
        df.open_inputs
            .entry(receiver.clone())
            .or_default()
            .insert(input.clone());
        let timeout = Duration::from_secs(5);
        df.input_deadlines.insert(
            (receiver.clone(), input.clone()),
            InputDeadline {
                timeout,
                last_received: Some(Instant::now()),
            },
        );

        // Setup mapping
        df.mappings
            .entry(OutputId(sender.clone(), output.clone()))
            .or_default()
            .insert((receiver.clone(), input.clone()));

        let running = test_running_node();
        let disable_restart = running.disable_restart.clone();
        df.running_nodes.insert(receiver.clone(), running);

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(receiver.clone(), tx);

        // Step 1: Simulate timeout — insert into broken_inputs then break
        df.broken_inputs
            .insert((receiver.clone(), input.clone()), timeout);
        break_input(&mut df, &receiver, &input, &clock);
        df.input_deadlines
            .remove(&(receiver.clone(), input.clone()));

        // Verify broken state
        assert!(!df.open_inputs(&receiver).contains(&input));
        assert!(
            df.broken_inputs
                .contains_key(&(receiver.clone(), input.clone()))
        );
        assert!(!disable_restart.load(atomic::Ordering::Acquire)); // deferred

        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 1);
        assert!(matches_event(&events[0], "InputClosed"));

        // Step 2: Upstream sends data again — recovery
        let metadata = metadata::Metadata::new(
            clock.new_timestamp(),
            ArrowTypeInfo {
                data_type: DataType::UInt8,
                len: 0,
                null_count: 0,
                validity: None,
                offset: 0,
                buffer_offsets: vec![],
                child_data: vec![],
                field_names: None,
                schema_hash: None,
            },
        );

        let result = send_output_to_local_receivers(
            sender, output, &mut df, &metadata, None, &clock, None, false,
        )
        .await;
        assert!(result.is_ok());

        // Verify recovered state
        assert!(df.open_inputs(&receiver).contains(&input));
        assert!(
            !df.broken_inputs
                .contains_key(&(receiver.clone(), input.clone()))
        );
        assert!(
            df.input_deadlines
                .contains_key(&(receiver.clone(), input.clone()))
        );
        assert!(!disable_restart.load(atomic::Ordering::Acquire));

        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 2);
        assert!(matches_event(&events[0], "Input"));
        assert!(matches_event(&events[1], "InputRecovered"));
    }

    // -- Test: send_with_timestamp delivers ParamUpdate to subscribed node --

    #[test]
    fn param_update_delivered_to_node() {
        let _df = test_dataflow();
        let clock = test_clock();
        let _node_id: NodeId = "node_a".to_string().into();

        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);

        // Simulate sending a ParamUpdate
        let result = send_with_timestamp(
            &tx,
            NodeEvent::ParamUpdate {
                key: "threshold".into(),
                value: serde_json::json!(42),
            },
            &clock,
        );
        assert!(result.is_ok());

        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 1);
        match &events[0] {
            NodeEvent::ParamUpdate { key, value } => {
                assert_eq!(key, "threshold");
                assert_eq!(value, &serde_json::json!(42));
            }
            other => panic!("expected ParamUpdate, got {other:?}"),
        }
    }

    #[test]
    fn param_update_fails_on_closed_channel() {
        let clock = test_clock();
        let (tx, rx) = mpsc::channel::<Timestamped<NodeEvent>>(NODE_EVENT_CHANNEL_CAPACITY);
        drop(rx); // close the receiver

        let result = send_with_timestamp(
            &tx,
            NodeEvent::ParamUpdate {
                key: "rate".into(),
                value: serde_json::json!(10),
            },
            &clock,
        );
        assert!(result.is_err());
    }

    #[test]
    fn param_delete_delivered_to_node() {
        let clock = test_clock();
        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);

        let result = send_with_timestamp(
            &tx,
            NodeEvent::ParamDeleted {
                key: "threshold".into(),
            },
            &clock,
        );
        assert!(result.is_ok());

        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 1);
        match &events[0] {
            NodeEvent::ParamDeleted { key } => {
                assert_eq!(key, "threshold");
            }
            other => panic!("expected ParamDeleted, got {other:?}"),
        }
    }

    #[test]
    fn param_delete_fails_on_closed_channel() {
        let clock = test_clock();
        let (tx, rx) = mpsc::channel::<Timestamped<NodeEvent>>(NODE_EVENT_CHANNEL_CAPACITY);
        drop(rx); // close the receiver

        let result =
            send_with_timestamp(&tx, NodeEvent::ParamDeleted { key: "rate".into() }, &clock);
        assert!(result.is_err());
    }
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
                output_types: BTreeMap::new(),
                output_framing: BTreeMap::new(),
                input_types: BTreeMap::new(),
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
