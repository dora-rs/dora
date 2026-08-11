use aligned_vec::{AVec, ConstAlign};
use coordinator::CoordinatorEvent;
use crossbeam::queue::ArrayQueue;
use dora_core::{
    build::{self, BuildInfo, GitManager, PrevGitSource},
    config::{DataId, Input, InputMapping, NodeId, NodeRunConfig, OperatorId},
    descriptor::{
        CoreNodeKind, DYNAMIC_SOURCE, Descriptor, DescriptorExt, ResolvedNode, RuntimeNode,
        read_as_descriptor, validate,
    },
    topics::{
        DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST, MulticastScouting,
        open_zenoh_session_with_listen, reserve_zenoh_endpoint, validate_zenoh_listen,
        zenoh_bind_address_for, zenoh_daemon_control_topic, zenoh_output_publish_topic,
    },
    uhlc::{self, HLC},
};
use dora_memory_pool::{MemoryPoolId, MemoryPoolManager, MemoryPoolMetadata};
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{
        DaemonId, DataMessage, GitSource, LogLevel, NodeError, NodeErrorCause, NodeExitStatus,
    },
    coordinator_to_cli::DataflowResult,
    coordinator_to_daemon::{
        BuildDataflowNodes, DaemonCoordinatorEvent, SpawnDataflowNodes, StateCatchUpEntry,
        StateCatchUpOperation,
    },
    daemon_to_coordinator::{
        CoordinatorRequest, DaemonCoordinatorReply, DaemonEvent, DataflowDaemonResult,
    },
    daemon_to_node::{DaemonReply, NodeConfig, NodeEvent},
    descriptor::{NodeSource, RestartPolicy},
    metadata::{self, MetadataParameters},
    node_to_daemon::{DynamicNodeEvent, Timestamped},
};
use eyre::{Context, ContextCompat, Result, bail, eyre};
use futures::{TryFutureExt, future, stream};
use futures_concurrency::stream::Merge;
use local_listener::DynamicNodeEventWrapper;
use log::{CoordinatorLogTarget, DaemonLogger, DataflowLogger, Logger};
use spawn::Spawner;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque},
    env::current_dir,
    future::Future,
    io,
    net::{IpAddr, SocketAddr},
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
use zenoh::qos::{CongestionControl, Priority};

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
            exit_when_nodes_finish: None,
            type_rules: vec![],
            env: None,
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
        RoutingFixture {
            sender_id: "sender".to_string().into(),
            output_id: "output".to_string().into(),
            data_msg: DataMessage::Vec(AVec::from_slice(128, &data)),
            metadata: metadata::Metadata::new(clock.new_timestamp()),
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
mod output_routing;
mod pending;
pub(crate) mod running_dataflow;
mod shutdown;
mod socket_stream_utils;
mod spawn;

pub(crate) use event_types::{
    CONTROL_EVENT_HEADROOM, DaemonNodeEvent, DoraEvent, Event, InterDaemonEvent,
    NODE_EVENT_CHANNEL_CAPACITY, OutputId, RunStatus, ZenohOutbound, send_with_timestamp,
};
pub(crate) use fault_tolerance::{CascadingErrorCauses, FaultToleranceStats};
pub(crate) use running_dataflow::{
    FinishDataflowWhen, InputDeadline, ProcessHandle, ProcessOperation, RunningDataflow,
    RunningNode,
};

use crate::{extract_err_from_stderr::extract_err_from_stderr, pending::DataflowStatus};

const STDERR_LOG_LINES_MAX: usize = 500;
const METRICS_INTERVAL: Duration = Duration::from_secs(2);
const METRICS_INTERVAL_SECS: f64 = METRICS_INTERVAL.as_secs_f64();
/// Capacity of the Zenoh publish drain channel. Large enough for burst
/// patterns; a full channel is recorded as publish backpressure and dropped so
/// the daemon event loop does not stall.
const ZENOH_PUBLISH_CHANNEL_CAPACITY: usize = 256;
/// Control-plane queue for lifecycle events such as `OutputClosed`. This queue
/// is separate from the data-plane publish channel so regular output bursts do
/// not starve close notifications.
const ZENOH_CONTROL_CHANNEL_CAPACITY: usize = 1024;
/// Pending control events admitted by the daemon event loop. A background
/// forwarder waits for capacity in the control publish queue; the event loop
/// itself never waits.
const ZENOH_CONTROL_PENDING_CHANNEL_CAPACITY: usize = 4096;
/// How long the daemon keeps trying to (re)connect to the coordinator before
/// giving up and exiting. Bounds the orphan-daemon window when the coordinator
/// is permanently gone (dora-rs/dora#1996); a reachable coordinator connects
/// well within this, so transient outages and reconnects are unaffected.
const COORDINATOR_RECONNECT_TIMEOUT: Duration = Duration::from_secs(90);
/// Pause between losing the coordinator connection and attempting to reconnect.
const COORDINATOR_RECONNECT_BACKOFF: Duration = Duration::from_secs(1);
/// After the daemon has connected at least once, how long it keeps retrying
/// failing reconnects before giving up and exiting. A fast TCP refusal (e.g.
/// the coordinator briefly restarting, so the OS releases and re-grabs the
/// port) returns an error almost instantly, which `COORDINATOR_RECONNECT_TIMEOUT`
/// does not bound — that only guards a *hung* connect. Without this window a
/// single refused attempt would exit the daemon and kill its running dataflows
/// (dora-rs/dora#1998). A coordinator that stays gone past this window is
/// treated as permanently gone -> exit rather than orphan (dora-rs/dora#1996).
const COORDINATOR_RECONNECT_RETRY_WINDOW: Duration = Duration::from_secs(30);

/// Records a failed reconnect attempt and reports whether the retry window has
/// elapsed (so the daemon should give up and exit). `deadline` tracks the
/// current run of consecutive failures: it is `None` until the first failure,
/// when it is set to `now + window`; callers clear it back to `None` on a
/// successful reconnect so each fresh outage gets a full window.
fn reconnect_window_elapsed(
    now: Instant,
    deadline: &mut Option<Instant>,
    window: Duration,
) -> bool {
    now >= *deadline.get_or_insert(now + window)
}

fn deliver_param_update_strict(
    dataflow: &RunningDataflow,
    node_id: &NodeId,
    key: String,
    value: serde_json::Value,
    clock: &HLC,
) -> eyre::Result<()> {
    let channel = dataflow
        .subscribe_channels
        .get(node_id)
        .ok_or_else(|| eyre!("node `{node_id}` not connected"))?;
    let value_json = serde_json::to_vec(&value)
        .map_err(|e| eyre!("failed to serialize param value for node `{node_id}`: {e}"))?;
    match send_with_timestamp(channel, NodeEvent::ParamUpdate { key, value_json }, clock) {
        Ok(true) => {
            dataflow.inc_pending(node_id);
            Ok(())
        }
        Ok(false) => Err(eyre!("node `{node_id}` channel full")),
        Err(_) => Err(eyre!("node `{node_id}` channel closed")),
    }
}

fn deliver_param_delete_strict(
    dataflow: &RunningDataflow,
    node_id: &NodeId,
    key: String,
    clock: &HLC,
) -> eyre::Result<()> {
    let channel = dataflow
        .subscribe_channels
        .get(node_id)
        .ok_or_else(|| eyre!("node `{node_id}` not connected"))?;
    match send_with_timestamp(channel, NodeEvent::ParamDeleted { key }, clock) {
        Ok(true) => {
            dataflow.inc_pending(node_id);
            Ok(())
        }
        Ok(false) => Err(eyre!("node `{node_id}` channel full")),
        Err(_) => Err(eyre!("node `{node_id}` channel closed")),
    }
}

/// The Daemon manages running dataflows, node communication, and inter-daemon
/// message routing. Fields are `pub(crate)` to enable `impl Daemon` blocks in
/// submodules (e.g., `node_events.rs`, `dataflow_lifecycle.rs`) as part of the
/// ongoing daemon module split.
/// Optional behavior for [`Daemon::run_dataflow_with`].
///
/// A struct rather than more positional parameters so that adding an
/// option later is not a breaking change for the published crate.
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct RunDataflowOptions {
    /// Let the dataflow finish once every node has, treating
    /// `dora/timer/...` inputs as a clock rather than as work.
    ///
    /// `None` leaves the descriptor's own `exit_when_nodes_finish`
    /// setting alone; `Some(v)` overrides it in either direction, so a
    /// caller can force the policy off for a descriptor that asks for
    /// it.
    ///
    /// A timer input never closes, so by default a node consuming one is
    /// never told its inputs are done and the graph cannot end on its own
    /// (dora-rs/dora#2920). Off by default: for a long-lived dataflow the
    /// timer is exactly what keeps it alive.
    pub exit_when_nodes_finish: Option<bool>,
}

impl RunDataflowOptions {
    /// Sets [`Self::exit_when_nodes_finish`].
    ///
    /// A setter rather than a struct literal because the type is
    /// `#[non_exhaustive]`: callers outside this crate cannot construct it
    /// directly, which is what lets a future option be added without
    /// breaking them.
    pub fn exit_when_nodes_finish(mut self, exit_when_nodes_finish: bool) -> Self {
        self.exit_when_nodes_finish = Some(exit_when_nodes_finish);
        self
    }
}

/// A destroy waiting for the daemon's nodes to exit before it replies.
///
/// Holding the reply is what gives the coordinator — and therefore
/// `dora down` — its synchronization: `destroy_daemons` awaits it.
pub(crate) struct PendingDestroy {
    reply_tx: oneshot::Sender<Option<DaemonCoordinatorReply>>,
    wait: shutdown::DestroyWait,
}

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
    /// Loopback endpoint that the daemon's zenoh session listens on. Injected
    /// into spawned nodes via `DORA_ZENOH_CONNECT` so they can find their
    /// peer without multicast (#1778). `None` when the OS rejected the
    /// reservation; nodes then fall back to multicast scouting.
    pub(crate) zenoh_listen_endpoint: Option<String>,
    /// Whether this daemon opened its zenoh session without multicast
    /// scouting. Forwarded to spawned nodes so they discover the same way the
    /// daemon does (see `DORA_ZENOH_MULTICAST`).
    pub(crate) disable_multicast: bool,
    /// Whether this daemon runs in-process with whoever started it (the
    /// `run_dataflow` shape), making that process's death the end of the
    /// dataflow. Forwarded to spawned nodes via `DORA_RUN_PARENT_PID` so a
    /// `SIGKILL`ed parent does not strand them (dora-rs/dora#2856). False for
    /// the `dora up` daemon, whose nodes are deliberately decoupled from it
    /// (#2029).
    pub(crate) bind_nodes_to_parent: bool,
    /// A `Destroy` that is holding its reply until this daemon's node
    /// processes are gone (#2980).
    pub(crate) pending_destroy: Option<PendingDestroy>,
    pub(crate) zenoh_publish_tx: mpsc::Sender<ZenohOutbound>,
    pub(crate) zenoh_control_tx: mpsc::Sender<ZenohOutbound>,
    pub(crate) remote_daemon_events_tx:
        Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
    pub(crate) logger: DaemonLogger,
    pub(crate) sessions: BTreeMap<SessionId, BuildId>,
    pub(crate) builds: BTreeMap<BuildId, BuildInfo>,
    pub(crate) git_manager: GitManager,
    pub(crate) metrics_system: Arc<std::sync::Mutex<sysinfo::System>>,
    pub(crate) memory_pool: MemoryPoolManager,
    /// Nodes already warned about for sending after their dataflow
    /// finished, so `log_late_node_output` warns once each instead of
    /// once per message. See `MAX_WARNED_LATE_OUTPUT_NODES`.
    pub(crate) warned_late_outputs: HashSet<(DataflowId, NodeId)>,
}

/// Cap on `Daemon::warned_late_outputs`, so a daemon that serves many
/// dataflows cannot accumulate an entry per (dataflow, node) forever.
/// Reaching it clears the set rather than freezing it: a daemon is meant
/// to run indefinitely, and simply refusing new entries would silence
/// the warning permanently after ~1024 dataflows — including for the
/// stale-node case that is the reason it is a warning at all.
const MAX_WARNED_LATE_OUTPUT_NODES: usize = 1024;

type DaemonRunResult = BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>;

/// Whether a node-connection event belongs to a superseded incarnation.
///
/// Strictly-older comparison, deliberately not equality: generations are
/// globally monotonic, and the restart loop publishes a successor's
/// generation to the listener BEFORE spawning it, so a fast successor's
/// events can carry a generation NEWER than the entry until its
/// `ProcessHandleReplaced` is processed. Dropping those would eat the
/// successor's one-shot `Subscribe` (dora-rs/dora#2988 review, finding 1).
fn event_generation_is_stale(entry_generation: u64, event_generation: u64) -> bool {
    event_generation < entry_generation
}

/// Patch a stored descriptor entry with a replacement node's definition
/// (dora-rs/dora#2988 review, finding 2). The stored descriptor is
/// serialized into every spawned process's `DORA_NODE_CONFIG`, so the
/// replacement must not receive the outgoing incarnation's path, env, or
/// restart configuration through `DoraNode::dataflow_descriptor()`.
/// Only `CoreNodeKind::Custom` reaches here — ReplaceNode's v1 scope
/// rejects other kinds up front.
fn patch_descriptor_entry(
    entry: &mut dora_message::descriptor::Node,
    resolved: &dora_core::descriptor::ResolvedNode,
) {
    if let CoreNodeKind::Custom(custom) = &resolved.kind {
        entry.path = Some(custom.path.clone());
        entry.args = custom.args.clone();
        entry.env = resolved.env.clone();
        entry.inputs = custom.run_config.inputs.clone();
        entry.outputs = custom.run_config.outputs.clone();
        entry.restart_policy = custom.restart_policy;
        entry.max_restarts = custom.max_restarts;
        entry.restart_delay = custom.restart_delay;
        entry.max_restart_delay = custom.max_restart_delay;
        entry.restart_window = custom.restart_window;
        entry.health_check_timeout = custom.health_check_timeout;
        entry.finish_grace_secs = custom.finish_grace_secs;
        entry.send_stdout_as = custom.send_stdout_as.clone();
        entry.send_logs_as = custom.send_logs_as.clone();
        entry.min_log_level = custom.min_log_level.clone();
    }
}

fn clear_node_result(results: &mut DaemonRunResult, dataflow_id: Uuid, node_id: &NodeId) {
    let remove_dataflow = results.get_mut(&dataflow_id).is_some_and(|node_results| {
        node_results.remove(node_id);
        node_results.is_empty()
    });
    if remove_dataflow {
        results.remove(&dataflow_id);
    }
}

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
            let msg = match serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: daemon_id.clone(),
                    event: DaemonEvent::NodeMetrics {
                        dataflow_id: df.dataflow_id,
                        metrics,
                        network,
                    },
                },
                timestamp: clock.new_timestamp(),
            }) {
                Ok(msg) => msg,
                // Skip this dataflow's batch rather than `?`-returning: an early
                // return here would bypass the `System` restore below, leaving
                // the shared metrics `System` (moved out via `mem::take`) empty
                // until the next successful collection repopulates it.
                // Serialization of this structure is effectively infallible, so
                // this is defense-in-depth — `continue` just keeps the cleanup
                // path unconditional against any future error here. Matches the
                // `send_event` failure handling just below.
                Err(e) => {
                    tracing::warn!("failed to serialize metrics for dataflow: {e}");
                    continue;
                }
            };
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

/// Convert MemoryPoolMetadata into daemon-protocol MetadataParameters.
fn pool_metadata_to_params(meta: &MemoryPoolMetadata) -> MetadataParameters {
    use dora_message::metadata::Parameter;
    let mut p = MetadataParameters::new();
    p.insert("ptr".into(), Parameter::Integer(meta.ptr as i64));
    p.insert("size".into(), Parameter::Integer(meta.size as i64));
    p.insert("dtype".into(), Parameter::String(meta.dtype.clone()));
    let shape: Vec<i64> = meta.shape.iter().map(|&x| x as i64).collect();
    p.insert("shape".into(), Parameter::ListInt(shape));
    p.insert("is_pinned".into(), Parameter::Bool(meta.is_pinned));
    if let Some(ref t) = meta.pinned_type {
        p.insert("pinned_type".into(), Parameter::String(t.clone()));
    }
    if let Some(ref n) = meta.shared_memory_name {
        p.insert("shared_memory_name".into(), Parameter::String(n.clone()));
    }
    if let Some(ipc) = meta.ipc_present {
        p.insert("ipc_present".into(), Parameter::Bool(ipc));
    }
    if let Some(ref b) = meta.buffer_id {
        p.insert("buffer_id".into(), Parameter::String(b.clone()));
    }
    p
}

/// Reconstruct MemoryPoolMetadata from MetadataParameters (best-effort).
fn pool_metadata_from_params(params: &MetadataParameters) -> MemoryPoolMetadata {
    use dora_message::metadata::Parameter;
    let get_int = |k: &str| -> Option<i64> {
        params.get(k).and_then(|p| {
            if let Parameter::Integer(v) = p {
                Some(*v)
            } else {
                None
            }
        })
    };
    let get_str = |k: &str| -> Option<String> {
        params.get(k).and_then(|p| {
            if let Parameter::String(s) = p {
                Some(s.clone())
            } else {
                None
            }
        })
    };
    let get_bool = |k: &str| -> Option<bool> {
        params.get(k).and_then(|p| {
            if let Parameter::Bool(v) = p {
                Some(*v)
            } else {
                None
            }
        })
    };
    MemoryPoolMetadata {
        ptr: get_int("ptr").unwrap_or(0) as u64,
        size: get_int("size").unwrap_or(0) as usize,
        dtype: get_str("dtype").unwrap_or_default(),
        shape: params
            .get("shape")
            .and_then(|p| {
                if let Parameter::ListInt(v) = p {
                    Some(v.iter().map(|&x| x as usize).collect())
                } else {
                    None
                }
            })
            .unwrap_or_default(),
        is_pinned: get_bool("is_pinned").unwrap_or(false),
        shared_memory_name: get_str("shared_memory_name"),
        buffer_id: get_str("buffer_id"),
        ipc_present: get_bool("ipc_present"),
        pinned_type: get_str("pinned_type"),
    }
}

/// Release a memory pool and tell every other node that touched it to drop
/// its per-process view of that pool.
///
/// Both daemon entry points that release a pool must go through here: the
/// explicit `FreePinnedMemory` request and `ReadPinnedMemory { free: true }`.
/// The read-with-free path used to free the pool without notifying anyone,
/// so every other node kept its GPU buffer, pinned host buffer, IPC handle
/// and shmem mapping for the rest of its life (#2935).
fn free_pool_and_notify(
    memory_pool: &MemoryPoolManager,
    dataflow: Option<&RunningDataflow>,
    id: &MemoryPoolId,
    initiator: &str,
    clock: &HLC,
) -> Result<(), String> {
    let (_metadata, touched) = memory_pool.free_memory_pool(id, initiator)?;

    // Without a running dataflow there are no subscribe channels to notify;
    // the pool table entry and its shmem segment are released either way.
    let Some(dataflow) = dataflow else {
        return Ok(());
    };

    let undelivered = notify_memory_pool_freed(dataflow, &id.id, &touched, initiator, clock);
    if !undelivered.is_empty() {
        tracing::error!(
            pool = %id.id,
            nodes = ?undelivered,
            "FreeMemoryPool notification dropped (event channel full); these \
             nodes hold their GPU/host/IPC/shmem resources for the pool until \
             they exit"
        );
    }
    Ok(())
}

/// Send `NodeEvent::FreeMemoryPool` to every node that registered or read
/// the pool, except the node that initiated the free — it releases its own
/// resources synchronously.
///
/// Returns the nodes whose notification could not be enqueued because their
/// event channel was full. Those nodes leak the pool's per-process
/// resources, so the caller surfaces them. Delivery is not retried: this
/// runs on the daemon main loop with the freeing node blocked on its reply,
/// so awaiting a backed-up receiver would stall every dataflow on the daemon.
fn notify_memory_pool_freed(
    dataflow: &RunningDataflow,
    shared_memory_id: &str,
    touched: &HashSet<String>,
    initiator: &str,
    clock: &HLC,
) -> BTreeSet<NodeId> {
    let mut undelivered = BTreeSet::new();
    for (node, channel) in &dataflow.subscribe_channels {
        if !touched.contains(node.as_ref()) || node.as_ref() == initiator {
            continue;
        }
        let event = NodeEvent::FreeMemoryPool {
            shared_memory_id: shared_memory_id.to_owned(),
        };
        match send_with_timestamp(channel, event, clock) {
            Ok(true) => {
                // The listener decrements `pending_messages` for every event
                // it drains, so an enqueue without the matching increment
                // underflows the reported count to `u64::MAX` (#2827).
                dataflow.inc_pending(node);
            }
            // A full channel is reported as `Ok(false)`, not `Err` — the
            // original loop inspected only `Err` and dropped this case
            // silently (#2935).
            Ok(false) => {
                undelivered.insert(node.clone());
            }
            // Channel closed: the node is gone, so the OS already reclaimed
            // the resources the notification would have released.
            Err(_) => tracing::debug!(
                node_id = %node,
                pool = %shared_memory_id,
                "skipping FreeMemoryPool notification: node disconnected"
            ),
        }
    }
    undelivered
}

#[cfg(test)]
mod metadata_roundtrip_tests {
    use super::*;
    use dora_memory_pool::MemoryPoolMetadata;

    /// `ipc_present` must survive a to_params → from_params round-trip
    /// so the read path receives the trusted flag from daemon metadata.
    #[test]
    fn ipc_present_survives_roundtrip_true() {
        let meta = MemoryPoolMetadata {
            ipc_present: Some(true),
            ..Default::default()
        };
        let params = pool_metadata_to_params(&meta);
        let restored = pool_metadata_from_params(&params);
        assert_eq!(restored.ipc_present, Some(true));
    }

    #[test]
    fn ipc_present_survives_roundtrip_false() {
        let meta = MemoryPoolMetadata {
            ipc_present: Some(false),
            ..Default::default()
        };
        let params = pool_metadata_to_params(&meta);
        let restored = pool_metadata_from_params(&params);
        assert_eq!(restored.ipc_present, Some(false));
    }

    #[test]
    fn ipc_present_survives_roundtrip_none() {
        let meta = MemoryPoolMetadata {
            ipc_present: None,
            ..Default::default()
        };
        let params = pool_metadata_to_params(&meta);
        let restored = pool_metadata_from_params(&params);
        assert_eq!(restored.ipc_present, None);
    }
}

/// Where the daemon's zenoh listen address came from, which decides what a bind
/// failure means.
///
/// A derived address is a best guess, so failing to bind it may fall back to
/// multicast scouting. An address the operator *named* must bind or the daemon
/// must exit: they are on a network where they had to say the address out loud,
/// which is precisely a network where multicast is not going to rescue us — and
/// a daemon with no listener is undialable and silently dead, which is the
/// failure this whole mechanism exists to prevent.
#[derive(Debug, Clone, Copy)]
enum ZenohBind {
    /// Derived from the coordinator address; a bind failure can fall back.
    Derived(IpAddr),
    /// Named via `--zenoh-listen`; a bind failure is fatal.
    Explicit(IpAddr),
}

impl ZenohBind {
    fn addr(self) -> IpAddr {
        match self {
            Self::Derived(addr) | Self::Explicit(addr) => addr,
        }
    }

    fn is_explicit(self) -> bool {
        matches!(self, Self::Explicit(_))
    }
}

/// Report whether `zenoh_bind` can be reached by remote daemons, and reject a
/// configuration that would run silently undialable.
///
/// A loopback listener advertises `127.0.0.1` to peers, who dial their own
/// loopback and reach nothing — and since zenoh 1.9 peers do not relay, that
/// pair is dead with no fallback. When the coordinator is remote (so other
/// daemons are expected):
///
/// * an *explicit* loopback address is a hard error — the operator named it, and
///   the [`ZenohBind::Explicit`] contract is "bind a routable address or exit",
///   the same "nothing to advertise" reason [`validate_zenoh_listen`] already
///   rejects the wildcard for;
/// * a *derived* loopback only warns — it is a best-effort fallback the operator
///   can override with `--zenoh-listen`.
///
/// A routable bind is announced at info level; a single-machine (loopback
/// coordinator) setup is silent.
fn announce_zenoh_bind(zenoh_bind: ZenohBind, coordinator_ws_addr: SocketAddr) -> eyre::Result<()> {
    if zenoh_bind.addr().is_loopback() && !coordinator_ws_addr.ip().is_loopback() {
        match zenoh_bind {
            ZenohBind::Explicit(addr) => {
                eyre::bail!(
                    "--zenoh-listen {addr} is a loopback address, but the coordinator \
                     at {coordinator_ws_addr} is remote, so other daemons must be able \
                     to reach this one. A loopback listener advertises {addr} to peers, \
                     who would dial their own loopback and reach nothing. Pass the \
                     address other daemons should use to reach this host (e.g. its LAN \
                     or VPN address)."
                );
            }
            ZenohBind::Derived(_) => {
                tracing::warn!(
                    "coordinator at {coordinator_ws_addr} is remote, but no routable local \
                     address toward it was found; zenoh will bind loopback and other daemons \
                     will not be able to reach this one. Pass --zenoh-listen <IP> explicitly."
                );
            }
        }
    } else if !zenoh_bind.addr().is_loopback() {
        tracing::info!(
            "zenoh listener binding {} ({}); this port accepts connections from \
             other hosts",
            zenoh_bind.addr(),
            match zenoh_bind {
                ZenohBind::Explicit(_) => "given via --zenoh-listen".to_string(),
                ZenohBind::Derived(_) =>
                    format!("derived from coordinator address {coordinator_ws_addr}"),
            }
        );
    }
    Ok(())
}

impl Daemon {
    /// Runs the daemon. `zenoh_listen_addr` overrides the address this
    /// daemon's zenoh listener binds, and therefore the locator its peers are
    /// told to dial.
    ///
    /// Leave it `None` to derive it from `coordinator_ws_addr`, which is correct
    /// whenever the daemons reach each other over the same network they reach
    /// the coordinator over — a LAN, or a mesh VPN. Set it explicitly on a
    /// multi-homed host that would otherwise advertise the wrong interface. An
    /// address given here must bind: unlike the derived one, failing to bind it
    /// is fatal rather than a fallback.
    pub async fn run_with_zenoh_listen(
        coordinator_ws_addr: SocketAddr,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        local_listen_port: u16,
        inter_daemon_peer: Option<String>,
        zenoh_listen_addr: Option<IpAddr>,
        disable_multicast: bool,
    ) -> eyre::Result<()> {
        Self::run_inner_with_builds(
            coordinator_ws_addr,
            machine_id,
            labels,
            local_listen_port,
            inter_daemon_peer,
            zenoh_listen_addr,
            disable_multicast,
            Default::default(),
        )
        .await
    }

    #[allow(clippy::too_many_arguments)]
    async fn run_inner_with_builds(
        coordinator_ws_addr: SocketAddr,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        local_listen_port: u16,
        inter_daemon_peer: Option<String>,
        zenoh_listen_addr: Option<IpAddr>,
        disable_multicast: bool,
        initial_builds: BTreeMap<BuildId, BuildInfo>,
    ) -> eyre::Result<()> {
        let zenoh_bind = match zenoh_listen_addr {
            Some(addr) => {
                validate_zenoh_listen(addr).wrap_err(
                    "invalid --zenoh-listen address (omit the flag to derive it \
                     from --coordinator-addr)",
                )?;
                ZenohBind::Explicit(addr)
            }
            None => ZenohBind::Derived(zenoh_bind_address_for(coordinator_ws_addr)),
        };
        // Fail fast on an address that does not exist on this host (a typo'd
        // digit gives EADDRNOTAVAIL). The reservation in `build_daemon` is the
        // real guard, but that only runs once the coordinator connects — which
        // may be many retries away, or never — so without this probe a typo
        // looks like a connectivity problem for minutes before it looks like a
        // typo. Whether the address exists is not racy, so checking it early
        // costs nothing but a bind and a drop.
        if let ZenohBind::Explicit(addr) = zenoh_bind {
            std::net::TcpListener::bind((addr, 0))
                .map(drop)
                .wrap_err_with(|| {
                    format!(
                        "cannot bind the zenoh listen address {addr} given via \
                         --zenoh-listen; it does not appear to exist on this host"
                    )
                })?;
        }
        announce_zenoh_bind(zenoh_bind, coordinator_ws_addr)?;
        let clock = Arc::new(HLC::default());
        let mut ctrlc_events = set_up_ctrlc_handler(clock.clone())?;
        // Tracks whether we've ever connected to the coordinator. The initial
        // connect is left to set_up_event_stream's own retry loop (the
        // coordinator may simply not be up yet at startup), but once connected,
        // a coordinator gone past the reconnect timeout is treated as
        // permanently gone -> exit instead of orphaning (dora-rs/dora#1996).
        let mut connected_once = false;
        // Deadline for the current run of consecutive failed reconnects. Set on
        // the first failure after a successful connection and cleared whenever
        // we reconnect, so each fresh outage gets a full
        // `COORDINATOR_RECONNECT_RETRY_WINDOW` before the daemon exits.
        let mut reconnect_deadline: Option<Instant> = None;

        // Node-serving daemon state. Built lazily on the first successful
        // connect, then reused across every reconnect so that running nodes —
        // and the `ProcessHandle`s that would otherwise kill them on drop — are
        // never torn down when the coordinator connection drops
        // (dora-rs/dora#2029). Each reconnect refreshes only the per-connection
        // fields (coordinator sender, inter-daemon event sender, heartbeat
        // clock, log destination); everything else persists.
        let mut daemon: Option<Daemon> = None;
        let mut dora_events_rx: Option<mpsc::Receiver<Timestamped<Event>>> = None;

        // Dynamic-node listener: bind once for the daemon's lifetime, not once
        // per reconnect. Rebinding on every reconnect leaked the listener task
        // and hit AddrInUse on the second bind, silently dropping dynamic-node
        // support (dora-rs/dora#1999). Keep `dynamic_node_events_rx` alive for
        // the whole loop so the channel stays open across reconnect gaps; each
        // iteration streams a clone of it.
        let (dynamic_node_events_tx, dynamic_node_events_rx) = flume::bounded(10);
        let _listen_port = local_listener::spawn_listener_loop(
            (LOCALHOST, local_listen_port).into(),
            dynamic_node_events_tx,
        )
        .await?;

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
                    dynamic_node_events_rx.clone(),
                );

                // Bound reconnects (but not the initial connect) so a
                // permanently gone coordinator makes the daemon exit rather than
                // orphan; see `connected_once` above. A reachable coordinator
                // reconnects well within the timeout, so legitimate reconnection
                // (e.g. the daemon-reconnect test) is unaffected.
                let connect = async move {
                    if connected_once {
                        tokio::time::timeout(COORDINATOR_RECONNECT_TIMEOUT, incoming_events).await
                    } else {
                        Ok(incoming_events.await)
                    }
                };

                let ctrl_c = pin!(ctrlc_events.recv());
                match futures::future::select(ctrl_c, pin!(connect)).await {
                    future::Either::Left((_ctrl_c, _)) => {
                        tracing::info!("received ctrl-c signal -> stopping daemon");
                        return Ok(());
                    }
                    future::Either::Right((Ok(events), _)) => events,
                    future::Either::Right((Err(_elapsed), _)) => {
                        return Err(eyre::eyre!(
                            "coordinator unreachable after \
                             {COORDINATOR_RECONNECT_TIMEOUT:?}; daemon exiting"
                        ));
                    }
                }
            };

            match connect_result {
                Ok((daemon_id, coordinator_sender, incoming_events)) => {
                    connected_once = true;
                    // Fresh successful connection: a later disconnect starts a
                    // new retry window rather than inheriting an old deadline.
                    reconnect_deadline = None;

                    // Build the daemon on the first connect; on later connects
                    // just point the existing daemon at the new connection. The
                    // node-serving state (running nodes, zenoh session, internal
                    // event channel) is preserved either way.
                    match daemon.as_mut() {
                        None => {
                            // The coordinator log destination wraps a shared,
                            // swappable target so per-node log-forwarding clones
                            // follow later reconnects (#2029 P2).
                            let log_destination = LogDestination::Coordinator {
                                target: CoordinatorLogTarget::shared(
                                    coordinator_sender.clone(),
                                    daemon_id.clone(),
                                ),
                            };
                            let (built, rx) = Self::build_daemon(
                                Some(coordinator_sender),
                                daemon_id,
                                None,
                                clock.clone(),
                                Some(remote_daemon_events_tx),
                                initial_builds.clone(),
                                log_destination,
                                inter_daemon_peer.clone(),
                                zenoh_bind,
                                disable_multicast,
                                // A standalone daemon outlives nothing its
                                // nodes depend on: they survive coordinator
                                // drops and reconnects on purpose (#2029).
                                false,
                            )
                            .await?;
                            daemon = Some(built);
                            dora_events_rx = Some(rx);
                        }
                        Some(d) => {
                            // Adopt the daemon id the coordinator assigned on
                            // this (re)registration. The coordinator allocates a
                            // fresh id whenever it has no live record of us (a
                            // reconnect always drops the old connection first, and
                            // a coordinator restart wipes its memory), and it
                            // rejects any event carrying a different id. Running
                            // nodes are unaffected: their data plane is keyed by
                            // dataflow/node/output, not by daemon id.
                            d.daemon_id = daemon_id.clone();
                            d.logger.set_daemon_id(daemon_id.clone());
                            // Swap the shared coordinator log target in place so
                            // surviving nodes' log-forwarding clones (captured at
                            // spawn) emit over this new connection with the
                            // freshly-assigned id (#2029 P2).
                            d.logger
                                .update_coordinator_target(coordinator_sender.clone(), daemon_id);
                            d.coordinator_sender = Some(coordinator_sender);
                            d.remote_daemon_events_tx = Some(remote_daemon_events_tx);
                            d.last_coordinator_heartbeat = Instant::now();
                        }
                    }

                    let d = daemon.as_mut().expect("daemon present");
                    let rx = dora_events_rx.as_mut().expect("dora_events_rx present");

                    // Don't pass ctrlc_events into run_inner — keep it in the
                    // outer loop so Ctrl+C works across reconnect cycles.
                    // (ctrlc::set_handler can only be called once per process)
                    let result = tokio::select! {
                        r = d.run_inner(incoming_events, rx, None) => r,
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
                    if connected_once {
                        // Connected before but this attempt failed. A fast TCP
                        // refusal (coordinator briefly restarting) fails almost
                        // instantly and is not bounded by the connect timeout, so
                        // retry within a window instead of exiting on the first
                        // refusal (dora-rs/dora#1998). Only once the window
                        // elapses do we treat the coordinator as permanently gone
                        // and exit rather than orphan (dora-rs/dora#1996).
                        if reconnect_window_elapsed(
                            Instant::now(),
                            &mut reconnect_deadline,
                            COORDINATOR_RECONNECT_RETRY_WINDOW,
                        ) {
                            return Err(eyre::eyre!(
                                "failed to reconnect to coordinator within \
                                 {COORDINATOR_RECONNECT_RETRY_WINDOW:?}: {e:#}; \
                                 daemon exiting"
                            ));
                        }
                        tracing::warn!("failed to reconnect to coordinator: {e:#}; retrying");
                    } else {
                        // Still waiting for the initial connect: keep retrying,
                        // the coordinator may not be up yet.
                        tracing::warn!("waiting for coordinator: {e:#}");
                    }
                }
            }

            // Reached while waiting for the initial connect, or after a mid-run
            // disconnect (to reconnect). Pause briefly before the next attempt.
            tracing::info!("retrying in {COORDINATOR_RECONNECT_BACKOFF:?}...");
            tokio::time::sleep(COORDINATOR_RECONNECT_BACKOFF).await;
        }
    }

    #[allow(clippy::too_many_arguments)]
    /// Runs a single dataflow to completion with the default options.
    ///
    /// Signature deliberately unchanged: `dora-daemon` is published, so
    /// adding a parameter here would break every downstream caller. New
    /// options belong on [`RunDataflowOptions`], which is passed to
    /// [`Daemon::run_dataflow_with`] and can grow without breaking anyone.
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
        working_dir_override: Option<PathBuf>,
        descriptor_override: Option<Descriptor>,
    ) -> eyre::Result<DataflowResult> {
        Self::run_dataflow_with(
            dataflow_path,
            build_id,
            local_build,
            session_id,
            uv,
            log_destination,
            write_events_to,
            stop_after,
            debug,
            working_dir_override,
            descriptor_override,
            RunDataflowOptions::default(),
        )
        .await
    }

    /// Runs a single dataflow to completion with explicit options.
    #[allow(clippy::too_many_arguments)]
    pub async fn run_dataflow_with(
        dataflow_path: &Path,
        build_id: Option<BuildId>,
        local_build: Option<BuildInfo>,
        session_id: SessionId,
        uv: bool,
        log_destination: LogDestination,
        write_events_to: Option<PathBuf>,
        stop_after: Option<Duration>,
        debug: bool,
        working_dir_override: Option<PathBuf>,
        descriptor_override: Option<Descriptor>,
        options: RunDataflowOptions,
    ) -> eyre::Result<DataflowResult> {
        let RunDataflowOptions {
            exit_when_nodes_finish,
        } = options;
        let working_dir = match working_dir_override {
            Some(p) => p
                .canonicalize()
                .context("failed to canonicalize working_dir override")?,
            None => dataflow_path
                .canonicalize()
                .context("failed to canonicalize dataflow path")?
                .parent()
                .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
                .to_owned(),
        };

        // `hub:` dataflows are desugared in memory by `dora build` — the
        // on-disk YAML still contains unresolved references, so the caller
        // passes the resolved descriptor from the dataflow session instead
        let raw_descriptor = match descriptor_override {
            Some(descriptor) => descriptor,
            None => read_as_descriptor(dataflow_path).await?,
        };
        // Expand module composition (must run before resolution; module
        // nodes cause `resolve_aliases_and_set_defaults` to fail otherwise).
        let mut descriptor = raw_descriptor
            .expand(&working_dir)
            .wrap_err("failed to expand modules in dataflow descriptor")?;
        if debug {
            descriptor.debug.enable_debug_inspection = true;
        }
        // Fold the option into the descriptor, which is what the daemon
        // actually reads. `dora start` sets the same field from its own
        // flag, so both entry points converge on one source of truth
        // rather than each carrying the setting separately (#2920).
        // Set explicitly, so it overrides the descriptor either way; left
        // unset, the descriptor's own setting stands.
        descriptor.apply_exit_when_nodes_finish(exit_when_nodes_finish);
        if let Some(node) = descriptor.nodes.iter().find(|n| n.deploy.is_some()) {
            eyre::bail!(
                "node {} has a `deploy` section, which is not supported in `dora run`\n\n
                Instead, you need to spawn a `dora coordinator` and one or more `dora daemon`
                instances and then use `dora start`.",
                node.id
            )
        }

        validate::check_dataflow(&descriptor, &working_dir)
            .wrap_err("Dataflow could not be validated.")?;
        let health_check_interval = descriptor
            .health_check_interval
            .map(Duration::from_secs_f64);
        let nodes = descriptor.resolve_aliases_and_set_defaults()?;

        let (events_tx, events_rx) = flume::bounded(10);
        let has_dynamic_nodes = nodes
            .iter()
            .any(|(_n, resolved_nodes)| resolved_nodes.kind.dynamic());
        if has_dynamic_nodes {
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
        let termination_events = ReceiverStream::new(set_up_termination_handler(clock.clone()));

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
            termination_events,
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
            // Local dataflow runs (one daemon, no cluster) never need
            // cross-daemon Zenoh discovery; the rendezvous is irrelevant.
            None,
            // `dora run` is single-machine by construction, and every node it
            // spawns is handed `DORA_ZENOH_CONNECT`, so all links are explicit
            // and multicast scouting buys nothing — while still exposing us to
            // a scouting bind that fails on a busy DDS/ROS2 network.
            //
            // Dynamic nodes are the exception: they are started by the user in
            // a separate process, inherit none of the daemon's environment, and
            // so have no endpoint to dial. Multicast is the only way they and
            // the daemon find each other, so keep it on when the descriptor
            // declares any.
            !has_dynamic_nodes,
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
        inter_daemon_peer: Option<String>,
        disable_multicast: bool,
    ) -> eyre::Result<DaemonRunResult> {
        // Single-shot path (`dora run`): build the daemon and run one event
        // loop. The reconnecting daemon binary instead builds the daemon once
        // and reuses it across reconnects (see `run_inner_with_builds`), so that node
        // processes are not killed when the coordinator connection drops.
        // `dora run` is single-machine by construction: the daemon, its nodes
        // and the in-process coordinator all live on this host, so loopback is
        // both sufficient and the least exposed choice.
        //
        // Being that single process is also why the nodes are bound to it: it
        // is their parent, and its death — `SIGKILL` included, which no
        // teardown code of ours survives — is the end of the dataflow (#2856).
        let bind_nodes_to_parent = true;
        let (mut daemon, mut dora_events_rx) = Self::build_daemon(
            coordinator_sender,
            daemon_id,
            exit_when_done,
            clock,
            remote_daemon_events_tx,
            builds,
            log_destination,
            inter_daemon_peer,
            ZenohBind::Derived(LOCALHOST),
            disable_multicast,
            bind_nodes_to_parent,
        )
        .await?;
        daemon
            .run_inner(
                external_events,
                &mut dora_events_rx,
                health_check_interval_duration,
            )
            .await
    }

    /// Construct the node-serving daemon state: open the zenoh session, spawn
    /// the publish-drain task, and assemble the `Daemon`. Returns the daemon
    /// plus the receiver half of its internal event channel (`events_tx`),
    /// which `run_inner` folds into the merged event stream.
    ///
    /// Split out from `run_inner` so the reconnecting daemon can build this
    /// state **once** and keep it alive across coordinator reconnects. Dropping
    /// it would drop every `ProcessHandle` and kill all running nodes — the bug
    /// fixed in dora-rs/dora#2029.
    #[allow(clippy::too_many_arguments)]
    async fn build_daemon(
        coordinator_sender: Option<coordinator::CoordinatorSender>,
        daemon_id: DaemonId,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        clock: Arc<HLC>,
        remote_daemon_events_tx: Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
        builds: BTreeMap<BuildId, BuildInfo>,
        log_destination: LogDestination,
        inter_daemon_peer: Option<String>,
        zenoh_bind: ZenohBind,
        disable_multicast: bool,
        bind_nodes_to_parent: bool,
    ) -> eyre::Result<(Self, mpsc::Receiver<Timestamped<Event>>)> {
        // Fold in `DORA_ZENOH_MULTICAST` so this is the daemon's *effective*
        // decision, not just its flag. The zenoh session honors the variable on
        // its own, but the spawner forwards this field to nodes — and nodes no
        // longer inherit the variable — so a daemon started with the variable
        // instead of `--zenoh-no-multicast` would otherwise leave its nodes
        // scouting by multicast (#2991 review).
        let disable_multicast = dora_core::topics::multicast_disabled(disable_multicast);

        // Reserve a port and have zenoh listen on it. The endpoint is injected
        // into spawned nodes via `DORA_ZENOH_CONNECT` so peer discovery works
        // without multicast (#1778).
        //
        // `zenoh_bind` is loopback for single-machine deployments and an
        // address on the coordinator's network otherwise. It is not merely a
        // question of which interface accepts connections: zenoh advertises the
        // address it bound as its locator, and remote daemons dial exactly
        // that. A daemon bound to loopback therefore tells its peers to dial
        // `127.0.0.1` — their own loopback — where they reach nothing. Since
        // zenoh 1.9 peers do not relay for each other, such a pair has no
        // fallback path and simply never exchanges data.
        //
        // Falling back to multicast when the reservation fails is only defensible
        // for a *derived* address: loopback always exists, so the error was
        // effectively unreachable. An operator-supplied address can genuinely
        // fail — a typo'd digit gives EADDRNOTAVAIL — and continuing would leave
        // the daemon with no listener at all: undialable, silently dead, which is
        // the exact failure this code exists to prevent. Someone who named an
        // address explicitly is also, almost by definition, on a network where
        // multicast will not save them. So: fatal when explicit.
        let requested_listen_endpoint = match reserve_zenoh_endpoint(zenoh_bind.addr()) {
            Ok(ep) => Some(ep),
            Err(err) if zenoh_bind.is_explicit() => {
                return Err(err).wrap_err_with(|| {
                    format!(
                        "failed to bind the zenoh listen address {} given via \
                         --zenoh-listen; other daemons would have no way to reach this \
                         one. Check that this address exists on this host",
                        zenoh_bind.addr()
                    )
                });
            }
            Err(err) => {
                tracing::warn!(
                    "failed to reserve zenoh listen endpoint on {}: {err}; \
                     falling back to multicast scouting only",
                    zenoh_bind.addr()
                );
                None
            }
        };
        // The helper is the source of truth for whether the listener actually
        // bound: `zenoh_listen_endpoint` only becomes `Some` if zenoh accepted
        // the listen/endpoints insert. Otherwise we must not inject
        // `DORA_ZENOH_CONNECT` into spawned nodes — they would try to connect
        // to an endpoint that nothing is listening on (#1856).
        let (zenoh_session, zenoh_listen_endpoint) = open_zenoh_session_with_listen(
            None,
            requested_listen_endpoint.as_deref(),
            inter_daemon_peer.as_deref(),
            if disable_multicast {
                MulticastScouting::Disabled
            } else {
                MulticastScouting::Allowed
            },
        )
        .await
        .wrap_err("failed to open zenoh session")?;
        if requested_listen_endpoint.is_some() && zenoh_listen_endpoint.is_none() {
            // Same argument as the reservation above: an address the operator
            // named must actually be listening, or this daemon is unreachable
            // and nothing will tell them.
            if zenoh_bind.is_explicit() {
                eyre::bail!(
                    "zenoh did not bind the listen address {} given via --zenoh-listen; \
                     other daemons would have no way to reach this one",
                    zenoh_bind.addr()
                );
            }
            tracing::warn!(
                "requested zenoh listener but zenoh did not bind it; \
                 spawned nodes will use multicast scouting only"
            );
        }
        // Use a large channel capacity to prevent deadlock
        let (dora_events_tx, dora_events_rx) = mpsc::channel(1000);

        // Zenoh publish drain tasks: offload `.put().await` from the main event
        // loop. Regular outputs use a best-effort data queue; lifecycle control
        // events first enter a bounded pending queue, then a background
        // forwarder waits for capacity in a dedicated control publish queue.
        let (zenoh_publish_tx, mut zenoh_publish_rx) =
            mpsc::channel::<ZenohOutbound>(ZENOH_PUBLISH_CHANNEL_CAPACITY);
        let (zenoh_control_publish_tx, mut zenoh_control_publish_rx) =
            mpsc::channel::<ZenohOutbound>(ZENOH_CONTROL_CHANNEL_CAPACITY);
        let (zenoh_control_tx, mut zenoh_control_rx) =
            mpsc::channel::<ZenohOutbound>(ZENOH_CONTROL_PENDING_CHANNEL_CAPACITY);
        let _zenoh_control_forwarder_handle = tokio::spawn(async move {
            while let Some(msg) = zenoh_control_rx.recv().await {
                if zenoh_control_publish_tx.send(msg).await.is_err() {
                    tracing::debug!("zenoh control publish queue closed");
                    break;
                }
            }
            tracing::debug!("zenoh control forwarder task exiting");
        });
        let _zenoh_drain_handle = tokio::spawn(async move {
            let mut control_open = true;
            let mut data_open = true;
            while control_open || data_open {
                tokio::select! {
                    biased;
                    msg = zenoh_control_publish_rx.recv(), if control_open => {
                        match msg {
                            Some(msg) => publish_zenoh_outbound(msg).await,
                            None => control_open = false,
                        }
                    }
                    msg = zenoh_publish_rx.recv(), if data_open => {
                        match msg {
                            Some(msg) => publish_zenoh_outbound(msg).await,
                            None => data_open = false,
                        }
                    }
                }
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
            warned_late_outputs: HashSet::new(),
            clock,
            ft_stats: Default::default(),
            zenoh_session,
            zenoh_listen_endpoint,
            disable_multicast,
            bind_nodes_to_parent,
            pending_destroy: None,
            zenoh_publish_tx,
            zenoh_control_tx,
            remote_daemon_events_tx,
            git_manager: Default::default(),
            memory_pool: MemoryPoolManager::new(),
            builds,
            sessions: Default::default(),
            metrics_system: Arc::new(std::sync::Mutex::new(sysinfo::System::new())),
        };

        Ok((daemon, dora_events_rx))
    }

    /// Run the daemon event loop for one coordinator connection.
    ///
    /// Borrows `&mut self` (rather than consuming) so that returning on a
    /// reconnect-triggering error (heartbeat timeout / coordinator-send
    /// failure) leaves the node-serving state — including every node's
    /// `ProcessHandle` — intact. The caller's reconnect loop then re-enters
    /// with a fresh connection and the same running nodes (dora-rs/dora#2029).
    ///
    /// `dora_events_rx` (the receiver half of `self.events_tx`, into which node
    /// listeners push events) is likewise owned by the caller and borrowed for
    /// the connection's lifetime, so buffered node events survive a reconnect.
    #[tracing::instrument(skip(external_events, dora_events_rx, self), fields(?self.daemon_id))]
    async fn run_inner(
        &mut self,
        external_events: impl Stream<Item = Timestamped<Event>> + Unpin,
        dora_events_rx: &mut mpsc::Receiver<Timestamped<Event>>,
        health_check_interval_duration: Option<Duration>,
    ) -> eyre::Result<DaemonRunResult> {
        // Borrow the persistent node-event receiver as a stream for this
        // connection. The borrow ends when this function returns, so the
        // caller can re-borrow it on the next reconnect iteration.
        let dora_events = stream::poll_fn(|cx| dora_events_rx.poll_recv(cx));

        let watchdog_clock = self.clock.clone();
        let watchdog_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(5),
        ))
        .map(move |_| Timestamped {
            inner: Event::HeartbeatInterval,
            timestamp: watchdog_clock.new_timestamp(),
        });

        let metrics_clock = self.clock.clone();
        let metrics_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            METRICS_INTERVAL, // Collect metrics every 2 seconds
        ))
        .map(move |_| Timestamped {
            inner: Event::MetricsInterval,
            timestamp: metrics_clock.new_timestamp(),
        });

        let health_check_clock = self.clock.clone();
        // `tokio::time::interval` panics on a zero period. Descriptor validation
        // already rejects a zero `health_check_interval` (see `check_dataflow`),
        // but guard here as well so a stray `Duration::ZERO` falls back to the
        // default rather than crashing the daemon (defense-in-depth for #2752).
        let health_check_period = health_check_interval_duration
            .filter(|d| !d.is_zero())
            .unwrap_or(Duration::from_secs(5));
        let health_check_interval =
            tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(health_check_period))
                .map(move |_| Timestamped {
                    inner: Event::NodeHealthCheckInterval,
                    timestamp: health_check_clock.new_timestamp(),
                });

        let mut events = (
            external_events,
            dora_events,
            watchdog_interval,
            metrics_interval,
            health_check_interval,
        )
            .merge();

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
                Event::DebugTopicData {
                    dataflow_id,
                    output_id,
                    metadata,
                    data,
                } => {
                    self.handle_debug_topic_data(dataflow_id, output_id, metadata, data)
                        .await?;
                }
                Event::Node {
                    dataflow_id: dataflow,
                    node_id,
                    generation,
                    event,
                } => {
                    // Drop control events from a superseded incarnation: a
                    // replaced or re-added id's old connection must not
                    // mutate the current entry (close its outputs, remove
                    // its subscription, ...) — dora-rs/dora#2926, #2927.
                    // STRICTLY older only: the restart loop publishes the
                    // successor's generation before spawning it, so a fast
                    // successor can connect (and Subscribe) while the entry
                    // still holds the predecessor's generation — a NEWER
                    // event generation is that successor racing its own
                    // `ProcessHandleReplaced` and must not be dropped, or
                    // its one-shot Subscribe is lost and the incarnation
                    // stays disconnected. Entry-absent passes through:
                    // during startup a node can register before its
                    // RunningNode is inserted, and the pending-nodes
                    // barrier owns that window.
                    let superseded = self
                        .running
                        .get(&dataflow)
                        .and_then(|df| df.running_nodes.get(&node_id))
                        .is_some_and(|node| event_generation_is_stale(node.generation, generation));
                    if superseded {
                        tracing::debug!(
                            %dataflow,
                            %node_id,
                            generation,
                            "dropping node event from superseded incarnation"
                        );
                    } else {
                        self.handle_node_event(event, dataflow, node_id).await?
                    }
                }
                Event::Dora(event) => self.handle_dora_event(event).await?,
                Event::DynamicNode(event) => self.handle_dynamic_node_event(event).await?,
                Event::DestroyTick => {
                    if self.handle_destroy_tick().await {
                        break;
                    }
                }
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
                            // Return error to trigger the reconnection loop in
                            // `run_inner_with_builds`. Because `run_inner` borrows
                            // `&mut self`, this error does NOT drop the daemon:
                            // running nodes and their `ProcessHandle`s survive,
                            // and the next reconnect re-adopts them
                            // (dora-rs/dora#2029).
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
                    self.check_finish_stragglers();
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
                    Ok(mut running_node) => {
                        if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                            // Open the restart-loop gate before the insert:
                            // the loop's first events are queued behind this
                            // handler on the same event loop, so they cannot
                            // be processed before the entry is registered.
                            running_node.mark_registered();
                            dataflow.running_nodes.insert(node_id.clone(), running_node);
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
                        // Error arm: surface as Failed (not Stopped).
                        self.handle_node_stop(dataflow_id, &node_id, dynamic_node, false)
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

        // Clean up any unfreed memory pool entries on daemon exit
        if let Err(errors) = self.memory_pool.cleanup_all() {
            for error in errors {
                tracing::warn!("{error}");
            }
        }

        // `run_inner` borrows `&mut self`, so move the accumulated results out
        // (the daemon may be reused for a reconnect, where these are ignored).
        Ok(std::mem::take(&mut self.dataflow_node_results))
    }

    /// Hold a destroy until this daemon's nodes are gone, or hand the reply
    /// channel back when there is nothing to wait for.
    ///
    /// Returning the channel means "reply and exit now"; keeping it means the
    /// event loop stays alive, serving the shutdown handshakes of nodes that
    /// are on their way out, until [`Self::handle_destroy_tick`] decides.
    fn begin_pending_destroy(
        &mut self,
        reply_tx: oneshot::Sender<Option<DaemonCoordinatorReply>>,
    ) -> Option<oneshot::Sender<Option<DaemonCoordinatorReply>>> {
        if self.pending_destroy.is_some() {
            // A second destroy while the first is still waiting: answer it,
            // but let the original wait finish rather than exiting on top of
            // it and orphaning the nodes it is still watching.
            tokio::spawn(Self::finish_destroy(reply_tx));
            return None;
        }
        if self.running_node_pids().is_empty() {
            return Some(reply_tx);
        }

        self.pending_destroy = Some(PendingDestroy {
            reply_tx,
            wait: shutdown::DestroyWait::new(),
        });
        // Nothing else wakes the loop once the dataflows are stopping, so the
        // wait needs its own tick. The task ends with the daemon, when the
        // event channel closes.
        let events_tx = self.events_tx.clone();
        let clock = self.clock.clone();
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(shutdown::POLL_INTERVAL).await;
                let tick = Timestamped {
                    inner: Event::DestroyTick,
                    timestamp: clock.new_timestamp(),
                };
                if events_tx.send(tick).await.is_err() {
                    break;
                }
            }
        });
        None
    }

    /// Re-check a pending destroy. Returns true when the daemon may exit.
    async fn handle_destroy_tick(&mut self) -> bool {
        if self.pending_destroy.is_none() {
            return false;
        }
        let pids = self.running_node_pids();
        let progress = self
            .pending_destroy
            .as_mut()
            .expect("checked above")
            .wait
            .poll(&pids);

        let survivors = match progress {
            shutdown::DestroyProgress::Waiting => return false,
            shutdown::DestroyProgress::Done => Vec::new(),
            shutdown::DestroyProgress::Abandoned(survivors) => survivors,
        };

        let pending = self.pending_destroy.take().expect("checked above");
        if !pending.wait.killed_pids.is_empty() {
            tracing::warn!(
                "killed {} node process(es) that were still running at destroy: {:?}",
                pending.wait.killed_pids.len(),
                pending.wait.killed_pids
            );
        }
        if !survivors.is_empty() {
            tracing::error!(
                "{} node process(es) survived the destroy kill and are now orphaned: {survivors:?}",
                survivors.len(),
            );
        }
        Self::finish_destroy(pending.reply_tx).await;
        true
    }

    /// Pids of every node process this daemon currently has running.
    ///
    /// Dynamic nodes have none: the daemon did not spawn them, so they are
    /// not its children and it cannot reap them.
    fn running_node_pids(&self) -> Vec<u32> {
        self.running
            .values()
            .flat_map(|dataflow| dataflow.running_nodes.values())
            .filter_map(|node| node.pid.as_ref())
            // Zero means the entry exists but its process has not reported a
            // pid yet (`prepared.rs` stores it right after spawn).
            .map(|pid| pid.load(atomic::Ordering::Acquire))
            .filter(|pid| *pid != 0)
            .collect()
    }

    /// Send the destroy reply and wait for it to go out.
    async fn finish_destroy(reply_tx: oneshot::Sender<Option<DaemonCoordinatorReply>>) {
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
                        deliver_param_update_strict(dataflow, &node_id, key, value, &self.clock)
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
                        deliver_param_delete_strict(dataflow, &node_id, key, &self.clock)
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
                // Anything still running when this process ends is orphaned
                // to `ppid 1`: the per-node supervision tasks that would
                // deliver a kill are never polled again. So hold the reply
                // until the nodes are gone (#2980) — the coordinator's
                // destroy completes when it lands, which is what makes
                // `dora down` mean what it documents.
                //
                // Deferred rather than awaited here: a node shutting down
                // cooperatively ends by asking this very event loop to close
                // its outputs, so blocking the loop would deadlock every
                // well-behaved node against the wait and turn its clean exit
                // into a signal kill.
                match self.begin_pending_destroy(reply_tx) {
                    Some(reply_tx) => {
                        Self::finish_destroy(reply_tx).await;
                        RunStatus::Exit
                    }
                    None => RunStatus::Continue,
                }
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

                    // Startup-handshake routing for the added node, from the
                    // *live* dataflow state rather than the (stale) descriptor
                    // — see `added_node_output_routing` for the policy
                    // (existing receivers on a re-added id handshake as usual;
                    // receiver-less outputs are pinned to the daemon path so
                    // `dora node connect` edges can deliver).
                    let output_routing = output_routing::added_node_output_routing(
                        &node_id,
                        node.kind.run_config().outputs,
                        &dataflow.mappings,
                        &dataflow.open_external_mappings,
                        &dataflow.dynamic_nodes,
                    );

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
                        zenoh_connect_endpoint: self.zenoh_listen_endpoint.clone(),
                        zenoh_peering: dataflow.zenoh_peering.clone(),
                        disable_multicast: self.disable_multicast,
                        bind_nodes_to_parent: self.bind_nodes_to_parent,
                    };
                    let mut logger = self
                        .logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .try_clone()
                        .await
                        .context("failed to clone logger")?;
                    // Re-derive the managed Python env dir deterministically so
                    // node restarts reuse the same venv that `dora build` prepared.
                    let python_env_dir =
                        dora_core::build::managed_python_env_dir(&node, &base_working_dir);
                    let task = spawner
                        .spawn_node(
                            node.clone(),
                            base_working_dir,
                            python_env_dir,
                            false,
                            node_stderr,
                            None,
                            output_routing,
                            &mut logger,
                        )
                        .await
                        .wrap_err("failed to prepare node")?;
                    let prepared = task.await.wrap_err("failed to build node")?;
                    let mut running_node = prepared
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
                        // See the spawn path: only `User` inputs can ever
                        // close, so only they gate the drain (#2920).
                        if matches!(input.mapping, InputMapping::User(_)) {
                            dataflow
                                .data_inputs
                                .entry(node_id.clone())
                                .or_default()
                                .insert(input_id.clone());
                        }
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

                    if is_dynamic {
                        dataflow.dynamic_nodes.insert(node_id.clone());
                    }
                    // Deliberately NOT enrolled in `pending_nodes`: this
                    // node is not part of the descriptor the dataflow
                    // started from, so it must neither gate that
                    // cohort's barrier nor inherit its failures.
                    // Enrolling it meant a crash here was broadcast as
                    // the subscribe result of unrelated nodes, and a
                    // node that never subscribed could stall startup
                    // outright (dora-rs/dora#2917).

                    // Open the restart-loop gate and insert the running
                    // node. Marking before the insert is safe: the loop's
                    // first events queue behind this handler on the same
                    // event loop, so they cannot be processed before the
                    // entry is registered.
                    running_node.mark_registered();
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
                            path_sha256: None,
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
                            shared_memory_pool_size: None,
                            output_metadata: Default::default(),
                            pattern: None,
                            send_stdout_as: None,
                            send_logs_as: None,
                            min_log_level: None,
                            max_log_size: None,
                            max_rotated_files: None,
                            build: None,
                            git: None,
                            hub: None,
                            branch: None,
                            tag: None,
                            rev: None,
                            restart_policy: Default::default(),
                            max_restarts: 0,
                            restart_delay: None,
                            max_restart_delay: None,
                            restart_window: None,
                            health_check_timeout: None,
                            finish_grace_secs: None,
                            module: None,
                            params: Default::default(),
                            cpu_affinity: None,
                            deploy: None,
                        });

                    // Spawn timer tasks for any interval this node just
                    // registered. Timer tasks are only ever created in
                    // `start()`, so a node added to an already-running dataflow
                    // whose timer input uses an interval no existing node uses
                    // would otherwise register into `dataflow.timers` but never
                    // get a tick-emitting task — silently starving that input.
                    // `start()` is idempotent (it skips intervals that already
                    // have a handle), so this only spawns tasks for genuinely
                    // new intervals. Guard on `dataflow_started` so that during
                    // initial bring-up the readiness path stays the sole
                    // trigger. Done last, after all state mutations, so a
                    // spawn/registration failure above never leaves a half-added
                    // node with live timer tasks.
                    if dataflow.dataflow_started {
                        dataflow.start(&self.events_tx, &self.clock).await?;
                    }

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
                if result.is_ok() {
                    clear_node_result(&mut self.dataflow_node_results, dataflow_id, &node_id);
                }
                // Return a specific `AddNodeResult` variant so the
                // coordinator can validate the reply against its
                // expected request, instead of treating any non-error
                // reply as success (#1682, rescue of #1757).
                let reply =
                    DaemonCoordinatorReply::AddNodeResult(result.map_err(|err| format!("{err:?}")));
                let _ = reply_tx.send(Some(reply));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::RemoveNode {
                dataflow_id,
                node_id,
                grace_duration,
            } => {
                tracing::info!(%dataflow_id, %node_id, "removing node from running dataflow");
                let result: eyre::Result<()> = (|| {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
                    dataflow.stop_single_node(&node_id, &self.clock, grace_duration)?;

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
                    // Drop this node's timer/log virtual-input subscriptions —
                    // both to stop delivering to a removed node and so a re-added
                    // ID is classified by its own inputs, not stale timer/log
                    // state (which would mark it never-finishing forever, #2270).
                    // Cancels the timer task of any interval left with no
                    // subscribers (#2585); see the method for details.
                    dataflow.unsubscribe_node_from_timers(&node_id);
                    dataflow
                        .log_subscribers
                        .retain(|sub| sub.node_id != node_id);

                    // Clean up remaining state for this node.
                    dataflow.running_nodes.remove(&node_id);
                    dataflow.open_inputs.remove(&node_id);
                    dataflow.data_inputs.remove(&node_id);
                    dataflow.subscribe_channels.remove(&node_id);
                    dataflow.pending_messages.remove(&node_id);
                    dataflow.all_inputs_closed_at.remove(&node_id);
                    // clear the connected marker too, else a re-added node ID
                    // would look already-connected before its new incarnation
                    // subscribes and could be selected mid-startup (dora#2270).
                    dataflow.connected_nodes.remove(&node_id);
                    dataflow.finish_escalated.remove(&node_id);
                    // Purge per-node bookkeeping keyed by node id that the
                    // routing cleanup above doesn't touch. Otherwise stale
                    // input_deadlines/broken_inputs entries are re-scanned
                    // every tick forever and the stderr queue leaks across
                    // repeated dynamic add/remove cycles.
                    dataflow.forget_node_bookkeeping(&node_id);

                    // Remove from stored descriptor (inverse of AddNode
                    // push) so descriptor-based lookups stay consistent.
                    dataflow.descriptor.nodes.retain(|n| n.id != node_id);
                    Ok(())
                })();

                // Outside the closure because it is async. Why removal
                // has to drive the barrier at all: see
                // `PendingNodes::handle_node_removal`.
                let result = match result {
                    Err(err) => Err(err),
                    Ok(()) => {
                        let mut logger = self.logger.for_dataflow(dataflow_id);
                        // The closure above already resolved this id, and
                        // nothing awaits in between, so a miss here is a
                        // bug rather than a race — report it instead of
                        // returning success like the closure's own
                        // `no running dataflow` arm would.
                        match self.running.get_mut(&dataflow_id) {
                            Some(dataflow) => {
                                let status = dataflow
                                    .pending_nodes
                                    .handle_node_removal(
                                        &node_id,
                                        &mut self.coordinator_sender,
                                        &self.clock,
                                        &mut dataflow.cascading_error_causes,
                                        &mut logger,
                                    )
                                    .await;
                                match status {
                                    Ok(DataflowStatus::AllNodesReady)
                                        if !dataflow.dataflow_started =>
                                    {
                                        logger
                                            .log(
                                                LogLevel::Info,
                                                None,
                                                Some("daemon".into()),
                                                "all nodes are ready after node removal, \
                                                 starting dataflow",
                                            )
                                            .await;
                                        dataflow.start(&self.events_tx, &self.clock).await
                                    }
                                    Ok(_) => Ok(()),
                                    Err(err) => Err(err),
                                }
                            }
                            None => Err(eyre!(
                                "dataflow `{dataflow_id}` disappeared while removing `{node_id}`"
                            )),
                        }
                    }
                };

                if let Err(err) = &result {
                    tracing::error!(%dataflow_id, %node_id, "RemoveNode failed: {err:?}");
                }
                let reply = DaemonCoordinatorReply::RemoveNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::ReplaceNode {
                dataflow_id,
                node,
                uv,
                grace_duration,
            } => {
                let node_id = node.id.clone();
                tracing::info!(%dataflow_id, %node_id, "replacing node in running dataflow");

                let result: eyre::Result<()> = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
                    // Replace must not swap a node the startup barrier is
                    // still waiting on: the barrier tracks the original
                    // incarnation's subscription, and a mid-barrier swap
                    // would let the replacement inherit or corrupt that
                    // cohort's state (cf. AddNode's deliberate
                    // non-enrollment, dora-rs/dora#2917). Deliberately NOT
                    // gated on `dataflow_started`: that flag is set by the
                    // all-daemons-ready roundtrip, which can lag seconds
                    // behind this node being visibly Running — the precise
                    // hazard is this id gating the barrier, nothing more.
                    eyre::ensure!(
                        !dataflow.pending_nodes.is_pending(&node_id),
                        "node `{node_id}` is still starting (startup barrier); \
                         retry once the dataflow is ready"
                    );
                    eyre::ensure!(
                        dataflow.running_nodes.contains_key(&node_id),
                        "no running node `{node_id}` to replace; use `dora node add`"
                    );
                    // v1 scope (dora-rs/dora#2927): spawned custom nodes
                    // only. A dynamic replacement spawns no process (the
                    // command would kill the old node and leave nothing
                    // running behind the id), an outgoing dynamic node has
                    // no handle for the grace-kill escalation, and
                    // runtime/operator nodes keep their inputs in a
                    // different descriptor location than the node-level
                    // comparison below.
                    eyre::ensure!(
                        !node.kind.dynamic()
                            && matches!(node.kind, dora_core::descriptor::CoreNodeKind::Custom(_)),
                        "`dora node replace` currently supports spawned custom nodes only; \
                         the replacement definition for `{node_id}` is a {} node — use \
                         `dora node remove` + `dora node add` instead",
                        if node.kind.dynamic() {
                            "dynamic"
                        } else {
                            "runtime/operator"
                        }
                    );
                    eyre::ensure!(
                        !dataflow.dynamic_nodes.contains(&node_id),
                        "node `{node_id}` is a dynamic node; `dora node replace` currently \
                         supports spawned custom nodes only — use `dora node remove` + \
                         `dora node add` instead"
                    );

                    // --- same-edges validation -------------------------------
                    // A replace is a swap, not a topology edit
                    // (dora-rs/dora#2927): the replacement must keep the
                    // node's LIVE input edges exactly and still produce
                    // every output a consumer (local or on a remote
                    // daemon) is mapped to. The live edge state
                    // (mappings/timers/log subscriptions) is the source of
                    // truth rather than the stored descriptor entry: entry
                    // input locations vary by node kind and by how the
                    // node entered the dataflow (spawn vs AddNode), while
                    // the live maps are uniformly keyed and already
                    // reflect `dora node connect`/`disconnect` edits.
                    let new_inputs = node_inputs(&node);
                    let mut current_edges: BTreeMap<DataId, InputMapping> = BTreeMap::new();
                    for (output_id, receivers) in &dataflow.mappings {
                        for (receiver, input_id) in receivers {
                            if receiver == &node_id {
                                current_edges.insert(
                                    input_id.clone(),
                                    InputMapping::User(dora_message::config::UserInputMapping {
                                        source: output_id.0.clone(),
                                        output: output_id.1.clone(),
                                    }),
                                );
                            }
                        }
                    }
                    for (interval, receivers) in &dataflow.timers {
                        for (receiver, input_id) in receivers {
                            if receiver == &node_id {
                                current_edges.insert(
                                    input_id.clone(),
                                    InputMapping::Timer {
                                        interval: *interval,
                                    },
                                );
                            }
                        }
                    }
                    for subscriber in &dataflow.log_subscribers {
                        if subscriber.node_id == node_id {
                            current_edges.insert(
                                subscriber.input_id.clone(),
                                InputMapping::Logs(subscriber.filter.clone()),
                            );
                        }
                    }
                    const EDGE_HINT: &str = "`dora node replace` keeps the node's edges — use \
                         `dora node remove`/`add` or `dora node connect`/`disconnect` for \
                         topology changes";
                    for (input_id, input) in &new_inputs {
                        match current_edges.remove(input_id) {
                            Some(mapping) if mapping == input.mapping => {}
                            Some(_) => {
                                eyre::bail!("replacement remaps input `{input_id}`; {EDGE_HINT}")
                            }
                            None => {
                                eyre::bail!("replacement adds input `{input_id}`; {EDGE_HINT}")
                            }
                        }
                    }
                    if let Some((input_id, _)) = current_edges.into_iter().next() {
                        eyre::bail!("replacement drops input `{input_id}`; {EDGE_HINT}");
                    }
                    // `node_output_ids` chains local mappings AND
                    // `open_external_mappings`, so outputs consumed only by
                    // nodes on other daemons are covered too.
                    let new_outputs = node.kind.run_config().outputs;
                    for output_id in dataflow.node_output_ids(&node_id) {
                        eyre::ensure!(
                            new_outputs.contains(&output_id),
                            "replacement does not declare output `{output_id}`, which \
                             downstream nodes consume"
                        );
                    }

                    // --- spawn the replacement (old incarnation untouched) ---
                    // Any failure up to and including the spawn leaves the
                    // current incarnation running (dora-rs/dora#2927).
                    // NOTE: this block deliberately mirrors the AddNode
                    // arm's spawn sequence above — keep the two in sync
                    // when adding Spawner fields or spawn parameters.
                    let base_working_dir = self
                        .working_dir
                        .get(&dataflow_id)
                        .cloned()
                        .unwrap_or_else(|| std::path::PathBuf::from("."));
                    let is_dynamic = node.kind.dynamic();
                    let output_routing = output_routing::added_node_output_routing(
                        &node_id,
                        node.kind.run_config().outputs,
                        &dataflow.mappings,
                        &dataflow.open_external_mappings,
                        &dataflow.dynamic_nodes,
                    );
                    // Fresh stderr buffer for the new incarnation; installed
                    // into the map only after the spawn succeeds.
                    let node_stderr = Arc::new(ArrayQueue::new(STDERR_LOG_LINES_MAX));
                    // Patch the descriptor CLONE handed to the spawner with
                    // the replacement's definition before it is serialized
                    // into the child's DORA_NODE_CONFIG — the stored
                    // descriptor still describes the outgoing incarnation
                    // at this point (state mutations are deferred until the
                    // spawn succeeds).
                    let mut descriptor = dataflow.descriptor.clone();
                    if let Some(entry) = descriptor.nodes.iter_mut().find(|n| n.id == node_id) {
                        patch_descriptor_entry(entry, &node);
                    }
                    let spawner = Spawner {
                        dataflow_id,
                        daemon_tx: self.events_tx.clone(),
                        dataflow_descriptor: descriptor,
                        clock: self.clock.clone(),
                        uv,
                        ft_stats: self.ft_stats.clone(),
                        shutdown: dataflow.listener_shutdown_rx.clone(),
                        zenoh_connect_endpoint: self.zenoh_listen_endpoint.clone(),
                        zenoh_peering: dataflow.zenoh_peering.clone(),
                        disable_multicast: self.disable_multicast,
                        bind_nodes_to_parent: self.bind_nodes_to_parent,
                    };
                    let mut logger = self
                        .logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .try_clone()
                        .await
                        .context("failed to clone logger")?;
                    let python_env_dir =
                        dora_core::build::managed_python_env_dir(&node, &base_working_dir);
                    let task = spawner
                        .spawn_node(
                            node.clone(),
                            base_working_dir,
                            python_env_dir,
                            false,
                            node_stderr.clone(),
                            None,
                            output_routing,
                            &mut logger,
                        )
                        .await
                        .wrap_err("failed to prepare replacement node")?;
                    let prepared = task.await.wrap_err("failed to build replacement node")?;
                    let mut running_node = prepared
                        .spawn(logger)
                        .await
                        .wrap_err("failed to spawn replacement node")?;

                    // --- commit: swap entries and stop the outgoing one ------
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("dataflow disappeared during replacement spawn"))?;
                    // If the entry vanished mid-spawn (concurrent remove),
                    // dropping `running_node` kills the fresh process via
                    // its ProcessHandle Drop and cancels its (ungated)
                    // restart loop — no orphan.
                    let mut outgoing =
                        dataflow.running_nodes.remove(&node_id).ok_or_else(|| {
                            eyre!("node `{node_id}` was removed during the replacement spawn")
                        })?;
                    outgoing.disable_restart();
                    let outgoing_generation = outgoing.generation;
                    let outgoing_process = outgoing.process.take();
                    // Stop the outgoing incarnation while its subscribe
                    // channel is still installed so the graceful `Stop`
                    // reaches it; its later exit event carries
                    // `outgoing_generation` and is dropped by the
                    // generation guard instead of being attributed to the
                    // replacement (dora-rs/dora#2926).
                    dataflow.stop_replaced_incarnation(
                        &node_id,
                        outgoing_generation,
                        outgoing_process,
                        &self.clock,
                        grace_duration,
                    );

                    // Per-incarnation state reset. Three deliberate
                    // NON-resets:
                    // - mappings/timers/log subscriptions are keyed by
                    //   (node id, input id) and the edges are validated
                    //   identical, so they carry over unchanged;
                    // - `open_inputs`/`data_inputs` record DATAFLOW-level
                    //   input closure (an upstream that already finished),
                    //   which is delivered exactly once and afterwards
                    //   reconstructed from these maps by the subscribe-time
                    //   replay — resetting them would make a replacement
                    //   installed after an upstream finished wait forever
                    //   on a dead input and hang dataflow completion;
                    // - the descriptor entry keeps the outgoing
                    //   definition's other fields (path/env/...) — stale
                    //   metadata the daemon itself never consults (only
                    //   `id` and `inputs` are read; the coordinator's
                    //   descriptor holds the authoritative new definition).
                    dataflow.subscribe_channels.remove(&node_id);
                    dataflow.pending_messages.remove(&node_id);
                    dataflow.all_inputs_closed_at.remove(&node_id);
                    dataflow.connected_nodes.remove(&node_id);
                    dataflow.finish_escalated.remove(&node_id);
                    dataflow.cascading_error_causes.forget(&node_id);
                    dataflow.forget_node_bookkeeping(&node_id);
                    dataflow
                        .node_stderr_most_recent
                        .insert(node_id.clone(), node_stderr);

                    // Re-register input deadlines, but only for inputs that
                    // are still open — a deadline on an already-closed
                    // input would be re-scanned forever without ever
                    // arming.
                    let still_open = dataflow.open_inputs.get(&node_id).cloned();
                    for (input_id, input) in &new_inputs {
                        if matches!(input.mapping, InputMapping::User(_))
                            && let Some(timeout) = input.input_timeout
                            && still_open
                                .as_ref()
                                .is_some_and(|open| open.contains(input_id))
                        {
                            dataflow.input_deadlines.insert(
                                (node_id.clone(), input_id.clone()),
                                InputDeadline {
                                    timeout: Duration::from_secs_f64(timeout),
                                    last_received: None,
                                },
                            );
                        }
                    }

                    // Update the stored descriptor entry to the
                    // replacement's definition — the same patch applied to
                    // the spawn-time clone, so later spawns (and restart
                    // respawns of OTHER nodes) serialize the replacement,
                    // not the outgoing incarnation.
                    if let Some(entry) = dataflow
                        .descriptor
                        .nodes
                        .iter_mut()
                        .find(|n| n.id == node_id)
                    {
                        patch_descriptor_entry(entry, &node);
                    }

                    running_node.mark_registered();
                    dataflow.running_nodes.insert(node_id.clone(), running_node);

                    tracing::info!(
                        %dataflow_id,
                        %node_id,
                        outgoing_generation,
                        dynamic = is_dynamic,
                        "node replaced successfully"
                    );
                    Ok(())
                }
                .await;

                if let Err(err) = &result {
                    tracing::error!(%dataflow_id, %node_id, "ReplaceNode failed: {err:?}");
                }
                if result.is_ok() {
                    // The outgoing (or an even earlier) incarnation's stale
                    // result must not be attributed to the replacement.
                    clear_node_result(&mut self.dataflow_node_results, dataflow_id, &node_id);
                }
                let reply = DaemonCoordinatorReply::ReplaceNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply));
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
                // Previously this handler ignored unknown dataflow_id and
                // always replied `None`, which the WS layer dropped on the
                // floor → coordinator timed out after 30s without ever
                // knowing whether the mapping applied. Reply with an
                // explicit `AddMappingResult` so the coordinator can pattern-
                // match the outcome (same class as #1682's AddNodeResult).
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    dataflow.add_mapping(source_node, source_output, target_node, target_input);
                    Ok(())
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::AddMappingResult(result)));
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
                // Same silent-reply fix as AddMapping above.
                // Errors on missing-mapping (rather than silently succeeding)
                // to match the `RemoveNode` semantics in stop_single_node,
                // which surface "node not found" as a daemon Err. A double-
                // disconnect or typo'd edge then produces a clear CLI error
                // instead of a misleading "Mapping removed" message.
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    let output_id = OutputId(source_node.clone(), source_output.clone());
                    let removed = dataflow
                        .mappings
                        .get_mut(&output_id)
                        .map(|r| r.remove(&(target_node.clone(), target_input.clone())))
                        .unwrap_or(false);
                    if removed {
                        close_input(dataflow, &target_node, &target_input, &self.clock);
                        Ok(())
                    } else {
                        Err(format!(
                            "mapping `{source_node}/{source_output}` -> \
                             `{target_node}/{target_input}` not found"
                        ))
                    }
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::RemoveMappingResult(result)));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StartTopicDebugStream {
                dataflow_id,
                outputs,
                subscription_id,
            } => {
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    for (node_id, data_id) in outputs {
                        dataflow
                            .debug_topic_watchers
                            .entry(OutputId(node_id, data_id))
                            .or_default()
                            .insert(subscription_id);
                    }
                    Ok(())
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::StartTopicDebugStreamResult(
                    result,
                )));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StopTopicDebugStream {
                dataflow_id,
                subscription_id,
            } => {
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    // Scan watchers rather than maintain an inverse map. Unsubscribe
                    // is rare; scan is bounded by the count of outputs with active
                    // subscribers. retain() drops empty entries in one pass.
                    dataflow
                        .debug_topic_watchers
                        .retain(|_output_id, watchers| {
                            watchers.remove(&subscription_id);
                            !watchers.is_empty()
                        });
                    Ok(())
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::StopTopicDebugStreamResult(
                    result,
                )));
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
                let applied_through = match self.running.get(&dataflow_id) {
                    Some(dataflow) => apply_state_catch_up_entries(dataflow, &entries, &self.clock),
                    None => {
                        tracing::warn!(
                            "state catch-up: dataflow `{dataflow_id}` no longer running on daemon"
                        );
                        0
                    }
                };
                // Ack only the prefix that was actually accepted for delivery.
                if applied_through > 0
                    && let Some(sender) = &self.coordinator_sender
                {
                    let ack = DaemonEvent::StateCatchUpAck {
                        dataflow_id,
                        ack_sequence: applied_through,
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

    /// Watchdog for nodes that block an otherwise-finished dataflow
    /// (dora-rs/dora#2152).
    ///
    /// The natural-finish path sends `AllInputsClosed` and waits for nodes
    /// to exit voluntarily; unlike explicit stops it had no deadline, so a
    /// stuck node hung `dora run` (and CI) until an external timeout. Once
    /// every running node of a dataflow has been draining for longer than
    /// the grace period, escalate through the same Stop → SIGTERM → SIGKILL
    /// ladder used by explicit stops — after capturing a stack sample of
    /// the stuck process so the hang itself stays diagnosable.
    fn check_finish_stragglers(&mut self) {
        // On by default; only DORA_FINISH_DRAIN_GRACE_SECS=off/disabled turns
        // escalation off entirely (see `finish_drain_grace`).
        let Some(grace) = finish_drain_grace() else {
            return;
        };
        let now_millis = node_communication::current_millis();
        for (dataflow_id, dataflow) in self.running.iter_mut() {
            for node_id in dataflow.finish_stragglers(grace, now_millis) {
                let drained_for_secs = dataflow
                    .all_inputs_closed_at
                    .get(&node_id)
                    .map(|since| since.elapsed().as_secs())
                    .unwrap_or_default();
                let pid = dataflow
                    .running_nodes
                    .get(&node_id)
                    .and_then(|node| node.pid.as_ref())
                    .map(|pid| pid.load(atomic::Ordering::Relaxed));
                // Escalate first: if the node exited naturally between
                // selection and now, this fails benignly and nothing
                // should be logged or sampled.
                if dataflow
                    .stop_single_node(&node_id, &self.clock, None)
                    .is_err()
                {
                    tracing::debug!(
                        "finish straggler `{node_id}` exited before escalation; skipping"
                    );
                    continue;
                }
                dataflow.finish_escalated.insert(node_id.clone());
                tracing::warn!(
                    "dataflow {dataflow_id} is finished except node `{node_id}` \
                     (AllInputsClosed sent {drained_for_secs}s ago) — escalating stop \
                     (set DORA_FINISH_DRAIN_GRACE_SECS to adjust the grace period)"
                );
                spawn_stack_sample_capture(node_id, pid);
            }
        }
    }

    fn check_node_health(&self) {
        let now_millis = node_communication::current_millis();
        for dataflow in self.running.values() {
            for (node_id, node) in &dataflow.running_nodes {
                let Some(timeout) = node.health_check_timeout else {
                    continue;
                };
                let last = node.last_activity.load(atomic::Ordering::Acquire);
                // The health-check watchdog only monitors *post-connection*
                // liveness. `last_activity` is seeded to the spawn timestamp
                // (not 0), so a node that has not yet sent its first
                // `DaemonRequest` would be measured as silent from spawn and
                // SIGKILLed mid-startup — e.g. a node whose cold start (Python
                // imports + model-weight load) legitimately exceeds
                // `health_check_timeout`. Gate the kill on the node having
                // connected, mirroring the finish-straggler watchdog (#2937).
                let connected = dataflow.connected_nodes.contains(node_id);
                if !health_check_should_kill(connected, last, now_millis, timeout) {
                    continue;
                }
                let elapsed_ms = now_millis.saturating_sub(last);
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

    /// Forward a debug-observed local output to coordinator debug watchers.
    ///
    /// Reconstructs the same `InterDaemonEvent::Output` frame that `send_out`
    /// produced before #1787 moved data routing off the daemon, so the CLI's
    /// `dora topic` inspection sees an unchanged wire format. Does **not**
    /// deliver to local receivers — the publishing node already did that over
    /// Zenoh.
    async fn handle_debug_topic_data(
        &self,
        dataflow_id: DataflowId,
        output_id: OutputId,
        metadata: dora_message::metadata::Metadata,
        data: Option<Vec<u8>>,
    ) -> eyre::Result<()> {
        // Skip (re)serialization when no CLI is currently watching this topic.
        let has_watchers = self
            .running
            .get(&dataflow_id)
            .map(|df| df.debug_topic_watchers.contains_key(&output_id))
            .unwrap_or(false);
        if !has_watchers {
            return Ok(());
        }

        let event = InterDaemonEvent::Output {
            dataflow_id,
            node_id: output_id.0.clone(),
            output_id: output_id.1.clone(),
            metadata,
            data: data.map(|d| AVec::from_slice(128, &d)),
        };
        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        }
        .serialize()
        .wrap_err("failed to serialize debug topic event")?;

        self.send_topic_debug_frames(dataflow_id, &output_id, serialized_event)
            .await
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
                // compare clone identity (repo + commit) only: hub provenance
                // and subdir don't change which directory the clone occupies
                still_needed_for_this_build: git_sources.values().any(|s| {
                    s.repo == prev_source.repo && s.commit_hash == prev_source.commit_hash
                }),
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

        // hub-sourced nodes (recognized by the provenance marker on their
        // git source) are spawned with confined path resolution (spec §11)
        let confined_nodes: BTreeSet<NodeId> = git_sources
            .iter()
            .filter(|(_, source)| source.hub.is_some())
            .map(|(node_id, _)| node_id.clone())
            .collect();
        let task = async move {
            let mut info = BuildInfo {
                node_working_dirs: Default::default(),
                python_env_dirs: Default::default(),
                confined_nodes,
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
                    .insert(node_id.clone(), node.node_working_dir);
                if let Some(python_env_dir) = node.python_env_dir {
                    info.python_env_dirs.insert(node_id, python_env_dir);
                }
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
        // Sweep orphaned /dev/shm segments from a previous crash of the
        // same dataflow (keyed by dataflow_id, a UUID — safe in multi-
        // daemon setups).
        MemoryPoolManager::cleanup_orphans(&dataflow_id.to_string());

        let mut logger = self
            .logger
            .for_dataflow(dataflow_id)
            .try_clone()
            .await
            .context("failed to clone logger")?;
        let mut dataflow = RunningDataflow::new(
            dataflow_id,
            self.daemon_id.clone(),
            dataflow_descriptor.clone(),
        );
        // Read from the descriptor, which is the one copy that survives
        // everything a dataflow outlives: auto-recovery re-spawn,
        // coordinator restart with state reconstruction, and `dora
        // restart`. A daemon serving a coordinator hosts many dataflows
        // and only some are batch-style, so this is necessarily
        // per-dataflow rather than daemon-wide (#2920).
        dataflow.timers_gate_drain = !dataflow.descriptor.exit_when_nodes_finish.unwrap_or(false);
        // Decide who dials whom before anything spawns: zenoh 1.9 peers do not
        // relay, so a producer/consumer pair that never forms a direct link can
        // never exchange data. Assign the links explicitly instead of leaving
        // them to gossip's best-effort autoconnect.
        dataflow.zenoh_peering = Arc::new(crate::spawn::plan_zenoh_peering(
            &nodes,
            &spawn_nodes,
            self.zenoh_listen_endpoint.as_deref(),
        ));
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

        // A present `build_id` means the session has a build that wasn't
        // invalidated (the CLI clears it when build inputs change). We use this
        // below to decide whether re-deriving an on-disk managed env is safe.
        let have_build_id = build_id.is_some();
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
        // Reuse build-time metadata so runtime spawn can follow the same
        // working-directory and managed-env decisions.
        let (node_working_dirs, python_env_dirs, confined_nodes) = build_info
            .map(|info| {
                (
                    info.node_working_dirs.clone(),
                    info.python_env_dirs.clone(),
                    info.confined_nodes.clone(),
                )
            })
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
                    // Record which inputs can ever close. Timer and Logs
                    // inputs are fed by the daemon and have no upstream
                    // node, so they never close; only a `User` mapping is
                    // a real data dependency (#2920).
                    if matches!(input.mapping, InputMapping::User(_)) {
                        dataflow
                            .data_inputs
                            .entry(node.id.clone())
                            .or_default()
                            .insert(input_id.clone());
                    }
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

        // When debug inspection is enabled, the daemon subscribes to its own
        // local nodes' Zenoh output topics so `dora topic info/echo/hz` keep
        // working. Since #1787, node→node data flows directly over Zenoh and
        // never reaches the daemon's `send_out`; without this the inspection
        // commands observe zero messages. Each sample is forwarded as an
        // `Event::DebugTopicData` and only relayed to coordinator debug
        // watchers — never re-delivered to local receivers (the publishing
        // node already delivered the payload via Zenoh).
        if dataflow.enable_debug_inspection {
            use zenoh_ext::{AdvancedSubscriberBuilderExt, HistoryConfig};
            for node in nodes.values().filter(|n| spawn_nodes.contains(&n.id)) {
                for output_id in node.kind.run_config().outputs {
                    let topic = zenoh_output_publish_topic(dataflow_id, &node.id, &output_id);
                    tracing::debug!("declaring debug subscriber on {topic}");
                    let subscriber = self
                        .zenoh_session
                        .declare_subscriber(topic.clone())
                        .await
                        .map_err(|e| eyre!(e))
                        .wrap_err_with(|| {
                            format!("failed to declare debug subscriber on {topic}")
                        })?;

                    // Small node→node outputs use the schema-once path: the
                    // schema is published once on the `@schema` subtopic and each
                    // data sample carries only the schema-less record batch (see
                    // `DoraNode::publish_schema_once`). `dora topic` expects a
                    // full self-describing stream, so cache the schema here and
                    // rebuild it before forwarding. An `AdvancedSubscriber`
                    // (history + late-publisher detection) mirrors the node
                    // receive path so the (single, stable) schema is fetched even
                    // when the producer starts after this subscriber.
                    let schema_cache: DebugSchemaCache =
                        Arc::new(std::sync::Mutex::new(Vec::new()));
                    let schema_topic = dora_core::topics::zenoh_output_schema_topic(
                        dataflow_id,
                        &node.id,
                        &output_id,
                    );
                    let schema_subscriber = {
                        let cache = schema_cache.clone();
                        match self
                            .zenoh_session
                            .declare_subscriber(schema_topic.clone())
                            .history(HistoryConfig::default().detect_late_publishers())
                            .callback(move |sample| {
                                let bytes = sample.payload().to_bytes();
                                let hash = dora_message::metadata::fnv1a(&bytes);
                                retain_debug_schema(
                                    &mut cache.lock().unwrap_or_else(|e| e.into_inner()),
                                    hash,
                                    &bytes,
                                );
                            })
                            .await
                        {
                            Ok(s) => Some(s),
                            Err(e) => {
                                tracing::warn!(
                                    "failed to declare @schema debug subscriber on \
                                     {schema_topic} ({e}); schema-once outputs may render as \
                                     undecodable in `dora topic`"
                                );
                                None
                            }
                        }
                    };

                    let events_tx = self.events_tx.clone();
                    let clock = self.clock.clone();
                    let node_id = node.id.clone();
                    let mut finished_rx = dataflow.finished_tx.subscribe();
                    tokio::spawn(async move {
                        // Keep the `@schema` subscriber alive for the task's life.
                        let _schema_subscriber = schema_subscriber;
                        let mut finished = pin!(finished_rx.recv());
                        loop {
                            match future::select(finished, subscriber.recv_async()).await {
                                future::Either::Left((_, _)) => break,
                                future::Either::Right((sample, f)) => {
                                    finished = f;
                                    let Ok(sample) = sample else { break };
                                    // Node publishes raw payload + bincode metadata
                                    // attachment (see `DoraNode::zenoh_publish`).
                                    let Some(mut metadata) = sample.attachment().and_then(|a| {
                                        bincode::deserialize::<dora_message::metadata::Metadata>(
                                            &a.to_bytes(),
                                        )
                                        .ok()
                                    }) else {
                                        continue;
                                    };
                                    // Startup-handshake markers ride the real data
                                    // topic (empty payload); consumers filter them
                                    // before decode and so must this inspection
                                    // path, or `dora topic echo`/`hz` would show
                                    // spurious empty frames and inflated rates
                                    // during every startup handshake.
                                    if metadata.is_startup_marker() {
                                        continue;
                                    }
                                    let payload = sample.payload().to_bytes();
                                    let data = {
                                        let mut cached =
                                            schema_cache.lock().unwrap_or_else(|e| e.into_inner());
                                        rebuild_debug_topic_stream(
                                            &mut cached,
                                            &metadata.parameters,
                                            &payload[..],
                                        )
                                    };
                                    let Some(data) = data else {
                                        // schema-once batch whose schema hasn't been
                                        // cached yet — drop this inspection frame
                                        // (transient, recovers once `@schema` or the
                                        // producer's next full-stream refresh arrives).
                                        continue;
                                    };
                                    // `data` is now always a full self-describing
                                    // stream, so drop the internal wire keys — a
                                    // `_schema_hash` here would contradict the
                                    // rebuilt payload for any consumer applying the
                                    // wire rule "hash present => schema-less batch".
                                    dora_message::metadata::strip_internal_parameters(
                                        &mut metadata.parameters,
                                    );
                                    // Record the true on-wire size so `dora topic
                                    // info` measures the schema-less batch that
                                    // actually travelled, not the rebuilt stream
                                    // (which prepends the schema to every frame even
                                    // though it ships once). For a full
                                    // self-describing frame this equals `data.len()`;
                                    // for a schema-once batch it excludes the
                                    // prepended schema block (dora-rs/dora#2584).
                                    metadata.parameters.insert(
                                        dora_message::metadata::WIRE_SIZE.to_string(),
                                        dora_message::metadata::Parameter::Integer(
                                            payload.len() as i64
                                        ),
                                    );
                                    let event = Event::DebugTopicData {
                                        dataflow_id,
                                        output_id: OutputId(node_id.clone(), output_id.clone()),
                                        metadata,
                                        data: Some(data),
                                    };
                                    if events_tx
                                        .send(Timestamped {
                                            inner: event,
                                            timestamp: clock.new_timestamp(),
                                        })
                                        .await
                                        .is_err()
                                    {
                                        break;
                                    }
                                }
                            }
                        }
                    });
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
            zenoh_connect_endpoint: self.zenoh_listen_endpoint.clone(),
            zenoh_peering: dataflow.zenoh_peering.clone(),
            disable_multicast: self.disable_multicast,
            bind_nodes_to_parent: self.bind_nodes_to_parent,
        };

        // Startup-handshake routing, from actual placement (`spawn_nodes`):
        // which outputs are pinned to the daemon path (remote consumers) and
        // which local static consumers must ack before an output may switch to
        // the direct zenoh path.
        let mut output_routing = output_routing::compute_output_routing(&nodes, &spawn_nodes);

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
                let mut configured_python_env_dir = python_env_dirs.get(&node_id).cloned();
                // Under `--uv`, a node may have no recorded managed env even when
                // `dora build` prepared one: a networked build runs in a separate
                // process from the daemon that later serves `dora start`, so the
                // daemon's in-memory build record can be empty (dora-rs/dora#2004).
                // The env dir is deterministic, so re-derive it (the restart path
                // does the same) and reuse the on-disk env -- but ONLY when a
                // build id is present, i.e. a build occurred and this daemon just
                // lacks the in-memory record. When the build id was cleared (the
                // CLI invalidates it on build-input changes, a prior non-`--uv`
                // build, or `start --uv` with no build), do NOT reuse a possibly
                // stale env; require a rebuild. Either way, never silently fall
                // back to the ambient Python.
                if uv
                    && configured_python_env_dir.is_none()
                    && let Some(expected) =
                        dora_core::build::managed_python_env_dir(&node, &node_working_dir)
                {
                    if have_build_id
                        && dora_core::build::managed_python_interpreter(&expected).is_file()
                    {
                        configured_python_env_dir = Some(expected);
                    } else {
                        eyre::bail!(
                            "node `{node_id}` is a Python node that needs a managed env under `--uv`, \
                             but no current build provides one (the build cache is absent or was \
                             invalidated by changed build inputs). \
                             Run `dora build --uv <dataflow>` before `dora start --uv`, \
                             or omit `--uv` to run against the ambient Python."
                        );
                    }
                }
                match spawner
                    .clone()
                    .spawn_node(
                        node,
                        node_working_dir,
                        configured_python_env_dir,
                        confined_nodes.contains(&node_id),
                        node_stderr_most_recent,
                        node_write_events_to,
                        output_routing.remove(&node_id).unwrap_or_default(),
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
                        zenoh_daemon_control_topic(dataflow.id, &output_id.0, &output_id.1);
                    tracing::debug!("declaring control subscriber on {subscribe_topic}");
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
                                    match sample {
                                        Ok(s) => {
                                            // Count telemetry for every received control sample.
                                            net_bytes_rx.fetch_add(
                                                s.payload().len() as u64,
                                                std::sync::atomic::Ordering::Relaxed,
                                            );
                                            net_msgs_rx
                                                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                            let bytes = s.payload().to_bytes();
                                            let event =
                                                Timestamped::deserialize_inter_daemon_event(&bytes)
                                                    .map_err(|e| eyre!(e));
                                            if tx.send_async(event).await.is_err() {
                                                // daemon finished
                                                break;
                                            }
                                        }
                                        Err(e) => {
                                            if tx.send_async(Err(eyre!(e))).await.is_err() {
                                                // daemon finished
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    });
                }
            }
        }
        for (node_id, dynamic) in stopped {
            // Pre-spawn failures (resolve/validate errors). Surface
            // as Failed so the dataflow doesn't silently hide them.
            self.handle_node_stop(dataflow_id, &node_id, dynamic, false)
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
                                // `start()` sets `dataflow_started`; the guard
                                // above keeps this to a single spawn.
                                dataflow.start(&self.events_tx, &self.clock).await?;
                            }
                            _ => {}
                        }
                    }
                }
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
            DaemonNodeEvent::OutputSent {
                output_id,
                metadata,
            } => self
                .output_sent(dataflow_id, node_id, output_id, metadata)
                .context("failed to mark output sent")?,
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
            DaemonNodeEvent::RegisterPinnedMemory {
                shared_memory_id,
                metadata,
                reply_sender,
            } => {
                let result = (|| -> Result<(), String> {
                    let pool_metadata = pool_metadata_from_params(&metadata.parameters);
                    // Validate required fields that the helper fills with defaults
                    if pool_metadata.ptr == 0 {
                        return Err("missing or invalid ptr".to_string());
                    }
                    if pool_metadata.size == 0 {
                        return Err("missing or invalid size".to_string());
                    }
                    if pool_metadata.dtype.is_empty() {
                        return Err("missing or invalid dtype".to_string());
                    }
                    if pool_metadata.shape.is_empty() {
                        return Err("missing shape".to_string());
                    }
                    // Mirror the Python-side size cap.
                    if pool_metadata.size > 1024 * 1024 * 1024 {
                        return Err(format!("size {} exceeds 1 GiB cap", pool_metadata.size));
                    }
                    // Require a shared memory name for cleanup.
                    let shm_name = pool_metadata
                        .shared_memory_name
                        .as_ref()
                        .filter(|n| !n.is_empty())
                        .ok_or_else(|| "missing shared_memory_name".to_string())?;
                    // Validate prefix in the same way free_shared_memory does.
                    if !shm_name.starts_with("dora_pool_")
                        || shm_name.contains('/')
                        || shm_name.contains("..")
                    {
                        return Err(format!("shared_memory_name `{}` is invalid", shm_name));
                    }

                    // Per-daemon pool cap (soft limit — rejects excess registrations).
                    const MAX_POOLS: usize = 512;
                    if self.memory_pool.table_size() >= MAX_POOLS {
                        return Err(format!(
                            "daemon pool table full ({MAX_POOLS} entries); \
                             free unused pools before registering more"
                        ));
                    }

                    self.memory_pool.register_memory_pool(
                        MemoryPoolId {
                            dataflow_id: dataflow_id.to_string(),
                            id: shared_memory_id,
                        },
                        pool_metadata,
                        node_id.to_string(),
                    )
                })();
                let _ = reply_sender.send(DaemonReply::Result(result));
            }
            DaemonNodeEvent::ReadPinnedMemory {
                shared_memory_id,
                free,
                reply_sender,
            } => {
                let result = (|| -> Result<dora_message::metadata::Metadata, String> {
                    let id = MemoryPoolId {
                        dataflow_id: dataflow_id.to_string(),
                        id: shared_memory_id.clone(),
                    };
                    let metadata = self
                        .memory_pool
                        .read_memory_pool(&id, node_id.as_ref())
                        .ok_or_else(|| {
                            format!("memory pool with ID {} not found", shared_memory_id)
                        })?;

                    if free
                        && let Err(err) = free_pool_and_notify(
                            &self.memory_pool,
                            self.running.get(&dataflow_id),
                            &id,
                            node_id.as_ref(),
                            &self.clock,
                        )
                    {
                        tracing::warn!(
                            "Failed to free memory pool {} after reading: {}",
                            shared_memory_id,
                            err
                        );
                    }

                    let mut parameters = pool_metadata_to_params(&metadata);
                    // When freeing, drop shared_memory_name — the segment has
                    // been unlinked and the name is a dangling reference.
                    if free {
                        parameters.remove("shared_memory_name");
                    }

                    let timestamp = self.clock.new_timestamp();
                    Ok(dora_message::metadata::Metadata::from_parameters(
                        timestamp, parameters,
                    ))
                })();

                match result {
                    Ok(metadata) => {
                        let _ = reply_sender.send(DaemonReply::PinnedMemoryMetadata { metadata });
                    }
                    Err(err) => {
                        let _ = reply_sender.send(DaemonReply::Result(Err(err)));
                    }
                }
            }
            DaemonNodeEvent::FreePinnedMemory {
                shared_memory_id,
                reply_sender,
            } => {
                let id = MemoryPoolId {
                    dataflow_id: dataflow_id.to_string(),
                    id: shared_memory_id,
                };
                let result = free_pool_and_notify(
                    &self.memory_pool,
                    self.running.get(&dataflow_id),
                    &id,
                    node_id.as_ref(),
                    &self.clock,
                );
                let _ = reply_sender.send(DaemonReply::Result(result));
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

    /// Record an output event that named a dataflow the daemon no longer
    /// runs, and drop it.
    ///
    /// Usually this means the dataflow finished while the node still had
    /// traffic in flight. Two ways in, both normal:
    ///
    /// * a node's exit and its already-transmitted outputs reach the
    ///   daemon's event loop on independent paths, so a burst sent just
    ///   before exit can be queued behind the `SpawnedNodeResult` that
    ///   finished the dataflow (the intermittent nightly `smoke-suite`
    ///   failure in dora-rs/dora#2742);
    /// * `should_finish` ignores still-running *dynamic* nodes, so a
    ///   dynamic node is expected to outlive `finish_dataflow` and may
    ///   keep sending for as long as it likes afterwards.
    ///
    /// It can also mean a stale or misconfigured node: registration takes
    /// `dataflow_id` from the node's own `Register` request and only
    /// version-checks it, so an id the daemon never ran reaches here too.
    /// That is why this warns rather than logging at `debug`.
    ///
    /// Fatal it must not be: neither `SendOut` nor `OutputSent` carries a
    /// reply channel, so `handle_node_event` cannot report the error back
    /// to the node the way every sibling arm does — it can only return
    /// `Err`, which unwinds the daemon's main loop and drops its
    /// coordinator connection — which loses this message *and* every
    /// other dataflow's connection with it.
    ///
    /// Dropping it is not always free, though: `should_finish` only looks
    /// at this daemon's non-dynamic nodes, so a still-running local
    /// dynamic consumer, or a consumer on another daemon reached through
    /// `open_external_mappings`, can still have been waiting for it. That
    /// is a pre-existing finish-ordering gap, strictly better than taking
    /// the daemon down for it, and why this is a warning rather than a
    /// silent drop.
    ///
    /// Warns once per node, then falls to `debug`: the dynamic-node case
    /// above is unbounded, and a node sending at 1 kHz would otherwise
    /// emit 1000 warn lines a second for as long as it stayed alive.
    fn log_late_node_output(
        &mut self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        output_id: &DataId,
        what: &'static str,
    ) {
        if self.warned_late_outputs.len() >= MAX_WARNED_LATE_OUTPUT_NODES {
            self.warned_late_outputs.clear();
        }
        if self
            .warned_late_outputs
            .insert((*dataflow_id, node_id.clone()))
        {
            tracing::warn!(
                %dataflow_id, %node_id, %output_id,
                "ignoring `{what}` for a dataflow that already finished \
                 (node outlived its dataflow); further such messages from \
                 this node are logged at debug level"
            );
        } else {
            tracing::debug!(
                %dataflow_id, %node_id, %output_id,
                "ignoring `{what}` for a dataflow that already finished"
            );
        }
    }

    async fn send_out(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        metadata: dora_message::metadata::Metadata,
        data: Option<DataMessage>,
    ) -> Result<(), eyre::ErrReport> {
        let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
            self.log_late_node_output(&dataflow_id, &node_id, &output_id, "send out");
            return Ok(());
        };
        let output_id_key = OutputId(node_id.clone(), output_id.clone());
        let remote_receivers = dataflow.open_external_mappings.contains(&output_id_key)
            || dataflow.enable_debug_inspection;
        let has_debug_watchers = dataflow.debug_topic_watchers.contains_key(&output_id_key);
        // `node_id`/`output_id` are not read past this call — the later
        // inter-daemon path uses `output_id_key` (cloned just above) instead —
        // so move them in rather than adding a second per-message clone of the
        // `NodeId`/`DataId` on this output-dispatch hot path.
        let data_bytes = send_output_to_local_receivers(
            node_id,
            output_id,
            dataflow,
            &metadata,
            data,
            &self.clock,
            Some(&self.ft_stats),
            remote_receivers || has_debug_watchers,
        )
        .await?;

        if !remote_receivers && !has_debug_watchers {
            return Ok(());
        }

        let output_id = output_id_key;
        let event = InterDaemonEvent::Output {
            dataflow_id,
            node_id: output_id.0.clone(),
            output_id: output_id.1.clone(),
            metadata,
            data: data_bytes,
        };
        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        }
        .serialize()
        .wrap_err("failed to serialize inter-daemon event")?;

        if has_debug_watchers {
            if remote_receivers {
                self.send_topic_debug_frames(dataflow_id, &output_id, serialized_event.clone())
                    .await?;
            } else {
                self.send_topic_debug_frames(dataflow_id, &output_id, serialized_event)
                    .await?;
                return Ok(());
            }
        }

        if remote_receivers {
            // Regular outputs are data-plane messages. Backpressure should drop
            // them instead of surfacing an error that stops the daemon event loop.
            self.try_send_to_remote_receivers(dataflow_id, &output_id, serialized_event)
                .await?;
        }

        Ok(())
    }

    fn output_sent(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        _metadata: dora_message::metadata::Metadata,
    ) -> Result<(), eyre::ErrReport> {
        let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
            self.log_late_node_output(&dataflow_id, &node_id, &output_id, "output sent");
            return Ok(());
        };
        note_output_sent_to_local_receivers(
            node_id,
            output_id,
            dataflow,
            &self.clock,
            Some(&self.ft_stats),
        );
        Ok(())
    }

    async fn send_to_remote_receivers(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        serialized_event: Vec<u8>,
    ) -> Result<(), eyre::Error> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;

        // Get or create publisher (lazy, cached per output)
        let publisher = match dataflow.publishers.entry(output_id.clone()) {
            std::collections::btree_map::Entry::Occupied(e) => e.get().clone(),
            std::collections::btree_map::Entry::Vacant(e) => {
                let publish_topic =
                    zenoh_daemon_control_topic(dataflow.id, &output_id.0, &output_id.1);
                tracing::debug!("declaring control publisher on {publish_topic}");
                let publisher = self
                    .zenoh_session
                    .declare_publisher(publish_topic)
                    .congestion_control(CongestionControl::Drop)
                    .express(true)
                    .priority(Priority::RealTime)
                    .await
                    .map_err(|err| eyre!(err))
                    .context("failed to create zenoh publisher")?;
                let arc = Arc::new(publisher);
                e.insert(arc.clone());
                arc
            }
        };
        let payload_len = serialized_event.len() as u64;

        let outbound = ZenohOutbound {
            publisher,
            serialized: serialized_event,
            payload_len,
            net_bytes_sent: dataflow.net_bytes_sent.clone(),
            net_messages_sent: dataflow.net_messages_sent.clone(),
            net_publish_failures: dataflow.net_publish_failures.clone(),
        };
        handle_control_enqueue_result(
            dataflow,
            try_enqueue_zenoh_outbound(&self.zenoh_control_tx, outbound),
        )
    }

    async fn try_send_to_remote_receivers(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        serialized_event: Vec<u8>,
    ) -> Result<(), eyre::Error> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;

        // Get or create publisher (lazy, cached per output)
        let publisher = match dataflow.publishers.entry(output_id.clone()) {
            std::collections::btree_map::Entry::Occupied(e) => e.get().clone(),
            std::collections::btree_map::Entry::Vacant(e) => {
                let publish_topic =
                    zenoh_daemon_control_topic(dataflow.id, &output_id.0, &output_id.1);
                tracing::debug!("declaring control publisher on {publish_topic}");
                let publisher = self
                    .zenoh_session
                    .declare_publisher(publish_topic)
                    .congestion_control(CongestionControl::Drop)
                    .express(true)
                    .priority(Priority::RealTime)
                    .await
                    .map_err(|err| eyre!(err))
                    .context("failed to create zenoh publisher")?;
                let arc = Arc::new(publisher);
                e.insert(arc.clone());
                arc
            }
        };
        let payload_len = serialized_event.len() as u64;

        let outbound = ZenohOutbound {
            publisher,
            serialized: serialized_event,
            payload_len,
            net_bytes_sent: dataflow.net_bytes_sent.clone(),
            net_messages_sent: dataflow.net_messages_sent.clone(),
            net_publish_failures: dataflow.net_publish_failures.clone(),
        };
        match try_enqueue_zenoh_outbound(&self.zenoh_publish_tx, outbound) {
            ZenohEnqueueResult::Queued => {}
            ZenohEnqueueResult::Full => {
                dataflow
                    .net_publish_failures
                    .fetch_add(1, atomic::Ordering::Relaxed);
                tracing::warn!(
                    "zenoh publish channel full ({ZENOH_PUBLISH_CHANNEL_CAPACITY}); \
                     dropping regular inter-daemon output"
                );
            }
            ZenohEnqueueResult::Closed => {
                dataflow
                    .net_publish_failures
                    .fetch_add(1, atomic::Ordering::Relaxed);
                tracing::error!("zenoh drain task is gone; inter-daemon publish channel closed");
            }
        }
        Ok(())
    }

    async fn send_topic_debug_frames(
        &self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        serialized_event: Vec<u8>,
    ) -> Result<(), eyre::Error> {
        let Some(sender) = &self.coordinator_sender else {
            return Ok(());
        };
        let Some(dataflow) = self.running.get(&dataflow_id) else {
            return Ok(());
        };
        let Some(subscription_ids) = dataflow.debug_topic_watchers.get(output_id) else {
            return Ok(());
        };
        let subscription_ids: Vec<_> = subscription_ids.iter().copied().collect();
        let subscription_count = subscription_ids.len();

        let message = serde_json::to_vec(&Timestamped {
            inner: CoordinatorRequest::Event {
                daemon_id: self.daemon_id.clone(),
                event: DaemonEvent::TopicDebugData {
                    dataflow_id,
                    subscription_ids,
                    payload: serialized_event,
                },
            },
            timestamp: self.clock.new_timestamp(),
        })?;
        match sender.try_send_event(&message) {
            Ok(()) => {}
            Err(crate::coordinator::TrySendEventError::Full) => {
                tracing::warn!(
                    %dataflow_id,
                    output = %format!("{}/{}", output_id.0, output_id.1),
                    subscriptions = subscription_count,
                    "dropping topic debug frame because coordinator WS send channel is full"
                );
            }
            Err(crate::coordinator::TrySendEventError::Closed) => {
                tracing::warn!(
                    %dataflow_id,
                    output = %format!("{}/{}", output_id.0, output_id.1),
                    subscriptions = subscription_count,
                    "dropping topic debug frame because coordinator WS send channel is closed"
                );
            }
            Err(crate::coordinator::TrySendEventError::InvalidUtf8(err)) => {
                return Err(eyre!(
                    "failed to encode topic debug frame for coordinator: {err}"
                ));
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
            let serialized_event = Timestamped {
                inner: InterDaemonEvent::OutputClosed {
                    dataflow_id,
                    node_id: output_id.0.clone(),
                    output_id: output_id.1.clone(),
                },
                timestamp: self.clock.new_timestamp(),
            }
            .serialize()
            .wrap_err("failed to serialize inter-daemon output-closed event")?;
            self.send_to_remote_receivers(dataflow_id, &output_id, serialized_event)
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
        // record that this node has connected — it stays a finish-straggler
        // candidate even if it later drops its event stream (dora#2270).
        dataflow.connected_nodes.insert(node_id.clone());

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
        // Restart bookkeeping needs BOTH conditions, and they diverge under
        // the drain opt-in:
        //   - "nothing left open" keeps the pre-existing behavior. Other
        //     code depends on it firing at subscribe time for source nodes
        //     (see the `grace_duration_kill` comment in `handle_node_stop`),
        //     and `is_drained` deliberately reports false for a source.
        //     Dropping it would let a source with `restart_policy: always`
        //     respawn forever, so its outputs never close and the graph
        //     hangs — the failure #2920 exists to prevent.
        //   - "is drained" covers the node we are about to tell to finish.
        //     Without it, such a node exits, gets restarted, is told to
        //     finish again, and loops until `max_restarts`.
        // In the default mode the two are the same predicate. A node with a
        // circuit-broken input is excluded either way: the input is
        // recoverable, so the node is not done and must keep its restart
        // policy — `disable_restart` is one-way, with no re-enable on
        // recovery. This matches `signal_all_inputs_closed_if_drained`.
        if (dataflow.open_inputs(&node_id).is_empty() || dataflow.is_drained(&node_id))
            && !dataflow.has_broken_input(&node_id)
            && let Some(node) = dataflow.running_nodes.get_mut(&node_id)
        {
            node.disable_restart();
        }
        // Sources are not told to finish. The check reads the recorded data
        // inputs rather than the descriptor: `descriptor.nodes[].inputs` is
        // empty for a runtime (`operators:`) node, whose inputs live under
        // `operators[].config.inputs`, so the descriptor calls every operator
        // node a source and skips the event — the hang this issue is about
        // (#2920). Under the opt-in `is_drained` already excludes sources;
        // this still matters in the default mode, where a source has no open
        // inputs and so looks drained.
        if dataflow.is_finished_non_source(&node_id)
            && send_with_timestamp(&event_sender, NodeEvent::AllInputsClosed, clock).ok()
                == Some(true)
        {
            dataflow.inc_pending(&node_id);
            dataflow
                .all_inputs_closed_at
                .insert(node_id.clone(), Instant::now());
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

        // Include outputs consumed only by nodes on other daemons
        // (`open_external_mappings`), not just those with a local consumer
        // (`mappings`). Otherwise a remote-only output never triggers an
        // `OutputClosed` event when the producing node finishes, leaving the
        // remote consumer's input open until it is force-killed.
        let outputs = dataflow.node_output_ids(node_id).into_iter().collect();

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

        Ok(())
    }

    async fn handle_node_stop(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
        dynamic_node: bool,
        exit_clean: bool,
    ) -> eyre::Result<()> {
        let result = self
            .handle_node_stop_inner(dataflow_id, node_id, dynamic_node, exit_clean)
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

    /// `exit_clean` indicates the process exited successfully (Success
    /// exit status). Combined with `restarts_disabled` (operator
    /// requested stop via `dora node stop` → `disable_restart()` set,
    /// even when the SIGTERM-induced exit code is non-zero) it produces
    /// the `clean_stop` flag sent to the coordinator: clean_stop = true
    /// → `NodeStatus::Stopped` (auto-expires from `dora node list` after
    /// the 60s grace), clean_stop = false → `NodeStatus::Failed` (stays
    /// visible so `dora doctor` keeps reporting it). A finish-straggler
    /// escalation (dora-rs/dora#2152) forces clean_stop = false unless
    /// the node still exited 0.
    async fn handle_node_stop_inner(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
        dynamic_node: bool,
        exit_clean: bool,
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
            None => {
                // The dataflow finished before this non-dynamic node's stop
                // event was processed. A node's exit and its `SpawnNodeResult`
                // arrive on independent paths, so `should_finish` can conclude
                // the dataflow is done (every remaining `running_nodes` entry is
                // dynamic) and `finish_dataflow` can run before this stop lands
                // — the same "two independent paths" ordering that produced the
                // late-output race fixed in #2742, just narrower. Bailing here
                // returns `Err` up through the daemon's main loop, unwinding
                // `run()` and tearing down the coordinator connection (or
                // failing the whole `dora run`). Treat the missing dataflow as
                // benign — matching the `dynamic_node` arm above and the
                // late-output handling in `send_out`/`output_sent`. Kept at
                // `warn` rather than `debug` because a non-dynamic node arriving
                // late is less expected than a dynamic one (#2986).
                tracing::warn!("node {dataflow_id}/{node_id} stopped after dataflow was done");
                return Ok(());
            }
        };

        let status = dataflow
            .pending_nodes
            .handle_node_stop(
                node_id,
                &mut self.coordinator_sender,
                &self.clock,
                &mut dataflow.cascading_error_causes,
                &mut logger,
            )
            .await?;

        // If this death completed the startup barrier (the dying node was the
        // last cohort member yet to subscribe), the barrier resolves to
        // `AllNodesReady` just as a final subscribe or a `RemoveNode` would —
        // and both of those callers start the dataflow here. Do the same, so a
        // non-cohort survivor that was answered `Ok(())` (since #2933) isn't
        // left parked on a dataflow that never starts and never tears down
        // (#2967). `start()` is idempotent and guarded by `dataflow_started`.
        if matches!(status, DataflowStatus::AllNodesReady) && !dataflow.dataflow_started {
            logger
                .log(
                    LogLevel::Info,
                    None,
                    Some("daemon".into()),
                    "startup barrier completed by a node exit, starting dataflow",
                )
                .await;
            dataflow.start(&self.events_tx, &self.clock).await?;
        }

        // node only reaches here if it will not be restarted
        let might_restart = false;

        self.handle_outputs_done(dataflow_id, node_id, might_restart)
            .await?;

        // Capture `restarts_disabled` BEFORE the remove. Combined with
        // `exit_clean` (passed from the caller, set when the exit_status
        // was Success), produces the `clean_stop` flag:
        //   - operator-requested stop: stop_single_node() set
        //     disable_restart, SIGTERM-induced exit is non-zero
        //     (exit_clean=false). restarts_disabled covers it.
        //   - node finished its own work and exited 0: exit_clean=true.
        //   - crash / panic / restart_policy=Never with non-zero exit:
        //     neither bit is set → clean_stop=false → `Failed`, so the
        //     row sticks around and `dora doctor` keeps reporting it.
        //   - finish-straggler escalation (dora-rs/dora#2152): the
        //     watchdog also goes through stop_single_node, so
        //     restarts_disabled is set — but a force-killed straggler is
        //     a node FAILURE and must not auto-expire from `dora node
        //     list` as a clean stop. `finish_escalated` overrides
        //     restarts_disabled (an escalated node that still exited 0
        //     keeps exit_clean=true and stays clean).
        let (should_finish, clean_stop) = {
            let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                format!(
                    "failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`"
                )
            })?;
            let restarts_disabled = dataflow
                .running_nodes
                .get(node_id)
                .map(|n| n.restarts_disabled())
                .unwrap_or(false);
            let finish_escalated = dataflow.finish_escalated.remove(node_id);
            dataflow.all_inputs_closed_at.remove(node_id);
            let clean_stop = exit_clean || (restarts_disabled && !finish_escalated);
            dataflow.running_nodes.remove(node_id);
            // Check if all remaining nodes are dynamic (won't send SpawnedNodeResult)
            let should_finish = !dataflow.pending_nodes.local_nodes_pending()
                && dataflow
                    .running_nodes
                    .iter()
                    .all(|(_id, n)| n.node_config.dynamic);
            (should_finish, clean_stop)
        };

        // Tell the coordinator the node is gone so its cached
        // `node_metrics[node_id]` row stops claiming `Running` with the
        // pre-exit PID/CPU/memory snapshot. Without this signal the
        // daemon's metrics-snapshot loop simply omits the dead node and
        // the coordinator's cache stays frozen at the last pre-exit
        // values forever.
        if let Some(sender) = self.coordinator_sender.as_mut() {
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: self.daemon_id.clone(),
                    event: DaemonEvent::NodeStopped {
                        dataflow_id,
                        node_id: node_id.clone(),
                        clean_stop,
                    },
                },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to serialize NodeStopped")?;
            if let Err(err) = sender.send_event(&msg).await {
                tracing::warn!(
                    %dataflow_id, %node_id,
                    "failed to send NodeStopped to coordinator: {err}"
                );
            }
        }

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

                let metadata = Arc::new(metadata);
                let mut closed = Vec::new();
                for (receiver_id, input_id) in subscribers {
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

                let metadata = Arc::new(metadata);
                let message = Arc::new(message);
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
                            metadata: metadata.clone(),
                            data: Some(message.clone()),
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

                // Convert to a self-describing Arrow IPC stream once, share the
                // sample across subscribers. The node-side receive path decodes
                // the IPC stream, so set the `arrow-ipc` framing parameter.
                use dora_arrow_convert::IntoArrow;
                let array = json.as_str().into_arrow();
                let array: dora_node_api::arrow::array::ArrayData = array.into();
                let ipc_bytes = match dora_node_api::arrow_utils::encode_arrow_ipc(&array) {
                    Ok(bytes) => bytes,
                    Err(e) => {
                        tracing::warn!("failed to Arrow-IPC-encode LogMessage: {e}");
                        return Ok(());
                    }
                };
                let sample: aligned_vec::AVec<u8, aligned_vec::ConstAlign<128>> =
                    aligned_vec::AVec::from_slice(128, &ipc_bytes);
                let data = Arc::new(DataMessage::Vec(sample));

                // Build the metadata once and share the `Arc` across subscribers.
                // It is identical for every delivery (only the `arrow-ipc`
                // framing parameter), so rebuilding it (and re-allocating the
                // `Arc`) per subscriber was wasted work — mirror the Timer/Logs
                // broadcast loops, which build the `Arc<Metadata>` once.
                let mut params = MetadataParameters::new();
                params.insert(
                    dora_message::metadata::FRAMING.to_string(),
                    dora_message::metadata::Parameter::String(
                        dora_message::metadata::FRAMING_ARROW_IPC.to_string(),
                    ),
                );
                let metadata = Arc::new(metadata::Metadata::from_parameters(
                    self.clock.new_timestamp(),
                    params,
                ));

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

                    let send_result = send_with_timestamp(
                        channel,
                        NodeEvent::Input {
                            id: sub.input_id.clone(),
                            metadata: metadata.clone(),
                            data: Some(data.clone()),
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
                generation,
                dynamic_node,
                exit_status,
                restart,
                restart_count,
                pid,
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

                let current_generation = self
                    .running
                    .get(&dataflow_id)
                    .and_then(|dataflow| dataflow.running_nodes.get(&node_id));
                // Deliberate semantics of the entry-absent case: an exit
                // arriving after `RemoveNode` took the entry out is dropped
                // here WITHOUT running the finish accounting below. That
                // means removing the last non-dynamic node leaves the
                // dataflow alive (zero running nodes) until an explicit stop
                // or a re-add — live-editing keep-alive, chosen over the
                // pre-generation behavior where such a stale exit could both
                // record a bogus result (dora-rs/dora#2926) and finish a
                // dataflow out from under a concurrent re-add of the same id.
                if !current_generation.is_some_and(|node| node.matches_generation(generation)) {
                    logger
                        .log(
                            LogLevel::Debug,
                            Some("daemon".into()),
                            format!(
                                "ignoring stale exit from pid {pid} (generation {generation}); \
                                 `{node_id}` has been removed or replaced"
                            ),
                        )
                        .await;
                    if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                        dataflow
                            .grace_duration_kills
                            .remove(&(node_id.clone(), generation));
                    }
                    return Ok(());
                }

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
                            .map(|d| {
                                d.grace_duration_kills
                                    .contains(&(node_id.clone(), generation))
                            })
                            .unwrap_or_default();
                        // Killed by the finish-straggler watchdog
                        // (dora-rs/dora#2152): the node blocked an
                        // otherwise-finished dataflow past the drain grace
                        // period. Unlike an operator-initiated stop this
                        // must NOT classify as clean — a node that needed
                        // force-killing during natural finish is a shutdown
                        // bug, and reporting success would hide every
                        // recurrence behind a green run. (A straggler that
                        // exits 0 on the escalation's Stop event takes the
                        // `Success` arm above and stays clean: it finished
                        // its work and responded to stop, just late.)
                        let finish_escalated = dataflow
                            .map(|d| d.finish_escalated.contains(&node_id))
                            .unwrap_or_default();
                        // The daemon explicitly sent SoftKill (SIGTERM)
                        // to this node as part of an operator-initiated
                        // stop (`dora stop`, `dora destroy`, `dora run
                        // --stop-after`, `dora node stop`, `dora node
                        // restart`) and the node responded by exiting.
                        // On Unix the exit reports as `Signal(15)`.
                        // Wrappers like `uv run python` catch SIGTERM
                        // and exit with code 143 (= 128 + 15) instead
                        // of propagating the signal, so `child.wait()`
                        // returns `ExitCode(143)` not `Signal(15)`.
                        // Same shape for SIGINT (2 / 130). On Windows the
                        // daemon's SoftKill sends `CTRL_BREAK_EVENT`, so a
                        // node without its own console handler reports
                        // `STATUS_CONTROL_C_EXIT` (`ExitCode(-1073741510)`)
                        // — the Windows analog (dora-rs/dora#2425). Treat any
                        // of those as a clean planned stop so `dora run
                        // --stop-after` doesn't report a fake "Node
                        // failed: exited with code 143" when the
                        // dataflow shut down exactly as requested
                        // (dora-rs/dora#1882). See `is_sigterm_like_exit`.
                        //
                        // `grace_duration_kill` is the right
                        // discriminant — not `restarts_disabled` —
                        // because `disable_restart()` fires at subscribe
                        // time for source nodes (see lib.rs:3203 where
                        // `open_inputs().is_empty()` triggers it).
                        // Using `restarts_disabled` would silently
                        // swallow externally-sent SIGTERMs on source
                        // nodes (e.g. `kill -TERM <pid>`) as clean.
                        // `grace_duration_kills` is only populated by
                        // the daemon's own SoftKill/Kill submission
                        // path (`running_dataflow.rs::stop_all` and
                        // `::send_stop_and_schedule_kill`), so it
                        // accurately encodes "daemon asked this node to
                        // stop."
                        //
                        // SIGKILL exits (Signal(9), happens when the
                        // node didn't respond to SoftKill within the
                        // secondary grace) fall through to the existing
                        // `GraceDuration` branch — that's the original
                        // semantic of GraceDuration and we want to
                        // preserve it.
                        //
                        // Cascading failures still win — if some other
                        // node failed first and this one was killed as
                        // collateral, we want to surface the original
                        // failure rather than hide it behind the
                        // shutdown that followed.
                        let is_sigterm_like = is_sigterm_like_exit(&exit_status);
                        if caused_by_node.is_none()
                            && grace_duration_kill
                            && is_sigterm_like
                            && !finish_escalated
                        {
                            logger
                                .log(
                                    LogLevel::Info,
                                    Some("daemon".into()),
                                    format!(
                                        "`{node_id}` exited with {exit_status:?} during planned stop; treating as clean"
                                    ),
                                )
                                .await;
                            Ok(())
                        } else {
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
                                None if grace_duration_kill || finish_escalated => {
                                    NodeErrorCause::GraceDuration
                                }
                                None => {
                                    let cause = dataflow
                                        .and_then(|d| d.node_stderr_most_recent.get(&node_id))
                                        .map(|queue| {
                                            let mut lines = Vec::new();
                                            if queue.is_full() {
                                                lines.push("[...]\n".into());
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
                    }
                };

                // Drop the consumed kill marker. `grace_duration_kills` is
                // keyed by `(node_id, generation)`, so a successor can no
                // longer inherit its predecessor's marker structurally —
                // removal here is hygiene for this incarnation's own entry
                // (it was consumed classifying this exit), not the
                // cross-incarnation leak protection it used to be. Same for
                // the drain clock: a respawned node under the same id must
                // start fresh.
                // (`finish_escalated` is NOT cleared here — it is read
                // and consumed by `handle_node_stop_inner` below to keep
                // the coordinator-facing `clean_stop` flag honest; an
                // escalated node never restarts, so it cannot leak into
                // a next incarnation.)
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    dataflow
                        .grace_duration_kills
                        .remove(&(node_id.clone(), generation));
                    dataflow.all_inputs_closed_at.remove(&node_id);
                    // a respawned node must re-subscribe before it counts as
                    // connected, else a slow restart could be silence-escalated
                    // mid-startup (dora-rs/dora#2270).
                    dataflow.connected_nodes.remove(&node_id);
                }

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
                    // Send NodeFailed events to downstream nodes when a
                    // node exits with a non-zero exit code (matches
                    // upstream dora behavior for error propagation).
                    if let Err(e) = &node_result
                        && let Some(dataflow) = self.running.get(&dataflow_id)
                    {
                        dataflow.propagate_node_failed(&node_id, &e.to_string(), &self.clock);
                    }

                    let exit_clean = node_result.is_ok();
                    self.dataflow_node_results
                        .entry(dataflow_id)
                        .or_default()
                        .insert(node_id.clone(), node_result);

                    self.handle_node_stop(dataflow_id, &node_id, dynamic_node, exit_clean)
                        .await?;
                }
            }
            DoraEvent::ProcessHandleReplaced {
                dataflow_id,
                node_id,
                previous_generation,
                new_generation,
                new_handle,
            } => {
                // The per-node restart_loop just spawned a replacement
                // process with a fresh op_tx/op_rx pair. Swap it into
                // running_nodes so subsequent stop/kill operations
                // target the new incarnation rather than a dead
                // predecessor's channel (dora-rs/adora#152).
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                        match node.replace_process_handle(
                            previous_generation,
                            new_generation,
                            new_handle,
                        ) {
                            running_dataflow::HandleReplacement::Replaced => {}
                            running_dataflow::HandleReplacement::RejectedTeardown(new_handle) => {
                                dataflow.stop_rejected_replacement(
                                    &node_id,
                                    new_generation,
                                    new_handle,
                                );
                                tracing::info!(
                                    %dataflow_id,
                                    %node_id,
                                    new_generation,
                                    "teardown won the respawn race: stopping the replacement \
                                     process with the active dataflow stop policy"
                                );
                            }
                            running_dataflow::HandleReplacement::RejectedStale => {
                                tracing::warn!(
                                    %dataflow_id,
                                    %node_id,
                                    previous_generation,
                                    current_generation = node.generation,
                                    "ignoring stale ProcessHandleReplaced event"
                                );
                            }
                        }
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

fn apply_state_catch_up_entries(
    dataflow: &RunningDataflow,
    entries: &[StateCatchUpEntry],
    clock: &HLC,
) -> u64 {
    let mut applied_through = 0;

    for entry in entries {
        let delivered = match &entry.operation {
            StateCatchUpOperation::SetParam {
                node_id,
                key,
                value,
            } => {
                let Some(channel) = dataflow.subscribe_channels.get(node_id) else {
                    tracing::warn!(
                        "catch-up: node `{node_id}` not connected; stopping replay at seq {}",
                        entry.sequence
                    );
                    break;
                };
                let value_json = match serde_json::to_vec(value) {
                    Ok(bytes) => bytes,
                    Err(err) => {
                        tracing::warn!(
                            "catch-up: failed to serialize param value for node `{node_id}` at seq {}: {err}",
                            entry.sequence
                        );
                        break;
                    }
                };
                match send_with_timestamp(
                    channel,
                    NodeEvent::ParamUpdate {
                        key: key.clone(),
                        value_json,
                    },
                    clock,
                ) {
                    Ok(true) => {
                        dataflow.inc_pending(node_id);
                        true
                    }
                    Ok(false) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel full; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                    Err(_) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel closed; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                }
            }
            StateCatchUpOperation::DeleteParam { node_id, key } => {
                let Some(channel) = dataflow.subscribe_channels.get(node_id) else {
                    tracing::warn!(
                        "catch-up: node `{node_id}` not connected; stopping replay at seq {}",
                        entry.sequence
                    );
                    break;
                };
                match send_with_timestamp(
                    channel,
                    NodeEvent::ParamDeleted { key: key.clone() },
                    clock,
                ) {
                    Ok(true) => {
                        dataflow.inc_pending(node_id);
                        true
                    }
                    Ok(false) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel full; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                    Err(_) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel closed; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                }
            }
        };

        if !delivered {
            break;
        }
        applied_through = entry.sequence;
    }

    applied_through
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
        // `tail` is the client-supplied `--tail N` line count with no upper
        // bound, so `estimated_line_length * tail` can overflow `usize` (a debug
        // panic in the log task; a benign wrap in release). It only feeds a
        // buffer-growth heuristic, so saturate instead of overflowing.
        let estimated_buffer_length = estimated_line_length.saturating_mul(tail);
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
    // Events from the dynamic-node listener. The listener is bound once by the
    // caller (for the daemon's lifetime) and its receiver passed in, rather than
    // rebinding the port on every reconnect (dora-rs/dora#1999).
    dynamic_node_events_rx: flume::Receiver<Timestamped<DynamicNodeEventWrapper>>,
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
    let dynamic_node_events = dynamic_node_events_rx.into_stream().map(|e| Timestamped {
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

fn note_output_sent_to_local_receivers(
    node_id: NodeId,
    output_id: DataId,
    dataflow: &mut RunningDataflow,
    clock: &HLC,
    ft_stats: Option<&FaultToleranceStats>,
) {
    // Both side effects below are gated on a non-empty `input_deadlines`
    // (deadline refresh) or `broken_inputs` (circuit-breaker recovery). When
    // neither feature is configured — the common case — the whole loop is a
    // no-op, so skip it entirely rather than paying a clock read plus a
    // `subscribe_channels` lookup per receiver on every `OutputSent`.
    if dataflow.input_deadlines.is_empty() && dataflow.broken_inputs.is_empty() {
        return;
    }

    let empty_set = BTreeSet::new();
    let output_id = OutputId(node_id, output_id);
    let local_receivers = dataflow.mappings.get(&output_id).unwrap_or(&empty_set);
    let now = Instant::now();

    for (receiver_id, input_id) in local_receivers {
        // Refresh the input deadline only when the receiver is actually
        // keeping up. Since #1787 the data payload is published directly
        // over zenoh (not routed through the daemon), so the bare
        // `OutputSent` notification is *not* a delivery confirmation — the
        // node-side zenoh callback drops the input with `try_send` when its
        // event channel is full. Treating `OutputSent` as delivery made
        // `input_timeout` deadlines never fire for a slow consumer (#2021).
        //
        // We use the receiver's daemon-side channel headroom as a
        // backpressure proxy (the same signal `send_output_to_local_receivers`
        // gates on): a node that has fallen far enough behind to saturate its
        // channel is also dropping the zenoh payloads, so its deadline must be
        // allowed to expire. A node that is keeping up still gets refreshed.
        //
        // A *missing* channel means the receiver has no daemon-side event
        // stream at all — e.g. it dropped its stream (`EventStreamDropped`)
        // or has not subscribed yet. Such a receiver is not draining inputs
        // either, so it must NOT count as keeping up; mirror
        // `send_output_to_local_receivers`, which does nothing without a
        // channel.
        let receiver_keeping_up = dataflow
            .subscribe_channels
            .get(receiver_id)
            .is_some_and(|channel| channel.capacity() >= CONTROL_EVENT_HEADROOM);
        // Looking up these maps requires cloning the `(NodeId, DataId)` key (the
        // tuple key type can't borrow). Both maps are empty unless input
        // deadlines or circuit breakers are configured, so skip the per-message
        // key clone + hash in the common case — mirroring
        // `send_output_to_local_receivers`.
        if receiver_keeping_up
            && !dataflow.input_deadlines.is_empty()
            && let Some(deadline) = dataflow
                .input_deadlines
                .get_mut(&(receiver_id.clone(), input_id.clone()))
        {
            deadline.last_received = Some(now);
        }

        // Circuit-breaker recovery must be gated on the same backpressure
        // signal as the deadline refresh above. A bare `OutputSent` is not a
        // delivery confirmation (see the comment above), so re-opening a broken
        // input while the receiver is still saturated only makes it flap
        // `broken ↔ recovered` every `input_timeout` without any data actually
        // getting through (#2627). Leave the `broken_inputs` entry in place
        // until the receiver drains enough to keep up — matching the recovery
        // in `send_output_to_local_receivers`, which only fires inside the
        // successful `try_send` arm.
        if !receiver_keeping_up {
            continue;
        }

        // `broken_inputs` is empty unless circuit breakers are configured, so
        // skip the per-message `(NodeId, DataId)` key clone + hash + remove in
        // the common case — mirroring `send_output_to_local_receivers`.
        if dataflow.broken_inputs.is_empty() {
            continue;
        }
        let Some(timeout) = dataflow
            .broken_inputs
            .remove(&(receiver_id.clone(), input_id.clone()))
        else {
            continue;
        };

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
                last_received: Some(now),
            },
        );

        let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
            continue;
        };
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
                tracing::warn!("failed to send InputRecovered for `{receiver_id}/{input_id}`");
            }
        }
    }
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
    let data = data.map(Arc::new);
    let mut closed = Vec::new();
    // Clone the metadata into an `Arc` lazily, on the first actual delivery.
    // Fan-out clones are then O(1) atomic ref bumps instead of O(payload_size)
    // memcpy. For a pure-remote output topology `local_receivers` is empty (all
    // subscribers live on other daemons), so this deep clone (a `BTreeMap` of
    // owned `Parameter`s) is skipped entirely rather than built and dropped on
    // every such message.
    let mut metadata_arc = None;
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
                metadata: metadata_arc
                    .get_or_insert_with(|| Arc::new(metadata.clone()))
                    .clone(),
                data: data.clone(),
            };
            match channel.try_send(Timestamped {
                inner: item,
                timestamp,
            }) {
                Ok(()) => {
                    dataflow.inc_pending(receiver_id);
                    // Looking up these maps requires cloning the `(NodeId, DataId)`
                    // key (the tuple key type can't borrow). Both maps are empty
                    // unless input deadlines or circuit breakers are configured, so
                    // skip the per-message key clone + hash in the common case.
                    if !dataflow.input_deadlines.is_empty()
                        && let Some(deadline) = dataflow
                            .input_deadlines
                            .get_mut(&(receiver_id.clone(), input_id.clone()))
                    {
                        deadline.last_received = Some(Instant::now());
                    }
                    // Circuit breaker recovery: re-open broken input
                    if !dataflow.broken_inputs.is_empty()
                        && let Some(timeout) = dataflow
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
    let data_bytes = if need_data_bytes {
        // If no local receiver kept a reference (pure remote topology), the
        // Arc is uniquely owned and the payload can be moved out instead of
        // copied.
        data.map(|arc| match Arc::try_unwrap(arc) {
            Ok(DataMessage::Vec(v)) => v,
            Err(arc) => {
                let DataMessage::Vec(v) = arc.as_ref();
                v.clone()
            }
        })
    } else {
        None
    };
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

    // Drop any armed deadline for this input. A closed input can never
    // meaningfully time out, and leaving the entry behind lets
    // `check_input_timeouts` fire on it later, insert a `broken_inputs`
    // record, and then no-op in `break_input` (the input is already gone from
    // `open_inputs`) — orphaning that record so `has_broken_input` stays true
    // forever and the drained node is never sent `AllInputsClosed` (#2968).
    dataflow
        .input_deadlines
        .remove(&(receiver_id.clone(), input_id.clone()));

    let was_open = dataflow
        .open_inputs
        .get_mut(receiver_id)
        .map(|inputs| inputs.remove(input_id))
        .unwrap_or(false);

    if !was_open && !was_broken {
        return;
    }

    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id)
        && was_open
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

    signal_all_inputs_closed_if_drained(dataflow, receiver_id, clock);
}

/// If `receiver_id` has finished, disable its restart policy and notify it
/// that all inputs are closed.
///
/// "Finished" is [`RunningDataflow::should_signal_all_inputs_closed`], which
/// under `--exit-when-nodes-finish` means its data inputs have closed even
/// while a timer keeps ticking.
///
/// Shared drain-completion tail of [`close_input`] and [`break_input`]; a
/// no-op if the node still has open/broken inputs or has no subscribe channel.
fn signal_all_inputs_closed_if_drained(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    clock: &HLC,
) {
    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
        return;
    };
    // As at the subscribe site: either "nothing left open" (pre-existing
    // behavior, and true for a source) or "drained" (the node we are about
    // to tell to finish) disables restart.
    if (dataflow.open_inputs(receiver_id).is_empty() || dataflow.is_drained(receiver_id))
        && !dataflow.has_broken_input(receiver_id)
        && let Some(node) = dataflow.running_nodes.get_mut(receiver_id)
    {
        node.disable_restart();
    }
    if dataflow.is_finished(receiver_id)
        && send_with_timestamp(channel, NodeEvent::AllInputsClosed, clock).ok() == Some(true)
    {
        dataflow.inc_pending(receiver_id);
        dataflow
            .all_inputs_closed_at
            .insert(receiver_id.clone(), Instant::now());
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
        // The input already left `open_inputs` (e.g. it was closed before this
        // timeout fired). The caller inserted a `broken_inputs` record before
        // calling us; a no-op break would orphan it, permanently pinning
        // `has_broken_input` true. Roll that insert back so we don't leave the
        // node undrainable (#2968).
        dataflow
            .broken_inputs
            .remove(&(receiver_id.clone(), input_id.clone()));
        return;
    }
    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id)
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

    signal_all_inputs_closed_if_drained(dataflow, receiver_id, clock);
}

/// Grace period used when the finish-straggler watchdog is enabled but
/// `DORA_FINISH_DRAIN_GRACE_SECS` is set to an unparseable value. Conservative:
/// a sink may legitimately keep working for a while after its inputs close
/// (flushing recordings, final writes).
const DEFAULT_FINISH_DRAIN_GRACE: Duration = Duration::from_secs(120);

/// Windows `STATUS_CONTROL_C_EXIT`. A process terminated by an unhandled
/// console `CTRL_C` / `CTRL_BREAK` event exits with this NTSTATUS, which Rust's
/// [`std::process::ExitStatus::code`] surfaces as this `i32` (`0xC000013A`).
/// The daemon's Windows `SoftKill` stops nodes with
/// `GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT)` (see `running_dataflow.rs`), so
/// a node that doesn't install its own console handler reports this code on a
/// planned stop — the Windows analog of Unix `Signal(15)` or the `143` wrapper
/// exit (dora-rs/dora#2425).
const STATUS_CONTROL_C_EXIT: i32 = -1073741510;

/// Whether `exit_status` has the *shape* of a node that exited because the
/// daemon asked it to stop (SoftKill), rather than because of an application
/// error.
///
/// Callers must additionally gate on `grace_duration_kills` — this predicate
/// only recognises the shape of a stop-induced exit, not whether the daemon
/// actually initiated one. A node that produces one of these codes on its own
/// (without a preceding daemon SoftKill) is still reported as a failure.
///
/// - Unix: `SIGTERM` (15) / `SIGINT` (2) surface as `Signal`.
/// - Wrappers such as `uv run python` catch the signal and exit `128 + signo`
///   (143 / 130) instead of propagating it, so the code appears as `ExitCode`.
/// - Windows: an unhandled `CTRL_BREAK_EVENT` terminates the node with
///   [`STATUS_CONTROL_C_EXIT`] (dora-rs/dora#2425).
fn is_sigterm_like_exit(exit_status: &NodeExitStatus) -> bool {
    matches!(
        exit_status,
        NodeExitStatus::Signal(15)
            | NodeExitStatus::Signal(2)
            | NodeExitStatus::ExitCode(143)
            | NodeExitStatus::ExitCode(130)
            | NodeExitStatus::ExitCode(STATUS_CONTROL_C_EXIT)
    )
}

/// Decide whether the health-check watchdog should kill a node.
///
/// Returns `true` only once the node has **connected** at least once (sent its
/// first `DaemonRequest`) and has then been silent for longer than `timeout`.
///
/// A node that has not yet connected is still starting up and must never be
/// killed by this watchdog: `last_activity` is seeded to the spawn timestamp
/// (see `spawner.rs` / `prepared.rs`), so without the `connected` gate the
/// timeout clock would start ticking at spawn — before the node has had any
/// chance to communicate — and a node whose legitimate cold start exceeds
/// `health_check_timeout` would be SIGKILLed mid-startup, mirroring the trap
/// the finish-straggler watchdog was explicitly fixed for (#2937).
fn health_check_should_kill(
    connected: bool,
    last_activity_millis: u64,
    now_millis: u64,
    timeout: Duration,
) -> bool {
    if !connected {
        return false;
    }
    let elapsed_ms = now_millis.saturating_sub(last_activity_millis);
    elapsed_ms > timeout.as_millis() as u64
}

/// Grace period before the finish-straggler watchdog escalates a stuck node, or
/// `None` if the watchdog has been explicitly **disabled**.
///
/// The watchdog is **on by default** (dora-rs/dora#2270): a node still stuck
/// past the grace once its dataflow is otherwise finished is escalated rather
/// than hanging until an external timeout. `DORA_FINISH_DRAIN_GRACE_SECS` tunes
/// it — a whole number of seconds sets the grace; `off`/`disabled` opts out
/// entirely (the escape hatch for a deployment that hits a false positive). It
/// shipped dark first and was validated on nightly + a deterministic e2e
/// (#2271, #2276, #2280) before this default flip.
fn finish_drain_grace() -> Option<Duration> {
    parse_finish_drain_grace(
        std::env::var("DORA_FINISH_DRAIN_GRACE_SECS")
            .ok()
            .as_deref(),
    )
}

fn parse_finish_drain_grace(value: Option<&str>) -> Option<Duration> {
    let Some(value) = value else {
        // unset → enabled at the conservative default grace (on by default)
        return Some(DEFAULT_FINISH_DRAIN_GRACE);
    };
    // explicit opt-out escape hatch for a deployment that hits a false positive
    if value.eq_ignore_ascii_case("off") || value.eq_ignore_ascii_case("disabled") {
        return None;
    }
    match value.parse::<u64>() {
        Ok(secs) => Some(Duration::from_secs(secs)),
        Err(_) => {
            static WARNED: std::sync::atomic::AtomicBool =
                std::sync::atomic::AtomicBool::new(false);
            if !WARNED.swap(true, atomic::Ordering::Relaxed) {
                tracing::warn!(
                    "invalid DORA_FINISH_DRAIN_GRACE_SECS value `{value}` \
                     (expected whole seconds or `off`); using the default of {}s",
                    DEFAULT_FINISH_DRAIN_GRACE.as_secs()
                );
            }
            Some(DEFAULT_FINISH_DRAIN_GRACE)
        }
    }
}

/// Best-effort stack capture of a stuck node before it is killed, so the
/// "why didn't it exit" half of dora-rs/dora#2152 stays answerable from
/// logs. macOS ships `sample`; other platforms have no portable
/// unprivileged stack tool, so only the escalation log is emitted there.
/// Runs as a background task — the kill ladder's grace period (10 s before
/// SIGTERM) leaves ample time for the capture to finish first.
fn spawn_stack_sample_capture(node_id: NodeId, pid: Option<u32>) {
    let Some(pid) = pid.filter(|pid| *pid != 0) else {
        tracing::warn!("no pid recorded for stuck node `{node_id}`; skipping stack sample");
        return;
    };
    if !cfg!(target_os = "macos") {
        tracing::info!(
            "stack sample capture is not supported on this platform \
             (stuck node `{node_id}`, pid {pid})"
        );
        return;
    }
    tokio::spawn(async move {
        match tokio::process::Command::new("sample")
            .args([&pid.to_string(), "1"])
            .output()
            .await
        {
            Ok(output) if output.status.success() => {
                let mut text = String::from_utf8_lossy(&output.stdout).into_owned();
                const MAX_SAMPLE_BYTES: usize = 64 * 1024;
                if text.len() > MAX_SAMPLE_BYTES {
                    let mut cut = MAX_SAMPLE_BYTES;
                    while !text.is_char_boundary(cut) {
                        cut -= 1;
                    }
                    text.truncate(cut);
                    text.push_str("\n…(truncated)");
                }
                tracing::warn!("stack sample of stuck node `{node_id}` (pid {pid}):\n{text}");
            }
            Ok(output) => {
                tracing::warn!(
                    "`sample {pid}` failed for stuck node `{node_id}`: {}",
                    String::from_utf8_lossy(&output.stderr)
                );
            }
            Err(err) => {
                tracing::warn!("failed to run `sample` for stuck node `{node_id}`: {err}");
            }
        }
    });
}

// RunningDataflow and related types are in running_dataflow.rs
// FaultToleranceStats and CascadingErrorCauses are in fault_tolerance.rs

/// Treat SIGTERM/SIGHUP like Ctrl-C, but ONLY for the run-a-single-
/// dataflow modes (`dora run`, `dora daemon --run-dataflow`).
///
/// Without this, killing the CLI runs no teardown at all and every node
/// it spawned is orphaned: nodes are deliberately spawned as
/// process-group leaders (so a terminal Ctrl-C cannot kill them out from
/// under the daemon), which leaves an orphan with `ppid=1` and its own
/// pgid — unreachable by both inherited signal delivery and a group-kill
/// of the CLI. Only this teardown can reap them (dora-rs/dora#2920).
///
/// Deliberately scoped here rather than enabling `ctrlc`'s `termination`
/// feature. `dora run`, `dora daemon` and `dora coordinator` are
/// subcommands of ONE binary, and cargo unifies features, so that flag
/// would change signal handling process-wide — making SIGTERM a graceful
/// request everywhere. That is wrong for the long-lived services: a
/// `pkill -TERM` of the coordinator would gracefully destroy its daemons
/// instead of crashing (which the nightly crash-recovery jobs rely on),
/// and it would override the `SIG_IGN` that `nohup` installs for
/// `dora cluster up`'s remote daemons.
///
/// Escalates on the same ladder as the ctrl-c handler below, and for the
/// same reason: aborting on the SECOND signal would kill the process
/// while the first signal's teardown is still running, orphaning exactly
/// the node processes this exists to reap. So the second signal takes
/// the `SecondCtrlC` path — an early but still unwinding exit, which
/// drops the `RunningDataflow` and with it every `ProcessHandle` — and
/// only a third aborts outright.
///
/// Delivery uses `try_send` rather than an await so a wedged event loop
/// cannot swallow the escalation. A failure to deliver the FIRST signal
/// is not fatal by itself (the operator can signal again); by the second
/// there is nothing left to wait for.
fn set_up_termination_handler(clock: Arc<HLC>) -> tokio::sync::mpsc::Receiver<Timestamped<Event>> {
    // Room for both rungs of the ladder, so a queued `CtrlC` cannot make
    // the follow-up `SecondCtrlC` undeliverable.
    let (tx, rx) = mpsc::channel(2);

    #[cfg(unix)]
    tokio::spawn(async move {
        use tokio::signal::unix::{SignalKind, signal};
        let (mut sigterm, mut sighup) = match (
            signal(SignalKind::terminate()),
            signal(SignalKind::hangup()),
        ) {
            (Ok(term), Ok(hup)) => (term, hup),
            _ => {
                tracing::warn!(
                    "failed to install SIGTERM/SIGHUP handler; killing this process \
                         will leak its node processes"
                );
                return;
            }
        };

        let mut signals_seen = 0_u32;
        loop {
            tokio::select! {
                _ = sigterm.recv() => {}
                _ = sighup.recv() => {}
            }
            signals_seen += 1;
            let event = match signals_seen {
                1 => {
                    tracing::info!("received termination signal -> stopping dataflow");
                    Event::CtrlC
                }
                2 => {
                    tracing::warn!("received second termination signal -> exiting early");
                    Event::SecondCtrlC
                }
                _ => {
                    tracing::warn!("received third termination signal -> aborting immediately");
                    std::process::abort();
                }
            };
            if tx
                .try_send(Timestamped {
                    inner: event,
                    timestamp: clock.new_timestamp(),
                })
                .is_err()
            {
                // The event loop is not consuming. On the first signal
                // that is worth reporting but not worth killing over —
                // teardown may still be in flight, and a kill here would
                // orphan the nodes. A second undeliverable signal means
                // nothing is going to drain it.
                if signals_seen >= 2 {
                    tracing::warn!("could not deliver termination event -> aborting immediately");
                    std::process::abort();
                }
                tracing::warn!("could not deliver termination event; signal again to force exit");
            }
        }
    });

    #[cfg(not(unix))]
    {
        // No SIGTERM/SIGHUP on Windows; console close is already covered
        // by the ctrl-c handler. Keep the sender alive so the receiver
        // pends forever instead of reading as an immediate event.
        let _ = &clock;
        std::mem::forget(tx);
    }

    rx
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
                shared_memory_pool_size: None,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ZenohEnqueueResult {
    Queued,
    Full,
    Closed,
}

fn try_enqueue_zenoh_outbound<T>(tx: &mpsc::Sender<T>, outbound: T) -> ZenohEnqueueResult {
    match tx.try_send(outbound) {
        Ok(()) => ZenohEnqueueResult::Queued,
        Err(mpsc::error::TrySendError::Full(_)) => ZenohEnqueueResult::Full,
        Err(mpsc::error::TrySendError::Closed(_)) => ZenohEnqueueResult::Closed,
    }
}

async fn publish_zenoh_outbound(msg: ZenohOutbound) {
    if let Err(e) = msg.publisher.put(msg.serialized).await {
        tracing::error!("zenoh publish failed: {e}");
        msg.net_publish_failures
            .fetch_add(1, atomic::Ordering::Relaxed);
        return;
    }
    // Relaxed ordering is correct: counters are read-only for metrics
    // reporting and never used as synchronization guards.
    msg.net_bytes_sent
        .fetch_add(msg.payload_len, atomic::Ordering::Relaxed);
    msg.net_messages_sent
        .fetch_add(1, atomic::Ordering::Relaxed);
}

fn handle_control_enqueue_result(
    dataflow: &RunningDataflow,
    result: ZenohEnqueueResult,
) -> eyre::Result<()> {
    match result {
        ZenohEnqueueResult::Queued => Ok(()),
        ZenohEnqueueResult::Full => {
            dataflow
                .net_publish_failures
                .fetch_add(1, atomic::Ordering::Relaxed);
            tracing::warn!(
                "zenoh control pending queue full ({ZENOH_CONTROL_PENDING_CHANNEL_CAPACITY}); \
                 dropping inter-daemon control event"
            );
            Ok(())
        }
        ZenohEnqueueResult::Closed => Err(eyre!(
            "zenoh control forwarder is gone — inter-daemon control channel closed"
        )),
    }
}

/// Cached `@schema` bytes (keyed by their FNV-1a hash, most-recent last) for
/// one debug-watched output, shared between the `@schema` subscriber callback
/// and the data task. Retains several schemas — mirroring the node receive
/// path's `InputDecoder` retention — so an output that interleaves schemas
/// (e.g. schema-once batches of schema A between large full streams of schema
/// B) doesn't have its schema-once schema clobbered, and a stale `@schema`
/// history-query result arriving after an in-band prime can't evict the
/// fresher schema (both stay retained).
type DebugSchemaCache = Arc<std::sync::Mutex<Vec<(u64, Vec<u8>)>>>;

/// Retention bound for [`DebugSchemaCache`], matching the node receive path's
/// `InputDecoder` (see `MAX_RETAINED_SCHEMAS` in dora-node-api's `ipc_encode`).
const MAX_DEBUG_SCHEMAS: usize = 8;

/// Remember `schema` for `hash` in the debug cache (most-recent last), evicting
/// the oldest entry beyond [`MAX_DEBUG_SCHEMAS`]. An already-retained hash is
/// moved to the back without re-copying its bytes.
fn retain_debug_schema(cache: &mut Vec<(u64, Vec<u8>)>, hash: u64, schema: &[u8]) {
    if let Some(pos) = cache.iter().position(|(h, _)| *h == hash) {
        let entry = cache.remove(pos);
        cache.push(entry);
        return;
    }
    cache.push((hash, schema.to_vec()));
    if cache.len() > MAX_DEBUG_SCHEMAS {
        cache.remove(0);
    }
}

/// Rebuild a self-describing Arrow IPC stream for a `dora topic` debug frame.
///
/// Small node→node outputs travel as schema-once batches: the schema is
/// published once on the `@schema` subtopic and each data sample carries only
/// the schema-less record batch (`DoraNode::publish_schema_once`). The
/// `dora topic` decoder expects a full self-describing stream, so for such a
/// batch we prepend the retained schema block — concatenation reconstructs the
/// exact original stream (`schema_block ++ batch_slice == full_stream`).
///
/// Returns `None` (drop the inspection frame) when the batch is schema-once but
/// no schema matching its `SCHEMA_HASH` has been retained yet — a transient,
/// inspection-only gap. A full self-describing payload (no `SCHEMA_HASH`) is
/// forwarded as-is.
fn rebuild_debug_topic_stream(
    cache: &mut Vec<(u64, Vec<u8>)>,
    parameters: &dora_message::metadata::MetadataParameters,
    payload: &[u8],
) -> Option<Vec<u8>> {
    use dora_message::metadata::{SCHEMA_HASH, carries_pattern_correlation, get_integer_param};

    match get_integer_param(parameters, SCHEMA_HASH) {
        Some(hash) => {
            // Only rebuild with the schema matching this batch's hash —
            // otherwise wait for the right `@schema` rather than emit a stream
            // the CLI would decode against the wrong (stale/future) schema.
            let (_, schema) = cache.iter().find(|(h, _)| *h == hash as u64)?;
            let mut full = Vec::with_capacity(schema.len() + payload.len());
            full.extend_from_slice(schema);
            full.extend_from_slice(payload);
            Some(full)
        }
        None => {
            // In-band priming, mirroring the node receive path: a full
            // self-describing stream carries the same schema block the producer
            // publishes on `@schema`, so retain it here — then the schema-less
            // batches that follow rebuild even if the `@schema` subscription
            // missed the (single) schema emission. The producer guarantees such
            // a full stream for the first message of every output and
            // periodically thereafter (`DoraNode::publish_schema_once`).
            // Service/action messages (pattern correlation) are excluded from
            // schema-once; don't let their per-request schemas churn the cache.
            if !carries_pattern_correlation(parameters)
                && let Some((hash, schema)) =
                    dora_node_api::arrow_utils::ipc_encode::schema_block_and_hash(payload)
            {
                retain_debug_schema(cache, hash, schema);
            }
            Some(payload.to_vec())
        }
    }
}

#[cfg(test)]
mod debug_topic_tests {
    use super::rebuild_debug_topic_stream;
    use dora_message::metadata::{
        MetadataParameters, Parameter, SCHEMA_HASH, WIRE_SIZE, debug_frame_wire_size, fnv1a,
        get_integer_param, strip_internal_parameters,
    };

    fn params_with_schema_hash(hash: u64) -> MetadataParameters {
        let mut params = MetadataParameters::default();
        params.insert(SCHEMA_HASH.to_string(), Parameter::Integer(hash as i64));
        params
    }

    /// Wrap `msg` in an IPC message frame (continuation marker + length prefix)
    /// so `schema_block_len` parses it like a real schema block.
    fn framed(msg: &[u8]) -> Vec<u8> {
        let mut out = vec![0xff, 0xff, 0xff, 0xff];
        out.extend_from_slice(&(msg.len() as i32).to_le_bytes());
        out.extend_from_slice(msg);
        out
    }

    #[test]
    fn full_stream_payload_is_forwarded_verbatim() {
        // No SCHEMA_HASH ⇒ already a self-describing stream (large/SHM or
        // service/action output); forward the bytes unchanged.
        let payload = b"self-describing-stream".to_vec();
        let mut cache = Vec::new();
        let out = rebuild_debug_topic_stream(&mut cache, &MetadataParameters::default(), &payload);
        assert_eq!(out, Some(payload));
        assert!(
            cache.is_empty(),
            "an unframed payload must not prime the cache"
        );
    }

    #[test]
    fn schema_once_batch_is_prepended_with_cached_schema() {
        let schema = b"SCHEMA-BLOCK".to_vec();
        let hash = fnv1a(&schema);
        let batch = b"schema-less-batch".to_vec();
        let mut cache = vec![(hash, schema.clone())];
        let out = rebuild_debug_topic_stream(&mut cache, &params_with_schema_hash(hash), &batch);
        let mut expected = schema;
        expected.extend_from_slice(&batch);
        assert_eq!(out, Some(expected));
    }

    /// End-to-end guard for the `dora topic info` bandwidth fix, mirroring the
    /// daemon debug path (the topic subscriber in `Daemon::spawn_dataflow`):
    /// rebuild the inspection stream, strip the internal wire keys, then stamp
    /// the *input* payload length under `WIRE_SIZE`. The CLI reader
    /// ([`debug_frame_wire_size`]) must then charge the schema-less on-wire
    /// size, not the rebuilt stream — otherwise bandwidth is over-reported by
    /// the prepended schema on every schema-once frame, worst for the
    /// small/frequent primitives the schema-once optimization targets (#2584).
    /// Keep this in sync with the stamp site in `spawn_dataflow`.
    #[test]
    fn on_wire_size_excludes_prepended_schema() {
        // --- schema-once frame: rebuilt = schema ++ batch, only batch on-wire.
        let schema = b"SCHEMA-BLOCK".to_vec();
        let hash = fnv1a(&schema);
        let batch = b"schema-less-batch".to_vec();
        let mut cache = vec![(hash, schema.clone())];
        let mut params = params_with_schema_hash(hash);

        let rebuilt =
            rebuild_debug_topic_stream(&mut cache, &params, &batch).expect("schema cached");
        assert_eq!(rebuilt.len(), schema.len() + batch.len());

        // Replicate the daemon stamp: strip wire keys, record the input length.
        strip_internal_parameters(&mut params);
        assert!(
            get_integer_param(&params, SCHEMA_HASH).is_none(),
            "internal wire keys must be stripped before stamping"
        );
        params.insert(
            WIRE_SIZE.to_string(),
            Parameter::Integer(batch.len() as i64),
        );

        // The CLI reader charges the schema-less size, not the rebuilt stream.
        let charged = debug_frame_wire_size(&params, Some(&rebuilt));
        assert_eq!(charged, batch.len());
        assert!(
            charged < rebuilt.len(),
            "schema-once accounting must exclude the prepended schema"
        );

        // --- full self-describing frame: rebuilt == payload, charge full len.
        let full = b"self-describing-stream".to_vec();
        let mut full_params = MetadataParameters::default();
        let out = rebuild_debug_topic_stream(&mut Vec::new(), &full_params, &full)
            .expect("full stream forwarded");
        assert_eq!(out.len(), full.len());
        strip_internal_parameters(&mut full_params);
        full_params.insert(WIRE_SIZE.to_string(), Parameter::Integer(full.len() as i64));
        assert_eq!(debug_frame_wire_size(&full_params, Some(&out)), full.len());
    }

    #[test]
    fn schema_once_batch_dropped_when_schema_not_cached() {
        let batch = b"schema-less-batch".to_vec();
        let out = rebuild_debug_topic_stream(&mut Vec::new(), &params_with_schema_hash(42), &batch);
        assert_eq!(out, None);
    }

    #[test]
    fn schema_once_batch_dropped_on_hash_mismatch() {
        // Retained schema is for a different hash (e.g. a stale schema
        // mid-change): drop rather than emit a stream the CLI would decode
        // incorrectly.
        let schema = b"OTHER-SCHEMA".to_vec();
        let out = rebuild_debug_topic_stream(
            &mut vec![(999, schema)],
            &params_with_schema_hash(42),
            b"batch",
        );
        assert_eq!(out, None);
    }

    /// A full self-describing stream must prime the debug schema cache in-band
    /// (mirroring the node receive path), so `dora topic` can rebuild the
    /// schema-less batches that follow even when the `@schema` subscription
    /// missed the (single) schema emission — the producer guarantees a full
    /// stream for the first message of every output and periodically
    /// thereafter (dora-rs/dora#2366 review).
    #[test]
    fn full_stream_primes_debug_schema_cache_in_band() {
        let schema_block = framed(b"SCHEMA-FLATBUFFER");
        let hash = fnv1a(&schema_block);
        let batch = framed(b"BATCH-FLATBUFFER");
        let mut full = schema_block.clone();
        full.extend_from_slice(&batch);

        // The full stream is forwarded verbatim AND primes the cache.
        let mut cache = Vec::new();
        let out = rebuild_debug_topic_stream(&mut cache, &MetadataParameters::default(), &full);
        assert_eq!(out, Some(full.clone()));
        assert_eq!(cache, vec![(hash, schema_block)]);

        // A schema-less batch tagged with that hash now rebuilds, without any
        // `@schema` sample ever having been received.
        let out = rebuild_debug_topic_stream(&mut cache, &params_with_schema_hash(hash), &batch);
        assert_eq!(out, Some(full));
    }

    /// The cache retains multiple schemas: a full stream of a different schema
    /// (e.g. a large message interleaved with schema-once batches on the same
    /// output) must not clobber the schema those batches reference — the node
    /// receive path retains 8 schemas for exactly this interleave, and the
    /// `dora topic` mirror must not silently diverge from what nodes decode.
    #[test]
    fn interleaved_full_stream_does_not_clobber_schema_once_schema() {
        let schema_a = framed(b"SCHEMA-A");
        let hash_a = fnv1a(&schema_a);
        let batch_a = framed(b"BATCH-A");

        // Prime A (as the @schema subscriber or an earlier full stream would).
        let mut cache = vec![(hash_a, schema_a.clone())];

        // A large full stream of schema B passes through on the same output...
        let mut full_b = framed(b"SCHEMA-B");
        full_b.extend_from_slice(&framed(b"BATCH-B"));
        let out = rebuild_debug_topic_stream(&mut cache, &MetadataParameters::default(), &full_b);
        assert_eq!(out, Some(full_b));

        // ...and the next A-tagged schema-once batch still rebuilds.
        let out =
            rebuild_debug_topic_stream(&mut cache, &params_with_schema_hash(hash_a), &batch_a);
        let mut expected = schema_a;
        expected.extend_from_slice(&batch_a);
        assert_eq!(out, Some(expected));
    }

    /// Service/action full streams (pattern-correlated) are excluded from
    /// schema-once and must not churn the debug schema cache.
    #[test]
    fn pattern_correlated_stream_does_not_prime_debug_cache() {
        let schema_block = framed(b"SCHEMA-FLATBUFFER");
        let hash = fnv1a(&schema_block);

        let mut cache = vec![(hash, schema_block.clone())];
        let mut params = MetadataParameters::default();
        params.insert(
            dora_message::metadata::REQUEST_ID.to_string(),
            Parameter::String("req-1".into()),
        );
        let mut service_full = framed(b"OTHER-SCHEMA");
        service_full.extend_from_slice(&framed(b"OTHER-BATCH"));
        let out = rebuild_debug_topic_stream(&mut cache, &params, &service_full);
        assert_eq!(out, Some(service_full));
        assert_eq!(
            cache,
            vec![(hash, schema_block)],
            "a service reply must not churn the schema-once cache"
        );
    }
}

#[cfg(test)]
mod fault_tolerance_tests {
    use super::*;
    use crate::running_dataflow::{HandleReplacement, StopProcessPolicy};
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
            exit_when_nodes_finish: None,
            type_rules: vec![],
            env: None,
        };
        RunningDataflow::new(Uuid::nil(), DaemonId::new(None), descriptor)
    }

    // dora-rs/dora: a node added to an already-running dataflow (via
    // `AddNode`) may register a timer input on an interval no existing node
    // uses. `start()` is the only place timer tasks are spawned, so the
    // `AddNode` handler re-invokes it. This test locks in the property that
    // makes that safe: `start()` is idempotent for existing intervals yet
    // still spawns a task for a newly-added one.
    #[tokio::test]
    async fn start_spawns_task_for_interval_added_after_first_start() {
        let mut df = test_dataflow();
        let clock = Arc::new(HLC::default());
        let (events_tx, _events_rx) = mpsc::channel(8);

        let first = Duration::from_millis(100);
        df.timers
            .entry(first)
            .or_default()
            .insert((NodeId::from("a".to_string()), DataId::from("t".to_string())));
        df.start(&events_tx, &clock).await.unwrap();
        assert!(df._timer_handles.contains_key(&first));

        // Simulate a node added later that registers a brand-new interval.
        let second = Duration::from_millis(250);
        df.timers
            .entry(second)
            .or_default()
            .insert((NodeId::from("b".to_string()), DataId::from("t".to_string())));
        df.start(&events_tx, &clock).await.unwrap();

        assert!(df._timer_handles.contains_key(&first));
        assert!(df._timer_handles.contains_key(&second));
        assert_eq!(df._timer_handles.len(), 2);
    }

    // dora-rs/dora: removing the last subscriber of a timer interval (via
    // `RemoveNode`) must cancel that interval's timer task and forget the
    // entry, so it doesn't keep ticking to an empty subscriber set for the
    // rest of the dataflow's life (#2585). An interval that still has other
    // subscribers must be left running.
    #[tokio::test]
    async fn unsubscribe_last_subscriber_cancels_timer_task() {
        let mut df = test_dataflow();
        let clock = Arc::new(HLC::default());
        let (events_tx, _events_rx) = mpsc::channel(8);

        let node_a = NodeId::from("a".to_string());
        let node_b = NodeId::from("b".to_string());
        let solo = Duration::from_millis(100); // only `a` subscribes
        let shared = Duration::from_millis(250); // `a` and `b` subscribe

        df.timers
            .entry(solo)
            .or_default()
            .insert((node_a.clone(), DataId::from("t".to_string())));
        df.timers
            .entry(shared)
            .or_default()
            .insert((node_a.clone(), DataId::from("t".to_string())));
        df.timers
            .entry(shared)
            .or_default()
            .insert((node_b.clone(), DataId::from("t".to_string())));
        df.start(&events_tx, &clock).await.unwrap();
        assert!(df._timer_handles.contains_key(&solo));
        assert!(df._timer_handles.contains_key(&shared));

        df.unsubscribe_node_from_timers(&node_a);

        // `solo` lost its only subscriber: entry and task both gone.
        assert!(!df.timers.contains_key(&solo));
        assert!(!df._timer_handles.contains_key(&solo));
        // `shared` still has `b`: it keeps its subscriber set and its task.
        assert_eq!(
            df.timers.get(&shared).map(|s| s.len()),
            Some(1),
            "shared interval must keep its remaining subscriber"
        );
        assert!(df._timer_handles.contains_key(&shared));
    }

    // dora-rs/dora: the `AddNode` handler guards its re-invocation of
    // `start()` on `dataflow_started`, so that flag must be set on *every*
    // start path. It used to be set only at the single-daemon `Subscribe`
    // readiness call site, leaving it `false` for the whole life of a
    // distributed dataflow (which starts via the coordinator `AllNodesReady`
    // path instead) — so a node added later was silently starved of ticks.
    // `start()` now sets the flag itself; this pins that invariant.
    #[tokio::test]
    async fn start_marks_dataflow_started() {
        let mut df = test_dataflow();
        let clock = Arc::new(HLC::default());
        let (events_tx, _events_rx) = mpsc::channel(8);

        assert!(!df.dataflow_started);
        df.start(&events_tx, &clock).await.unwrap();
        assert!(df.dataflow_started);
    }

    fn test_running_node() -> RunningNode {
        RunningNode {
            process: None,
            restart_loop_start: None,
            _listener_shutdown: None,
            generation: 7,
            node_config: NodeConfig {
                dataflow_id: Uuid::nil(),
                node_id: NodeId::from("test".to_string()),
                run_config: NodeRunConfig {
                    inputs: BTreeMap::new(),
                    outputs: BTreeSet::new(),
                    output_types: BTreeMap::new(),
                    input_types: BTreeMap::new(),
                    output_framing: BTreeMap::new(),
                    shared_memory_pool_size: None,
                },
                daemon_communication: None,
                dataflow_descriptor: serde_yaml::Value::Null,
                dynamic: false,
                write_events_to: None,
                restart_count: 0,
                output_routing: None,
            },
            pid: None,
            restart_count: Arc::new(AtomicU32::new(0)),
            restart_policy: RestartPolicy::Never,
            disable_restart: Arc::new(AtomicBool::new(false)),
            force_restart_next: Arc::new(AtomicBool::new(false)),
            last_activity: Arc::new(AtomicU64::new(0)),
            health_check_timeout: None,
            finish_grace_secs: None,
        }
    }

    /// dora-rs/dora#2988 review, finding 1: the Event::Node gate must be
    /// STRICTLY-older, not exact-match. The restart loop publishes a
    /// successor's generation before spawning it, so the successor's
    /// connection can emit its one-shot Subscribe with a generation NEWER
    /// than the entry — dropping that leaves the incarnation disconnected
    /// forever.
    #[test]
    fn node_event_gate_drops_only_strictly_older_generations() {
        // Predecessor's connection after a swap advanced the entry: stale.
        assert!(event_generation_is_stale(8, 7));
        // The entry's current incarnation: current.
        assert!(!event_generation_is_stale(8, 8));
        // Successor racing its own ProcessHandleReplaced: must pass.
        assert!(
            !event_generation_is_stale(8, 9),
            "a successor's early events must not be dropped while the \
             entry still holds the predecessor's generation"
        );
    }

    #[test]
    fn running_node_rejects_stale_generation() {
        let mut node = test_running_node();
        let reused_pid = Arc::new(AtomicU32::new(42));
        node.pid = Some(reused_pid);

        assert!(node.matches_generation(7));
        assert!(
            !node.matches_generation(6),
            "a reused PID must not make an older generation current"
        );
    }

    // The registration gate itself is tested through `restart_loop` in
    // `spawn::prepared::tests` (`restart_loop_aborts_when_registration_never_happens`
    // and `cancelled_restart_settles_terminal_exit`), which drive the real
    // loop rather than restating oneshot-channel semantics here.

    #[test]
    fn planned_stop_markers_are_scoped_to_generation() {
        let dataflow = test_dataflow();
        let node_id: NodeId = "readded".to_string().into();

        dataflow.grace_duration_kills.insert((node_id.clone(), 7));

        assert!(
            dataflow
                .grace_duration_kills
                .contains(&(node_id.clone(), 7))
        );
        assert!(
            !dataflow
                .grace_duration_kills
                .contains(&(node_id.clone(), 8)),
            "a successor must not inherit its predecessor's planned-stop marker"
        );
        dataflow.grace_duration_kills.remove(&(node_id.clone(), 6));
        assert!(
            dataflow
                .grace_duration_kills
                .contains(&(node_id.clone(), 7)),
            "cleaning a stale event must not clear another generation's marker"
        );
    }

    #[test]
    fn successful_readd_clears_only_the_previous_node_result() {
        let dataflow_id = Uuid::new_v4();
        let other_dataflow_id = Uuid::new_v4();
        let single_result_dataflow_id = Uuid::new_v4();
        let node_id: NodeId = "readded".to_string().into();
        let other_node_id: NodeId = "other".to_string().into();
        let mut results = BTreeMap::from([
            (
                dataflow_id,
                BTreeMap::from([(node_id.clone(), Ok(())), (other_node_id.clone(), Ok(()))]),
            ),
            (
                other_dataflow_id,
                BTreeMap::from([(node_id.clone(), Ok(()))]),
            ),
            (
                single_result_dataflow_id,
                BTreeMap::from([(node_id.clone(), Ok(()))]),
            ),
        ]);

        clear_node_result(&mut results, dataflow_id, &node_id);

        assert!(!results[&dataflow_id].contains_key(&node_id));
        assert!(results[&dataflow_id].contains_key(&other_node_id));
        assert!(results[&other_dataflow_id].contains_key(&node_id));

        clear_node_result(&mut results, single_result_dataflow_id, &node_id);
        assert!(!results.contains_key(&single_result_dataflow_id));
    }

    #[test]
    fn stale_handle_replacement_preserves_live_process() {
        let mut node = test_running_node();
        let (live_tx, live_rx) = flume::bounded(2);
        node.process = Some(ProcessHandle::new(live_tx));

        let (stale_tx, stale_rx) = flume::bounded(2);
        let outcome = node.replace_process_handle(6, 8, ProcessHandle::new(stale_tx));

        assert!(
            matches!(outcome, HandleReplacement::RejectedStale),
            "a dead incarnation must not replace a live handle"
        );
        assert!(
            node.matches_generation(7),
            "a stale rejection must leave the live generation untouched"
        );
        assert!(
            live_rx.try_recv().is_err(),
            "rejecting a stale replacement must not kill the live successor"
        );
        assert!(
            matches!(stale_rx.try_recv(), Ok(ProcessOperation::Kill)),
            "the orphan process owned by the stale event should be killed"
        );

        // Avoid sending a drop-time Kill into `live_rx` after the assertions.
        let _ = node.process.take();
    }

    #[test]
    fn teardown_rejects_matching_handle_replacement() {
        let mut node = test_running_node();
        let (live_tx, live_rx) = flume::bounded(2);
        node.process = Some(ProcessHandle::new(live_tx));
        node.disable_restart();

        let (replacement_tx, replacement_rx) = flume::bounded(2);
        let outcome = node.replace_process_handle(7, 8, ProcessHandle::new(replacement_tx));

        let HandleReplacement::RejectedTeardown(replacement) = outcome else {
            panic!("teardown must win a race with process replacement");
        };
        assert!(
            replacement_rx.try_recv().is_err(),
            "the caller must retain the replacement for planned-stop routing"
        );
        drop(replacement);
        assert!(
            matches!(replacement_rx.try_recv(), Ok(ProcessOperation::Kill)),
            "teardown must win a race with process replacement"
        );
        // The generation must still advance: the restart loop now speaks
        // generation 8, and its terminal SpawnedNodeResult has to match this
        // entry or the node would stay registered forever and the dataflow
        // could never finish.
        assert!(node.matches_generation(8));
        assert!(!node.matches_generation(7));
        assert!(
            node.process.is_some(),
            "teardown rejection must not install the replacement handle"
        );
        assert!(live_rx.try_recv().is_err());
        let _ = node.process.take();
    }

    #[tokio::test(start_paused = true)]
    async fn teardown_replacement_uses_configured_grace_period() {
        let mut dataflow = test_dataflow();
        dataflow.stop_process_policy = Some(StopProcessPolicy::Graceful(Duration::from_secs(4)));
        let mut node = test_running_node();
        node.disable_restart();

        let (replacement_tx, replacement_rx) = flume::bounded(4);
        let outcome = node.replace_process_handle(7, 8, ProcessHandle::new(replacement_tx));
        let HandleReplacement::RejectedTeardown(replacement) = outcome else {
            panic!("teardown must retain ownership of the replacement handle");
        };

        let node_id: NodeId = "test".to_string().into();
        dataflow.stop_rejected_replacement(&node_id, 8, replacement);
        tokio::task::yield_now().await;

        assert!(
            replacement_rx.try_recv().is_err(),
            "a racing replacement must not be killed immediately"
        );
        tokio::time::advance(Duration::from_secs(3)).await;
        assert!(replacement_rx.try_recv().is_err());

        tokio::time::advance(Duration::from_secs(1)).await;
        tokio::task::yield_now().await;
        assert!(matches!(
            replacement_rx.try_recv(),
            Ok(ProcessOperation::SoftKill)
        ));
        assert!(
            dataflow
                .grace_duration_kills
                .contains(&(node_id.clone(), 8)),
            "the replacement stop must be classified as daemon-initiated"
        );

        tokio::time::advance(Duration::from_secs(2)).await;
        tokio::task::yield_now().await;
        assert!(matches!(
            replacement_rx.try_recv(),
            Ok(ProcessOperation::Kill)
        ));
    }

    #[test]
    fn current_handle_replacement_advances_generation() {
        let mut node = test_running_node();
        let (old_tx, old_rx) = flume::bounded(2);
        node.process = Some(ProcessHandle::new(old_tx));

        let (new_tx, new_rx) = flume::bounded(2);
        let outcome = node.replace_process_handle(7, 8, ProcessHandle::new(new_tx));

        assert!(matches!(outcome, HandleReplacement::Replaced));
        assert!(node.matches_generation(8));
        assert!(
            matches!(old_rx.try_recv(), Ok(ProcessOperation::Kill)),
            "replacing the current incarnation should retire its old handle"
        );
        assert!(
            new_rx.try_recv().is_err(),
            "the replacement process must remain live"
        );

        let _ = node.process.take();
    }

    fn test_clock() -> HLC {
        HLC::default()
    }

    #[test]
    fn zenoh_publish_enqueue_reports_queued() {
        let (tx, mut rx) = mpsc::channel(1);

        let result = try_enqueue_zenoh_outbound(&tx, "output");

        assert_eq!(result, ZenohEnqueueResult::Queued);
        assert_eq!(rx.try_recv().unwrap(), "output");
    }

    #[test]
    fn zenoh_publish_enqueue_reports_full_without_waiting() {
        let (tx, _rx) = mpsc::channel(1);
        tx.try_send("occupied").unwrap();

        let result = try_enqueue_zenoh_outbound(&tx, "output");

        assert_eq!(
            result,
            ZenohEnqueueResult::Full,
            "enqueue backpressure must be reported synchronously so the daemon event loop does not sleep"
        );
    }

    #[test]
    fn zenoh_publish_enqueue_reports_closed() {
        let (tx, rx) = mpsc::channel(1);
        drop(rx);

        let result = try_enqueue_zenoh_outbound(&tx, "output");

        assert_eq!(result, ZenohEnqueueResult::Closed);
    }

    #[test]
    fn regular_output_enqueue_full_is_nonfatal() {
        let (tx, _rx) = mpsc::channel(1);
        tx.try_send("occupied").unwrap();

        let result = try_enqueue_zenoh_outbound(&tx, "output");

        assert!(
            matches!(result, ZenohEnqueueResult::Full),
            "regular output enqueue should report backpressure without surfacing a fatal error"
        );
    }

    #[test]
    fn data_queue_full_does_not_block_control_enqueue() {
        let (data_tx, _data_rx) = mpsc::channel(1);
        data_tx.try_send("data-occupied").unwrap();
        let (control_tx, mut control_rx) = mpsc::channel(1);

        let data_result = try_enqueue_zenoh_outbound(&data_tx, "regular-output");
        let control_result = try_enqueue_zenoh_outbound(&control_tx, "output-closed");

        assert_eq!(data_result, ZenohEnqueueResult::Full);
        assert_eq!(control_result, ZenohEnqueueResult::Queued);
        assert_eq!(control_rx.try_recv().unwrap(), "output-closed");
    }

    #[test]
    fn control_pending_enqueue_full_is_recorded_without_error() {
        let dataflow = test_dataflow();

        let result = handle_control_enqueue_result(&dataflow, ZenohEnqueueResult::Full);

        assert!(
            result.is_ok(),
            "a full control pending queue for OutputClosed must not bubble up as a coordinator disconnect"
        );
        assert_eq!(
            dataflow
                .net_publish_failures
                .load(atomic::Ordering::Relaxed),
            1,
            "dropping a control event must be visible in network metrics"
        );
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

    // ---- cross-process memory-pool cleanup (#2935) ----

    /// Subscribe `node` to `df` with a channel of `capacity` slots and a
    /// pending-message counter, mirroring what `DaemonNodeEvent::Subscribe`
    /// installs for a live node.
    fn subscribe_test_node(
        df: &mut RunningDataflow,
        node: &NodeId,
        capacity: usize,
    ) -> (
        mpsc::Receiver<Timestamped<NodeEvent>>,
        Arc<std::sync::atomic::AtomicU64>,
    ) {
        let (tx, rx) = mpsc::channel(capacity);
        df.subscribe_channels.insert(node.clone(), tx);
        let counter = Arc::new(AtomicU64::new(0));
        df.pending_messages.insert(node.clone(), counter.clone());
        (rx, counter)
    }

    fn test_pool_id(name: &str) -> MemoryPoolId {
        MemoryPoolId {
            dataflow_id: Uuid::nil().to_string(),
            id: name.to_string(),
        }
    }

    /// Metadata without a backing segment, so `free_memory_pool` skips the
    /// Linux-only `/dev/shm` unlink and the tests stay cross-platform.
    fn test_pool_metadata() -> MemoryPoolMetadata {
        MemoryPoolMetadata {
            size: 1024,
            dtype: "float32".into(),
            shape: vec![256],
            ..Default::default()
        }
    }

    fn freed_pool_ids(events: &[NodeEvent]) -> Vec<&str> {
        events
            .iter()
            .filter_map(|event| match event {
                NodeEvent::FreeMemoryPool { shared_memory_id } => Some(shared_memory_id.as_str()),
                _ => None,
            })
            .collect()
    }

    /// The cleanup broadcast must reach every node that registered or read
    /// the pool, and nobody else: not the node that initiated the free (it
    /// released synchronously) and not a subscriber that never touched the
    /// pool.
    ///
    /// `free_pool_and_notify` is the single entry point behind both
    /// `FreePinnedMemory` and `ReadPinnedMemory { free: true }` — before
    /// #2935 the read-with-free path freed the pool without notifying
    /// anyone, so every other node kept its GPU/host/IPC/shmem resources.
    #[test]
    fn free_pool_notifies_touched_nodes_only() {
        let clock = test_clock();
        let mut df = test_dataflow();
        let pools = MemoryPoolManager::new();

        let registrar: NodeId = "registrar".to_string().into();
        let reader: NodeId = "reader".to_string().into();
        let bystander: NodeId = "bystander".to_string().into();
        let (mut registrar_rx, registrar_pending) =
            subscribe_test_node(&mut df, &registrar, NODE_EVENT_CHANNEL_CAPACITY);
        let (mut reader_rx, _) = subscribe_test_node(&mut df, &reader, NODE_EVENT_CHANNEL_CAPACITY);
        let (mut bystander_rx, _) =
            subscribe_test_node(&mut df, &bystander, NODE_EVENT_CHANNEL_CAPACITY);

        let id = test_pool_id("pool-1");
        pools
            .register_memory_pool(id.clone(), test_pool_metadata(), registrar.to_string())
            .unwrap();
        pools
            .read_memory_pool(&id, reader.as_ref())
            .expect("pool should exist");

        // The reader frees the pool it just read — the normal lifecycle.
        free_pool_and_notify(&pools, Some(&df), &id, reader.as_ref(), &clock)
            .expect("free should succeed");

        assert_eq!(
            freed_pool_ids(&drain_events(&mut registrar_rx)),
            vec!["pool-1"],
            "the registering node must be told to release its pool resources"
        );
        assert!(
            drain_events(&mut reader_rx).is_empty(),
            "the initiator released synchronously and must not be notified"
        );
        assert!(
            drain_events(&mut bystander_rx).is_empty(),
            "a node that never touched the pool must not be notified"
        );

        // Every delivery site pairs an enqueue with `inc_pending`; the
        // listener decrements unconditionally, so a missing increment
        // underflows the reported count to `u64::MAX` (#2827).
        assert_eq!(registrar_pending.load(atomic::Ordering::Relaxed), 1);
    }

    /// `send_with_timestamp` reports a full channel as `Ok(false)`, not
    /// `Err`. The original broadcast matched only on `Err`, so a full
    /// channel silently skipped the cleanup (#2935). The undelivered node
    /// must be reported so the leak is visible.
    #[test]
    fn notify_pool_freed_reports_nodes_with_full_channels() {
        let clock = test_clock();
        let mut df = test_dataflow();

        let registrar: NodeId = "registrar".to_string().into();
        let backed_up: NodeId = "backed-up".to_string().into();
        let (mut registrar_rx, registrar_pending) =
            subscribe_test_node(&mut df, &registrar, NODE_EVENT_CHANNEL_CAPACITY);
        // A one-slot channel, already full: the next send is dropped.
        let (_backed_up_rx, backed_up_pending) = subscribe_test_node(&mut df, &backed_up, 1);
        assert!(
            send_with_timestamp(&df.subscribe_channels[&backed_up], NodeEvent::Stop, &clock)
                .expect("channel is open")
        );

        let touched = HashSet::from([
            registrar.to_string(),
            backed_up.to_string(),
            "initiator".to_string(),
        ]);
        let undelivered = notify_memory_pool_freed(&df, "pool-1", &touched, "initiator", &clock);

        assert_eq!(
            undelivered,
            BTreeSet::from([backed_up.clone()]),
            "a node whose channel is full must be reported as undelivered"
        );
        assert_eq!(
            backed_up_pending.load(atomic::Ordering::Relaxed),
            0,
            "a dropped event must not increment the pending counter"
        );
        assert_eq!(
            freed_pool_ids(&drain_events(&mut registrar_rx)),
            vec!["pool-1"],
            "one full channel must not stop the broadcast to the other nodes"
        );
        assert_eq!(registrar_pending.load(atomic::Ordering::Relaxed), 1);
    }

    /// A closed channel means the node is gone and the OS reclaimed its
    /// resources — benign, and it must not abort the rest of the broadcast
    /// or be reported as a leak.
    #[test]
    fn notify_pool_freed_skips_disconnected_nodes() {
        let clock = test_clock();
        let mut df = test_dataflow();

        let registrar: NodeId = "registrar".to_string().into();
        let gone: NodeId = "gone".to_string().into();
        let (mut registrar_rx, _) =
            subscribe_test_node(&mut df, &registrar, NODE_EVENT_CHANNEL_CAPACITY);
        let (gone_rx, _) = subscribe_test_node(&mut df, &gone, NODE_EVENT_CHANNEL_CAPACITY);
        drop(gone_rx);

        let touched = HashSet::from([registrar.to_string(), gone.to_string()]);
        let undelivered = notify_memory_pool_freed(&df, "pool-1", &touched, "third-party", &clock);

        assert!(
            undelivered.is_empty(),
            "a closed channel is benign, not an undelivered notification"
        );
        assert_eq!(
            freed_pool_ids(&drain_events(&mut registrar_rx)),
            vec!["pool-1"],
            "a disconnected peer must not abort the rest of the broadcast"
        );
    }

    /// Freeing an unknown pool must surface the manager's error to the
    /// requesting node and broadcast nothing.
    #[test]
    fn free_pool_propagates_unknown_pool_error_without_notifying() {
        let clock = test_clock();
        let mut df = test_dataflow();
        let pools = MemoryPoolManager::new();

        let registrar: NodeId = "registrar".to_string().into();
        let (mut registrar_rx, _) =
            subscribe_test_node(&mut df, &registrar, NODE_EVENT_CHANNEL_CAPACITY);

        let err = free_pool_and_notify(
            &pools,
            Some(&df),
            &test_pool_id("never-registered"),
            registrar.as_ref(),
            &clock,
        )
        .expect_err("freeing an unknown pool must fail");
        assert!(
            err.contains("memory pool not found"),
            "unexpected error: {err}"
        );
        assert!(drain_events(&mut registrar_rx).is_empty());
    }

    /// A dataflow that is no longer running has no subscribe channels; the
    /// pool must still be released rather than the free failing.
    #[test]
    fn free_pool_without_running_dataflow_still_releases_the_entry() {
        let clock = test_clock();
        let pools = MemoryPoolManager::new();
        let id = test_pool_id("pool-1");
        pools
            .register_memory_pool(id.clone(), test_pool_metadata(), "registrar".into())
            .unwrap();

        free_pool_and_notify(&pools, None, &id, "registrar", &clock).expect("free should succeed");
        assert_eq!(pools.table_size(), 0);
    }

    // -- Test 1: close_input removes input, sends InputClosed, no AllInputsClosed with remaining inputs --

    /// Regression test for #241. The daemon used to carry two inverse maps
    /// (`debug_topic_subscriptions: Uuid → Set<OutputId>` and
    /// `debug_topic_watchers: OutputId → Set<Uuid>`). The inverse map was
    /// dropped — unsubscribe now scans `debug_topic_watchers` with `retain`.
    /// This test locks in the invariants the inverse map used to provide:
    /// after stop, the subscription_id must be gone from every watcher set,
    /// and outputs with no remaining watchers must be removed entirely.
    #[test]
    fn stop_debug_stream_scan_cleans_up_watchers() {
        let mut df = test_dataflow();
        let node_a: NodeId = "node_a".to_string().into();
        let out_1: DataId = "out_1".to_string().into();
        let out_2: DataId = "out_2".to_string().into();
        let sub_a = uuid::Uuid::new_v4();
        let sub_b = uuid::Uuid::new_v4();

        // Two subs, both watching out_1; only sub_a watches out_2.
        df.debug_topic_watchers
            .entry(OutputId(node_a.clone(), out_1.clone()))
            .or_default()
            .extend([sub_a, sub_b]);
        df.debug_topic_watchers
            .entry(OutputId(node_a.clone(), out_2.clone()))
            .or_default()
            .insert(sub_a);

        // Mirror the production stop path exactly.
        df.debug_topic_watchers.retain(|_output_id, watchers| {
            watchers.remove(&sub_a);
            !watchers.is_empty()
        });

        // out_1 still has sub_b → entry retained with just sub_b.
        let out_1_watchers = df
            .debug_topic_watchers
            .get(&OutputId(node_a.clone(), out_1))
            .expect("out_1 entry must remain because sub_b still watches it");
        assert_eq!(out_1_watchers.len(), 1);
        assert!(out_1_watchers.contains(&sub_b));

        // out_2 had only sub_a → entry removed entirely after scan.
        assert!(
            !df.debug_topic_watchers
                .contains_key(&OutputId(node_a, out_2)),
            "out_2 must be dropped because its only watcher was sub_a"
        );
    }

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

    // -- dora#2270: finish-straggler watchdog must spare timer/log-fed nodes --

    /// Insert a connected node that has been silent since the epoch, so
    /// `finish_stragglers` sees it as long-idle regardless of grace.
    fn insert_silent_node(df: &mut RunningDataflow, node: &NodeId) {
        let running = test_running_node();
        running.last_activity.store(1, atomic::Ordering::Release);
        df.running_nodes.insert(node.clone(), running);
        // a real running node has subscribed (can receive finish events)
        let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node.clone(), tx);
        df.connected_nodes.insert(node.clone());
    }

    #[test]
    fn timer_fed_node_is_not_a_finish_straggler() {
        // A long-running timer-only node never drains and sends no daemon
        // traffic, so it looks "silent + never drained" exactly like a wedge —
        // but it is alive by design and must NOT be force-killed (#2270).
        let mut df = test_dataflow();
        let timer_node: NodeId = "timer_node".to_string().into();
        insert_silent_node(&mut df, &timer_node);
        df.timers
            .entry(Duration::from_millis(100))
            .or_default()
            .insert((timer_node.clone(), "tick".to_string().into()));

        let now = node_communication::current_millis();
        let selected = df.finish_stragglers(Duration::from_millis(1), now);
        assert!(
            selected.is_empty(),
            "timer-fed node must not be escalated: {selected:?}"
        );
    }

    #[test]
    fn wedged_user_input_node_is_a_finish_straggler() {
        // A connected node with no timer/log input that has gone silent past
        // grace while the rest of the dataflow finished IS a straggler.
        let mut df = test_dataflow();
        let stuck: NodeId = "stuck".to_string().into();
        insert_silent_node(&mut df, &stuck);

        let now = node_communication::current_millis();
        let selected = df.finish_stragglers(Duration::from_millis(1), now);
        assert_eq!(selected, vec![stuck]);
    }

    #[test]
    fn unconnected_slow_starting_node_is_not_a_finish_straggler() {
        // `last_activity` is seeded at spawn, so a node that has not subscribed
        // yet looks long-silent — but it is still starting up (e.g. loading a
        // model) and must not be force-killed (#2270 review).
        let mut df = test_dataflow();
        let starting: NodeId = "slow_loader".to_string().into();
        let running = test_running_node();
        running.last_activity.store(1, atomic::Ordering::Release);
        df.running_nodes.insert(starting.clone(), running);
        // NOTE: deliberately NOT added to subscribe_channels (not connected).

        let now = node_communication::current_millis();
        let selected = df.finish_stragglers(Duration::from_millis(1), now);
        assert!(
            selected.is_empty(),
            "a node that has not subscribed must not be escalated: {selected:?}"
        );
    }

    #[test]
    fn dropped_stream_node_is_still_a_finish_straggler() {
        // A node that connected, then dropped its event stream (channel removed)
        // but kept its process alive, is still a wedge candidate — `connected`
        // tracks `connected_nodes`, not current channel presence (#2270 review).
        let mut df = test_dataflow();
        let stuck: NodeId = "stuck".to_string().into();
        let running = test_running_node();
        running.last_activity.store(1, atomic::Ordering::Release);
        df.running_nodes.insert(stuck.clone(), running);
        df.connected_nodes.insert(stuck.clone());
        // NOTE: no subscribe_channels entry — the event stream was dropped.

        let now = node_communication::current_millis();
        let selected = df.finish_stragglers(Duration::from_millis(1), now);
        assert_eq!(selected, vec![stuck]);
    }

    #[test]
    fn removed_node_id_is_not_connected_on_reuse() {
        // RemoveNode clears connected_nodes, so a re-added node ID starts fresh:
        // its slow-starting new incarnation must not be selected before it
        // subscribes, even though the previous incarnation had connected.
        let mut df = test_dataflow();
        let node_a: NodeId = "node_a".to_string().into();
        df.connected_nodes.insert(node_a.clone());
        df.connected_nodes.remove(&node_a); // (the RemoveNode cleanup line)

        let running = test_running_node();
        running.last_activity.store(1, atomic::Ordering::Release);
        df.running_nodes.insert(node_a.clone(), running);

        let now = node_communication::current_millis();
        let selected = df.finish_stragglers(Duration::from_millis(1), now);
        assert!(
            selected.is_empty(),
            "a re-added node ID must not be selected before its new incarnation subscribes"
        );
    }

    #[test]
    fn cleared_timer_state_makes_reused_id_escalatable() {
        // A timer-fed node is never-finishing (vetoed). RemoveNode clears its
        // timer subscription, so a re-added user-input node under the same ID is
        // classified by its own inputs and can be escalated (#2270 review).
        let mut df = test_dataflow();
        let node: NodeId = "reused".to_string().into();
        insert_silent_node(&mut df, &node);
        df.timers
            .entry(Duration::from_millis(100))
            .or_default()
            .insert((node.clone(), "tick".to_string().into()));

        let now = node_communication::current_millis();
        // as a timer node → vetoed
        assert!(
            df.finish_stragglers(Duration::from_millis(1), now)
                .is_empty(),
            "timer-fed node must be vetoed"
        );

        // RemoveNode clears the timer subscription
        for receivers in df.timers.values_mut() {
            receivers.retain(|(nid, _)| nid != &node);
        }
        assert_eq!(
            df.finish_stragglers(Duration::from_millis(1), now),
            vec![node],
            "once its timer state is cleared the node is classified by its own inputs"
        );
    }

    #[test]
    fn reopened_input_clears_stale_drain_timestamp() {
        // A node drained long ago is eligible via the drained arm. Reopening a
        // mapping must clear that timestamp so the node — now actively receiving
        // again, not silent — is not force-stopped on a stale drain (#2270 review).
        let mut df = test_dataflow();
        let node: NodeId = "node_a".to_string().into();
        let running = test_running_node();
        // recent activity → not silent (only a stale drain clock could select it)
        running.last_activity.store(
            node_communication::current_millis(),
            atomic::Ordering::Release,
        );
        df.running_nodes.insert(node.clone(), running);
        df.connected_nodes.insert(node.clone());
        df.all_inputs_closed_at.insert(
            node.clone(),
            std::time::Instant::now() - Duration::from_secs(1),
        );

        let grace = Duration::from_millis(500);
        let now = node_communication::current_millis();
        // drained past grace → selected
        assert_eq!(df.finish_stragglers(grace, now), vec![node.clone()]);

        // AddMapping reopen clears the drain clock
        df.all_inputs_closed_at.remove(&node);
        assert!(
            df.finish_stragglers(grace, now).is_empty(),
            "an active node with a reopened input must not be selected on a stale drain"
        );
    }

    #[tokio::test]
    async fn restart_clears_connected_marker() {
        // A restarting node keeps its ID but is a new incarnation: clear the
        // connected marker so the restarting process isn't silence-escalatable
        // before it re-subscribes (a slow restart / model reload) — #2270 review.
        use crate::running_dataflow::{ProcessHandle, ProcessOperation};
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();

        let mut running = test_running_node();
        let (op_tx, _op_rx) = flume::unbounded::<ProcessOperation>();
        running.process = Some(ProcessHandle::new(op_tx));
        df.running_nodes.insert(node_a.clone(), running);
        df.connected_nodes.insert(node_a.clone());
        df.all_inputs_closed_at
            .insert(node_a.clone(), std::time::Instant::now());

        df.restart_single_node(&node_a, &clock, None).unwrap();

        assert!(
            !df.connected_nodes.contains(&node_a),
            "restart must clear the connected marker so the new incarnation re-subscribes"
        );
    }

    #[test]
    fn finish_drain_grace_defaults_on_with_opt_out() {
        // unset → enabled at the default grace (on by default, dora#2270 step 3)
        assert_eq!(
            parse_finish_drain_grace(None),
            Some(DEFAULT_FINISH_DRAIN_GRACE)
        );
        // explicit opt-out escape hatch → disabled
        assert_eq!(parse_finish_drain_grace(Some("off")), None);
        assert_eq!(parse_finish_drain_grace(Some("disabled")), None);
        assert_eq!(parse_finish_drain_grace(Some("OFF")), None);
        // set → enabled at the given grace
        assert_eq!(
            parse_finish_drain_grace(Some("30")),
            Some(Duration::from_secs(30))
        );
        assert_eq!(
            parse_finish_drain_grace(Some("0")),
            Some(Duration::from_secs(0))
        );
        // set-but-garbage → enabled at the default (the user meant to turn it on)
        assert_eq!(
            parse_finish_drain_grace(Some("not-a-number")),
            Some(DEFAULT_FINISH_DRAIN_GRACE)
        );
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

    /// dora-rs/dora#2920: under `--exit-when-nodes-finish` a node drains
    /// once its DATA inputs close, even with a timer still open — and it
    /// must have restart disabled at the same moment. Otherwise it exits,
    /// gets restarted, is told to finish again, and loops until
    /// `max_restarts`.
    #[test]
    fn opt_in_drains_node_with_open_timer_and_disables_its_restart() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let data_in: DataId = "value".to_string().into();
        let timer_in: DataId = "tick".to_string().into();

        df.timers_gate_drain = false;
        for input in [&data_in, &timer_in] {
            df.open_inputs
                .entry(node_a.clone())
                .or_default()
                .insert(input.clone());
        }
        // Only `value` is a real data dependency; `tick` is daemon-fed.
        df.data_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(data_in.clone());
        df.timers
            .entry(Duration::from_millis(100))
            .or_default()
            .insert((node_a.clone(), timer_in.clone()));

        let running = test_running_node();
        let disable_restart = running.disable_restart.clone();
        df.running_nodes.insert(node_a.clone(), running);
        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        close_input(&mut df, &node_a, &data_in, &clock);

        assert!(
            df.open_inputs(&node_a).contains(&timer_in),
            "the timer is still open — that is the whole point of the opt-in"
        );
        let events = drain_events(&mut rx);
        assert!(
            events.iter().any(|e| matches_event(e, "AllInputsClosed")),
            "node should be told to finish once its data inputs closed, got {events:?}"
        );
        assert!(
            disable_restart.load(atomic::Ordering::Acquire),
            "a node told to finish must not be restarted into a finish/exit loop"
        );
    }

    /// The opt-in must not stop disabling restart for SOURCE nodes.
    /// `is_drained` reports false for them by design, so keying the
    /// restart decision on it alone would let a source with
    /// `restart_policy: always` respawn forever.
    #[test]
    fn opt_in_still_disables_restart_for_source_nodes() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();

        df.timers_gate_drain = false;
        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_x.clone());
        // No `data_inputs` entry: as far as the drain rule is concerned
        // this node has nothing that can finish, i.e. it is a source.
        assert!(!df.is_drained(&node_a));

        let running = test_running_node();
        let disable_restart = running.disable_restart.clone();
        df.running_nodes.insert(node_a.clone(), running);
        let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_a.clone(), tx);

        close_input(&mut df, &node_a, &input_x, &clock);

        assert!(
            disable_restart.load(atomic::Ordering::Acquire),
            "source nodes must still have restart disabled once nothing is \
             open, independent of the drain opt-in"
        );
    }

    #[test]
    fn forget_node_bookkeeping_purges_only_that_node() {
        // Regression: RemoveNode must drop the removed node's per-node
        // bookkeeping, otherwise stale input_deadlines/broken_inputs entries
        // are re-scanned every tick forever and the stderr queue leaks across
        // repeated dynamic add/remove cycles.
        let mut df = test_dataflow();
        let node_a: NodeId = "node_a".to_string().into();
        let node_b: NodeId = "node_b".to_string().into();
        let input_x: DataId = "input_x".to_string().into();
        let timeout = Duration::from_secs(1);

        for node in [&node_a, &node_b] {
            df.input_deadlines.insert(
                (node.clone(), input_x.clone()),
                InputDeadline {
                    timeout,
                    last_received: None,
                },
            );
            df.broken_inputs
                .insert((node.clone(), input_x.clone()), timeout);
            df.node_stderr_most_recent
                .insert(node.clone(), Arc::new(ArrayQueue::new(4)));
        }

        df.forget_node_bookkeeping(&node_a);

        // node_a's entries are gone …
        assert!(
            !df.input_deadlines
                .contains_key(&(node_a.clone(), input_x.clone()))
        );
        assert!(
            !df.broken_inputs
                .contains_key(&(node_a.clone(), input_x.clone()))
        );
        assert!(!df.node_stderr_most_recent.contains_key(&node_a));

        // … while node_b's are untouched.
        assert!(
            df.input_deadlines
                .contains_key(&(node_b.clone(), input_x.clone()))
        );
        assert!(
            df.broken_inputs
                .contains_key(&(node_b.clone(), input_x.clone()))
        );
        assert!(df.node_stderr_most_recent.contains_key(&node_b));
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

    // -- #2968: a closed input's armed deadline must not orphan a broken record --

    #[test]
    fn close_input_drops_armed_deadline() {
        // Regression (#2968): `close_input` must drop the input's
        // `input_deadlines` entry. Otherwise the stale, still-armed deadline
        // keeps counting after the input has closed; `check_input_timeouts`
        // then fires on the already-closed input, inserting a `broken_inputs`
        // record that `break_input` can never clear.
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_a: NodeId = "node_a".to_string().into();
        let input_x: DataId = "input_x".to_string().into();

        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input_x.clone());
        // Armed (a message was received), so it would time out if left behind.
        df.input_deadlines.insert(
            (node_a.clone(), input_x.clone()),
            InputDeadline {
                timeout: Duration::from_millis(1),
                last_received: Some(Instant::now() - Duration::from_secs(10)),
            },
        );

        close_input(&mut df, &node_a, &input_x, &clock);

        assert!(
            !df.input_deadlines
                .contains_key(&(node_a.clone(), input_x.clone())),
            "close_input must drop the input's deadline so it can't time out later"
        );
    }

    #[test]
    fn drained_node_finishes_despite_stale_deadline_timeout() {
        // End-to-end of #2968: a node with two inputs, one carrying an
        // `input_timeout`. The timed input's producer exits first, then the
        // stale deadline "fires" (as `check_input_timeouts` would), then the
        // second producer exits. The node must still be told `AllInputsClosed`
        // — not left with an orphaned `broken_inputs` record that pins
        // `has_broken_input` true forever and gets it SIGKILLed as a straggler.
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_c: NodeId = "node_c".to_string().into();
        let input_a: DataId = "input_a".to_string().into();
        let input_b: DataId = "input_b".to_string().into();

        df.open_inputs
            .entry(node_c.clone())
            .or_default()
            .extend([input_a.clone(), input_b.clone()]);
        // input_a has an armed deadline (producer sent messages before exiting).
        df.input_deadlines.insert(
            (node_c.clone(), input_a.clone()),
            InputDeadline {
                timeout: Duration::from_millis(1),
                last_received: Some(Instant::now() - Duration::from_secs(10)),
            },
        );

        let running = test_running_node();
        df.running_nodes.insert(node_c.clone(), running);
        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_c.clone(), tx);

        // 1. Producer of input_a exits.
        close_input(&mut df, &node_c, &input_a, &clock);

        // 2. Simulate the stale-deadline timeout firing as `check_input_timeouts`
        //    would: it inserts a broken record then calls break_input. With the
        //    fix, close_input already dropped the deadline, so this loop finds
        //    nothing; we force the worst case anyway to prove break_input does
        //    not leave an orphan even if it is reached on a closed input.
        df.broken_inputs
            .insert((node_c.clone(), input_a.clone()), Duration::from_millis(1));
        break_input(&mut df, &node_c, &input_a, &clock);
        assert!(
            !df.has_broken_input(&node_c),
            "break_input on an already-closed input must not orphan a broken record"
        );

        // 3. Producer of input_b exits — node is now fully drained.
        close_input(&mut df, &node_c, &input_b, &clock);

        let events = drain_events(&mut rx);
        assert!(
            events.iter().any(|e| matches_event(e, "AllInputsClosed")),
            "a cleanly-drained node must be told AllInputsClosed, got {events:?}"
        );
    }

    // -- Circuit-breaker recovery must be gated on receiver backpressure (#2627) --

    /// Helper: wire `sender/output -> receiver/input` and mark the input broken.
    /// `channel_capacity` controls whether the receiver counts as "keeping up"
    /// (>= CONTROL_EVENT_HEADROOM) or "saturated" (< CONTROL_EVENT_HEADROOM).
    fn broken_input_dataflow(
        channel_capacity: usize,
    ) -> (
        RunningDataflow,
        NodeId,
        DataId,
        NodeId,
        DataId,
        mpsc::Receiver<Timestamped<NodeEvent>>,
    ) {
        let mut df = test_dataflow();
        let sender: NodeId = "sender".to_string().into();
        let output: DataId = "output".to_string().into();
        let receiver: NodeId = "receiver".to_string().into();
        let input: DataId = "input".to_string().into();

        let mut mapping = BTreeSet::new();
        mapping.insert((receiver.clone(), input.clone()));
        df.mappings
            .insert(OutputId(sender.clone(), output.clone()), mapping);

        let (tx, rx) = mpsc::channel(channel_capacity);
        df.subscribe_channels.insert(receiver.clone(), tx);

        // Input is currently broken (circuit breaker open).
        df.broken_inputs
            .insert((receiver.clone(), input.clone()), Duration::from_secs(5));

        (df, sender, output, receiver, input, rx)
    }

    /// A bare `OutputSent` is not a delivery confirmation. When the receiver's
    /// event channel is still saturated (`capacity() < CONTROL_EVENT_HEADROOM`),
    /// recovery must NOT fire — otherwise the circuit breaker flaps
    /// `broken ↔ recovered` every `input_timeout` for a persistently-slow
    /// consumer without any data actually getting through.
    #[test]
    fn output_sent_does_not_recover_broken_input_when_receiver_saturated() {
        let clock = test_clock();
        let ft_stats = FaultToleranceStats::default();
        // capacity 1 < CONTROL_EVENT_HEADROOM (50) => not keeping up
        let (mut df, sender, output, receiver, input, mut rx) = broken_input_dataflow(1);

        note_output_sent_to_local_receivers(sender, output, &mut df, &clock, Some(&ft_stats));

        assert!(
            df.broken_inputs.contains_key(&(receiver, input)),
            "broken input must stay broken while the receiver is saturated"
        );
        assert_eq!(
            ft_stats
                .circuit_breaker_recoveries
                .load(atomic::Ordering::Relaxed),
            0,
            "no recovery may be counted for a saturated receiver"
        );
        let events = drain_events(&mut rx);
        assert!(
            !events.iter().any(|e| matches_event(e, "InputRecovered")),
            "no InputRecovered may be emitted while the receiver is saturated"
        );
    }

    /// Companion case: once the receiver has channel headroom (is keeping up),
    /// the same `OutputSent` recovers the broken input exactly as before.
    #[test]
    fn output_sent_recovers_broken_input_when_receiver_keeping_up() {
        let clock = test_clock();
        let ft_stats = FaultToleranceStats::default();
        // an empty NODE_EVENT_CHANNEL_CAPACITY channel has ample headroom
        let (mut df, sender, output, receiver, input, mut rx) =
            broken_input_dataflow(NODE_EVENT_CHANNEL_CAPACITY);

        note_output_sent_to_local_receivers(sender, output, &mut df, &clock, Some(&ft_stats));

        assert!(
            !df.broken_inputs.contains_key(&(receiver, input)),
            "broken input must recover once the receiver is keeping up"
        );
        assert_eq!(
            ft_stats
                .circuit_breaker_recoveries
                .load(atomic::Ordering::Relaxed),
            1,
            "recovery must be counted once the receiver is keeping up"
        );
        let events = drain_events(&mut rx);
        assert!(
            events.iter().any(|e| matches_event(e, "InputRecovered")),
            "InputRecovered must be emitted on recovery"
        );
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
        let metadata = metadata::Metadata::new(clock.new_timestamp());

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

    // -- need_data_bytes: payload must be returned for remote forwarding --

    #[tokio::test]
    async fn data_bytes_returned_with_and_without_local_receivers() {
        let clock = test_clock();
        let payload = [1u8, 2, 3, 4];
        let sender: NodeId = "sender".to_string().into();
        let output: DataId = "output".to_string().into();

        // Without local receivers the data Arc is uniquely owned; the payload
        // is moved out (no copy) but must still be returned for the remote
        // forwarding path.
        let mut df = test_dataflow();
        let metadata = metadata::Metadata::new(clock.new_timestamp());
        let data = DataMessage::Vec(AVec::from_slice(128, &payload));
        let result = send_output_to_local_receivers(
            sender.clone(),
            output.clone(),
            &mut df,
            &metadata,
            Some(data),
            &clock,
            None,
            true,
        )
        .await
        .unwrap();
        assert_eq!(result.as_deref(), Some(&payload[..]));

        // With a local receiver holding a reference, the bytes are cloned and
        // must match the payload delivered to the receiver.
        let mut df = test_dataflow();
        let receiver: NodeId = "receiver".to_string().into();
        let input: DataId = "input".to_string().into();
        df.mappings
            .entry(OutputId(sender.clone(), output.clone()))
            .or_default()
            .insert((receiver.clone(), input.clone()));
        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(receiver.clone(), tx);
        let metadata = metadata::Metadata::new(clock.new_timestamp());
        let data = DataMessage::Vec(AVec::from_slice(128, &payload));
        let result = send_output_to_local_receivers(
            sender,
            output,
            &mut df,
            &metadata,
            Some(data),
            &clock,
            None,
            true,
        )
        .await
        .unwrap();
        assert_eq!(result.as_deref(), Some(&payload[..]));
        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 1);
        assert!(matches_event(&events[0], "Input"));
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
        let metadata = metadata::Metadata::new(clock.new_timestamp());

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

    // -- Regression test for #2021: `OutputSent` must not refresh an
    //    input deadline when the receiver has fallen behind. --

    /// Since #1787 data is published directly over zenoh, so the daemon's
    /// `OutputSent` notification is not a delivery confirmation. A slow
    /// consumer whose event channel is saturated drops the zenoh payloads,
    /// so its `input_timeout` deadline must be allowed to expire instead of
    /// being refreshed on every `OutputSent`.
    #[test]
    fn output_sent_does_not_refresh_deadline_for_backpressured_receiver() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let sender: NodeId = "sender".to_string().into();
        let receiver: NodeId = "receiver".to_string().into();
        let output: DataId = "output".to_string().into();
        let input: DataId = "input".to_string().into();

        df.mappings
            .entry(OutputId(sender.clone(), output.clone()))
            .or_default()
            .insert((receiver.clone(), input.clone()));

        // Armed deadline that has already exceeded its timeout.
        let timeout = Duration::from_millis(10);
        let stale = Instant::now() - Duration::from_secs(60);
        df.input_deadlines.insert(
            (receiver.clone(), input.clone()),
            InputDeadline {
                timeout,
                last_received: Some(stale),
            },
        );

        // Saturate the receiver's channel so it has no headroom left —
        // a proxy for "the node is dropping zenoh inputs".
        let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        for _ in 0..NODE_EVENT_CHANNEL_CAPACITY {
            tx.try_send(Timestamped {
                inner: NodeEvent::AllInputsClosed,
                timestamp: clock.new_timestamp(),
            })
            .unwrap();
        }
        df.subscribe_channels.insert(receiver.clone(), tx);

        note_output_sent_to_local_receivers(sender, output, &mut df, &clock, None);

        // The deadline must NOT have been refreshed, so it still times out.
        let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
        assert_eq!(
            deadline.last_received,
            Some(stale),
            "OutputSent must not refresh the deadline when the receiver is backpressured"
        );
        assert!(
            deadline.is_timed_out(),
            "input_timeout watchdog must still be able to fire for a slow consumer (#2021)"
        );
    }

    /// Counterpart to the above: a receiver that is keeping up (channel has
    /// headroom) still has its deadline refreshed on `OutputSent`.
    #[test]
    fn output_sent_refreshes_deadline_when_receiver_keeps_up() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let sender: NodeId = "sender".to_string().into();
        let receiver: NodeId = "receiver".to_string().into();
        let output: DataId = "output".to_string().into();
        let input: DataId = "input".to_string().into();

        df.mappings
            .entry(OutputId(sender.clone(), output.clone()))
            .or_default()
            .insert((receiver.clone(), input.clone()));

        let timeout = Duration::from_secs(5);
        let stale = Instant::now() - Duration::from_secs(60);
        df.input_deadlines.insert(
            (receiver.clone(), input.clone()),
            InputDeadline {
                timeout,
                last_received: Some(stale),
            },
        );

        // Empty channel: full headroom available.
        let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(receiver.clone(), tx);

        note_output_sent_to_local_receivers(sender, output, &mut df, &clock, None);

        let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
        assert!(
            deadline.last_received.is_some_and(|t| t > stale),
            "OutputSent must refresh the deadline when the receiver is keeping up"
        );
        assert!(!deadline.is_timed_out());
    }

    /// A receiver with no daemon-side channel at all (e.g. after
    /// `EventStreamDropped`) is not draining inputs, so `OutputSent` must
    /// not refresh its deadline either — otherwise a sender's continued
    /// output would keep a disconnected receiver's deadline alive forever.
    #[test]
    fn output_sent_does_not_refresh_deadline_for_receiver_without_channel() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let sender: NodeId = "sender".to_string().into();
        let receiver: NodeId = "receiver".to_string().into();
        let output: DataId = "output".to_string().into();
        let input: DataId = "input".to_string().into();

        df.mappings
            .entry(OutputId(sender.clone(), output.clone()))
            .or_default()
            .insert((receiver.clone(), input.clone()));

        // Armed deadline that has already exceeded its timeout.
        let timeout = Duration::from_millis(10);
        let stale = Instant::now() - Duration::from_secs(60);
        df.input_deadlines.insert(
            (receiver.clone(), input.clone()),
            InputDeadline {
                timeout,
                last_received: Some(stale),
            },
        );

        // No `subscribe_channels` entry for the receiver — it has dropped
        // its event stream.
        assert!(!df.subscribe_channels.contains_key(&receiver));

        note_output_sent_to_local_receivers(sender, output, &mut df, &clock, None);

        let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
        assert_eq!(
            deadline.last_received,
            Some(stale),
            "OutputSent must not refresh the deadline when the receiver has no channel"
        );
        assert!(
            deadline.is_timed_out(),
            "input_timeout watchdog must still fire for a disconnected receiver (#2021)"
        );
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
                value_json: serde_json::to_vec(&serde_json::json!(42)).unwrap(),
            },
            &clock,
        );
        assert!(result.is_ok());

        let events = drain_events(&mut rx);
        assert_eq!(events.len(), 1);
        match &events[0] {
            NodeEvent::ParamUpdate { key, value_json } => {
                assert_eq!(key, "threshold");
                let value: serde_json::Value = serde_json::from_slice(value_json).unwrap();
                assert_eq!(value, serde_json::json!(42));
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
                value_json: serde_json::to_vec(&serde_json::json!(10)).unwrap(),
            },
            &clock,
        );
        assert!(result.is_err());
    }

    #[test]
    fn strict_param_update_fails_when_node_not_connected() {
        let df = test_dataflow();
        let clock = test_clock();
        let node_id: NodeId = "node_missing".to_string().into();

        let err = deliver_param_update_strict(
            &df,
            &node_id,
            "threshold".into(),
            serde_json::json!(1),
            &clock,
        )
        .expect_err("strict delivery should fail when node channel is missing");
        assert!(err.to_string().contains("not connected"));
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

    #[test]
    fn state_catch_up_stops_at_first_full_channel_and_returns_applied_prefix() {
        let clock = test_clock();
        let mut dataflow = test_dataflow();
        let node_a: NodeId = "node_a".to_string().into();
        let node_b: NodeId = "node_b".to_string().into();

        let (tx_a, mut rx_a) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        let (tx_b, _rx_b) = mpsc::channel::<Timestamped<NodeEvent>>(1);
        tx_b.try_send(Timestamped {
            inner: NodeEvent::Stop,
            timestamp: clock.new_timestamp(),
        })
        .expect("prefill node_b channel");

        dataflow.subscribe_channels.insert(node_a.clone(), tx_a);
        dataflow.subscribe_channels.insert(node_b.clone(), tx_b);

        let applied_through = apply_state_catch_up_entries(
            &dataflow,
            &[
                StateCatchUpEntry {
                    sequence: 1,
                    operation: StateCatchUpOperation::SetParam {
                        node_id: node_a.clone(),
                        key: "threshold".into(),
                        value: serde_json::json!(42),
                    },
                },
                StateCatchUpEntry {
                    sequence: 2,
                    operation: StateCatchUpOperation::DeleteParam {
                        node_id: node_b,
                        key: "threshold".into(),
                    },
                },
            ],
            &clock,
        );

        assert_eq!(applied_through, 1);

        let events = drain_events(&mut rx_a);
        assert_eq!(events.len(), 1);
        match &events[0] {
            NodeEvent::ParamUpdate { key, value_json } => {
                assert_eq!(key, "threshold");
                let value: serde_json::Value = serde_json::from_slice(value_json).unwrap();
                assert_eq!(value, serde_json::json!(42));
            }
            other => panic!("expected ParamUpdate, got {other:?}"),
        }
    }

    #[test]
    fn state_catch_up_returns_zero_when_first_entry_cannot_be_delivered() {
        let clock = test_clock();
        let dataflow = test_dataflow();
        let node_id: NodeId = "node_a".to_string().into();

        let applied_through = apply_state_catch_up_entries(
            &dataflow,
            &[StateCatchUpEntry {
                sequence: 7,
                operation: StateCatchUpOperation::SetParam {
                    node_id,
                    key: "threshold".into(),
                    value: serde_json::json!(42),
                },
            }],
            &clock,
        );

        assert_eq!(applied_through, 0);
    }

    #[test]
    fn strict_param_delete_fails_when_channel_full() {
        let mut df = test_dataflow();
        let clock = test_clock();
        let node_id: NodeId = "node_a".to_string().into();
        let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        df.subscribe_channels.insert(node_id.clone(), tx.clone());

        // Saturate channel so strict delivery observes a dropped send.
        for _ in 0..NODE_EVENT_CHANNEL_CAPACITY {
            let sent = send_with_timestamp(
                &tx,
                NodeEvent::ParamDeleted {
                    key: "prefill".into(),
                },
                &clock,
            )
            .unwrap();
            assert!(sent);
        }

        let err = deliver_param_delete_strict(&df, &node_id, "threshold".into(), &clock)
            .expect_err("strict delivery should fail when node channel is full");
        assert!(err.to_string().contains("channel full"));
    }

    #[test]
    fn reconnect_window_retries_then_exits() {
        let window = Duration::from_secs(30);
        let mut deadline = None;

        // First failed reconnect: starts the window, must not exit yet. This
        // is the regression from dora-rs/dora#1998 where a fast TCP refusal
        // (coordinator briefly restarting) exited the daemon immediately.
        let t0 = Instant::now();
        assert!(!reconnect_window_elapsed(t0, &mut deadline, window));
        assert_eq!(deadline, Some(t0 + window));

        // Subsequent failures inside the window keep retrying without
        // resetting the deadline.
        assert!(!reconnect_window_elapsed(
            t0 + Duration::from_secs(5),
            &mut deadline,
            window
        ));
        assert!(!reconnect_window_elapsed(
            t0 + Duration::from_secs(29),
            &mut deadline,
            window
        ));
        assert_eq!(deadline, Some(t0 + window));

        // Once the window has elapsed, the daemon gives up and exits so a
        // permanently-gone coordinator doesn't leave an orphan (#1996).
        assert!(reconnect_window_elapsed(t0 + window, &mut deadline, window));
        assert!(reconnect_window_elapsed(
            t0 + Duration::from_secs(31),
            &mut deadline,
            window
        ));
    }

    #[test]
    fn reconnect_window_resets_after_successful_connect() {
        let window = Duration::from_secs(30);
        let mut deadline = None;

        let t0 = Instant::now();
        assert!(!reconnect_window_elapsed(t0, &mut deadline, window));

        // A successful reconnect clears the deadline (the loop sets
        // `reconnect_deadline = None`), so a later outage gets a full fresh
        // window rather than inheriting the old, possibly-elapsed deadline.
        deadline = None;
        let t1 = t0 + Duration::from_secs(100);
        assert!(!reconnect_window_elapsed(t1, &mut deadline, window));
        assert_eq!(deadline, Some(t1 + window));
        assert!(!reconnect_window_elapsed(
            t1 + Duration::from_secs(29),
            &mut deadline,
            window
        ));
    }
}

#[cfg(test)]
mod planned_stop_exit_tests {
    use super::{STATUS_CONTROL_C_EXIT, is_sigterm_like_exit};
    use dora_message::common::NodeExitStatus;

    #[test]
    fn status_control_c_exit_constant_matches_ntstatus() {
        // STATUS_CONTROL_C_EXIT = 0xC000013A, surfaced by
        // `ExitStatus::code()` (u32 -> i32) on Windows.
        assert_eq!(STATUS_CONTROL_C_EXIT, 0xC000013Au32 as i32);
        assert_eq!(STATUS_CONTROL_C_EXIT, -1073741510);
    }

    #[test]
    fn recognises_planned_stop_exit_shapes() {
        // Unix SIGTERM / SIGINT.
        assert!(is_sigterm_like_exit(&NodeExitStatus::Signal(15)));
        assert!(is_sigterm_like_exit(&NodeExitStatus::Signal(2)));
        // Wrapper (`uv run python`) that catches the signal and exits 128 + signo.
        assert!(is_sigterm_like_exit(&NodeExitStatus::ExitCode(143)));
        assert!(is_sigterm_like_exit(&NodeExitStatus::ExitCode(130)));
        // Windows: unhandled CTRL_BREAK_EVENT -> STATUS_CONTROL_C_EXIT (dora-rs/dora#2425).
        assert!(is_sigterm_like_exit(&NodeExitStatus::ExitCode(
            STATUS_CONTROL_C_EXIT
        )));
    }

    #[test]
    fn does_not_recognise_genuine_failures() {
        assert!(!is_sigterm_like_exit(&NodeExitStatus::Success));
        assert!(!is_sigterm_like_exit(&NodeExitStatus::ExitCode(1)));
        assert!(!is_sigterm_like_exit(&NodeExitStatus::ExitCode(-1)));
        assert!(!is_sigterm_like_exit(&NodeExitStatus::Unknown));
        // SIGKILL is a hard kill (grace exceeded), not a graceful stop —
        // it must keep flowing through the GraceDuration branch.
        assert!(!is_sigterm_like_exit(&NodeExitStatus::Signal(9)));
    }
}

#[cfg(test)]
mod log_tail_tests {
    use super::*;
    use std::io::Write as _;

    #[tokio::test]
    async fn read_last_n_lines_handles_huge_tail_without_overflow() {
        // `--tail N` is unbounded client input. A very large `N` used to make
        // the `estimated_line_length * tail` buffer-growth estimate overflow
        // `usize` (a debug-mode panic in the log task). With fewer lines than
        // requested the whole file is returned; the request must not panic.
        let mut tmp = tempfile::NamedTempFile::new().unwrap();
        write!(tmp, "line1\nline2\nline3\n").unwrap();
        tmp.flush().unwrap();

        let mut file = File::open(tmp.path()).await.unwrap();
        let out = read_last_n_lines(&mut file, usize::MAX).await.unwrap();

        assert_eq!(out, b"line1\nline2\nline3");
    }
}

#[cfg(test)]
mod announce_zenoh_bind_tests {
    use super::*;
    use std::net::Ipv4Addr;

    const LOOPBACK: IpAddr = IpAddr::V4(Ipv4Addr::LOCALHOST);
    const REMOTE_COORDINATOR: IpAddr = IpAddr::V4(Ipv4Addr::new(203, 0, 113, 5));
    const ROUTABLE_LOCAL: IpAddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 20));

    fn sock(ip: IpAddr) -> SocketAddr {
        SocketAddr::new(ip, 6012)
    }

    #[test]
    fn explicit_loopback_under_remote_coordinator_is_rejected() {
        // #2770: an operator who names a loopback address while the coordinator
        // is remote gets a silently-undialable daemon. The `Explicit` contract
        // is "bind a routable address or exit", so this must be a hard error.
        let err = announce_zenoh_bind(ZenohBind::Explicit(LOOPBACK), sock(REMOTE_COORDINATOR))
            .unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("loopback"), "unexpected error: {msg}");
        assert!(msg.contains("--zenoh-listen"), "unexpected error: {msg}");
    }

    #[test]
    fn derived_loopback_under_remote_coordinator_only_warns() {
        // A derived loopback is a best-effort fallback (no routable address was
        // found), so the daemon still starts — the operator can override it.
        announce_zenoh_bind(ZenohBind::Derived(LOOPBACK), sock(REMOTE_COORDINATOR)).unwrap();
    }

    #[test]
    fn explicit_loopback_under_local_coordinator_is_allowed() {
        // Single-machine `dora run`: coordinator and nodes share loopback, so a
        // loopback listener is both sufficient and correct.
        announce_zenoh_bind(ZenohBind::Explicit(LOOPBACK), sock(LOOPBACK)).unwrap();
    }

    #[test]
    fn explicit_routable_address_is_allowed() {
        announce_zenoh_bind(
            ZenohBind::Explicit(ROUTABLE_LOCAL),
            sock(REMOTE_COORDINATOR),
        )
        .unwrap();
    }
}

#[cfg(test)]
mod health_check_tests {
    use super::health_check_should_kill;
    use std::time::Duration;

    const TIMEOUT: Duration = Duration::from_secs(5);

    #[test]
    fn not_connected_is_never_killed_even_when_long_silent() {
        // Regression for #2937: `last_activity` is seeded to the spawn
        // timestamp, so a node in a slow cold start (imports + model-weight
        // load) reads as long-silent well before it ever connects. The
        // watchdog must not kill it while it is still starting up — even if
        // the elapsed time since spawn already exceeds the timeout.
        let spawn = 1_000u64;
        let now = spawn + 10_000; // 10s after spawn, timeout is 5s
        assert!(!health_check_should_kill(false, spawn, now, TIMEOUT));
    }

    #[test]
    fn connected_and_silent_past_timeout_is_killed() {
        let last = 1_000u64;
        let now = last + 6_000; // 6s of post-connection silence, timeout 5s
        assert!(health_check_should_kill(true, last, now, TIMEOUT));
    }

    #[test]
    fn connected_and_recently_active_is_not_killed() {
        let last = 1_000u64;
        let now = last + 4_000; // 4s < 5s timeout
        assert!(!health_check_should_kill(true, last, now, TIMEOUT));
    }

    #[test]
    fn exactly_at_timeout_is_not_killed() {
        // The comparison is strictly greater-than, so a node silent for
        // exactly the timeout is given the benefit of the doubt.
        let last = 1_000u64;
        let now = last + 5_000;
        assert!(!health_check_should_kill(true, last, now, TIMEOUT));
    }

    #[test]
    fn clock_skew_does_not_underflow() {
        // `now` before `last` (clock went backwards) must not panic via
        // subtraction underflow, and must not be treated as elapsed time.
        let last = 10_000u64;
        let now = 1_000u64;
        assert!(!health_check_should_kill(true, last, now, TIMEOUT));
    }
}
