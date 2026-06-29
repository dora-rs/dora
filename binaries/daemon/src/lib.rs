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
        DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST, open_zenoh_session_with_listen,
        reserve_loopback_zenoh_endpoint, zenoh_daemon_control_topic, zenoh_output_publish_topic,
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
    metadata::{self, ArrowTypeInfo, MetadataParameters},
    node_to_daemon::{DynamicNodeEvent, Timestamped},
};
use dora_node_api::arrow::datatypes::DataType;
use eyre::{Context, ContextCompat, Result, bail, eyre};
use futures::{TryFutureExt, future, stream};
use futures_concurrency::stream::Merge;
use local_listener::DynamicNodeEventWrapper;
use log::{CoordinatorLogTarget, DaemonLogger, DataflowLogger, Logger};
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
/// patterns; messages are dropped with a warning when full.
const ZENOH_PUBLISH_CHANNEL_CAPACITY: usize = 256;
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
    pub(crate) zenoh_publish_tx: mpsc::Sender<ZenohOutbound>,
    pub(crate) remote_daemon_events_tx:
        Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
    pub(crate) logger: DaemonLogger,
    pub(crate) sessions: BTreeMap<SessionId, BuildId>,
    pub(crate) builds: BTreeMap<BuildId, BuildInfo>,
    pub(crate) git_manager: GitManager,
    pub(crate) metrics_system: Arc<std::sync::Mutex<sysinfo::System>>,
    pub(crate) memory_pool: MemoryPoolManager,
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
        pinned_type: get_str("pinned_type"),
    }
}

impl Daemon {
    pub async fn run(
        coordinator_ws_addr: SocketAddr,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        local_listen_port: u16,
        inter_daemon_peer: Option<String>,
    ) -> eyre::Result<()> {
        Self::run_with_builds(
            coordinator_ws_addr,
            machine_id,
            labels,
            local_listen_port,
            inter_daemon_peer,
            Default::default(),
        )
        .await
    }

    pub async fn run_with_builds(
        coordinator_ws_addr: SocketAddr,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        local_listen_port: u16,
        inter_daemon_peer: Option<String>,
        initial_builds: BTreeMap<BuildId, BuildInfo>,
    ) -> eyre::Result<()> {
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
            // Local dataflow runs (one daemon, no cluster) never need
            // cross-daemon Zenoh discovery; the rendezvous is irrelevant.
            None,
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
    ) -> eyre::Result<DaemonRunResult> {
        // Single-shot path (`dora run`): build the daemon and run one event
        // loop. The reconnecting daemon binary instead builds the daemon once
        // and reuses it across reconnects (see `run_with_builds`), so that node
        // processes are not killed when the coordinator connection drops.
        let (mut daemon, mut dora_events_rx) = Self::build_daemon(
            coordinator_sender,
            daemon_id,
            exit_when_done,
            clock,
            remote_daemon_events_tx,
            builds,
            log_destination,
            inter_daemon_peer,
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
    ) -> eyre::Result<(Self, mpsc::Receiver<Timestamped<Event>>)> {
        // Reserve a loopback port and have zenoh listen on it. The endpoint is
        // injected into spawned nodes via `DORA_ZENOH_CONNECT` so peer
        // discovery works without multicast (#1778).
        let requested_listen_endpoint = match reserve_loopback_zenoh_endpoint() {
            Ok(ep) => Some(ep),
            Err(err) => {
                tracing::warn!(
                    "failed to reserve loopback zenoh listen endpoint: {err}; \
                     falling back to multicast scouting only"
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
        )
        .await
        .wrap_err("failed to open zenoh session")?;
        if requested_listen_endpoint.is_some() && zenoh_listen_endpoint.is_none() {
            tracing::warn!(
                "requested zenoh listener but zenoh did not bind it; \
                 spawned nodes will use multicast scouting only"
            );
        }
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
            zenoh_listen_endpoint,
            zenoh_publish_tx,
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
        let health_check_interval = tokio_stream::wrappers::IntervalStream::new(
            tokio::time::interval(health_check_interval_duration.unwrap_or(Duration::from_secs(5))),
        )
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
                            // Return error to trigger the reconnection loop in
                            // `run_with_builds`. Because `run_inner` borrows
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
                        zenoh_connect_endpoint: self.zenoh_listen_endpoint.clone(),
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
                    for receivers in dataflow.timers.values_mut() {
                        receivers.retain(|(nid, _)| nid != &node_id);
                    }
                    dataflow
                        .log_subscribers
                        .retain(|sub| sub.node_id != node_id);

                    // Clean up remaining state for this node.
                    dataflow.running_nodes.remove(&node_id);
                    dataflow.open_inputs.remove(&node_id);
                    dataflow.subscribe_channels.remove(&node_id);
                    dataflow.pending_messages.remove(&node_id);
                    dataflow.all_inputs_closed_at.remove(&node_id);
                    // clear the connected marker too, else a re-added node ID
                    // would look already-connected before its new incarnation
                    // subscribes and could be selected mid-startup (dora#2270).
                    dataflow.connected_nodes.remove(&node_id);
                    dataflow.finish_escalated.remove(&node_id);

                    // Remove from stored descriptor (inverse of AddNode
                    // push) so descriptor-based lookups stay consistent.
                    dataflow.descriptor.nodes.retain(|n| n.id != node_id);
                    Ok(())
                })();

                if let Err(err) = &result {
                    tracing::error!(%dataflow_id, %node_id, "RemoveNode failed: {err:?}");
                }
                let reply = DaemonCoordinatorReply::RemoveNodeResult(
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
                    let output_id = OutputId(source_node, source_output);
                    dataflow
                        .mappings
                        .entry(output_id)
                        .or_default()
                        .insert((target_node.clone(), target_input.clone()));
                    // Reopening an input ends any drain: clear the stale clock so
                    // the selector does not treat the node as drained-and-eligible
                    // on a timestamp from before the mapping was re-added (#2270).
                    dataflow.all_inputs_closed_at.remove(&target_node);
                    dataflow
                        .open_inputs
                        .entry(target_node)
                        .or_default()
                        .insert(target_input);
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
                    let events_tx = self.events_tx.clone();
                    let clock = self.clock.clone();
                    let node_id = node.id.clone();
                    let mut finished_rx = dataflow.finished_tx.subscribe();
                    tokio::spawn(async move {
                        let mut finished = pin!(finished_rx.recv());
                        loop {
                            match future::select(finished, subscriber.recv_async()).await {
                                future::Either::Left((_, _)) => break,
                                future::Either::Right((sample, f)) => {
                                    finished = f;
                                    let Ok(sample) = sample else { break };
                                    // Node publishes raw payload + bincode metadata
                                    // attachment (see `DoraNode::zenoh_publish`).
                                    let Some(metadata) = sample.attachment().and_then(|a| {
                                        bincode::deserialize::<dora_message::metadata::Metadata>(
                                            &a.to_bytes(),
                                        )
                                        .ok()
                                    }) else {
                                        continue;
                                    };
                                    let event = Event::DebugTopicData {
                                        dataflow_id,
                                        output_id: OutputId(node_id.clone(), output_id.clone()),
                                        metadata,
                                        data: Some(sample.payload().to_bytes().to_vec()),
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
                                dataflow.start(&self.events_tx, &self.clock).await?;
                                dataflow.dataflow_started = true;
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
                        && let Err(err) = self.memory_pool.free_memory_pool(&id, node_id.as_ref())
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
                    let type_info = dora_message::metadata::ArrowTypeInfo {
                        data_type: DataType::Null,
                        len: 0,
                        null_count: 0,
                        validity: None,
                        offset: 0,
                        buffer_offsets: vec![],
                        child_data: vec![],
                        field_names: None,
                        schema_hash: None,
                    };

                    Ok(dora_message::metadata::Metadata::from_parameters(
                        timestamp, type_info, parameters,
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
                    id: shared_memory_id.clone(),
                };
                let result: Result<(), String> =
                    match self.memory_pool.free_memory_pool(&id, node_id.as_ref()) {
                        Ok(_) => Ok(()),
                        Err(e) => Err(e),
                    };
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
            || dataflow.enable_debug_inspection;
        let has_debug_watchers = dataflow.debug_topic_watchers.contains_key(&output_id_key);
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
            self.send_to_remote_receivers(dataflow_id, &output_id, serialized_event)
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
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("output sent failed: no running dataflow with ID `{dataflow_id}`")
        })?;
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
                    dataflow
                        .all_inputs_closed_at
                        .insert(node_id.clone(), Instant::now());
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

                // Convert to Arrow once, share the sample across subscribers
                use dora_arrow_convert::IntoArrow;
                let array = json.as_str().into_arrow();
                let array: dora_node_api::arrow::array::ArrayData = array.into();
                let total_len = dora_node_api::arrow_utils::required_data_size(&array);
                let mut sample: aligned_vec::AVec<u8, aligned_vec::ConstAlign<128>> =
                    aligned_vec::AVec::__from_elem(128, 0, total_len);
                let type_info =
                    dora_node_api::arrow_utils::copy_array_into_sample(&mut sample, &array);
                let data = Arc::new(DataMessage::Vec(sample));

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
                        // Same shape for SIGINT (2 / 130). Treat any of
                        // those as a clean planned stop so `dora run
                        // --stop-after` doesn't report a fake "Node
                        // failed: exited with code 143" when the
                        // dataflow shut down exactly as requested
                        // (dora-rs/dora#1882).
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
                        let is_sigterm_like = matches!(
                            exit_status,
                            NodeExitStatus::Signal(15)
                                | NodeExitStatus::Signal(2)
                                | NodeExitStatus::ExitCode(143)
                                | NodeExitStatus::ExitCode(130)
                        );
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

                // Clear the per-incarnation kill marker so it doesn't
                // leak into the next incarnation. `grace_duration_kills`
                // is keyed only by `node_id`; if `restart=true` and we
                // didn't clear here, a later external SIGTERM or
                // unrelated 143 exit from the restarted process would
                // still see `grace_duration_kill=true` and be
                // misreported as `Ok(())`. The marker has done its job
                // for this exit — drop it. Same for the drain clock: a
                // respawned node under the same id must start fresh.
                // (`finish_escalated` is NOT cleared here — it is read
                // and consumed by `handle_node_stop_inner` below to keep
                // the coordinator-facing `clean_stop` flag honest; an
                // escalated node never restarts, so it cannot leak into
                // a next incarnation.)
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    dataflow.grace_duration_kills.remove(&node_id);
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
                        let error_msg = e.to_string();
                        let mut affected_by_receiver: BTreeMap<NodeId, Vec<DataId>> =
                            BTreeMap::new();
                        for (output_id, receivers) in &dataflow.mappings {
                            if output_id.0 == node_id {
                                for (recv_id, input_id) in receivers {
                                    affected_by_receiver
                                        .entry(recv_id.clone())
                                        .or_default()
                                        .push(input_id.clone());
                                }
                            }
                        }
                        for (recv_id, affected_ids) in affected_by_receiver {
                            if let Some(channel) = dataflow.subscribe_channels.get(&recv_id) {
                                let _ = send_with_timestamp(
                                    channel,
                                    NodeEvent::NodeFailed {
                                        affected_input_ids: affected_ids,
                                        error: error_msg.clone(),
                                        source_node_id: node_id.clone(),
                                    },
                                    &self.clock,
                                );
                            }
                        }
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
        if receiver_keeping_up
            && let Some(deadline) = dataflow
                .input_deadlines
                .get_mut(&(receiver_id.clone(), input_id.clone()))
        {
            deadline.last_received = Some(now);
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
                dataflow
                    .all_inputs_closed_at
                    .insert(receiver_id.clone(), Instant::now());
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
                dataflow
                    .all_inputs_closed_at
                    .insert(receiver_id.clone(), Instant::now());
            }
        }
    }
}

/// Grace period used when the finish-straggler watchdog is enabled but
/// `DORA_FINISH_DRAIN_GRACE_SECS` is set to an unparseable value. Conservative:
/// a sink may legitimately keep working for a while after its inputs close
/// (flushing recordings, final writes).
const DEFAULT_FINISH_DRAIN_GRACE: Duration = Duration::from_secs(120);

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
            env: None,
        };
        RunningDataflow::new(Uuid::nil(), DaemonId::new(None), descriptor)
    }

    fn test_running_node() -> RunningNode {
        RunningNode {
            process: None,
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

    // -- need_data_bytes: payload must be returned for remote forwarding --

    #[tokio::test]
    async fn data_bytes_returned_with_and_without_local_receivers() {
        let clock = test_clock();
        let payload = [1u8, 2, 3, 4];
        let type_info = ArrowTypeInfo {
            data_type: DataType::UInt8,
            len: payload.len(),
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![],
            child_data: vec![],
            field_names: None,
            schema_hash: None,
        };
        let sender: NodeId = "sender".to_string().into();
        let output: DataId = "output".to_string().into();

        // Without local receivers the data Arc is uniquely owned; the payload
        // is moved out (no copy) but must still be returned for the remote
        // forwarding path.
        let mut df = test_dataflow();
        let metadata = metadata::Metadata::new(clock.new_timestamp(), type_info.clone());
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
        let metadata = metadata::Metadata::new(clock.new_timestamp(), type_info);
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
