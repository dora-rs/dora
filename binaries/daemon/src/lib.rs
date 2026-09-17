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
use coordinator::CoordinatorEvent;
use dora_core::{
    build::{BuildInfo, GitManager},
    config::{DataId, Input, NodeId, NodeRunConfig},
    descriptor::{
        CoreNodeKind, DYNAMIC_SOURCE, Descriptor, DescriptorExt, RuntimeNode, read_as_descriptor,
        validate,
    },
    topics::{
        DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST, MulticastScouting,
        open_zenoh_session_with_listen, validate_zenoh_listen, zenoh_bind_address_for,
    },
    uhlc::{self, HLC},
};
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{DaemonId, DataMessage, LogLevel, NodeError},
    coordinator_to_cli::DataflowResult,
    coordinator_to_daemon::{DaemonCoordinatorEvent, SpawnDataflowNodes},
    daemon_to_coordinator::{CoordinatorRequest, DaemonCoordinatorReply, DaemonEvent},
    daemon_to_node::NodeEvent,
    descriptor::NodeSource,
    node_to_daemon::Timestamped,
};
use eyre::{Context, ContextCompat, Result, bail, eyre};
use futures::{TryFutureExt, future, stream};
use futures_concurrency::stream::Merge;
use local_listener::DynamicNodeEventWrapper;
use log::{CoordinatorLogTarget, DaemonLogger, Logger};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque},
    io,
    net::{IpAddr, SocketAddr},
    path::{Path, PathBuf},
    pin::pin,
    sync::{Arc, atomic},
    time::{Duration, Instant},
};
use tokio::{
    fs::File,
    io::{AsyncReadExt, AsyncSeekExt},
    sync::{mpsc, oneshot},
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
    use aligned_vec::AVec;
    use dora_message::metadata;
    use std::sync::atomic::AtomicU64;

    /// Create a minimal `RunningDataflow` with the given sender->receiver mapping.
    /// Returns the dataflow and a vec of receivers (one per subscriber).
    pub fn setup_routing(
        fan_out: usize,
    ) -> (
        RunningDataflow,
        HLC,
        Vec<mpsc::Receiver<Timestamped<NodeEvent>>>,
    ) {
        let descriptor = dora_message::descriptor::Descriptor::new(vec![]);
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
        let output_id = OutputId(fixture.sender_id.clone(), fixture.output_id.clone());
        let _ = send_output_to_local_receivers(
            &output_id,
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
mod coordinator_events;
mod dataflow_lifecycle;
mod debug_topic;
mod dora_events;
pub(crate) mod event_types;
mod extension_table;
mod extract_err_from_stderr;
pub(crate) mod fault_tolerance;
mod local_delivery;
mod local_listener;
mod log;
mod metrics;
mod node_communication;
mod node_events;
mod node_exit;
mod output_routing;
mod pending;
pub(crate) mod running_dataflow;
mod shutdown;
mod socket_stream_utils;
mod spawn;
#[cfg(test)]
mod tests;
mod zenoh_bind;

pub(crate) use debug_topic::{DebugSchemaCache, rebuild_debug_topic_stream, retain_debug_schema};
pub(crate) use event_types::{
    CONTROL_EVENT_HEADROOM, DaemonNodeEvent, DoraEvent, Event, InterDaemonEvent,
    NODE_EVENT_CHANNEL_CAPACITY, OutputId, RunStatus, ZenohOutbound, send_with_timestamp,
};
pub(crate) use fault_tolerance::{CascadingErrorCauses, FaultToleranceStats};
pub(crate) use local_delivery::{
    break_input, close_input, close_inputs_best_effort, node_inputs,
    note_output_sent_to_local_receivers, reject_unpinnable_backpressure_inputs,
    send_output_to_local_receivers,
};
pub(crate) use metrics::{METRICS_INTERVAL, MetricsState};
pub(crate) use node_exit::{
    finish_drain_grace, health_check_should_kill, is_sigkill_like_exit, is_sigterm_like_exit,
    startup_timeout_should_kill,
};
pub(crate) use running_dataflow::{
    FinishDataflowWhen, InputDeadline, ProcessHandle, ProcessOperation, RunningDataflow,
    RunningNode,
};
pub use zenoh_bind::ZenohOptions;
pub(crate) use zenoh_bind::{
    AdvertiseListener, ZenohBind, ZenohRegistration, announce_zenoh_bind,
    reserve_zenoh_listen_endpoint,
};

use crate::extension_table::{ExtensionKey, ExtensionTable};

const STDERR_LOG_LINES_MAX: usize = 500;
/// Per-output Zenoh publish queue capacity. Keeping this smaller than the old
/// daemon-wide queue bounds memory while preserving FIFO ordering for each
/// output's data and close events.
const ZENOH_OUTPUT_QUEUE_CAPACITY: usize = 32;
/// Bound blocking control publishes so a stuck Zenoh put cannot hold a
/// per-output drain task forever. Regular data-plane publishes do not use this.
const ZENOH_PUBLISH_TIMEOUT: Duration = Duration::from_secs(1);
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

/// Tell every node that touched `key` that it is gone.
///
/// Returns the nodes that could not be reached. A full or closed channel is
/// reported rather than retried: the entry is already out of the table, so the
/// alternative to a warning is a silent resource leak in that node
/// (dora-rs/dora#2935 is what that looks like in practice).
fn notify_extension_dropped(
    dataflow: &RunningDataflow,
    namespace: &str,
    key: &str,
    touched_by: &BTreeSet<NodeId>,
    clock: &HLC,
) -> Vec<NodeId> {
    let mut undelivered = Vec::new();
    for node_id in touched_by {
        let Some(channel) = dataflow.subscribe_channels.get(node_id) else {
            // Not connected: it has either exited (nothing to release) or has
            // not subscribed yet (it cannot hold the key either).
            continue;
        };
        let event = NodeEvent::ExtensionDropped {
            namespace: namespace.to_owned(),
            key: key.to_owned(),
        };
        match send_with_timestamp(channel, event, clock) {
            Ok(true) => dataflow.inc_pending(node_id),
            Ok(false) | Err(_) => undelivered.push(node_id.clone()),
        }
    }
    undelivered
}

/// Drop `key` from the table and notify its readers, logging any that could
/// not be reached. Shared by the explicit drop request and by reclamation.
pub(crate) fn drop_extension_and_notify(
    extensions: &mut ExtensionTable,
    dataflow: Option<&RunningDataflow>,
    key: &ExtensionKey,
    clock: &HLC,
) -> bool {
    let Some(touched_by) = extensions.drop_key(key) else {
        return false;
    };
    if let Some(dataflow) = dataflow {
        let undelivered =
            notify_extension_dropped(dataflow, &key.namespace, &key.key, &touched_by, clock);
        if !undelivered.is_empty() {
            tracing::warn!(
                namespace = %key.namespace,
                key = %key.key,
                nodes = ?undelivered,
                "extension drop notification undelivered; these nodes keep whatever \
                 they derived from the value until they exit"
            );
        }
    }
    true
}

/// Drop every extension entry owned by an exited node, notifying readers.
fn reclaim_extensions_of_exited_node(
    extensions: &mut ExtensionTable,
    dataflow: Option<&RunningDataflow>,
    dataflow_id: DataflowId,
    node_id: &NodeId,
    clock: &HLC,
) {
    let reclaimed = extensions.reclaim_owner(&dataflow_id.to_string(), node_id);
    for (key, touched_by) in reclaimed {
        if let Some(dataflow) = dataflow {
            let undelivered =
                notify_extension_dropped(dataflow, &key.namespace, &key.key, &touched_by, clock);
            if !undelivered.is_empty() {
                tracing::warn!(
                    namespace = %key.namespace,
                    key = %key.key,
                    nodes = ?undelivered,
                    "extension reclaim notification undelivered after node `{node_id}` exited"
                );
            }
        }
        tracing::debug!(
            namespace = %key.namespace,
            key = %key.key,
            "reclaimed extension entry of exited node `{node_id}`"
        );
    }
}

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

#[cfg(feature = "tensor-pool")]
mod pool_extension;

/// The Daemon manages running dataflows, node communication, and inter-daemon
/// message routing.
///
/// Its behavior is spread over several `impl Daemon` blocks, one per concern:
/// the startup and event loop here, `coordinator_events` for commands from
/// the coordinator, `node_events` for requests from local nodes,
/// `dora_events` for daemon-internal events and watchdogs,
/// `dataflow_lifecycle` for building and spawning, `debug_topic` for topic
/// inspection, and `metrics` for the `dora top` sampler. Fields are
/// `pub(crate)` so those blocks can reach them.
pub struct Daemon {
    /// This machine's id (as registered with the coordinator), if any.
    /// Used to gate which daemon mirrors a cross-machine pool and to
    /// inject `DORA_MACHINE_ID` into spawned nodes.
    pub(crate) machine_id: Option<String>,
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
    /// Address other machines reach this host at, when there is one.
    ///
    /// `None` for a single-machine daemon, whose zenoh listener is on loopback:
    /// handing that address to a consumer on another machine would point it at
    /// its own loopback. Used to give a node with cross-machine consumers a
    /// second, network-reachable listener (see `spawn::reserve_node_listeners`).
    pub(crate) zenoh_routable_addr: Option<IpAddr>,
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
    pub(crate) remote_daemon_events_tx:
        Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
    pub(crate) logger: DaemonLogger,
    pub(crate) sessions: BTreeMap<SessionId, BuildId>,
    pub(crate) builds: BTreeMap<BuildId, BuildInfo>,
    pub(crate) git_manager: GitManager,
    /// Carry-over state for the `dora top` metrics sampler. See [`MetricsState`].
    pub(crate) metrics_state: Arc<std::sync::Mutex<MetricsState>>,
    /// Opaque, dataflow-scoped store for out-of-tree extensions. See
    /// `extension_table` and `docs/extensions.md`.
    pub(crate) extensions: ExtensionTable,
    /// State owned by the tensor-pool extension. Behind the `tensor-pool`
    /// feature, so a default daemon carries none of it — see
    /// `libraries/extensions/tensor-pool/README.md`.
    #[cfg(feature = "tensor-pool")]
    pub(crate) pool: dora_tensor_pool::daemon::PoolState,
    /// Nodes already warned about for sending after their dataflow
    /// finished, so `log_late_node_output` warns once each instead of
    /// once per message. See `MAX_WARNED_LATE_OUTPUT_NODES`.
    pub(crate) warned_late_outputs: HashSet<(DataflowId, NodeId)>,
}

type DaemonRunResult = BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>;

/// Whether a node-connection event belongs to a superseded incarnation.
///
/// An event is fresh only when its generation belongs to this entry's own
/// incarnation lineage: either the currently-registered incarnation
/// (`entry_generation`) or the successor this entry's OWN restart loop
/// announced before spawning it (`successor_generation`, published into the
/// entry's shared `generation_counter` before the successor connects).
///
/// The successor exception is why this is not a plain `== entry_generation`
/// check: the restart loop publishes the successor's generation to the
/// listener and spawns it BEFORE the daemon processes the matching
/// `ProcessHandleReplaced` (which advances `entry_generation`), so a fast
/// successor's `Subscribe`/`SendOut` can arrive while the entry still holds
/// the predecessor's generation. Dropping those would eat the successor's
/// one-shot `Subscribe` (dora-rs/dora#2988 review, finding 1).
///
/// The reason it is not a plain `event < entry` check either: that wrongly
/// accepts a *stranger's* higher generation. When `dora node replace` races a
/// restart of the outgoing node, the outgoing incarnation's restart loop mints
/// a generation ABOVE the replacement's (both draw from the same global
/// counter, but the replacement's is minted first, before its slow build). Its
/// zombie's events would then pass a `<` gate and be applied to the
/// replacement — reinstalling a dead subscribe channel, closing the
/// replacement's outputs, injecting spurious output (dora-rs/dora#2997).
/// Matching on the entry's OWN announced successor rejects that stranger — the
/// stranger's generation lives in the outgoing lineage's counter, never the
/// replacement's — while still letting the real successor through.
fn event_generation_is_stale(
    entry_generation: u64,
    successor_generation: u64,
    event_generation: u64,
) -> bool {
    event_generation != entry_generation && event_generation != successor_generation
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

/// Extract a finished dataflow's collected node results out of
/// `dataflow_node_results` for its terminal [`DataflowDaemonResult`].
///
/// When `keep_entry` is `false` (a persistent, coordinator-managed daemon) the
/// entry is removed, because nothing reads it again after the result is
/// reported — leaving it in place makes the map grow without bound over the
/// daemon's lifetime. When `keep_entry` is `true` (the run-once `dora run`
/// path) the entry is cloned and left in place, because `run_inner` drains the
/// whole map via [`std::mem::take`] *after* `finish_dataflow` returns.
fn extract_node_results(
    results: &mut BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>,
    dataflow_id: Uuid,
    keep_entry: bool,
) -> BTreeMap<NodeId, Result<(), NodeError>> {
    if keep_entry {
        results.get(&dataflow_id).cloned().unwrap_or_default()
    } else {
        results.remove(&dataflow_id).unwrap_or_default()
    }
}

impl Daemon {
    /// Release every extension-table entry owned by `node_id`, notifying the
    /// nodes that read them. Thin wrapper over the free function so the call
    /// sites do not each repeat the `running` lookup — the exception is the
    /// clean-exit site, which holds a `&mut self.logger` borrow and so must
    /// name the disjoint fields itself.
    fn reclaim_extensions(&mut self, dataflow_id: DataflowId, node_id: &NodeId) {
        reclaim_extensions_of_exited_node(
            &mut self.extensions,
            self.running.get(&dataflow_id),
            dataflow_id,
            node_id,
            &self.clock,
        );
    }

    /// Runs the daemon with the given zenoh wiring; see [`ZenohOptions`].
    pub async fn run_with_zenoh_listen(
        coordinator_ws_addr: SocketAddr,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        local_listen_port: u16,
        zenoh: ZenohOptions,
    ) -> eyre::Result<()> {
        Self::run_inner_with_builds(
            coordinator_ws_addr,
            machine_id,
            labels,
            local_listen_port,
            zenoh,
            Default::default(),
        )
        .await
    }

    async fn run_inner_with_builds(
        coordinator_ws_addr: SocketAddr,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        local_listen_port: u16,
        zenoh: ZenohOptions,
        initial_builds: BTreeMap<BuildId, BuildInfo>,
    ) -> eyre::Result<()> {
        let ZenohOptions {
            inter_daemon_peer,
            listen: zenoh_listen,
            connect: zenoh_connect,
            disable_multicast,
        } = zenoh;
        let zenoh_bind = match zenoh_listen {
            Some(listen) => {
                validate_zenoh_listen(listen.addr).wrap_err(
                    "invalid --zenoh-listen address (omit the flag to derive it \
                     from --coordinator-addr)",
                )?;
                ZenohBind::Explicit(listen)
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
        //
        // A *named* port is probed too, and there the bind is not only an
        // existence check: it also reports the port already being in use, which
        // is the likely mistake when two daemons share a host. That much is
        // racy — the probe drops the socket before zenoh binds — but losing the
        // race just defers the same error to the `info().locators()` check,
        // which is fatal for an explicit bind either way.
        if let ZenohBind::Explicit(listen) = zenoh_bind {
            std::net::TcpListener::bind((listen.addr, listen.port.unwrap_or(0)))
                .map(drop)
                .wrap_err_with(|| match listen.port {
                    Some(port) => format!(
                        "cannot bind the zenoh listen address {listen} given via \
                         --zenoh-listen; either the address does not exist on this \
                         host or port {port} is already in use"
                    ),
                    None => format!(
                        "cannot bind the zenoh listen address {listen} given via \
                         --zenoh-listen; it does not appear to exist on this host"
                    ),
                })?;
        }
        announce_zenoh_bind(zenoh_bind, coordinator_ws_addr)?;
        // A *derived* address is reserved lazily, by `register`, once the
        // coordinator socket is up. A reservation binds an ephemeral port and
        // drops the socket, so everything between it and zenoh's own bind is a
        // window in which another process can take that port; reserving here
        // would stretch that window across the whole coordinator connect loop,
        // which retries for many minutes when the coordinator is not up yet.
        //
        // An address the operator *named* is reserved here instead, because
        // failing to reserve it is fatal, and a fatal error raised from inside
        // `register` would surface as a coordinator-connection failure: the
        // daemon would log "waiting for coordinator", reconnect, re-reserve and
        // repeat forever, blaming the network for a local bind problem. Doing
        // it up front costs a wider window for the one case that names an
        // address without a port; a named port reserves nothing at all, so the
        // common explicit form pays nothing. It also means the call inside
        // `register` only ever sees a derived bind, whose failures are
        // non-fatal by construction.
        //
        // Either way the result is cached across reconnects, because the
        // session — and the port it bound — outlives them.
        let mut requested_listen_endpoint: Option<String> = if zenoh_bind.is_explicit() {
            reserve_zenoh_listen_endpoint(zenoh_bind)?
        } else {
            None
        };
        // Only a routable listener is worth advertising — handing `127.0.0.1`
        // to a daemon on another machine would point it at its own loopback,
        // and dialing it would cost that daemon its multicast fallback for
        // nothing. A single-machine deployment therefore advertises nothing and
        // keeps exactly the behavior it has today.
        let advertise_listen_endpoint = !zenoh_bind.addr().is_loopback();
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
                    ZenohRegistration {
                        bind: zenoh_bind,
                        // On a reconnect the session is already open, so the
                        // endpoint it *bound* is the truth — not the one we
                        // reserved, which may be a port we lost.
                        advertise: if !advertise_listen_endpoint {
                            AdvertiseListener::Never
                        } else if let Some(d) = daemon.as_ref() {
                            AdvertiseListener::Bound(d.zenoh_listen_endpoint.clone())
                        } else {
                            AdvertiseListener::Reserved
                        },
                        reserved: requested_listen_endpoint.clone(),
                    },
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
                Ok((
                    daemon_id,
                    reserved_listen_endpoint,
                    peer_zenoh_endpoints,
                    coordinator_sender,
                    incoming_events,
                )) => {
                    // Cache the port `register` reserved so a reconnect reuses
                    // it rather than reserving a second one the session never
                    // bound.
                    requested_listen_endpoint = reserved_listen_endpoint;
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
                                machine_id.clone(),
                                Some(coordinator_sender),
                                daemon_id,
                                None,
                                clock.clone(),
                                Some(remote_daemon_events_tx),
                                initial_builds.clone(),
                                log_destination,
                                inter_daemon_peer.clone(),
                                zenoh_connect.clone(),
                                requested_listen_endpoint.clone(),
                                // Peers the coordinator knows about, dialed in
                                // addition to any named on the command line.
                                // `open_zenoh_session_with_listen` deduplicates,
                                // so an endpoint given both ways costs nothing.
                                //
                                // Only applied on the first connect: this is
                                // where the zenoh session is opened, and zenoh
                                // reads `connect/endpoints` once. A reconnect
                                // reuses the existing session (and its links),
                                // so a later reply's list has nothing to act on.
                                peer_zenoh_endpoints,
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
            None,
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
        machine_id: Option<String>,
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
        let working_dir = dora_core::descriptor::canonicalize_working_dir(
            working_dir_override.as_deref(),
            dataflow_path,
        )?;

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
            machine_id.clone(),
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
        machine_id: Option<String>,
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
            machine_id.clone(),
            coordinator_sender,
            daemon_id,
            exit_when_done,
            clock,
            remote_daemon_events_tx,
            builds,
            log_destination,
            inter_daemon_peer,
            // `dora run` is single-machine: no remote daemon to dial, and no
            // coordinator to have discovered one from.
            Vec::new(),
            reserve_zenoh_listen_endpoint(ZenohBind::Derived(LOCALHOST))?,
            Vec::new(),
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
        machine_id: Option<String>,
        coordinator_sender: Option<coordinator::CoordinatorSender>,
        daemon_id: DaemonId,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        clock: Arc<HLC>,
        remote_daemon_events_tx: Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
        builds: BTreeMap<BuildId, BuildInfo>,
        log_destination: LogDestination,
        inter_daemon_peer: Option<String>,
        zenoh_connect: Vec<String>,
        // The endpoint this daemon will bind, already reserved by the caller.
        requested_listen_endpoint: Option<String>,
        // Peers the coordinator reported at registration. Kept separate from
        // `zenoh_connect` (which the operator named) because a discovered list
        // must not disable multicast scouting — see
        // `ZenohSessionParams::discovered_connect_endpoints`.
        zenoh_discovered_connect: Vec<String>,
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

        // Decide the endpoint zenoh listens on. A named port is used verbatim;
        // otherwise a free one is reserved from the OS. The endpoint is
        // injected into spawned nodes via `DORA_ZENOH_CONNECT` so peer
        // discovery works without multicast (#1778).
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
        // Reserved by the caller, before registering, so the endpoint can
        // travel *with* the registration — see
        // `DaemonRegisterRequest::zenoh_listen_endpoint`.
        // The helper is the source of truth for whether the listener actually
        // bound: `zenoh_listen_endpoint` only becomes `Some` if zenoh accepted
        // the listen/endpoints insert. Otherwise we must not inject
        // `DORA_ZENOH_CONNECT` into spawned nodes — they would try to connect
        // to an endpoint that nothing is listening on (#1856).
        let (zenoh_session, zenoh_listen_endpoint) =
            open_zenoh_session_with_listen(dora_core::topics::ZenohSessionParams {
                listen_endpoint: requested_listen_endpoint.as_deref(),
                inter_daemon_peer: inter_daemon_peer.as_deref(),
                connect_endpoints: &zenoh_connect,
                discovered_connect_endpoints: &zenoh_discovered_connect,
                multicast: if disable_multicast {
                    MulticastScouting::Disabled
                } else {
                    MulticastScouting::Allowed
                },
                ..Default::default()
            })
            .await
            .wrap_err("failed to open zenoh session")?;
        // Same-host control notifications (`PeerMessage::Register`/`PeerMessage::Free`) go over
        // zenoh SHM: the payload stays in shared memory and peer daemons
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

        let daemon = Self {
            machine_id,
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
            // A loopback bind is not an address anyone else can dial, so it is
            // not a routable one; see the field docs.
            zenoh_routable_addr: Some(zenoh_bind.addr()).filter(|addr| !addr.is_loopback()),
            disable_multicast,
            bind_nodes_to_parent,
            pending_destroy: None,
            remote_daemon_events_tx,
            git_manager: Default::default(),
            extensions: ExtensionTable::new(),
            #[cfg(feature = "tensor-pool")]
            pool: dora_tensor_pool::daemon::PoolState::new(),
            builds,
            sessions: Default::default(),
            metrics_state: Arc::new(std::sync::Mutex::new(MetricsState::default())),
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

        // A previous incarnation of this daemon may have been killed without
        // running shutdown cleanup, so give the extension a chance to reclaim
        // whatever it left behind. What that means is its business; the daemon
        // only supplies the machine identity it is scoped to.
        #[cfg(feature = "tensor-pool")]
        dora_tensor_pool::daemon::PoolState::sweep_orphans_at_startup(self.machine_id.as_deref());

        // This function is re-entered on every reconnect, while the dataflows
        // (and their debug-topic watchers) survive across attempts. Watchers
        // registered over a dropped connection otherwise keep producing frames
        // into its dead channel forever; there is no stall on the daemon side
        // either, so the leak is silent (dora-rs/dora#3509). Drop them all here
        // and let the coordinator's `restore_topic_debug_streams_for_daemon`
        // re-install the still-active subscriptions.
        for dataflow in self.running.values_mut() {
            dataflow.debug_topic_watchers.clear();
        }

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

            // Confirm — or withdraw — the endpoint advertised at registration,
            // now that the session is open and the listener has been checked
            // against `info().locators()`.
            //
            // The announcement itself happened in the register request, because
            // it has to be on record before the *next* daemon's reply is built;
            // see `DaemonRegisterRequest::zenoh_listen_endpoint`. This is only
            // the correction, and it matters most in the failure case: a daemon
            // that advertised a port and then lost it to another process must
            // say so, or the coordinator hands that dead endpoint to every
            // daemon that registers afterwards.
            //
            // Skipped entirely when nothing was advertised (`zenoh_routable_addr`
            // is `None` for a loopback bind — a single-machine deployment), since
            // there is then nothing to confirm or withdraw.
            //
            // Re-sent on every reconnect, so a coordinator that restarted and
            // lost the registry relearns this daemon's endpoint.
            if self.zenoh_routable_addr.is_some() {
                let stamped = Timestamped {
                    inner: CoordinatorRequest::Event {
                        daemon_id: self.daemon_id.clone(),
                        event: DaemonEvent::ZenohListenEndpoint {
                            endpoint: self.zenoh_listen_endpoint.clone(),
                        },
                    },
                    timestamp: self.clock.new_timestamp(),
                };
                if let Ok(bytes) = serde_json::to_vec(&stamped)
                    && let Err(err) = sender.send_event(&bytes).await
                {
                    tracing::warn!("failed to report zenoh listen endpoint to coordinator: {err}");
                }
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
                    // Accept only this entry's own lineage: its current
                    // generation, or the successor its OWN restart loop
                    // announced into `generation_counter` before spawning it.
                    // The successor exception lets a fast successor's Subscribe
                    // through while the entry still holds the predecessor's
                    // generation (before its `ProcessHandleReplaced` lands),
                    // without which its one-shot Subscribe is lost and the
                    // incarnation stays disconnected. The lineage restriction
                    // rejects a stranger's HIGHER generation — e.g. a
                    // concurrent restart of a node being replaced mints a
                    // generation above the replacement's — instead of
                    // misapplying its zombie's events to the replacement
                    // (dora-rs/dora#2997). Entry-absent passes through:
                    // during startup a node can register before its
                    // RunningNode is inserted, and the pending-nodes
                    // barrier owns that window.
                    let superseded = self
                        .running
                        .get(&dataflow)
                        .and_then(|df| df.running_nodes.get(&node_id))
                        .is_some_and(|node| {
                            event_generation_is_stale(
                                node.generation,
                                node.generation_counter.load(atomic::Ordering::Acquire),
                                generation,
                            )
                        });
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
                            startup_kills = self
                                .ft_stats
                                .startup_timeout_kills
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

        // Clean up any unfreed cross-machine pool bookkeeping on daemon
        // exit. Mirror segments are deliberately NOT unlinked here: they
        // may still be open by the peer daemon's readers on a shared host,
        // and `cleanup_orphan_mirrors` sweeps this machine's own leftovers
        // on the next startup.
        #[cfg(feature = "tensor-pool")]
        self.pool.cleanup_all();

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
                    // `node_id` is still needed below for the error logger, so
                    // clone it into the key; `output_id` is moved.
                    let output_id_key = OutputId(node_id.clone(), output_id);
                    send_output_to_local_receivers(
                        &output_id_key,
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
                        close_inputs_best_effort(
                            dataflow,
                            inputs,
                            &self.clock,
                            "failed to handle OutputClosed for input",
                        );
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
            InterDaemonEvent::ExtensionMessage {
                dataflow_id,
                namespace,
                target_machine,
                payload,
            } => {
                #[cfg(feature = "tensor-pool")]
                {
                    self.handle_extension_message(dataflow_id, namespace, target_machine, payload)
                        .await
                }
                // No extension is compiled in, so nothing can service the
                // payload. A peer that runs one is not an error here: this
                // daemon simply takes no part in it.
                #[cfg(not(feature = "tensor-pool"))]
                {
                    let _ = (dataflow_id, target_machine, payload);
                    tracing::debug!(
                        "ignoring inter-daemon message for extension {namespace:?}: \
                         no extension is compiled into this daemon"
                    );
                    Ok(())
                }
            }
            // `InterDaemonEvent` is `#[non_exhaustive]`: a peer daemon running a
            // newer dora may send an event this one predates. Warn and continue —
            // tearing down the dataflow over an unknown peer message would be worse.
            other => {
                tracing::warn!("ignoring unrecognized inter-daemon event: {other:?}");
                Ok(())
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
    // Reservation happens inside `register`, once the coordinator socket is up.
    zenoh: ZenohRegistration,
    clock: &Arc<HLC>,
    remote_daemon_events_rx: flume::Receiver<eyre::Result<Timestamped<InterDaemonEvent>>>,
    // Events from the dynamic-node listener. The listener is bound once by the
    // caller (for the daemon's lifetime) and its receiver passed in, rather than
    // rebinding the port on every reconnect (dora-rs/dora#1999).
    dynamic_node_events_rx: flume::Receiver<Timestamped<DynamicNodeEventWrapper>>,
) -> eyre::Result<(
    DaemonId,
    Option<String>,
    Vec<String>,
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
    let (
        daemon_id,
        reserved_listen_endpoint,
        peer_zenoh_endpoints,
        coordinator_sender,
        coordinator_events,
    ) = coordinator::register(
        coordinator_ws_addr,
        machine_id.clone(),
        labels,
        zenoh,
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
    Ok((
        daemon_id,
        reserved_listen_endpoint,
        peer_zenoh_endpoints,
        coordinator_sender,
        incoming,
    ))
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
                    text.truncate(text.floor_char_boundary(MAX_SAMPLE_BYTES));
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
            CoreNodeKind::Runtime(n) => {
                // A runtime node's I/O is the union of its operators', and the
                // remaining keys (type annotations, framing, pool size) have no
                // runtime-node surface — they stay at their defaults, which is
                // also the right answer for any I/O key added later.
                let mut run_config = NodeRunConfig::default();
                run_config.inputs = runtime_node_inputs(n);
                run_config.outputs = runtime_node_outputs(n);
                run_config
            }
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
mod node_results_cleanup_tests {
    use super::{DaemonRunResult, NodeId, extract_node_results};
    use std::collections::BTreeMap;
    use uuid::Uuid;

    fn populated() -> (Uuid, DaemonRunResult) {
        let id = Uuid::new_v4();
        let mut inner = BTreeMap::new();
        inner.insert(NodeId::from("node".to_string()), Ok(()));
        let mut map = BTreeMap::new();
        map.insert(id, inner);
        (id, map)
    }

    #[test]
    fn persistent_daemon_removes_the_entry() {
        let (id, mut map) = populated();
        let results = extract_node_results(&mut map, id, false);
        assert_eq!(
            results.len(),
            1,
            "the reported results must still be returned"
        );
        assert!(
            !map.contains_key(&id),
            "a coordinator-managed daemon must drop the entry to avoid unbounded growth"
        );
    }

    #[test]
    fn run_once_daemon_keeps_the_entry() {
        let (id, mut map) = populated();
        let results = extract_node_results(&mut map, id, true);
        assert_eq!(results.len(), 1);
        assert!(
            map.contains_key(&id),
            "the run-once path relies on run_inner's later mem::take, so the entry must survive"
        );
    }

    #[test]
    fn missing_entry_yields_empty_results() {
        let mut map: BTreeMap<Uuid, BTreeMap<NodeId, Result<(), super::NodeError>>> =
            BTreeMap::new();
        assert!(extract_node_results(&mut map, Uuid::new_v4(), false).is_empty());
        assert!(extract_node_results(&mut map, Uuid::new_v4(), true).is_empty());
    }
}
