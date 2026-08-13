//! Running dataflow state and associated types.

use crate::{
    DoraEvent, OutputId, coordinator, fault_tolerance::CascadingErrorCauses, pending::PendingNodes,
    send_with_timestamp,
};
use dora_core::{
    config::{DataId, NodeId},
    descriptor::Descriptor,
    uhlc::HLC,
};
use dora_message::{
    common::DaemonId,
    daemon_to_node::{NodeConfig, NodeEvent},
    descriptor::RestartPolicy,
    metadata::{self},
    node_to_daemon::Timestamped,
};
/// Default grace period before force-killing a stopped node. A stopped node is
/// hard-killed at `DEFAULT_STOP_GRACE + DEFAULT_STOP_GRACE/2` (= 15s), so a node's own
/// zenoh teardown deadline must stay under that or it gets force-killed mid-teardown
/// (`ExitCode(1)` on Windows — dora-rs/dora#2742). Kept in sync by
/// `ZENOH_TEARDOWN_TIMEOUT` in `apis/rust/node/src/node/mod.rs` and its
/// `zenoh_teardown_fits_within_daemon_force_kill_grace` guard test.
const DEFAULT_STOP_GRACE: Duration = Duration::from_millis(10_000);
/// Default grace period before force-killing a restarting node.
const DEFAULT_RESTART_GRACE: Duration = Duration::from_millis(5_000);

use crossbeam::queue::ArrayQueue;
use eyre::eyre;
use futures::FutureExt;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    sync::{
        Arc,
        atomic::{self, AtomicBool, AtomicU32, AtomicU64},
    },
    time::{Duration, Instant},
};
use tokio::sync::{
    broadcast,
    mpsc::{self, Sender},
    oneshot,
};
use tracing::warn;

#[cfg(feature = "telemetry")]
use tracing_opentelemetry::OpenTelemetrySpanExt;

use crate::Event;
use process_wrap::tokio::ChildWrapper;

pub(crate) struct InputDeadline {
    pub timeout: Duration,
    /// Wall-clock time of the most recent message delivery.
    ///
    /// `None` means the input has never received a message and the
    /// circuit-breaker clock should NOT count against it yet. This
    /// prevents false-positive timeouts on on-demand inputs that are
    /// legitimately idle at dataflow startup — most notably service
    /// response inputs on clients that haven't sent a request yet
    /// (dora-rs/adora#149).
    pub last_received: Option<Instant>,
}

impl InputDeadline {
    /// Returns `true` if the circuit breaker should fire for this input.
    ///
    /// An input is only considered timed out once it has received at
    /// least one message (`last_received = Some(_)`) and the elapsed
    /// time since that message exceeds the configured timeout. An
    /// input that has never received a message is never timed out
    /// — its clock is unarmed (dora-rs/adora#149).
    pub fn is_timed_out(&self) -> bool {
        self.last_received
            .map(|t| t.elapsed() > self.timeout)
            .unwrap_or(false)
    }
}

#[derive(Debug)]
pub struct RunningNode {
    pub(crate) process: Option<ProcessHandle>,
    /// Gates this node's `restart_loop` until the daemon has inserted the
    /// entry into `running_nodes` (`mark_registered`), so the loop's first
    /// events cannot race ahead of registration. Dropping the sender without
    /// firing it (entry never registered) cancels the loop.
    pub(crate) restart_loop_start: Option<oneshot::Sender<()>>,
    /// Keeps the node's TCP listener alive; when this entry (and the
    /// restart loop's clone) drops, the listener stops accepting — so a
    /// replaced or removed node does not leak its listener until the whole
    /// dataflow finishes (dora-rs/dora#2988 review, finding 3).
    pub(crate) _listener_shutdown: Option<tokio::sync::watch::Sender<bool>>,
    /// Monotonic identity of the process incarnation currently registered
    /// under this node ID. Lifecycle events from older incarnations must not
    /// mutate this entry or contribute results to it.
    pub(crate) generation: u64,
    pub(crate) node_config: NodeConfig,
    pub(crate) pid: Option<Arc<AtomicU32>>,
    pub(crate) restart_count: Arc<AtomicU32>,
    pub(crate) restart_policy: RestartPolicy,
    pub(crate) disable_restart: Arc<AtomicBool>,
    /// One-shot flag set by `restart_single_node` (operator-requested
    /// `dora node restart`). When `true`, the restart loop forces a
    /// respawn on the next exit **regardless of `restart_policy`**, then
    /// clears the flag. Without this, `dora node restart` is a no-op on
    /// nodes with the default `restart_policy: Never` — contradicting
    /// the CLI help ("the daemon's restart loop re-spawns it").
    pub(crate) force_restart_next: Arc<AtomicBool>,
    pub(crate) last_activity: Arc<AtomicU64>,
    pub(crate) health_check_timeout: Option<Duration>,
    /// Per-node finish-drain grace override (from `finish_grace_secs` in the
    /// descriptor). When `Some`, overrides the global `DORA_FINISH_DRAIN_GRACE_SECS`
    /// for this node in the finish-straggler watchdog.
    pub(crate) finish_grace_secs: Option<Duration>,
}

impl RunningNode {
    pub(crate) fn mark_registered(&mut self) {
        if let Some(start) = self.restart_loop_start.take() {
            let _ = start.send(());
        }
    }

    pub(crate) fn matches_generation(&self, generation: u64) -> bool {
        self.generation == generation
    }

    /// Install a respawned incarnation's process handle.
    ///
    /// A stale replacement is dropped here, which kills that orphan process.
    /// A replacement rejected during teardown is returned to the caller so it
    /// can follow the dataflow's configured stop policy instead of being
    /// hard-killed by [`ProcessHandle::drop`].
    pub(crate) fn replace_process_handle(
        &mut self,
        previous_generation: u64,
        new_generation: u64,
        new_handle: ProcessHandle,
    ) -> HandleReplacement {
        if !self.matches_generation(previous_generation) {
            // A dead incarnation must not replace a live successor's handle
            // (dora-rs/dora#2926). Dropping `new_handle` kills the orphan.
            return HandleReplacement::RejectedStale;
        }
        if self.restarts_disabled() {
            // Teardown won the race. Still advance the generation: the restart
            // loop already speaks `new_generation`, and its terminal
            // `SpawnedNodeResult` must match this entry or the node would stay
            // registered forever and the dataflow could never finish. Return
            // the handle so RunningDataflow can apply the active stop policy.
            self.generation = new_generation;
            return HandleReplacement::RejectedTeardown(new_handle);
        }

        self.process = Some(new_handle);
        self.generation = new_generation;
        HandleReplacement::Replaced
    }

    pub fn restarts_disabled(&self) -> bool {
        self.disable_restart.load(atomic::Ordering::Acquire)
    }

    pub fn disable_restart(&mut self) {
        self.disable_restart.store(true, atomic::Ordering::Release);
    }
}

/// Outcome of [`RunningNode::replace_process_handle`].
#[derive(Debug)]
pub(crate) enum HandleReplacement {
    /// The handle was installed and the entry advanced to the new generation.
    Replaced,
    /// Teardown disabled restarts first. The entry's generation was advanced
    /// so the restart loop's terminal exit event is accepted, and the handle
    /// must be stopped according to the active dataflow stop policy.
    RejectedTeardown(ProcessHandle),
    /// The event belongs to a dead incarnation (generation mismatch); the
    /// entry was left untouched.
    RejectedStale,
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum StopProcessPolicy {
    Force,
    Graceful(Duration),
}

static NEXT_NODE_GENERATION: AtomicU64 = AtomicU64::new(1);

/// Generations are unique across the whole daemon process — not per node —
/// so a re-added node ID (or any freshly built `RunningNode`) can never
/// reuse a predecessor's generation. Equality against the entry's stored
/// generation is therefore collision-free proof that an event belongs to
/// the currently registered incarnation.
pub(crate) fn next_node_generation() -> u64 {
    NEXT_NODE_GENERATION
        .fetch_update(
            atomic::Ordering::Relaxed,
            atomic::Ordering::Relaxed,
            |generation| generation.checked_add(1),
        )
        .expect("node generation counter exhausted")
}

#[derive(Debug)]
pub(crate) enum ProcessOperation {
    SoftKill,
    Kill,
}

impl ProcessOperation {
    pub fn execute(&self, child: &mut dyn ChildWrapper) {
        match self {
            Self::SoftKill => {
                #[cfg(unix)]
                {
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
pub(crate) struct ProcessHandle {
    pub op_tx: flume::Sender<ProcessOperation>,
}

impl ProcessHandle {
    pub fn new(op_tx: flume::Sender<ProcessOperation>) -> Self {
        Self { op_tx }
    }

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

/// A subscriber to the `dora/logs` virtual input.
pub struct LogSubscriber {
    pub node_id: NodeId,
    pub input_id: DataId,
    pub filter: dora_message::config::LogSubscriptionFilter,
}

pub struct RunningDataflow {
    pub(crate) id: uuid::Uuid,
    pub(crate) descriptor: Descriptor,
    /// Per-node zenoh listener + dial-list, so the node↔node links this dataflow
    /// needs are established deterministically rather than left to gossip.
    /// Populated when the dataflow is spawned; see `plan_zenoh_peering`.
    pub(crate) zenoh_peering: Arc<BTreeMap<NodeId, crate::spawn::NodeZenohPeering>>,
    pub(crate) pending_nodes: PendingNodes,
    pub(crate) dataflow_started: bool,
    pub(crate) subscribe_channels: HashMap<NodeId, Sender<Timestamped<NodeEvent>>>,
    /// Per-node pending message counters (incremented on send, decremented on recv)
    pub(crate) pending_messages: HashMap<NodeId, Arc<AtomicU64>>,
    pub(crate) mappings: HashMap<OutputId, BTreeSet<(NodeId, DataId)>>,
    pub(crate) timers: BTreeMap<Duration, BTreeSet<(NodeId, DataId)>>,
    /// Nodes subscribing to `dora/logs` virtual input.
    pub(crate) log_subscribers: Vec<LogSubscriber>,
    pub(crate) open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    pub(crate) input_deadlines: HashMap<(NodeId, DataId), InputDeadline>,
    pub(crate) broken_inputs: HashMap<(NodeId, DataId), Duration>,
    pub(crate) running_nodes: BTreeMap<NodeId, RunningNode>,
    pub(crate) dynamic_nodes: BTreeSet<NodeId>,
    pub(crate) open_external_mappings: BTreeSet<OutputId>,
    pub(crate) _timer_handles: BTreeMap<Duration, futures::future::RemoteHandle<()>>,
    /// When the daemon sent `AllInputsClosed` to each node — the start of
    /// that node's drain phase. Drives the finish-straggler watchdog
    /// (dora-rs/dora#2152).
    pub(crate) all_inputs_closed_at: HashMap<NodeId, Instant>,
    /// Nodes that have subscribed at least once. Distinguishes a node that has
    /// connected (and may since have dropped its event stream) from one still
    /// starting up — only the former is a finish-straggler candidate. Set on
    /// subscribe, cleared on node removal so a re-added node ID starts a fresh
    /// incarnation rather than looking already-connected (dora-rs/dora#2270).
    pub(crate) connected_nodes: BTreeSet<NodeId>,
    /// Nodes already escalated by the finish-straggler watchdog (one-shot).
    pub(crate) finish_escalated: BTreeSet<NodeId>,
    /// Per node, the inputs that can ever close — i.e. those fed by
    /// another node's output rather than by the daemon.
    ///
    /// Recorded where inputs are registered, so it sees the same
    /// resolved view as `open_inputs` (notably `node_inputs`, which
    /// flattens a runtime node's `operators[].config.inputs`). Timer and
    /// Logs inputs are excluded: they have no upstream node and so never
    /// close. Used by [`RunningDataflow::is_drained`] (#2920).
    pub(crate) data_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    /// Whether timer inputs keep a node from draining.
    ///
    /// A timer input is registered in `open_inputs` like any other but is
    /// never closed — there is no upstream node to finish — so a node
    /// consuming `dora/timer/...` can never reach "all inputs closed"
    /// and is never told to finish. Any timer anywhere therefore makes a
    /// graph unable to terminate on its own (dora-rs/dora#2920).
    ///
    /// `dora run --exit-when-nodes-finish` sets this false, which treats
    /// a timer as a clock rather than a data dependency: a node drains
    /// once its DATA inputs have closed. Off by default because it
    /// changes when nodes are told to stop.
    pub(crate) timers_gate_drain: bool,
    pub(crate) stop_sent: bool,
    /// Resolved process-stop policy for this dataflow. A respawn that races
    /// after `stop_all` moved the previously-known handles out of
    /// `running_nodes` must use this same policy.
    pub(crate) stop_process_policy: Option<StopProcessPolicy>,
    pub(crate) empty_set: BTreeSet<DataId>,
    pub(crate) cascading_error_causes: CascadingErrorCauses,
    /// Planned-stop kill markers keyed by `(node id, generation)`: scoped to
    /// one process incarnation so a re-added successor never inherits its
    /// predecessor's marker, and cleaning up a stale event cannot clear a
    /// live incarnation's marker.
    pub(crate) grace_duration_kills: Arc<crossbeam_skiplist::SkipSet<(NodeId, u64)>>,
    pub(crate) node_stderr_most_recent: BTreeMap<NodeId, Arc<ArrayQueue<String>>>,
    pub(crate) publishers: BTreeMap<OutputId, Arc<zenoh::pubsub::Publisher<'static>>>,
    /// Reverse index from output to the set of CLI subscribers watching it.
    /// Hot-path read on every node output dispatch (`send_topic_debug_frames`)
    /// and on the `has_debug_watchers` check. Unsubscribe scans this map
    /// rather than maintaining a separate inverse map: unsubscribe is rare
    /// compared to dispatch, and the scan is bounded by the count of outputs
    /// that currently have at least one subscriber.
    pub(crate) debug_topic_watchers: BTreeMap<OutputId, BTreeSet<uuid::Uuid>>,
    pub(crate) finished_tx: broadcast::Sender<()>,
    /// Shutdown signal for listener loops — send `true` when dataflow finishes.
    pub(crate) listener_shutdown_tx: tokio::sync::watch::Sender<bool>,
    pub(crate) listener_shutdown_rx: tokio::sync::watch::Receiver<bool>,
    pub(crate) enable_debug_inspection: bool,
    /// Cross-daemon Zenoh network counters
    pub(crate) net_bytes_sent: Arc<AtomicU64>,
    pub(crate) net_bytes_received: Arc<AtomicU64>,
    pub(crate) net_messages_sent: Arc<AtomicU64>,
    pub(crate) net_messages_received: Arc<AtomicU64>,
    pub(crate) net_publish_failures: Arc<AtomicU64>,
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
    pub(crate) fn new(
        dataflow_id: uuid::Uuid,
        daemon_id: DaemonId,
        dataflow_descriptor: Descriptor,
    ) -> RunningDataflow {
        let (finished_tx, _) = broadcast::channel(1);
        let (listener_shutdown_tx, listener_shutdown_rx) = tokio::sync::watch::channel(false);
        Self {
            id: dataflow_id,
            zenoh_peering: Arc::new(BTreeMap::new()),
            pending_nodes: PendingNodes::new(dataflow_id, daemon_id),
            dataflow_started: false,
            subscribe_channels: HashMap::new(),
            pending_messages: HashMap::new(),
            mappings: HashMap::new(),
            timers: BTreeMap::new(),
            log_subscribers: Vec::new(),
            open_inputs: BTreeMap::new(),
            input_deadlines: HashMap::new(),
            broken_inputs: HashMap::new(),
            running_nodes: BTreeMap::new(),
            dynamic_nodes: BTreeSet::new(),
            open_external_mappings: Default::default(),
            _timer_handles: BTreeMap::new(),
            all_inputs_closed_at: HashMap::new(),
            connected_nodes: BTreeSet::new(),
            finish_escalated: BTreeSet::new(),
            data_inputs: BTreeMap::new(),
            timers_gate_drain: true,
            stop_sent: false,
            stop_process_policy: None,
            empty_set: BTreeSet::new(),
            cascading_error_causes: Default::default(),
            grace_duration_kills: Default::default(),
            node_stderr_most_recent: BTreeMap::new(),
            publishers: Default::default(),
            debug_topic_watchers: Default::default(),
            finished_tx,
            listener_shutdown_tx,
            listener_shutdown_rx,
            enable_debug_inspection: dataflow_descriptor.debug.enable_debug_inspection,
            descriptor: dataflow_descriptor,
            net_bytes_sent: Default::default(),
            net_bytes_received: Default::default(),
            net_messages_sent: Default::default(),
            net_messages_received: Default::default(),
            net_publish_failures: Default::default(),
        }
    }

    /// Drop per-node bookkeeping that is keyed by node id but not covered by
    /// the routing-table cleanup in the `RemoveNode` handler.
    ///
    /// Without this, removing a node (dynamic reconfiguration) leaves stale
    /// `input_deadlines` / `broken_inputs` entries behind. A never-armed
    /// `input_deadlines` entry never times out, so `check_input_timeouts`
    /// re-scans it every tick forever; a `broken_inputs` entry can never
    /// recover once the node is gone (recovery only happens on message
    /// receipt, which a removed node never sees). Together with the
    /// `node_stderr_most_recent` queue this is an unbounded accumulation
    /// across repeated add/remove cycles.
    pub(crate) fn forget_node_bookkeeping(&mut self, node_id: &NodeId) {
        self.input_deadlines.retain(|(n, _), _| n != node_id);
        self.broken_inputs.retain(|(n, _), _| n != node_id);
        self.node_stderr_most_recent.remove(node_id);
    }

    pub(crate) async fn start(
        &mut self,
        events_tx: &mpsc::Sender<Timestamped<Event>>,
        clock: &Arc<HLC>,
    ) -> eyre::Result<()> {
        for interval in self.timers.keys().copied() {
            if self._timer_handles.contains_key(&interval) {
                continue;
            }
            let events_tx = events_tx.clone();
            let dataflow_id = self.id;
            let clock = clock.clone();
            let task = async move {
                let mut interval_stream = tokio::time::interval(interval);
                interval_stream.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
                loop {
                    interval_stream.tick().await;

                    let span = tracing::span!(tracing::Level::TRACE, "tick");
                    let _ = span.enter();

                    // Build metadata with minimal allocations.
                    // Use shared daemon clock (not per-timer HLC) for causality.
                    #[cfg(feature = "telemetry")]
                    let parameters = {
                        let ctx = dora_tracing::telemetry::serialize_context(&span.context());
                        if ctx.is_empty() {
                            BTreeMap::new()
                        } else {
                            let mut m = BTreeMap::new();
                            m.insert(
                                "open_telemetry_context".to_string(),
                                dora_node_api::Parameter::String(ctx),
                            );
                            m
                        }
                    };
                    #[cfg(not(feature = "telemetry"))]
                    let parameters = BTreeMap::new();

                    let metadata =
                        metadata::Metadata::from_parameters(clock.new_timestamp(), parameters);

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

        // Record that the dataflow has been started. This must hold on *every*
        // start path — the single-daemon `Subscribe`-readiness path and the
        // distributed coordinator `AllNodesReady` path both funnel through
        // here — so the flag lives in `start()` rather than at the call sites.
        // The `AddNode` handler relies on it to tell "already running, spawn a
        // task for a freshly-added interval" apart from "still bringing up, let
        // the readiness path spawn the tasks". Setting it at only one call site
        // (as before) left it `false` for the entire life of a distributed
        // dataflow, silently starving timer inputs on nodes added later.
        self.dataflow_started = true;

        Ok(())
    }

    /// Drop `node_id` from every timer subscriber set, and for any interval it
    /// was the *last* subscriber of, cancel that interval's timer task and
    /// forget the now-empty entry.
    ///
    /// Without the cancellation, the per-interval task spawned by [`start`]
    /// keeps ticking and dispatching `DoraEvent::Timer`s to an empty subscriber
    /// set for the remaining life of the dataflow — a bounded but pointless
    /// stream of wakeups after the last consumer of that interval is removed
    /// via `RemoveNode` (#2585). Dropping the [`RemoteHandle`] stored in
    /// `_timer_handles` cancels the spawned future. A later `AddNode` that
    /// re-subscribes to the same interval re-spawns the task through `start`,
    /// whose `_timer_handles.contains_key` guard makes that safe.
    ///
    /// [`start`]: RunningDataflow::start
    /// [`RemoteHandle`]: futures::future::RemoteHandle
    pub(crate) fn unsubscribe_node_from_timers(&mut self, node_id: &NodeId) {
        let mut drained_intervals = Vec::new();
        for (interval, receivers) in self.timers.iter_mut() {
            receivers.retain(|(nid, _)| nid != node_id);
            if receivers.is_empty() {
                drained_intervals.push(*interval);
            }
        }
        for interval in drained_intervals {
            self.timers.remove(&interval);
            // Dropping the RemoteHandle cancels the spawned timer task.
            self._timer_handles.remove(&interval);
        }
    }

    pub(crate) async fn stop_all(
        &mut self,
        coordinator_sender: &mut Option<coordinator::CoordinatorSender>,
        clock: &HLC,
        grace_duration: Option<Duration>,
        force: bool,
        logger: &mut crate::log::DataflowLogger<'_>,
    ) -> eyre::Result<FinishDataflowWhen> {
        self.pending_nodes
            .handle_dataflow_stop(
                coordinator_sender,
                clock,
                &mut self.cascading_error_causes,
                &self.dynamic_nodes,
                logger,
            )
            .await?;

        for node in self.running_nodes.values_mut() {
            node.disable_restart();
        }

        let stop_process_policy = if force {
            StopProcessPolicy::Force
        } else {
            StopProcessPolicy::Graceful(grace_duration.unwrap_or(DEFAULT_STOP_GRACE))
        };
        self.stop_process_policy = Some(stop_process_policy);

        for (node_id, channel) in self.subscribe_channels.drain() {
            if send_with_timestamp(&channel, NodeEvent::Stop, clock).ok() == Some(true)
                && let Some(counter) = self.pending_messages.get(&node_id)
            {
                counter.fetch_add(1, atomic::Ordering::Relaxed);
            }
        }

        let running_processes: Vec<_> = self
            .running_nodes
            .iter_mut()
            .map(|(id, n)| (id.clone(), n.generation, n.process.take()))
            .collect();
        for (node_id, generation, process) in running_processes {
            if let Some(process) = process {
                self.schedule_process_stop(node_id, generation, process, stop_process_policy);
            }
        }
        self.stop_sent = true;

        Ok(self.should_finish_immediately())
    }

    fn schedule_process_stop(
        &self,
        node_id: NodeId,
        generation: u64,
        process: ProcessHandle,
        policy: StopProcessPolicy,
    ) {
        let grace_duration_kills = self.grace_duration_kills.clone();
        match policy {
            StopProcessPolicy::Force => {
                if process.submit(ProcessOperation::Kill) {
                    grace_duration_kills.insert((node_id, generation));
                }
            }
            StopProcessPolicy::Graceful(duration) => {
                tokio::spawn(async move {
                    tokio::time::sleep(duration).await;
                    if process.submit(ProcessOperation::SoftKill) {
                        grace_duration_kills.insert((node_id.clone(), generation));
                    }

                    let kill_duration = duration / 2;
                    tokio::time::sleep(kill_duration).await;
                    if process.submit(ProcessOperation::Kill) {
                        grace_duration_kills.insert((node_id.clone(), generation));
                        warn!(
                            "{node_id} was killed due to not stopping within the {:#?} grace period",
                            duration + kill_duration
                        );
                    }
                });
            }
        }
    }

    pub(crate) fn stop_rejected_replacement(
        &self,
        node_id: &NodeId,
        generation: u64,
        process: ProcessHandle,
    ) {
        if let Some(policy) = self.stop_process_policy {
            self.schedule_process_stop(node_id.clone(), generation, process, policy);
        } else {
            // Restarts can also be disabled by AllInputsClosed. That path has
            // no operator-supplied grace policy, so retain its existing
            // immediate orphan-reaping behavior.
            drop(process);
        }
    }

    fn should_finish_immediately(&self) -> FinishDataflowWhen {
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

    /// Stop a single node. Sets `disable_restart` so it won't auto-restart.
    pub(crate) fn stop_single_node(
        &mut self,
        node_id: &NodeId,
        clock: &HLC,
        grace_duration: Option<Duration>,
    ) -> eyre::Result<()> {
        let node = self
            .running_nodes
            .get_mut(node_id)
            .ok_or_else(|| eyre!("node `{node_id}` not found in running dataflow"))?;
        node.disable_restart();
        let generation = node.generation;
        let process = node.process.take();
        self.send_stop_and_schedule_kill(
            node_id,
            generation,
            process,
            clock,
            grace_duration,
            DEFAULT_STOP_GRACE,
        );
        Ok(())
    }

    /// Stop an already-unregistered (replaced) incarnation
    /// (dora-rs/dora#2927): send `Stop` on the id's still-installed
    /// subscribe channel and schedule the grace-kill escalation on the
    /// taken process handle. Must be called BEFORE the old incarnation's
    /// subscribe channel is removed from `subscribe_channels`, or the
    /// `Stop` cannot reach it and only the kill escalation applies.
    pub(crate) fn stop_replaced_incarnation(
        &self,
        node_id: &NodeId,
        generation: u64,
        process: Option<ProcessHandle>,
        clock: &HLC,
        grace_duration: Option<Duration>,
    ) {
        self.send_stop_and_schedule_kill(
            node_id,
            generation,
            process,
            clock,
            grace_duration,
            DEFAULT_STOP_GRACE,
        );
    }

    /// Restart a single node. Re-enables restart so `restart_loop` picks it up.
    pub(crate) fn restart_single_node(
        &mut self,
        node_id: &NodeId,
        clock: &HLC,
        grace_duration: Option<Duration>,
    ) -> eyre::Result<()> {
        let node = self
            .running_nodes
            .get_mut(node_id)
            .ok_or_else(|| eyre!("node `{node_id}` not found in running dataflow"))?;
        // Reject the restart if the process slot is empty. `process ==
        // None` means the previous incarnation has exited and either:
        //   (a) `restart_loop` is between exit and respawn (transient),
        //       in which case a second restart would arm
        //       `force_restart_next` a second time and the leftover
        //       `true` would spuriously force a restart on the next
        //       natural exit — poisoning a future incarnation.
        //   (b) `restart_loop` has already exited for good (`restart=
        //       false` branch at spawn/prepared.rs:396), in which case
        //       no consumer for `force_restart_next` remains. Setting
        //       the flag here would leak silently: the CLI would see
        //       success but the node would stay down.
        // Both (a) and (b) are silent-failure paths; reject loudly.
        if node.process.is_none() {
            return Err(eyre!(
                "node `{node_id}` is not in a restartable state (process \
                 slot is empty; the node is between restarts or has \
                 already exited terminally)"
            ));
        }
        let generation = node.generation;
        let process = node.process.take();
        // Clear any prior disable (e.g. from an earlier stop_single_node
        // or a cascading AllInputsClosed) so the restart_loop will pick
        // up the next exit and spawn a replacement.
        //
        // With the per-incarnation channel pair introduced for
        // dora-rs/adora#152, the old grace-kill task sends to a closed
        // op_rx and cannot reach the replacement process, so ordering
        // between this store and the grace-kill submission no longer
        // matters.
        node.disable_restart.store(false, atomic::Ordering::Release);
        // Arm the one-shot force-restart flag so the restart loop
        // bypasses `restart_policy` for this single incarnation. Without
        // this a node with the default `restart_policy: Never` would
        // exit on SIGTERM and never come back, violating the CLI help's
        // promise that `dora node restart` "re-spawns it".
        node.force_restart_next
            .store(true, atomic::Ordering::Release);
        // A fresh incarnation starts with a clean drain state: a stale
        // AllInputsClosed timestamp from the previous incarnation must not
        // trip the finish-straggler watchdog on the restarted node. The
        // connected marker is cleared too, so the restarting process is not
        // treated as connected (and silence-escalatable) before it re-subscribes
        // (dora-rs/dora#2270).
        self.all_inputs_closed_at.remove(node_id);
        self.connected_nodes.remove(node_id);
        self.finish_escalated.remove(node_id);
        self.send_stop_and_schedule_kill(
            node_id,
            generation,
            process,
            clock,
            grace_duration,
            DEFAULT_RESTART_GRACE,
        );
        Ok(())
    }

    /// Send a Stop event to a node and schedule a grace-period kill.
    fn send_stop_and_schedule_kill(
        &self,
        node_id: &NodeId,
        generation: u64,
        process: Option<ProcessHandle>,
        clock: &HLC,
        grace_duration: Option<Duration>,
        default_grace: Duration,
    ) {
        if let Some(channel) = self.subscribe_channels.get(node_id)
            && send_with_timestamp(channel, NodeEvent::Stop, clock).ok() == Some(true)
            && let Some(counter) = self.pending_messages.get(node_id)
        {
            counter.fetch_add(1, atomic::Ordering::Relaxed);
        }

        if let Some(proc) = process {
            let duration = grace_duration.unwrap_or(default_grace);
            // Mirror `stop_all`'s population of `grace_duration_kills`
            // so SpawnedNodeResult can distinguish "daemon explicitly
            // sent SIGTERM to this node" from "node received SIGTERM
            // from somewhere else". Without this marker, a source node
            // (which has `disable_restart` set at subscribe time, see
            // lib.rs:3203) cannot be told apart from an externally
            // killed source node when classifying the exit status
            // (dora-rs/dora#1882).
            let grace_duration_kills = self.grace_duration_kills.clone();
            let node_id = node_id.clone();
            tokio::spawn(async move {
                tokio::time::sleep(duration).await;
                if proc.submit(ProcessOperation::SoftKill) {
                    grace_duration_kills.insert((node_id.clone(), generation));
                }
                tokio::time::sleep(duration / 2).await;
                if proc.submit(ProcessOperation::Kill) {
                    grace_duration_kills.insert((node_id, generation));
                }
            });
        }
    }

    /// Increment the pending message counter for a node after a successful send.
    pub(crate) fn inc_pending(&self, node_id: &NodeId) {
        if let Some(counter) = self.pending_messages.get(node_id) {
            counter.fetch_add(1, atomic::Ordering::Relaxed);
        }
    }

    /// Propagate a `NodeFailed` event to every downstream subscriber of the
    /// failed node's outputs.
    ///
    /// Each successful enqueue is paired with `inc_pending`, mirroring every
    /// other delivery site: the `Listener` unconditionally decrements a node's
    /// `pending_messages` counter for every event it drains (`NodeFailed`
    /// included), so an enqueue without the matching increment would leave the
    /// reported count one too low — and underflow it to `u64::MAX` when the
    /// receiver's channel was already empty (dora-rs/dora#2827).
    pub(crate) fn propagate_node_failed(&self, failed_node: &NodeId, error_msg: &str, clock: &HLC) {
        let mut affected_by_receiver: BTreeMap<NodeId, Vec<DataId>> = BTreeMap::new();
        for (output_id, receivers) in &self.mappings {
            if output_id.0 == *failed_node {
                for (recv_id, input_id) in receivers {
                    affected_by_receiver
                        .entry(recv_id.clone())
                        .or_default()
                        .push(input_id.clone());
                }
            }
        }
        for (recv_id, affected_ids) in affected_by_receiver {
            if let Some(channel) = self.subscribe_channels.get(&recv_id) {
                let delivered = send_with_timestamp(
                    channel,
                    NodeEvent::NodeFailed {
                        affected_input_ids: affected_ids,
                        error: error_msg.to_string(),
                        source_node_id: failed_node.clone(),
                    },
                    clock,
                )
                .ok()
                    == Some(true);
                if delivered {
                    self.inc_pending(&recv_id);
                }
            }
        }
    }

    pub(crate) fn open_inputs(&self, node_id: &NodeId) -> &BTreeSet<DataId> {
        self.open_inputs.get(node_id).unwrap_or(&self.empty_set)
    }

    /// Wire up a node-to-node edge added at runtime (`dora node connect`).
    ///
    /// Kept here, rather than inline in the `AddMapping` handler, so the
    /// state it has to keep in step — `mappings`, `open_inputs`,
    /// `data_inputs` and the drain clock — can be exercised directly by a
    /// test instead of only through the daemon event loop.
    pub(crate) fn add_mapping(
        &mut self,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    ) {
        self.mappings
            .entry(OutputId(source_node, source_output))
            .or_default()
            .insert((target_node.clone(), target_input.clone()));
        // Reopening an input ends any drain: clear the stale clock so
        // the selector does not treat the node as drained-and-eligible
        // on a timestamp from before the mapping was re-added (#2270).
        self.all_inputs_closed_at.remove(&target_node);
        self.open_inputs
            .entry(target_node.clone())
            .or_default()
            .insert(target_input.clone());
        // A mapping is by construction a node-to-node edge, so this is a
        // data input and must gate the drain like any other. Without it
        // the opt-in would not see the new input at all, and could report
        // the node drained while an input that can still deliver is open
        // (#2920).
        self.data_inputs
            .entry(target_node)
            .or_default()
            .insert(target_input);
    }

    /// Whether `node_id` has finished draining — i.e. whether it should
    /// be told `AllInputsClosed`.
    ///
    /// With `timers_gate_drain` (the default) this is simply "no open
    /// inputs left". With it off, only inputs that *can* close count, so
    /// a node drains once its data inputs have closed even though its
    /// clock keeps ticking (dora-rs/dora#2920).
    ///
    /// A node with no data inputs at all — timer-only, logs-only, or no
    /// inputs — is never considered drained. It has no dependency that
    /// could ever finish, which makes it a source, and sources are not
    /// told to finish. Without this, enabling the opt-in would stop
    /// every timer-driven producer the moment the dataflow started.
    ///
    /// This deliberately reads `data_inputs`, recorded when inputs were
    /// registered, rather than re-deriving the answer from the
    /// descriptor: a runtime (`operators:`) node keeps its inputs under
    /// `operators[].config.inputs` and has an empty top-level `inputs`
    /// map, so a descriptor-derived check reports "no data inputs" for
    /// every operator node and hangs the dataflow it was meant to end.
    pub(crate) fn is_drained(&self, node_id: &NodeId) -> bool {
        let open = self.open_inputs(node_id);
        if self.timers_gate_drain {
            return open.is_empty();
        }
        let Some(data_inputs) = self.data_inputs.get(node_id) else {
            return false;
        };
        !data_inputs.is_empty() && data_inputs.is_disjoint(open)
    }

    /// Whether `node_id` has finished: drained, with no circuit-broken
    /// input that could still recover and deliver more data.
    ///
    /// The shared half of both drain sites (`Daemon::subscribe` and
    /// `signal_all_inputs_closed_if_drained`), which had already drifted —
    /// only one of them excluded broken inputs. The source check is NOT
    /// folded in here: it belongs only at the subscribe site, where a node
    /// that never had inputs can reach the test. Adding it to the close
    /// path would change default-mode behavior, and the failure mode of
    /// getting that wrong is a missed `AllInputsClosed`, i.e. the hang
    /// this issue exists to remove.
    pub(crate) fn is_finished(&self, node_id: &NodeId) -> bool {
        self.is_drained(node_id) && !self.has_broken_input(node_id)
    }

    /// Whether any of this node's inputs is circuit-broken (out of
    /// `open_inputs`, but recoverable).
    pub(crate) fn has_broken_input(&self, node_id: &NodeId) -> bool {
        self.broken_inputs.keys().any(|(nid, _)| nid == node_id)
    }

    /// Whether a node that has just subscribed should immediately be told
    /// `AllInputsClosed` — i.e. it is finished AND is not a source.
    ///
    /// The source half only matters here. On the close path a node reached
    /// the test by having an input close, so it necessarily had one; but a
    /// node can subscribe having never had any input, and in the default
    /// mode "nothing open" reads as finished.
    ///
    /// Deliberately one method rather than two conditions at the call site:
    /// the descriptor-derived version of this check silently reported every
    /// runtime (`operators:`) node as a source and hung it, and a decision
    /// spelled out at the call site cannot be unit-tested.
    pub(crate) fn is_finished_non_source(&self, node_id: &NodeId) -> bool {
        self.is_finished(node_id) && self.has_data_input(node_id)
    }

    /// Whether this node declared at least one input that can ever close.
    pub(crate) fn has_data_input(&self, node_id: &NodeId) -> bool {
        self.data_inputs
            .get(node_id)
            .is_some_and(|inputs| !inputs.is_empty())
    }

    /// All output ids produced by `node_id`, whether they are consumed by a
    /// local node (recorded in `mappings`) or only by nodes on another daemon
    /// (recorded in `open_external_mappings`).
    ///
    /// Deriving the set from `mappings` alone misses outputs that have no local
    /// consumer, so their remote consumers would never receive the
    /// `OutputClosed` event when the producing node finishes (dora-rs/dora#2152
    /// region — graceful cross-daemon shutdown).
    pub(crate) fn node_output_ids(&self, node_id: &NodeId) -> BTreeSet<DataId> {
        node_output_ids(&self.mappings, &self.open_external_mappings, node_id)
    }

    /// Nodes blocking an otherwise-finished dataflow (dora-rs/dora#2152).
    ///
    /// Returns nodes that should be force-stopped because the dataflow is
    /// "otherwise finished" — every running non-dynamic node is quiescent —
    /// but they have not exited. A node is quiescent once it has either been
    /// draining (received `AllInputsClosed`) past `grace`, or — never having
    /// reached the drain state — gone silent (no daemon traffic) past `grace`.
    /// The second arm catches a node that wedges *before* draining (dora#2152:
    /// the operator runtime stuck after stop), which the drain-only gate let
    /// hang indefinitely.
    ///
    /// A running source (no inputs, never drains) means the dataflow is still
    /// producing, so nothing escalates; likewise an active non-source node
    /// (recent traffic) means work is still in progress. Explicitly stopped
    /// dataflows are excluded (`stop_all` runs its own kill escalation), as are
    /// dataflows with open cross-daemon output mappings — a local node that
    /// looks like a straggler may still be flushing outputs to consumers on
    /// other daemons, which this daemon cannot see.
    ///
    /// `now_millis` is the current `node_communication::current_millis()`, the
    /// clock `RunningNode::last_activity` is stamped against.
    pub(crate) fn finish_stragglers(&self, grace: Duration, now_millis: u64) -> Vec<NodeId> {
        if self.stop_sent || !self.open_external_mappings.is_empty() {
            return Vec::new();
        }
        select_finish_stragglers(
            self.running_nodes.iter().map(|(id, node)| {
                let last = node.last_activity.load(atomic::Ordering::Acquire);
                StragglerNode {
                    id,
                    dynamic: node.node_config.dynamic,
                    never_finishes: self.node_never_finishes(id),
                    // A node is only a silence candidate once it has subscribed
                    // — i.e. has connected and can receive AllInputsClosed/Stop.
                    // Until then it is still starting (a slow model-load node),
                    // not a wedge, even though `last_activity` is seeded at spawn
                    // time and would otherwise read as long-silent. Uses the
                    // sticky `connected_nodes` rather than current channel
                    // presence so a node that dropped its event stream but is
                    // still alive remains a candidate (dora#2270 review).
                    connected: self.connected_nodes.contains(id),
                    drained_for: self.all_inputs_closed_at.get(id).map(Instant::elapsed),
                    silent_for: Duration::from_millis(now_millis.saturating_sub(last)),
                    node_grace: node.finish_grace_secs,
                }
            }),
            &self.finish_escalated,
            grace,
        )
    }

    /// Whether a node can never reach natural finish, so the finish-straggler
    /// watchdog must leave it alone (it is stopped only via the explicit-stop
    /// path). True for a source (no inputs) and — crucially — for any node fed
    /// by a `Timer` or `Logs` input: those virtual inputs never close, so the
    /// node never receives `AllInputsClosed`, and timer/log delivery is
    /// daemon-to-node so it never refreshes `last_activity`. Such a node looks
    /// "silent and never drained" exactly like a wedge, but is alive by design
    /// (dora-rs/dora#2270) — e.g. a long-running timer-only side-effect node.
    fn node_never_finishes(&self, node_id: &NodeId) -> bool {
        // A node that HAS drained under the opt-in really can finish, even
        // though a timer or logs input keeps it fed. Treating it as
        // never-finishing would veto the straggler watchdog for the whole
        // dataflow (see `select_finish_stragglers`) exactly on the path
        // where nodes newly receive `AllInputsClosed` — so a drained node
        // that then wedges could never be escalated (#2920).
        //
        // Keyed on having drained, not on merely declaring data inputs: a
        // timer-fed node whose data inputs are still open has not finished,
        // and arming the watchdog for it would let a slow upstream plus a
        // long timer interval read as "silent past the grace period" and
        // get it escalated to SIGKILL while it is healthy.
        if !self.timers_gate_drain && self.is_drained(node_id) {
            return false;
        }
        // NOTE: this misclassifies a runtime (`operators:`) node as a
        // source, because its inputs live under `operators[].config.inputs`
        // and the top-level map is empty. Pre-existing and left alone here:
        // correcting it would newly arm the straggler watchdog for operator
        // dataflows, which is a behavior change well outside #2920.
        let is_source = self
            .descriptor
            .nodes
            .iter()
            .find(|n| &n.id == node_id)
            .is_some_and(|n| n.inputs.is_empty());
        let has_timer = self
            .timers
            .values()
            .any(|receivers| receivers.iter().any(|(id, _)| id == node_id));
        let has_logs = self
            .log_subscribers
            .iter()
            .any(|sub| &sub.node_id == node_id);
        is_source || has_timer || has_logs
    }
}

/// One running node's view for [`select_finish_stragglers`].
struct StragglerNode<'a> {
    id: &'a NodeId,
    dynamic: bool,
    /// Node that never reaches natural finish — a source, or one kept alive by
    /// a `Timer`/`Logs` input (see [`RunningDataflow::node_never_finishes`]).
    never_finishes: bool,
    /// Whether the node has subscribed and can receive finish events. A node
    /// that has not connected yet is still starting up, not wedged.
    connected: bool,
    /// Time since `AllInputsClosed`, or `None` if it never reached drain.
    drained_for: Option<Duration>,
    /// Time since the last daemon-bound message from this node. Only meaningful
    /// once `connected` — `last_activity` is seeded at spawn time.
    silent_for: Duration,
    /// Per-node grace override from `finish_grace_secs` in the descriptor.
    /// When `Some`, takes precedence over the global grace passed to
    /// [`select_finish_stragglers`] for this node only.
    node_grace: Option<Duration>,
}

/// Pure core of [`RunningDataflow::node_output_ids`].
///
/// Unions the output ids of `node_id` across both the locally-consumed
/// `mappings` and the remote-only `open_external_mappings`.
fn node_output_ids(
    mappings: &HashMap<OutputId, BTreeSet<(NodeId, DataId)>>,
    open_external_mappings: &BTreeSet<OutputId>,
    node_id: &NodeId,
) -> BTreeSet<DataId> {
    mappings
        .keys()
        .chain(open_external_mappings.iter())
        .filter(|output| &output.0 == node_id)
        .map(|output| output.1.clone())
        .collect()
}

/// Pure core of [`RunningDataflow::finish_stragglers`].
fn select_finish_stragglers<'a>(
    running_nodes: impl Iterator<Item = StragglerNode<'a>>,
    already_escalated: &BTreeSet<NodeId>,
    grace: Duration,
) -> Vec<NodeId> {
    // Gate first: every running non-dynamic node must be quiescent (drained or
    // silent past grace), otherwise the dataflow is not "otherwise finished"
    // (a running source, or a node still producing) and nothing may escalate.
    let mut eligible = Vec::new();
    for node in running_nodes {
        if node.dynamic {
            continue;
        }
        if node.never_finishes {
            // a live source (or timer/log-fed node) keeps the dataflow running;
            // it is stopped only via the explicit-stop path, never escalated here
            return Vec::new();
        }
        // Per-node `finish_grace_secs` overrides the global grace so that nodes
        // with long post-input compute (ML training, large-batch inference) are
        // not SIGKILLed while legitimately busy (dora-rs/dora#2284).
        let effective_grace = node.node_grace.unwrap_or(grace);
        match node.drained_for {
            // draining: ready past grace; still within grace it is progressing
            // toward exit and does not veto, but is not escalated yet
            Some(drained_for) => {
                if drained_for >= effective_grace {
                    eligible.push(node.id.clone());
                }
            }
            // never drained: only "finished" if a connected node has wedged
            // silent past grace. An unconnected node is still starting up (so
            // the dataflow is not otherwise finished); a connected but active
            // node still has work in progress — both veto.
            None => {
                if node.connected && node.silent_for >= effective_grace {
                    eligible.push(node.id.clone());
                } else {
                    return Vec::new();
                }
            }
        }
    }

    eligible
        .into_iter()
        .filter(|node_id| !already_escalated.contains(node_id))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- dora-rs/dora#2152: finish-straggler selection ----

    fn node_id(name: &str) -> NodeId {
        NodeId::from(name.to_string())
    }

    fn data_id(name: &str) -> DataId {
        DataId::from(name.to_string())
    }

    // ---- node_output_ids: remote-only outputs must be included ----

    #[test]
    fn node_output_ids_unions_local_and_remote_outputs() {
        let source = node_id("source");
        let mut mappings: HashMap<OutputId, BTreeSet<(NodeId, DataId)>> = HashMap::new();
        // `source/local_out` is consumed by a local node.
        mappings.insert(
            OutputId(source.clone(), data_id("local_out")),
            BTreeSet::from([(node_id("sink"), data_id("in"))]),
        );
        // `source/remote_out` is consumed only by a node on another daemon.
        let open_external_mappings =
            BTreeSet::from([OutputId(source.clone(), data_id("remote_out"))]);

        let outputs = node_output_ids(&mappings, &open_external_mappings, &source);

        // Both the locally-consumed and the remote-only output must appear, so
        // the remote consumer receives an `OutputClosed` when `source` finishes.
        assert_eq!(
            outputs,
            BTreeSet::from([data_id("local_out"), data_id("remote_out")]),
        );
    }

    #[test]
    fn node_output_ids_ignores_other_producers() {
        let mut mappings: HashMap<OutputId, BTreeSet<(NodeId, DataId)>> = HashMap::new();
        mappings.insert(
            OutputId(node_id("other"), data_id("x")),
            BTreeSet::from([(node_id("sink"), data_id("in"))]),
        );
        let open_external_mappings = BTreeSet::from([OutputId(node_id("other"), data_id("y"))]);

        let outputs = node_output_ids(&mappings, &open_external_mappings, &node_id("source"));
        assert!(outputs.is_empty());
    }

    // ---- dora-rs/dora#2827: NodeFailed propagation must not underflow the
    //      receiver's pending_messages counter ----

    /// Build a descriptor with one node whose inputs are as given, via
    /// the real YAML shape — `(input_id, is_timer)`. Parsing rather than
    /// constructing means the `InputMapping` variants are exactly the
    /// ones production builds.
    fn descriptor_with_node(node_id: &str, inputs: &[(&str, bool)]) -> Descriptor {
        let mut yaml = format!("nodes:\n  - id: {node_id}\n    path: dummy\n");
        if !inputs.is_empty() {
            yaml.push_str("    inputs:\n");
            for (id, is_timer) in inputs {
                if *is_timer {
                    yaml.push_str(&format!("      {id}: dora/timer/millis/100\n"));
                } else {
                    yaml.push_str(&format!("      {id}: upstream/value\n"));
                }
            }
        }
        serde_yaml::from_str(&yaml).expect("test descriptor should parse")
    }

    /// A dataflow with `node` declared as in `descriptor_with_node`, its
    /// inputs all currently open, timer inputs registered as timers, and
    /// data inputs recorded in `data_inputs` — mirroring what the daemon
    /// does when it registers a node's inputs.
    fn dataflow_with_node(node_id: &str, inputs: &[(&str, bool)]) -> RunningDataflow {
        let mut df = RunningDataflow::new(
            uuid::Uuid::nil(),
            DaemonId::new(None),
            descriptor_with_node(node_id, inputs),
        );
        register_inputs(&mut df, node_id, inputs);
        df
    }

    /// Register one node's inputs the way the daemon's spawn path does.
    fn register_inputs(df: &mut RunningDataflow, node_id: &str, inputs: &[(&str, bool)]) {
        let node: NodeId = node_id.to_string().into();
        for (id, is_timer) in inputs {
            let input: DataId = id.to_string().into();
            df.open_inputs
                .entry(node.clone())
                .or_default()
                .insert(input.clone());
            if *is_timer {
                df.timers
                    .entry(Duration::from_millis(100))
                    .or_default()
                    .insert((node.clone(), input));
            } else {
                df.data_inputs
                    .entry(node.clone())
                    .or_default()
                    .insert(input);
            }
        }
    }

    /// Default behavior is unchanged: every input gates the drain, so a
    /// node with a live timer is never told its inputs closed. This is
    /// the dora-rs/dora#2920 hang.
    #[test]
    fn timers_gate_drain_by_default() {
        let node: NodeId = "consumer".to_string().into();
        let mut df = dataflow_with_node("consumer", &[("value", false), ("tick", true)]);
        assert!(!df.is_drained(&node), "nothing has closed yet");

        // The data input closes; the timer keeps ticking.
        df.open_inputs
            .get_mut(&node)
            .unwrap()
            .remove(&DataId::from("value".to_string()));
        assert!(
            !df.is_drained(&node),
            "by default a live timer must still gate the drain — that is the \
             behavior the opt-in flag changes, and changing it silently would \
             stop nodes that are still expected to run"
        );
    }

    /// With the opt-in, a timer is a clock rather than a data
    /// dependency: the node drains once its data inputs have closed.
    #[test]
    fn data_inputs_alone_gate_drain_when_opted_in() {
        let node: NodeId = "consumer".to_string().into();
        let mut df = dataflow_with_node("consumer", &[("value", false), ("tick", true)]);
        df.timers_gate_drain = false;
        assert!(!df.is_drained(&node), "the data input is still open");

        df.open_inputs
            .get_mut(&node)
            .unwrap()
            .remove(&DataId::from("value".to_string()));
        assert!(
            df.is_drained(&node),
            "with the opt-in, a closed data input drains the node even though \
             its timer is still registered"
        );
    }

    /// A node whose inputs are ALL timers has no data dependency that
    /// could ever finish, so it is a source. Draining it would tell every
    /// timer-driven producer to stop the moment the dataflow started.
    #[test]
    fn timer_only_nodes_are_never_drained_even_when_opted_in() {
        let node: NodeId = "ticker".to_string().into();
        let mut df = dataflow_with_node("ticker", &[("tick", true)]);
        df.timers_gate_drain = false;
        assert!(
            !df.is_drained(&node),
            "a timer-only node is a source; draining it at startup would stop \
             every timer-driven producer immediately"
        );
    }

    /// And a node with no inputs at all stays a source under the opt-in.
    #[test]
    fn input_less_nodes_are_never_drained_even_when_opted_in() {
        let node: NodeId = "source".to_string().into();
        let mut df = dataflow_with_node("source", &[]);
        df.timers_gate_drain = false;
        assert!(!df.is_drained(&node));
    }

    /// A data input must only satisfy the node that declared it. Keying
    /// the drain on the input id alone would drain any node sharing an
    /// input name with another node's timer — and `tick` is the most
    /// common input name in this repo's own dataflows.
    #[test]
    fn drain_does_not_confuse_inputs_that_share_a_name_across_nodes() {
        let ticker: NodeId = "ticker".to_string().into();
        let worker: NodeId = "worker".to_string().into();
        let mut df = dataflow_with_node("ticker", &[("tick", true)]);
        // `worker` also has an input called `tick`, but its is real data.
        register_inputs(&mut df, "worker", &[("tick", false)]);
        df.timers_gate_drain = false;

        assert!(
            !df.is_drained(&worker),
            "worker's `tick` is a DATA input and is still open; it must not \
             be drained just because another node has a timer named `tick`"
        );
        assert!(!df.is_drained(&ticker), "ticker is timer-only, so a source");

        df.open_inputs
            .get_mut(&worker)
            .unwrap()
            .remove(&DataId::from("tick".to_string()));
        assert!(df.is_drained(&worker), "worker's data input has now closed");
    }

    /// An input added at runtime (`dora node connect` -> `AddMapping`)
    /// must gate the drain like a declared one. It is recorded in
    /// `open_inputs`, so if it were missing from `data_inputs` the
    /// disjointness test would simply not see it, and a node could be
    /// told all its inputs were closed while an input that can still
    /// deliver data was open.
    #[test]
    fn inputs_added_at_runtime_gate_the_drain() {
        let node: NodeId = "consumer".to_string().into();
        let mut df = dataflow_with_node("consumer", &[("value", false), ("tick", true)]);
        df.timers_gate_drain = false;

        // `dora node connect` wires up a second data input. This must go
        // through the real handler helper: routing it via the test's own
        // `register_inputs` would assert against a re-implementation of
        // the very bookkeeping under test, and would still pass if
        // `add_mapping` stopped recording the input.
        df.add_mapping(
            "producer".to_string().into(),
            "late_out".to_string().into(),
            node.clone(),
            "late".to_string().into(),
        );
        assert!(
            df.data_inputs
                .get(&node)
                .is_some_and(|inputs| inputs.contains(&DataId::from("late".to_string()))),
            "add_mapping must record the new edge as a data input"
        );

        // The originally-declared input closes; `late` is still open.
        df.open_inputs
            .get_mut(&node)
            .unwrap()
            .remove(&DataId::from("value".to_string()));
        assert!(
            !df.is_drained(&node),
            "a runtime-added input is still open, so the node has not finished \
             — draining here would drop data that input can still deliver"
        );

        df.open_inputs
            .get_mut(&node)
            .unwrap()
            .remove(&DataId::from("late".to_string()));
        assert!(df.is_drained(&node), "now every data input has closed");
    }

    /// A runtime (`operators:`) node keeps its inputs under
    /// `operators[].config.inputs`, leaving the node's own `inputs` map
    /// empty. Deriving "has data inputs" from the descriptor therefore
    /// reported false for every operator node and hung the dataflow the
    /// opt-in was meant to end. Recording the inputs at registration time
    /// is what makes this work.
    #[test]
    fn operator_nodes_drain_under_the_opt_in() {
        let yaml = "nodes:\n  \
                    - id: sink\n    \
                      operators:\n      \
                        - id: op\n        \
                          shared-library: dummy\n        \
                          inputs:\n          \
                            data: source/op/data\n";
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let node: NodeId = "sink".to_string().into();
        assert!(
            descriptor.nodes[0].inputs.is_empty(),
            "precondition: an operator node's top-level `inputs` map is empty, \
             which is exactly why the descriptor cannot answer this question"
        );

        let mut df = RunningDataflow::new(uuid::Uuid::nil(), DaemonId::new(None), descriptor);
        // The daemon registers the flattened `op/data` input.
        register_inputs(&mut df, "sink", &[("op/data", false)]);
        df.timers_gate_drain = false;

        assert!(!df.is_drained(&node), "the operator's data input is open");
        df.open_inputs
            .get_mut(&node)
            .unwrap()
            .remove(&DataId::from("op/data".to_string()));
        assert!(
            df.is_drained(&node),
            "an operator node must drain once its data inputs close, exactly \
             like a custom node"
        );
    }

    /// The send decision, not just the drain predicate. An operator node
    /// must be told `AllInputsClosed`: deciding "is this a source" from
    /// the descriptor calls every runtime node a source (its top-level
    /// `inputs` map is empty) and skips the event, which is the hang this
    /// issue is about.
    #[test]
    fn operator_nodes_are_told_all_inputs_closed() {
        let yaml = "nodes:\n  \
                    - id: sink\n    \
                      operators:\n      \
                        - id: op\n        \
                          shared-library: dummy\n        \
                          inputs:\n          \
                            data: source/op/data\n";
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let node: NodeId = "sink".to_string().into();
        let mut df = RunningDataflow::new(uuid::Uuid::nil(), DaemonId::new(None), descriptor);
        register_inputs(&mut df, "sink", &[("op/data", false)]);
        df.timers_gate_drain = false;

        assert!(!df.is_finished(&node), "input still open");
        df.open_inputs
            .get_mut(&node)
            .unwrap()
            .remove(&DataId::from("op/data".to_string()));
        assert!(
            df.is_finished_non_source(&node),
            "an operator node must not read as a source, or subscribe skips \
             `AllInputsClosed` and it hangs forever — the failure this issue \
             exists to fix. Deriving this from the descriptor gets it wrong, \
             because a runtime node's top-level `inputs` map is empty."
        );
    }

    /// Sources are never told to finish, in either mode.
    #[test]
    fn sources_are_not_told_all_inputs_closed() {
        let source: NodeId = "source".to_string().into();
        let mut df = dataflow_with_node("source", &[]);
        // In the default mode a source looks drained (nothing is open), so
        // the source check is what stops the event being sent.
        assert!(df.is_finished(&source), "nothing is open");
        assert!(
            !df.is_finished_non_source(&source),
            "but it is a source, so it must not be told to finish"
        );
        df.timers_gate_drain = false;
        // Under the opt-in `is_drained` already excludes it.
        assert!(!df.is_finished_non_source(&source));
    }

    /// A circuit-broken input is recoverable, so the node is not finished.
    /// It leaves `open_inputs`, which would otherwise read as drained.
    #[test]
    fn circuit_broken_inputs_do_not_count_as_finished() {
        let node: NodeId = "consumer".to_string().into();
        let value = DataId::from("value".to_string());
        let mut df = dataflow_with_node("consumer", &[("value", false), ("tick", true)]);
        df.timers_gate_drain = false;

        // break_input moves the input out of `open_inputs` but keeps it
        // recoverable in `broken_inputs`.
        df.open_inputs.get_mut(&node).unwrap().remove(&value);
        df.broken_inputs
            .insert((node.clone(), value.clone()), Default::default());

        assert!(
            df.is_drained(&node),
            "precondition: with the input out of `open_inputs`, the drain \
             predicate alone cannot tell this from a finished node"
        );
        assert!(
            !df.is_finished(&node),
            "a recoverable input must not end the node; it would exit for good \
             and never see the data that arrives after recovery"
        );
    }

    /// The straggler watchdog must stay armed for nodes the opt-in drains.
    /// `node_never_finishes` vetoes escalation for the whole dataflow, so
    /// leaving timer-fed nodes marked never-finishing would disable the
    /// safety net precisely where nodes newly get `AllInputsClosed`.
    #[test]
    fn opt_in_keeps_the_straggler_watchdog_armed_for_drainable_nodes() {
        let node: NodeId = "consumer".to_string().into();
        let mut df = dataflow_with_node("consumer", &[("value", false), ("tick", true)]);

        assert!(
            df.node_never_finishes(&node),
            "by default a timer-fed node genuinely never finishes"
        );

        df.timers_gate_drain = false;
        assert!(
            df.node_never_finishes(&node),
            "the opt-in alone is not enough: this node's data input is still \
             open, so it has NOT finished. Arming the watchdog here would let \
             a slow upstream plus a long timer interval look like a wedge and \
             get a healthy node SIGKILLed"
        );

        // Now it actually drains.
        df.open_inputs
            .get_mut(&node)
            .unwrap()
            .remove(&DataId::from("value".to_string()));
        assert!(
            !df.node_never_finishes(&node),
            "having drained, this node CAN finish, so the watchdog must be \
             able to escalate it if it then wedges"
        );
    }

    /// Timer-only nodes stay exempt from the watchdog even under the
    /// opt-in — they really never finish.
    #[test]
    fn opt_in_leaves_timer_only_nodes_exempt_from_the_watchdog() {
        let node: NodeId = "ticker".to_string().into();
        let mut df = dataflow_with_node("ticker", &[("tick", true)]);
        df.timers_gate_drain = false;
        assert!(df.node_never_finishes(&node));
    }

    fn empty_descriptor() -> Descriptor {
        use dora_message::descriptor::Debug as DescriptorDebug;
        Descriptor {
            nodes: vec![],
            deploy: None,
            debug: DescriptorDebug::default(),
            health_check_interval: None,
            strict_types: None,
            exit_when_nodes_finish: None,
            type_rules: vec![],
            env: None,
        }
    }

    #[test]
    fn propagate_node_failed_keeps_idle_receiver_counter_consistent() {
        let mut df =
            RunningDataflow::new(uuid::Uuid::nil(), DaemonId::new(None), empty_descriptor());
        let failed = node_id("source");
        let receiver = node_id("sink");

        // `source/out` is consumed by `sink/in`.
        df.mappings.insert(
            OutputId(failed.clone(), data_id("out")),
            BTreeSet::from([(receiver.clone(), data_id("in"))]),
        );

        // The receiver is subscribed and idle: its pending counter starts at 0.
        let (tx, mut rx) = mpsc::channel(16);
        df.subscribe_channels.insert(receiver.clone(), tx);
        let counter = Arc::new(AtomicU64::new(0));
        df.pending_messages
            .insert(receiver.clone(), counter.clone());

        let clock = HLC::default();
        df.propagate_node_failed(&failed, "boom", &clock);

        // The successful enqueue must have incremented the counter to match the
        // in-flight event.
        assert_eq!(counter.load(atomic::Ordering::Relaxed), 1);

        // Drain the event exactly as the Listener does: one unconditional
        // decrement per drained event. With the matching increment in place the
        // counter returns to 0; without it, this decrement would wrap to
        // `u64::MAX`.
        let mut drained = 0;
        while let Ok(event) = rx.try_recv() {
            counter.fetch_sub(1, atomic::Ordering::Relaxed);
            assert!(matches!(event.inner, NodeEvent::NodeFailed { .. }));
            drained += 1;
        }
        assert_eq!(drained, 1, "receiver should get exactly one NodeFailed");
        assert_eq!(
            counter.load(atomic::Ordering::Relaxed),
            0,
            "pending counter must stay consistent (no u64::MAX underflow)"
        );
    }

    const TEST_GRACE: Duration = Duration::from_millis(100);
    const PAST_GRACE: Duration = Duration::from_secs(2);
    const WITHIN_GRACE: Duration = Duration::ZERO;

    /// A finishing (non-source, no timer/log), connected node draining for `age`.
    fn drained<'a>(id: &'a NodeId, age: Duration) -> StragglerNode<'a> {
        StragglerNode {
            id,
            dynamic: false,
            never_finishes: false,
            connected: true,
            drained_for: Some(age),
            silent_for: Duration::ZERO,
            node_grace: None,
        }
    }

    /// A finishing, connected node that never drained, silent for `silent_for`.
    fn never_drained<'a>(id: &'a NodeId, silent_for: Duration) -> StragglerNode<'a> {
        StragglerNode {
            id,
            dynamic: false,
            never_finishes: false,
            connected: true,
            drained_for: None,
            silent_for,
            node_grace: None,
        }
    }

    #[test]
    fn straggler_past_grace_is_selected() {
        let sink = node_id("sink");
        let selected = select_finish_stragglers(
            [drained(&sink, PAST_GRACE)].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert_eq!(selected, vec![node_id("sink")]);
    }

    #[test]
    fn straggler_within_grace_is_not_selected() {
        let sink = node_id("sink");
        let selected = select_finish_stragglers(
            [drained(&sink, WITHIN_GRACE)].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert!(selected.is_empty());
    }

    #[test]
    fn running_source_blocks_escalation_of_drained_nodes() {
        // `source` never receives AllInputsClosed; while it runs, the
        // dataflow is not "otherwise finished" and nothing escalates.
        let source = node_id("source");
        let sink = node_id("sink");
        let source_node = StragglerNode {
            id: &source,
            dynamic: false,
            never_finishes: true,
            connected: true,
            drained_for: None,
            silent_for: PAST_GRACE,
            node_grace: None,
        };
        let selected = select_finish_stragglers(
            [source_node, drained(&sink, PAST_GRACE)].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert!(selected.is_empty());
    }

    #[test]
    fn dynamic_nodes_neither_block_nor_escalate() {
        let dynamic = node_id("dynamic");
        let sink = node_id("sink");
        let dynamic_node = StragglerNode {
            id: &dynamic,
            dynamic: true,
            never_finishes: false,
            connected: true,
            drained_for: None,
            silent_for: WITHIN_GRACE,
            node_grace: None,
        };
        let selected = select_finish_stragglers(
            [dynamic_node, drained(&sink, PAST_GRACE)].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert_eq!(selected, vec![node_id("sink")]);
    }

    #[test]
    fn already_escalated_nodes_are_not_reselected() {
        let sink = node_id("sink");
        let escalated: BTreeSet<_> = [node_id("sink")].into();
        let selected = select_finish_stragglers(
            [drained(&sink, PAST_GRACE)].into_iter(),
            &escalated,
            TEST_GRACE,
        );
        assert!(selected.is_empty());
    }

    #[test]
    fn multiple_drained_stragglers_are_all_selected() {
        let a = node_id("a");
        let b = node_id("b");
        let selected = select_finish_stragglers(
            [drained(&a, PAST_GRACE), drained(&b, PAST_GRACE)].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert_eq!(selected, vec![node_id("a"), node_id("b")]);
    }

    // ---- dora-rs/dora#2270: wedge-before-drain escalation ----

    #[test]
    fn non_source_wedged_silent_past_grace_is_escalated() {
        // the #2152/#2270 case: a node that never reached the drain state but
        // has gone silent past grace while the rest of the dataflow finished.
        let stuck = node_id("runtime");
        let selected = select_finish_stragglers(
            [never_drained(&stuck, PAST_GRACE)].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert_eq!(selected, vec![node_id("runtime")]);
    }

    #[test]
    fn unconnected_node_silent_past_grace_is_not_escalated() {
        // `last_activity` is seeded at spawn, so a slow-starting node that has
        // not subscribed yet reads as long-silent — but it is still coming up,
        // not wedged, and must not be killed (dora#2270 review regression).
        let starting = node_id("slow_loader");
        let node = StragglerNode {
            id: &starting,
            dynamic: false,
            never_finishes: false,
            connected: false,
            drained_for: None,
            silent_for: PAST_GRACE,
            node_grace: None,
        };
        let selected = select_finish_stragglers([node].into_iter(), &BTreeSet::new(), TEST_GRACE);
        assert!(selected.is_empty());
    }

    #[test]
    fn non_source_active_without_drain_blocks_escalation() {
        // a non-source node still sending daemon traffic means work is in
        // progress — the dataflow is not otherwise finished.
        let active = node_id("active");
        let sink = node_id("sink");
        let selected = select_finish_stragglers(
            [
                never_drained(&active, WITHIN_GRACE),
                drained(&sink, PAST_GRACE),
            ]
            .into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert!(selected.is_empty());
    }

    #[test]
    fn never_finishing_node_is_not_escalated_when_silent() {
        // a source or timer/log-fed node is stopped via the explicit-stop path,
        // never via finish-straggler escalation — even when silent past grace.
        // Guards the dora#2270 regression where a long-running timer-only node
        // (silent, never drained) was force-killed after the grace period.
        let perpetual = node_id("timer_node");
        let node = StragglerNode {
            id: &perpetual,
            dynamic: false,
            never_finishes: true,
            connected: true,
            drained_for: None,
            silent_for: PAST_GRACE,
            node_grace: None,
        };
        let selected = select_finish_stragglers([node].into_iter(), &BTreeSet::new(), TEST_GRACE);
        assert!(selected.is_empty());
    }

    #[test]
    fn drained_and_wedged_stragglers_escalate_together() {
        let sink = node_id("sink");
        let stuck = node_id("runtime");
        let selected = select_finish_stragglers(
            [
                drained(&sink, PAST_GRACE),
                never_drained(&stuck, PAST_GRACE),
            ]
            .into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert_eq!(selected, vec![node_id("sink"), node_id("runtime")]);
    }

    // ---- dora-rs/dora#2284: per-node finish-grace override ----

    #[test]
    fn node_with_longer_per_node_grace_is_not_escalated_when_global_grace_expired() {
        // Trainer has a 10-minute per-node grace; global grace is 100ms.
        // After PAST_GRACE (> global grace) the trainer must NOT be killed.
        let trainer = node_id("trainer");
        let node = StragglerNode {
            id: &trainer,
            dynamic: false,
            never_finishes: false,
            connected: true,
            drained_for: Some(PAST_GRACE),
            silent_for: PAST_GRACE,
            node_grace: Some(Duration::from_secs(600)),
        };
        let selected = select_finish_stragglers([node].into_iter(), &BTreeSet::new(), TEST_GRACE);
        assert!(
            selected.is_empty(),
            "trainer should not be escalated before its own grace"
        );
    }

    #[test]
    fn node_with_longer_per_node_grace_is_escalated_after_its_own_grace() {
        // After the per-node grace the node is eligible for escalation.
        let trainer = node_id("trainer");
        let long_grace = Duration::from_millis(50);
        let node = StragglerNode {
            id: &trainer,
            dynamic: false,
            never_finishes: false,
            connected: true,
            drained_for: Some(Duration::from_millis(200)), // well past long_grace
            silent_for: Duration::ZERO,
            node_grace: Some(long_grace),
        };
        let selected = select_finish_stragglers([node].into_iter(), &BTreeSet::new(), TEST_GRACE);
        assert_eq!(selected, vec![node_id("trainer")]);
    }

    #[test]
    fn per_node_grace_does_not_block_other_nodes_already_past_global_grace() {
        // sink is past global grace; trainer has a longer per-node grace and is
        // still within it.  The trainer's long grace must NOT veto sink's escalation.
        // But sink cannot escalate alone — the gate requires ALL non-dynamic
        // nodes to be quiescent.  The trainer is still "within grace" (drained_for
        // is Some but < effective_grace), so it does not veto — but it also is
        // not yet escalated.  Sink therefore escalates on its own.
        let sink = node_id("sink");
        let trainer = node_id("trainer");
        let long_grace = Duration::from_secs(600);
        let trainer_node = StragglerNode {
            id: &trainer,
            dynamic: false,
            never_finishes: false,
            connected: true,
            drained_for: Some(PAST_GRACE), // past global grace, within per-node grace
            silent_for: PAST_GRACE,
            node_grace: Some(long_grace),
        };
        let selected = select_finish_stragglers(
            [drained(&sink, PAST_GRACE), trainer_node].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        // sink is past global grace (no per-node override); trainer is within
        // its own grace — only sink should be escalated
        assert_eq!(selected, vec![node_id("sink")]);
    }

    #[test]
    fn never_drained_node_within_per_node_grace_vetoes_whole_dataflow() {
        // Intended behavioral expansion (dora-rs/dora#2284): a connected node
        // that has NOT yet drained (no AllInputsClosed) but declares a large
        // `finish_grace_secs` is treated as legitimately busy. While it is
        // within its own grace it takes the never-drained `None` arm and vetoes
        // escalation for the ENTIRE dataflow — even a sibling that drained long
        // past the global grace. This deliberately holds the finish-straggler
        // watchdog open for the long-grace node's window (e.g. a trainer that
        // keeps computing after a sibling sink finished), rather than killing
        // the dataflow at the global grace as before this PR.
        let sink = node_id("sink");
        let trainer = node_id("trainer");
        let long_grace = Duration::from_secs(600);
        let trainer_node = StragglerNode {
            id: &trainer,
            dynamic: false,
            never_finishes: false,
            connected: true,
            drained_for: None,      // has not received AllInputsClosed yet
            silent_for: PAST_GRACE, // silent past global grace, within per-node grace
            node_grace: Some(long_grace),
        };
        let selected = select_finish_stragglers(
            [drained(&sink, PAST_GRACE), trainer_node].into_iter(),
            &BTreeSet::new(),
            TEST_GRACE,
        );
        assert!(
            selected.is_empty(),
            "a busy long-grace node not yet drained must hold the dataflow open, \
             vetoing escalation of drained siblings"
        );
    }

    // ---- dora-rs/adora#149: InputDeadline::is_timed_out ----

    #[test]
    fn unarmed_deadline_is_never_timed_out() {
        let deadline = InputDeadline {
            timeout: Duration::from_millis(1),
            last_received: None,
        };
        std::thread::sleep(Duration::from_millis(5));
        assert!(!deadline.is_timed_out());
    }

    #[test]
    fn armed_deadline_within_timeout_is_not_timed_out() {
        let deadline = InputDeadline {
            timeout: Duration::from_secs(60),
            last_received: Some(Instant::now()),
        };
        assert!(!deadline.is_timed_out());
    }

    #[test]
    fn armed_deadline_past_timeout_is_timed_out() {
        let deadline = InputDeadline {
            timeout: Duration::from_millis(1),
            last_received: Some(Instant::now() - Duration::from_millis(10)),
        };
        assert!(deadline.is_timed_out());
    }

    #[test]
    fn arming_a_previously_unarmed_deadline_starts_the_clock() {
        let mut deadline = InputDeadline {
            timeout: Duration::from_millis(50),
            last_received: None,
        };
        assert!(!deadline.is_timed_out(), "unarmed should not fire");
        deadline.last_received = Some(Instant::now());
        assert!(
            !deadline.is_timed_out(),
            "just-armed should not fire immediately"
        );
    }

    // ---- dora-rs/adora#152: restart isolation via fresh op_tx/op_rx ----

    #[test]
    fn submit_to_dropped_receiver_returns_false() {
        let (tx, rx) = flume::bounded::<ProcessOperation>(2);
        let handle = ProcessHandle::new(tx);
        drop(rx);
        assert!(!handle.submit(ProcessOperation::SoftKill));
        assert!(!handle.submit(ProcessOperation::Kill));
    }

    #[test]
    fn submit_to_live_receiver_succeeds() {
        let (tx, rx) = flume::bounded::<ProcessOperation>(2);
        let handle = ProcessHandle::new(tx);
        assert!(handle.submit(ProcessOperation::SoftKill));
        let received = rx.try_recv().expect("op should have been queued");
        assert!(matches!(received, ProcessOperation::SoftKill));
    }

    #[test]
    fn two_independent_channels_do_not_cross_deliver() {
        let (old_tx, old_rx) = flume::bounded::<ProcessOperation>(2);
        let old_handle = ProcessHandle::new(old_tx);
        drop(old_rx);
        let (_new_tx, new_rx) = flume::bounded::<ProcessOperation>(2);
        let _ = old_handle.submit(ProcessOperation::Kill);
        assert!(
            new_rx.try_recv().is_err(),
            "kill from the previous incarnation must not reach the replacement process"
        );
    }
}
