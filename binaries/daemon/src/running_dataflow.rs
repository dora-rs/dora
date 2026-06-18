//! Running dataflow state and associated types.

use crate::{
    DoraEvent, OutputId, coordinator, empty_type_info, fault_tolerance::CascadingErrorCauses,
    pending::PendingNodes, send_with_timestamp,
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
/// Default grace period before force-killing a stopped node.
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
};
use tracing::warn;

#[cfg(feature = "telemetry")]
use tracing_opentelemetry::OpenTelemetrySpanExt;

use crate::Event;
use process_wrap::tokio::TokioChildWrapper;

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
pub(crate) enum ProcessOperation {
    SoftKill,
    Kill,
}

impl ProcessOperation {
    pub fn execute(&self, child: &mut dyn TokioChildWrapper) {
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
    /// Nodes already escalated by the finish-straggler watchdog (one-shot).
    pub(crate) finish_escalated: BTreeSet<NodeId>,
    pub(crate) stop_sent: bool,
    pub(crate) empty_set: BTreeSet<DataId>,
    pub(crate) cascading_error_causes: CascadingErrorCauses,
    pub(crate) grace_duration_kills: Arc<crossbeam_skiplist::SkipSet<NodeId>>,
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
            finish_escalated: BTreeSet::new(),
            stop_sent: false,
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

                    let metadata = metadata::Metadata::from_parameters(
                        clock.new_timestamp(),
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
            .map(|(id, n)| (id.clone(), n.process.take()))
            .collect();
        if force {
            for (_, proc) in &running_processes {
                if let Some(proc) = proc {
                    proc.submit(ProcessOperation::Kill);
                }
            }
        } else {
            let grace_duration_kills = self.grace_duration_kills.clone();
            tokio::spawn(async move {
                let duration = grace_duration.unwrap_or(Duration::from_millis(10000));
                tokio::time::sleep(duration).await;

                for (node, proc) in &running_processes {
                    if let Some(proc) = proc
                        && proc.submit(ProcessOperation::SoftKill)
                    {
                        grace_duration_kills.insert(node.clone());
                    }
                }

                let kill_duration = duration / 2;
                tokio::time::sleep(kill_duration).await;

                for (node, proc) in &running_processes {
                    if let Some(proc) = proc
                        && proc.submit(ProcessOperation::Kill)
                    {
                        grace_duration_kills.insert(node.clone());
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
        let process = node.process.take();
        self.send_stop_and_schedule_kill(
            node_id,
            process,
            clock,
            grace_duration,
            DEFAULT_STOP_GRACE,
        );
        Ok(())
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
        // trip the finish-straggler watchdog on the restarted node.
        self.all_inputs_closed_at.remove(node_id);
        self.finish_escalated.remove(node_id);
        self.send_stop_and_schedule_kill(
            node_id,
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
                    grace_duration_kills.insert(node_id.clone());
                }
                tokio::time::sleep(duration / 2).await;
                if proc.submit(ProcessOperation::Kill) {
                    grace_duration_kills.insert(node_id);
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

    pub(crate) fn open_inputs(&self, node_id: &NodeId) -> &BTreeSet<DataId> {
        self.open_inputs.get(node_id).unwrap_or(&self.empty_set)
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
                let is_source = self
                    .descriptor
                    .nodes
                    .iter()
                    .find(|n| &n.id == id)
                    .is_some_and(|n| n.inputs.is_empty());
                let last = node.last_activity.load(atomic::Ordering::Acquire);
                // last == 0 means never connected; treat as active so a
                // node that has not started is not mistaken for a wedge.
                let silent_for = if last == 0 {
                    Duration::ZERO
                } else {
                    Duration::from_millis(now_millis.saturating_sub(last))
                };
                StragglerNode {
                    id,
                    dynamic: node.node_config.dynamic,
                    is_source,
                    drained_for: self.all_inputs_closed_at.get(id).map(Instant::elapsed),
                    silent_for,
                }
            }),
            &self.finish_escalated,
            grace,
        )
    }
}

/// One running node's view for [`select_finish_stragglers`].
struct StragglerNode<'a> {
    id: &'a NodeId,
    dynamic: bool,
    /// No inputs in the descriptor — a source that never drains.
    is_source: bool,
    /// Time since `AllInputsClosed`, or `None` if it never reached drain.
    drained_for: Option<Duration>,
    /// Time since the last daemon-bound message from this node.
    silent_for: Duration,
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
        if node.is_source {
            // a live source keeps the dataflow producing
            return Vec::new();
        }
        match node.drained_for {
            // draining: ready past grace; still within grace it is progressing
            // toward exit and does not veto, but is not escalated yet
            Some(drained_for) => {
                if drained_for >= grace {
                    eligible.push(node.id.clone());
                }
            }
            // never drained: only "finished" if it has wedged silent past grace
            None => {
                if node.silent_for >= grace {
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

    const TEST_GRACE: Duration = Duration::from_millis(100);
    const PAST_GRACE: Duration = Duration::from_secs(2);
    const WITHIN_GRACE: Duration = Duration::ZERO;

    /// A non-source node draining for `age` (received `AllInputsClosed`).
    fn drained<'a>(id: &'a NodeId, age: Duration) -> StragglerNode<'a> {
        StragglerNode {
            id,
            dynamic: false,
            is_source: false,
            drained_for: Some(age),
            silent_for: Duration::ZERO,
        }
    }

    /// A non-source node that never drained, silent for `silent_for`.
    fn never_drained<'a>(id: &'a NodeId, silent_for: Duration) -> StragglerNode<'a> {
        StragglerNode {
            id,
            dynamic: false,
            is_source: false,
            drained_for: None,
            silent_for,
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
            is_source: true,
            drained_for: None,
            silent_for: PAST_GRACE,
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
            is_source: false,
            drained_for: None,
            silent_for: WITHIN_GRACE,
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
    fn silent_source_is_never_escalated() {
        // a source has no inputs and is stopped via the explicit-stop path,
        // never via finish-straggler escalation — even when silent past grace.
        let source = node_id("source");
        let source_node = StragglerNode {
            id: &source,
            dynamic: false,
            is_source: true,
            drained_for: None,
            silent_for: PAST_GRACE,
        };
        let selected =
            select_finish_stragglers([source_node].into_iter(), &BTreeSet::new(), TEST_GRACE);
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
