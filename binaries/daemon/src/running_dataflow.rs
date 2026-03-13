//! Running dataflow state and associated types.

use crate::{
    AdoraEvent, OutputId, coordinator, empty_type_info, fault_tolerance::CascadingErrorCauses,
    pending::PendingNodes, send_with_timestamp,
};
use adora_core::{
    config::{DataId, NodeId},
    descriptor::Descriptor,
    uhlc::HLC,
};
use adora_message::{
    common::{DaemonId, DropToken},
    daemon_to_node::{NodeConfig, NodeDropEvent, NodeEvent},
    descriptor::RestartPolicy,
    metadata::{self},
    node_to_daemon::Timestamped,
};
/// Default grace period before force-killing a stopped node.
const DEFAULT_STOP_GRACE: Duration = Duration::from_millis(10_000);
/// Default grace period before force-killing a restarting node.
const DEFAULT_RESTART_GRACE: Duration = Duration::from_millis(5_000);

use crossbeam::queue::ArrayQueue;
use eyre::{Context, eyre};
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
    mpsc::{self, UnboundedSender},
};
use tracing::warn;

#[cfg(feature = "telemetry")]
use tracing_opentelemetry::OpenTelemetrySpanExt;

use crate::Event;
use process_wrap::tokio::TokioChildWrapper;

pub(crate) struct InputDeadline {
    pub timeout: Duration,
    pub last_received: Instant,
}

#[derive(Debug)]
pub struct RunningNode {
    pub(crate) process: Option<ProcessHandle>,
    pub(crate) node_config: NodeConfig,
    pub(crate) pid: Option<Arc<AtomicU32>>,
    pub(crate) restart_count: Arc<AtomicU32>,
    pub(crate) restart_policy: RestartPolicy,
    pub(crate) disable_restart: Arc<AtomicBool>,
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

pub(crate) struct DropTokenInformation {
    pub owner: NodeId,
    pub pending_nodes: BTreeSet<NodeId>,
}

/// A subscriber to the `adora/logs` virtual input.
pub struct LogSubscriber {
    pub node_id: NodeId,
    pub input_id: DataId,
    pub filter: adora_message::config::LogSubscriptionFilter,
}

pub struct RunningDataflow {
    pub(crate) id: uuid::Uuid,
    pub(crate) descriptor: Descriptor,
    pub(crate) pending_nodes: PendingNodes,
    pub(crate) dataflow_started: bool,
    pub(crate) subscribe_channels: HashMap<NodeId, UnboundedSender<Timestamped<NodeEvent>>>,
    /// Per-node pending message counters (incremented on send, decremented on recv)
    pub(crate) pending_messages: HashMap<NodeId, Arc<AtomicU64>>,
    pub(crate) drop_channels: HashMap<NodeId, UnboundedSender<Timestamped<NodeDropEvent>>>,
    pub(crate) mappings: HashMap<OutputId, BTreeSet<(NodeId, DataId)>>,
    pub(crate) timers: BTreeMap<Duration, BTreeSet<(NodeId, DataId)>>,
    /// Nodes subscribing to `adora/logs` virtual input.
    pub(crate) log_subscribers: Vec<LogSubscriber>,
    pub(crate) open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    pub(crate) input_deadlines: HashMap<(NodeId, DataId), InputDeadline>,
    pub(crate) broken_inputs: HashMap<(NodeId, DataId), Duration>,
    pub(crate) running_nodes: BTreeMap<NodeId, RunningNode>,
    pub(crate) dynamic_nodes: BTreeSet<NodeId>,
    pub(crate) open_external_mappings: BTreeSet<OutputId>,
    pub(crate) pending_drop_tokens: HashMap<DropToken, DropTokenInformation>,
    pub(crate) _timer_handles: BTreeMap<Duration, futures::future::RemoteHandle<()>>,
    pub(crate) stop_sent: bool,
    pub(crate) empty_set: BTreeSet<DataId>,
    pub(crate) cascading_error_causes: CascadingErrorCauses,
    pub(crate) grace_duration_kills: Arc<crossbeam_skiplist::SkipSet<NodeId>>,
    pub(crate) node_stderr_most_recent: BTreeMap<NodeId, Arc<ArrayQueue<String>>>,
    pub(crate) publishers: BTreeMap<OutputId, zenoh::pubsub::Publisher<'static>>,
    pub(crate) finished_tx: broadcast::Sender<()>,
    pub(crate) publish_all_messages_to_zenoh: bool,
    /// Cross-daemon Zenoh network counters
    pub(crate) net_bytes_sent: Arc<AtomicU64>,
    pub(crate) net_bytes_received: Arc<AtomicU64>,
    pub(crate) net_messages_sent: Arc<AtomicU64>,
    pub(crate) net_messages_received: Arc<AtomicU64>,
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
        Self {
            id: dataflow_id,
            pending_nodes: PendingNodes::new(dataflow_id, daemon_id),
            dataflow_started: false,
            subscribe_channels: HashMap::new(),
            pending_messages: HashMap::new(),
            drop_channels: HashMap::new(),
            mappings: HashMap::new(),
            timers: BTreeMap::new(),
            log_subscribers: Vec::new(),
            open_inputs: BTreeMap::new(),
            input_deadlines: HashMap::new(),
            broken_inputs: HashMap::new(),
            running_nodes: BTreeMap::new(),
            dynamic_nodes: BTreeSet::new(),
            open_external_mappings: Default::default(),
            pending_drop_tokens: HashMap::new(),
            _timer_handles: BTreeMap::new(),
            stop_sent: false,
            empty_set: BTreeSet::new(),
            cascading_error_causes: Default::default(),
            grace_duration_kills: Default::default(),
            node_stderr_most_recent: BTreeMap::new(),
            publishers: Default::default(),
            finished_tx,
            publish_all_messages_to_zenoh: dataflow_descriptor.debug.publish_all_messages_to_zenoh,
            descriptor: dataflow_descriptor,
            net_bytes_sent: Default::default(),
            net_bytes_received: Default::default(),
            net_messages_sent: Default::default(),
            net_messages_received: Default::default(),
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
                let hlc = HLC::default();
                loop {
                    interval_stream.tick().await;

                    let span = tracing::span!(tracing::Level::TRACE, "tick");
                    let _ = span.enter();

                    let mut parameters = BTreeMap::new();
                    parameters.insert(
                        "open_telemetry_context".to_string(),
                        #[cfg(feature = "telemetry")]
                        adora_node_api::Parameter::String(
                            adora_tracing::telemetry::serialize_context(&span.context()),
                        ),
                        #[cfg(not(feature = "telemetry"))]
                        adora_node_api::Parameter::String("".into()),
                    );

                    let metadata = metadata::Metadata::from_parameters(
                        hlc.new_timestamp(),
                        empty_type_info(),
                        parameters,
                    );

                    let event = Timestamped {
                        inner: AdoraEvent::Timer {
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
            if send_with_timestamp(&channel, NodeEvent::Stop, clock).is_ok() {
                if let Some(counter) = self.pending_messages.get(&node_id) {
                    counter.fetch_add(1, atomic::Ordering::Relaxed);
                }
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
                    if let Some(proc) = proc {
                        if proc.submit(ProcessOperation::SoftKill) {
                            grace_duration_kills.insert(node.clone());
                        }
                    }
                }

                let kill_duration = duration / 2;
                tokio::time::sleep(kill_duration).await;

                for (node, proc) in &running_processes {
                    if let Some(proc) = proc {
                        if proc.submit(ProcessOperation::Kill) {
                            grace_duration_kills.insert(node.clone());
                            warn!(
                                "{node} was killed due to not stopping within the {:#?} grace period",
                                duration + kill_duration
                            );
                        }
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
        let process = node.process.take();
        // Re-enable restart only after the process handle is taken, so the
        // restart loop cannot fire before the grace timer has a chance to run.
        node.disable_restart.store(false, atomic::Ordering::Release);
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
        if let Some(channel) = self.subscribe_channels.get(node_id) {
            if send_with_timestamp(channel, NodeEvent::Stop, clock).is_ok() {
                if let Some(counter) = self.pending_messages.get(node_id) {
                    counter.fetch_add(1, atomic::Ordering::Relaxed);
                }
            }
        }

        if let Some(proc) = process {
            let duration = grace_duration.unwrap_or(default_grace);
            tokio::spawn(async move {
                tokio::time::sleep(duration).await;
                proc.submit(ProcessOperation::SoftKill);
                tokio::time::sleep(duration / 2).await;
                proc.submit(ProcessOperation::Kill);
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

    pub(crate) async fn check_drop_token(
        &mut self,
        token: DropToken,
        clock: &HLC,
    ) -> eyre::Result<()> {
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
