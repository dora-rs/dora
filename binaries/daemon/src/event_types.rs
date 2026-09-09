//! Event types and channel helpers for the daemon event loop.

use std::{
    sync::{Arc, OnceLock, atomic::AtomicU64},
    time::Duration,
};

use dora_core::uhlc::HLC;
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{DataMessage, LogMessage},
    daemon_to_node::{DaemonReply, NodeEvent},
    id::{DataId, NodeId},
    metadata,
    node_to_daemon::Timestamped,
};
use tokio::sync::{OwnedSemaphorePermit, Semaphore, mpsc, oneshot};

use dora_core::build::BuildInfo;
use dora_message::common::{NodeError, NodeExitStatus};
pub use dora_message::daemon_to_daemon::InterDaemonEvent;

use crate::{
    coordinator::CoordinatorEvent, local_listener::DynamicNodeEventWrapper,
    running_dataflow::RunningNode,
};

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct OutputId(pub NodeId, pub DataId);

#[allow(clippy::large_enum_variant)]
#[derive(Debug)]
pub enum Event {
    Node {
        dataflow_id: DataflowId,
        node_id: NodeId,
        /// The process incarnation this event's connection belongs to,
        /// stamped by the per-spawn listener. Events from a superseded
        /// incarnation (replaced or re-added id) must not mutate the
        /// current entry's state (dora-rs/dora#2926, #2927).
        generation: u64,
        event: DaemonNodeEvent,
    },
    Coordinator(CoordinatorEvent),
    Daemon(InterDaemonEvent),
    Dora(DoraEvent),
    DynamicNode(DynamicNodeEventWrapper),
    HeartbeatInterval,
    /// Re-check whether a pending `Destroy`'s nodes have exited (#2980).
    DestroyTick,
    MetricsInterval,
    NodeHealthCheckInterval,
    CtrlC,
    StopAfter(Duration),
    SecondCtrlC,
    DaemonError(eyre::Report),
    SpawnNodeResult {
        dataflow_id: DataflowId,
        node_id: NodeId,
        dynamic_node: bool,
        result: Result<RunningNode, NodeError>,
    },
    BuildDataflowResult {
        build_id: BuildId,
        session_id: SessionId,
        result: eyre::Result<BuildInfo>,
    },
    SpawnDataflowResult {
        dataflow_id: uuid::Uuid,
        result: eyre::Result<()>,
    },
    NodeStopped {
        dataflow_id: uuid::Uuid,
        node_id: NodeId,
    },
    /// A data-plane output observed by the daemon's debug subscriber.
    ///
    /// Only emitted when the dataflow has `enable_debug_inspection` set.
    /// Since #1787 routes node→node data directly over Zenoh (bypassing the
    /// daemon's `send_out`), the daemon subscribes to its local nodes' output
    /// topics so `dora topic info/echo/hz` can still observe live traffic.
    /// Forwarded only to coordinator debug watchers — never re-delivered to
    /// local receivers, which already got the payload directly via Zenoh.
    DebugTopicData {
        dataflow_id: DataflowId,
        output_id: OutputId,
        metadata: metadata::Metadata,
        data: Option<Vec<u8>>,
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
            Event::Coordinator(_) => "Coordinator",
            Event::Daemon(_) => "Daemon",
            Event::Dora(_) => "Dora",
            Event::DynamicNode(_) => "DynamicNode",
            Event::HeartbeatInterval => "HeartbeatInterval",
            Event::DestroyTick => "DestroyTick",
            Event::MetricsInterval => "MetricsInterval",
            Event::NodeHealthCheckInterval => "NodeHealthCheckInterval",
            Event::CtrlC => "CtrlC",
            Event::StopAfter(_) => "StopAfter",
            Event::SecondCtrlC => "SecondCtrlC",
            Event::DaemonError(_) => "DaemonError",
            Event::SpawnNodeResult { .. } => "SpawnNodeResult",
            Event::BuildDataflowResult { .. } => "BuildDataflowResult",
            Event::SpawnDataflowResult { .. } => "SpawnDataflowResult",
            Event::NodeStopped { .. } => "NodeStopped",
            Event::DebugTopicData { .. } => "DebugTopicData",
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
        event_sender: NodeEventSender,
        pending_counter: Arc<AtomicU64>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    CloseOutputs {
        outputs: Vec<DataId>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SendOut {
        output_id: DataId,
        metadata: metadata::Metadata,
        data: Option<DataMessage>,
        replay_ingress_permit: Option<OwnedSemaphorePermit>,
    },
    OutputSent {
        output_id: DataId,
        metadata: metadata::Metadata,
    },
    EventStreamDropped {
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    ExtensionStore {
        namespace: String,
        key: String,
        value: Vec<u8>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    ExtensionLoad {
        namespace: String,
        key: String,
        remove: bool,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    ExtensionDrop {
        namespace: String,
        key: String,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    /// An opaque call from a node to the extension registered under
    /// `namespace`. The daemon routes the bytes and hands back whatever
    /// the extension returns; it interprets neither.
    ExtensionRequest {
        namespace: String,
        payload: Vec<u8>,
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
    LogBroadcast {
        dataflow_id: DataflowId,
        log_message: LogMessage,
    },
    SpawnedNodeResult {
        dataflow_id: DataflowId,
        node_id: NodeId,
        generation: u64,
        dynamic_node: bool,
        exit_status: NodeExitStatus,
        restart: bool,
        restart_count: u32,
        pid: u32,
    },
    /// The per-node `restart_loop` spawned a fresh process after an exit
    /// and now wants the daemon to swap the tracked `ProcessHandle` in
    /// `running_nodes` so subsequent kill/stop operations reach the new
    /// incarnation rather than the dead predecessor.
    ///
    /// Restores per-incarnation isolation of the process-operation
    /// channel and closes the stale-kill race in dora-rs/adora#152.
    ProcessHandleReplaced {
        dataflow_id: DataflowId,
        node_id: NodeId,
        previous_generation: u64,
        new_generation: u64,
        new_handle: crate::ProcessHandle,
    },
}

#[must_use]
pub(crate) enum RunStatus {
    Continue,
    Exit,
}

/// Default capacity for per-node event channels.
pub(crate) const NODE_EVENT_CHANNEL_CAPACITY: usize = 1000;

/// Headroom reserved for control events (Stop, InputClosed, etc.).
pub(crate) const CONTROL_EVENT_HEADROOM: usize = 50;

/// Combined byte limit for a verified replay consumer's subscription and listener queues.
pub(crate) const REPLAY_NODE_EVENT_BYTE_LIMIT: usize = 4 * dora_message::MAX_MESSAGE_BYTES;

/// Internal queue item. Its permit never crosses the daemon wire.
pub(crate) struct QueuedNodeEvent {
    event: Timestamped<NodeEvent>,
    wire_size: Option<usize>,
    _permit: Option<OwnedSemaphorePermit>,
}

impl QueuedNodeEvent {
    #[cfg(test)]
    pub(crate) fn unbudgeted(event: Timestamped<NodeEvent>) -> Self {
        Self {
            event,
            wire_size: None,
            _permit: None,
        }
    }

    pub(crate) fn wire_size(&self) -> Option<usize> {
        self.wire_size
    }

    pub(crate) fn ensure_wire_size(&mut self) -> usize {
        if let Some(size) = self.wire_size {
            return size;
        }
        let size = dora_message::serialized_size(&self.event).unwrap_or_else(|err| {
            tracing::warn!("cannot size queued node event ({err}); using the size hint");
            self.event.inner.encode_size_hint()
        });
        self.wire_size = Some(size);
        size
    }

    pub(crate) fn event(&self) -> &Timestamped<NodeEvent> {
        &self.event
    }

    pub(crate) fn into_event(mut self) -> Timestamped<NodeEvent> {
        self._permit.take();
        self.event
    }
}

#[derive(Clone, Debug)]
pub(crate) struct NodeEventSender {
    inner: mpsc::Sender<QueuedNodeEvent>,
    budget: Arc<OnceLock<Arc<Semaphore>>>,
}

#[doc(hidden)]
/// Internal receiver exposed only for daemon routing benchmarks.
pub struct NodeEventReceiver {
    inner: mpsc::Receiver<QueuedNodeEvent>,
}

pub(crate) fn node_event_channel(capacity: usize) -> (NodeEventSender, NodeEventReceiver) {
    let (tx, rx) = mpsc::channel(capacity);
    let budget = Arc::new(OnceLock::new());
    (
        NodeEventSender { inner: tx, budget },
        NodeEventReceiver { inner: rx },
    )
}

impl NodeEventSender {
    pub(crate) fn enable_data_byte_limit(&self, limit: usize) {
        let _ = self.budget.set(Arc::new(Semaphore::new(limit)));
    }

    pub(crate) fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    #[cfg(test)]
    pub(crate) fn available_data_bytes(&self) -> Option<usize> {
        self.budget.get().map(|budget| budget.available_permits())
    }

    #[allow(clippy::result_large_err)]
    pub(crate) fn try_send(
        &self,
        event: Timestamped<NodeEvent>,
    ) -> Result<(), mpsc::error::TrySendError<Timestamped<NodeEvent>>> {
        let queued = self.queue_event(event)?;
        self.inner.try_send(queued).map_err(|err| match err {
            mpsc::error::TrySendError::Full(queued) => {
                mpsc::error::TrySendError::Full(queued.into_event())
            }
            mpsc::error::TrySendError::Closed(queued) => {
                mpsc::error::TrySendError::Closed(queued.into_event())
            }
        })
    }

    #[allow(clippy::result_large_err)]
    pub(crate) async fn send(
        &self,
        event: Timestamped<NodeEvent>,
    ) -> Result<(), mpsc::error::SendError<Timestamped<NodeEvent>>> {
        let queued = self.queue_event(event).map_err(|err| {
            let event = match err {
                mpsc::error::TrySendError::Full(event)
                | mpsc::error::TrySendError::Closed(event) => event,
            };
            mpsc::error::SendError(event)
        })?;
        self.inner
            .send(queued)
            .await
            .map_err(|err| mpsc::error::SendError(err.0.into_event()))
    }

    #[allow(clippy::result_large_err)]
    fn queue_event(
        &self,
        event: Timestamped<NodeEvent>,
    ) -> Result<QueuedNodeEvent, mpsc::error::TrySendError<Timestamped<NodeEvent>>> {
        let (wire_size, permit) = if let (NodeEvent::Input { id, .. }, Some(budget)) =
            (&event.inner, self.budget.get())
        {
            let wire_size = match dora_message::serialized_size(&event) {
                Ok(size) => size,
                Err(err) => {
                    tracing::error!(input = %id, "cannot size verified replay input: {err}");
                    return Err(mpsc::error::TrySendError::Full(event));
                }
            };
            let Ok(wire_size_u32) = u32::try_from(wire_size) else {
                tracing::error!(input = %id, wire_size, "verified replay input is too large");
                return Err(mpsc::error::TrySendError::Full(event));
            };
            let Ok(permit) = budget.clone().try_acquire_many_owned(wire_size_u32) else {
                tracing::warn!(
                    input = %id,
                    wire_size,
                    available_bytes = budget.available_permits(),
                    "verified replay input byte limit reached"
                );
                return Err(mpsc::error::TrySendError::Full(event));
            };
            (Some(wire_size), Some(permit))
        } else {
            (None, None)
        };
        Ok(QueuedNodeEvent {
            event,
            wire_size,
            _permit: permit,
        })
    }
}

impl NodeEventReceiver {
    pub(crate) fn try_recv_queued(&mut self) -> Result<QueuedNodeEvent, mpsc::error::TryRecvError> {
        self.inner.try_recv()
    }

    pub(crate) async fn recv_queued(&mut self) -> Option<QueuedNodeEvent> {
        self.inner.recv().await
    }

    pub(crate) fn poll_recv_queued(
        &mut self,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<QueuedNodeEvent>> {
        self.inner.poll_recv(cx)
    }

    #[cfg(any(test, feature = "bench"))]
    /// Receives one queued event without waiting.
    pub fn try_recv(&mut self) -> Result<Timestamped<NodeEvent>, mpsc::error::TryRecvError> {
        self.try_recv_queued().map(QueuedNodeEvent::into_event)
    }

    #[cfg(any(test, feature = "bench"))]
    /// Waits for one queued event.
    pub async fn recv(&mut self) -> Option<Timestamped<NodeEvent>> {
        self.recv_queued().await.map(QueuedNodeEvent::into_event)
    }
}

/// Send a node event with timestamp. Returns Ok(true) if delivered,
/// Ok(false) if dropped (channel full/headroom), Err if channel closed.
#[allow(clippy::result_large_err)]
pub(crate) fn send_with_timestamp(
    sender: &NodeEventSender,
    event: NodeEvent,
    clock: &HLC,
) -> Result<bool, mpsc::error::SendError<Timestamped<NodeEvent>>> {
    let is_control = !matches!(event, NodeEvent::Input { .. });
    let msg = Timestamped {
        inner: event,
        timestamp: clock.new_timestamp(),
    };

    if !is_control && sender.capacity() < CONTROL_EVENT_HEADROOM {
        tracing::warn!(
            "event channel low on capacity, dropping data event to preserve control headroom"
        );
        return Ok(false);
    }

    match sender.try_send(msg) {
        Ok(()) => Ok(true),
        Err(mpsc::error::TrySendError::Closed(msg)) => Err(mpsc::error::SendError(msg)),
        Err(mpsc::error::TrySendError::Full(msg)) => {
            if is_control {
                tracing::error!(
                    "CRITICAL: control event dropped despite headroom reservation: {:?}",
                    msg.inner
                );
            } else {
                tracing::warn!("event channel full, dropping data event (slow receiver)");
            }
            Ok(false)
        }
    }
}

/// Outbound Zenoh message for the drain task.
pub(crate) struct ZenohOutbound {
    pub publisher: Arc<zenoh::pubsub::Publisher<'static>>,
    pub serialized: Vec<u8>,
    pub payload_len: u64,
    pub net_bytes_sent: Arc<AtomicU64>,
    pub net_messages_sent: Arc<AtomicU64>,
    pub net_publish_failures: Arc<AtomicU64>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use aligned_vec::AVec;
    use dora_message::{
        id::DataId,
        metadata::{Metadata, Parameter},
    };

    fn input_with_metadata(clock: &HLC, metadata_bytes: usize) -> Timestamped<NodeEvent> {
        let mut metadata = Metadata::new(clock.new_timestamp());
        metadata.parameters.insert(
            "blob".to_string(),
            Parameter::String("x".repeat(metadata_bytes)),
        );
        Timestamped {
            inner: NodeEvent::Input {
                id: DataId::from("in".to_string()),
                metadata: Arc::new(metadata),
                data: None,
            },
            timestamp: clock.new_timestamp(),
        }
    }

    fn input_with_payload(clock: &HLC, payload_bytes: usize) -> Timestamped<NodeEvent> {
        Timestamped {
            inner: NodeEvent::Input {
                id: DataId::from("in".to_string()),
                metadata: Arc::new(Metadata::new(clock.new_timestamp())),
                data: Some(Arc::new(DataMessage::Vec(AVec::from_slice(
                    128,
                    &vec![0; payload_bytes],
                )))),
            },
            timestamp: clock.new_timestamp(),
        }
    }

    #[tokio::test]
    async fn byte_permit_lives_until_queued_event_is_consumed() {
        let clock = HLC::default();
        let event = input_with_metadata(&clock, 128);
        let size = dora_message::serialized_size(&event).unwrap();
        let (tx, mut rx) = node_event_channel(2);
        tx.enable_data_byte_limit(size);

        tx.try_send(event.clone()).unwrap();
        assert_eq!(tx.available_data_bytes(), Some(0));
        assert!(tx.try_send(event.clone()).is_err());

        let queued = rx.recv_queued().await.unwrap();
        assert_eq!(tx.available_data_bytes(), Some(0));
        let _event = queued.into_event();
        assert_eq!(tx.available_data_bytes(), Some(size));
        tx.try_send(event).unwrap();
    }

    #[tokio::test]
    async fn failed_channel_send_releases_reserved_bytes() {
        let clock = HLC::default();
        let event = input_with_metadata(&clock, 128);
        let size = dora_message::serialized_size(&event).unwrap();
        let (tx, mut rx) = node_event_channel(1);
        tx.enable_data_byte_limit(size * 2);

        tx.try_send(event.clone()).unwrap();
        assert!(tx.try_send(event).is_err());
        assert_eq!(tx.available_data_bytes(), Some(size));

        drop(rx.recv_queued().await.unwrap());
        assert_eq!(tx.available_data_bytes(), Some(size * 2));
    }

    #[test]
    fn dropping_channel_queue_releases_reserved_bytes() {
        let clock = HLC::default();
        let event = input_with_metadata(&clock, 128);
        let size = dora_message::serialized_size(&event).unwrap();
        let (tx, rx) = node_event_channel(1);
        tx.enable_data_byte_limit(size);

        tx.try_send(event).unwrap();
        assert_eq!(tx.available_data_bytes(), Some(0));
        drop(rx);
        assert_eq!(tx.available_data_bytes(), Some(size));
    }

    #[test]
    fn metadata_bytes_count_toward_admission() {
        let clock = HLC::default();
        let event = input_with_metadata(&clock, 4096);
        let size = dora_message::serialized_size(&event).unwrap();
        assert!(size > 4096);
        let (tx, _rx) = node_event_channel(2);
        tx.enable_data_byte_limit(size - 1);

        assert!(tx.try_send(event).is_err());
        assert_eq!(tx.available_data_bytes(), Some(size - 1));
    }

    #[test]
    fn payload_bytes_count_toward_admission() {
        let clock = HLC::default();
        let event = input_with_payload(&clock, 1024 * 1024);
        let size = dora_message::serialized_size(&event).unwrap();
        assert!(size > 1024 * 1024);
        let (tx, _rx) = node_event_channel(2);
        tx.enable_data_byte_limit(size - 1);

        assert!(tx.try_send(event).is_err());
        assert_eq!(tx.available_data_bytes(), Some(size - 1));
    }

    #[test]
    fn control_event_does_not_use_data_byte_budget() {
        let clock = HLC::default();
        let (tx, mut rx) = node_event_channel(2);
        tx.enable_data_byte_limit(1);
        tx.try_send(Timestamped {
            inner: NodeEvent::Stop,
            timestamp: clock.new_timestamp(),
        })
        .unwrap();

        assert_eq!(tx.available_data_bytes(), Some(1));
        assert!(matches!(rx.try_recv().unwrap().inner, NodeEvent::Stop));
    }
}
