//! Event types and channel helpers for the daemon event loop.

use std::{
    sync::{Arc, atomic::AtomicU64},
    time::Duration,
};

use adora_core::uhlc::HLC;
use adora_message::{
    BuildId, DataflowId, SessionId,
    common::{DataMessage, DropToken, LogMessage},
    daemon_to_node::{DaemonReply, NodeDropEvent, NodeEvent},
    id::{DataId, NodeId},
    metadata,
    node_to_daemon::Timestamped,
};
use tokio::sync::{mpsc, oneshot};

use adora_core::build::BuildInfo;
use adora_message::common::{NodeError, NodeExitStatus};
pub use adora_message::daemon_to_daemon::InterDaemonEvent;

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
        event: DaemonNodeEvent,
    },
    Coordinator(CoordinatorEvent),
    Daemon(InterDaemonEvent),
    Adora(AdoraEvent),
    DynamicNode(DynamicNodeEventWrapper),
    HeartbeatInterval,
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
}

impl From<AdoraEvent> for Event {
    fn from(event: AdoraEvent) -> Self {
        Event::Adora(event)
    }
}

impl Event {
    pub fn kind(&self) -> &'static str {
        match self {
            Event::Node { .. } => "Node",
            Event::Coordinator(_) => "Coordinator",
            Event::Daemon(_) => "Daemon",
            Event::Adora(_) => "Adora",
            Event::DynamicNode(_) => "DynamicNode",
            Event::HeartbeatInterval => "HeartbeatInterval",
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
        event_sender: mpsc::Sender<Timestamped<NodeEvent>>,
        pending_counter: Arc<AtomicU64>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SubscribeDrop {
        event_sender: mpsc::UnboundedSender<Timestamped<NodeDropEvent>>,
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
    },
    ReportDrop {
        tokens: Vec<DropToken>,
    },
    EventStreamDropped {
        reply_sender: oneshot::Sender<DaemonReply>,
    },
}

#[derive(Debug)]
pub enum AdoraEvent {
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
        dynamic_node: bool,
        exit_status: NodeExitStatus,
        restart: bool,
        restart_count: u32,
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

/// Send a node event with timestamp. Returns Ok(true) if delivered,
/// Ok(false) if dropped (channel full/headroom), Err if channel closed.
#[allow(clippy::result_large_err)]
pub(crate) fn send_with_timestamp(
    sender: &mpsc::Sender<Timestamped<NodeEvent>>,
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

/// Send a drop event with timestamp (unbounded — critical for shmem cleanup).
pub(crate) fn send_drop_with_timestamp(
    sender: &mpsc::UnboundedSender<Timestamped<NodeDropEvent>>,
    event: NodeDropEvent,
    clock: &HLC,
) -> Result<(), mpsc::error::SendError<Timestamped<NodeDropEvent>>> {
    sender.send(Timestamped {
        inner: event,
        timestamp: clock.new_timestamp(),
    })
}

/// Outbound Zenoh message for the drain task.
pub(crate) struct ZenohOutbound {
    pub publisher: Arc<zenoh::pubsub::Publisher<'static>>,
    pub serialized: Vec<u8>,
    pub payload_len: u64,
    pub net_bytes_sent: Arc<AtomicU64>,
    pub net_messages_sent: Arc<AtomicU64>,
}
