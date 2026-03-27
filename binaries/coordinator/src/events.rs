use crate::control::ControlEvent;
use crate::state::DaemonConnection;
use adora_core::config::NodeId;
use adora_message::{
    BuildId, DataflowId,
    common::DaemonId,
    coordinator_to_cli::LogMessage,
    daemon_to_coordinator::{
        DataflowDaemonResult, FaultToleranceSnapshot, NetworkMetrics, NodeMetrics,
    },
};
use eyre::WrapErr;
use futures::Stream;
use std::collections::BTreeMap;
use tokio::sync::{mpsc, oneshot};
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;

#[derive(Debug)]
pub enum Event {
    DaemonHeartbeat {
        daemon_id: DaemonId,
        ft_stats: Option<FaultToleranceSnapshot>,
    },
    Dataflow {
        uuid: Uuid,
        event: DataflowEvent,
    },
    Control(ControlEvent),
    Daemon(DaemonRequest),
    DaemonHeartbeatInterval,
    CtrlC,
    Log(LogMessage),
    DaemonExit {
        daemon_id: DaemonId,
    },
    DataflowBuildResult {
        build_id: BuildId,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
    DataflowSpawnResult {
        dataflow_id: Uuid,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
    NodeMetrics {
        dataflow_id: Uuid,
        metrics: BTreeMap<NodeId, NodeMetrics>,
        network: Option<NetworkMetrics>,
    },
    DaemonStatusReport {
        daemon_id: DaemonId,
        running_dataflows: Vec<adora_message::daemon_to_coordinator::DataflowStatusEntry>,
    },
    DaemonStateCatchUpAck {
        daemon_id: DaemonId,
        dataflow_id: DataflowId,
        ack_sequence: u64,
    },
}

impl Event {
    /// Whether this event should be logged.
    #[allow(clippy::match_like_matches_macro)]
    pub fn log(&self) -> bool {
        match self {
            Event::DaemonHeartbeatInterval => false,
            _ => true,
        }
    }

    pub(crate) fn kind(&self) -> &'static str {
        match self {
            Event::DaemonHeartbeat { .. } => "DaemonHeartbeat",
            Event::Dataflow { .. } => "Dataflow",
            Event::Control(_) => "Control",
            Event::Daemon(_) => "Daemon",
            Event::DaemonHeartbeatInterval => "DaemonHeartbeatInterval",
            Event::CtrlC => "CtrlC",
            Event::Log(_) => "Log",
            Event::DaemonExit { .. } => "DaemonExit",
            Event::DataflowBuildResult { .. } => "DataflowBuildResult",
            Event::DataflowSpawnResult { .. } => "DataflowSpawnResult",
            Event::NodeMetrics { .. } => "NodeMetrics",
            Event::DaemonStatusReport { .. } => "DaemonStatusReport",
            Event::DaemonStateCatchUpAck { .. } => "DaemonStateCatchUpAck",
        }
    }
}

#[derive(Debug)]
pub enum DataflowEvent {
    DataflowFinishedOnDaemon {
        daemon_id: DaemonId,
        result: DataflowDaemonResult,
    },
    ReadyOnDaemon {
        daemon_id: DaemonId,
        exited_before_subscribe: Vec<NodeId>,
    },
}

#[allow(private_interfaces)]
pub enum DaemonRequest {
    Register {
        version_check_result: Result<(), String>,
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        connection: DaemonConnection,
        /// Sends back the assigned DaemonId on successful registration.
        daemon_id_tx: oneshot::Sender<DaemonId>,
    },
}

// Manual Debug since DaemonConnection doesn't derive Debug
impl std::fmt::Debug for DaemonRequest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DaemonRequest::Register {
                version_check_result,
                machine_id,
                labels,
                ..
            } => f
                .debug_struct("Register")
                .field("version_check_result", version_check_result)
                .field("machine_id", machine_id)
                .field("labels", labels)
                .finish_non_exhaustive(),
        }
    }
}

pub(crate) fn set_up_ctrlc_handler() -> Result<impl Stream<Item = Event>, eyre::ErrReport> {
    let (ctrlc_tx, ctrlc_rx) = mpsc::channel(1);

    let mut ctrlc_sent = false;
    ctrlc::set_handler(move || {
        if ctrlc_sent {
            tracing::warn!("received second ctrlc signal -> aborting immediately");
            std::process::abort();
        } else {
            tracing::info!("received ctrlc signal");
            if ctrlc_tx.blocking_send(Event::CtrlC).is_err() {
                tracing::error!("failed to report ctrl-c event to adora-coordinator");
            }

            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    Ok(ReceiverStream::new(ctrlc_rx))
}
