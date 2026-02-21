use crate::control::ControlEvent;
use adora_core::config::NodeId;
use adora_message::{
    BuildId,
    common::DaemonId,
    coordinator_to_cli::LogMessage,
    daemon_to_coordinator::{DataflowDaemonResult, NodeMetrics},
};
use eyre::WrapErr;
use futures::Stream;
use std::collections::BTreeMap;
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;

#[derive(Debug)]
pub enum Event {
    NewDaemonConnection(TcpStream),
    DaemonConnectError(eyre::Report),
    DaemonHeartbeat {
        daemon_id: DaemonId,
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
        daemon_id: adora_message::common::DaemonId,
    },
    DataflowBuildResult {
        build_id: BuildId,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
    DataflowSpawnResult {
        dataflow_id: uuid::Uuid,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
    NodeMetrics {
        dataflow_id: uuid::Uuid,
        metrics: BTreeMap<NodeId, NodeMetrics>,
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
            Event::NewDaemonConnection(_) => "NewDaemonConnection",
            Event::DaemonConnectError(_) => "DaemonConnectError",
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

#[derive(Debug)]
pub enum DaemonRequest {
    Register {
        version_check_result: Result<(), String>,
        machine_id: Option<String>,
        connection: TcpStream,
    },
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
