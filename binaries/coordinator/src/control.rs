use adora_message::{
    BuildId, cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
};
use tokio::sync::{mpsc, oneshot};
use uuid::Uuid;

#[derive(Debug)]
pub enum ControlEvent {
    IncomingRequest {
        request: ControlRequest,
        reply_sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
    },
    LogSubscribe {
        dataflow_id: Uuid,
        level: log::LevelFilter,
        sender: mpsc::Sender<String>,
    },
    BuildLogSubscribe {
        build_id: BuildId,
        level: log::LevelFilter,
        sender: mpsc::Sender<String>,
    },
    Error(eyre::Report),
}

impl From<eyre::Report> for ControlEvent {
    fn from(err: eyre::Report) -> Self {
        ControlEvent::Error(err)
    }
}
