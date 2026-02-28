use adora_message::{
    BuildId,
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::ControlRequestReply,
    id::{DataId, NodeId},
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
        /// Sends `true` if the dataflow exists, `false` otherwise.
        found_tx: oneshot::Sender<bool>,
    },
    BuildLogSubscribe {
        build_id: BuildId,
        level: log::LevelFilter,
        sender: mpsc::Sender<String>,
        /// Sends `true` if the build exists, `false` otherwise.
        found_tx: oneshot::Sender<bool>,
    },
    TopicSubscribe {
        dataflow_id: Uuid,
        topics: Vec<(NodeId, DataId)>,
        /// Sends `true` if the dataflow exists and has publish_all_messages_to_zenoh enabled.
        found_tx: oneshot::Sender<bool>,
    },
    Error(eyre::Report),
}

impl From<eyre::Report> for ControlEvent {
    fn from(err: eyre::Report) -> Self {
        ControlEvent::Error(err)
    }
}
