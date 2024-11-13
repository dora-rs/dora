pub use crate::common::{
    DataMessage, DropToken, LogLevel, LogMessage, SharedMemoryId, Timestamped,
};
use crate::{
    current_crate_version,
    id::{DataId, NodeId},
    metadata::Metadata,
    versions_compatible, DataflowId,
};

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonRequest {
    Register(NodeRegisterRequest),
    Subscribe,
    SendMessage {
        output_id: DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    },
    CloseOutputs(Vec<DataId>),
    /// Signals that the node is finished sending outputs and that it received all
    /// required drop tokens.
    OutputsDone,
    NextEvent {
        drop_tokens: Vec<DropToken>,
    },
    ReportDropTokens {
        drop_tokens: Vec<DropToken>,
    },
    SubscribeDrop,
    NextFinishedDropTokens,
    EventStreamDropped,
    NodeConfig {
        node_id: NodeId,
    },
}

impl DaemonRequest {
    pub fn expects_tcp_bincode_reply(&self) -> bool {
        #[allow(clippy::match_like_matches_macro)]
        match self {
            DaemonRequest::SendMessage { .. }
            | DaemonRequest::NodeConfig { .. }
            | DaemonRequest::ReportDropTokens { .. } => false,
            DaemonRequest::Register(NodeRegisterRequest { .. })
            | DaemonRequest::Subscribe
            | DaemonRequest::CloseOutputs(_)
            | DaemonRequest::OutputsDone
            | DaemonRequest::NextEvent { .. }
            | DaemonRequest::SubscribeDrop
            | DaemonRequest::NextFinishedDropTokens
            | DaemonRequest::EventStreamDropped => true,
        }
    }

    pub fn expects_tcp_json_reply(&self) -> bool {
        #[allow(clippy::match_like_matches_macro)]
        match self {
            DaemonRequest::NodeConfig { .. } => true,
            DaemonRequest::Register(NodeRegisterRequest { .. })
            | DaemonRequest::Subscribe
            | DaemonRequest::CloseOutputs(_)
            | DaemonRequest::OutputsDone
            | DaemonRequest::NextEvent { .. }
            | DaemonRequest::SubscribeDrop
            | DaemonRequest::NextFinishedDropTokens
            | DaemonRequest::ReportDropTokens { .. }
            | DaemonRequest::SendMessage { .. }
            | DaemonRequest::EventStreamDropped => false,
        }
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct NodeRegisterRequest {
    pub dataflow_id: DataflowId,
    pub node_id: NodeId,
    dora_version: semver::Version,
}

impl NodeRegisterRequest {
    pub fn new(dataflow_id: DataflowId, node_id: NodeId) -> Self {
        Self {
            dataflow_id,
            node_id,
            dora_version: semver::Version::parse(env!("CARGO_PKG_VERSION")).unwrap(),
        }
    }

    pub fn check_version(&self) -> Result<(), String> {
        let crate_version = current_crate_version();
        let specified_version = &self.dora_version;

        if versions_compatible(&crate_version, specified_version)? {
            Ok(())
        } else {
            Err(format!(
                "version mismatch: message format v{} is not compatible \
                with expected message format v{crate_version}",
                self.dora_version
            ))
        }
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DropEvent {
    pub tokens: Vec<DropToken>,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum InputData {
    SharedMemory(SharedMemoryInput),
    Vec(Vec<u8>),
}

impl InputData {
    pub fn drop_token(&self) -> Option<DropToken> {
        match self {
            InputData::SharedMemory(data) => Some(data.drop_token),
            InputData::Vec(_) => None,
        }
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct SharedMemoryInput {
    pub shared_memory_id: SharedMemoryId,
    pub len: usize,
    pub drop_token: DropToken,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DynamicNodeEvent {
    NodeConfig { node_id: NodeId },
}
