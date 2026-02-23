pub use crate::common::{
    DataMessage, DropToken, LogLevel, LogMessage, SharedMemoryId, Timestamped,
};
use crate::{
    DataflowId, current_crate_version,
    id::{DataId, NodeId},
    metadata::Metadata,
    versions_compatible,
};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
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
    Fail(NodeFailureError),
}

impl DaemonRequest {
    pub fn expects_tcp_bincode_reply(&self) -> bool {
        #[allow(clippy::match_like_matches_macro)]
        match self {
            DaemonRequest::SendMessage { .. }
            | DaemonRequest::NodeConfig { .. }
            | DaemonRequest::ReportDropTokens { .. }
            | DaemonRequest::Fail { .. } => false,
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
            | DaemonRequest::EventStreamDropped
            | DaemonRequest::Fail { .. } => false,
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
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

/// Reports a node failure to the Dora daemon.
///
/// Use the `node_failure_error!` macro to create an instance of this struct with
/// file, line, and column information.
///
/// This struct is intended to be used with the `DoraNode::report_failure_error` method.
#[derive(Debug, Default, Clone, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub struct NodeFailureError {
    /// A short summary of the error.
    pub summary: String,
    /// Detailed information about the error.
    pub detailed: Option<String>,
    /// The source file where the error occurred.
    pub file: Option<String>,
    /// The line number in the source file where the error occurred.
    pub line: Option<u32>,
    /// The column number in the source file line where the error occurred.
    pub column: Option<u32>,
}

impl NodeFailureError {
    /// Creates a new `NodeFailureError` with the given summary.
    ///
    /// All other fields are initialized to `None`.
    ///
    /// See the `node_failure_error!` macro for a convenient way to
    /// create an instance with file, line, and column information.
    pub fn new<S: Into<String>>(error: S) -> Self {
        Self {
            summary: error.into(),
            detailed: None,
            file: None,
            line: None,
            column: None,
        }
    }
}

impl core::fmt::Display for NodeFailureError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.summary)?;
        if let Some(file) = &self.file {
            write!(f, " at {}", file)?;
            if let Some(line) = self.line {
                write!(f, ":{}", line)?;
                if let Some(column) = self.column {
                    write!(f, ":{}", column)?;
                }
            }
        }
        if let Some(detailed) = &self.detailed {
            write!(f, ": {}", detailed)?;
        }
        Ok(())
    }
}
