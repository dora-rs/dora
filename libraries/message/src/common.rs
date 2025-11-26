use core::fmt;
use std::{borrow::Cow, collections::BTreeMap};

use aligned_vec::{AVec, ConstAlign};
use chrono::{DateTime, Utc};
use eyre::Context as _;
use serde::{Deserialize, Deserializer};
use uuid::Uuid;

use crate::{BuildId, DataflowId, daemon_to_daemon::InterDaemonEvent, id::NodeId};

pub use log::Level as LogLevel;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq)]
#[must_use]
pub struct LogMessage {
    pub build_id: Option<BuildId>,
    pub dataflow_id: Option<DataflowId>,
    pub node_id: Option<NodeId>,
    pub daemon_id: Option<DaemonId>,
    pub level: LogLevelOrStdout,
    pub target: Option<String>,
    pub module_path: Option<String>,
    pub file: Option<String>,
    pub line: Option<u32>,
    pub message: String,
    pub timestamp: DateTime<Utc>,
    pub fields: Option<BTreeMap<String, String>>,
}

#[derive(Deserialize)]
pub struct LogMessageHelper {
    build_id: Option<BuildId>,
    dataflow_id: Option<DataflowId>,
    node_id: Option<NodeId>,
    daemon_id: Option<DaemonId>,
    level: LogLevelOrStdout,
    target: Option<String>,
    module_path: Option<String>,
    file: Option<String>,
    line: Option<u32>,
    message: Option<String>,
    timestamp: DateTime<Utc>,
    fields: Option<BTreeMap<String, String>>,
}

impl From<LogMessageHelper> for LogMessage {
    fn from(helper: LogMessageHelper) -> Self {
        let fields = helper.fields.as_ref();
        LogMessage {
            build_id: helper.build_id.or(fields
                .and_then(|f| f.get("build_id").cloned())
                .map(|id| BuildId(Uuid::parse_str(&id).unwrap()))),
            dataflow_id: helper.dataflow_id.or(fields
                .and_then(|f| f.get("dataflow_id").cloned())
                .map(|id| DataflowId::from(Uuid::parse_str(&id).unwrap()))),
            node_id: helper.node_id.or(fields
                .and_then(|f| f.get("node_id").cloned())
                .map(|id| NodeId(id))),
            daemon_id: helper
                .daemon_id
                .or(fields.and_then(|f| f.get("daemon_id").cloned()).map(|id| {
                    let parts: Vec<&str> = id.splitn(2, '-').collect();
                    if parts.len() == 2 {
                        DaemonId {
                            machine_id: Some(parts[0].to_string()),
                            uuid: Uuid::parse_str(parts[1]).unwrap(),
                        }
                    } else {
                        DaemonId {
                            machine_id: None,
                            uuid: Uuid::parse_str(&parts[0]).unwrap(),
                        }
                    }
                })),
            level: helper.level,
            target: helper
                .target
                .or(fields.and_then(|f| f.get("target").cloned())),
            module_path: helper
                .module_path
                .or(fields.and_then(|f| f.get("module_path").cloned())),
            file: helper.file.or(fields.and_then(|f| f.get("file").cloned())),
            line: helper.line.or(fields
                .and_then(|f| f.get("line").cloned())
                .and_then(|s| s.parse().ok())),
            message: helper
                .message
                .or(fields.and_then(|f| f.get("message").cloned()))
                .unwrap_or_default(),
            fields: helper.fields,
            timestamp: helper.timestamp,
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "UPPERCASE")]
pub enum LogLevelOrStdout {
    #[serde(rename = "stdout")]
    Stdout,
    #[serde(untagged)]
    LogLevel(LogLevel),
}

impl From<LogLevel> for LogLevelOrStdout {
    fn from(level: LogLevel) -> Self {
        Self::LogLevel(level)
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct NodeError {
    pub timestamp: uhlc::Timestamp,
    pub cause: NodeErrorCause,
    pub exit_status: NodeExitStatus,
}

impl std::fmt::Display for NodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let NodeErrorCause::FailedToSpawn(err) = &self.cause {
            return write!(f, "failed to spawn node: {err}");
        }
        match &self.exit_status {
            NodeExitStatus::Success => write!(f, "<success>"),
            NodeExitStatus::IoError(err) => write!(f, "I/O error while reading exit status: {err}"),
            NodeExitStatus::ExitCode(code) => write!(f, "exited with code {code}"),
            NodeExitStatus::Signal(signal) => {
                let signal_str: Cow<_> = match signal {
                    1 => "SIGHUP".into(),
                    2 => "SIGINT".into(),
                    3 => "SIGQUIT".into(),
                    4 => "SIGILL".into(),
                    6 => "SIGABRT".into(),
                    8 => "SIGFPE".into(),
                    9 => "SIGKILL".into(),
                    11 => "SIGSEGV".into(),
                    13 => "SIGPIPE".into(),
                    14 => "SIGALRM".into(),
                    15 => "SIGTERM".into(),
                    22 => "SIGABRT".into(),
                    23 => "NSIG".into(),
                    other => other.to_string().into(),
                };
                if matches!(self.cause, NodeErrorCause::GraceDuration) {
                    write!(
                        f,
                        "node was killed by dora because it didn't react to a stop message in time ({signal_str})"
                    )
                } else {
                    write!(f, "exited because of signal {signal_str}")
                }
            }
            NodeExitStatus::Unknown => write!(f, "unknown exit status"),
        }?;

        match &self.cause {
            NodeErrorCause::GraceDuration => {} // handled above
            NodeErrorCause::Cascading { caused_by_node } => write!(
                f,
                ". This error occurred because node `{caused_by_node}` exited before connecting to dora."
            )?,
            NodeErrorCause::FailedToSpawn(_) => unreachable!(), // handled above
            NodeErrorCause::Other { stderr } if stderr.is_empty() => {}
            NodeErrorCause::Other { stderr } => {
                let line: &str = "---------------------------------------------------------------------------------\n";
                let stderr = stderr.trim_end();
                write!(f, " with stderr output:\n{line}{stderr}\n{line}")?
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum NodeErrorCause {
    /// Node was killed because it didn't react to a stop message in time.
    GraceDuration,
    /// Node failed because another node failed before,
    Cascading {
        caused_by_node: NodeId,
    },
    FailedToSpawn(String),
    Other {
        stderr: String,
    },
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum NodeExitStatus {
    Success,
    IoError(String),
    ExitCode(i32),
    Signal(i32),
    Unknown,
}

impl NodeExitStatus {
    pub fn is_success(&self) -> bool {
        matches!(self, NodeExitStatus::Success)
    }
}

impl From<Result<std::process::ExitStatus, std::io::Error>> for NodeExitStatus {
    fn from(result: Result<std::process::ExitStatus, std::io::Error>) -> Self {
        match result {
            Ok(status) => {
                if status.success() {
                    NodeExitStatus::Success
                } else if let Some(code) = status.code() {
                    Self::ExitCode(code)
                } else {
                    #[cfg(unix)]
                    {
                        use std::os::unix::process::ExitStatusExt;
                        if let Some(signal) = status.signal() {
                            return Self::Signal(signal);
                        }
                    }
                    Self::Unknown
                }
            }
            Err(err) => Self::IoError(err.to_string()),
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Timestamped<T> {
    pub inner: T,
    pub timestamp: uhlc::Timestamp,
}

impl<T> Timestamped<T>
where
    T: serde::Serialize,
{
    pub fn serialize(&self) -> Vec<u8> {
        bincode::serialize(self).unwrap()
    }
}

impl Timestamped<InterDaemonEvent> {
    pub fn deserialize_inter_daemon_event(bytes: &[u8]) -> eyre::Result<Self> {
        bincode::deserialize(bytes).wrap_err("failed to deserialize InterDaemonEvent")
    }
}

pub type SharedMemoryId = String;

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub enum DataMessage {
    Vec(AVec<u8, ConstAlign<128>>),
    SharedMemory {
        shared_memory_id: String,
        len: usize,
        drop_token: DropToken,
    },
}

impl DataMessage {
    pub fn drop_token(&self) -> Option<DropToken> {
        match self {
            DataMessage::Vec(_) => None,
            DataMessage::SharedMemory { drop_token, .. } => Some(*drop_token),
        }
    }
}

impl fmt::Debug for DataMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Vec(v) => f
                .debug_struct("Vec")
                .field("len", &v.len())
                .finish_non_exhaustive(),
            Self::SharedMemory {
                shared_memory_id,
                len,
                drop_token,
            } => f
                .debug_struct("SharedMemory")
                .field("shared_memory_id", shared_memory_id)
                .field("len", len)
                .field("drop_token", drop_token)
                .finish(),
        }
    }
}

#[derive(
    Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct DropToken(Uuid);

impl DropToken {
    pub fn generate() -> Self {
        Self(Uuid::new_v7(uuid::Timestamp::now(uuid::NoContext)))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub struct DaemonId {
    machine_id: Option<String>,
    uuid: Uuid,
}

impl DaemonId {
    pub fn new(machine_id: Option<String>) -> Self {
        DaemonId {
            machine_id,
            uuid: Uuid::new_v4(),
        }
    }

    pub fn matches_machine_id(&self, machine_id: &str) -> bool {
        self.machine_id
            .as_ref()
            .map(|id| id == machine_id)
            .unwrap_or_default()
    }

    pub fn machine_id(&self) -> Option<&str> {
        self.machine_id.as_deref()
    }
}

impl std::fmt::Display for DaemonId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(id) = &self.machine_id {
            write!(f, "{id}-")?;
        }
        write!(f, "{}", self.uuid)
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize, Clone, PartialEq, Eq)]
pub struct GitSource {
    pub repo: String,
    pub commit_hash: String,
}

// Test roundtrip serialization of LogMessage
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_log_message_serialization() {
        let log_message = LogMessage {
            build_id: Some(BuildId(Uuid::new_v4())),
            dataflow_id: Some(DataflowId::from(Uuid::new_v4())),
            node_id: Some(NodeId("node-1".to_string())),
            daemon_id: Some(DaemonId::new(Some("machine-1".to_string()))),
            level: LogLevelOrStdout::LogLevel(LogLevel::Info),
            target: Some("target".to_string()),
            module_path: Some("module::path".to_string()),
            file: Some("file.rs".to_string()),
            line: Some(42),
            message: "This is a log message".to_string(),
            timestamp: Utc::now(),
            fields: Some(BTreeMap::from([("key".to_string(), "value".to_string())])),
        };
        let serialized = serde_yaml::to_string(&log_message).unwrap();
        let deserialized: LogMessageHelper = serde_yaml::from_str(&serialized).unwrap();
        assert_eq!(log_message, LogMessage::from(deserialized));
    }
}
