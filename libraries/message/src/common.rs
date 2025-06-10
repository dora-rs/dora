use core::fmt;
use std::borrow::Cow;

use aligned_vec::{AVec, ConstAlign};
use eyre::Context as _;
use uuid::Uuid;

use crate::{daemon_to_daemon::InterDaemonEvent, id::NodeId, BuildId, DataflowId};

pub use log::Level as LogLevel;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[must_use]
pub struct LogMessage {
    pub build_id: Option<BuildId>,
    pub dataflow_id: Option<DataflowId>,
    pub node_id: Option<NodeId>,
    pub daemon_id: Option<DaemonId>,
    pub level: LogLevel,
    pub target: Option<String>,
    pub module_path: Option<String>,
    pub file: Option<String>,
    pub line: Option<u32>,
    pub message: String,
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
                    write!(f, "node was killed by dora because it didn't react to a stop message in time ({signal_str})")
                } else {
                    write!(f, "exited because of signal {signal_str}")
                }
            }
            NodeExitStatus::Unknown => write!(f, "unknown exit status"),
        }?;

        match &self.cause {
            NodeErrorCause::GraceDuration => {}, // handled above
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
            },
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
            write!(f, "{id}")?;
        }
        Ok(())
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize, Clone, PartialEq, Eq)]
pub struct GitSource {
    pub repo: String,
    pub commit_hash: String,
}
