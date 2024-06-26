use dora_message::uhlc;
use std::{
    borrow::Cow,
    collections::{BTreeMap, BTreeSet},
    fmt::Display,
    net::{IpAddr, Ipv4Addr},
    path::PathBuf,
    time::Duration,
};
use uuid::Uuid;

use crate::{
    config::{NodeId, OperatorId},
    descriptor::Descriptor,
};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_COORDINATOR_PORT_DEFAULT: u16 = 0xD02A;
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 0xD02B;
pub const DORA_COORDINATOR_PORT_CONTROL_DEFAULT: u16 = 0x177C;

pub const MANUAL_STOP: &str = "dora/stop";

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum ControlRequest {
    Start {
        dataflow: Descriptor,
        name: Option<String>,
        // TODO: remove this once we figure out deploying of node/operator
        // binaries from CLI to coordinator/daemon
        local_working_dir: PathBuf,
    },
    Reload {
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Check {
        dataflow_uuid: Uuid,
    },
    Stop {
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
    },
    StopByName {
        name: String,
        grace_duration: Option<Duration>,
    },
    Logs {
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
    },
    Destroy,
    List,
    DaemonConnected,
    ConnectedMachines,
    LogSubscribe {
        dataflow_id: Uuid,
        level: log::LevelFilter,
    },
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowList(pub Vec<DataflowListEntry>);

impl DataflowList {
    pub fn get_active(&self) -> Vec<DataflowId> {
        self.0
            .iter()
            .filter(|d| d.status == DataflowStatus::Running)
            .map(|d| d.id.clone())
            .collect()
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowListEntry {
    pub id: DataflowId,
    pub status: DataflowStatus,
}

#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize, PartialEq, Eq)]
pub enum DataflowStatus {
    Running,
    Finished,
    Failed,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequestReply {
    Error(String),
    CoordinatorStopped,
    DataflowStarted { uuid: Uuid },
    DataflowReloaded { uuid: Uuid },
    DataflowStopped { uuid: Uuid, result: DataflowResult },
    DataflowList(DataflowList),
    DestroyOk,
    DaemonConnected(bool),
    ConnectedMachines(BTreeSet<String>),
    Logs(Vec<u8>),
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowId {
    pub uuid: Uuid,
    pub name: Option<String>,
}

impl Display for DataflowId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(name) = &self.name {
            write!(f, "[{name}] {}", self.uuid)
        } else {
            write!(f, "[<unnamed>] {}", self.uuid)
        }
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowResult {
    pub uuid: Uuid,
    pub timestamp: uhlc::Timestamp,
    pub node_results: BTreeMap<NodeId, Result<(), NodeError>>,
}

impl DataflowResult {
    pub fn ok_empty(uuid: Uuid, timestamp: uhlc::Timestamp) -> Self {
        Self {
            uuid,
            timestamp,
            node_results: Default::default(),
        }
    }

    pub fn is_ok(&self) -> bool {
        self.node_results.values().all(|r| r.is_ok())
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowDaemonResult {
    pub timestamp: uhlc::Timestamp,
    pub node_results: BTreeMap<NodeId, Result<(), NodeError>>,
}

impl DataflowDaemonResult {
    pub fn is_ok(&self) -> bool {
        self.node_results.values().all(|r| r.is_ok())
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
            NodeErrorCause::Other { stderr } if stderr.is_empty() => {}
            NodeErrorCause::Other { stderr } => {
                let line: &str = "---------------------------------------------------------------------------------\n";
                write!(f, " with stderr output:\n{line}{stderr}{line}")?
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
