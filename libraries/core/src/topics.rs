use dora_message::uhlc;
use std::{
    collections::{BTreeMap, BTreeSet},
    fmt::Display,
    path::PathBuf,
    time::Duration,
};
use uuid::Uuid;

use crate::{
    config::{NodeId, OperatorId},
    descriptor::Descriptor,
};

pub const DORA_COORDINATOR_PORT_DEFAULT: u16 = 0xD02A;
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
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequestReply {
    Error(String),
    CoordinatorStopped,
    DataflowStarted { uuid: Uuid },
    DataflowReloaded { uuid: Uuid },
    DataflowStopped { uuid: Uuid, result: DataflowResult },
    DataflowList { dataflows: Vec<DataflowId> },
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

    pub fn root_error(&self) -> RootError<'_> {
        RootError(self)
    }
}

pub struct RootError<'a>(&'a DataflowResult);

impl std::fmt::Display for RootError<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let failed = self
            .0
            .node_results
            .iter()
            .filter_map(|(id, r)| r.as_ref().err().map(|e| (id, e)));
        let total_failed = failed.clone().count();

        let mut non_cascading: Vec<_> = failed
            .clone()
            .filter(|(_, e)| !matches!(e.cause, NodeErrorCause::Cascading))
            .collect();
        non_cascading.sort_by_key(|(_, e)| e.timestamp);
        // try to print earliest non-cascading error
        if let Some((id, err)) = non_cascading.first() {
            // TODO: better error formatting
            write!(f, "Node `{id}` failed: {err:?}")?;
        } else {
            // no non-cascading errors -> print earliest cascading
            let mut all: Vec<_> = failed.collect();
            all.sort_by_key(|(_, e)| e.timestamp);
            if let Some((id, err)) = all.first() {
                // TODO: better error formatting
                write!(f, "Node `{id}` failed: {err:?}")?;
            } else {
                write!(f, "unknown error")?;
            }
        }

        if total_failed > 1 {
            write!(
                f,
                "\n\nThere are {} more errors. Check the `out/{}` folder for full details.",
                total_failed - 1,
                self.0.uuid
            )?;
        }

        Ok(())
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowDaemonResult {
    pub timestamp: uhlc::Timestamp,
    pub node_results: BTreeMap<NodeId, Result<(), NodeError>>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct NodeError {
    pub timestamp: uhlc::Timestamp,
    pub cause: NodeErrorCause,
    pub exit_status: NodeExitStatus,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum NodeErrorCause {
    /// Node failed because another node failed before,
    Cascading,
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
