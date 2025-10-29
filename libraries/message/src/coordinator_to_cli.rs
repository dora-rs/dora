use std::{
    collections::{BTreeMap, BTreeSet},
    net::IpAddr,
};

use uuid::Uuid;

pub use crate::common::{LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus};
use crate::{common::DaemonId, descriptor::Descriptor, id::NodeId, BuildId};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequestReply {
    Error(String),
    CoordinatorStopped,
    DataflowBuildTriggered {
        build_id: BuildId,
    },
    DataflowBuildFinished {
        build_id: BuildId,
        result: Result<(), String>,
    },
    DataflowStartTriggered {
        uuid: Uuid,
    },
    DataflowSpawned {
        uuid: Uuid,
    },
    DataflowReloaded {
        uuid: Uuid,
    },
    DataflowStopped {
        uuid: Uuid,
        result: DataflowResult,
    },
    DataflowList(DataflowList),
    DataflowInfo {
        uuid: Uuid,
        name: Option<String>,
        descriptor: Descriptor,
    },
    DestroyOk,
    DaemonConnected(bool),
    ConnectedDaemons(BTreeSet<DaemonId>),
    Logs(Vec<u8>),
    CliAndDefaultDaemonIps {
        default_daemon: Option<IpAddr>,
        cli: Option<IpAddr>,
    },
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
pub struct DataflowList(pub Vec<DataflowListEntry>);

impl DataflowList {
    pub fn get_active(&self) -> Vec<DataflowIdAndName> {
        self.0
            .iter()
            .filter(|d| d.status == DataflowStatus::Running)
            .map(|d| d.id.clone())
            .collect()
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowListEntry {
    pub id: DataflowIdAndName,
    pub status: DataflowStatus,
}

#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize, PartialEq, Eq)]
pub enum DataflowStatus {
    Running,
    Finished,
    Failed,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowIdAndName {
    pub uuid: Uuid,
    pub name: Option<String>,
}

impl std::fmt::Display for DataflowIdAndName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(name) = &self.name {
            write!(f, "[{name}] {}", self.uuid)
        } else {
            write!(f, "[<unnamed>] {}", self.uuid)
        }
    }
}
