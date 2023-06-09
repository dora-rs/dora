use std::{
    collections::BTreeSet,
    fmt::Display,
    net::{Ipv4Addr, SocketAddr},
    path::PathBuf,
};
use uuid::Uuid;

use crate::{
    config::{NodeId, OperatorId},
    descriptor::Descriptor,
};

pub const DORA_COORDINATOR_PORT_DEFAULT: u16 = 0xD02A;

pub const MANUAL_STOP: &str = "dora/stop";

pub fn control_socket_addr() -> SocketAddr {
    SocketAddr::new(Ipv4Addr::new(127, 0, 0, 1).into(), 6012)
}

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
    },
    StopByName {
        name: String,
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
    DataflowStarted {
        uuid: Uuid,
    },
    DataflowReloaded {
        uuid: Uuid,
    },
    DataflowStopped {
        uuid: Uuid,
        result: Result<(), String>,
    },

    DataflowList {
        dataflows: Vec<DataflowId>,
    },
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
