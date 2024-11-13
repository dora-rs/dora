use std::{path::PathBuf, time::Duration};

use uuid::Uuid;

use crate::{
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
};

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
