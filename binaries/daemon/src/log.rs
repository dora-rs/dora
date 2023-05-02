use std::path::PathBuf;

use dora_core::config::NodeId;
use uuid::Uuid;

pub fn log_path(dataflow_id: &Uuid, node_id: &NodeId) -> PathBuf {
    PathBuf::from(format!("{dataflow_id}-{node_id}.txt"))
}
