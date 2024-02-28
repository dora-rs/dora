use std::path::{Path, PathBuf};

use dora_core::config::NodeId;
use uuid::Uuid;

pub fn log_path(working_dir: &Path, dataflow_id: &Uuid, node_id: &NodeId) -> PathBuf {
    let dataflow_dir = working_dir.join("out").join(dataflow_id.to_string());
    dataflow_dir.join(format!("log_{node_id}.txt"))
}
