use std::{collections::BTreeMap, path::PathBuf};

use dora_core::descriptor::Descriptor;
use dora_message::{common::GitSource, id::NodeId};

use crate::session::DataflowSession;

pub fn build_dataflow_locally(
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    working_dir: PathBuf,
    uv: bool,
) -> eyre::Result<()> {
    todo!()
}
