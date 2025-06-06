use std::{collections::BTreeMap, path::PathBuf};

use dora_core::{
    build::{BuildInfo, BuildLogger, Builder, GitManager},
    descriptor::{Descriptor, DescriptorExt},
};
use dora_message::{common::GitSource, id::NodeId};
use eyre::Context;

use crate::session::DataflowSession;

pub fn build_dataflow_locally(
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    working_dir: PathBuf,
    uv: bool,
) -> eyre::Result<BuildInfo> {
    let runtime = tokio::runtime::Runtime::new()?;

    runtime.block_on(build_dataflow(
        dataflow,
        git_sources,
        dataflow_session,
        working_dir,
        uv,
    ))
}

async fn build_dataflow(
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    base_working_dir: PathBuf,
    uv: bool,
) -> eyre::Result<BuildInfo> {
    let builder = Builder {
        session_id: dataflow_session.session_id,
        base_working_dir,
        uv,
    };
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

    let mut git_manager = GitManager::default();
    let prev_git_sources = &dataflow_session.git_sources;

    let mut tasks = Vec::new();

    // build nodes
    for node in nodes.into_values() {
        let node_id = node.id.clone();
        let git_source = git_sources.get(&node_id).cloned();
        let prev_git_source = prev_git_sources.get(&node_id).cloned();

        let task = builder
            .clone()
            .build_node(
                node,
                git_source,
                prev_git_source,
                LocalBuildLogger {
                    node_id: node_id.clone(),
                },
                &mut git_manager,
            )
            .await
            .wrap_err_with(|| format!("failed to build node `{node_id}`"))?;
        tasks.push((node_id, task));
    }

    let mut info = BuildInfo {
        node_working_dirs: Default::default(),
    };
    let mut errors = Vec::new();
    for (node_id, task) in tasks {
        match task.await {
            Ok(node) => {
                info.node_working_dirs
                    .insert(node_id, node.node_working_dir);
            }
            Err(err) => {
                errors.push((node_id, err));
            }
        }
    }
    if errors.is_empty() {
        Ok(info)
    } else {
        let mut message = "failed to build dataflow:\n".to_owned();
        for (node_id, err) in errors {
            message.push_str(&format!("- {node_id}: {err:?}\n-------------------\n\n"));
        }
        Err(eyre::eyre!(message))
    }
}

struct LocalBuildLogger {
    node_id: NodeId,
}

impl BuildLogger for LocalBuildLogger {
    type Clone = Self;

    async fn log_message(&mut self, level: log::Level, message: impl Into<String> + Send) {
        let message: String = message.into();
        println!("node {}: \t{level}: \t{message}", self.node_id);
    }

    async fn try_clone(&self) -> eyre::Result<Self::Clone> {
        Ok(LocalBuildLogger {
            node_id: self.node_id.clone(),
        })
    }
}
