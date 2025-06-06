use std::{
    collections::{BTreeMap, BTreeSet},
    future::Future,
    path::PathBuf,
};

use dora_core::{
    build::{BuildInfo, BuildLogger, Builder, GitManager},
    descriptor::{self, Descriptor, NodeExt, ResolvedNode, SINGLE_OPERATOR_DEFAULT_ID},
};
use dora_message::{
    common::GitSource,
    id::{NodeId, OperatorId},
    BuildId, SessionId,
};
use eyre::Context;
use futures::executor::block_on;

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
        dataflow_session.session_id,
        working_dir,
        nodes,
        git_sources,
        prev_git_sources,
        local_nodes,
        uv,
    ))
}

async fn build_dataflow(
    session_id: SessionId,
    base_working_dir: PathBuf,
    nodes: BTreeMap<NodeId, ResolvedNode>,
    git_sources: BTreeMap<NodeId, GitSource>,
    prev_git_sources: BTreeMap<NodeId, GitSource>,
    local_nodes: BTreeSet<NodeId>,
    uv: bool,
) -> eyre::Result<BuildInfo> {
    let builder = Builder {
        session_id,
        base_working_dir,
        uv,
    };

    let mut git_manager = GitManager::default();

    let mut tasks = Vec::new();

    // build nodes
    for node in nodes.into_values().filter(|n| local_nodes.contains(&n.id)) {
        let node_id = node.id.clone();
        let git_source = git_sources.get(&node_id).cloned();
        let prev_git_source = prev_git_sources.get(&node_id).cloned();

        let task = builder
            .clone()
            .build_node(
                node,
                git_source,
                prev_git_source,
                LocalBuildLogger,
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

struct LocalBuildLogger;

impl BuildLogger for LocalBuildLogger {
    type Clone = Self;

    fn log_message(
        &mut self,
        level: log::Level,
        message: impl Into<String> + Send,
    ) -> impl Future<Output = ()> + Send {
        async move {
            let message: String = message.into();
            println!("{level}: \t{message}");
        }
    }

    fn try_clone(&self) -> impl Future<Output = eyre::Result<Self::Clone>> + Send {
        async { Ok(LocalBuildLogger) }
    }
}
