use std::{collections::BTreeMap, path::PathBuf};

use colored::Colorize;
use dora_core::{
    build::{BuildInfo, BuildLogger, Builder, GitManager, LogLevelOrStdout, PrevGitSource},
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
        let prev_git = prev_git_source.map(|prev_source| PrevGitSource {
            still_needed_for_this_build: git_sources.values().any(|s| s == &prev_source),
            git_source: prev_source,
        });

        let task = builder
            .clone()
            .build_node(
                node,
                git_source,
                prev_git,
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
    for (node_id, task) in tasks {
        let node = task
            .await
            .with_context(|| format!("failed to build node `{node_id}`"))?;
        info.node_working_dirs
            .insert(node_id, node.node_working_dir);
    }
    Ok(info)
}

struct LocalBuildLogger {
    node_id: NodeId,
}

impl BuildLogger for LocalBuildLogger {
    type Clone = Self;

    async fn log_message(
        &mut self,
        level: impl Into<LogLevelOrStdout> + Send,
        message: impl Into<String> + Send,
    ) {
        let level = match level.into() {
            LogLevelOrStdout::LogLevel(level) => match level {
                log::Level::Error => "ERROR ".red(),
                log::Level::Warn => "WARN  ".yellow(),
                log::Level::Info => "INFO  ".green(),
                log::Level::Debug => "DEBUG ".bright_blue(),
                log::Level::Trace => "TRACE ".dimmed(),
            },
            LogLevelOrStdout::Stdout => "stdout".italic().dimmed(),
        };
        let node = self.node_id.to_string().bold().bright_black();
        let message: String = message.into();
        println!("{node}: {level}   {message}");
    }

    async fn try_clone(&self) -> eyre::Result<Self::Clone> {
        Ok(LocalBuildLogger {
            node_id: self.node_id.clone(),
        })
    }
}