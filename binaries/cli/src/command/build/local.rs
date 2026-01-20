use std::{collections::BTreeMap, path::PathBuf, sync::Arc};

use colored::Colorize;
use dora_core::{
    build::{BuildInfo, BuildLogger, Builder, GitManager, LogLevelOrStdout, PrevGitSource},
    descriptor::{Descriptor, DescriptorExt},
};
use dora_message::{common::GitSource, id::NodeId};
use eyre::Context;

use crate::{
    progress::{MultiProgress, ProgressBar},
    session::DataflowSession,
};

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
    let node_count = nodes.len() as u64;

    let mut git_manager = GitManager::default();
    let prev_git_sources = &dataflow_session.git_sources;

    // Create multi-progress for showing all nodes being built
    let multi = Arc::new(MultiProgress::new());
    let overall_pb = multi.add_bar(node_count, "Building nodes");

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

        // Create a progress spinner for this specific node
        let node_pb = Arc::new(multi.add_spinner(format!("Building {}", node_id)));

        let task = builder
            .clone()
            .build_node(
                node,
                git_source,
                prev_git,
                LocalBuildLogger {
                    node_id: node_id.clone(),
                    progress_bar: Some(node_pb),
                    multi: Some(multi.clone()),
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
        overall_pb.inc(1);
    }

    overall_pb.finish_with_message(format!("Built {} nodes successfully", node_count));
    Ok(info)
}

struct LocalBuildLogger {
    node_id: NodeId,
    progress_bar: Option<Arc<ProgressBar>>,
    multi: Option<Arc<MultiProgress>>,
}

impl BuildLogger for LocalBuildLogger {
    type Clone = Self;

    async fn log_message(
        &mut self,
        level: impl Into<LogLevelOrStdout> + Send,
        message: impl Into<String> + Send,
    ) {
        let message_str: String = message.into();
        let log_level = level.into();

        // Format the log message with colors
        let level_colored = match log_level {
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
        let formatted_msg = format!("{node}: {level_colored}   {message_str}");

        // Print using MultiProgress::println to keep progress bars pinned at bottom
        // Falls back to regular println! for non-terminal outputs
        if let Some(multi) = &self.multi {
            let _ = multi.println(&formatted_msg);
        } else {
            println!("{}", formatted_msg);
        }

        // Also update progress bar message if available
        if let Some(pb) = &self.progress_bar {
            let level_indicator = match log_level {
                LogLevelOrStdout::LogLevel(level) => match level {
                    log::Level::Error => "ERROR",
                    log::Level::Warn => "WARN",
                    log::Level::Info => "",
                    log::Level::Debug => "DEBUG",
                    log::Level::Trace => "TRACE",
                },
                LogLevelOrStdout::Stdout => "",
            };

            if !level_indicator.is_empty() {
                pb.set_message(format!(
                    "{}: {} - {}",
                    self.node_id, level_indicator, message_str
                ));
            } else {
                pb.set_message(format!("{}: {}", self.node_id, message_str));
            }
        }
    }

    async fn try_clone(&self) -> eyre::Result<Self::Clone> {
        Ok(LocalBuildLogger {
            node_id: self.node_id.clone(),
            progress_bar: self.progress_bar.clone(),
            multi: self.multi.clone(),
        })
    }
}
