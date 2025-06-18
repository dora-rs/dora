pub use git::GitManager;
pub use logger::{BuildLogger, LogLevelOrStdout};

use url::Url;

use std::{collections::BTreeMap, future::Future, path::PathBuf};

use crate::descriptor::ResolvedNode;
use dora_message::{
    common::{GitSource, LogLevel},
    descriptor::{CoreNodeKind, EnvValue},
    id::NodeId,
    SessionId,
};
use eyre::Context;

use build_command::run_build_command;
use git::GitFolder;

mod build_command;
mod git;
mod logger;

#[derive(Clone)]
pub struct Builder {
    pub session_id: SessionId,
    pub base_working_dir: PathBuf,
    pub uv: bool,
}

impl Builder {
    pub async fn build_node(
        self,
        node: ResolvedNode,
        git: Option<GitSource>,
        prev_git: Option<GitSource>,
        mut logger: impl BuildLogger,
        git_manager: &mut GitManager,
    ) -> eyre::Result<impl Future<Output = eyre::Result<BuiltNode>>> {
        let prepared_git = if let Some(GitSource { repo, commit_hash }) = git {
            let repo_url = Url::parse(&repo).context("failed to parse git repository URL")?;
            let target_dir = self.base_working_dir.join("git");
            let prev_hash = prev_git.filter(|p| p.repo == repo).map(|p| p.commit_hash);
            let git_folder = git_manager.choose_clone_dir(
                self.session_id,
                repo_url,
                commit_hash,
                prev_hash,
                &target_dir,
            )?;
            Some(git_folder)
        } else {
            None
        };

        let task = async move { self.build_node_inner(node, &mut logger, prepared_git).await };
        Ok(task)
    }

    async fn build_node_inner(
        self,
        node: ResolvedNode,
        logger: &mut impl BuildLogger,
        git_folder: Option<GitFolder>,
    ) -> eyre::Result<BuiltNode> {
        logger.log_message(LogLevel::Debug, "building node").await;
        let node_working_dir = match &node.kind {
            CoreNodeKind::Custom(n) => {
                let node_working_dir = match git_folder {
                    Some(git_folder) => {
                        let clone_dir = git_folder.prepare(logger).await?;
                        tracing::warn!(
                            "using git clone directory as working dir: \
                            this behavior is unstable and might change \
                            (see https://github.com/dora-rs/dora/pull/901)"
                        );
                        clone_dir
                    }
                    None => self.base_working_dir,
                };

                if let Some(build) = &n.build {
                    build_node(logger, &node.env, node_working_dir.clone(), build, self.uv).await?;
                }
                node_working_dir
            }
            CoreNodeKind::Runtime(n) => {
                // run build commands
                for operator in &n.operators {
                    if let Some(build) = &operator.config.build {
                        build_node(
                            logger,
                            &node.env,
                            self.base_working_dir.clone(),
                            build,
                            self.uv,
                        )
                        .await?;
                    }
                }
                self.base_working_dir.clone()
            }
        };
        Ok(BuiltNode { node_working_dir })
    }
}

async fn build_node(
    logger: &mut impl BuildLogger,
    node_env: &Option<BTreeMap<String, EnvValue>>,
    working_dir: PathBuf,
    build: &String,
    uv: bool,
) -> eyre::Result<()> {
    logger
        .log_message(LogLevel::Info, format!("running build command: `{build}"))
        .await;
    let build = build.to_owned();
    let node_env = node_env.clone();
    let mut logger = logger.try_clone().await.context("failed to clone logger")?;
    let (stdout_tx, mut stdout) = tokio::sync::mpsc::channel(10);
    let task = tokio::task::spawn_blocking(move || {
        run_build_command(&build, &working_dir, uv, &node_env, stdout_tx)
            .context("build command failed")
    });
    tokio::spawn(async move {
        while let Some(line) = stdout.recv().await {
            logger
                .log_stdout(line.unwrap_or_else(|err| format!("io err: {}", err.kind())))
                .await;
        }
    });
    task.await??;
    Ok(())
}

pub struct BuiltNode {
    pub node_working_dir: PathBuf,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BuildInfo {
    pub node_working_dirs: BTreeMap<NodeId, PathBuf>,
}
