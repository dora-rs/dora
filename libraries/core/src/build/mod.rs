pub use git::GitManager;
pub use logger::{BuildLogger, LogLevelOrStdout};

use std::{
    collections::BTreeMap,
    future::Future,
    path::{Path, PathBuf},
};

use crate::descriptor::ResolvedNode;
use dora_message::{
    SessionId,
    common::{GitSource, LogLevel},
    descriptor::{CoreNodeKind, CustomNode, EnvValue, OperatorSource},
    id::NodeId,
};
use eyre::Context;

use build_command::{prepare_managed_python_env, run_build_command};
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
    pub async fn build_node<L>(
        self,
        node: ResolvedNode,
        git: Option<GitSource>,
        prev_git: Option<PrevGitSource>,
        mut logger: L,
        git_manager: &mut GitManager,
    ) -> eyre::Result<impl Future<Output = eyre::Result<BuiltNode>> + use<L>>
    where
        L: BuildLogger,
    {
        let prepared_git = if let Some(GitSource { repo, commit_hash }) = git {
            let target_dir = self.base_working_dir.join("git");
            let git_folder = git_manager.choose_clone_dir(
                self.session_id,
                repo,
                commit_hash,
                prev_git,
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
        let python_env_dir;
        let node_working_dir = match &node.kind {
            CoreNodeKind::Custom(n) => {
                let node_working_dir = match git_folder {
                    Some(git_folder) => {
                        let clone_dir = git_folder.prepare(logger).await?;
                        tracing::warn!(
                            "using git clone directory as working dir: \
                            this behavior is unstable and might change"
                        );
                        clone_dir
                    }
                    None => self.base_working_dir,
                };

                // Only build-backed custom Python nodes get Dora's managed env today;
                // script-only nodes still run in the caller's ambient uv environment.
                python_env_dir =
                    managed_python_env_dir(&node, &node_working_dir).filter(|_| n.build.is_some());
                if self.uv
                    && let Some(python_env_dir) = &python_env_dir
                {
                    prepare_python_env(
                        logger,
                        &node.env,
                        node_working_dir.clone(),
                        python_env_dir.clone(),
                    )
                    .await?;
                }

                if let Some(build) = &n.build {
                    build_node(
                        logger,
                        &node.env,
                        node_working_dir.clone(),
                        build,
                        self.uv,
                        python_env_dir.clone(),
                    )
                    .await?;
                }
                node_working_dir
            }
            CoreNodeKind::Runtime(n) => {
                // Runtime nodes use one managed Python env per node; prepare it once before
                // running any Python operator build commands.
                python_env_dir = managed_python_env_dir(&node, &self.base_working_dir);
                if self.uv
                    && let Some(python_env_dir) = &python_env_dir
                {
                    prepare_python_env(
                        logger,
                        &node.env,
                        self.base_working_dir.clone(),
                        python_env_dir.clone(),
                    )
                    .await?;
                }
                // run build commands
                for operator in &n.operators {
                    if let Some(build) = &operator.config.build {
                        build_node(
                            logger,
                            &node.env,
                            self.base_working_dir.clone(),
                            build,
                            self.uv,
                            python_env_dir.clone(),
                        )
                        .await?;
                    }
                }
                self.base_working_dir.clone()
            }
        };
        Ok(BuiltNode {
            node_working_dir,
            python_env_dir,
        })
    }
}

async fn build_node(
    logger: &mut impl BuildLogger,
    node_env: &Option<BTreeMap<String, EnvValue>>,
    working_dir: PathBuf,
    build: &String,
    uv: bool,
    python_env_dir: Option<PathBuf>,
) -> eyre::Result<()> {
    logger
        .log_message(
            LogLevel::Info,
            format!(
                "running build command: `{build}` in {}",
                working_dir.display()
            ),
        )
        .await;
    let build = build.to_owned();
    let node_env = node_env.clone();
    let mut logger = logger.try_clone().await.context("failed to clone logger")?;
    let (stdout_tx, mut stdout) = tokio::sync::mpsc::channel(10);
    let task = tokio::spawn(async move {
        run_build_command(
            &build,
            &working_dir,
            uv,
            python_env_dir,
            &node_env,
            stdout_tx,
        )
        .await
        .context("build command failed")
    });
    let stdout_task = tokio::spawn(async move {
        while let Some(line) = stdout.recv().await {
            logger
                .log_stdout(line.unwrap_or_else(|err| format!("io err: {}", err.kind())))
                .await;
        }
    });
    stdout_task.await?;
    task.await??;

    Ok(())
}

async fn prepare_python_env(
    logger: &mut impl BuildLogger,
    node_env: &Option<BTreeMap<String, EnvValue>>,
    working_dir: PathBuf,
    python_env_dir: PathBuf,
) -> eyre::Result<()> {
    logger
        .log_message(
            LogLevel::Info,
            format!(
                "preparing managed Python env in {}",
                python_env_dir.display()
            ),
        )
        .await;
    let node_env = node_env.clone();
    let mut logger = logger.try_clone().await.context("failed to clone logger")?;
    let (stdout_tx, mut stdout) = tokio::sync::mpsc::channel(10);
    let task = tokio::spawn(async move {
        prepare_managed_python_env(&working_dir, &python_env_dir, &node_env, stdout_tx)
            .await
            .context("managed Python env preparation failed")
    });
    let stdout_task = tokio::spawn(async move {
        while let Some(line) = stdout.recv().await {
            logger
                .log_stdout(line.unwrap_or_else(|err| format!("io err: {}", err.kind())))
                .await;
        }
    });
    stdout_task.await?;
    task.await??;

    Ok(())
}

pub struct BuiltNode {
    pub node_working_dir: PathBuf,
    pub python_env_dir: Option<PathBuf>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BuildInfo {
    pub node_working_dirs: BTreeMap<NodeId, PathBuf>,
    #[serde(default)]
    pub python_env_dirs: BTreeMap<NodeId, PathBuf>,
}

/// Computes the managed Python env directory for `node`, if it needs one.
///
/// Returns `Some(working_dir/.dora/python-env)` when the node is a Python custom
/// node or a runtime node with at least one Python operator; `None` otherwise.
pub fn managed_python_env_dir(node: &ResolvedNode, node_working_dir: &Path) -> Option<PathBuf> {
    node_requires_managed_python_env(node)
        .then(|| node_working_dir.join(".dora").join("python-env"))
}

/// Returns the bin directory inside an env dir (`bin/` on Unix, `Scripts/` on Windows).
pub fn managed_python_bin_dir(python_env_dir: &Path) -> PathBuf {
    if cfg!(windows) {
        python_env_dir.join("Scripts")
    } else {
        python_env_dir.join("bin")
    }
}

/// Returns the interpreter path inside an env dir (`bin/python` or `Scripts/python.exe`).
pub fn managed_python_interpreter(python_env_dir: &Path) -> PathBuf {
    let executable = if cfg!(windows) {
        "python.exe"
    } else {
        "python"
    };
    managed_python_bin_dir(python_env_dir).join(executable)
}

fn node_requires_managed_python_env(node: &ResolvedNode) -> bool {
    match &node.kind {
        CoreNodeKind::Custom(custom) => is_python_custom_node(custom),
        CoreNodeKind::Runtime(runtime) => runtime
            .operators
            .iter()
            .any(|operator| matches!(operator.config.source, OperatorSource::Python(_))),
    }
}

fn is_python_custom_node(custom: &CustomNode) -> bool {
    Path::new(&custom.path)
        .extension()
        .and_then(|ext| ext.to_str())
        .is_some_and(|ext| ext.eq_ignore_ascii_case("py"))
}

pub struct PrevGitSource {
    pub git_source: GitSource,
    /// `True` if any nodes of this dataflow still require the source for building.
    pub still_needed_for_this_build: bool,
}

#[cfg(test)]
mod tests {
    use super::{managed_python_bin_dir, managed_python_env_dir, managed_python_interpreter};
    use crate::descriptor::ResolvedNode;
    use std::path::PathBuf;

    /// Deserialize from YAML to avoid coupling tests to ResolvedNode's exact
    /// field list — `#[serde(default)]` on the optional fields handles drift.
    fn resolved_node_from_yaml(yaml: &str) -> ResolvedNode {
        serde_yaml::from_str(yaml).expect("test node yaml should parse")
    }

    #[test]
    fn assigns_managed_env_dir_to_python_custom_nodes() {
        let node = resolved_node_from_yaml(
            "id: python-custom\n\
             custom:\n  \
             path: node.py\n  \
             source: Local\n",
        );
        let env_dir = managed_python_env_dir(&node, &PathBuf::from("workdir"))
            .expect("python custom node should get managed env dir");
        assert_eq!(env_dir, PathBuf::from("workdir/.dora/python-env"));
    }

    #[test]
    fn skips_managed_env_dir_for_non_python_custom_nodes() {
        let node = resolved_node_from_yaml(
            "id: rust-custom\n\
             custom:\n  \
             path: target/release/my-node\n  \
             source: Local\n",
        );
        assert!(managed_python_env_dir(&node, &PathBuf::from("workdir")).is_none());
    }

    #[test]
    fn build_info_deserializes_without_python_env_dirs() {
        let build_info: super::BuildInfo =
            serde_yaml::from_str("node_working_dirs: {}\n").expect("build info should parse");
        assert!(build_info.python_env_dirs.is_empty());
    }

    #[test]
    fn managed_python_helpers_use_platform_layout() {
        let env_dir = PathBuf::from("env-dir");
        let expected_bin_dir = if cfg!(windows) {
            PathBuf::from("env-dir/Scripts")
        } else {
            PathBuf::from("env-dir/bin")
        };
        let expected_interpreter = if cfg!(windows) {
            PathBuf::from("env-dir/Scripts/python.exe")
        } else {
            PathBuf::from("env-dir/bin/python")
        };

        assert_eq!(managed_python_bin_dir(&env_dir), expected_bin_dir);
        assert_eq!(managed_python_interpreter(&env_dir), expected_interpreter);
    }
}
