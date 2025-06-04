pub use git::GitManager;
use url::Url;

use std::{
    collections::{BTreeMap, BTreeSet},
    future::Future,
    path::PathBuf,
    sync::Arc,
};

use dora_core::{
    build::run_build_command,
    descriptor::{CustomNode, Descriptor, ResolvedNode},
    uhlc::HLC,
};
use dora_message::{
    common::{LogLevel, Timestamped},
    coordinator_to_daemon::GitSource,
    daemon_to_node::NodeConfig,
    descriptor::{EnvValue, GitRepoRev, ResolvedNodeSource},
    id::NodeId,
    BuildId, DataflowId,
};
use eyre::Context;
use tokio::sync::mpsc;

use crate::{build::git::GitFolder, log::DaemonLogger, Event};

mod git;

#[derive(Clone)]
pub struct Builder {
    pub build_id: BuildId,
    pub prev_build_id: Option<BuildId>,
    pub working_dir: PathBuf,
    pub daemon_tx: mpsc::Sender<Timestamped<Event>>,
    pub dataflow_descriptor: Descriptor,
    /// clock is required for generating timestamps when dropping messages early because queue is full
    pub clock: Arc<HLC>,
    pub uv: bool,
}

impl Builder {
    pub async fn prepare_node(
        self,
        node: ResolvedNode,
        git: Option<GitSource>,
        logger: &mut DaemonLogger,
        git_manager: &mut GitManager,
    ) -> eyre::Result<impl Future<Output = eyre::Result<PreparedNode>>> {
        let build_id = self.build_id;
        logger
            .log_build(
                build_id,
                LogLevel::Debug,
                Some(node.id.clone()),
                "building node",
            )
            .await;

        let prepared_git = if let Some(GitSource { repo, commit_hash }) = git {
            let repo_url = Url::parse(&repo).context("failed to parse git repository URL")?;
            let target_dir = self.working_dir.join("build");
            let git_folder = git_manager.choose_clone_dir(
                self.build_id,
                self.prev_build_id,
                repo_url,
                commit_hash.clone(),
                &target_dir,
            )?;
            Some(git_folder)
        } else {
            None
        };

        let mut logger = logger
            .try_clone()
            .await
            .wrap_err("failed to clone logger")?;
        let task = async move {
            self.prepare_node_inner(node, &mut logger, build_id, prepared_git)
                .await
        };
        Ok(task)
    }

    async fn prepare_node_inner(
        mut self,
        node: ResolvedNode,
        logger: &mut DaemonLogger,
        build_id: uuid::Uuid,
        git_folder: Option<GitFolder>,
    ) -> eyre::Result<PreparedNode> {
        let (command, error_msg) = match &node.kind {
            dora_core::descriptor::CoreNodeKind::Custom(n) => {
                let build_dir = match git_folder {
                    Some(git_folder) => git_folder.prepare(logger).await?,
                    None => self.working_dir.clone(),
                };

                if let Some(build) = &n.build {
                    self.build_node(logger, &node.env, build_dir.clone(), build)
                        .await?;
                }
                let mut command = if self.build_only {
                    None
                } else {
                    path_spawn_command(&build_dir, self.uv, logger, n, true).await?
                };

                if let Some(command) = &mut command {
                    command.current_dir(&self.working_dir);
                    command.stdin(Stdio::null());

                    command.env(
                        "DORA_NODE_CONFIG",
                        serde_yaml::to_string(&node_config.clone())
                            .wrap_err("failed to serialize node config")?,
                    );
                    // Injecting the env variable defined in the `yaml` into
                    // the node runtime.
                    if let Some(envs) = &node.env {
                        for (key, value) in envs {
                            command.env(key, value.to_string());
                        }
                    }
                    if let Some(envs) = &n.envs {
                        // node has some inner env variables -> add them too
                        for (key, value) in envs {
                            command.env(key, value.to_string());
                        }
                    }

                    // Set the process group to 0 to ensure that the spawned process does not exit immediately on CTRL-C
                    #[cfg(unix)]
                    command.process_group(0);

                    command.env("PYTHONUNBUFFERED", "1");
                    command
                        .stdin(Stdio::null())
                        .stdout(Stdio::piped())
                        .stderr(Stdio::piped());
                };

                let error_msg = format!(
                    "failed to run `{}` with args `{}`",
                    n.path,
                    n.args.as_deref().unwrap_or_default(),
                );
                (command, error_msg)
            }
            dora_core::descriptor::CoreNodeKind::Runtime(n) => {
                // run build commands
                for operator in &n.operators {
                    if let Some(build) = &operator.config.build {
                        self.build_node(logger, &node.env, self.working_dir.clone(), build)
                            .await?;
                    }
                }

                let python_operators: Vec<&OperatorDefinition> = n
                    .operators
                    .iter()
                    .filter(|x| matches!(x.config.source, OperatorSource::Python { .. }))
                    .collect();

                let other_operators = n
                    .operators
                    .iter()
                    .any(|x| !matches!(x.config.source, OperatorSource::Python { .. }));

                let mut command = if self.build_only {
                    None
                } else if !python_operators.is_empty() && !other_operators {
                    // Use python to spawn runtime if there is a python operator

                    // TODO: Handle multi-operator runtime once sub-interpreter is supported
                    if python_operators.len() > 2 {
                        eyre::bail!(
                            "Runtime currently only support one Python Operator.
                     This is because pyo4 sub-interpreter is not yet available.
                     See: https://github.com/PyO4/pyo3/issues/576"
                        );
                    }

                    let python_operator = python_operators
                        .first()
                        .context("Runtime had no operators definition.")?;

                    if let OperatorSource::Python(PythonSource {
                        source: _,
                        conda_env: Some(conda_env),
                    }) = &python_operator.config.source
                    {
                        let conda = which::which("conda").context(
                        "failed to find `conda`, yet a `conda_env` was defined. Make sure that `conda` is available.",
                    )?;
                        let mut command = tokio::process::Command::new(conda);
                        command.args([
                            "run",
                            "-n",
                            conda_env,
                            "python",
                            "-c",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(command)
                    } else {
                        let mut cmd = if self.uv {
                            let mut cmd = tokio::process::Command::new("uv");
                            cmd.arg("run");
                            cmd.arg("python");
                            tracing::info!(
                            "spawning: uv run python -uc import dora; dora.start_runtime() # {}",
                            node.id
                        );
                            cmd
                        } else {
                            let python = get_python_path()
                                .wrap_err("Could not find python path when spawning custom node")?;
                            tracing::info!(
                                "spawning: python -uc import dora; dora.start_runtime() # {}",
                                node.id
                            );

                            tokio::process::Command::new(python)
                        };
                        // Force python to always flush stdout/stderr buffer
                        cmd.args([
                            "-c",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(cmd)
                    }
                } else if python_operators.is_empty() && other_operators {
                    let mut cmd = tokio::process::Command::new(
                        std::env::current_exe()
                            .wrap_err("failed to get current executable path")?,
                    );
                    cmd.arg("runtime");
                    Some(cmd)
                } else {
                    eyre::bail!("Runtime can not mix Python Operator with other type of operator.");
                };

                let runtime_config = RuntimeConfig {
                    node: node_config.clone(),
                    operators: n.operators.clone(),
                };

                if let Some(command) = &mut command {
                    command.current_dir(&self.working_dir);

                    command.env(
                        "DORA_RUNTIME_CONFIG",
                        serde_yaml::to_string(&runtime_config)
                            .wrap_err("failed to serialize runtime config")?,
                    );
                    // Injecting the env variable defined in the `yaml` into
                    // the node runtime.
                    if let Some(envs) = &node.env {
                        for (key, value) in envs {
                            command.env(key, value.to_string());
                        }
                    }
                    // Set the process group to 0 to ensure that the spawned process does not exit immediately on CTRL-C
                    #[cfg(unix)]
                    command.process_group(0);

                    command
                        .stdin(Stdio::null())
                        .stdout(Stdio::piped())
                        .stderr(Stdio::piped());
                };
                let error_msg = format!(
                    "failed to run runtime {}/{}",
                    runtime_config.node.dataflow_id, runtime_config.node.node_id
                );
                (command, error_msg)
            }
        };
        Ok(PreparedNode {
            command,
            spawn_error_msg: error_msg,
            working_dir: self.working_dir,
            dataflow_id,
            node,
            node_config,
            clock: self.clock,
            daemon_tx: self.daemon_tx,
        })
    }
}

pub async fn build_node(
    node_id: NodeId,
    logger: &mut DaemonLogger<'_>,
    node_env: &Option<BTreeMap<String, EnvValue>>,
    working_dir: PathBuf,
    build: &String,
    uv: bool,
) -> eyre::Result<()> {
    logger
        .log(
            LogLevel::Info,
            None,
            Some(node_id),
            Some("build".to_owned()),
            format!("running build command: `{build}"),
        )
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
                .log(
                    LogLevel::Info,
                    None,
                    Some(node_id),
                    Some("build command".into()),
                    line.unwrap_or_else(|err| format!("io err: {}", err.kind())),
                )
                .await;
        }
    });
    task.await??;
    Ok(())
}

pub struct PreparedNode {
    command: Option<tokio::process::Command>,
    spawn_error_msg: String,
    working_dir: PathBuf,
    dataflow_id: DataflowId,
    node: ResolvedNode,
    node_config: NodeConfig,
    clock: Arc<HLC>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
}
