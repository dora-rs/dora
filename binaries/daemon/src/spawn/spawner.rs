use crate::{
    CoreNodeKindExt, Event,
    log::NodeLogger,
    node_communication::spawn_listener_loop,
    node_inputs,
    spawn::{command::path_spawn_command, prepared::PreparedNode},
};
use clonable_command::{Command, Stdio};
use crossbeam::queue::ArrayQueue;
use dora_core::{
    descriptor::{Descriptor, OperatorDefinition, OperatorSource, PythonSource, ResolvedNode},
    get_python_path,
    uhlc::HLC,
};
use dora_message::{
    DataflowId,
    common::LogLevel,
    daemon_to_coordinator::Timestamped,
    daemon_to_node::{NodeConfig, RuntimeConfig},
};
use eyre::{ContextCompat, WrapErr, bail};
use std::{future::Future, path::PathBuf, sync::Arc};
use tokio::sync::mpsc;

#[derive(Clone)]
pub struct Spawner {
    pub dataflow_id: DataflowId,
    pub daemon_tx: mpsc::Sender<Timestamped<Event>>,
    pub dataflow_descriptor: Descriptor,
    /// clock is required for generating timestamps when dropping messages early because queue is full
    pub clock: Arc<HLC>,
    pub uv: bool,
}

impl Spawner {
    pub async fn spawn_node(
        self,
        node: ResolvedNode,
        node_working_dir: PathBuf,
        node_stderr_most_recent: Arc<ArrayQueue<String>>,
        write_events_to: Option<PathBuf>,
        logger: &mut NodeLogger<'_>,
    ) -> eyre::Result<impl Future<Output = eyre::Result<PreparedNode>> + use<>> {
        let dataflow_id = self.dataflow_id;
        let node_id = node.id.clone();
        logger
            .log(
                LogLevel::Debug,
                Some("daemon::spawner".into()),
                "spawning node",
            )
            .await;

        let queue_sizes = node_inputs(&node)
            .into_iter()
            .map(|(k, v)| (k, v.queue_size.unwrap_or(10)))
            .collect();
        let daemon_communication = spawn_listener_loop(
            &dataflow_id,
            &node_id,
            &self.daemon_tx,
            self.dataflow_descriptor.communication.local,
            queue_sizes,
            self.clock.clone(),
        )
        .await?;

        let node_config = NodeConfig {
            dataflow_id,
            node_id: node_id.clone(),
            run_config: node.kind.run_config(),
            daemon_communication: Some(daemon_communication),
            dataflow_descriptor: serde_yaml::to_value(&self.dataflow_descriptor)
                .context("failed to serialize dataflow descriptor to YAML")?,
            dynamic: node.kind.dynamic(),
            write_events_to,
        };

        let mut logger = logger
            .try_clone()
            .await
            .wrap_err("failed to clone logger")?;
        let task = async move {
            self.prepare_node_inner(
                node,
                node_working_dir,
                &mut logger,
                dataflow_id,
                node_config,
                node_stderr_most_recent,
            )
            .await
        };
        Ok(task)
    }

    async fn prepare_node_inner(
        self,
        node: ResolvedNode,
        node_working_dir: PathBuf,
        logger: &mut NodeLogger<'_>,
        dataflow_id: uuid::Uuid,
        node_config: NodeConfig,
        node_stderr_most_recent: Arc<ArrayQueue<String>>,
    ) -> eyre::Result<PreparedNode> {
        std::fs::create_dir_all(&node_working_dir)
            .context("failed to create node working directory")?;
        let (command, error_msg) = match &node.kind {
            dora_core::descriptor::CoreNodeKind::Custom(n) => {
                let command =
                    path_spawn_command(&node_working_dir, self.uv, logger, n, true).await?;

                let command = if let Some(mut command) = command {
                    command = command.current_dir(&node_working_dir);
                    command = command.stdin(Stdio::Null);

                    command = command.env(
                        "DORA_NODE_CONFIG",
                        serde_yaml::to_string(&node_config.clone())
                            .wrap_err("failed to serialize node config")?,
                    );
                    // Injecting the env variable defined in the `yaml` into
                    // the node runtime.
                    if let Some(envs) = &node.env {
                        for (key, value) in envs {
                            command = command.env(key, value.to_string());
                        }
                    }
                    if let Some(envs) = &n.envs {
                        // node has some inner env variables -> add them too
                        for (key, value) in envs {
                            command = command.env(key, value.to_string());
                        }
                    }

                    command = command.env("PYTHONUNBUFFERED", "1");
                    command = command
                        .stdin(Stdio::Null)
                        .stdout(Stdio::Piped)
                        .stderr(Stdio::Piped);
                    Some(command)
                } else {
                    command
                };

                let error_msg = format!(
                    "failed to run `{}` with args `{}`",
                    n.path,
                    n.args.as_deref().unwrap_or_default(),
                );
                (command, error_msg)
            }
            dora_core::descriptor::CoreNodeKind::Runtime(n) => {
                let python_operators: Vec<&OperatorDefinition> = n
                    .operators
                    .iter()
                    .filter(|x| matches!(x.config.source, OperatorSource::Python { .. }))
                    .collect();

                let other_operators = n
                    .operators
                    .iter()
                    .any(|x| !matches!(x.config.source, OperatorSource::Python { .. }));

                let command = if !python_operators.is_empty() && !other_operators {
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
                        let mut command = Command::new(conda);
                        command = command.args([
                            "run",
                            "-n",
                            conda_env,
                            "python",
                            "-uc",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(command)
                    } else {
                        let mut cmd = if self.uv {
                            let mut cmd = Command::new("uv");
                            cmd = cmd.arg("run");
                            cmd = cmd.arg("python");
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

                            Command::new(python)
                        };
                        // Force python to always flush stdout/stderr buffer
                        cmd = cmd.args([
                            "-uc",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(cmd)
                    }
                } else if python_operators.is_empty() && other_operators {
                    let current_exe = std::env::current_exe()
                        .wrap_err("failed to get current executable path")?;
                    let mut file_name = current_exe.clone();
                    file_name.set_extension("");
                    let file_name = file_name
                        .file_name()
                        .and_then(|s| s.to_str())
                        .context("failed to get file name from current executable")?;

                    // Check if the current executable is a python binary meaning that dora is installed within the python environment
                    if file_name.ends_with("python") || file_name.ends_with("python3") {
                        // Use the current executable to spawn runtime
                        let python = get_python_path()
                            .wrap_err("Could not find python path when spawning custom node")?;
                        let mut cmd = Command::new(python);

                        tracing::info!(
                            "spawning: python -uc import dora; dora.start_runtime() # {}",
                            node.id
                        );

                        cmd = cmd.args([
                            "-uc",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(cmd)
                    } else {
                        let mut cmd =
                            Command::new(which::which("dora").wrap_err("failed to get dora path")?);
                        cmd = cmd.arg("runtime");
                        Some(cmd)
                    }
                } else {
                    bail!(
                        "Cannot spawn runtime with both Python and non-Python operators. \
                        Please use a single operator or ensure that all operators are Python-based."
                    );
                };

                let runtime_config = RuntimeConfig {
                    node: node_config.clone(),
                    operators: n.operators.clone(),
                };

                let command = if let Some(mut command) = command {
                    command = command.current_dir(&node_working_dir);

                    command = command.env(
                        "DORA_RUNTIME_CONFIG",
                        serde_yaml::to_string(&runtime_config)
                            .wrap_err("failed to serialize runtime config")?,
                    );
                    // Injecting the env variable defined in the `yaml` into
                    // the node runtime.
                    if let Some(envs) = &node.env {
                        for (key, value) in envs {
                            command = command.env(key, value.to_string());
                        }
                    }

                    command = command
                        .stdin(Stdio::Null)
                        .stdout(Stdio::Piped)
                        .stderr(Stdio::Piped);
                    Some(command)
                } else {
                    command
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
            node_working_dir,
            dataflow_id,
            node,
            node_config,
            clock: self.clock,
            daemon_tx: self.daemon_tx,
            node_stderr_most_recent,
        })
    }
}
