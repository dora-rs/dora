use crate::{
    log::{self, NodeLogger},
    node_communication::spawn_listener_loop,
    node_inputs, CoreNodeKindExt, DoraEvent, Event, OutputId, RunningNode,
};
use aligned_vec::{AVec, ConstAlign};
use crossbeam::queue::ArrayQueue;
use dora_arrow_convert::IntoArrow;
use dora_core::{
    build::run_build_command,
    config::DataId,
    descriptor::{
        resolve_path, source_is_url, CustomNode, Descriptor, OperatorDefinition, OperatorSource,
        PythonSource, ResolvedNode, ResolvedNodeExt, DYNAMIC_SOURCE, SHELL_SOURCE,
    },
    get_python_path,
    uhlc::HLC,
};
use dora_download::download_file;
use dora_message::{
    common::{LogLevel, LogMessage},
    daemon_to_coordinator::{DataMessage, NodeExitStatus, Timestamped},
    daemon_to_node::{NodeConfig, RuntimeConfig},
    descriptor::EnvValue,
    id::NodeId,
    DataflowId,
};
use dora_node_api::{
    arrow::array::ArrayData,
    arrow_utils::{copy_array_into_sample, required_data_size},
    Metadata,
};
use eyre::{ContextCompat, WrapErr};
use git::GitFolder;
use std::{
    collections::{BTreeMap, BTreeSet},
    future::Future,
    path::{Path, PathBuf},
    process::Stdio,
    sync::Arc,
};
use tokio::{
    fs::File,
    io::{AsyncBufReadExt, AsyncWriteExt},
    sync::{mpsc, oneshot},
};
use tracing::error;

mod git;

#[derive(Clone)]
pub struct Spawner {
    pub dataflow_id: DataflowId,
    pub working_dir: PathBuf,
    pub daemon_tx: mpsc::Sender<Timestamped<Event>>,
    pub dataflow_descriptor: Descriptor,
    /// clock is required for generating timestamps when dropping messages early because queue is full
    pub clock: Arc<HLC>,
    pub uv: bool,
    pub build_only: bool,
}

impl Spawner {
    pub async fn prepare_node(
        self,
        node: ResolvedNode,
        node_stderr_most_recent: Arc<ArrayQueue<String>>,
        logger: &mut NodeLogger<'_>,
        repos_in_use: &mut BTreeMap<PathBuf, BTreeSet<DataflowId>>,
    ) -> eyre::Result<impl Future<Output = eyre::Result<PreparedNode>>> {
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
            daemon_communication,
            dataflow_descriptor: self.dataflow_descriptor.clone(),
            dynamic: node.kind.dynamic(),
        };

        let prepared_git = if let dora_core::descriptor::CoreNodeKind::Custom(CustomNode {
            source: dora_message::descriptor::NodeSource::GitBranch { repo, rev },
            ..
        }) = &node.kind
        {
            let target_dir = self.working_dir.join("build");
            let git_folder = GitFolder::choose_clone_dir(
                self.dataflow_id,
                repo.clone(),
                rev.clone(),
                &target_dir,
                repos_in_use,
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
            self.prepare_node_inner(
                node,
                &mut logger,
                dataflow_id,
                node_config,
                prepared_git,
                node_stderr_most_recent,
            )
            .await
        };
        Ok(task)
    }

    async fn prepare_node_inner(
        mut self,
        node: ResolvedNode,
        logger: &mut NodeLogger<'_>,
        dataflow_id: uuid::Uuid,
        node_config: NodeConfig,
        git_folder: Option<GitFolder>,
        node_stderr_most_recent: Arc<ArrayQueue<String>>,
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
            node_stderr_most_recent,
        })
    }

    async fn build_node(
        &mut self,
        logger: &mut NodeLogger<'_>,
        node_env: &Option<BTreeMap<String, EnvValue>>,
        working_dir: PathBuf,
        build: &String,
    ) -> Result<(), eyre::Error> {
        logger
            .log(
                LogLevel::Info,
                None,
                format!("running build command: `{build}"),
            )
            .await;
        let build = build.to_owned();
        let uv = self.uv;
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
                        Some("build command".into()),
                        line.unwrap_or_else(|err| format!("io err: {}", err.kind())),
                    )
                    .await;
            }
        });
        task.await??;
        Ok(())
    }
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
    node_stderr_most_recent: Arc<ArrayQueue<String>>,
}

impl PreparedNode {
    pub fn node_id(&self) -> &NodeId {
        &self.node.id
    }

    pub fn dynamic(&self) -> bool {
        self.node.kind.dynamic()
    }

    pub async fn spawn(mut self, logger: &mut NodeLogger<'_>) -> eyre::Result<RunningNode> {
        let mut child = match &mut self.command {
            Some(command) => command.spawn().wrap_err(self.spawn_error_msg)?,
            None => {
                return Ok(RunningNode {
                    pid: None,
                    node_config: self.node_config,
                })
            }
        };

        let pid = crate::ProcessId::new(child.id().context(
            "Could not get the pid for the just spawned node and indicate that there is an error",
        )?);
        logger
            .log(
                LogLevel::Debug,
                Some("spawner".into()),
                format!("spawned node with pid {pid:?}"),
            )
            .await;

        let dataflow_dir: PathBuf = self
            .working_dir
            .join("out")
            .join(self.dataflow_id.to_string());
        if !dataflow_dir.exists() {
            std::fs::create_dir_all(&dataflow_dir).context("could not create dataflow_dir")?;
        }
        let (tx, mut rx) = mpsc::channel(10);
        let mut file = File::create(log::log_path(
            &self.working_dir,
            &self.dataflow_id,
            &self.node.id,
        ))
        .await
        .expect("Failed to create log file");
        let mut child_stdout =
            tokio::io::BufReader::new(child.stdout.take().expect("failed to take stdout"));
        let running_node = RunningNode {
            pid: Some(pid),
            node_config: self.node_config,
        };
        let stdout_tx = tx.clone();
        let node_id = self.node.id.clone();
        // Stdout listener stream
        tokio::spawn(async move {
            let mut buffer = String::new();
            let mut finished = false;
            while !finished {
                let mut raw = Vec::new();
                finished = match child_stdout
                    .read_until(b'\n', &mut raw)
                    .await
                    .wrap_err_with(|| {
                        format!("failed to read stdout line from spawned node {node_id}")
                    }) {
                    Ok(0) => true,
                    Ok(_) => false,
                    Err(err) => {
                        tracing::warn!("{err:?}");
                        false
                    }
                };

                match String::from_utf8(raw) {
                    Ok(s) => buffer.push_str(&s),
                    Err(err) => {
                        let lossy = String::from_utf8_lossy(err.as_bytes());
                        tracing::warn!(
                            "stdout not valid UTF-8 string (node {node_id}): {}: {lossy}",
                            err.utf8_error()
                        );
                        buffer.push_str(&lossy)
                    }
                };

                if buffer.contains("TRACE")
                    || buffer.contains("INFO")
                    || buffer.contains("DEBUG")
                    || buffer.contains("WARN")
                    || buffer.contains("ERROR")
                {
                    // tracing output, potentially multi-line -> keep reading following lines
                    // until double-newline
                    if !buffer.ends_with("\n\n") && !finished {
                        continue;
                    }
                }

                // send the buffered lines
                let lines = std::mem::take(&mut buffer);
                let sent = stdout_tx.send(lines.clone()).await;
                if sent.is_err() {
                    println!("Could not log: {lines}");
                }
            }
        });

        let mut child_stderr =
            tokio::io::BufReader::new(child.stderr.take().expect("failed to take stderr"));

        // Stderr listener stream
        let stderr_tx = tx.clone();
        let node_id = self.node.id.clone();
        let uhlc = self.clock.clone();
        let daemon_tx_log = self.daemon_tx.clone();
        tokio::spawn(async move {
            let mut buffer = String::new();
            let mut finished = false;
            while !finished {
                let mut raw = Vec::new();
                finished = match child_stderr
                    .read_until(b'\n', &mut raw)
                    .await
                    .wrap_err_with(|| {
                        format!("failed to read stderr line from spawned node {node_id}")
                    }) {
                    Ok(0) => true,
                    Ok(_) => false,
                    Err(err) => {
                        tracing::warn!("{err:?}");
                        true
                    }
                };

                let new = match String::from_utf8(raw) {
                    Ok(s) => s,
                    Err(err) => {
                        let lossy = String::from_utf8_lossy(err.as_bytes());
                        tracing::warn!(
                            "stderr not valid UTF-8 string (node {node_id}): {}: {lossy}",
                            err.utf8_error()
                        );
                        lossy.into_owned()
                    }
                };

                buffer.push_str(&new);

                self.node_stderr_most_recent.force_push(new);

                // send the buffered lines
                let lines = std::mem::take(&mut buffer);
                let sent = stderr_tx.send(lines.clone()).await;
                if sent.is_err() {
                    println!("Could not log: {lines}");
                }
            }
        });

        let node_id = self.node.id.clone();
        let dynamic_node = self.node.kind.dynamic();
        let (log_finish_tx, log_finish_rx) = oneshot::channel();
        let clock = self.clock.clone();
        let daemon_tx = self.daemon_tx.clone();
        let dataflow_id = self.dataflow_id;
        tokio::spawn(async move {
            let exit_status = NodeExitStatus::from(child.wait().await);
            let _ = log_finish_rx.await;
            let event = DoraEvent::SpawnedNodeResult {
                dataflow_id,
                node_id,
                exit_status,
                dynamic_node,
            }
            .into();
            let event = Timestamped {
                inner: event,
                timestamp: clock.new_timestamp(),
            };
            let _ = daemon_tx.send(event).await;
        });

        let node_id = self.node.id.clone();
        let daemon_id = logger.inner().inner().daemon_id().clone();
        let mut cloned_logger = logger
            .inner()
            .inner()
            .inner()
            .try_clone()
            .await
            .context("failed to clone logger")?;

        let send_stdout_to = self
            .node
            .send_stdout_as()
            .context("Could not resolve `send_stdout_as` configuration")?;

        // Log to file stream.
        tokio::spawn(async move {
            while let Some(message) = rx.recv().await {
                // If log is an output, we're sending the logs to the dataflow
                if let Some(stdout_output_name) = &send_stdout_to {
                    // Convert logs to DataMessage
                    let array = message.into_arrow();

                    let array: ArrayData = array.into();
                    let total_len = required_data_size(&array);
                    let mut sample: AVec<u8, ConstAlign<128>> =
                        AVec::__from_elem(128, 0, total_len);

                    let type_info = copy_array_into_sample(&mut sample, &array);

                    let metadata = Metadata::new(uhlc.new_timestamp(), type_info);
                    let output_id = OutputId(
                        node_id.clone(),
                        DataId::from(stdout_output_name.to_string()),
                    );
                    let event = DoraEvent::Logs {
                        dataflow_id,
                        output_id,
                        metadata,
                        message: DataMessage::Vec(sample),
                    }
                    .into();
                    let event = Timestamped {
                        inner: event,
                        timestamp: uhlc.new_timestamp(),
                    };
                    let _ = daemon_tx_log.send(event).await;
                }

                let _ = file
                    .write_all(message.as_bytes())
                    .await
                    .map_err(|err| error!("Could not log {message} to file due to {err}"));
                let formatted = message.lines().fold(String::default(), |mut output, line| {
                    output.push_str(line);
                    output
                });
                if std::env::var("DORA_QUIET").is_err() {
                    cloned_logger
                        .log(LogMessage {
                            daemon_id: Some(daemon_id.clone()),
                            dataflow_id,
                            level: LogLevel::Info,
                            node_id: Some(node_id.clone()),
                            target: Some("stdout".into()),
                            message: formatted,
                            file: None,
                            line: None,
                            module_path: None,
                        })
                        .await;
                }
                // Make sure that all data has been synced to disk.
                let _ = file
                    .sync_all()
                    .await
                    .map_err(|err| error!("Could not sync logs to file due to {err}"));
            }
            let _ = log_finish_tx
                .send(())
                .map_err(|_| error!("Could not inform that log file thread finished"));
        });
        Ok(running_node)
    }
}

async fn path_spawn_command(
    working_dir: &Path,
    uv: bool,
    logger: &mut NodeLogger<'_>,
    node: &dora_core::descriptor::CustomNode,
    permit_url: bool,
) -> eyre::Result<Option<tokio::process::Command>> {
    let cmd = match node.path.as_str() {
        DYNAMIC_SOURCE => return Ok(None),
        SHELL_SOURCE => {
            if cfg!(target_os = "windows") {
                let mut cmd = tokio::process::Command::new("cmd");
                cmd.args(["/C", &node.args.clone().unwrap_or_default()]);
                cmd
            } else {
                let mut cmd = tokio::process::Command::new("sh");
                cmd.args(["-c", &node.args.clone().unwrap_or_default()]);
                cmd
            }
        }
        source => {
            let resolved_path = if source_is_url(source) {
                if !permit_url {
                    eyre::bail!("URL paths are not supported in this case");
                }
                // try to download the shared library
                let target_dir = Path::new("build");
                download_file(source, target_dir)
                    .await
                    .wrap_err("failed to download custom node")?
            } else {
                resolve_path(source, working_dir)
                    .wrap_err_with(|| format!("failed to resolve node source `{}`", source))?
            };

            // If extension is .py, use python to run the script
            let mut cmd = match resolved_path.extension().map(|ext| ext.to_str()) {
                Some(Some("py")) => {
                    let mut cmd = if uv {
                        let mut cmd = tokio::process::Command::new("uv");
                        cmd.arg("run");
                        cmd.arg("python");
                        logger
                            .log(
                                LogLevel::Info,
                                Some("spawner".into()),
                                format!("spawning: uv run python -u {}", resolved_path.display()),
                            )
                            .await;
                        cmd
                    } else {
                        let python = get_python_path()
                            .wrap_err("Could not find python path when spawning custom node")?;
                        logger
                            .log(
                                LogLevel::Info,
                                Some("spawner".into()),
                                format!("spawning: {:?} -u {}", &python, resolved_path.display()),
                            )
                            .await;

                        tokio::process::Command::new(python)
                    };
                    // Force python to always flush stdout/stderr buffer
                    cmd.arg("-u");
                    cmd.arg(&resolved_path);
                    cmd
                }
                _ => {
                    logger
                        .log(
                            LogLevel::Info,
                            Some("spawner".into()),
                            format!("spawning: {}", resolved_path.display()),
                        )
                        .await;
                    if uv {
                        let mut cmd = tokio::process::Command::new("uv");
                        cmd.arg("run");
                        cmd.arg(&resolved_path);
                        cmd
                    } else {
                        tokio::process::Command::new(&resolved_path)
                    }
                }
            };

            if let Some(args) = &node.args {
                cmd.args(args.split_ascii_whitespace());
            }
            cmd
        }
    };

    Ok(Some(cmd))
}
