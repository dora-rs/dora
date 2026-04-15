//! The `dora run` command runs a dataflow locally with an embedded
//! coordinator and daemon so that CLI monitoring commands (`dora list`,
//! `dora stop`, `dora logs`, etc.) work during execution.

use super::{Executable, system::status::daemon_running};
use crate::{
    LOCALHOST, build as build_dataflow,
    common::{connect_with_retry, handle_dataflow_result, resolve_dataflow, write_events_to},
    output::{
        LogFormat, LogOutputConfig, parse_log_filter, parse_log_level_str, print_log_message,
    },
    session::DataflowSession,
};
use dora_coordinator::{CoordinatorStore, InMemoryStore, SpanStore};
use dora_core::{
    build::LogLevelOrStdout,
    descriptor::{Descriptor, DescriptorExt},
    topics::DORA_COORDINATOR_PORT_WS_DEFAULT,
};
use dora_message::{
    cli_to_coordinator::ControlRequest, common::LogMessage, coordinator_to_cli::ControlRequestReply,
};
use duration_str::parse as parse_duration_str;
use eyre::{Context, ContextCompat, bail};
use std::{collections::BTreeMap, net::SocketAddr, path::PathBuf, sync::Arc, time::Duration};
use tokio::runtime::Builder;

#[derive(Debug, clap::Args)]
/// Run a dataflow locally.
///
/// Runs the given dataflow with an embedded coordinator and daemon so that
/// CLI commands like `dora list`, `dora stop`, and `dora logs` work.
pub struct Run {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    pub dataflow: String,
    // Use UV to run nodes.
    #[clap(long, action)]
    pub uv: bool,
    /// Automatically stop the dataflow after the given duration
    ///
    /// The command will send a stop message after the specified time has elapsed,
    /// similar to pressing Ctrl-C. This gracefully stops all nodes in the dataflow.
    ///
    /// Examples:
    ///   --stop-after 30      # 30 seconds
    ///   --stop-after 30s     # 30 seconds
    ///   --stop-after 5m      # 5 minutes
    #[clap(long, value_name = "DURATION", verbatim_doc_comment)]
    #[arg(value_parser = parse_duration_str)]
    pub stop_after: Option<Duration>,
    /// Minimum log level to display
    ///
    /// Levels: error, warn, info, debug, trace, stdout (default).
    /// "stdout" shows everything including raw stdout from nodes.
    #[clap(long, default_value = "stdout", env = "DORA_LOG_LEVEL")]
    #[arg(value_parser = parse_log_level_str)]
    pub log_level: LogLevelOrStdout,
    /// Output format for log messages
    #[clap(long, default_value = "pretty", env = "DORA_LOG_FORMAT")]
    pub log_format: LogFormat,
    /// Per-node log level filter
    ///
    /// Format: "node1=level,node2=level". Overrides --log-level for matched nodes.
    ///
    /// Examples:
    ///   --log-filter "sensor=debug,processor=warn"
    #[clap(
        long,
        value_name = "FILTER",
        env = "DORA_LOG_FILTER",
        verbatim_doc_comment
    )]
    pub log_filter: Option<String>,
    /// Allow shell nodes to execute arbitrary commands.
    ///
    /// Shell nodes are disabled by default for security reasons. This flag
    /// sets the DORA_ALLOW_SHELL_NODES environment variable.
    #[clap(long)]
    pub allow_shell_nodes: bool,
    /// Enable debug mode (publishes all messages to Zenoh for topic echo/hz/info)
    #[clap(long, action)]
    pub debug: bool,
    /// Use pinned git source commits from a lockfile during pre-run build.
    #[clap(long, action, conflicts_with = "write_lockfile")]
    pub locked: bool,
    /// Write resolved git source commits to a lockfile during pre-run build.
    #[clap(long, action)]
    pub write_lockfile: bool,
    /// Path to build lockfile (defaults to `<dataflow-stem>.dora-lock.yaml`).
    #[clap(long, value_name = "PATH")]
    pub lockfile: Option<PathBuf>,
}

impl Run {
    pub fn new(dataflow: String) -> Self {
        Self {
            dataflow,
            uv: false,
            stop_after: None,
            log_level: LogLevelOrStdout::Stdout,
            log_format: LogFormat::Pretty,
            log_filter: None,
            allow_shell_nodes: false,
            debug: false,
            locked: false,
            write_lockfile: false,
            lockfile: None,
        }
    }
}

pub fn run(dataflow: String, uv: bool) -> eyre::Result<()> {
    let mut run = Run::new(dataflow);
    run.uv = uv;
    run.execute()
}

/// Events delivered to the main wait loop.
enum RunEvent {
    CtrlC,
    StopAfterElapsed,
}

impl Executable for Run {
    fn execute(self) -> eyre::Result<()> {
        if self.allow_shell_nodes {
            // SAFETY: Called before spawning any threads (tokio runtime not yet built),
            // so there are no concurrent reads of environment variables.
            unsafe { std::env::set_var("DORA_ALLOW_SHELL_NODES", "true") };
        }

        // Register ctrlc handler FIRST (before tokio runtime) so the daemon's
        // handler gracefully skips (it already handles the "already registered" case).
        let (event_tx, event_rx) = std::sync::mpsc::channel::<RunEvent>();
        let ctrlc_tx = event_tx.clone();
        ctrlc::set_handler(move || {
            let _ = ctrlc_tx.send(RunEvent::CtrlC);
        })
        .context("failed to set ctrl-c handler")?;

        let rt = Builder::new_multi_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;

        #[cfg(feature = "tracing")]
        let _guard = {
            let _enter = rt.enter();
            let env_log = std::env::var("RUST_LOG").unwrap_or("info".to_string());
            dora_tracing::init_tracing_subscriber(
                "dora-run",
                Some(&env_log),
                None,
                tracing::metadata::LevelFilter::INFO,
            )
            .context("failed to initialize tracing")?
        };

        let dataflow_path =
            resolve_dataflow(self.dataflow.clone()).context("could not resolve dataflow")?;
        build_dataflow(
            dataflow_path.to_string_lossy().into_owned(),
            None,
            None,
            self.uv,
            true,
            false,
            self.locked,
            self.write_lockfile,
            self.lockfile.clone(),
            false, // sequential build for `dora run`
        )
        .context("failed to build dataflow before run")?;
        let dataflow_session = DataflowSession::read_session(&dataflow_path)
            .context("failed to read DataflowSession")?;

        let working_dir = dataflow_path
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."));
        let mut dataflow_descriptor = Descriptor::blocking_read(&dataflow_path)
            .wrap_err_with(|| {
                format!(
                    "failed to read dataflow at `{}`\n\n  \
                     hint: check the file exists and is valid YAML",
                    dataflow_path.display()
                )
            })?
            .expand(working_dir)
            .wrap_err("failed to expand modules in dataflow descriptor")?;
        if self.debug {
            dataflow_descriptor.debug.enable_debug_inspection = true;
        }

        // Validate: dora run doesn't support deploy keys
        if let Some(node) = dataflow_descriptor
            .nodes
            .iter()
            .find(|n| n.deploy.is_some())
        {
            bail!(
                "node {} has a `deploy` section, which is not supported in `dora run`\n\n\
                 Instead, you need to spawn a `dora coordinator` and one or more `dora daemon`\n\
                 instances and then use `dora start`.",
                node.id
            );
        }

        // --- Spawn embedded coordinator ---
        log::warn!(
            "embedded coordinator has no authentication enabled; \
             any local process can connect and control dataflows. \
             For production use, run `dora up --auth` instead."
        );
        let bind_addr: SocketAddr = (LOCALHOST, DORA_COORDINATOR_PORT_WS_DEFAULT).into();
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        #[cfg(feature = "tracing")]
        let span_store: SpanStore = None;
        #[cfg(not(feature = "tracing"))]
        let span_store: SpanStore = ();

        let (coordinator_port, coordinator_handle) = rt.block_on(async {
            let (port, future) = dora_coordinator::start_embedded(
                bind_addr,
                futures::stream::empty(),
                store,
                span_store,
            )
            .await
            .context("failed to start embedded coordinator")?;

            let handle = tokio::spawn(future);
            Ok::<_, eyre::Report>((port, handle))
        })?;

        let coordinator_addr: SocketAddr = (LOCALHOST, coordinator_port).into();

        // --- Spawn embedded daemon ---
        // Pass build info from the session so git-source nodes can be found.
        let initial_builds = match (
            dataflow_session.build_id,
            dataflow_session.local_build.clone(),
        ) {
            (Some(id), Some(info)) => {
                let mut m = BTreeMap::new();
                m.insert(id, info);
                m
            }
            _ => BTreeMap::new(),
        };
        let daemon_handle = rt.spawn(dora_daemon::Daemon::run_with_builds(
            coordinator_addr,
            None,
            BTreeMap::new(),
            0,
            initial_builds,
        ));

        // --- Wait for coordinator to accept connections ---
        let session = connect_with_retry(coordinator_addr, Duration::from_secs(10))
            .context("failed to connect to embedded coordinator")?;

        // --- Wait for daemon to connect (poll on the same session) ---
        {
            let deadline = std::time::Instant::now() + Duration::from_secs(10);
            loop {
                match daemon_running(&session) {
                    Ok(true) => break,
                    Ok(false) if std::time::Instant::now() < deadline => {
                        std::thread::sleep(Duration::from_millis(100));
                    }
                    Ok(false) => {
                        bail!("timed out waiting for daemon to connect to coordinator");
                    }
                    Err(e) => return Err(e).context("failed to check daemon status"),
                }
            }
        }

        // --- Start the dataflow via coordinator ---
        let local_working_dir = Some(
            dunce::canonicalize(&dataflow_path)
                .context("failed to canonicalize dataflow file path")?
                .parent()
                .context("dataflow path has no parent dir")?
                .to_owned(),
        );

        let dataflow_id = {
            let reply_raw = session
                .request(
                    &serde_json::to_vec(&ControlRequest::Start {
                        build_id: dataflow_session.build_id,
                        session_id: dataflow_session.session_id,
                        dataflow: dataflow_descriptor.clone(),
                        name: None,
                        local_working_dir,
                        uv: self.uv,
                        write_events_to: write_events_to(),
                    })
                    .unwrap(),
                )
                .context("failed to send start dataflow message")?;

            let result: ControlRequestReply =
                serde_json::from_slice(&reply_raw).context("failed to parse reply")?;
            match result {
                ControlRequestReply::DataflowStartTriggered { uuid } => uuid,
                ControlRequestReply::Error(err) => bail!("{err}"),
                other => bail!("unexpected start dataflow reply: {other:?}"),
            }
        };

        // --- Wait for spawn ---
        {
            let reply_raw = session
                .request(
                    &serde_json::to_vec(&ControlRequest::WaitForSpawn { dataflow_id }).unwrap(),
                )
                .context("failed to send WaitForSpawn message")?;
            let result: ControlRequestReply =
                serde_json::from_slice(&reply_raw).context("failed to parse reply")?;
            match result {
                ControlRequestReply::DataflowSpawned { uuid: _ } => {}
                ControlRequestReply::Error(err) => bail!(
                    "dataflow failed to start: {err}\n\n  \
                     hint: if nodes require building, run `dora build <dataflow.yml>` first"
                ),
                other => bail!("unexpected WaitForSpawn reply: {other:?}"),
            }
        }

        // --- Subscribe to logs ---
        let node_filters = match &self.log_filter {
            Some(filter) => parse_log_filter(filter).map_err(|e| eyre::eyre!(e))?,
            None => Default::default(),
        };

        let log_level = match &self.log_level {
            LogLevelOrStdout::LogLevel(level) => match level {
                ::log::Level::Error => ::log::LevelFilter::Error,
                ::log::Level::Warn => ::log::LevelFilter::Warn,
                ::log::Level::Info => ::log::LevelFilter::Info,
                ::log::Level::Debug => ::log::LevelFilter::Debug,
                ::log::Level::Trace => ::log::LevelFilter::Trace,
            },
            LogLevelOrStdout::Stdout => ::log::LevelFilter::Trace,
        };

        let log_config = LogOutputConfig {
            min_level: self.log_level,
            format: self.log_format,
            node_filters,
            print_dataflow_id: false,
            print_daemon_name: false,
        };

        let log_rx = session
            .subscribe_logs(
                &serde_json::to_vec(&ControlRequest::LogSubscribe {
                    dataflow_id,
                    level: log_level,
                })
                .context("failed to serialize log subscribe")?,
            )
            .context("failed to subscribe to logs")?;

        let log_event_tx = event_tx.clone();
        std::thread::spawn(move || {
            // Forward log messages to the print loop.
            // When the log subscription closes (coordinator shuts down), the thread exits.
            while let Ok(raw) = log_rx.recv() {
                let parsed: eyre::Result<LogMessage> = match raw {
                    Ok(bytes) => {
                        serde_json::from_slice(&bytes).context("failed to parse log message")
                    }
                    Err(err) => Err(err),
                };
                match parsed {
                    Ok(msg) => print_log_message(msg, &log_config),
                    Err(err) => tracing::warn!("failed to parse log message: {err:#}"),
                }
            }
            // Signal the main loop when log subscription closes (coordinator shutting down)
            let _ = log_event_tx;
        });

        // --- Stop-after timer ---
        if let Some(stop_after) = self.stop_after {
            let stop_tx = event_tx;
            std::thread::spawn(move || {
                std::thread::sleep(stop_after);
                let _ = stop_tx.send(RunEvent::StopAfterElapsed);
            });
        }

        // --- Main wait loop ---
        let mut ctrlc_count = 0u32;
        let result = loop {
            match event_rx.recv_timeout(Duration::from_secs(1)) {
                Ok(RunEvent::CtrlC) => {
                    ctrlc_count += 1;
                    if ctrlc_count >= 2 {
                        // Force stop
                        let _ = session.request(
                            &serde_json::to_vec(&ControlRequest::Stop {
                                dataflow_uuid: dataflow_id,
                                grace_duration: None,
                                force: true,
                            })
                            .unwrap(),
                        );
                        break None;
                    }
                    // Graceful stop
                    let _ = session.request(
                        &serde_json::to_vec(&ControlRequest::Stop {
                            dataflow_uuid: dataflow_id,
                            grace_duration: None,
                            force: false,
                        })
                        .unwrap(),
                    );
                }
                Ok(RunEvent::StopAfterElapsed) => {
                    let _ = session.request(
                        &serde_json::to_vec(&ControlRequest::Stop {
                            dataflow_uuid: dataflow_id,
                            grace_duration: None,
                            force: false,
                        })
                        .unwrap(),
                    );
                }
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    // Poll dataflow status
                    let reply_raw = session.request(
                        &serde_json::to_vec(&ControlRequest::Check {
                            dataflow_uuid: dataflow_id,
                        })
                        .unwrap(),
                    );
                    match reply_raw {
                        Ok(raw) => {
                            let result: ControlRequestReply = match serde_json::from_slice(&raw) {
                                Ok(r) => r,
                                Err(_) => continue,
                            };
                            if let ControlRequestReply::DataflowStopped { uuid, result } = result {
                                break Some((uuid, result));
                            }
                        }
                        Err(_) => {
                            // Connection lost (coordinator gone), break out
                            break None;
                        }
                    }
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    // All senders dropped, break out
                    break None;
                }
            }
        };

        // --- Cleanup: destroy coordinator + daemon ---
        let _ = session.request(&serde_json::to_vec(&ControlRequest::Destroy).unwrap());
        drop(session);

        // Wait for coordinator and daemon to finish
        rt.block_on(async {
            let _ = coordinator_handle.await;
            let _ = daemon_handle.await;
        });

        match result {
            Some((uuid, dataflow_result)) => handle_dataflow_result(dataflow_result, Some(uuid)),
            None => Ok(()),
        }
    }
}
