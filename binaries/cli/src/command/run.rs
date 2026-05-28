//! The `dora run` command runs a dataflow locally in isolation: it
//! spawns nodes in-process via `dora_daemon::Daemon::run_dataflow`
//! without binding any coordinator port. As a result, `dora run` does
//! NOT integrate with the CLI monitoring commands (`dora list`,
//! `dora stop`, `dora logs`, `dora top`, ...). Use `dora up` + `dora
//! start` instead when you need to attach those tools.
//!
//! Running isolated means:
//!   - `dora run` can be invoked while `dora up` is already running
//!     (the coordinator port is not contested).
//!   - Multiple `dora run` calls can execute in parallel.

use super::Executable;
use crate::{
    BuildConfig, build as build_dataflow,
    common::{handle_dataflow_result, resolve_dataflow, write_events_to},
    output::{
        LogFormat, LogOutputConfig, parse_log_filter, parse_log_level_str, print_log_message,
    },
    session::DataflowSession,
};
use dora_core::build::LogLevelOrStdout;
use dora_daemon::{Daemon, LogDestination, flume};
use duration_str::parse as parse_duration_str;
use eyre::Context;
use std::{path::PathBuf, time::Duration};
use tokio::runtime::Builder;

#[derive(Debug, clap::Args)]
/// Run a dataflow locally in isolation.
///
/// Spawns nodes in-process without a coordinator. CLI monitoring
/// commands (`dora list`, `dora stop`, `dora logs`, ...) do NOT attach
/// to a `dora run` execution. Use `dora up` + `dora start` instead when
/// you need to attach those tools or run multiple coordinated
/// dataflows.
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
    /// Decouples the working dir used for descriptor-relative paths
    /// from the dataflow file's parent. Needed when the path passed in
    /// is a rewritten copy (e.g. `dora record`'s tempfile) whose parent
    /// can't resolve the original `build:` / relative-path references.
    #[clap(skip)]
    pub working_dir: Option<PathBuf>,
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
            working_dir: None,
        }
    }

    pub fn with_working_dir(mut self, working_dir: PathBuf) -> Self {
        self.working_dir = Some(working_dir);
        self
    }
}

pub fn run(dataflow: String, uv: bool) -> eyre::Result<()> {
    let mut run = Run::new(dataflow);
    run.uv = uv;
    run.execute()
}

impl Executable for Run {
    fn execute(self) -> eyre::Result<()> {
        if self.allow_shell_nodes {
            // SAFETY: Called before spawning any threads (tokio runtime not yet built),
            // so there are no concurrent reads of environment variables.
            unsafe { std::env::set_var("DORA_ALLOW_SHELL_NODES", "true") };
        }

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
        build_dataflow(BuildConfig {
            dataflow: dataflow_path.to_string_lossy().into_owned(),
            uv: self.uv,
            force_local: true,
            locked: self.locked,
            write_lockfile: self.write_lockfile,
            lockfile_override: self.lockfile.clone(),
            working_dir_override: self.working_dir.clone(),
            ..Default::default()
        })
        .context("failed to build dataflow before run")?;
        let dataflow_session = DataflowSession::read_session(&dataflow_path)
            .context("failed to read DataflowSession")?;

        let node_filters = match &self.log_filter {
            Some(filter) => parse_log_filter(filter).map_err(|e| eyre::eyre!(e))?,
            None => Default::default(),
        };

        let log_config = LogOutputConfig {
            min_level: self.log_level,
            format: self.log_format,
            node_filters,
            print_dataflow_id: false,
            print_daemon_name: false,
        };

        let (log_tx, log_rx) = flume::bounded(100);
        std::thread::spawn(move || {
            for message in log_rx {
                print_log_message(message, &log_config);
            }
        });

        let result = rt.block_on(Daemon::run_dataflow(
            &dataflow_path,
            dataflow_session.build_id,
            dataflow_session.local_build,
            dataflow_session.session_id,
            self.uv,
            LogDestination::Channel { sender: log_tx },
            write_events_to(),
            self.stop_after,
            self.debug,
            self.working_dir.clone(),
        ))?;
        handle_dataflow_result(result, None)
    }
}
