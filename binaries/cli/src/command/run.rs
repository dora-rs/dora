//! The `adora run` command is a quick and easy way to run a dataflow locally.
//! It does not support distributed dataflows and will throw an error if there are any `deploy` keys in the YAML file.
//!
//! The `adora run` command does not interact with any `adora coordinator` or `adora daemon` instances, or with any other parallel `adora run` commands.
//!
//! Use `adora build --local` or manual build commands to build your nodes.

use super::Executable;
use crate::{
    common::{handle_dataflow_result, resolve_dataflow, write_events_to},
    output::{
        LogFormat, LogOutputConfig, parse_log_filter, parse_log_level_str, print_log_message,
    },
    session::DataflowSession,
};
use adora_core::build::LogLevelOrStdout;
use adora_daemon::{Daemon, LogDestination, flume};
use duration_str::parse as parse_duration_str;
use eyre::Context;
use std::time::Duration;
use tokio::runtime::Builder;

#[derive(Debug, clap::Args)]
/// Run a dataflow locally.
///
/// Directly runs the given dataflow without connecting to a adora
/// coordinator or daemon. The dataflow is executed on the local machine.
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
    #[clap(long, default_value = "stdout", env = "ADORA_LOG_LEVEL")]
    #[arg(value_parser = parse_log_level_str)]
    pub log_level: LogLevelOrStdout,
    /// Output format for log messages
    #[clap(long, default_value = "pretty", env = "ADORA_LOG_FORMAT")]
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
        env = "ADORA_LOG_FILTER",
        verbatim_doc_comment
    )]
    pub log_filter: Option<String>,
    /// Allow shell nodes to execute arbitrary commands.
    ///
    /// Shell nodes are disabled by default for security reasons. This flag
    /// sets the ADORA_ALLOW_SHELL_NODES environment variable.
    #[clap(long)]
    pub allow_shell_nodes: bool,
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
        }
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
            unsafe { std::env::set_var("ADORA_ALLOW_SHELL_NODES", "true") };
        }

        let rt = Builder::new_multi_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;

        #[cfg(feature = "tracing")]
        let _guard = {
            let _enter = rt.enter();
            let env_log = std::env::var("RUST_LOG").unwrap_or("info".to_string());
            adora_tracing::init_tracing_subscriber(
                "adora-run",
                Some(&env_log),
                None,
                tracing::metadata::LevelFilter::INFO,
            )
            .context("failed to initialize tracing")?
        };

        let dataflow_path =
            resolve_dataflow(self.dataflow).context("could not resolve dataflow")?;
        let dataflow_session = DataflowSession::read_session(&dataflow_path)
            .context("failed to read DataflowSession")?;

        let node_filters = match &self.log_filter {
            Some(filter) => parse_log_filter(filter).map_err(|e| eyre::eyre!(e))?,
            None => Default::default(),
        };

        let config = LogOutputConfig {
            min_level: self.log_level,
            format: self.log_format,
            node_filters,
            print_dataflow_id: false,
            print_daemon_name: false,
        };

        let (log_tx, log_rx) = flume::bounded(100);
        std::thread::spawn(move || {
            for message in log_rx {
                print_log_message(message, &config);
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
        ))?;
        handle_dataflow_result(result, None)
    }
}
