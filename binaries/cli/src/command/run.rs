//! The `dora run` command is a quick and easy way to run a dataflow locally.
//! It does not support distributed dataflows and will throw an error if there are any `deploy` keys in the YAML file.
//!
//! The `dora run` command does not interact with any `dora coordinator` or `dora daemon` instances, or with any other parallel `dora run` commands.
//!
//! Use `dora build --local` or manual build commands to build your nodes.

use super::Executable;
use crate::{
    common::{handle_dataflow_result, resolve_dataflow, write_events_to},
    output::print_log_message,
    session::DataflowSession,
};
use dora_daemon::{Daemon, LogDestination, flume};
use duration_str::parse as parse_duration_str;
use eyre::Context;
use std::time::Duration;

#[derive(Debug, clap::Args)]
/// Run a dataflow locally.
///
/// Directly runs the given dataflow without connecting to a dora
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
}

impl Run {
    pub fn new(dataflow: String) -> Self {
        Self {
            dataflow,
            uv: false,
            stop_after: None,
        }
    }
}

#[deprecated(note = "use `run` instead")]
pub fn run_func(dataflow: String, uv: bool) -> eyre::Result<()> {
    run(dataflow, uv)
}

pub fn run(dataflow: String, uv: bool) -> eyre::Result<()> {
    let mut run = Run::new(dataflow);
    run.uv = uv;
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;
    rt.block_on(run.execute())
}

impl Executable for Run {
    async fn execute(self) -> eyre::Result<()> {
        #[cfg(feature = "tracing")]
        let _guard = {
            let env_log = std::env::var("RUST_LOG").unwrap_or("info".to_string());
            dora_tracing::init_tracing_subscriber(
                "dora-run",
                Some(&env_log),
                None,
                tracing::metadata::LevelFilter::INFO,
            )
            .context("failed to initialize tracing")?
        };

        let dataflow_path = resolve_dataflow(self.dataflow)
            .await
            .context("could not resolve dataflow")?;
        let dataflow_session = DataflowSession::read_session(&dataflow_path)
            .context("failed to read DataflowSession")?;

        let (log_tx, log_rx) = flume::bounded(100);
        std::thread::spawn(move || {
            for message in log_rx {
                print_log_message(message, false, false);
            }
        });

        let result = Daemon::run_dataflow(
            &dataflow_path,
            dataflow_session.build_id,
            dataflow_session.local_build,
            dataflow_session.session_id,
            self.uv,
            LogDestination::Channel { sender: log_tx },
            write_events_to(),
            self.stop_after,
        )
        .await?;
        handle_dataflow_result(result, None)
    }
}
