//! The `dora run` command is a quick and easy way to run a dataflow locally.
//! It does not support distributed dataflows and will throw an error if there are any `deploy` keys in the YAML file.
//!
//! The `dora run` command does not interact with any `dora coordinator` or `dora daemon` instances, or with any other parallel `dora run` commands.
//!
//! Use `dora build --local` or manual build commands to build your nodes.

use super::Executable;
use crate::{
    common::{handle_dataflow_result, resolve_dataflow, write_events_to},
    hot_reload::setup_file_watcher,
    output::print_log_message,
    session::DataflowSession,
};
use dora_core::descriptor::{Descriptor, DescriptorExt};
use dora_daemon::{Daemon, LogDestination, flume};
use dora_tracing::TracingBuilder;
use eyre::Context;
use std::sync::mpsc;
use tokio::runtime::Builder;
use uuid::Uuid;

#[derive(Debug, clap::Args)]
/// Run a dataflow locally.
///
/// Directly runs the given dataflow without connecting to a dora
/// coordinator or daemon. The dataflow is executed on the local machine.
pub struct Run {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    dataflow: String,
    // Use UV to run nodes.
    #[clap(long, action)]
    uv: bool,
    /// Enable hot-reload: watch node binaries and restart on changes.
    #[clap(long, action)]
    hot_reload: bool,
}

#[deprecated(note = "use `run` instead")]
pub fn run_func(dataflow: String, uv: bool) -> eyre::Result<()> {
    run(dataflow, uv, false)
}

pub fn run(dataflow: String, uv: bool, hot_reload: bool) -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
    {
        let log_level = std::env::var("RUST_LOG").ok().unwrap_or("info".to_string());
        TracingBuilder::new("run")
            .with_stdout(log_level, false)
            .build()
            .wrap_err("failed to set up tracing subscriber")?;
    }

    let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;

    // Set up file watcher for hot-reload if enabled
    let (reload_receiver, _watcher) = if hot_reload {
        let working_dir = dataflow_path
            .canonicalize()
            .context("failed to canonicalize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
            .to_owned();

        let descriptor = Descriptor::blocking_read(&dataflow_path).context("could not read dataflow")?;
        let nodes = descriptor.resolve_aliases_and_set_defaults()?;

        // Use a temporary dataflow_id - the actual one is assigned in the daemon
        let dataflow_id = Uuid::nil();

        let (watcher, reload_rx) = setup_file_watcher(dataflow_id, &nodes, &working_dir)
            .context("failed to setup file watcher")?;

        // Create a channel to forward reload events to the daemon
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            while let Ok(event) = reload_rx.recv() {
                if tx.send((event.node_id, event.operator_id)).is_err() {
                    break;
                }
            }
        });

        (Some(rx), Some(watcher))
    } else {
        (None, None)
    };

    let rt = Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;

    let (log_tx, log_rx) = flume::bounded(100);
    std::thread::spawn(move || {
        for message in log_rx {
            print_log_message(message, false, false);
        }
    });

    let result = rt.block_on(Daemon::run_dataflow(
        &dataflow_path,
        dataflow_session.build_id,
        dataflow_session.local_build,
        dataflow_session.session_id,
        uv,
        LogDestination::Channel { sender: log_tx },
        write_events_to(),
        hot_reload,
        reload_receiver,
    ))?;
    handle_dataflow_result(result, None)
}

impl Executable for Run {
    fn execute(self) -> eyre::Result<()> {
        run(self.dataflow, self.uv, self.hot_reload)
    }
}
