//! The `dora run` command is a quick and easy way to run a dataflow locally.
//! It does not support distributed dataflows and will throw an error if there are any `deploy` keys in the YAML file.
//!
//! The `dora run` command does not interact with any `dora coordinator` or `dora daemon` instances, or with any other parallel `dora run` commands.
//!
//! Use `dora build --local` or manual build commands to build your nodes.

use super::Executable;
use crate::{
    common::{handle_dataflow_result, resolve_dataflow, write_events_to},
    hot_reload::{setup_comprehensive_watcher, DataflowChangeEvent, HotReloadEvent},
    output::print_log_message,
    session::DataflowSession,
};
use dora_core::descriptor::{Descriptor, DescriptorExt};
use dora_daemon::{Daemon, DaemonHotReloadEvent, LogDestination, flume};
use duration_str::parse as parse_duration_str;
use eyre::Context;
use std::sync::mpsc;
use std::time::Duration;
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
    /// Enable hot-reload: watch node binaries and restart on changes.
    #[clap(long, action)]
    pub hot_reload: bool,
}

impl Run {
    pub fn new(dataflow: String) -> Self {
        Self {
            dataflow,
            uv: false,
            stop_after: None,
            hot_reload: false,
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
    run.execute()
}

impl Executable for Run {
    fn execute(self) -> eyre::Result<()> {
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
            resolve_dataflow(self.dataflow).context("could not resolve dataflow")?;
        let dataflow_session = DataflowSession::read_session(&dataflow_path)
            .context("failed to read DataflowSession")?;

        // Hot-reload file watcher setup
        let (reload_receiver, _watcher) = if self.hot_reload {
            let working_dir = dataflow_path
                .canonicalize()
                .context("failed to canonicalize dataflow path")?
                .parent()
                .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
                .to_owned();

            let descriptor =
                Descriptor::blocking_read(&dataflow_path).context("could not read dataflow")?;
            let nodes = descriptor.resolve_aliases_and_set_defaults()?;

            // Use a temporary dataflow_id - the actual one is assigned in the daemon
            let dataflow_id = Uuid::nil();

            let (watcher, hot_reload_rx) = setup_comprehensive_watcher(
                dataflow_id,
                &dataflow_path,
                &nodes,
                &working_dir,
            )
            .context("failed to setup comprehensive file watcher")?;

            // Create a channel to forward reload events to the daemon
            let (tx, rx) = mpsc::channel::<DaemonHotReloadEvent>();
            std::thread::spawn(move || {
                while let Ok(event) = hot_reload_rx.recv() {
                    match event {
                        HotReloadEvent::FileChanged(reload_event) => {
                            if tx
                                .send(DaemonHotReloadEvent::Reload {
                                    node_id: reload_event.node_id,
                                    operator_id: reload_event.operator_id,
                                })
                                .is_err()
                            {
                                break;
                            }
                        }
                        HotReloadEvent::DataflowChanged {
                            changes,
                            new_descriptor,
                            new_nodes,
                        } => {
                            for change in changes {
                                let event = match change {
                                    DataflowChangeEvent::NodeAdded { node_id, .. } => {
                                        if let Some(node) = new_nodes.get(&node_id) {
                                            tracing::info!(
                                                "Hot-reload: spawning new node '{}' from YAML",
                                                node_id
                                            );
                                            DaemonHotReloadEvent::SpawnNode {
                                                node_id,
                                                node: node.clone(),
                                                new_descriptor: new_descriptor.clone(),
                                            }
                                        } else {
                                            tracing::warn!(
                                                "Hot-reload: node '{}' added but not found in resolved nodes",
                                                node_id
                                            );
                                            continue;
                                        }
                                    }
                                    DataflowChangeEvent::NodeRemoved { node_id } => {
                                        tracing::info!(
                                            "Hot-reload: stopping removed node '{}' from YAML",
                                            node_id
                                        );
                                        DaemonHotReloadEvent::StopNode { node_id }
                                    }
                                    DataflowChangeEvent::NodeChanged { node_id, new_node } => {
                                        tracing::info!(
                                            "Hot-reload: restarting node '{}' with new config",
                                            node_id
                                        );
                                        DaemonHotReloadEvent::RestartNode {
                                            node_id,
                                            new_node,
                                            new_descriptor: new_descriptor.clone(),
                                        }
                                    }
                                };
                                if tx.send(event).is_err() {
                                    break;
                                }
                            }
                        }
                    }
                }
            });

            (Some(rx), Some(watcher))
        } else {
            (None, None)
        };

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
            self.uv,
            LogDestination::Channel { sender: log_tx },
            write_events_to(),
            self.stop_after,
            self.hot_reload,
            reload_receiver,
        ))?;
        handle_dataflow_result(result, None)
    }
}
