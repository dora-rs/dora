use super::Executable;
use crate::{common::handle_dataflow_result, session::DataflowSession};
use adora_core::topics::{
    ADORA_COORDINATOR_PORT_WS_DEFAULT, ADORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST,
};

use adora_daemon::LogDestination;
use eyre::Context;
use std::{
    net::{IpAddr, SocketAddr},
    path::PathBuf,
};
use tokio::runtime::Builder;
use tracing::level_filters::LevelFilter;

#[derive(Debug, clap::Args)]
/// Run daemon
pub struct Daemon {
    /// Unique identifier for the machine (required for distributed dataflows)
    #[clap(long)]
    machine_id: Option<String>,
    /// Local listen port for event such as dynamic node.
    #[clap(long, default_value_t = ADORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT)]
    local_listen_port: u16,
    /// Address and port number of the adora coordinator
    #[clap(long, short, default_value_t = LOCALHOST, env = "ADORA_COORDINATOR_ADDR")]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator WebSocket server
    #[clap(long, default_value_t = ADORA_COORDINATOR_PORT_WS_DEFAULT, env = "ADORA_COORDINATOR_PORT")]
    coordinator_port: u16,
    #[clap(long, hide = true)]
    run_dataflow: Option<PathBuf>,
    /// Suppresses all log output to stdout.
    #[clap(long)]
    quiet: bool,
    /// Allow shell nodes to execute arbitrary commands.
    ///
    /// Shell nodes are disabled by default for security reasons. This flag
    /// sets the ADORA_ALLOW_SHELL_NODES environment variable.
    #[clap(long)]
    allow_shell_nodes: bool,
}

impl Executable for Daemon {
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

            let name = "adora-daemon";
            let filename = self
                .machine_id
                .as_ref()
                .map(|id| format!("{name}-{id}"))
                .unwrap_or(name.to_string());
            let quiet = self.quiet;

            let stdout_filter = if !quiet {
                Some(std::env::var("RUST_LOG").unwrap_or("info".to_string()))
            } else {
                None
            };

            adora_tracing::init_tracing_subscriber(
                name,
                stdout_filter.as_deref(),
                Some(&filename),
                LevelFilter::INFO,
            )
            .context("failed to initialize tracing")?
        };
        rt.block_on(async {
                match self.run_dataflow {
                    Some(dataflow_path) => {
                        tracing::info!("Starting dataflow `{}`", dataflow_path.display());
                        if self.coordinator_addr != LOCALHOST {
                            tracing::info!(
                                "Not using coordinator addr {} as `run_dataflow` is for local dataflow only. Please use the `start` command for remote coordinator",
                                self.coordinator_addr
                            );
                        }
                        let dataflow_session =
                            DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;

                        let result = adora_daemon::Daemon::run_dataflow(&dataflow_path,
                            dataflow_session.build_id, dataflow_session.local_build, dataflow_session.session_id, false,
                            LogDestination::Tracing, None, None,
                        ).await?;
                        handle_dataflow_result(result, None)
                    }
                    None => {
                        adora_daemon::Daemon::run(SocketAddr::new(self.coordinator_addr, self.coordinator_port), self.machine_id, self.local_listen_port).await
                    }
                }
            })
            .context("failed to run adora-daemon")
    }
}
