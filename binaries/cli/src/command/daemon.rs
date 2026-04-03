use super::Executable;
use crate::{common::handle_dataflow_result, session::DataflowSession};
use dora_core::topics::{
    DORA_COORDINATOR_PORT_DEFAULT, DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST,
};

use dora_daemon::LogDestination;
use eyre::Context;
use std::{
    net::{IpAddr, SocketAddr},
    path::PathBuf,
};
use tracing::level_filters::LevelFilter;

#[derive(Debug, clap::Args)]
/// Run daemon
pub struct Daemon {
    /// Unique identifier for the machine (required for distributed dataflows)
    #[clap(long)]
    machine_id: Option<String>,
    /// Local listen port for event such as dynamic node.
    #[clap(long, default_value_t = DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT)]
    local_listen_port: u16,
    /// Address and port number of the dora coordinator
    #[clap(long, short, default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, default_value_t = DORA_COORDINATOR_PORT_DEFAULT)]
    coordinator_port: u16,
    #[clap(long, hide = true)]
    run_dataflow: Option<PathBuf>,
    /// Suppresses all log output to stdout.
    #[clap(long)]
    quiet: bool,
}

impl Executable for Daemon {
    async fn execute(self) -> eyre::Result<()> {
        #[cfg(feature = "tracing")]
        let _guard = {
            let name = "dora-daemon";
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

            dora_tracing::init_tracing_subscriber(
                name,
                stdout_filter.as_deref(),
                Some(&filename),
                LevelFilter::INFO,
            )
            .context("failed to initialize tracing")?
        };
        async {
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

                    let result = dora_daemon::Daemon::run_dataflow(&dataflow_path,
                        dataflow_session.build_id, dataflow_session.local_build, dataflow_session.session_id, false,
                        LogDestination::Tracing, None, None,
                    ).await?;
                    handle_dataflow_result(result, None)
                }
                None => {
                    dora_daemon::Daemon::run(SocketAddr::new(self.coordinator_addr, self.coordinator_port), self.machine_id, self.local_listen_port).await
                }
            }
        }
        .await
        .context("failed to run dora-daemon")
    }
}
