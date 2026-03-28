use super::Executable;
use crate::{common::handle_dataflow_result, session::DataflowSession};
use adora_core::topics::{
    ADORA_COORDINATOR_PORT_WS_DEFAULT, ADORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT,
    ADORA_DAEMON_LOCAL_LISTEN_PORT_ENV, LOCALHOST,
};

use adora_daemon::LogDestination;
use eyre::Context;
use std::{
    collections::BTreeMap,
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
    /// Labels for this daemon (e.g. `--labels gpu=true,arch=arm64`).
    /// Used for label-based node scheduling.
    #[clap(long, value_parser = parse_labels)]
    labels: Option<BTreeMap<String, String>>,
    /// Suppresses all log output to stdout.
    #[clap(long)]
    quiet: bool,
    /// Allow shell nodes to execute arbitrary commands.
    ///
    /// Shell nodes are disabled by default for security reasons. This flag
    /// sets the ADORA_ALLOW_SHELL_NODES environment variable.
    #[clap(long)]
    allow_shell_nodes: bool,
    /// Number of tokio worker threads (default: number of CPU cores).
    #[clap(long)]
    worker_threads: Option<usize>,
    /// Enable real-time profile: mlockall + SCHED_FIFO priority.
    /// Requires CAP_SYS_NICE + CAP_IPC_LOCK capabilities.
    /// Warning: SCHED_FIFO applies to the main thread only (tokio workers
    /// are not promoted). Use with care — see docs/realtime-tuning.md.
    #[clap(long)]
    rt: bool,
}

impl Executable for Daemon {
    fn execute(self) -> eyre::Result<()> {
        if self.allow_shell_nodes {
            // SAFETY: Called before spawning any threads (tokio runtime not yet built),
            // so there are no concurrent reads of environment variables.
            unsafe { std::env::set_var("ADORA_ALLOW_SHELL_NODES", "true") };
        }
        // Export the listen port so dynamic nodes (and spawned child processes)
        // can discover it via env var.
        // SAFETY: Called before the tokio runtime is built (no threads yet).
        unsafe {
            std::env::set_var(
                ADORA_DAEMON_LOCAL_LISTEN_PORT_ENV,
                self.local_listen_port.to_string(),
            );
        }

        let mut builder = Builder::new_multi_thread();
        builder.enable_all();
        if let Some(threads) = self.worker_threads {
            builder.worker_threads(threads);
        }
        let rt = builder.build().context("tokio runtime failed")?;

        // Apply real-time profile if requested.
        if self.rt {
            #[cfg(unix)]
            {
                // Lock all memory to prevent page faults.
                let lock_result = unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) };
                if lock_result == 0 {
                    tracing::info!("RT: mlockall enabled (memory locked)");
                } else {
                    tracing::warn!(
                        "RT: mlockall failed: {}. Ensure CAP_IPC_LOCK or ulimit -l unlimited.",
                        std::io::Error::last_os_error()
                    );
                }

                // Set SCHED_FIFO priority 50 (Linux only).
                #[cfg(target_os = "linux")]
                {
                    let param = libc::sched_param { sched_priority: 50 };
                    let sched_result =
                        unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };
                    if sched_result == 0 {
                        tracing::info!("RT: SCHED_FIFO priority 50 enabled");
                    } else {
                        tracing::warn!(
                            "RT: sched_setscheduler failed: {}. Ensure CAP_SYS_NICE.",
                            std::io::Error::last_os_error()
                        );
                    }
                }
                #[cfg(not(target_os = "linux"))]
                tracing::info!("RT: SCHED_FIFO not available on this platform (mlockall applied)");
            }
            #[cfg(not(unix))]
            tracing::warn!("RT: --rt flag is only supported on Unix systems");
        }

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
                            LogDestination::Tracing, None, None, false,
                        ).await?;
                        handle_dataflow_result(result, None)
                    }
                    None => {
                        adora_daemon::Daemon::run(SocketAddr::new(self.coordinator_addr, self.coordinator_port), self.machine_id, self.labels.unwrap_or_default(), self.local_listen_port).await
                    }
                }
            })
            .context("failed to run adora-daemon")
    }
}

fn parse_labels(s: &str) -> Result<BTreeMap<String, String>, String> {
    let mut map = BTreeMap::new();
    for pair in s.split(',') {
        let pair = pair.trim();
        if pair.is_empty() {
            continue;
        }
        let (k, v) = pair
            .split_once('=')
            .ok_or_else(|| format!("invalid label `{pair}`, expected key=value"))?;
        map.insert(k.to_string(), v.to_string());
    }
    Ok(map)
}
