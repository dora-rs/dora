use super::Executable;
use crate::{common::handle_dataflow_result, session::DataflowSession};
use dora_core::{
    descriptor::DescriptorExt,
    topics::{
        DORA_COORDINATOR_PORT_WS_DEFAULT, DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT,
        DORA_DAEMON_LOCAL_LISTEN_PORT_ENV, LOCALHOST,
    },
};

use dora_daemon::LogDestination;
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
    #[clap(long, default_value_t = DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT)]
    local_listen_port: u16,
    /// Address and port number of the dora coordinator
    #[clap(long, short, default_value_t = LOCALHOST, env = "DORA_COORDINATOR_ADDR")]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator WebSocket server
    #[clap(long, default_value_t = DORA_COORDINATOR_PORT_WS_DEFAULT, env = "DORA_COORDINATOR_PORT")]
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
    /// sets the DORA_ALLOW_SHELL_NODES environment variable.
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
            unsafe { std::env::set_var("DORA_ALLOW_SHELL_NODES", "true") };
        }
        // Export the listen port so dynamic nodes (and spawned child processes)
        // can discover it via env var.
        // SAFETY: Called before the tokio runtime is built (no threads yet).
        unsafe {
            std::env::set_var(
                DORA_DAEMON_LOCAL_LISTEN_PORT_ENV,
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
        //
        // These diagnostics use `eprintln!` rather than `tracing::*` because
        // the tracing subscriber is not yet installed at this point (see the
        // `init_tracing_subscriber` call below) — `tracing::info!`/`warn!`
        // calls before global subscriber registration route to
        // `NoSubscriber` and are silently dropped (#1701). Going to stderr
        // also ensures these are visible even with `--quiet`, which matters
        // because a failed RT setup is an operational warning the user must
        // see.
        if self.rt {
            #[cfg(unix)]
            {
                // Lock all memory to prevent page faults.
                let lock_result = unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) };
                if lock_result == 0 {
                    eprintln!("RT: mlockall enabled (memory locked)");
                } else {
                    eprintln!(
                        "RT: mlockall failed: {}. Ensure CAP_IPC_LOCK or ulimit -l unlimited.",
                        std::io::Error::last_os_error()
                    );
                }

                // Set SCHED_FIFO priority 50 (Linux only).
                #[cfg(target_os = "linux")]
                {
                    // Use zeroed() + field set instead of struct literal
                    // because musl libc's sched_param has extra POSIX
                    // fields (sched_ss_*) that glibc doesn't expose
                    // (dora-rs/adora#170).
                    let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
                    param.sched_priority = 50;
                    let sched_result =
                        unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };
                    if sched_result == 0 {
                        eprintln!("RT: SCHED_FIFO priority 50 enabled");
                    } else {
                        eprintln!(
                            "RT: sched_setscheduler failed: {}. Ensure CAP_SYS_NICE.",
                            std::io::Error::last_os_error()
                        );
                    }
                }
                // Note: the previous "(mlockall applied)" parenthetical here
                // could lie on macOS where `mlockall` returns ENOTSUP, so the
                // message now stands on its own.
                #[cfg(not(target_os = "linux"))]
                eprintln!("RT: SCHED_FIFO not available on this platform");
            }
            #[cfg(not(unix))]
            eprintln!("RT: --rt flag is only supported on Unix systems");
        }

        #[cfg(feature = "tracing")]
        let _guard = {
            let _enter = rt.enter();

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
                        let mut dataflow_session =
                            DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;
                        // Invalidate cached build metadata if the descriptor's
                        // build-inputs changed since the last `dora build`.
                        // Without this, the daemon would consume a stale
                        // `build_id` and spawn nodes from the previous build's
                        // artifacts (#1444).
                        let dataflow_descriptor = dora_core::descriptor::Descriptor::blocking_read(&dataflow_path)
                            .wrap_err_with(|| format!(
                                "failed to read dataflow at `{}` for session fingerprinting",
                                dataflow_path.display()
                            ))?;
                        let working_dir = dataflow_path
                            .parent()
                            .filter(|p| !p.as_os_str().is_empty())
                            .unwrap_or_else(|| std::path::Path::new("."));
                        let expanded = dataflow_descriptor
                            .expand(working_dir)
                            .wrap_err("failed to expand modules in dataflow descriptor")?;
                        let resolved_for_fingerprint = expanded
                            .resolve_aliases_and_set_defaults()
                            .context("failed to resolve nodes for session fingerprint")?;
                        if dataflow_session.invalidate_if_build_inputs_changed(&resolved_for_fingerprint) {
                            dataflow_session
                                .write_out_for_dataflow(&dataflow_path)
                                .context("failed to persist invalidated dataflow session")?;
                        }
                        drop(resolved_for_fingerprint);

                        let result = dora_daemon::Daemon::run_dataflow(&dataflow_path,
                            dataflow_session.build_id, dataflow_session.local_build, dataflow_session.session_id, false,
                            LogDestination::Tracing, None, None, false, None,
                        ).await?;
                        handle_dataflow_result(result, None)
                    }
                    None => {
                        dora_daemon::Daemon::run(SocketAddr::new(self.coordinator_addr, self.coordinator_port), self.machine_id, self.labels.unwrap_or_default(), self.local_listen_port).await
                    }
                }
            })
            .context("failed to run dora-daemon")
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
