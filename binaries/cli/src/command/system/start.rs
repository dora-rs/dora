use crate::command::{Executable, default_tracing};
use crate::{LOCALHOST, common::connect_to_coordinator};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{Context, ContextCompat, bail};
use indicatif::{ProgressBar, ProgressStyle};
use std::net::{IpAddr, SocketAddr};
use std::path::PathBuf;
use std::{fs, path::Path, process::Command, time::Duration};

use super::status::daemon_running;

#[derive(Debug, clap::Args)]
/// Start coordinator and daemon in local mode (with default config)
pub struct Start {
    /// Use a custom configuration
    #[clap(long, hide = true, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: Option<PathBuf>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
}

impl Executable for Start {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let coordinator_addr = (self.coordinator_addr, self.coordinator_port).into();
        start_system(self.config.as_deref(), coordinator_addr)
    }
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct StartConfig {}

fn create_spinner(msg: &str) -> ProgressBar {
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::default_spinner()
            .tick_chars("⣾⣽⣻⢿⡿⣟⣯⣷")
            .template("{spinner} {msg}")
            .unwrap(),
    );
    pb.set_message(msg.to_string());
    pb.enable_steady_tick(Duration::from_millis(80));
    pb
}

fn print_success(msg: &str) {
    println!("\x1b[32m✓\x1b[0m {}", msg);
}

fn print_error(msg: &str) {
    eprintln!("\x1b[31m✗\x1b[0m {}", msg);
}

pub fn start_system(config_path: Option<&Path>, coordinator_addr: SocketAddr) -> eyre::Result<()> {
    let StartConfig {} = parse_config(config_path)?;

    // Start coordinator
    let spinner = create_spinner("Starting coordinator...");
    let mut session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => {
            spinner.finish_and_clear();
            print_success(&format!(
                "Coordinator already running on {}:{}",
                coordinator_addr.ip(),
                coordinator_addr.port()
            ));
            session
        }
        Err(_) => {
            if let Err(e) = start_coordinator() {
                spinner.finish_and_clear();
                print_error("Failed to start coordinator");
                return Err(e.wrap_err("failed to start dora-coordinator"));
            }

            // Wait for coordinator to accept connections
            let mut attempts = 0;
            loop {
                match connect_to_coordinator(coordinator_addr) {
                    Ok(session) => {
                        spinner.finish_and_clear();
                        print_success(&format!(
                            "Coordinator started on {}:{}",
                            coordinator_addr.ip(),
                            coordinator_addr.port()
                        ));
                        break session;
                    }
                    Err(_) => {
                        attempts += 1;
                        if attempts > 100 {
                            spinner.finish_and_clear();
                            print_error("Coordinator failed to start");
                            bail!(
                                "coordinator not responding after {}s",
                                attempts as f32 * 0.05
                            );
                        }
                        std::thread::sleep(Duration::from_millis(50));
                    }
                }
            }
        }
    };

    // Start daemon
    let spinner = create_spinner("Starting daemon...");
    if daemon_running(&mut *session)? {
        let pid = get_daemon_pid(&mut *session).unwrap_or(0);
        spinner.finish_and_clear();
        print_success(&format!("Daemon already running (PID: {})", pid));
    } else {
        if let Err(e) = start_daemon() {
            spinner.finish_and_clear();
            print_error("Failed to start daemon");
            return Err(e.wrap_err("failed to start dora-daemon"));
        }

        // Wait for daemon to connect
        let mut attempts = 0;
        const WAIT_S: f32 = 0.1;
        loop {
            if daemon_running(&mut *session)? {
                let pid = get_daemon_pid(&mut *session).unwrap_or(0);
                spinner.finish_and_clear();
                print_success(&format!("Daemon started (PID: {})", pid));
                break;
            }
            attempts += 1;
            if attempts > 20 {
                spinner.finish_and_clear();
                print_error("Daemon failed to connect");
                bail!("daemon not connected after {}s", WAIT_S * attempts as f32);
            }
            std::thread::sleep(Duration::from_secs_f32(WAIT_S));
        }
    }

    Ok(())
}

fn get_daemon_pid(session: &mut TcpRequestReplyConnection) -> Option<u32> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::GetNodeInfo).ok()?)
        .ok()?;

    let reply: ControlRequestReply = serde_json::from_slice(&reply_raw).ok()?;
    match reply {
        ControlRequestReply::NodeInfoList(nodes) => nodes
            .first()
            .and_then(|n| n.metrics.as_ref())
            .map(|m| m.pid),
        _ => None,
    }
}

fn parse_config(config_path: Option<&Path>) -> Result<StartConfig, eyre::ErrReport> {
    let path = config_path.or_else(|| Some(Path::new("dora-config.yml")).filter(|p| p.exists()));
    let config = match path {
        Some(path) => {
            let raw = fs::read_to_string(path)
                .with_context(|| format!("failed to read `{}`", path.display()))?;
            serde_yaml::from_str(&raw)
                .with_context(|| format!("failed to parse `{}`", path.display()))?
        }
        None => Default::default(),
    };
    Ok(config)
}

fn start_coordinator() -> eyre::Result<()> {
    let path = if cfg!(feature = "python") {
        std::env::args_os()
            .nth(1)
            .context("Could not get first argument correspond to dora with python installation")?
    } else {
        std::env::args_os()
            .next()
            .context("Could not get dora path")?
    };
    let mut cmd = Command::new(path);
    cmd.arg("coordinator");
    cmd.arg("--quiet");
    cmd.spawn().wrap_err("failed to run `dora coordinator`")?;
    Ok(())
}

fn start_daemon() -> eyre::Result<()> {
    let path = if cfg!(feature = "python") {
        std::env::args_os()
            .nth(1)
            .context("Could not get first argument correspond to dora with python installation")?
    } else {
        std::env::args_os()
            .next()
            .context("Could not get dora path")?
    };
    let mut cmd = Command::new(path);
    cmd.arg("daemon");
    cmd.arg("--quiet");
    cmd.spawn().wrap_err("failed to run `dora daemon`")?;
    Ok(())
}
