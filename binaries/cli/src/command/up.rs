use super::check::daemon_running;
use super::{Executable, default_tracing};
use crate::{LOCALHOST, common::connect_to_coordinator};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{Context, ContextCompat, bail};
use std::path::PathBuf;
use std::{fs, net::SocketAddr, path::Path, process::Command, time::Duration};

#[derive(Debug, clap::Args)]
/// Spawn coordinator and daemon in local mode (with default config)
pub struct Up {
    /// Use a custom configuration
    #[clap(long, hide = true, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: Option<PathBuf>,
}

impl Executable for Up {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        up(self.config.as_deref())
    }
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct UpConfig {}

pub(crate) fn up(config_path: Option<&Path>) -> eyre::Result<()> {
    let UpConfig {} = parse_dora_config(config_path)?;
    let coordinator_addr = (LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT).into();
    let mut session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => session,
        Err(_) => {
            start_coordinator().wrap_err("failed to start dora-coordinator")?;

            loop {
                match connect_to_coordinator(coordinator_addr) {
                    Ok(session) => break session,
                    Err(_) => {
                        // sleep a bit until the coordinator accepts connections
                        std::thread::sleep(Duration::from_millis(50));
                    }
                }
            }
        }
    };

    if !daemon_running(&mut *session)? {
        start_daemon().wrap_err("failed to start dora-daemon")?;

        // wait a bit until daemon is connected
        let mut i = 0;
        const WAIT_S: f32 = 0.1;
        loop {
            if daemon_running(&mut *session)? {
                break;
            }
            i += 1;
            if i > 20 {
                eyre::bail!("daemon not connected after {}s", WAIT_S * i as f32);
            }
            std::thread::sleep(Duration::from_secs_f32(WAIT_S));
        }
    }

    Ok(())
}

pub(crate) fn destroy(
    config_path: Option<&Path>,
    coordinator_addr: SocketAddr,
) -> Result<(), eyre::ErrReport> {
    let UpConfig {} = parse_dora_config(config_path)?;
    match connect_to_coordinator(coordinator_addr) {
        Ok(mut session) => {
            // send destroy command to dora-coordinator
            let reply_raw = session
                .request(&serde_json::to_vec(&ControlRequest::Destroy).unwrap())
                .wrap_err("failed to send destroy message")?;
            let result: ControlRequestReply =
                serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
            match result {
                ControlRequestReply::DestroyOk => {
                    println!("Coordinator and daemons destroyed successfully");
                }
                ControlRequestReply::Error(err) => {
                    bail!("Destroy command failed with error: {}", err);
                }
                _ => {
                    bail!("Unexpected reply from dora-coordinator");
                }
            }
        }
        Err(_) => {
            bail!("Could not connect to dora-coordinator");
        }
    }

    Ok(())
}

fn parse_dora_config(config_path: Option<&Path>) -> Result<UpConfig, eyre::ErrReport> {
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

    println!("started dora coordinator");

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

    println!("started dora daemon");

    Ok(())
}
