use super::system::status::daemon_running;
use super::{Executable, default_tracing};
use crate::{LOCALHOST, common::connect_to_coordinator};
use adora_core::topics::ADORA_COORDINATOR_PORT_WS_DEFAULT;
use adora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{Context, ContextCompat, bail};
use std::path::PathBuf;
use std::{fs, net::SocketAddr, path::Path, process::Command, time::Duration};

#[derive(Debug, clap::Args)]
/// Spawn coordinator and daemon in local mode (with default config)
pub struct Up {
    /// Use a custom configuration
    #[clap(long, hide = true, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: Option<PathBuf>,
    /// Enable token authentication for the coordinator.
    ///
    /// When enabled, the coordinator generates a random token on startup
    /// and writes it to ~/.config/adora/.adora-token. Clients must present
    /// this token to connect.
    #[clap(long)]
    auth: bool,
}

impl Executable for Up {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        up(self.config.as_deref(), self.auth)
    }
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct UpConfig {}

pub(crate) fn up(config_path: Option<&Path>, auth: bool) -> eyre::Result<()> {
    let UpConfig {} = parse_adora_config(config_path)?;
    let addr: std::net::IpAddr = std::env::var("ADORA_COORDINATOR_ADDR")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(LOCALHOST);
    let port: u16 = std::env::var("ADORA_COORDINATOR_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(ADORA_COORDINATOR_PORT_WS_DEFAULT);
    let coordinator_addr = (addr, port).into();
    let session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => session,
        Err(_) => {
            start_coordinator(auth).wrap_err("failed to start adora-coordinator")?;

            {
                let deadline = std::time::Instant::now() + Duration::from_secs(10);
                loop {
                    match connect_to_coordinator(coordinator_addr) {
                        Ok(session) => break session,
                        Err(_) if std::time::Instant::now() < deadline => {
                            std::thread::sleep(Duration::from_millis(50));
                        }
                        Err(err) => {
                            bail!(
                                "timed out waiting for coordinator to start at {coordinator_addr}: {err}"
                            );
                        }
                    }
                }
            }
        }
    };

    if !daemon_running(&session)? {
        start_daemon().wrap_err("failed to start adora-daemon")?;

        // wait a bit until daemon is connected
        let mut i = 0;
        const WAIT_S: f32 = 0.1;
        loop {
            if daemon_running(&session)? {
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

pub(crate) fn down(
    config_path: Option<&Path>,
    coordinator_addr: SocketAddr,
) -> Result<(), eyre::ErrReport> {
    let UpConfig {} = parse_adora_config(config_path)?;
    match connect_to_coordinator(coordinator_addr) {
        Ok(session) => {
            // send destroy command to adora-coordinator
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
                    bail!("Unexpected reply from adora-coordinator");
                }
            }
        }
        Err(_) => {
            bail!(
                "could not connect to coordinator at {coordinator_addr}\n\n  \
                 hint: is it running? Start it with `adora up`"
            );
        }
    }

    Ok(())
}

fn parse_adora_config(config_path: Option<&Path>) -> Result<UpConfig, eyre::ErrReport> {
    let path = config_path.or_else(|| Some(Path::new("adora-config.yml")).filter(|p| p.exists()));
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

pub(crate) fn adora_executable_path() -> eyre::Result<std::ffi::OsString> {
    if cfg!(feature = "python") {
        // When invoked via Python wrapper, argv[1] is the real adora binary path
        std::env::args_os()
            .nth(1)
            .context("could not get adora path from Python wrapper arguments")
    } else {
        std::env::current_exe()
            .map(Into::into)
            .wrap_err("could not determine adora executable path")
    }
}

fn start_coordinator(auth: bool) -> eyre::Result<()> {
    let path = adora_executable_path()?;
    let mut cmd = Command::new(path);
    cmd.arg("coordinator");
    cmd.arg("--quiet");
    if auth {
        cmd.arg("--auth");
    }
    cmd.spawn().wrap_err(
        "failed to run `adora coordinator`\n\n  \
         hint: ensure the `adora` binary is in your PATH",
    )?;

    println!("started adora coordinator");

    Ok(())
}

fn start_daemon() -> eyre::Result<()> {
    let path = adora_executable_path()?;
    let mut cmd = Command::new(path);
    cmd.arg("daemon");
    cmd.arg("--quiet");
    cmd.spawn().wrap_err(
        "failed to run `adora daemon`\n\n  \
         hint: ensure the `adora` binary is in your PATH",
    )?;

    println!("started adora daemon");

    Ok(())
}
