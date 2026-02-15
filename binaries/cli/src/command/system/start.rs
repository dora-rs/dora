use super::status::daemon_running;
use crate::LOCALHOST;
use crate::command::{Executable, default_tracing};
use crate::common::connect_to_coordinator;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use eyre::{Context, ContextCompat};
use std::path::PathBuf;
use std::time::Duration;
use std::{fs, path::Path, process::Command};

#[derive(Debug, clap::Args)]
/// Spawn coordinator and daemon in local mode (with default config)
pub struct Start {
    /// Use a custom configuration
    #[clap(long, hide = true, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: Option<PathBuf>,
}

impl Executable for Start {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        start_system(self.config.as_deref())
    }
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct UpConfig {}

pub(crate) fn start_system(config_path: Option<&Path>) -> eyre::Result<()> {
    let UpConfig {} = parse_dora_config(config_path)?;
    let coordinator_addr = (LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT).into();

    let mut session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => {
            println!("✓ Coordinator is already running on {}", coordinator_addr);
            session
        }
        Err(_) => {
            start_coordinator().wrap_err("failed to start dora-coordinator")?;

            let session = loop {
                match connect_to_coordinator(coordinator_addr) {
                    Ok(session) => break session,
                    Err(_) => {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                }
            };
            println!("✓ Coordinator started on {}", coordinator_addr);
            session
        }
    };

    if !daemon_running(&mut *session)? {
        start_daemon().wrap_err("failed to start dora-daemon")?;

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
        println!("✓ Daemon started");
    } else {
        println!("✓ Daemon is already running");
    }

    Ok(())
}

fn parse_dora_config(config_path: Option<&Path>) -> eyre::Result<UpConfig> {
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
