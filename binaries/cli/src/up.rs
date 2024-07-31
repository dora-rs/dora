use crate::{DoraConnection, LOCALHOST};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use eyre::Context;
use std::{fs, path::Path, process::Command, time::Duration};

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct UpConfig {}

pub fn up(config_path: Option<&Path>, dora_cli_path: &Path) -> eyre::Result<()> {
    let UpConfig {} = parse_dora_config(config_path)?;
    let coordinator_addr = (LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT).into();
    let mut session = match DoraConnection::connect(coordinator_addr) {
        Ok(session) => session,
        Err(_) => {
            start_coordinator(dora_cli_path).wrap_err("failed to start dora-coordinator")?;

            loop {
                match DoraConnection::connect(coordinator_addr) {
                    Ok(session) => break session,
                    Err(_) => {
                        // sleep a bit until the coordinator accepts connections
                        std::thread::sleep(Duration::from_millis(50));
                    }
                }
            }
        }
    };

    if !session.daemon_running()? {
        start_daemon(dora_cli_path).wrap_err("failed to start dora-daemon")?;

        // wait a bit until daemon is connected
        let mut i = 0;
        const WAIT_S: f32 = 0.1;
        loop {
            if session.daemon_running()? {
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

fn start_coordinator(dora_cli_path: &Path) -> eyre::Result<()> {
    let mut cmd = Command::new(dora_cli_path);
    cmd.arg("coordinator");
    cmd.arg("--quiet");
    cmd.spawn().wrap_err("failed to run `dora coordinator`")?;

    println!("started dora coordinator");

    Ok(())
}

fn start_daemon(dora_cli_path: &Path) -> eyre::Result<()> {
    let mut cmd = Command::new(dora_cli_path);
    cmd.arg("daemon");
    cmd.arg("--quiet");
    cmd.spawn().wrap_err("failed to run `dora daemon`")?;

    println!("started dora daemon");

    Ok(())
}
