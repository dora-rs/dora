use crate::{check::daemon_running, connect_to_coordinator};
use dora_core::topics::ControlRequest;
use eyre::Context;
use std::{fs, net::SocketAddr, path::Path, process::Command, time::Duration};
#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct UpConfig {}

pub(crate) fn up(config_path: Option<&Path>, coordinator_addr: SocketAddr) -> eyre::Result<()> {
    let UpConfig {} = parse_dora_config(config_path)?;
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
            session
                .request(&serde_json::to_vec(&ControlRequest::Destroy).unwrap())
                .wrap_err("failed to send destroy message")?;
            println!("Send destroy command to dora-coordinator");
        }
        Err(_) => {
            eprintln!("Could not connect to dora-coordinator");
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
    let mut cmd =
        Command::new(std::env::current_exe().wrap_err("failed to get current executable path")?);
    cmd.arg("coordinator");
    cmd.spawn().wrap_err("failed to run `dora coordinator`")?;

    println!("started dora coordinator");

    Ok(())
}

fn start_daemon() -> eyre::Result<()> {
    let mut cmd =
        Command::new(std::env::current_exe().wrap_err("failed to get current executable path")?);
    cmd.arg("daemon");
    cmd.spawn().wrap_err("failed to run `dora daemon`")?;

    println!("started dora daemon");

    Ok(())
}
