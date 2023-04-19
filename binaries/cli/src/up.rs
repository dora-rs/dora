use crate::{check::daemon_running, connect_to_coordinator};
use dora_core::topics::ControlRequest;
use eyre::Context;
use std::{fs, path::Path, process::Command, time::Duration};

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct UpConfig {}

pub(crate) fn up(
    config_path: Option<&Path>,
    coordinator: Option<&Path>,
    daemon: Option<&Path>,
) -> eyre::Result<()> {
    let UpConfig {} = parse_dora_config(config_path)?;

    let mut session = match connect_to_coordinator() {
        Ok(session) => session,
        Err(_) => {
            start_coordinator(coordinator).wrap_err("failed to start dora-coordinator")?;

            loop {
                match connect_to_coordinator() {
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
        start_daemon(daemon).wrap_err("failed to start dora-daemon")?;
    }

    Ok(())
}

pub(crate) fn destroy(config_path: Option<&Path>) -> Result<(), eyre::ErrReport> {
    let UpConfig {} = parse_dora_config(config_path)?;

    match connect_to_coordinator() {
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

fn start_coordinator(coordinator: Option<&Path>) -> eyre::Result<()> {
    let coordinator = coordinator.unwrap_or_else(|| Path::new("dora-coordinator"));

    let mut cmd = Command::new(coordinator);
    cmd.spawn()
        .wrap_err_with(|| format!("failed to run {}", coordinator.display()))?;

    println!("started dora coordinator");

    Ok(())
}

fn start_daemon(daemon: Option<&Path>) -> eyre::Result<()> {
    let daemon = daemon.unwrap_or_else(|| Path::new("dora-daemon"));

    let mut cmd = Command::new(daemon);
    cmd.spawn()
        .wrap_err_with(|| format!("failed to run {}", daemon.display()))?;

    println!("started dora daemon");

    Ok(())
}
