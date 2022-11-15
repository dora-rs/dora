use crate::{check::coordinator_running, control_connection};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::ControlRequest;
use eyre::{bail, Context};
use std::{fs, path::Path, process::Command};
use sysinfo::{ProcessExt, SystemExt};

#[derive(Debug, serde::Serialize, serde::Deserialize)]
struct UpConfig {
    iceoryx: bool,
}

impl Default for UpConfig {
    fn default() -> Self {
        Self { iceoryx: true }
    }
}

pub(crate) fn up(
    config_path: Option<&Path>,
    roudi: Option<&Path>,
    coordinator: Option<&Path>,
) -> eyre::Result<()> {
    let UpConfig { iceoryx } = parse_dora_config(config_path)?;

    if !coordinator_running()? {
        start_coordinator(coordinator).wrap_err("failed to start dora-coordinator")?;
    }

    if iceoryx {
        // try to start roudi
        start_roudi(roudi).wrap_err("failed to start iceoryx roudi daemon")?;
    }

    Ok(())
}

pub(crate) fn destroy(
    config_path: Option<&Path>,
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<(), eyre::ErrReport> {
    let UpConfig { iceoryx } = parse_dora_config(config_path)?;

    if coordinator_running()? {
        // send destroy command to dora-coordinator
        control_connection(session)?
            .request(&serde_json::to_vec(&ControlRequest::Destroy).unwrap())
            .wrap_err("failed to send destroy message")?;
        println!("Send destroy command to dora-coordinator");
    } else {
        eprintln!("The dora-coordinator is not running");
    }

    if iceoryx {
        // kill iox-roudi process
        let system = sysinfo::System::new_all();
        let processes: Vec<_> = system.processes_by_exact_name("iox-roudi").collect();
        if processes.is_empty() {
            eprintln!("No `iox-roudi` process found");
        } else if processes.len() == 1 {
            let process = processes[0];
            let success = process.kill();
            if success {
                println!("Killed `iox-roudi` process");
            } else {
                bail!("failed to kill iox-roudi process");
            }
        } else {
            bail!("multiple iox-roudi processes found, please kill the correct processes manually");
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

fn start_roudi(roudi: Option<&Path>) -> eyre::Result<()> {
    let roudi = roudi.unwrap_or_else(|| Path::new("iox-roudi"));

    let mut cmd = Command::new(roudi);
    cmd.spawn()
        .wrap_err_with(|| format!("failed to run {}", roudi.display()))?;

    println!("started iox-roudi daemon");

    Ok(())
}
