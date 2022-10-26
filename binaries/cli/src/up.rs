use crate::{check::coordinator_running, zenoh_control_session};
use dora_core::topics::ZENOH_CONTROL_DESTROY;
use eyre::{bail, eyre, Context};
use std::{path::Path, process::Command, sync::Arc};
use sysinfo::{ProcessExt, SystemExt};
use zenoh::{prelude::Receiver, sync::ZFuture};

pub(crate) fn up(roudi: Option<&Path>, coordinator: Option<&Path>) -> eyre::Result<()> {
    if !coordinator_running()? {
        start_coordinator(coordinator).wrap_err("failed to start dora-coordinator")?;
    }

    // try to start roudi
    start_roudi(roudi).wrap_err("failed to start iceoryx roudi daemon")?;

    Ok(())
}

pub(crate) fn destroy(session: &mut Option<Arc<zenoh::Session>>) -> Result<(), eyre::ErrReport> {
    if coordinator_running()? {
        // send destroy command to dora-coordinator
        let reply_receiver = zenoh_control_session(session)?
            .get(ZENOH_CONTROL_DESTROY)
            .wait()
            .map_err(|err| eyre!(err))
            .wrap_err("failed to create publisher for destroy message")?;
        reply_receiver
            .recv()
            .wrap_err("failed to receive reply from coordinator")?;
        println!("Send destroy command to dora-coordinator");
    } else {
        eprintln!("The dora-coordinator is not running");
    }

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

    Ok(())
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
