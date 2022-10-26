use crate::check::coordinator_running;
use eyre::Context;
use std::{path::Path, process::Command};

pub(crate) fn up(roudi: Option<&Path>, coordinator: Option<&Path>) -> eyre::Result<()> {
    if !coordinator_running()? {
        start_coordinator(coordinator).wrap_err("failed to start dora-coordinator")?;
    }

    // try to start roudi
    start_roudi(roudi).wrap_err("failed to start iceoryx roudi daemon")?;

    Ok(())
}

fn start_coordinator(coordinator: Option<&Path>) -> eyre::Result<()> {
    let coordinator = coordinator.unwrap_or_else(|| Path::new("dora-coordinator"));

    let mut cmd = Command::new(coordinator);
    cmd.spawn()
        .wrap_err_with(|| format!("failed to run {}", coordinator.display()))?;

    Ok(())
}

fn start_roudi(roudi: Option<&Path>) -> eyre::Result<()> {
    let roudi = roudi.unwrap_or_else(|| Path::new("iox-roudi"));

    let mut cmd = Command::new(roudi);
    cmd.spawn()
        .wrap_err_with(|| format!("failed to run {}", roudi.display()))?;

    Ok(())
}
