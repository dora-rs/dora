use crate::command::{Executable, default_tracing};
use crate::common::{connect_to_coordinator, query_running_dataflows};
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{Context, bail};
use std::net::{IpAddr, SocketAddr};
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Stop the system: stop running dataflows, then shut down coordinator and daemon.
pub struct Stop {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
    /// Force stop without confirmation prompt
    #[clap(long, short)]
    force: bool,
}

impl Executable for Stop {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        stop_system(
            (self.coordinator_addr, self.coordinator_port).into(),
            self.force,
        )
    }
}

pub(crate) fn stop_system(coordinator_addr: SocketAddr, force: bool) -> eyre::Result<()> {
    let mut session = match connect_to_coordinator(coordinator_addr) {
        Ok(s) => s,
        Err(_) => bail!("Could not connect to dora-coordinator"),
    };

    let running =
        query_running_dataflows(&mut *session).wrap_err("failed to query running dataflows")?;
    let active = running.get_active();

    if !active.is_empty() {
        if !force {
            let confirm = inquire::Confirm::new(&format!(
                "{} dataflow(s) are running. Stop them?",
                active.len()
            ))
            .with_default(false)
            .prompt()?;

            if !confirm {
                return Ok(());
            }
        }

        for dataflow in &active {
            let reply_raw = session
                .request(
                    &serde_json::to_vec(&ControlRequest::Stop {
                        dataflow_uuid: dataflow.uuid,
                        grace_duration: None,
                        force: false,
                    })
                    .unwrap(),
                )
                .wrap_err("failed to send dataflow stop message")?;
            let result: ControlRequestReply =
                serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
            if let ControlRequestReply::Error(err) = result {
                eprintln!("  ⚠ Failed to stop dataflow {}: {}", dataflow.uuid, err);
            }
        }
        println!("✓ All dataflows stopped");
    }

    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::Destroy).unwrap())
        .wrap_err("failed to send destroy message")?;
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DestroyOk => {
            println!("✓ Coordinator and daemons destroyed successfully");
        }
        ControlRequestReply::Error(err) => {
            bail!("Destroy command failed with error: {}", err);
        }
        _ => {
            bail!("Unexpected reply from dora-coordinator");
        }
    }

    Ok(())
}
