use super::{Executable, default_tracing};
use crate::common::{connect_to_coordinator, handle_dataflow_result, query_running_dataflows};
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::cli_to_coordinator::{CliToCoordinatorClient, DataflowStopped};
use duration_str::parse;
use eyre::Context;
use std::net::IpAddr;
use std::time::Duration;
use uuid::Uuid;

#[derive(Debug, clap::Args)]
/// Stop a running dataflow. If no id or name is provided, you will be able to choose between the running dataflows.
///
/// You could specify the strategy to stop the dataflow with `--grace-duration` or `--force`.
pub struct Stop {
    /// UUID of the dataflow that should be stopped
    uuid: Option<Uuid>,
    /// Name of the dataflow that should be stopped
    #[clap(long)]
    name: Option<String>,
    /// Kill the dataflow if it doesn't stop after the given duration
    ///
    /// Specifically, it does the following:
    /// 1. Sends `Event::Stop` to all nodes of the dataflow.
    /// 2. After DURATION, performs a soft kill (sending SIGTERM, or Ctrl-Break on Windows).
    /// 3. If the dataflow is still running after DURATION * 0.5, terminates all its processes.
    #[clap(
        long,
        value_name = "DURATION",
        group = "strategy",
        verbatim_doc_comment
    )]
    #[arg(value_parser = parse)]
    grace_duration: Option<Duration>,
    /// Force stop the dataflow by immediately terminating all its processes
    #[clap(short, long, action, group = "strategy")]
    force: bool,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
}

impl Executable for Stop {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .wrap_err("could not connect to dora coordinator")?;
        let stopped = match (self.uuid, self.name) {
            (Some(uuid), _) => session.stop(uuid, self.grace_duration, self.force)?,
            (None, Some(name)) => session.stop_by_name(name, self.grace_duration, self.force)?,
            (None, None) => {
                let Some(result) =
                    stop_dataflow_interactive(self.grace_duration, self.force, &mut session)?
                else {
                    return Ok(());
                };
                result
            }
        };
        handle_dataflow_result(stopped.result, Some(stopped.uuid))
    }
}

fn stop_dataflow_interactive(
    grace_duration: Option<Duration>,
    force: bool,
    session: &mut CliToCoordinatorClient,
) -> eyre::Result<Option<DataflowStopped>> {
    let list = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    let active = list.get_active();
    Ok(if active.is_empty() {
        eprintln!("No dataflows are running");
        None
    } else {
        let selection = inquire::Select::new("Choose dataflow to stop:", active).prompt()?;
        Some(session.stop(selection.uuid, grace_duration, force)?)
    })
}
