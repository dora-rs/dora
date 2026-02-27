use super::{Executable, default_tracing};
use crate::common::{connect_to_coordinator, handle_dataflow_result, query_running_dataflows};
use crate::ws_client::WsSession;
use adora_core::topics::{ADORA_COORDINATOR_PORT_WS_DEFAULT, LOCALHOST};
use adora_message::cli_to_coordinator::ControlRequest;
use adora_message::coordinator_to_cli::ControlRequestReply;
use duration_str::parse;
use eyre::{Context, bail};
use std::io::IsTerminal;
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
    #[clap(long, short = 'n')]
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
    /// Address of the adora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST, env = "ADORA_COORDINATOR_ADDR")]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = ADORA_COORDINATOR_PORT_WS_DEFAULT, env = "ADORA_COORDINATOR_PORT")]
    coordinator_port: u16,
}

impl Executable for Stop {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
            .wrap_err("could not connect to adora coordinator")?;
        match (self.uuid, self.name) {
            (Some(uuid), _) => stop_dataflow(uuid, self.grace_duration, self.force, &session),
            (None, Some(name)) => {
                stop_dataflow_by_name(name, self.grace_duration, self.force, &session)
            }
            (None, None) => stop_dataflow_interactive(self.grace_duration, self.force, &session),
        }
    }
}

fn stop_dataflow_interactive(
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> eyre::Result<()> {
    let list = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    let active = list.get_active();
    if active.is_empty() {
        eprintln!("No dataflows are running");
    } else if active.len() > 1 && !std::io::stdin().is_terminal() {
        bail!(
            "Multiple dataflows running. Specify one:\n  adora stop <UUID>\n  adora stop --name <NAME>"
        );
    } else {
        let selection = inquire::Select::new("Choose dataflow to stop:", active).prompt()?;
        stop_dataflow(selection.uuid, grace_duration, force, session)?;
    }

    Ok(())
}

fn stop_dataflow(
    uuid: Uuid,
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::Stop {
                dataflow_uuid: uuid,
                grace_duration,
                force,
            })
            .unwrap(),
        )
        .wrap_err("failed to send dataflow stop message")?;
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowStopped { uuid, result } => {
            handle_dataflow_result(result, Some(uuid))
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
    }
}

fn stop_dataflow_by_name(
    name: String,
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::StopByName {
                name,
                grace_duration,
                force,
            })
            .unwrap(),
        )
        .wrap_err("failed to send dataflow stop_by_name message")?;
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowStopped { uuid, result } => {
            handle_dataflow_result(result, Some(uuid))
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
    }
}
