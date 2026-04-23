use super::{Executable, default_tracing};
use crate::common::{
    CoordinatorOptions, expect_reply, handle_dataflow_result, query_running_dataflows,
    send_control_request,
};
use crate::ws_client::WsSession;
use dora_message::cli_to_coordinator::ControlRequest;
use duration_str::parse;
use eyre::{Context, bail};
use std::io::IsTerminal;
use std::time::Duration;
use uuid::Uuid;

#[derive(Debug, clap::Args)]
/// Stop a running dataflow. If no id or name is provided, you will be able to choose between the running dataflows.
///
/// You could specify the strategy to stop the dataflow with `--grace-duration` or `--force`.
pub struct Stop {
    /// Name or UUID of the dataflow that should be stopped
    #[clap(value_name = "NAME_OR_UUID", conflicts_with = "all")]
    identifier: Option<String>,
    /// Name of the dataflow (alternative to positional; kept for back-compat)
    #[clap(long, short = 'n', conflicts_with_all = ["identifier", "all"])]
    name: Option<String>,
    /// Stop every running dataflow.
    #[clap(long, conflicts_with_all = ["identifier", "name"])]
    all: bool,
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
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Stop {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;

        if self.all {
            return stop_all(self.grace_duration, self.force, &session);
        }

        let ident = self.identifier.or(self.name);
        match ident {
            // Identifier parses as UUID -> dispatch by UUID. Otherwise treat
            // as a dataflow name. Users who want strict name-only lookup can
            // still use `-n/--name` (which takes the explicit-name path).
            Some(s) => match Uuid::parse_str(&s) {
                Ok(uuid) => stop_dataflow(uuid, self.grace_duration, self.force, &session),
                Err(_) => stop_dataflow_by_name(s, self.grace_duration, self.force, &session),
            },
            None => stop_dataflow_interactive(self.grace_duration, self.force, &session),
        }
    }
}

fn stop_all(
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> eyre::Result<()> {
    let list = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    let active = list.get_active();
    if active.is_empty() {
        println!("No dataflows are running.");
        return Ok(());
    }
    let total = active.len();
    let mut errors = Vec::new();
    for entry in active {
        if let Err(e) = stop_dataflow(entry.uuid, grace_duration, force, session) {
            errors.push(format!("{}: {e}", entry.uuid));
        }
    }
    if !errors.is_empty() {
        bail!(
            "{} of {total} dataflow(s) failed to stop:\n  {}",
            errors.len(),
            errors.join("\n  ")
        );
    }
    println!("Stopped {total} dataflow(s).");
    Ok(())
}

fn stop_dataflow_interactive(
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> eyre::Result<()> {
    let list = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    let active = list.get_active();
    if active.is_empty() {
        println!("No dataflows are running. Use `dora list` to check dataflow status.");
    } else if active.len() == 1 {
        stop_dataflow(active[0].uuid, grace_duration, force, session)?;
    } else if !std::io::stdin().is_terminal() {
        bail!(
            "Multiple dataflows running. Specify one:\n  dora stop <UUID>\n  dora stop --name <NAME>"
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
    let reply = send_control_request(
        session,
        &ControlRequest::Stop {
            dataflow_uuid: uuid,
            grace_duration,
            force,
        },
    )?;
    let (uuid, result) = expect_reply!(reply, DataflowStopped { uuid, result })?;
    handle_dataflow_result(result, Some(uuid))
}

fn stop_dataflow_by_name(
    name: String,
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> Result<(), eyre::ErrReport> {
    let reply = send_control_request(
        session,
        &ControlRequest::StopByName {
            name,
            grace_duration,
            force,
        },
    )?;
    let (uuid, result) = expect_reply!(reply, DataflowStopped { uuid, result })?;
    handle_dataflow_result(result, Some(uuid))
}
