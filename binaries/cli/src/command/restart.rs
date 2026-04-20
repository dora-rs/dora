use super::{Executable, default_tracing};
use crate::common::{CoordinatorOptions, query_running_dataflows};
use crate::ws_client::WsSession;
use dora_message::cli_to_coordinator::ControlRequest;
use dora_message::coordinator_to_cli::ControlRequestReply;
use duration_str::parse;
use eyre::{Context, bail};
use std::io::IsTerminal;
use std::time::Duration;
use uuid::Uuid;

#[derive(Debug, clap::Args)]
/// Restart a running dataflow (stop + re-start with stored descriptor).
///
/// If no id or name is provided, you will be able to choose between the running dataflows.
pub struct Restart {
    /// Name or UUID of the dataflow that should be restarted
    #[clap(value_name = "NAME_OR_UUID")]
    identifier: Option<String>,
    /// Name of the dataflow (alternative to positional; kept for back-compat)
    #[clap(long, short = 'n', conflicts_with = "identifier")]
    name: Option<String>,
    /// Kill the dataflow if it doesn't stop after the given duration before restarting
    #[clap(
        long,
        value_name = "DURATION",
        group = "strategy",
        verbatim_doc_comment
    )]
    #[arg(value_parser = parse)]
    grace_duration: Option<Duration>,
    /// Force stop the dataflow before restarting
    #[clap(short, long, action, group = "strategy")]
    force: bool,
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Restart {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;

        let ident = self.identifier.or(self.name);
        match ident {
            Some(s) => match Uuid::parse_str(&s) {
                Ok(uuid) => restart_dataflow(uuid, self.grace_duration, self.force, &session),
                Err(_) => restart_dataflow_by_name(s, self.grace_duration, self.force, &session),
            },
            None => restart_dataflow_interactive(self.grace_duration, self.force, &session),
        }
    }
}

fn restart_dataflow_interactive(
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> eyre::Result<()> {
    let list = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    let active = list.get_active();
    if active.is_empty() {
        println!("No dataflows are running");
    } else if active.len() == 1 {
        restart_dataflow(active[0].uuid, grace_duration, force, session)?;
    } else if !std::io::stdin().is_terminal() {
        bail!(
            "Multiple dataflows running. Specify one:\n  dora restart <UUID>\n  dora restart --name <NAME>"
        );
    } else {
        let selection = inquire::Select::new("Choose dataflow to restart:", active).prompt()?;
        restart_dataflow(selection.uuid, grace_duration, force, session)?;
    }

    Ok(())
}

fn restart_dataflow(
    uuid: Uuid,
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> eyre::Result<()> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::Restart {
                dataflow_uuid: uuid,
                grace_duration,
                force,
            })
            .unwrap(),
        )
        .wrap_err("failed to send dataflow restart message")?;
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowRestarted { old_uuid, new_uuid } => {
            println!("dataflow restarted: {old_uuid} -> {new_uuid}");
            Ok(())
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected restart dataflow reply: {other:?}"),
    }
}

fn restart_dataflow_by_name(
    name: String,
    grace_duration: Option<Duration>,
    force: bool,
    session: &WsSession,
) -> eyre::Result<()> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::RestartByName {
                name,
                grace_duration,
                force,
            })
            .unwrap(),
        )
        .wrap_err("failed to send dataflow restart_by_name message")?;
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowRestarted { old_uuid, new_uuid } => {
            println!("dataflow restarted: {old_uuid} -> {new_uuid}");
            Ok(())
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected restart dataflow reply: {other:?}"),
    }
}
