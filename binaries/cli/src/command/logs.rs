use super::{Executable, default_tracing};
use crate::common::{connect_to_coordinator, query_running_dataflows};
use bat::{Input, PrettyPrinter};
use clap::Args;
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{Context, Result, bail};
use uuid::Uuid;

#[derive(Debug, Args)]
/// Show logs of a given dataflow and node.
pub struct LogsArgs {
    /// Identifier of the dataflow
    #[clap(value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Show logs for the given node
    #[clap(value_name = "NAME")]
    pub node: String,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: std::net::IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
}

impl Executable for LogsArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .wrap_err("failed to connect to dora coordinator")?;
        let list =
            query_running_dataflows(&mut *session).wrap_err("failed to query running dataflows")?;
        if let Some(dataflow) = self.dataflow {
            let uuid = Uuid::parse_str(&dataflow).ok();
            let name = if uuid.is_some() { None } else { Some(dataflow) };
            logs(&mut *session, uuid, name, self.node)
        } else {
            let active = list.get_active();
            let uuid = match &active[..] {
                [] => bail!("No dataflows are running"),
                [uuid] => uuid.clone(),
                _ => inquire::Select::new("Choose dataflow to show logs:", active).prompt()?,
            };
            logs(&mut *session, Some(uuid.uuid), None, self.node)
        }
    }
}

pub fn logs(
    session: &mut TcpRequestReplyConnection,
    uuid: Option<Uuid>,
    name: Option<String>,
    node: String,
) -> Result<()> {
    let logs = {
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Logs {
                    uuid,
                    name,
                    node: node.clone(),
                })
                .wrap_err("")?,
            )
            .wrap_err("failed to send Logs request message")?;

        let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match reply {
            ControlRequestReply::Logs(logs) => logs,
            other => bail!("unexpected reply to daemon logs: {other:?}"),
        }
    };

    PrettyPrinter::new()
        .header(false)
        .grid(false)
        .line_numbers(false)
        .paging_mode(bat::PagingMode::QuitIfOneScreen)
        .inputs(vec![
            Input::from_bytes(&logs)
                .name("Logs")
                .title(format!("Logs from {node}.").as_str()),
        ])
        .print()
        .wrap_err("Something went wrong with viewing log file")?;

    Ok(())
}
