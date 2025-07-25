use super::{default_tracing, Executable};
use crate::common::{connect_to_coordinator, resolve_dataflow_identifier};
use bat::{Input, PrettyPrinter};
use clap::Args;
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::{bail, Context, Result};
use uuid::Uuid;

#[derive(Debug, Args)]
/// Show logs of a given dataflow and node.
pub struct LogsArgs {
    /// Identifier of the dataflow
    #[clap(value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Show logs for the given node
    #[clap(value_name = "NAME")]
    pub node: NodeId,
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
        let uuid = resolve_dataflow_identifier(&mut *session, self.dataflow.as_deref())?;
        logs(&mut *session, uuid, self.node)
    }
}

pub fn logs(session: &mut TcpRequestReplyConnection, uuid: Uuid, node: NodeId) -> Result<()> {
    let logs = {
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Logs {
                    uuid,
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
        .inputs(vec![Input::from_bytes(&logs)
            .name("Logs")
            .title(format!("Logs from {node}.").as_str())])
        .print()
        .wrap_err("Something went wrong with viewing log file")?;

    Ok(())
}
