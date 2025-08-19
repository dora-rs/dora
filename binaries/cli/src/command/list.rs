use std::io::Write;

use super::{Executable, default_tracing};
use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, query_running_dataflows},
};
use clap::Args;
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::coordinator_to_cli::DataflowStatus;
use eyre::eyre;
use tabwriter::TabWriter;

#[derive(Debug, Args)]
/// List running dataflows.
pub struct ListArgs {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: std::net::IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
}

impl Executable for ListArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .map_err(|_| eyre!("Failed to connect to coordinator"))?;

        list(&mut *session)
    }
}

fn list(session: &mut TcpRequestReplyConnection) -> Result<(), eyre::ErrReport> {
    let list = query_running_dataflows(session)?;

    let mut tw = TabWriter::new(vec![]);
    tw.write_all(b"UUID\tName\tStatus\n")?;
    for entry in list.0 {
        let uuid = entry.id.uuid;
        let name = entry.id.name.unwrap_or_default();
        let status = match entry.status {
            DataflowStatus::Running => "Running",
            DataflowStatus::Finished => "Succeeded",
            DataflowStatus::Failed => "Failed",
        };
        tw.write_all(format!("{uuid}\t{name}\t{status}\n").as_bytes())?;
    }
    tw.flush()?;
    let formatted = String::from_utf8(tw.into_inner()?)?;

    println!("{formatted}");

    Ok(())
}
