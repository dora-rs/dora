use std::io::Write;

use super::{Executable, default_tracing};
use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, query_running_dataflows},
    formatting::OutputFormat,
};
use clap::Args;
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::coordinator_to_cli::DataflowStatus;
use eyre::eyre;
use serde::Serialize;
use tabwriter::TabWriter;
use uuid::Uuid;

#[derive(Debug, Args)]
/// List running dataflows.
pub struct ListArgs {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: std::net::IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
    /// Output format
    #[clap(long, value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,
}

impl Executable for ListArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .map_err(|_| eyre!("Failed to connect to coordinator"))?;

        list(&mut *session, self.format)
    }
}

#[derive(Serialize)]
struct OutputEntry {
    uuid: Uuid,
    name: String,
    status: DataflowStatus,
}

fn list(
    session: &mut TcpRequestReplyConnection,
    format: OutputFormat,
) -> Result<(), eyre::ErrReport> {
    let list = query_running_dataflows(session)?;

    let entries = list.0.into_iter().map(|entry| OutputEntry {
        uuid: entry.id.uuid,
        name: entry.id.name.unwrap_or_default(),
        status: entry.status,
    });

    match format {
        OutputFormat::Table => {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            tw.write_all(b"UUID\tName\tStatus\n")?;
            for entry in entries {
                let status = match entry.status {
                    DataflowStatus::Running => "Running",
                    DataflowStatus::Finished => "Succeeded",
                    DataflowStatus::Failed => "Failed",
                };
                tw.write_all(format!("{}\t{}\t{status}\n", entry.uuid, entry.name).as_bytes())?;
            }
            tw.flush()?;
        }
        OutputFormat::Json => {
            for entry in entries {
                println!("{}", serde_json::to_string(&entry)?);
            }
        }
    }

    Ok(())
}
