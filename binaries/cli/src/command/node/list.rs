use std::io::Write;

use clap::Args;
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowStatus, NodeHealthEntry},
    id::NodeId,
};
use eyre::eyre;
use serde::Serialize;
use tabwriter::TabWriter;

use crate::{
    LOCALHOST,
    command::{Executable, default_tracing},
    common::connect_to_coordinator,
    formatting::OutputFormat,
};

/// List nodes with health status.
///
/// Examples:
///
/// List all nodes in a dataflow with health:
///   dora node list --dataflow <uuid> --health
///
/// List nodes as JSON:
///   dora node list --dataflow <uuid> --health --format json
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct List {
    /// UUID of the dataflow
    #[clap(long, short = 'd', value_name = "UUID")]
    pub dataflow: uuid::Uuid,

    /// Show health status
    #[clap(long)]
    pub health: bool,

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

impl Executable for List {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .map_err(|_| eyre!("Failed to connect to coordinator"))?;

        if self.health {
            list_with_health(&mut *session, self.dataflow, self.format)
        } else {
            // Basic list without health (just show node IDs)
            list_basic(&mut *session, self.dataflow, self.format)
        }
    }
}

#[derive(Serialize)]
struct HealthOutputEntry {
    node: String,
    status: String,
    health: String,
    uptime: String,
}

fn list_with_health(
    session: &mut TcpRequestReplyConnection,
    dataflow_uuid: uuid::Uuid,
    format: OutputFormat,
) -> Result<(), eyre::ErrReport> {
    let request = ControlRequest::NodeHealth { dataflow_uuid };
    let reply_bytes = session
        .request(&serde_json::to_vec(&request)?)
        .map_err(|e| eyre!("failed to send request: {e}"))?;
    let reply: ControlRequestReply = serde_json::from_slice(&reply_bytes)?;

    let health_list = match reply {
        ControlRequestReply::NodeHealthList(list) => list,
        ControlRequestReply::Error(e) => return Err(eyre!("coordinator error: {e}")),
        _ => return Err(eyre!("unexpected reply from coordinator")),
    };

    let entries = health_list.0.into_iter().map(|entry: NodeHealthEntry| {
        let status = match entry.status {
            DataflowStatus::Running => "Running",
            DataflowStatus::Finished => "Finished",
            DataflowStatus::Failed => "Failed",
        };
        let health = format!("{:?}", entry.health);
        let uptime = format_duration(entry.uptime);

        HealthOutputEntry {
            node: entry.node_id.to_string(),
            status: status.to_string(),
            health,
            uptime,
        }
    });

    match format {
        OutputFormat::Table => {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            tw.write_all(b"NODE\tSTATUS\tHEALTH\tUPTIME\n")?;
            for entry in entries {
                tw.write_all(
                    format!(
                        "{}\t{}\t{}\t{}\n",
                        entry.node, entry.status, entry.health, entry.uptime
                    )
                    .as_bytes(),
                )?;
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

fn list_basic(
    session: &mut TcpRequestReplyConnection,
    dataflow_uuid: uuid::Uuid,
    format: OutputFormat,
) -> Result<(), eyre::ErrReport> {
    // For basic list, we can use the Info request to get the descriptor
    let request = ControlRequest::Info { dataflow_uuid };
    let reply_bytes = session
        .request(&serde_json::to_vec(&request)?)
        .map_err(|e| eyre!("failed to send request: {e}"))?;
    let reply: ControlRequestReply = serde_json::from_slice(&reply_bytes)?;

    match reply {
        ControlRequestReply::DataflowInfo { descriptor, .. } => {
            let nodes: Vec<String> = descriptor
                .nodes
                .into_iter()
                .map(|n| n.id.to_string())
                .collect();

            match format {
                OutputFormat::Table => {
                    for node in nodes {
                        println!("{}", node);
                    }
                }
                OutputFormat::Json => {
                    println!("{}", serde_json::to_string(&nodes)?);
                }
            }
            Ok(())
        }
        ControlRequestReply::Error(e) => Err(eyre!("coordinator error: {e}")),
        _ => Err(eyre!("unexpected reply from coordinator")),
    }
}

fn format_duration(duration: std::time::Duration) -> String {
    let secs = duration.as_secs();
    let hours = secs / 3600;
    let minutes = (secs % 3600) / 60;
    let seconds = secs % 60;
    format!("{:02}:{:02}:{:02}", hours, minutes, seconds)
}
