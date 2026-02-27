use super::{
    Executable, default_tracing,
    list::{OutputEntry, display_entries},
};
use crate::{
    LOCALHOST,
    common::{connect_and_check_version, rpc},
    formatting::OutputFormat,
};
use clap::Args;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{cli_to_coordinator::CliControlClient, tarpc};
use eyre::Context;

#[derive(Debug, Args)]
/// Remove finished and failed dataflows from the list.
pub struct CleanArgs {
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

impl Executable for CleanArgs {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let client = connect_and_check_version(self.coordinator_addr, self.coordinator_port)
            .await
            .wrap_err("failed to connect to dora coordinator")?;

        clean(&client, self.format).await
    }
}

async fn clean(client: &CliControlClient, format: OutputFormat) -> eyre::Result<()> {
    let list = rpc(
        "clean finished dataflows",
        client.clean(tarpc::context::current()),
    )
    .await?;

    let entries: Vec<OutputEntry> = list
        .0
        .into_iter()
        .map(|entry| OutputEntry {
            uuid: entry.id.uuid,
            name: entry.id.name.unwrap_or_default(),
            status: entry.status,
            nodes: 0,
            cpu: 0.0,
            memory: 0.0,
        })
        .collect();

    if entries.is_empty() {
        println!("No finished or failed dataflows to clean.");
        return Ok(());
    }

    display_entries(&entries, format, false)
}
