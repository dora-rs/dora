use std::io::Write;

use super::{Executable, default_tracing};
use crate::{
    common::{CoordinatorOptions, expect_reply, send_control_request},
    formatting::OutputFormat,
};
use clap::Args;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::DataflowStatus};
use serde::Serialize;
use tabwriter::TabWriter;
use uuid::Uuid;

/// Remove finished and failed dataflows from the coordinator.
///
/// Running dataflows are unaffected. Cached build results
/// (`finished_builds`) are intentionally NOT cleared — that would break
/// concurrent `dora build` calls with "unknown build id" errors.
///
/// **What's lost after cleaning:** the logs of cleaned dataflows
/// (`dora logs <uuid>` will fail) and their archived descriptors.
/// Anything you might want to reference later should be saved or copied
/// first.
#[derive(Debug, Args)]
pub struct CleanArgs {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    /// Output format
    #[clap(long, short = 'f', value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,
    /// Only print cleaned dataflow UUIDs, one per line
    #[clap(long, short = 'q', conflicts_with = "format")]
    pub quiet: bool,
}

impl Executable for CleanArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;
        clean(&session, self.format, self.quiet)
    }
}

#[derive(Serialize)]
struct CleanedEntry {
    uuid: Uuid,
    name: String,
    status: DataflowStatus,
}

fn clean(
    session: &crate::ws_client::WsSession,
    format: OutputFormat,
    quiet: bool,
) -> eyre::Result<()> {
    let reply = send_control_request(session, &ControlRequest::Clean)?;
    let list = expect_reply!(reply, DataflowList(list))?;

    let entries: Vec<CleanedEntry> = list
        .0
        .into_iter()
        .map(|entry| CleanedEntry {
            uuid: entry.id.uuid,
            name: entry.id.name.unwrap_or_default(),
            status: entry.status,
        })
        .collect();

    if entries.is_empty() {
        if !quiet {
            // Informational; goes to stderr so `dora clean | …` pipelines see
            // an empty stdout consistently with the non-empty case below.
            eprintln!("No finished or failed dataflows to clean.");
        }
        return Ok(());
    }

    if quiet {
        for entry in &entries {
            println!("{}", entry.uuid);
        }
        return Ok(());
    }

    match format {
        OutputFormat::Table => {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            tw.write_all("UUID\tName\tStatus\n".as_bytes())?;
            for entry in &entries {
                let status = match entry.status {
                    DataflowStatus::Running => "Running",
                    DataflowStatus::Finished => "Finished",
                    DataflowStatus::Failed => "Failed",
                };
                tw.write_all(format!("{}\t{}\t{}\n", entry.uuid, entry.name, status).as_bytes())?;
            }
            tw.flush()?;
            eprintln!("\nCleaned {} dataflow(s).", entries.len());
        }
        OutputFormat::Json => {
            for entry in &entries {
                println!("{}", serde_json::to_string(entry)?);
            }
        }
    }

    Ok(())
}
