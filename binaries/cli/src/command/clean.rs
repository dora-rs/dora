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
///
/// If the persisted-store delete fails for one or more dataflows the
/// command prints the failures to stderr and exits non-zero so scripted
/// callers don't mistake a partial outage for a clean run.
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

#[derive(Serialize)]
struct FailedEntry {
    uuid: Uuid,
    name: String,
    error: String,
}

fn clean(
    session: &crate::ws_client::WsSession,
    format: OutputFormat,
    quiet: bool,
) -> eyre::Result<()> {
    let reply = send_control_request(session, &ControlRequest::Clean)?;
    let (cleaned, failed) = expect_reply!(reply, CleanResult { cleaned, failed })?;

    let entries: Vec<CleanedEntry> = cleaned
        .0
        .into_iter()
        .map(|entry| CleanedEntry {
            uuid: entry.id.uuid,
            name: entry.id.name.unwrap_or_default(),
            status: entry.status,
        })
        .collect();

    let failures: Vec<FailedEntry> = failed
        .into_iter()
        .map(|entry| FailedEntry {
            uuid: entry.id.uuid,
            name: entry.id.name.unwrap_or_default(),
            error: entry.error,
        })
        .collect();

    if entries.is_empty() && failures.is_empty() {
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
    } else {
        match format {
            OutputFormat::Table => {
                if !entries.is_empty() {
                    let mut tw = TabWriter::new(std::io::stdout().lock());
                    tw.write_all("UUID\tName\tStatus\n".as_bytes())?;
                    for entry in &entries {
                        let status = match entry.status {
                            DataflowStatus::Running => "Running",
                            DataflowStatus::Finished => "Finished",
                            DataflowStatus::Failed => "Failed",
                        };
                        tw.write_all(
                            format!("{}\t{}\t{}\n", entry.uuid, entry.name, status).as_bytes(),
                        )?;
                    }
                    tw.flush()?;
                }
                if entries.is_empty() {
                    eprintln!("0 cleaned; {} failed (see warnings below).", failures.len());
                } else {
                    eprintln!(
                        "\nCleaned {} dataflow(s){}.",
                        entries.len(),
                        if failures.is_empty() {
                            String::new()
                        } else {
                            format!("; {} failed (see warnings below)", failures.len())
                        }
                    );
                }
            }
            OutputFormat::Json => {
                for entry in &entries {
                    println!("{}", serde_json::to_string(entry)?);
                }
            }
        }
    }

    // Failures go to stderr in every mode so stdout stays parseable.
    for failure in &failures {
        match format {
            OutputFormat::Json if !quiet => {
                eprintln!("{}", serde_json::to_string(failure)?);
            }
            _ => {
                let label = if failure.name.is_empty() {
                    failure.uuid.to_string()
                } else {
                    format!("{} ({})", failure.uuid, failure.name)
                };
                eprintln!(
                    "warning: failed to clean {label}: {err} (in-memory entry preserved, retry with `dora clean`)",
                    err = failure.error
                );
            }
        }
    }

    if failures.is_empty() {
        Ok(())
    } else {
        Err(eyre::eyre!(
            "{} dataflow(s) could not be removed from the persisted store; their in-memory entries are preserved so a later `dora clean` can retry",
            failures.len()
        ))
    }
}
