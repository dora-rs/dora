use std::io::Write;

use super::{Executable, default_tracing};
use crate::{
    common::{CoordinatorOptions, expect_reply, query_running_dataflows, send_control_request},
    formatting::OutputFormat,
    ws_client::WsSession,
};
use adora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::DataflowStatus};
use clap::Args;
use serde::Serialize;
use tabwriter::TabWriter;
use uuid::Uuid;

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum SortField {
    Cpu,
    Memory,
}

#[derive(Debug, Clone, Copy, clap::ValueEnum, Serialize)]
pub enum StatusFilter {
    Running,
    Finished,
    Failed,
}

#[derive(Debug, Args)]
/// List running dataflows.
pub struct ListArgs {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    /// Output format
    #[clap(long, short = 'f', value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,
    /// Filter by status (running, finished, failed)
    #[clap(long, value_name = "STATUS")]
    pub status: Option<StatusFilter>,
    /// Filter by dataflow name
    #[clap(long, value_name = "PATTERN")]
    pub name: Option<String>,
    /// Sort by field (memory, cpu)
    #[clap(long, value_name = "FIELD")]
    pub sort_by: Option<SortField>,
    /// Only print dataflow UUIDs, one per line
    #[clap(long, short = 'q', conflicts_with = "format")]
    pub quiet: bool,
}

impl Executable for ListArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;

        list(
            &session,
            self.format,
            self.status,
            self.name,
            self.sort_by,
            self.quiet,
        )
    }
}

#[derive(Serialize)]
struct OutputEntry {
    uuid: Uuid,
    name: String,
    status: DataflowStatus,
    nodes: usize,
    cpu: f64,
    memory: f64,
}

#[derive(Default)]
struct DataflowMetrics {
    node_count: usize,
    total_cpu: f64,
    total_memory_mb: f64,
}

fn list(
    session: &WsSession,
    format: OutputFormat,
    status_filter: Option<StatusFilter>,
    name_filter: Option<String>,
    sort_by: Option<SortField>,
    quiet: bool,
) -> Result<(), eyre::ErrReport> {
    let list = query_running_dataflows(session)?;

    // Get node information
    let reply = send_control_request(session, &ControlRequest::GetNodeInfo)?;
    let node_infos = expect_reply!(reply, NodeInfoList(infos))?;

    // Aggregate metrics by dataflow UUID
    let mut dataflow_metrics: std::collections::BTreeMap<Uuid, DataflowMetrics> =
        std::collections::BTreeMap::new();

    for node_info in node_infos {
        let metrics = dataflow_metrics.entry(node_info.dataflow_id).or_default();
        metrics.node_count += 1;

        if let Some(node_metrics) = node_info.metrics {
            metrics.total_cpu += node_metrics.cpu_usage as f64;
            metrics.total_memory_mb += node_metrics.memory_mb;
        }
    }

    let mut entries: Vec<OutputEntry> = list
        .0
        .into_iter()
        .map(|entry| {
            let uuid = entry.id.uuid;
            let metrics = dataflow_metrics.get(&uuid);

            let (nodes, cpu, memory) = if let Some(m) = metrics {
                (
                    m.node_count,
                    if m.total_cpu >= 0.1 { m.total_cpu } else { 0.0 },
                    if m.total_memory_mb >= 0.1 {
                        m.total_memory_mb / 1024.0
                    } else {
                        0.0
                    },
                )
            } else {
                (0, 0.0, 0.0)
            };

            OutputEntry {
                uuid,
                name: entry.id.name.unwrap_or_default(),
                status: entry.status,
                nodes,
                cpu,
                memory,
            }
        })
        .collect();

    // Apply status filter
    if let Some(status_filter) = status_filter {
        entries.retain(|entry| {
            matches!(
                (status_filter, &entry.status),
                (StatusFilter::Running, DataflowStatus::Running)
                    | (StatusFilter::Finished, DataflowStatus::Finished)
                    | (StatusFilter::Failed, DataflowStatus::Failed)
            )
        });
    }

    // Apply name filter (case-insensitive substring matching)
    if let Some(ref name_pattern) = name_filter {
        let pattern_lower = name_pattern.to_lowercase();
        entries.retain(|entry| entry.name.to_lowercase().contains(&pattern_lower));
    }

    // Apply sorting
    if let Some(ref sort_field) = sort_by {
        match sort_field {
            SortField::Cpu => {
                entries.sort_by(|a, b| {
                    b.cpu
                        .partial_cmp(&a.cpu)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            }
            SortField::Memory => {
                entries.sort_by(|a, b| {
                    b.memory
                        .partial_cmp(&a.memory)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            }
        }
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
            // Header
            tw.write_all(
                "UUID\tName\tStatus\tNodes\tCPU\tMemory\n"
                    .to_string()
                    .as_bytes(),
            )?;
            for entry in entries {
                let status = match entry.status {
                    DataflowStatus::Running => "Running",
                    DataflowStatus::Finished => "Finished",
                    DataflowStatus::Failed => "Failed",
                };

                let memory_str = format_memory_gb(entry.memory);
                tw.write_all(
                    format!(
                        "{}\t{}\t{}\t{}\t{:.1}%\t{}\n",
                        entry.uuid, entry.name, status, entry.nodes, entry.cpu, memory_str
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

/// Format a memory value (in GB) with auto-selected unit.
fn format_memory_gb(gb: f64) -> String {
    let mb = gb * 1024.0;
    if mb < 1.0 {
        format!("{:.1} MB", mb)
    } else if gb < 1.0 {
        format!("{:.0} MB", mb)
    } else {
        format!("{:.1} GB", gb)
    }
}
