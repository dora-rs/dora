use std::io::Write;

use super::{Executable, default_tracing};
use crate::{
    common::{connect_to_coordinator_rpc, query_running_dataflows, rpc},
    formatting::OutputFormat,
};
use clap::Args;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{
    cli_to_coordinator::CliControlClient, coordinator_to_cli::DataflowStatus, tarpc,
};
use eyre::Context;
use serde::Serialize;
use tabwriter::TabWriter;
use uuid::Uuid;

#[derive(Debug, Args)]
/// List running dataflows.
pub struct ListArgs {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP")]
    pub coordinator_addr: Option<std::net::IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT")]
    pub coordinator_port: Option<u16>,
    /// Output format
    #[clap(long, value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,
    /// Filter by status (running, finished, failed)
    #[clap(long, value_name = "STATUS")]
    pub status: Option<String>,
    /// Filter by dataflow name
    #[clap(long, value_name = "PATTERN")]
    pub name: Option<String>,
    /// Sort by field (memory, cpu)
    #[clap(long, value_name = "FIELD")]
    pub sort_by: Option<String>,
}

impl Executable for ListArgs {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        // Resolve coordinator address and port from CLI args, config, or defaults
        use crate::common::resolve_coordinator_addr;
        let (addr, port) = resolve_coordinator_addr(
            self.coordinator_addr,
            self.coordinator_port,
            DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        );

        let client = connect_to_coordinator_rpc(addr, port)
            .await
            .wrap_err("failed to connect to dora coordinator")?;

        list(&client, self.format, self.status, self.name, self.sort_by).await
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

async fn list(
    client: &CliControlClient,
    format: OutputFormat,
    status_filter: Option<String>,
    name_filter: Option<String>,
    sort_by: Option<String>,
) -> Result<(), eyre::ErrReport> {
    let list = query_running_dataflows(client).await?;

    // Get node information via tarpc
    let node_infos = rpc(
        "get node info",
        client.get_node_info(tarpc::context::current()),
    )
    .await?;

    // Aggregate metrics by dataflow UUID
    let mut dataflow_metrics: std::collections::BTreeMap<Uuid, DataflowMetrics> =
        std::collections::BTreeMap::new();

    for node_info in node_infos {
        let metrics = dataflow_metrics
            .entry(node_info.dataflow_id)
            .or_insert_with(DataflowMetrics::default);
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
    if let Some(ref status_str) = status_filter {
        let status_lower = status_str.to_lowercase();
        entries.retain(|entry| {
            let entry_status = match entry.status {
                DataflowStatus::Running => "running",
                DataflowStatus::Finished => "finished",
                DataflowStatus::Failed => "failed",
            };
            entry_status.starts_with(&status_lower)
        });
    }

    // Apply name filter (case-insensitive substring matching)
    if let Some(ref name_pattern) = name_filter {
        let pattern_lower = name_pattern.to_lowercase();
        entries.retain(|entry| entry.name.to_lowercase().contains(&pattern_lower));
    }

    // Apply sorting
    if let Some(ref sort_field) = sort_by {
        let sort_lower = sort_field.to_lowercase();
        match sort_lower.as_str() {
            "cpu" => {
                entries.sort_by(|a, b| {
                    b.cpu
                        .partial_cmp(&a.cpu)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            }
            "memory" => {
                entries.sort_by(|a, b| {
                    b.memory
                        .partial_cmp(&a.memory)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            }
            _ => {
                eprintln!(
                    "Unknown sort field: {}. Valid options: cpu, memory",
                    sort_field
                );
            }
        }
    }

    match format {
        OutputFormat::Table => {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            // Header
            tw.write_all(format!("UUID\tName\tStatus\tNodes\tCPU\tMemory\n").as_bytes())?;
            for entry in entries {
                let status = match entry.status {
                    DataflowStatus::Running => "Running",
                    DataflowStatus::Finished => "Succeeded",
                    DataflowStatus::Failed => "Failed",
                };

                tw.write_all(
                    format!(
                        "{}\t{}\t{}\t{}\t{}\t{}\n",
                        entry.uuid,
                        entry.name,
                        status,
                        entry.nodes,
                        format!("{:.1}%", entry.cpu),
                        format!("{:.1} GB", entry.memory)
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
