use clap::Args;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, NodeInfo},
    id::NodeId,
};
use eyre::{Context, bail};

use crate::{
    command::{Executable, default_tracing},
    common::CoordinatorOptions,
    formatting::OutputFormat,
};

/// Display detailed information about a specified node.
///
/// Examples:
///
/// Show info for a node:
///   dora node info detector_node
///
/// Show info for a node in a specific dataflow:
///   dora node info detector_node --dataflow my-dataflow
///
/// Show info as JSON:
///   dora node info detector_node --format json
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Info {
    /// ID of the node to inspect
    #[clap(value_name = "NODE_ID")]
    node_id: NodeId,

    /// Dataflow UUID or name to filter nodes (required if multiple dataflows have nodes with the same ID)
    #[clap(short, long, value_name = "UUID_OR_NAME")]
    dataflow: Option<String>,

    /// Output format
    #[clap(long, value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Info {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        info(self.coordinator, self.node_id, self.dataflow, self.format)
    }
}

fn info(
    coordinator: CoordinatorOptions,
    node_id: NodeId,
    dataflow_filter: Option<String>,
    format: OutputFormat,
) -> eyre::Result<()> {
    let mut session = coordinator.connect()?;

    // Query all running nodes from coordinator
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::GetNodeInfo).unwrap())
        .wrap_err("failed to send GetNodeInfo request")?;

    let node_list: Vec<NodeInfo> =
        match serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")? {
            ControlRequestReply::NodeInfoList(list) => list,
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected reply to GetNodeInfo: {other:?}"),
        };

    // Filter nodes by ID
    let matching_nodes: Vec<&NodeInfo> = node_list
        .iter()
        .filter(|node| node.node_id == node_id)
        .collect();

    if matching_nodes.is_empty() {
        bail!("No running node with ID `{node_id}` found");
    }

    // Further filter by dataflow if specified
    let selected_node = if let Some(dataflow_id_or_name) = dataflow_filter {
        // Try to parse as UUID first
        let matching_by_dataflow: Vec<&NodeInfo> = if let Ok(uuid) =
            uuid::Uuid::parse_str(&dataflow_id_or_name)
        {
            matching_nodes
                .iter()
                .filter(|node| node.dataflow_id == uuid)
                .copied()
                .collect()
        } else {
            // Filter by dataflow name
            matching_nodes
                .iter()
                .filter(|node| node.dataflow_name.as_deref() == Some(dataflow_id_or_name.as_str()))
                .copied()
                .collect()
        };

        if matching_by_dataflow.is_empty() {
            bail!("No node `{node_id}` found in dataflow `{dataflow_id_or_name}`");
        } else if matching_by_dataflow.len() == 1 {
            matching_by_dataflow[0]
        } else {
            bail!(
                "Multiple nodes with ID `{node_id}` found in dataflow `{dataflow_id_or_name}`. This should not happen."
            );
        }
    } else {
        // No dataflow filter specified
        if matching_nodes.len() == 1 {
            matching_nodes[0]
        } else {
            // Multiple nodes with same ID in different dataflows - prompt user
            let choices: Vec<String> = matching_nodes
                .iter()
                .map(|node| {
                    let dataflow_name = node
                        .dataflow_name
                        .as_ref()
                        .map(|n| n.as_str())
                        .unwrap_or("<unnamed>");
                    format!("[{}] {}", dataflow_name, node.dataflow_id)
                })
                .collect();

            let selection = inquire::Select::new(
                "Multiple dataflows have a node with this ID. Choose dataflow:",
                choices.clone(),
            )
            .prompt()?;

            // Find the matching node by comparing the formatted string
            matching_nodes
                .iter()
                .find(|node| {
                    let dataflow_name = node
                        .dataflow_name
                        .as_ref()
                        .map(|n| n.as_str())
                        .unwrap_or("<unnamed>");
                    let formatted = format!("[{}] {}", dataflow_name, node.dataflow_id);
                    formatted == selection
                })
                .expect("selected node not found")
        }
    };

    // Now we need to get the full descriptor to show inputs/outputs
    // Query dataflow info to get the descriptor
    let dataflow_info_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::Info {
                dataflow_uuid: selected_node.dataflow_id,
            })
            .unwrap(),
        )
        .wrap_err("failed to send Info request")?;

    let descriptor = match serde_json::from_slice(&dataflow_info_raw)
        .wrap_err("failed to parse dataflow info reply")?
    {
        ControlRequestReply::DataflowInfo { descriptor, .. } => descriptor,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected reply to Info: {other:?}"),
    };

    // Find the node in the descriptor to get inputs/outputs
    let node_descriptor = descriptor
        .nodes
        .iter()
        .find(|n| n.id == node_id)
        .ok_or_else(|| eyre::eyre!("Node `{node_id}` not found in dataflow descriptor"))?;

    // Display the information
    match format {
        OutputFormat::Table => {
            println!("ID: {}", selected_node.node_id);

            if let Some(metrics) = &selected_node.metrics {
                println!("PID: {}", metrics.pid);
                if let Some(start_time) = metrics.start_time {
                    let uptime = std::time::SystemTime::now()
                        .duration_since(start_time.get_time().to_system_time())
                        .unwrap_or_default();
                    let secs = uptime.as_secs();
                    let hours = secs / 3600;
                    let minutes = (secs % 3600) / 60;
                    let seconds = secs % 60;
                    if hours > 0 {
                        println!("Uptime: {}h {}m {}s", hours, minutes, seconds);
                    } else if minutes > 0 {
                        println!("Uptime: {}m {}s", minutes, seconds);
                    } else {
                        println!("Uptime: {}s", seconds);
                    }
                }
            }

            println!(
                "Dataflow: {} ({})",
                selected_node
                    .dataflow_name
                    .as_deref()
                    .unwrap_or("<unnamed>"),
                selected_node.dataflow_id
            );
            println!("Daemon: {}", selected_node.daemon_id);

            // Display inputs
            if node_descriptor.inputs.is_empty() {
                println!("Inputs: (none)");
            } else {
                println!("Inputs:");
                for (input_id, input) in &node_descriptor.inputs {
                    println!("  - {} (from {})", input_id, input.mapping);
                }
            }

            // Display outputs
            if node_descriptor.outputs.is_empty() {
                println!("Outputs: (none)");
            } else {
                println!("Outputs:");
                for output_id in &node_descriptor.outputs {
                    println!("  - {}", output_id);
                }
            }

            // Display metrics if available
            if let Some(metrics) = &selected_node.metrics {
                println!("\nMetrics:");
                println!("  CPU Usage: {:.1}%", metrics.cpu_usage);
                println!("  Memory: {:.1} MB", metrics.memory_mb);
                if let Some(disk_read) = metrics.disk_read_mb_s {
                    println!("  Disk Read: {:.2} MB/s", disk_read);
                }
                if let Some(disk_write) = metrics.disk_write_mb_s {
                    println!("  Disk Write: {:.2} MB/s", disk_write);
                }
            }
        }
        OutputFormat::Json => {
            // Create a JSON representation
            let mut json_output = serde_json::Map::new();
            json_output.insert("id".to_string(), serde_json::json!(selected_node.node_id));
            json_output.insert(
                "dataflow_id".to_string(),
                serde_json::json!(selected_node.dataflow_id),
            );
            json_output.insert(
                "dataflow_name".to_string(),
                serde_json::json!(selected_node.dataflow_name),
            );
            json_output.insert(
                "daemon_id".to_string(),
                serde_json::json!(selected_node.daemon_id),
            );

            if let Some(metrics) = &selected_node.metrics {
                let mut metrics_map = serde_json::Map::new();
                metrics_map.insert("pid".to_string(), serde_json::json!(metrics.pid));
                metrics_map.insert(
                    "cpu_usage".to_string(),
                    serde_json::json!(metrics.cpu_usage),
                );
                metrics_map.insert(
                    "memory_mb".to_string(),
                    serde_json::json!(metrics.memory_mb),
                );
                if let Some(disk_read) = metrics.disk_read_mb_s {
                    metrics_map.insert("disk_read_mb_s".to_string(), serde_json::json!(disk_read));
                }
                if let Some(disk_write) = metrics.disk_write_mb_s {
                    metrics_map
                        .insert("disk_write_mb_s".to_string(), serde_json::json!(disk_write));
                }
                if let Some(start_time) = metrics.start_time {
                    metrics_map.insert("start_time".to_string(), serde_json::json!(start_time));
                }
                json_output.insert("metrics".to_string(), serde_json::json!(metrics_map));
            }

            // Add inputs
            let inputs: Vec<String> = node_descriptor
                .inputs
                .iter()
                .map(|(id, input)| format!("{} (from {})", id, input.mapping))
                .collect();
            json_output.insert("inputs".to_string(), serde_json::json!(inputs));

            // Add outputs
            let outputs: Vec<String> = node_descriptor
                .outputs
                .iter()
                .map(|id| id.to_string())
                .collect();
            json_output.insert("outputs".to_string(), serde_json::json!(outputs));

            println!("{}", serde_json::to_string_pretty(&json_output)?);
        }
    }

    Ok(())
}
