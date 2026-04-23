use std::io::Write;

use clap::Args;
use colored::Colorize;
use serde::Serialize;
use tabwriter::TabWriter;

use crate::{
    command::{Executable, default_tracing},
    common::{
        CoordinatorOptions, expect_reply, resolve_dataflow_identifier_interactive,
        send_control_request,
    },
    formatting::OutputFormat,
    ws_client::WsSession,
};
use dora_core::config::InputMapping;
use dora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::NodeInfo, descriptor::Descriptor,
    id::NodeId,
};

/// Show detailed information about a specific node.
///
/// Displays inputs, outputs, subscribers, restart policy, and runtime metrics.
///
/// Examples:
///
/// Show info for a node:
///   dora node info camera_node
///
/// Show info for a node in a specific dataflow:
///   dora node info camera_node --dataflow my-dataflow
///
/// Output as JSON:
///   dora node info camera_node --format json
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Info {
    /// Node ID to inspect
    #[clap(value_name = "NODE")]
    pub node: String,

    /// Filter by dataflow name or UUID
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    pub dataflow: Option<String>,

    /// Output format
    #[clap(long, short = 'f', value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Info {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;
        info(&session, &self.node, self.dataflow.as_deref(), self.format)
    }
}

#[derive(Serialize)]
struct NodeInfoOutput {
    node_id: String,
    dataflow: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    path: Option<String>,
    outputs: Vec<OutputInfo>,
    inputs: Vec<InputInfo>,
    restart_policy: String,
    max_restarts: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    restart_delay: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    health_check_timeout: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    metrics: Option<MetricsOutput>,
}

#[derive(Serialize)]
struct OutputInfo {
    id: String,
    subscribers: Vec<String>,
}

#[derive(Serialize)]
struct InputInfo {
    id: String,
    source: String,
}

#[derive(Serialize)]
struct MetricsOutput {
    status: String,
    pid: u32,
    cpu: String,
    memory: String,
    restart_count: u32,
    pending_messages: u64,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    broken_inputs: Vec<String>,
}

fn info(
    session: &WsSession,
    node_name: &str,
    dataflow_filter: Option<&str>,
    format: OutputFormat,
) -> eyre::Result<()> {
    let node_id: NodeId = node_name
        .parse::<NodeId>()
        .map_err(|e| eyre::eyre!("invalid node ID: {e}"))?;

    // Resolve the dataflow
    let dataflow_uuid = resolve_dataflow_identifier_interactive(session, dataflow_filter)?;

    // Get descriptor
    let reply = send_control_request(session, &ControlRequest::Info { dataflow_uuid })?;
    let (dataflow_name, descriptor) = expect_reply!(reply, DataflowInfo { name, descriptor })?;

    // Find the node in the descriptor
    let node_desc = descriptor
        .nodes
        .iter()
        .find(|n| n.id == node_id)
        .ok_or_else(|| {
            let available: Vec<_> = descriptor.nodes.iter().map(|n| n.id.to_string()).collect();
            eyre::eyre!(
                "node `{node_name}` not found in dataflow\n\n  \
                 hint: available nodes: {}",
                available.join(", ")
            )
        })?;

    // Get runtime metrics
    let metrics = fetch_node_metrics(session, dataflow_uuid, &node_id)?;

    // Build output info with subscribers
    let outputs = build_output_info(node_desc, &descriptor);
    let inputs = build_input_info(node_desc);

    let dataflow_display = dataflow_name
        .clone()
        .unwrap_or_else(|| dataflow_uuid.to_string());

    let output = NodeInfoOutput {
        node_id: node_name.to_string(),
        dataflow: dataflow_display.clone(),
        name: node_desc.name.clone(),
        description: node_desc.description.clone(),
        path: node_desc.path.clone(),
        outputs,
        inputs,
        restart_policy: format!("{:?}", node_desc.restart_policy),
        max_restarts: node_desc.max_restarts,
        restart_delay: node_desc.restart_delay,
        health_check_timeout: node_desc.health_check_timeout,
        metrics: metrics.and_then(|m| m.metrics).map(|m| MetricsOutput {
            status: m.status.to_string(),
            pid: m.pid,
            cpu: format!("{:.1}%", m.cpu_usage),
            memory: format!("{:.0} MB", m.memory_mb),
            restart_count: m.restart_count,
            pending_messages: m.pending_messages,
            broken_inputs: m.broken_inputs,
        }),
    };

    match format {
        OutputFormat::Table => print_table(&output),
        OutputFormat::Json => {
            println!("{}", serde_json::to_string_pretty(&output)?);
        }
    }

    Ok(())
}

fn fetch_node_metrics(
    session: &WsSession,
    dataflow_uuid: uuid::Uuid,
    node_id: &NodeId,
) -> eyre::Result<Option<NodeInfo>> {
    let reply = send_control_request(session, &ControlRequest::GetNodeInfo)?;
    let node_infos = expect_reply!(reply, NodeInfoList(infos))?;

    Ok(node_infos
        .into_iter()
        .find(|n| n.dataflow_id == dataflow_uuid && n.node_id == *node_id))
}

fn build_output_info(
    node_desc: &dora_message::descriptor::Node,
    descriptor: &Descriptor,
) -> Vec<OutputInfo> {
    node_desc
        .outputs
        .iter()
        .map(|output_id| {
            let mut subscribers = Vec::new();
            for other_node in &descriptor.nodes {
                for (input_id, input) in &other_node.inputs {
                    if let InputMapping::User(user) = &input.mapping
                        && user.source == node_desc.id
                        && user.output == *output_id
                    {
                        subscribers.push(format!("{}/{}", other_node.id, input_id));
                    }
                }
            }
            OutputInfo {
                id: output_id.to_string(),
                subscribers,
            }
        })
        .collect()
}

fn build_input_info(node_desc: &dora_message::descriptor::Node) -> Vec<InputInfo> {
    node_desc
        .inputs
        .iter()
        .map(|(input_id, input)| {
            let source = match &input.mapping {
                InputMapping::User(user) => format!("{}/{}", user.source, user.output),
                InputMapping::Timer { interval } => {
                    format!("dora/timer/millis/{}", interval.as_millis())
                }
                mapping @ InputMapping::Logs(_) => mapping.to_string(),
            };
            InputInfo {
                id: input_id.to_string(),
                source,
            }
        })
        .collect()
}

fn print_table(output: &NodeInfoOutput) {
    println!("{}: {}", "Node".bold(), output.node_id);
    println!("{}: {}", "Dataflow".bold(), output.dataflow);
    if let Some(name) = &output.name {
        println!("{}: {}", "Name".bold(), name);
    }
    if let Some(desc) = &output.description {
        println!("{}: {}", "Description".bold(), desc);
    }
    if let Some(path) = &output.path {
        println!("{}: {}", "Path".bold(), path);
    }
    println!();

    // Restart policy
    println!("{}", "Restart Policy:".bold());
    println!("  Policy: {}", output.restart_policy);
    println!("  Max restarts: {}", output.max_restarts);
    if let Some(delay) = output.restart_delay {
        println!("  Restart delay: {delay}s");
    }
    if let Some(timeout) = output.health_check_timeout {
        println!("  Health check timeout: {timeout}s");
    }
    println!();

    // Inputs
    println!("{}", "Inputs:".bold());
    if output.inputs.is_empty() {
        println!("  <none>");
    } else {
        let mut tw = TabWriter::new(std::io::stdout().lock());
        for input in &output.inputs {
            let _ = tw.write_all(format!("  {}\t<- {}\n", input.id, input.source).as_bytes());
        }
        let _ = tw.flush();
    }
    println!();

    // Outputs
    println!("{}", "Outputs:".bold());
    if output.outputs.is_empty() {
        println!("  <none>");
    } else {
        for out in &output.outputs {
            if out.subscribers.is_empty() {
                println!("  {} (no subscribers)", out.id);
            } else {
                println!("  {} -> {}", out.id, out.subscribers.join(", "));
            }
        }
    }
    println!();

    // Runtime metrics
    println!("{}", "Runtime Metrics:".bold());
    if let Some(metrics) = &output.metrics {
        let mut tw = TabWriter::new(std::io::stdout().lock());
        let _ = tw.write_all(format!("  Status:\t{}\n", metrics.status).as_bytes());
        let _ = tw.write_all(format!("  PID:\t{}\n", metrics.pid).as_bytes());
        let _ = tw.write_all(format!("  CPU:\t{}\n", metrics.cpu).as_bytes());
        let _ = tw.write_all(format!("  Memory:\t{}\n", metrics.memory).as_bytes());
        let _ = tw.write_all(format!("  Restarts:\t{}\n", metrics.restart_count).as_bytes());
        let _ = tw.write_all(format!("  Pending msgs:\t{}\n", metrics.pending_messages).as_bytes());
        if !metrics.broken_inputs.is_empty() {
            let _ = tw.write_all(
                format!("  Broken inputs:\t{}\n", metrics.broken_inputs.join(", ")).as_bytes(),
            );
        }
        let _ = tw.flush();
    } else {
        println!("  <not running or metrics unavailable>");
    }
}
