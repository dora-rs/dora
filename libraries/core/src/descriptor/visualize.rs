use adora_message::{
    config::{Input, InputMapping, UserInputMapping, format_duration},
    descriptor::{CoreNodeKind, OperatorDefinition},
    id::{DataId, NodeId},
};

use super::{CustomNode, ModuleBoundaries, ResolvedNode, RuntimeNode};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
    fmt::Write as _,
    time::Duration,
};

pub fn visualize_nodes(nodes: &BTreeMap<NodeId, ResolvedNode>) -> String {
    visualize_nodes_with_boundaries(nodes, &ModuleBoundaries::default())
}

pub fn visualize_nodes_with_boundaries(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    boundaries: &ModuleBoundaries,
) -> String {
    let mut flowchart = "flowchart TB\n".to_owned();
    let mut all_nodes = HashMap::new();

    // Track which nodes are inside a module subgraph
    let mut module_nodes: HashSet<&str> = HashSet::new();
    for members in boundaries.modules.values() {
        for m in members {
            module_nodes.insert(m.as_str());
        }
    }

    // Render module subgraphs first
    for (module_id, members) in &boundaries.modules {
        // Sanitize ID for Mermaid (dots are invalid in subgraph IDs)
        let safe_id = module_id.replace('.', "_");
        writeln!(flowchart, "subgraph {safe_id} [{module_id}]").unwrap();
        for node in nodes.values() {
            let node_id_str: &str = node.id.as_ref();
            if members.iter().any(|m| m == node_id_str) {
                visualize_node(node, &mut flowchart);
                all_nodes.insert(&node.id, node);
            }
        }
        flowchart.push_str("end\n");
    }

    // Render non-module nodes
    for node in nodes.values() {
        let node_id_str: &str = node.id.as_ref();
        if !module_nodes.contains(node_id_str) {
            visualize_node(node, &mut flowchart);
            all_nodes.insert(&node.id, node);
        }
    }

    let adora_timers = collect_adora_timers(nodes);
    if !adora_timers.is_empty() {
        writeln!(flowchart, "subgraph ___adora___ [adora]").unwrap();
        writeln!(flowchart, "  subgraph ___timer_timer___ [timer]").unwrap();
        for interval in adora_timers {
            let duration = format_duration(interval);
            writeln!(flowchart, "    adora/timer/{duration}[\\{duration}/]").unwrap();
        }
        flowchart.push_str("  end\n");
        flowchart.push_str("end\n");
    }

    for node in nodes.values() {
        visualize_node_inputs(node, &mut flowchart, &all_nodes)
    }

    flowchart
}

pub fn collect_adora_timers(nodes: &BTreeMap<NodeId, ResolvedNode>) -> BTreeSet<Duration> {
    let mut adora_timers = BTreeSet::new();
    for node in nodes.values() {
        match &node.kind {
            CoreNodeKind::Runtime(node) => {
                for operator in &node.operators {
                    collect_adora_nodes(operator.config.inputs.values(), &mut adora_timers);
                }
            }
            CoreNodeKind::Custom(node) => {
                collect_adora_nodes(node.run_config.inputs.values(), &mut adora_timers);
            }
        }
    }
    adora_timers
}

fn collect_adora_nodes(
    values: std::collections::btree_map::Values<DataId, Input>,
    adora_timers: &mut BTreeSet<Duration>,
) {
    for input in values {
        match &input.mapping {
            InputMapping::User(_) => {}
            InputMapping::Timer { interval } => {
                adora_timers.insert(*interval);
            }
        }
    }
}

fn visualize_node(node: &ResolvedNode, flowchart: &mut String) {
    let node_id = &node.id;
    let description = if let Some(desc) = &node.description {
        format!("<hr/>*{desc}*")
    } else {
        "".to_string()
    };

    match &node.kind {
        CoreNodeKind::Custom(node) => visualize_custom_node(node_id, description, node, flowchart),
        CoreNodeKind::Runtime(RuntimeNode { operators, .. }) => {
            visualize_runtime_node(node_id, description, operators, flowchart)
        }
    }
}

fn visualize_custom_node(
    node_id: &NodeId,
    description: String,
    node: &CustomNode,
    flowchart: &mut String,
) {
    if node.run_config.inputs.is_empty() {
        // source node
        writeln!(flowchart, "  {node_id}[\\\"**{node_id}**{description}\"/]").unwrap();
    } else if node.run_config.outputs.is_empty() {
        // sink node
        writeln!(flowchart, "  {node_id}[/\"**{node_id}**{description}\"\\]").unwrap();
    } else {
        // normal node
        writeln!(flowchart, "  {node_id}[\"**{node_id}**{description}\"]").unwrap();
    }
}

fn visualize_runtime_node(
    node_id: &NodeId,
    _description: String,
    operators: &[OperatorDefinition],
    flowchart: &mut String,
) {
    if operators.len() == 1 && operators[0].id.to_string() == "op" {
        let operator = &operators[0];
        // single operator node
        if operator.config.inputs.is_empty() {
            // source node
            writeln!(flowchart, "  {node_id}/op[\\{node_id}/]").unwrap();
        } else if operator.config.outputs.is_empty() {
            // sink node
            writeln!(flowchart, "  {node_id}/op[/{node_id}\\]").unwrap();
        } else {
            // normal node
            writeln!(flowchart, "  {node_id}/op[{node_id}]").unwrap();
        }
    } else {
        writeln!(flowchart, "subgraph {node_id}").unwrap();
        for operator in operators {
            let operator_id = &operator.id;
            if operator.config.inputs.is_empty() {
                // source operator
                writeln!(flowchart, "  {node_id}/{operator_id}[\\{operator_id}/]").unwrap();
            } else if operator.config.outputs.is_empty() {
                // sink operator
                writeln!(flowchart, "  {node_id}/{operator_id}[/{operator_id}\\]").unwrap();
            } else {
                // normal operator
                writeln!(flowchart, "  {node_id}/{operator_id}[{operator_id}]").unwrap();
            }
        }
        flowchart.push_str("end\n");
    }
}

fn visualize_node_inputs(
    node: &ResolvedNode,
    flowchart: &mut String,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
) {
    let node_id = &node.id;
    match &node.kind {
        CoreNodeKind::Custom(node) => {
            visualize_inputs(node_id.as_ref(), &node.run_config.inputs, flowchart, nodes)
        }
        CoreNodeKind::Runtime(RuntimeNode { operators, .. }) => {
            for operator in operators {
                visualize_inputs(
                    &format!("{node_id}/{}", operator.id),
                    &operator.config.inputs,
                    flowchart,
                    nodes,
                )
            }
        }
    }
}

fn visualize_inputs(
    target: &str,
    inputs: &BTreeMap<DataId, Input>,
    flowchart: &mut String,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
) {
    for (input_id, input) in inputs {
        match &input.mapping {
            mapping @ InputMapping::Timer { .. } => {
                writeln!(flowchart, "  {mapping} -- {input_id} --> {target}").unwrap();
            }
            InputMapping::User(mapping) => {
                visualize_user_mapping(mapping, target, nodes, input_id, flowchart)
            }
        }
    }
}

fn visualize_user_mapping(
    mapping: &UserInputMapping,
    target: &str,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
    input_id: &DataId,
    flowchart: &mut String,
) {
    let UserInputMapping { source, output } = mapping;
    let mut source_found = false;
    if let Some(source_node) = nodes.get(source) {
        match &source_node.kind {
            CoreNodeKind::Custom(custom_node) => {
                if custom_node.run_config.outputs.contains(output) {
                    let type_label = custom_node
                        .run_config
                        .output_types
                        .get(output)
                        .map(|t| format_type_label(t))
                        .unwrap_or_default();
                    let data = if output == input_id {
                        format!("{output}{type_label}")
                    } else {
                        format!("{output} as {input_id}{type_label}")
                    };
                    writeln!(flowchart, "  {source} -- {data} --> {target}").unwrap();
                    source_found = true;
                }
            }
            CoreNodeKind::Runtime(RuntimeNode { operators, .. }) => {
                let (operator_id, output) = output.split_once('/').unwrap_or(("", output));
                if let Some(operator) = operators.iter().find(|o| o.id.as_ref() == operator_id) {
                    if operator.config.outputs.contains(output) {
                        let type_label = operator
                            .config
                            .output_types
                            .get(&DataId::from(output.to_owned()))
                            .map(|t| format_type_label(t))
                            .unwrap_or_default();
                        let data = if output == input_id.as_str() {
                            format!("{output}{type_label}")
                        } else {
                            format!("{output} as {input_id}{type_label}")
                        };
                        writeln!(flowchart, "  {source}/{operator_id} -- {data} --> {target}")
                            .unwrap();
                        source_found = true;
                    }
                }
            }
        }
    }
    if !source_found {
        writeln!(flowchart, "  missing>missing] -- {input_id} --> {target}").unwrap();
    }
}

/// Format a type URN as a short label for graph edges.
/// Extracts the type name from the URN (e.g. `std/media/v1/Image` -> ` [Image]`).
fn format_type_label(urn: &str) -> String {
    let short = crate::types::urn_short_name(urn);
    format!(" [{short}]")
}
