use super::{CoreNodeKind, CustomNode, OperatorDefinition, ResolvedNode, RuntimeNode};
use crate::config::{format_duration, DataId, InputMapping, NodeId, UserInputMapping};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    fmt::Write as _,
    time::Duration,
};

pub fn visualize_nodes(nodes: &[ResolvedNode]) -> String {
    let mut flowchart = "flowchart TB\n".to_owned();
    let mut all_nodes = HashMap::new();

    for node in nodes {
        visualize_node(node, &mut flowchart);
        all_nodes.insert(&node.id, node);
    }

    let dora_timers = collect_dora_timers(nodes);
    if !dora_timers.is_empty() {
        writeln!(flowchart, "subgraph ___dora___ [dora]").unwrap();
        writeln!(flowchart, "  subgraph ___timer_timer___ [timer]").unwrap();
        for interval in dora_timers {
            let duration = format_duration(interval);
            writeln!(flowchart, "    dora/timer/{duration}[\\{duration}/]").unwrap();
        }
        flowchart.push_str("  end\n");
        flowchart.push_str("end\n");
    }

    for node in nodes {
        visualize_node_inputs(node, &mut flowchart, &all_nodes)
    }

    flowchart
}

pub fn collect_dora_timers(nodes: &[ResolvedNode]) -> BTreeSet<Duration> {
    let mut dora_timers = BTreeSet::new();
    for node in nodes {
        match &node.kind {
            CoreNodeKind::Runtime(node) => {
                for operator in &node.operators {
                    collect_dora_nodes(operator.config.inputs.values(), &mut dora_timers);
                }
            }
            CoreNodeKind::Custom(node) => {
                collect_dora_nodes(node.run_config.inputs.values(), &mut dora_timers);
            }
        }
    }
    dora_timers
}

fn collect_dora_nodes(
    values: std::collections::btree_map::Values<DataId, InputMapping>,
    dora_timers: &mut BTreeSet<Duration>,
) {
    for input in values {
        match input {
            InputMapping::User(_) => {}
            InputMapping::Timer { interval } => {
                dora_timers.insert(*interval);
            }
        }
    }
}

fn visualize_node(node: &ResolvedNode, flowchart: &mut String) {
    let node_id = &node.id;
    match &node.kind {
        CoreNodeKind::Custom(node) => visualize_custom_node(node_id, node, flowchart),
        CoreNodeKind::Runtime(RuntimeNode { operators, .. }) => {
            visualize_runtime_node(node_id, operators, flowchart)
        }
    }
}

fn visualize_custom_node(node_id: &NodeId, node: &CustomNode, flowchart: &mut String) {
    if node.run_config.inputs.is_empty() {
        // source node
        writeln!(flowchart, "  {node_id}[\\{node_id}/]").unwrap();
    } else if node.run_config.outputs.is_empty() {
        // sink node
        writeln!(flowchart, "  {node_id}[/{node_id}\\]").unwrap();
    } else {
        // normal node
        writeln!(flowchart, "  {node_id}").unwrap();
    }
}

fn visualize_runtime_node(
    node_id: &NodeId,
    operators: &[OperatorDefinition],
    flowchart: &mut String,
) {
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

fn visualize_node_inputs(
    node: &ResolvedNode,
    flowchart: &mut String,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
) {
    let node_id = &node.id;
    match &node.kind {
        CoreNodeKind::Custom(node) => visualize_inputs(
            &node_id.to_string(),
            &node.run_config.inputs,
            flowchart,
            nodes,
        ),
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
    inputs: &BTreeMap<DataId, InputMapping>,
    flowchart: &mut String,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
) {
    for (input_id, mapping) in inputs {
        match mapping {
            mapping @ InputMapping::Timer { .. } => {
                writeln!(flowchart, "  {} -- {input_id} --> {target}", mapping).unwrap();
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
    let UserInputMapping {
        source,
        operator,
        output,
    } = mapping;
    let mut source_found = false;
    if let Some(source_node) = nodes.get(source) {
        match (&source_node.kind, operator) {
            (CoreNodeKind::Custom(custom_node), None) => {
                if custom_node.run_config.outputs.contains(output) {
                    let data = if output == input_id {
                        format!("{output}")
                    } else {
                        format!("{output} as {input_id}")
                    };
                    writeln!(flowchart, "  {source} -- {data} --> {target}").unwrap();
                    source_found = true;
                }
            }
            (CoreNodeKind::Runtime(RuntimeNode { operators, .. }), Some(operator_id)) => {
                if let Some(operator) = operators.iter().find(|o| &o.id == operator_id) {
                    if operator.config.outputs.contains(output) {
                        let data = if output == input_id {
                            format!("{output}")
                        } else {
                            format!("{output} as {input_id}")
                        };
                        writeln!(flowchart, "  {source}/{operator_id} -- {data} --> {target}")
                            .unwrap();
                        source_found = true;
                    }
                }
            }
            (CoreNodeKind::Custom(_), Some(_)) | (CoreNodeKind::Runtime(_), None) => {}
        }
    }
    if !source_found {
        writeln!(flowchart, "  missing>missing] -- {input_id} --> {target}").unwrap();
    }
}
