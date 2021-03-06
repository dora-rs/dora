use super::{CustomNode, Node, NodeKind, OperatorConfig, RuntimeNode};
use dora_node_api::config::{DataId, InputMapping, NodeId};
use std::{
    collections::{BTreeMap, HashMap},
    fmt::Write as _,
};

pub fn visualize_nodes(nodes: &[Node]) -> String {
    let mut flowchart = "flowchart TB\n".to_owned();
    let mut all_nodes = HashMap::new();

    for node in nodes {
        visualize_node(node, &mut flowchart);
        all_nodes.insert(&node.id, node);
    }

    for node in nodes {
        visualize_node_inputs(node, &mut flowchart, &all_nodes)
    }

    flowchart
}

fn visualize_node(node: &Node, flowchart: &mut String) {
    let node_id = &node.id;
    match &node.kind {
        NodeKind::Custom(node) => visualize_custom_node(node_id, node, flowchart),
        NodeKind::Runtime(RuntimeNode { operators }) => {
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

fn visualize_runtime_node(node_id: &NodeId, operators: &[OperatorConfig], flowchart: &mut String) {
    writeln!(flowchart, "subgraph {node_id}").unwrap();
    for operator in operators {
        let operator_id = &operator.id;
        if operator.inputs.is_empty() {
            // source operator
            writeln!(flowchart, "  {node_id}/{operator_id}[\\{operator_id}/]").unwrap();
        } else if operator.outputs.is_empty() {
            // sink operator
            writeln!(flowchart, "  {node_id}/{operator_id}[/{operator_id}\\]").unwrap();
        } else {
            // normal operator
            writeln!(flowchart, "  {node_id}/{operator_id}[{operator_id}]").unwrap();
        }
    }

    flowchart.push_str("end\n");
}

fn visualize_node_inputs(node: &Node, flowchart: &mut String, nodes: &HashMap<&NodeId, &Node>) {
    let node_id = &node.id;
    match &node.kind {
        NodeKind::Custom(node) => visualize_inputs(
            &node_id.to_string(),
            &node.run_config.inputs,
            flowchart,
            nodes,
        ),
        NodeKind::Runtime(RuntimeNode { operators }) => {
            for operator in operators {
                visualize_inputs(
                    &format!("{node_id}/{}", operator.id),
                    &operator.inputs,
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
    nodes: &HashMap<&NodeId, &Node>,
) {
    for (input_id, mapping) in inputs {
        let InputMapping {
            source,
            operator,
            output,
        } = mapping;

        let mut source_found = false;
        if let Some(source_node) = nodes.get(source) {
            match (&source_node.kind, operator) {
                (NodeKind::Custom(custom_node), None) => {
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
                (NodeKind::Runtime(RuntimeNode { operators }), Some(operator_id)) => {
                    if let Some(operator) = operators.iter().find(|o| &o.id == operator_id) {
                        if operator.outputs.contains(output) {
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
                (NodeKind::Custom(_), Some(_)) | (NodeKind::Runtime(_), None) => {}
            }
        }

        if !source_found {
            writeln!(flowchart, "  missing>missing] -- {input_id} --> {target}").unwrap();
        }
    }
}
