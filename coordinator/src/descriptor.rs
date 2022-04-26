use dora_api::config::{CommunicationConfig, InputMapping, NodeId, NodeRunConfig};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Descriptor {
    pub communication: CommunicationConfig,
    pub nodes: Vec<Node>,
}

impl Descriptor {
    pub fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let mut flowchart = "flowchart TB\n".to_owned();

        let mut nodes = HashMap::new();
        for node in &self.nodes {
            nodes.insert(&node.id, node);
        }

        for node in &self.nodes {
            let node_id = &node.id;

            match &node.kind {
                NodeKind::Custom(node) => {
                    if node.run_config.inputs.is_empty() {
                        // source node
                        flowchart.push_str(&format!("  {node_id}[\\{node_id}/]\n"));
                    } else if node.run_config.outputs.is_empty() {
                        // sink node
                        flowchart.push_str(&format!("  {node_id}[/{node_id}\\]\n"));
                    } else {
                        // normal node
                        flowchart.push_str(&format!("  {node_id}\n"));
                    }

                    for (input_id, mapping) in &node.run_config.inputs {
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
                                        flowchart.push_str(&format!(
                                            "  {source} -- {data} --> {node_id}\n"
                                        ));
                                        source_found = true;
                                    }
                                }
                                (NodeKind::Custom(_), Some(_)) => {}
                            }
                        }

                        if !source_found {
                            flowchart.push_str(&format!(
                                "  missing>missing] -- {input_id} --> {node_id}\n"
                            ));
                        }
                    }
                }
            }
        }

        Ok(flowchart)
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub name: Option<String>,
    pub description: Option<String>,

    #[serde(flatten)]
    pub kind: NodeKind,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NodeKind {
    Custom(CustomNode),
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CustomNode {
    pub run: String,
    pub env: Option<BTreeMap<String, EnvValue>>,
    pub working_directory: Option<BTreeMap<String, EnvValue>>,

    #[serde(flatten)]
    pub run_config: NodeRunConfig,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(untagged)]
pub enum EnvValue {
    Bool(bool),
    Integer(u64),
    String(String),
}
