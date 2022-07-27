use dora_node_api::config::{
    CommunicationConfig, DataId, InputMapping, NodeId, NodeRunConfig, OperatorId,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

mod visualize;

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Descriptor {
    pub communication: CommunicationConfig,
    pub nodes: Vec<Node>,
}

impl Descriptor {
    pub fn resolve_aliases(&self) -> Vec<ResolvedNode> {
        let default_op_id = OperatorId::from("op".to_string());

        let single_operator_nodes: HashMap<_, _> = self
            .nodes
            .iter()
            .filter_map(|n| match &n.kind {
                NodeKind::Operator(op) => Some((&n.id, op.id.as_ref().unwrap_or(&default_op_id))),
                _ => None,
            })
            .collect();

        let mut resolved = vec![];
        for mut node in self.nodes.clone() {
            // adjust input mappings
            let input_mappings: Vec<_> = match &mut node.kind {
                NodeKind::Runtime(node) => node
                    .operators
                    .iter_mut()
                    .flat_map(|op| op.config.inputs.values_mut())
                    .collect(),
                NodeKind::Custom(node) => node.run_config.inputs.values_mut().collect(),
                NodeKind::Operator(operator) => operator.config.inputs.values_mut().collect(),
            };
            for mapping in input_mappings.into_iter().filter_map(|m| match m {
                InputMapping::Timer { .. } => None,
                InputMapping::User(m) => Some(m),
            }) {
                if let Some(op_name) = single_operator_nodes.get(&mapping.source).copied() {
                    if mapping.operator.is_none() {
                        mapping.operator = Some(op_name.to_owned());
                    }
                }
            }

            // resolve nodes
            let kind = match node.kind {
                NodeKind::Custom(node) => CoreNodeKind::Custom(node),
                NodeKind::Runtime(node) => CoreNodeKind::Runtime(node),
                NodeKind::Operator(op) => CoreNodeKind::Runtime(RuntimeNode {
                    operators: vec![OperatorDefinition {
                        id: op.id.unwrap_or_else(|| default_op_id.clone()),
                        config: op.config,
                    }],
                }),
            };
            resolved.push(ResolvedNode {
                id: node.id,
                name: node.name,
                description: node.description,
                kind,
            });
        }
        resolved
    }

    pub fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let resolved = self.resolve_aliases();
        let flowchart = visualize::visualize_nodes(&resolved);

        Ok(flowchart)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub name: Option<String>,
    pub description: Option<String>,

    #[serde(flatten)]
    pub kind: NodeKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NodeKind {
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
    Operator(SingleOperatorDefinition),
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ResolvedNode {
    pub id: NodeId,
    pub name: Option<String>,
    pub description: Option<String>,

    #[serde(flatten)]
    pub kind: CoreNodeKind,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CoreNodeKind {
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RuntimeNode {
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OperatorDefinition {
    pub id: OperatorId,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SingleOperatorDefinition {
    /// ID is optional if there is only a single operator.
    pub id: Option<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OperatorConfig {
    pub name: Option<String>,
    pub description: Option<String>,

    #[serde(default)]
    pub inputs: BTreeMap<DataId, InputMapping>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,

    #[serde(flatten)]
    pub source: OperatorSource,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "kebab-case")]
pub enum OperatorSource {
    SharedLibrary(PathBuf),
    Python(PathBuf),
    Wasm(PathBuf),
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PythonOperatorConfig {
    pub path: PathBuf,
    #[serde(default)]
    pub inputs: BTreeMap<DataId, InputMapping>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomNode {
    pub run: String,
    pub env: Option<BTreeMap<String, EnvValue>>,
    pub working_directory: Option<BTreeMap<String, EnvValue>>,

    #[serde(flatten)]
    pub run_config: NodeRunConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum EnvValue {
    Bool(bool),
    Integer(u64),
    String(String),
}

impl fmt::Display for EnvValue {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        match self {
            EnvValue::Bool(bool) => fmt.write_str(&bool.to_string()),
            EnvValue::Integer(u64) => fmt.write_str(&u64.to_string()),
            EnvValue::String(str) => fmt.write_str(str),
        }
    }
}
