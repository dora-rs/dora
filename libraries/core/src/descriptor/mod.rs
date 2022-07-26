use dora_node_api::config::{
    CommunicationConfig, DataId, InputMapping, NodeId, NodeRunConfig, OperatorId,
};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
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
        const PYTHON_OP_NAME: &str = "op";

        let python_nodes: HashSet<_> = self
            .nodes
            .iter()
            .filter(|n| matches!(n.kind, NodeKind::Python(_)))
            .map(|n| &n.id)
            .collect();

        let mut resolved = vec![];
        for mut node in self.nodes.clone() {
            // adjust input mappings
            let input_mappings: Vec<_> = match &mut node.kind {
                NodeKind::Runtime(node) => node
                    .operators
                    .iter_mut()
                    .flat_map(|op| op.inputs.values_mut())
                    .collect(),
                NodeKind::Custom(node) => node.run_config.inputs.values_mut().collect(),
                NodeKind::Python(node) => node.inputs.values_mut().collect(),
            };
            for mapping in input_mappings {
                if python_nodes.contains(&mapping.source) {
                    assert_eq!(mapping.operator, None);
                    mapping.operator = Some(OperatorId::from(PYTHON_OP_NAME.to_string()));
                }
            }

            // resolve nodes
            let kind = match node.kind {
                NodeKind::Custom(node) => CoreNodeKind::Custom(node),
                NodeKind::Runtime(node) => CoreNodeKind::Runtime(node),
                NodeKind::Python(node) => CoreNodeKind::Runtime(RuntimeNode {
                    operators: vec![OperatorConfig {
                        id: OperatorId::from(PYTHON_OP_NAME.to_string()),
                        name: None,
                        description: None,
                        inputs: node.inputs,
                        outputs: node.outputs,
                        source: OperatorSource::Python(node.path),
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
    Python(PythonOperatorConfig),
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
    pub operators: Vec<OperatorConfig>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OperatorConfig {
    pub id: OperatorId,
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
