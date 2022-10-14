use dora_node_api::config::{CommunicationConfig, NodeRunConfig};
pub use dora_node_api::config::{DataId, InputMapping, NodeId, OperatorId, UserInputMapping};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};
use std::{fmt, iter};
pub use visualize::collect_dora_timers;

mod visualize;

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Descriptor {
    // see https://github.com/dtolnay/serde-yaml/issues/298
    #[serde(with = "serde_yaml::with::singleton_map")]
    pub communication: CommunicationConfig,
    pub nodes: Vec<Node>,
}
pub const SINGLE_OPERATOR_DEFAULT_ID: &str = "op";

impl Descriptor {
    pub fn resolve_aliases(&self) -> Vec<ResolvedNode> {
        let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

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

            resolved.extend(node.resolve());
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
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub clone_ids: Vec<NodeId>,
    pub name: Option<String>,
    pub description: Option<String>,

    #[serde(flatten)]
    pub kind: NodeKind,
}

impl Node {
    pub fn resolve(&self) -> impl Iterator<Item = ResolvedNode> {
        let kind = match &self.kind {
            NodeKind::Custom(node) => CoreNodeKind::Custom(node.clone()),
            NodeKind::Runtime(node) => CoreNodeKind::Runtime(node.resolve()),
            NodeKind::Operator(op) => CoreNodeKind::Runtime(op.resolve()),
        };
        let resolved = ResolvedNode {
            id: self.id.clone(),
            name: self.name.clone(),
            description: self.description.clone(),
            kind,
        };

        iter::once(self.id.clone())
            .chain(self.clone_ids.clone())
            .map(move |id| ResolvedNode {
                id,
                ..resolved.clone()
            })
    }
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

#[derive(Debug, Clone)]
pub struct ResolvedNode {
    pub id: NodeId,
    pub name: Option<String>,
    pub description: Option<String>,

    pub kind: CoreNodeKind,
}

#[derive(Debug, Clone)]
pub enum CoreNodeKind {
    /// Dora runtime node
    Runtime(ResolvedRuntimeNode),
    Custom(CustomNode),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RuntimeNode {
    pub operators: Vec<OperatorDefinition>,
}

impl RuntimeNode {
    fn resolve(&self) -> ResolvedRuntimeNode {
        let mut resolved = Vec::new();
        for operator in &self.operators {
            resolved.extend(operator.resolve());
        }
        ResolvedRuntimeNode {
            operators: resolved,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ResolvedRuntimeNode {
    pub operators: Vec<ResolvedOperatorDefinition>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OperatorDefinition {
    pub id: OperatorId,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub clone_ids: Vec<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

impl OperatorDefinition {
    fn resolve(&self) -> impl Iterator<Item = ResolvedOperatorDefinition> {
        let config = self.config.clone();
        iter::once(self.id.clone())
            .chain(self.clone_ids.clone())
            .map(move |id| ResolvedOperatorDefinition {
                id,
                config: config.clone(),
            })
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ResolvedOperatorDefinition {
    pub id: OperatorId,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SingleOperatorDefinition {
    /// ID is optional if there is only a single operator.
    pub id: Option<OperatorId>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub clone_ids: Vec<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

impl SingleOperatorDefinition {
    fn resolve(&self) -> ResolvedRuntimeNode {
        let operator = OperatorDefinition {
            id: self
                .id
                .clone()
                .unwrap_or_else(|| OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string())),
            clone_ids: self.clone_ids.clone(),
            config: self.config.clone(),
        };
        ResolvedRuntimeNode {
            operators: operator.resolve().collect(),
        }
    }
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

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "kebab-case")]
pub enum OperatorSource {
    SharedLibrary(PathBuf),
    Python(PathBuf),
    Wasm(PathBuf),
}

impl OperatorSource {
    pub fn canonicalize(&mut self) -> std::io::Result<()> {
        let path = match self {
            OperatorSource::SharedLibrary(path) => path,
            OperatorSource::Python(path) => path,
            OperatorSource::Wasm(path) => path,
        };
        *path = path.canonicalize()?;
        Ok(())
    }
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,

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
