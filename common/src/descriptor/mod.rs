use dora_node_api::config::{
    CommunicationConfig, DataId, InputMapping, NodeId, NodeRunConfig, OperatorId,
};
use serde::{Deserialize, Serialize};
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
    pub fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let flowchart = visualize::visualize_nodes(&self.nodes);

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
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
}

#[derive(Debug, Serialize, Deserialize)]
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

impl fmt::Display for EnvValue {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        match self {
            EnvValue::Bool(bool) => fmt.write_str(&bool.to_string()),
            EnvValue::Integer(u64) => fmt.write_str(&u64.to_string()),
            EnvValue::String(str) => fmt.write_str(str),
        }
    }
}
