use crate::{
    config::{CommunicationConfig, Input, InputMapping, NodeRunConfig},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{eyre, Result};
use log::warn;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_with_expand_env::with_expand_envs;
use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
    path::PathBuf,
};

pub const SHELL_SOURCE: &str = "shell";
pub const DYNAMIC_SOURCE: &str = "dynamic";

/// Dataflow description
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[schemars(title = "dora-rs specification")]
pub struct Descriptor {
    #[schemars(skip)]
    #[serde(default)]
    pub communication: CommunicationConfig,
    #[schemars(skip)]
    #[serde(default, rename = "_unstable_deploy")]
    pub deploy: Deploy,
    pub nodes: Vec<Node>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Deploy {
    pub machine: Option<String>,
}

/// Dora Node
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Node {
    /// Node identifier
    pub id: NodeId,
    /// Node name
    pub name: Option<String>,
    /// Description of the node
    pub description: Option<String>,
    /// Environment variables
    pub env: Option<BTreeMap<String, EnvValue>>,

    /// Unstable machine deployment configuration
    #[schemars(skip)]
    #[serde(default, rename = "_unstable_deploy")]
    pub deploy: Deploy,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operators: Option<RuntimeNode>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom: Option<CustomNode>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operator: Option<SingleOperatorDefinition>,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,
    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedNode {
    pub id: NodeId,
    pub name: Option<String>,
    pub description: Option<String>,
    pub env: Option<BTreeMap<String, EnvValue>>,

    #[serde(default)]
    pub deploy: ResolvedDeploy,

    #[serde(flatten)]
    pub kind: CoreNodeKind,
}

impl ResolvedNode {
    pub fn send_stdout_as(&self) -> Result<Option<String>> {
        match &self.kind {
            // TODO: Split stdout between operators
            CoreNodeKind::Runtime(n) => {
                let count = n
                    .operators
                    .iter()
                    .filter(|op| op.config.send_stdout_as.is_some())
                    .count();
                if count == 1 && n.operators.len() > 1 {
                    warn!("All stdout from all operators of a runtime are going to be sent in the selected `send_stdout_as` operator.")
                } else if count > 1 {
                    return Err(eyre!("More than one `send_stdout_as` entries for a runtime node. Please only use one `send_stdout_as` per runtime."));
                }
                Ok(n.operators.iter().find_map(|op| {
                    op.config
                        .send_stdout_as
                        .clone()
                        .map(|stdout| format!("{}/{}", op.id, stdout))
                }))
            }
            CoreNodeKind::Custom(n) => Ok(n.send_stdout_as.clone()),
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ResolvedDeploy {
    pub machine: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CoreNodeKind {
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
}

pub fn runtime_node_inputs(n: &RuntimeNode) -> BTreeMap<DataId, Input> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator.config.inputs.iter().map(|(input_id, mapping)| {
                (
                    DataId::from(format!("{}/{input_id}", operator.id)),
                    mapping.clone(),
                )
            })
        })
        .collect()
}

fn runtime_node_outputs(n: &RuntimeNode) -> BTreeSet<DataId> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator
                .config
                .outputs
                .iter()
                .map(|output_id| DataId::from(format!("{}/{output_id}", operator.id)))
        })
        .collect()
}

impl CoreNodeKind {
    pub fn run_config(&self) -> NodeRunConfig {
        match self {
            CoreNodeKind::Runtime(n) => NodeRunConfig {
                inputs: runtime_node_inputs(n),
                outputs: runtime_node_outputs(n),
            },
            CoreNodeKind::Custom(n) => n.run_config.clone(),
        }
    }

    pub fn dynamic(&self) -> bool {
        match self {
            CoreNodeKind::Runtime(_n) => false,
            CoreNodeKind::Custom(n) => n.source == DYNAMIC_SOURCE,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct RuntimeNode {
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorDefinition {
    pub id: OperatorId,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct SingleOperatorDefinition {
    /// ID is optional if there is only a single operator.
    pub id: Option<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorConfig {
    pub name: Option<String>,
    pub description: Option<String>,

    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,

    #[serde(flatten)]
    pub source: OperatorSource,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
#[serde(rename_all = "kebab-case")]
pub enum OperatorSource {
    SharedLibrary(String),
    Python(PythonSource),
    #[schemars(skip)]
    Wasm(String),
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(
    deny_unknown_fields,
    from = "PythonSourceDef",
    into = "PythonSourceDef"
)]
pub struct PythonSource {
    pub source: String,
    pub conda_env: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum PythonSourceDef {
    SourceOnly(String),
    WithOptions {
        source: String,
        conda_env: Option<String>,
    },
}

impl From<PythonSource> for PythonSourceDef {
    fn from(input: PythonSource) -> Self {
        match input {
            PythonSource {
                source,
                conda_env: None,
            } => Self::SourceOnly(source),
            PythonSource { source, conda_env } => Self::WithOptions { source, conda_env },
        }
    }
}

impl From<PythonSourceDef> for PythonSource {
    fn from(value: PythonSourceDef) -> Self {
        match value {
            PythonSourceDef::SourceOnly(source) => Self {
                source,
                conda_env: None,
            },
            PythonSourceDef::WithOptions { source, conda_env } => Self { source, conda_env },
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct PythonOperatorConfig {
    pub path: PathBuf,
    #[serde(default)]
    pub inputs: BTreeMap<DataId, InputMapping>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct CustomNode {
    /// Path of the source code
    ///
    /// If you want to use a specific `conda` environment.
    /// Provide the python path within the source.
    ///
    /// source: /home/peter/miniconda3/bin/python
    ///
    /// args: some_node.py
    ///
    /// Source can match any executable in PATH.
    pub source: String,
    /// Args for the executable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,
    /// Environment variables for the custom nodes
    ///
    /// Deprecated, use outer-level `env` field instead.
    pub envs: Option<BTreeMap<String, EnvValue>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// Send stdout and stderr to another node
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,

    #[serde(flatten)]
    pub run_config: NodeRunConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum EnvValue {
    #[serde(deserialize_with = "with_expand_envs")]
    Bool(bool),
    #[serde(deserialize_with = "with_expand_envs")]
    Integer(u64),
    #[serde(deserialize_with = "with_expand_envs")]
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
