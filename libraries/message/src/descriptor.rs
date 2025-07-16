use crate::{
    config::{CommunicationConfig, Input, InputMapping, NodeRunConfig},
    id::{DataId, NodeId, OperatorId},
};
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

/// # Dataflow Specification
///
/// The main configuration structure for defining a Dora dataflow. Dataflows are
/// specified through YAML files that describe the nodes, their connections, and
/// execution parameters.
///
/// ## Structure
///
/// A dataflow consists of:
/// - **Nodes**: The computational units that process data
/// - **Communication**: Optional communication configuration
/// - **Debug options**: Optional development and debugging settings
/// - **Deployment**: Optional deployment configuration
///
/// ## Example
///
/// ```yaml
/// nodes:
///  - id: webcam
///     operator:
///       python: webcam.py
///       inputs:
///         tick: dora/timer/millis/100
///       outputs:
///         - image
///   - id: plot
///     operator:
///       python: plot.py
///       inputs:
///         image: webcam/image
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[schemars(title = "dora-rs specification")]
pub struct Descriptor {
    /// Communication configuration (optional, uses defaults)
    #[schemars(skip)]
    #[serde(default)]
    pub communication: CommunicationConfig,

    /// Deployment configuration (unstable feature)
    #[schemars(skip)]
    #[serde(rename = "_unstable_deploy")]
    pub deploy: Option<Deploy>,

    /// List of nodes in the dataflow
    pub nodes: Vec<Node>,

    /// Debug options (unstable feature)
    #[schemars(skip)]
    #[serde(default, rename = "_unstable_debug")]
    pub debug: Debug,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Deploy {
    /// Target machine for deployment
    pub machine: Option<String>,
    /// Working directory for the deployment
    pub working_dir: Option<PathBuf>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
pub struct Debug {
    /// Whether to publish all messages to Zenoh for debugging
    #[serde(default)]
    pub publish_all_messages_to_zenoh: bool,
}

/// # Dora Node Configuration
///
/// A node represents a computational unit in a Dora dataflow. Each node runs as a
/// separate process and can communicate with other nodes through inputs and outputs.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Node {
    /// Unique node identifier. Must not contain `/` characters.
    ///
    /// ```yaml
    /// id: camera_node
    /// ```
    pub id: NodeId,

    /// Human-readable node name for documentation.
    ///
    /// ```yaml
    /// name: "Camera Input Handler"
    /// ```
    pub name: Option<String>,

    /// Detailed description of the node's functionality.
    ///
    /// ```yaml
    /// description: "Captures video frames from webcam"
    /// ```
    pub description: Option<String>,

    /// Environment variables for node execution. Supports strings, numbers, and booleans.
    ///
    /// ```yaml
    /// env:
    ///   DEBUG: true
    ///   PORT: 8080
    ///   API_KEY: "secret-key"
    /// ```
    pub env: Option<BTreeMap<String, EnvValue>>,

    /// Unstable machine deployment configuration
    #[schemars(skip)]
    #[serde(rename = "_unstable_deploy")]
    pub deploy: Option<Deploy>,

    /// Multiple operators running in a shared runtime process.
    ///
    /// ```yaml
    /// operators:
    ///   - id: processor
    ///     python: process.py
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operators: Option<RuntimeNode>,

    /// Legacy custom node configuration (deprecated).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom: Option<CustomNode>,

    /// Single operator configuration for simple nodes.
    ///
    /// ```yaml
    /// operator:
    ///   python: script.py
    ///   outputs: [data]
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operator: Option<SingleOperatorDefinition>,

    /// Path to executable or script for custom nodes.
    ///
    /// ```yaml
    /// path: ./my_node.py
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,

    /// Git repository URL for remote nodes.
    ///
    /// ```yaml
    /// git: https://github.com/user/repo.git
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub git: Option<String>,

    /// Git branch to checkout. Only one of branch/tag/rev allowed.
    ///
    /// ```yaml
    /// branch: main
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub branch: Option<String>,

    /// Git tag to checkout. Only one of branch/tag/rev allowed.
    ///
    /// ```yaml
    /// tag: v1.0.0
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tag: Option<String>,

    /// Git commit hash to checkout. Only one of branch/tag/rev allowed.
    ///
    /// ```yaml
    /// rev: abc123def456
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rev: Option<String>,

    /// Command-line arguments passed to the executable.
    ///
    /// ```yaml
    /// args: --verbose --config config.json
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,

    /// Build commands executed during `dora build`. Each line runs separately.
    ///
    /// ```yaml
    /// build: |
    ///   pip install requirements.txt
    ///   cargo build --release
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,

    /// Redirect stdout/stderr to a data output.
    ///
    /// ```yaml
    /// send_stdout_as: logs
    /// ```
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,

    /// Input data connections from other nodes.
    ///
    /// ```yaml
    /// inputs:
    ///   image: camera/frame
    ///   config: settings/params
    /// ```
    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,

    /// Output data identifiers produced by this node.
    ///
    /// ```yaml
    /// outputs:
    ///   - processed_image
    ///   - metadata
    /// ```
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
    pub deploy: Option<Deploy>,

    #[serde(flatten)]
    pub kind: CoreNodeKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CoreNodeKind {
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct RuntimeNode {
    /// List of operators running in this runtime
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorDefinition {
    /// Unique operator identifier within the runtime
    pub id: OperatorId,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct SingleOperatorDefinition {
    /// Operator identifier (optional for single operators)
    pub id: Option<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorConfig {
    /// Human-readable operator name
    pub name: Option<String>,
    /// Detailed description of the operator
    pub description: Option<String>,

    /// Input data connections
    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,
    /// Output data identifiers
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,

    /// Operator source configuration (Python, shared library, etc.)
    #[serde(flatten)]
    pub source: OperatorSource,

    /// Build commands for this operator
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// Redirect stdout to data output
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
#[serde(from = "PythonSourceDef", into = "PythonSourceDef")]
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
    pub path: String,
    pub source: NodeSource,
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
pub enum NodeSource {
    Local,
    GitBranch {
        repo: String,
        rev: Option<GitRepoRev>,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum ResolvedNodeSource {
    Local,
    GitCommit { repo: String, commit_hash: String },
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum GitRepoRev {
    Branch(String),
    Tag(String),
    Rev(String),
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum EnvValue {
    #[serde(deserialize_with = "with_expand_envs")]
    Bool(bool),
    #[serde(deserialize_with = "with_expand_envs")]
    Integer(i64),
    #[serde(deserialize_with = "with_expand_envs")]
    Float(f64),
    #[serde(deserialize_with = "with_expand_envs")]
    String(String),
}

impl fmt::Display for EnvValue {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        match self {
            EnvValue::Bool(bool) => fmt.write_str(&bool.to_string()),
            EnvValue::Integer(i64) => fmt.write_str(&i64.to_string()),
            EnvValue::Float(f64) => fmt.write_str(&f64.to_string()),
            EnvValue::String(str) => fmt.write_str(str),
        }
    }
}
