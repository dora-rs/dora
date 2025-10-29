#![warn(missing_docs)]

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

/// Source identifier used when compiling operators from shell commands.
pub const SHELL_SOURCE: &str = "shell";
/// Set the [`Node::path`] field to this value to treat the node as a
/// [_dynamic node_](https://docs.rs/dora-node-api/latest/dora_node_api/).
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
/// - **Deployment**: Optional deployment configuration (unstable)
/// - **Debug options**: Optional development and debugging settings (unstable)
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
    /// List of nodes in the dataflow
    ///
    /// This is the most important field of the dataflow specification.
    /// Each node must be identified by a unique `id`:
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: foo
    ///     path: path/to/the/executable
    ///     # ... (see below)
    ///   - id: bar
    ///     path: path/to/another/executable
    ///     # ... (see below)
    /// ```
    ///
    /// For each node, you need to specify the `path` of the executable or script that Dora should run when starting the node.
    /// Most of the other node fields are optional, but you typically want to specify at least some `inputs` and/or `outputs`.
    pub nodes: Vec<Node>,

    /// Communication configuration (optional, uses defaults)
    #[schemars(skip)]
    #[serde(default)]
    pub communication: CommunicationConfig,

    /// Deployment configuration (optional, unstable)
    #[schemars(skip)]
    #[serde(rename = "_unstable_deploy")]
    pub deploy: Option<Deploy>,

    /// Debug options (optional, unstable)
    #[schemars(skip)]
    #[serde(default, rename = "_unstable_debug")]
    pub debug: Debug,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
/// Deployment settings for distributing a node across machines.
pub struct Deploy {
    /// Target machine for deployment
    pub machine: Option<String>,
    /// Working directory for the deployment
    pub working_dir: Option<PathBuf>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
/// Debug configuration that controls developer-centric features.
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
    /// Node IDs can be arbitrary strings with the following limitations:
    ///
    /// - They must not contain any `/` characters (slashes).
    /// - We do not recommend using whitespace characters (e.g. spaces) in IDs
    ///
    /// Each node must have an ID field.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: camera_node
    ///   - id: some_other_node
    /// ```
    pub id: NodeId,

    /// Human-readable node name for documentation.
    ///
    /// This optional field can be used to define a more descriptive name in addition to a short
    /// [`id`](Self::id).
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: camera_node
    ///     name: "Camera Input Handler"
    pub name: Option<String>,

    /// Detailed description of the node's functionality.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: camera_node
    ///     description: "Captures video frames from webcam"
    /// ```
    pub description: Option<String>,

    /// Path to executable or script that should be run.
    ///
    /// Specifies the path of the executable or script that Dora should run when starting the
    /// dataflow.
    /// This can point to a normal executable (e.g. when using a compiled language such as Rust) or
    /// a Python script.
    ///
    /// Dora will automatically append a `.exe` extension on Windows systems when the specified
    /// file name has no extension.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: rust-example
    ///     path: target/release/rust-node
    ///   - id: python-example
    ///     path: ./receive_data.py
    /// ```
    ///
    /// ## URL as Path
    ///
    /// The `path` field can also point to a URL instead of a local path.
    /// In this case, Dora will download the given file when starting the dataflow.
    ///
    /// Note that this is quite an old feature and using this functionality is **not recommended**
    /// anymore. Instead, we recommend using a [`git`][Self::git] and/or [`build`](Self::build)
    /// key.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,

    /// Command-line arguments passed to the executable.
    ///
    /// The command-line arguments that should be passed to the executable/script specified in `path`.
    /// The arguments should be separated by space.
    /// This field is optional and defaults to an empty argument list.
    ///
    /// ## Example
    /// ```yaml
    /// nodes:
    ///   - id: example
    ///     path: example-node
    ///     args: -v --some-flag foo
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,

    /// Environment variables for node builds and execution.
    ///
    /// Key-value map of environment variables that should be set for both the
    /// [`build`](Self::build) operation and the node execution (i.e. when the node is spawned
    /// through [`path`](Self::path)).
    ///
    /// Supports strings, numbers, and booleans.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: example-node
    ///     path: path/to/node
    ///     env:
    ///       DEBUG: true
    ///       PORT: 8080
    ///       API_KEY: "secret-key"
    /// ```
    pub env: Option<BTreeMap<String, EnvValue>>,

    /// Multiple operators running in a shared runtime process.
    ///
    /// Operators are an experimental, lightweight alternative to nodes.
    /// Instead of running as a separate process, operators are linked into a runtime process.
    /// This allows running multiple operators to share a single address space (not supported for
    /// Python currently).
    ///
    /// Operators are defined as part of the node list, as children of a runtime node.
    /// A runtime node is a special node that specifies no [`path`](Self::path) field, but contains
    /// an `operators` field instead.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: runtime-node
    ///     operators:
    ///       - id: processor
    ///         python: process.py
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operators: Option<RuntimeNode>,

    /// Single operator configuration.
    ///
    /// This is a convenience field for defining runtime nodes that contain only a single operator.
    /// This field is an alternative to the [`operators`](Self::operators) field, which can be used
    /// if there is only a single operator defined for the runtime node.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: runtime-node
    ///     operator:
    ///       id: processor
    ///       python: script.py
    ///       outputs: [data]
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operator: Option<SingleOperatorDefinition>,

    /// Legacy node configuration (deprecated).
    ///
    /// Please use the top-level [`path`](Self::path), [`args`](Self::args), etc. fields instead.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom: Option<CustomNode>,

    /// Output data identifiers produced by this node.
    ///
    /// List of output identifiers that the node sends.
    /// Must contain all `output_id` values that the node uses when sending output, e.g. through the
    /// [`send_output`](https://docs.rs/dora-node-api/latest/dora_node_api/struct.DoraNode.html#method.send_output)
    /// function.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: example-node
    ///     outputs:
    ///       - processed_image
    ///       - metadata
    /// ```
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,

    /// Input data connections from other nodes.
    ///
    /// Defines the inputs that this node is subscribing to.
    ///
    /// The `inputs` field should be a key-value map of the following format:
    ///
    /// `input_id: source_node_id/source_node_output_id`
    ///
    /// The components are defined as follows:
    ///
    ///   - `input_id` is the local identifier that should be used for this input.
    ///
    ///     This will map to the `id` field of
    ///     [`Event::Input`](https://docs.rs/dora-node-api/latest/dora_node_api/enum.Event.html#variant.Input)
    ///     events sent to the node event loop.
    ///   - `source_node_id` should be the `id` field of the node that sends the output that we want
    ///     to subscribe to
    ///   - `source_node_output_id` should be the identifier of the output that that we want
    ///     to subscribe to
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: example-node
    ///     outputs:
    ///       - one
    ///       - two
    ///   - id: receiver
    ///     inputs:
    ///         my_input: example-node/two
    /// ```
    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,

    /// Redirect stdout/stderr to a data output.
    ///
    /// This field can be used to send all stdout and stderr output of the node as a Dora output.
    /// Each output line is sent as a separate message.
    ///
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: example
    ///     send_stdout_as: stdout_output
    ///   - id: logger
    ///     inputs:
    ///         example_output: example/stdout_output
    /// ```
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,

    /// Build commands executed during `dora build`. Each line runs separately.
    ///
    /// The `build` key specifies the command that should be invoked for building the node.
    /// The key expects a single- or multi-line string.
    ///
    /// Each line is run as a separate command.
    /// Spaces are used to separate arguments.
    ///
    /// Note that all the environment variables specified in the [`env`](Self::env) field are also
    /// applied to the build commands.
    ///
    /// ## Special treatment of `pip`
    ///
    /// Build lines that start with `pip` or `pip3` are treated in a special way:
    /// If the `--uv` argument is passed to the `dora build` command, all `pip`/`pip3` commands are
    /// run through the [`uv` package manager](https://docs.astral.sh/uv/).
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    /// - id: build-example
    ///   build: cargo build -p receive_data --release
    ///   path: target/release/receive_data
    /// - id: multi-line-example
    ///   build: |
    ///       pip install requirements.txt
    ///       pip install -e some/local/package
    ///   path: package
    /// ```
    ///
    /// In the above example, the `pip` commands will be replaced by `uv pip` when run through
    /// `dora build --uv`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,

    /// Git repository URL for downloading nodes.
    ///
    /// The `git` key allows downloading nodes (i.e. their source code) from git repositories.
    /// This can be especially useful for distributed dataflows.
    ///
    /// When a `git` key is specified, `dora build` automatically clones the specified repository
    /// (or reuse an existing clone).
    /// Then it checks out the specified [`branch`](Self::branch), [`tag`](Self::tag), or
    /// [`rev`](Self::rev), or the default branch if none of them are specified.
    /// Afterwards it runs the [`build`](Self::build) command if specified.
    ///
    /// Note that the git clone directory is set as working directory for both the
    /// [`build`](Self::build) command and the specified [`path`](Self::path).
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: rust-node
    ///     git: https://github.com/dora-rs/dora.git
    ///     build: cargo build -p rust-dataflow-example-node
    ///     path: target/debug/rust-dataflow-example-node
    /// ```
    ///
    /// In the above example, `dora build` will first clone the specified `git` repository and then
    /// run the specified `build` inside the local clone directory.
    /// When `dora run` or `dora start` is invoked, the working directory will be the git clone
    /// directory too. So a relative `path` will start from the clone directory.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub git: Option<String>,

    /// Git branch to checkout after cloning.
    ///
    /// The `branch` field is only allowed in combination with the [`git`](#git) field.
    /// It specifies the branch that should be checked out after cloning.
    /// Only one of `branch`, `tag`, or `rev` can be specified.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: rust-node
    ///     git: https://github.com/dora-rs/dora.git
    ///     branch: some-branch-name
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub branch: Option<String>,

    /// Git tag to checkout after cloning.
    ///
    /// The `tag` field is only allowed in combination with the [`git`](#git) field.
    /// It specifies the git tag that should be checked out after cloning.
    /// Only one of `branch`, `tag`, or `rev` can be specified.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: rust-node
    ///     git: https://github.com/dora-rs/dora.git
    ///     tag: v0.3.0
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tag: Option<String>,

    /// Git revision (e.g. commit hash) to checkout after cloning.
    ///
    /// The `rev` field is only allowed in combination with the [`git`](#git) field.
    /// It specifies the git revision (e.g. a commit hash) that should be checked out after cloning.
    /// Only one of `branch`, `tag`, or `rev` can be specified.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: rust-node
    ///     git: https://github.com/dora-rs/dora.git
    ///     rev: 64ab0d7c
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rev: Option<String>,

    /// Unstable machine deployment configuration
    #[schemars(skip)]
    #[serde(rename = "_unstable_deploy")]
    pub deploy: Option<Deploy>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
/// Intermediate representation describing a node after configuration resolution.
pub struct ResolvedNode {
    /// Unique identifier of the node.
    pub id: NodeId,
    /// Optional human readable name.
    pub name: Option<String>,
    /// Optional description of the node's purpose.
    pub description: Option<String>,
    /// Environment variables applied to the node.
    pub env: Option<BTreeMap<String, EnvValue>>,

    /// Deployment overrides determined during resolution.
    #[serde(default)]
    pub deploy: Option<Deploy>,

    /// Concrete node variant (runtime or custom).
    #[serde(flatten)]
    pub kind: CoreNodeKind,
}

impl ResolvedNode {
    /// Return whether the node originates from a Git repository.
    pub fn has_git_source(&self) -> bool {
        self.kind
            .as_custom()
            .map(|n| n.source.is_git())
            .unwrap_or_default()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
#[allow(clippy::large_enum_variant)]
/// Differentiates built-in runtime nodes from user-provided custom nodes.
pub enum CoreNodeKind {
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    /// Custom node definition supplied by the user.
    Custom(CustomNode),
}

impl CoreNodeKind {
    /// Extract a custom node if the variant matches.
    pub fn as_custom(&self) -> Option<&CustomNode> {
        match self {
            CoreNodeKind::Runtime(_) => None,
            CoreNodeKind::Custom(custom_node) => Some(custom_node),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
/// Wrapper around a list of operator definitions for runtime nodes.
pub struct RuntimeNode {
    /// List of operators running in this runtime
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
/// Operator entry belonging to a runtime node.
pub struct OperatorDefinition {
    /// Unique operator identifier within the runtime
    pub id: OperatorId,
    /// Configuration of the concrete operator implementation.
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
/// Representation of a single operator dataflow node.
pub struct SingleOperatorDefinition {
    /// Operator identifier (optional for single operators)
    pub id: Option<OperatorId>,
    /// Configuration shared with runtime operator definitions.
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
/// Declarative configuration for an operator implementation.
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
/// Supported origins for operator implementations.
pub enum OperatorSource {
    /// Load operator from a compiled shared library.
    SharedLibrary(String),
    /// Run operator implemented in Python.
    Python(PythonSource),
    #[schemars(skip)]
    /// Execute operator provided as a WebAssembly module.
    Wasm(String),
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(from = "PythonSourceDef", into = "PythonSourceDef")]
/// Describes Python operator script path and optional environment.
pub struct PythonSource {
    /// Path to the Python entry script or module.
    pub source: String,
    /// Optional name of the Conda environment to activate.
    pub conda_env: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
/// Helper enum enabling inline or structured python source definitions.
pub enum PythonSourceDef {
    /// Only the script path is provided.
    SourceOnly(String),
    /// Script path with additional configuration options.
    WithOptions {
        /// Path to the Python entry script or module.
        source: String,
        /// Optional name of the Conda environment to activate.
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
/// Python-specific operator configuration resolved from descriptor.
pub struct PythonOperatorConfig {
    /// Absolute path to the resolved Python script.
    pub path: PathBuf,
    #[serde(default)]
    /// Input mappings used when launching the operator.
    pub inputs: BTreeMap<DataId, InputMapping>,
    #[serde(default)]
    /// Output identifiers produced by the operator.
    pub outputs: BTreeSet<DataId>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Configuration for user-supplied custom nodes.
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
    /// Location from which the node should be fetched.
    pub source: NodeSource,
    /// Args for the executable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,
    /// Environment variables for the custom nodes
    ///
    /// Deprecated, use outer-level `env` field instead.
    pub envs: Option<BTreeMap<String, EnvValue>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    /// Optional build command that prepares the node binary.
    pub build: Option<String>,
    /// Send stdout and stderr to another node
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,

    #[serde(flatten)]
    /// Runtime configuration (environment, restart policy, etc.).
    pub run_config: NodeRunConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Supported origins for custom nodes.
pub enum NodeSource {
    /// Use files available locally.
    Local,
    /// Check out the node from a Git repository.
    GitBranch {
        /// Repository URL hosting the node.
        repo: String,
        /// Optional revision override (branch, tag, or commit).
        rev: Option<GitRepoRev>,
    },
}

impl NodeSource {
    /// Return true when the node is backed by a Git repository.
    pub fn is_git(&self) -> bool {
        matches!(self, Self::GitBranch { .. })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Source details for runtime after resolution.
pub enum ResolvedNodeSource {
    /// Local filesystem source.
    Local,
    /// Resolved Git commit with pinned hash.
    GitCommit {
        /// Repository URL hosting the node.
        repo: String,
        /// Exact commit hash that was resolved.
        commit_hash: String,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
/// Git revision specifiers supported by Dora.
pub enum GitRepoRev {
    /// Refer to a branch name.
    Branch(String),
    /// Refer to a tag name.
    Tag(String),
    /// Refer to an arbitrary revision string.
    Rev(String),
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
/// Environment variable value type used in descriptors.
pub enum EnvValue {
    /// Boolean value interpreted from the descriptor.
    #[serde(deserialize_with = "with_expand_envs")]
    Bool(bool),
    /// Integer literal expanded from descriptor.
    #[serde(deserialize_with = "with_expand_envs")]
    Integer(i64),
    /// Floating point literal expanded from descriptor.
    #[serde(deserialize_with = "with_expand_envs")]
    Float(f64),
    /// Raw string environment value.
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
