#![warn(missing_docs)]

use crate::{
    config::{ByteSize, CommunicationConfig, Input, InputMapping, NodeRunConfig},
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

/// Node path value for executing commands directly in the shell.
///
/// When a node's [`path`](Node::path) is set to this value, Dora will execute
/// the command specified in [`args`](Node::args) directly in the system shell
/// rather than running an executable file.
///
/// ## Example
///
/// ```yaml
/// nodes:
///   - id: shell-example
///     path: shell
///     args: echo "Hello from shell node"
/// ```
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

    /// Global Environment variables inherited by all nodes (optional)
    ///
    /// ## Example
    ///
    /// ```yaml
    /// env:
    ///     MY_VAR: "my_var"
    ///
    /// nodes:
    ///   - id: foo
    ///     path: path/to/the/executable
    ///     # ... (see below)
    ///   - id: bar
    ///     path: path/to/another/executable
    ///     # ... (see below)
    /// ```
    ///
    /// Note that, If there is an env at the node level, Node level env will have more priority than the global env
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub env: Option<BTreeMap<String, EnvValue>>,

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

/// Specifies when a node should be restarted.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum RestartPolicy {
    /// Never restart the node (default)
    #[default]
    Never,
    /// Restart the node if it exits with a non-zero exit code.
    OnFailure,
    /// Always restart the node when it exits, regardless of exit code.
    ///
    /// The node will not be restarted on the following conditions:
    ///
    /// - The node was stopped by the user (e.g., via `dora stop`).
    /// - All inputs to the node have been closed and the node finished with a non-zero exit code.
    Always,
}

/// Deployment configuration for targeting specific machines in distributed dataflows.
///
/// This struct is part of the unstable deployment configuration, prefixed with
/// `_unstable_deploy` in YAML files. It allows specifying which machine a node
/// should run on in a multi-machine setup.
///
/// ## YAML Example
///
/// ```yaml
/// _unstable_deploy:
///   machine: "robot-arm-controller"
///   working_dir: "/home/dora/projects"
/// ```
///
/// ## Stability
///
/// ⚠️ **Unstable**: This API may change in future versions.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Deploy {
    /// Target machine identifier for deployment.
    ///
    /// Must match one of the daemon machine IDs in the distributed setup.
    pub machine: Option<String>,
    /// Working directory on the target machine.
    ///
    /// If not specified, defaults to the daemon's working directory.
    pub working_dir: Option<PathBuf>,
}

/// Debug and development options for dataflow execution.
///
/// This struct is part of the unstable debug configuration, prefixed with
/// `_unstable_debug` in YAML files. It provides options useful for
/// development and troubleshooting.
///
/// ## YAML Example
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
///
/// ## Stability
///
/// ⚠️ **Unstable**: This API may change in future versions.
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
pub struct Debug {
    /// Whether to publish all messages to Zenoh for debugging.
    ///
    /// When enabled, all inter-node messages are also published to the
    /// Zenoh network, allowing external tools to monitor dataflow activity.
    /// This is useful for debugging but adds overhead.
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

    /// Legacy node configuration.
    #[deprecated(
        since = "0.3.5",
        note = "Use top-level `path`, `args`, etc fields instead"
    )]
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

    /// Whether this node should be restarted on exit or error.
    ///
    /// Defaults to `RestartPolicy::Never`.
    #[serde(default)]
    pub restart_policy: RestartPolicy,

    /// Size of the zenoh shared memory pool for zero-copy output publishing.
    ///
    /// Accepts an integer (raw bytes) or a string with a unit suffix
    /// (`KB`, `MB`, `GB`, case-insensitive). Defaults to 8 MB if not set.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: camera-node
    ///     shared_memory_pool_size: 128MB
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shared_memory_pool_size: Option<ByteSize>,

    /// Unstable machine deployment configuration
    #[schemars(skip)]
    #[serde(rename = "_unstable_deploy")]
    pub deploy: Option<Deploy>,
}

/// A fully resolved node with all aliases expanded and defaults applied.
///
/// This type represents a node after the [`Descriptor`] has been processed
/// by [`resolve_aliases_and_set_defaults`](crate::descriptor::DescriptorExt::resolve_aliases_and_set_defaults).
/// It contains the complete configuration ready for execution.
///
/// Unlike [`Node`], which may contain shortcuts and aliases, `ResolvedNode`
/// has all fields fully expanded.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedNode {
    /// Unique node identifier.
    ///
    /// Must not contain `/` characters.
    pub id: NodeId,
    /// Human-readable node name (if specified).
    pub name: Option<String>,
    /// Detailed description of the node's functionality (if specified).
    pub description: Option<String>,
    /// Environment variables for the node.
    ///
    /// Merged from global and node-level environment variables,
    /// with node-level taking precedence.
    pub env: Option<BTreeMap<String, EnvValue>>,

    /// Deployment configuration (if specified).
    #[serde(default)]
    pub deploy: Option<Deploy>,

    /// The kind of this node, determining its execution model.
    #[serde(flatten)]
    pub kind: CoreNodeKind,
}

impl ResolvedNode {
    /// Returns `true` if this node has a git source.
    ///
    /// This is useful for determining whether the node's source code
    /// needs to be cloned from a repository before execution.
    pub fn has_git_source(&self) -> bool {
        self.kind
            .as_custom()
            .map(|n| n.source.is_git())
            .unwrap_or_default()
    }
}

/// The execution model for a resolved node.
///
/// Determines how the node's code is executed:
/// - [`Runtime`](CoreNodeKind::Runtime): Operators running in a shared runtime process
/// - [`Custom`](CoreNodeKind::Custom): A standalone custom node process
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
#[allow(clippy::large_enum_variant)]
pub enum CoreNodeKind {
    /// One or more operators running in a shared runtime process.
    ///
    /// Operators share an address space, allowing efficient communication
    /// between them. Serialized as `"operators"` in YAML.
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    /// A standalone custom node running as its own process.
    ///
    /// Custom nodes are isolated from other nodes and can be
    /// any executable (Rust binary, Python script, etc.).
    Custom(CustomNode),
}

impl CoreNodeKind {
    /// Returns a reference to the [`CustomNode`] if this is a custom node.
    ///
    /// Returns `None` if this is a runtime node.
    pub fn as_custom(&self) -> Option<&CustomNode> {
        match self {
            CoreNodeKind::Runtime(_) => None,
            CoreNodeKind::Custom(custom_node) => Some(custom_node),
        }
    }
}

/// A runtime node containing one or more operators.
///
/// Runtime nodes allow multiple operators to run in a single process,
/// sharing memory and reducing inter-process communication overhead.
///
/// ## YAML Example
///
/// ```yaml
/// nodes:
///   - id: my-runtime
///     operators:
///       - id: processor
///         python: process.py
///       - id: filter
///         python: filter.py
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct RuntimeNode {
    /// List of operator definitions within this runtime.
    pub operators: Vec<OperatorDefinition>,
}

/// A complete operator definition within a runtime node.
///
/// Operators are lightweight alternatives to full nodes, running within
/// a shared runtime process. They are ideal for simple transformations
/// or when multiple processing steps need tight coupling.
///
/// ## YAML Example
///
/// ```yaml
/// operators:
///   - id: image-processor
///     python: process.py
///     inputs:
///       image: camera/image
///     outputs:
///       - processed
/// ```
#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorDefinition {
    /// Unique identifier for this operator within the runtime.
    pub id: OperatorId,
    /// The operator's complete configuration.
    #[serde(flatten)]
    pub config: OperatorConfig,
}

/// Configuration for a runtime node with a single operator.
///
/// This is a convenience type for the common case of defining a runtime
/// node with only one operator. It allows omitting the operator ID since
/// there's only one operator in the runtime.
///
/// ## YAML Example
///
/// ```yaml
/// nodes:
///   - id: single-op-node
///     operator:
///       id: processor
///       python: process.py
/// ```
#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct SingleOperatorDefinition {
    /// Operator identifier (optional for single operators).
    ///
    /// If not specified, defaults to `"op"`.
    pub id: Option<OperatorId>,
    /// The operator's complete configuration.
    #[serde(flatten)]
    pub config: OperatorConfig,
}

/// Configuration for an operator within a runtime node.
///
/// Defines the operator's source, inputs, outputs, and build settings.
/// Similar to [`Node`] but simplified for the runtime context.
///
/// ## YAML Example
///
/// ```yaml
/// operators:
///   - id: processor
///     name: "Image Processor"
///     python: process.py
///     inputs:
///       image: camera/image
///     outputs:
///       - result
///     build: pip install -r requirements.txt
/// ```
#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorConfig {
    /// Human-readable operator name for documentation.
    pub name: Option<String>,
    /// Detailed description of the operator's functionality.
    pub description: Option<String>,

    /// Input data connections from other nodes or operators.
    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,
    /// Output data identifiers produced by this operator.
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,

    /// Operator source configuration (Python script or shared library).
    #[serde(flatten)]
    pub source: OperatorSource,

    /// Build commands executed during `dora build`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// Redirect stdout to a data output.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,
}

/// The source type for an operator's implementation.
///
/// Operators can be implemented as either:
/// - A Python script (recommended for rapid development)
/// - A compiled shared library (for performance-critical code)
#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
#[serde(rename_all = "kebab-case")]
pub enum OperatorSource {
    /// A compiled shared library (.so, .dll, .dylib).
    ///
    /// The path points to the shared library file. Dora will automatically
    /// load it as a dynamic library.
    SharedLibrary(String),
    /// A Python script or module.
    Python(PythonSource),
}
/// Configuration for a Python-based operator.
///
/// Specifies the Python source file and optional conda environment.
///
/// ## YAML Examples
///
/// Simple form (just the path):
/// ```yaml
/// python: process.py
/// ```
///
/// With options:
/// ```yaml
/// python:
///   source: process.py
///   conda_env: my-env
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(from = "PythonSourceDef", into = "PythonSourceDef")]
pub struct PythonSource {
    /// Path to the Python source file.
    pub source: String,
    /// Optional conda environment name.
    ///
    /// If specified, Dora will activate this conda environment
    /// before running the Python script.
    pub conda_env: Option<String>,
}

/// Internal representation for Python source configuration.
///
/// This enum is used for serde serialization/deserialization and allows
/// the Python source to be specified as either a simple string or
/// an object with options.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum PythonSourceDef {
    /// Simple form: just the source path as a string.
    SourceOnly(String),
    /// Extended form: an object with source and optional conda_env.
    WithOptions {
        /// Path to the Python source file.
        source: String,
        /// Optional conda environment name.
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

/// Configuration for a Python operator (legacy format).
///
/// This struct is used for Python operators in the legacy configuration
/// format. For new configurations, use [`OperatorConfig`] instead.
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct PythonOperatorConfig {
    /// Path to the Python script.
    pub path: PathBuf,
    /// Input data connections.
    #[serde(default)]
    pub inputs: BTreeMap<DataId, InputMapping>,
    /// Output data identifiers.
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

/// A custom node running as its own process.
///
/// Custom nodes are standalone executables or scripts that communicate
/// with other nodes through inputs and outputs. They can be written
/// in any language (Rust, Python, C++, etc.).
///
/// This type represents the resolved form of a custom node configuration.
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
    /// Source type for the custom node (local or git).
    pub source: NodeSource,
    /// Args for the executable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,
    /// Environment variables for the custom nodes.
    #[deprecated(note = "Use the outer-level `env` field on `Node` instead")]
    pub envs: Option<BTreeMap<String, EnvValue>>,
    /// Build commands executed during `dora build`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// Send stdout and stderr to another node
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,

    /// Restart policy for this node.
    #[serde(default)]
    pub restart_policy: RestartPolicy,

    /// Input and output configuration for this node.
    #[serde(flatten)]
    pub run_config: NodeRunConfig,
}

/// The source location for a custom node's code.
///
/// Specifies where the node's source code comes from:
/// - A local file or directory
/// - A git repository with optional branch/tag/revision
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum NodeSource {
    /// Local file or directory.
    Local,
    /// Git repository with optional revision specification.
    GitBranch {
        /// Git repository URL.
        repo: String,
        /// Optional revision (branch, tag, or commit hash).
        rev: Option<GitRepoRev>,
    },
}

impl NodeSource {
    /// Returns `true` if this source is a git repository.
    pub fn is_git(&self) -> bool {
        matches!(self, Self::GitBranch { .. })
    }
}

/// The resolved source location for a custom node's code.
///
/// This type represents a node source after git resolution, where
/// branch/tag names have been converted to specific commit hashes.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum ResolvedNodeSource {
    /// Local file or directory.
    Local,
    /// Git repository with resolved commit hash.
    GitCommit {
        /// Git repository URL.
        repo: String,
        /// Resolved commit hash.
        commit_hash: String,
    },
}

/// A specific git revision specification.
///
/// Can be one of:
/// - A branch name
/// - A tag name
/// - A specific commit hash
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum GitRepoRev {
    /// Git branch name.
    Branch(String),
    /// Git tag name.
    Tag(String),
    /// Specific commit hash.
    Rev(String),
}

/// A value for environment variables.
///
/// Supports multiple types to allow flexible environment variable configuration:
/// - Boolean values
/// - Integer values
/// - Floating-point values
/// - String values
///
/// Values are automatically expanded from environment variable references
/// (e.g., `$HOME` or `${USER}`) during deserialization.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum EnvValue {
    /// Boolean value.
    #[serde(deserialize_with = "with_expand_envs")]
    Bool(bool),
    /// Integer value.
    #[serde(deserialize_with = "with_expand_envs")]
    Integer(i64),
    /// Floating-point value.
    #[serde(deserialize_with = "with_expand_envs")]
    Float(f64),
    /// String value.
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
