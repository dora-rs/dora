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

/// Source identifier for shell-based nodes.
pub const SHELL_SOURCE: &str = "shell";
/// Set the [`Node::path`] field to this value to treat the node as a
/// [_dynamic node_](https://docs.rs/adora-node-api/latest/adora_node_api/).
pub const DYNAMIC_SOURCE: &str = "dynamic";

/// # Dataflow Specification
///
/// The main configuration structure for defining a Adora dataflow. Dataflows are
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
///         tick: adora/timer/millis/100
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
#[schemars(title = "adora-rs specification")]
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
    /// For each node, you need to specify the `path` of the executable or script that Adora should run when starting the node.
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

    /// How often the daemon checks node health (in seconds).
    ///
    /// Defaults to 5.0 seconds if not specified. Lower values detect hung nodes
    /// faster but add more overhead.
    #[serde(default)]
    pub health_check_interval: Option<f64>,
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
    /// - The node was stopped by the user (e.g., via `adora stop`).
    /// - All inputs to the node have been closed and the node finished with a non-zero exit code.
    Always,
}

/// Deployment configuration for distributing nodes across machines.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Deploy {
    /// Target machine for deployment
    pub machine: Option<String>,
    /// Working directory for the deployment
    pub working_dir: Option<PathBuf>,
}

/// Debug options for dataflow development and troubleshooting.
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
pub struct Debug {
    /// Whether to publish all messages to Zenoh for debugging
    #[serde(default)]
    pub publish_all_messages_to_zenoh: bool,
}

/// # Adora Node Configuration
///
/// A node represents a computational unit in a Adora dataflow. Each node runs as a
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
    /// Specifies the path of the executable or script that Adora should run when starting the
    /// dataflow.
    /// This can point to a normal executable (e.g. when using a compiled language such as Rust) or
    /// a Python script.
    ///
    /// Adora will automatically append a `.exe` extension on Windows systems when the specified
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
    /// In this case, Adora will download the given file when starting the dataflow.
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

    /// ROS2 bridge configuration (unstable).
    ///
    /// Declares this node as a ROS2 bridge that automatically subscribes to or
    /// publishes on ROS2 topics. No custom code is needed -- the framework spawns
    /// a bridge binary that converts between ROS2 DDS messages and Adora's Arrow
    /// format.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: camera_bridge
    ///     ros2:
    ///       topic: /camera/image_raw
    ///       message_type: sensor_msgs/Image
    ///       direction: subscribe
    ///     outputs:
    ///       - image
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ros2: Option<Ros2BridgeConfig>,

    /// Legacy node configuration (deprecated).
    ///
    /// Please use the top-level [`path`](Self::path), [`args`](Self::args), etc. fields instead.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom: Option<CustomNode>,

    /// Output data identifiers produced by this node.
    ///
    /// List of output identifiers that the node sends.
    /// Must contain all `output_id` values that the node uses when sending output, e.g. through the
    /// [`send_output`](https://docs.rs/adora-node-api/latest/adora_node_api/struct.AdoraNode.html#method.send_output)
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
    ///     [`Event::Input`](https://docs.rs/adora-node-api/latest/adora_node_api/enum.Event.html#variant.Input)
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
    /// This field can be used to send all stdout and stderr output of the node as a Adora output.
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

    /// Redirect structured log entries to a data output as JSON strings.
    ///
    /// Unlike `send_stdout_as` which sends raw stdout lines, this sends only
    /// parsed structured log entries (with level, timestamp, message, fields).
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: sensor
    ///     path: ./sensor
    ///     send_logs_as: logs
    ///     outputs:
    ///       - data
    ///       - logs
    /// ```
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_logs_as: Option<String>,

    /// Minimum log level for this node (error, warn, info, debug, trace, stdout).
    ///
    /// Logs below this level are suppressed from file output, coordinator
    /// forwarding, and `send_logs_as` routing.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: noisy_sensor
    ///     path: ./sensor
    ///     min_log_level: info
    /// ```
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_log_level: Option<String>,

    /// Maximum log file size before rotation (e.g. "50MB", "1GB").
    ///
    /// When the JSONL log file exceeds this size, it is rotated. Old files
    /// are renamed with numeric suffixes (`.1.jsonl`, `.2.jsonl`, etc.) and
    /// the oldest are deleted once 5 rotated files exist.
    ///
    /// ## Example
    ///
    /// ```yaml
    /// nodes:
    ///   - id: sensor
    ///     path: ./sensor
    ///     max_log_size: "100MB"
    /// ```
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_log_size: Option<String>,
    /// Maximum number of rotated log files to keep (default: 5)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_rotated_files: Option<u32>,

    /// Build commands executed during `adora build`. Each line runs separately.
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
    /// If the `--uv` argument is passed to the `adora build` command, all `pip`/`pip3` commands are
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
    /// `adora build --uv`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,

    /// Git repository URL for downloading nodes.
    ///
    /// The `git` key allows downloading nodes (i.e. their source code) from git repositories.
    /// This can be especially useful for distributed dataflows.
    ///
    /// When a `git` key is specified, `adora build` automatically clones the specified repository
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
    ///     git: https://github.com/adora-rs/adora.git
    ///     build: cargo build -p rust-dataflow-example-node
    ///     path: target/debug/rust-dataflow-example-node
    /// ```
    ///
    /// In the above example, `adora build` will first clone the specified `git` repository and then
    /// run the specified `build` inside the local clone directory.
    /// When `adora run` or `adora start` is invoked, the working directory will be the git clone
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
    ///     git: https://github.com/adora-rs/adora.git
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
    ///     git: https://github.com/adora-rs/adora.git
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
    ///     git: https://github.com/adora-rs/adora.git
    ///     rev: 64ab0d7c
    /// ```
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rev: Option<String>,

    /// Whether this node should be restarted on exit or error.
    ///
    /// Defaults to `RestartPolicy::Never`.
    #[serde(default)]
    pub restart_policy: RestartPolicy,

    /// Maximum number of restart attempts. 0 means unlimited.
    ///
    /// When combined with `restart_window`, this limits restarts within the window period.
    /// For example, `max_restarts: 5` with `restart_window: 300` means "5 restarts per 5 minutes".
    #[serde(default)]
    pub max_restarts: u32,

    /// Initial delay in seconds before restarting. Doubles each attempt (exponential backoff).
    ///
    /// For example, with `restart_delay: 1.0`, delays will be 1s, 2s, 4s, 8s, ...
    /// Use `max_restart_delay` to cap the backoff.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub restart_delay: Option<f64>,

    /// Maximum delay in seconds for exponential backoff.
    ///
    /// Caps the exponentially growing `restart_delay`. For example, with
    /// `restart_delay: 1.0` and `max_restart_delay: 30.0`, delays grow as
    /// 1s, 2s, 4s, 8s, 16s, 30s, 30s, ...
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_restart_delay: Option<f64>,

    /// Time window in seconds for counting restarts.
    ///
    /// When set, the restart counter resets after this period of time elapses since the
    /// first restart in the current window. This enables "N restarts within M seconds" semantics.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub restart_window: Option<f64>,

    /// Health check timeout in seconds.
    ///
    /// When set, the daemon monitors this node for activity. If the node does not
    /// communicate with the daemon within this timeout, it is killed and the restart
    /// policy is evaluated.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub health_check_timeout: Option<f64>,

    /// Unstable machine deployment configuration
    #[schemars(skip)]
    #[serde(rename = "_unstable_deploy")]
    pub deploy: Option<Deploy>,
}

#[allow(missing_docs)]
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

#[allow(missing_docs)]
impl ResolvedNode {
    pub fn has_git_source(&self) -> bool {
        self.kind
            .as_custom()
            .map(|n| n.source.is_git())
            .unwrap_or_default()
    }
}

#[allow(missing_docs)]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
#[allow(clippy::large_enum_variant)]
pub enum CoreNodeKind {
    /// Adora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
}

#[allow(missing_docs)]
impl CoreNodeKind {
    pub fn as_custom(&self) -> Option<&CustomNode> {
        match self {
            CoreNodeKind::Runtime(_) => None,
            CoreNodeKind::Custom(custom_node) => Some(custom_node),
        }
    }
}

#[allow(missing_docs)]
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct RuntimeNode {
    /// List of operators running in this runtime
    pub operators: Vec<OperatorDefinition>,
}

#[allow(missing_docs)]
#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorDefinition {
    /// Unique operator identifier within the runtime
    pub id: OperatorId,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[allow(missing_docs)]
#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct SingleOperatorDefinition {
    /// Operator identifier (optional for single operators)
    pub id: Option<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[allow(missing_docs)]
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
    /// Redirect structured log entries to a data output as JSON strings
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_logs_as: Option<String>,
    /// Minimum log level for this operator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_log_level: Option<String>,
    /// Maximum log file size before rotation (e.g. "50MB", "1GB")
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_log_size: Option<String>,
    /// Maximum number of rotated log files to keep (default: 5)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_rotated_files: Option<u32>,
}

#[allow(missing_docs)]
#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
#[serde(rename_all = "kebab-case")]
pub enum OperatorSource {
    SharedLibrary(String),
    Python(PythonSource),
    #[schemars(skip)]
    Wasm(String),
}
#[allow(missing_docs)]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(from = "PythonSourceDef", into = "PythonSourceDef")]
pub struct PythonSource {
    pub source: String,
    pub conda_env: Option<String>,
}

#[allow(missing_docs)]
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

#[allow(missing_docs)]
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct PythonOperatorConfig {
    pub path: PathBuf,
    #[serde(default)]
    pub inputs: BTreeMap<DataId, InputMapping>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

#[allow(missing_docs)]
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
    /// Redirect structured log entries to a data output as JSON strings
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_logs_as: Option<String>,
    /// Minimum log level for this node
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_log_level: Option<String>,
    /// Maximum log file size before rotation (e.g. "50MB", "1GB")
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_log_size: Option<String>,
    /// Maximum number of rotated log files to keep (default: 5)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_rotated_files: Option<u32>,

    #[serde(default)]
    pub restart_policy: RestartPolicy,

    /// Maximum number of restart attempts. 0 means unlimited.
    #[serde(default)]
    pub max_restarts: u32,

    /// Initial delay in seconds before restarting (exponential backoff).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub restart_delay: Option<f64>,

    /// Maximum delay in seconds for exponential backoff.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_restart_delay: Option<f64>,

    /// Time window in seconds for counting restarts.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub restart_window: Option<f64>,

    /// Health check timeout in seconds.
    ///
    /// When set, the daemon monitors this node for activity. If the node does not
    /// communicate with the daemon within this timeout, it is killed and the restart
    /// policy is evaluated.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub health_check_timeout: Option<f64>,

    #[serde(flatten)]
    pub run_config: NodeRunConfig,
}

#[allow(missing_docs)]
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum NodeSource {
    Local,
    GitBranch {
        repo: String,
        rev: Option<GitRepoRev>,
    },
}

#[allow(missing_docs)]
impl NodeSource {
    pub fn is_git(&self) -> bool {
        matches!(self, Self::GitBranch { .. })
    }
}

#[allow(missing_docs)]
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum ResolvedNodeSource {
    Local,
    GitCommit { repo: String, commit_hash: String },
}

#[allow(missing_docs)]
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub enum GitRepoRev {
    Branch(String),
    Tag(String),
    Rev(String),
}

#[allow(missing_docs)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
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

/// ROS2 bridge configuration for declarative ROS2 bridging.
///
/// This allows nodes to interact with ROS2 topics, services, and actions
/// without writing any custom code. The framework spawns a bridge binary that
/// handles the ROS2 DDS communication and Arrow data conversion.
///
/// Exactly one of `topic`, `topics`, `service`, or `action` must be set.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Ros2BridgeConfig {
    /// ROS2 topic name (e.g. "/camera/image_raw").
    /// Mutually exclusive with `topics`, `service`, `action`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub topic: Option<String>,

    /// ROS2 message type (e.g. "sensor_msgs/Image").
    /// Required when `topic` is set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub message_type: Option<String>,

    /// Direction: subscribe (ROS2 -> Adora) or publish (Adora -> ROS2).
    /// Defaults to subscribe. Only used with `topic`/`topics`.
    #[serde(default)]
    pub direction: Ros2Direction,

    /// Multiple topics on a single ROS2 node context.
    /// Mutually exclusive with `topic`, `service`, `action`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub topics: Option<Vec<Ros2TopicConfig>>,

    /// ROS2 service name (e.g. "/add_two_ints").
    /// Mutually exclusive with `topic`, `topics`, `action`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub service: Option<String>,

    /// ROS2 service type (e.g. "example_interfaces/AddTwoInts").
    /// Required when `service` is set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub service_type: Option<String>,

    /// ROS2 action name (e.g. "/navigate").
    /// Mutually exclusive with `topic`, `topics`, `service`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action: Option<String>,

    /// ROS2 action type (e.g. "nav2_msgs/NavigateToPose").
    /// Required when `action` is set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub action_type: Option<String>,

    /// Role: client or server. Required for `service` and `action`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<Ros2Role>,

    /// QoS policies applied to all topics (can be overridden per-topic).
    #[serde(default)]
    pub qos: Ros2QosConfig,

    /// ROS2 namespace (default: "/").
    #[serde(default = "default_ros2_namespace")]
    pub namespace: String,

    /// ROS2 node name. Defaults to the adora node id.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_name: Option<String>,
}

impl Default for Ros2BridgeConfig {
    fn default() -> Self {
        Self {
            topic: None,
            message_type: None,
            direction: Ros2Direction::default(),
            topics: None,
            service: None,
            service_type: None,
            action: None,
            action_type: None,
            role: None,
            qos: Ros2QosConfig::default(),
            namespace: default_ros2_namespace(),
            node_name: None,
        }
    }
}

/// Role of a ROS2 service or action bridge node.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Ros2Role {
    /// Client: sends requests/goals, receives responses/results.
    Client,
    /// Server: receives requests, sends responses.
    Server,
}

fn default_ros2_namespace() -> String {
    "/".to_string()
}

/// Configuration for a single ROS2 topic in multi-topic mode.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Ros2TopicConfig {
    /// ROS2 topic name.
    pub topic: String,

    /// ROS2 message type (e.g. "geometry_msgs/Twist").
    pub message_type: String,

    /// Direction: subscribe or publish.
    #[serde(default)]
    pub direction: Ros2Direction,

    /// Maps to an adora output id (for subscribe direction).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub output: Option<String>,

    /// Maps to an adora input id (for publish direction).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub input: Option<String>,

    /// Per-topic QoS override.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<Ros2QosConfig>,
}

/// Direction of ROS2 bridge communication.
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Ros2Direction {
    /// Subscribe: receive from ROS2, forward to adora outputs.
    #[default]
    Subscribe,
    /// Publish: receive from adora inputs, publish to ROS2.
    Publish,
}

/// ROS2 Quality of Service configuration.
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Ros2QosConfig {
    /// Use reliable transport (default: false = best effort).
    #[serde(default)]
    pub reliable: bool,

    /// Durability: "volatile" (default), "transient_local".
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub durability: Option<String>,

    /// Liveliness: "automatic" (default), "manual_by_participant", "manual_by_topic".
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub liveliness: Option<String>,

    /// Lease duration in seconds (default: infinity).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lease_duration: Option<f64>,

    /// Max blocking time in seconds for reliable transport.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_blocking_time: Option<f64>,

    /// History depth for KeepLast policy (default: 1).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_last: Option<i32>,

    /// Use KeepAll history policy instead of KeepLast.
    #[serde(default)]
    pub keep_all: bool,
}
