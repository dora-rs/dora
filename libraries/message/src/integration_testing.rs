//! Use these types for integration testing nodes.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

use semver::Op;

use crate::{
    config::Input,
    descriptor::EnvValue,
    id::{DataId, NodeId},
    metadata::MetadataParameters,
};

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct IntegrationTestInput {
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

    pub events: Vec<TimedInputEvent>,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TimedInputEvent {
    pub time_offset_secs: f64,
    #[serde(flatten)]
    pub event: InputEvent,
}

#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type")]
pub enum InputEvent {
    Stop,
    Input {
        id: DataId,
        metadata: Option<MetadataParameters>,
        #[serde(default, flatten)]
        data: Option<InputData>,
    },
    InputClosed {
        id: DataId,
    },
    AllInputsClosed,
}

#[derive(Debug, PartialEq, Eq, Clone, serde::Deserialize, serde::Serialize)]
#[serde(tag = "data_format")]
pub enum InputData {
    /// Converts the given JSON object to the closest Arrow representation.
    ///
    /// No schema is required, but not all arrow types are representable.
    JsonObject {
        data: serde_json::Value,
        schema: Option<arrow_schema::Schema>,
    },
    /// Use [Arrow JSON test data format](https://github.com/apache/arrow/blob/main/docs/source/format/Integration.rst#json-test-data-format)
    ///
    /// Requires specifying the exact schema. Always returns an Arrow StructArray.
    ArrowTest { data: serde_json::Value },
    /// Use first column of [Arrow JSON test data format](https://github.com/apache/arrow/blob/main/docs/source/format/Integration.rst#json-test-data-format).
    ///
    /// Like [`ArrowTest`][InputDataFormat::ArrowTest], but unwraps the first column of the
    /// `StructArray`. This makes it possible to provide other Arrow array types as input.
    ///
    /// Errors if the given `RecordBatch` specifies more than one column.
    ArrowTestUnwrap { data: serde_json::Value },

    /// Use [Arrow file format](https://arrow.apache.org/docs/python/ipc.html#writing-and-reading-random-access-files)
    ArrowFile {
        path: PathBuf,
        #[serde(default)]
        batch_index: usize,
        column: Option<String>,
    },
}
