//! Use these types for integration testing nodes.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

use crate::{
    config::Input,
    descriptor::EnvValue,
    id::{DataId, NodeId},
    metadata::MetadataParameters,
};

/// Defines the input data and events for integration testing a node.
///
/// Most of the fields are similar to the fields defined in the [`Node`](crate::descriptor::Node)
/// struct, which is used to define nodes in a dataflow YAML file.
///
/// For integration testing, the most important field is the [`events`](Self::events) field, which
/// specifies the events that should be sent to the node during the test.
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

    /// List of incoming events for the integration test.
    ///
    /// The node event stream will yield these events during the test. Once the list is exhausted,
    /// the event stream will close itself.
    pub events: Vec<TimedIncomingEvent>,

    /// Status of the recording that produced this file. Emitted by the
    /// `DORA_WRITE_EVENTS_TO` recorder; `None` for files produced by
    /// older versions of dora (before #1857) or hand-authored fixtures.
    ///
    /// Consumers should refuse to replay (or at minimum warn loudly)
    /// when this is `Some(RecordingStatus::Poisoned { .. })`, because
    /// the `events` array is incomplete relative to the original run.
    ///
    /// Boxed so the (large) `RecordingStatus::Poisoned` variant doesn't
    /// inflate `IntegrationTestInput`'s size for the common-case
    /// `Clean`-or-`None` recording. This also avoids
    /// `large_enum_variant` lints on `TestingInput::Input(...)` (32
    /// call sites; widening that enum would be intrusive).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recording_status: Option<Box<RecordingStatus>>,
}

/// State of a `DORA_WRITE_EVENTS_TO` recording (#1857).
///
/// `Clean` means the recorder observed every event it was asked to
/// record. `Poisoned` means at least one `record_event()` call failed
/// (Arrow→JSON conversion error, file I/O error, etc.) — the `events`
/// array is missing at least one event, and replay results will not
/// match the original run.
#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "state", rename_all = "lowercase")]
pub enum RecordingStatus {
    Clean,
    Poisoned {
        /// Index in the `events` array where the first missed event
        /// would have been. Equals the number of events that WERE
        /// successfully recorded before the first failure.
        first_failure_event_index: usize,
        /// Seconds since the EventStream's start timestamp at the
        /// moment of the first failure.
        first_failure_time_offset_secs: f64,
        /// `format!("{:?}", err)` of the first `record_event()` error.
        first_failure_error: String,
        /// Count of subsequent failures after the first one.
        additional_failures: u64,
    },
}

impl IntegrationTestInput {
    pub fn new(id: NodeId, events: Vec<TimedIncomingEvent>) -> Self {
        Self {
            id,
            name: None,
            description: None,
            args: None,
            env: None,
            outputs: BTreeSet::new(),
            inputs: BTreeMap::new(),
            send_stdout_as: None,
            events,
            recording_status: None,
        }
    }
}

/// An incoming event with a time offset.
#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TimedIncomingEvent {
    /// The time offset in seconds from the start of the node.
    pub time_offset_secs: f64,
    /// The incoming event.
    #[serde(flatten)]
    pub event: IncomingEvent,
}

/// An event that is sent to a node during an integration test.
///
/// This struct is very similar to the `Event` enum used during normal node operation.
#[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type")]
pub enum IncomingEvent {
    Stop,
    Input {
        id: DataId,
        metadata: Option<MetadataParameters>,
        #[serde(flatten)]
        data: Option<Box<InputData>>,
    },
    InputClosed {
        id: DataId,
    },
    AllInputsClosed,
}

/// Represents the data of an incoming input event for integration testing.
#[derive(Debug, PartialEq, Eq, Clone, serde::Deserialize, serde::Serialize)]
#[serde(untagged)]
pub enum InputData {
    /// Converts the given JSON object to the closest Arrow representation.
    ///
    /// An optional data type can be provided to guide the conversion.
    JsonObject {
        /// The input data as JSON.
        ///
        /// This can be a JSON array, object, string, number, boolean, etc. Dora automatically
        /// converts the JSON to an Apache Arrow array, wrapping the data if needed (e.g. wrap
        /// bare integers into an array because Arrow requires all data to be in array form).
        data: serde_json::Value,
        /// Specifies the arrow `DataType` of the `data` field.
        ///
        /// This field is optional. If not set, Dora will try to infer the data type automatically.
        ///
        /// Use this field if the exact data type is important (e.g. to distinguish between
        /// different integer sizes).
        data_type: Option<serde_json::Value>,
    },
    /// Load data from an Arrow IPC file.
    ///
    /// The data must be in the
    /// [Arrow IPC file format](https://arrow.apache.org/docs/python/ipc.html#writing-and-reading-random-access-files)
    ArrowFile {
        /// The path to the Arrow IPC file.
        path: PathBuf,
        /// The optional batch index to read from the file.
        ///
        /// Arrow IPC files can contain multiple record batches. Only one batch is read per input
        /// event. This field specifies which batch to read. Defaults to `0`.
        #[serde(default)]
        batch_index: usize,
        /// Optional column name to read from the record batch.
        ///
        /// If not set, the entire record batch is read and converted to an Arrow `StructArray`.
        column: Option<String>,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pre-#1857 recordings and hand-authored fixtures have no
    /// `recording_status` field. They must continue to deserialize
    /// cleanly with `recording_status: None`, otherwise the new
    /// schema breaks the entire existing fixture corpus.
    #[test]
    fn integration_test_input_without_recording_status_deserializes() {
        let json = r#"{
            "id": "test-node",
            "events": []
        }"#;
        let v: IntegrationTestInput = serde_json::from_str(json).expect("deserialize legacy form");
        assert!(v.recording_status.is_none());
    }

    /// Clean recordings emit `{"recording_status": {"state": "clean"}}`.
    /// Deserialization should land on `Some(RecordingStatus::Clean)`.
    #[test]
    fn integration_test_input_with_clean_recording_status_deserializes() {
        let json = r#"{
            "id": "test-node",
            "events": [],
            "recording_status": { "state": "clean" }
        }"#;
        let v: IntegrationTestInput = serde_json::from_str(json).expect("deserialize clean form");
        assert_eq!(v.recording_status.as_deref(), Some(&RecordingStatus::Clean));
    }

    /// Poisoned recordings carry the failure details; the deserialized
    /// enum variant must preserve every field. The exact JSON shape
    /// here is what `event_stream::WriteEventsTo::write_out` produces
    /// in dora-node-api — pinning both sides of the wire contract.
    #[test]
    fn integration_test_input_with_poisoned_recording_status_deserializes() {
        let json = r#"{
            "id": "test-node",
            "events": [],
            "recording_status": {
                "state": "poisoned",
                "first_failure_event_index": 7,
                "first_failure_time_offset_secs": 1.25,
                "first_failure_error": "Arrow conversion failed: bad type",
                "additional_failures": 3
            }
        }"#;
        let v: IntegrationTestInput =
            serde_json::from_str(json).expect("deserialize poisoned form");
        match v.recording_status.map(|b| *b) {
            Some(RecordingStatus::Poisoned {
                first_failure_event_index,
                first_failure_time_offset_secs,
                first_failure_error,
                additional_failures,
            }) => {
                assert_eq!(first_failure_event_index, 7);
                assert_eq!(first_failure_time_offset_secs, 1.25);
                assert_eq!(first_failure_error, "Arrow conversion failed: bad type");
                assert_eq!(additional_failures, 3);
            }
            other => panic!("expected Poisoned, got {other:?}"),
        }
    }
}
