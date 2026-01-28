use dora_arrow_convert::ArrowData;
use dora_core::config::{DataId, NodeId, OperatorId};
use dora_message::metadata::Metadata;

/// Represents an incoming Dora event.
///
/// Events might be triggered by other nodes, by Dora itself, or by some external user input.
///
/// It's safe to ignore event types that are not relevant to the node.
///
/// This enum is marked as `non_exhaustive` because we might add additional
/// variants in the future. Please ignore unknown event types instead of throwing an
/// error to avoid breakage when updating Dora.
#[derive(Debug)]
#[non_exhaustive]
#[allow(clippy::large_enum_variant)]
pub enum Event {
    /// An input was received from another node.
    ///
    /// This event corresponds to one of the `inputs` of the node as specified
    /// in the dataflow YAML file.
    Input {
        /// The input ID, as specified in the YAML file.
        ///
        /// Note that this is not the output ID of the sender, but the ID
        /// assigned to the input in the YAML file.
        id: DataId,
        /// Meta information about this input, e.g. the timestamp.
        metadata: Metadata,
        /// The actual data in the Apache Arrow data format.
        data: ArrowData,
    },
    /// An input was closed by the sender.
    ///
    /// The sending node mapped to an input exited, so this input will receive
    /// no more data.
    InputClosed {
        /// The ID of the input that was closed, as specified in the YAML file.
        ///
        /// Note that this is not the output ID of the sender, but the ID
        /// assigned to the input in the YAML file.
        id: DataId,
    },
    /// A node failed and exited with a non-zero exit code.
    ///
    /// The daemon automatically creates this event when a node exits with a non-zero exit code.
    /// This allows downstream nodes to handle the error gracefully (e.g., use cached data,
    /// switch to backup source, log and continue).
    NodeFailed {
        /// The IDs of the inputs that are affected by the node failure, as specified in the YAML file.
        ///
        /// A node failure can affect multiple inputs if the failed node produced multiple outputs
        /// that are consumed by this node.
        affected_input_ids: Vec<DataId>,
        /// The error message describing the failure.
        error: String,
        /// The ID of the node that failed.
        source_node_id: NodeId,
    },
    /// Notification that the event stream is about to close.
    ///
    /// The [`StopCause`] field contains the reason for the event stream closure.
    ///
    /// Nodes should exit once the event stream closes.
    Stop(StopCause),
    /// Instructs the node to reload itself or one of its operators.
    ///
    /// This event is currently only used for reloading Python operators that are
    /// started by a `dora runtime` process. So this event should not be sent to normal
    /// nodes yet.
    Reload {
        /// The ID of the operator that should be reloaded.
        ///
        /// There is currently no case where `operator_id` is `None`.
        operator_id: Option<OperatorId>,
    },
    /// Notifies the node about an unexpected error that happened inside Dora.
    ///
    /// It's a good idea to output or log this error for debugging.
    Error(String),
}

/// The reason for closing the event stream.
///
/// This enum is marked as `non_exhaustive` because we might add additional
/// variants in the future.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum StopCause {
    /// The dataflow is stopped early after a `dora stop` command (or on `ctrl-c`).
    ///
    /// Nodes should exit as soon as possible if they receive a stop event of
    /// this type. Dora will kill nodes that keep running for too long after
    /// receiving such a stop event.
    Manual,
    /// The event stream is closed because all of the node's inputs were closed.
    ///
    /// This stop event type is only sent for nodes that have at least one input.
    AllInputsClosed,
}
