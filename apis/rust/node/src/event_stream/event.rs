use adora_arrow_convert::ArrowData;
use adora_core::config::{DataId, NodeId, OperatorId};
use adora_message::metadata::Metadata;

/// Represents an incoming Adora event.
///
/// Events might be triggered by other nodes, by Adora itself, or by some external user input.
///
/// It's safe to ignore event types that are not relevant to the node.
///
/// This enum is marked as `non_exhaustive` because we might add additional
/// variants in the future. Please ignore unknown event types instead of throwing an
/// error to avoid breakage when updating Adora.
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
    /// A previously closed input has recovered and will receive data again.
    ///
    /// This happens when an upstream node that timed out (via `input_timeout`)
    /// starts producing data again. The circuit breaker automatically re-opens
    /// the input.
    InputRecovered {
        /// The ID of the recovered input, as specified in the YAML file.
        id: DataId,
    },
    /// An upstream node has restarted.
    ///
    /// Sent to downstream nodes when a node with a restart policy successfully
    /// restarts after a failure. Nodes can use this to reset state, clear caches,
    /// or log the recovery.
    NodeRestarted {
        /// The ID of the upstream node that restarted.
        id: NodeId,
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
    /// started by a `adora runtime` process. So this event should not be sent to normal
    /// nodes yet.
    Reload {
        /// The ID of the operator that should be reloaded.
        ///
        /// There is currently no case where `operator_id` is `None`.
        operator_id: Option<OperatorId>,
    },
    /// A runtime parameter has been updated via `adora param set`.
    ///
    /// Nodes can use this to dynamically adjust behavior (e.g., thresholds,
    /// rates) without restarting.
    ParamUpdate {
        /// The parameter key that was set.
        key: String,
        /// The new JSON value.
        value: serde_json::Value,
    },
    /// Notifies the node about an unexpected error that happened inside Adora.
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
    /// The dataflow is stopped early after a `adora stop` command (or on `ctrl-c`).
    ///
    /// Nodes should exit as soon as possible if they receive a stop event of
    /// this type. Adora will kill nodes that keep running for too long after
    /// receiving such a stop event.
    Manual,
    /// The event stream is closed because all of the node's inputs were closed.
    ///
    /// This stop event type is only sent for nodes that have at least one input.
    AllInputsClosed,
}
