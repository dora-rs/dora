use dora_arrow_convert::ArrowData;
use dora_core::config::{DataId, OperatorId};
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
    /// Notification that the event stream is about to close.
    ///
    /// The [`StopCause`] field contains the reason for the event stream closure.
    ///
    /// Typically, nodes should exit once the event stream closes. One notable
    /// exception are nodes with no inputs, which will receive a
    /// `Event::Stop(StopCause::AllInputsClosed)` right at startup. Source nodes
    /// might want to keep producing outputs still. (There is currently an open
    /// discussion of changing this behavior and not sending `AllInputsClosed`
    /// to nodes without inputs.)
    ///
    /// Note: Stop events with `StopCause::Manual` indicate a manual stop operation
    /// issued through `dora stop` or a `ctrl-c`. Nodes **must exit** once receiving
    /// such a stop event, otherwise they will be killed by Dora.
    Stop(StopCause),
    Reload {
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
    AllInputsClosed,
}
