use dora_arrow_convert::ArrowData;
use dora_core::config::{DataId, OperatorId};
pub use dora_message::daemon_to_node::StopCause;
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
