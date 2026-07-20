use dora_core::{
    config::{DataId, NodeId},
    descriptor::{Descriptor, OperatorDefinition},
};
use dora_node_api::{DataSample, Event, MetadataParameters, arrow::array::ArrayData};
use eyre::Result;
use std::any::Any;
use tokio::sync::{mpsc::Sender, oneshot};

/// A language/ABI-specific operator backend.
///
/// Each runtime backend (shared-library, Python, WASM, third-party, …)
/// implements this trait and hands it to [`crate::main`], which drives the
/// language-neutral event loop. The implementation is invoked once, on the
/// **main thread** (PyO3 and `libloading` both want a dedicated thread), and is
/// responsible for loading the operator described by `operator` and running it
/// until it stops.
///
/// The runtime↔operator contract is language-neutral: consume
/// [`dora_node_api::Event`]s off `incoming_events`, emit [`OperatorEvent`]s on
/// `events_tx`, and signal readiness (or an init failure) exactly once on
/// `init_done`.
pub trait OperatorRunner {
    fn run_operator(
        &self,
        node_id: &NodeId,
        operator: OperatorDefinition,
        incoming_events: flume::Receiver<Event>,
        events_tx: Sender<OperatorEvent>,
        init_done: oneshot::Sender<Result<()>>,
        dataflow_descriptor: &Descriptor,
    ) -> eyre::Result<()>;
}

#[derive(Debug)]
#[allow(dead_code, clippy::large_enum_variant)]
pub enum OperatorEvent {
    AllocateOutputSample {
        len: usize,
        sample: oneshot::Sender<eyre::Result<DataSample>>,
    },
    Output {
        output_id: DataId,
        parameters: MetadataParameters,
        /// The output payload as an Arrow array. The runtime IPC-encodes it via
        /// [`dora_node_api::DoraNode::send_output`] (every data-plane payload is
        /// an Arrow IPC stream).
        arrow_array: ArrayData,
    },
    Error(eyre::Error),
    Panic(Box<dyn Any + Send>),
    Finished {
        reason: StopReason,
    },
}

#[derive(Debug)]
pub enum StopReason {
    InputsClosed,
    ExplicitStop,
    ExplicitStopAll,
}
