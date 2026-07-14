use dora_core::{
    config::{DataId, NodeId},
    descriptor::{Descriptor, OperatorDefinition, OperatorSource},
};
use dora_node_api::{Event, MetadataParameters, arrow::array::ArrayData};
use eyre::{Context, Result};
use std::any::Any;
use tokio::sync::{mpsc::Sender, oneshot};

pub mod channel;
#[cfg(feature = "python")]
mod python;
mod shared_lib;

#[allow(unused_variables)]
pub fn run_operator(
    node_id: &NodeId,
    operator_definition: OperatorDefinition,
    incoming_events: flume::Receiver<Event>,
    events_tx: Sender<OperatorEvent>,
    init_done: oneshot::Sender<Result<()>>,
    dataflow_descriptor: &Descriptor,
) -> eyre::Result<()> {
    match &operator_definition.config.source {
        OperatorSource::SharedLibrary(source) => {
            shared_lib::run(
                node_id,
                &operator_definition.id,
                source,
                events_tx,
                incoming_events,
                init_done,
            )
            .wrap_err_with(|| {
                format!(
                    "failed to spawn shared library operator for {}",
                    operator_definition.id
                )
            })?;
        }
        #[allow(unused_variables)]
        OperatorSource::Python(source) => {
            #[cfg(feature = "python")]
            python::run(
                node_id,
                &operator_definition.id,
                source,
                events_tx,
                incoming_events,
                init_done,
                dataflow_descriptor,
            )
            .wrap_err_with(|| {
                format!(
                    "failed to spawn Python operator for {}",
                    operator_definition.id
                )
            })?;
            #[cfg(not(feature = "python"))]
            tracing::error!(
                "Dora runtime tried spawning Python Operator outside of python environment."
            );
        }
        OperatorSource::Wasm(_) => {
            tracing::error!("WASM operators are not supported yet");
        }
    }
    Ok(())
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum OperatorEvent {
    Output {
        output_id: DataId,
        parameters: MetadataParameters,
        /// The output payload as an Arrow array. The runtime IPC-encodes it via
        /// [`DoraNode::send_output`] (every data-plane payload is an Arrow IPC
        /// stream).
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
