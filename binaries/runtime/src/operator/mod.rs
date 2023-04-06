use dora_core::{
    config::{DataId, NodeId},
    descriptor::{OperatorDefinition, OperatorSource},
    message::MetadataParameters,
};
use dora_node_api::Event;
use eyre::{Context, Result};
use std::any::Any;
use tokio::sync::{mpsc::Sender, oneshot};

pub mod channel;
mod python;
mod shared_lib;

pub fn run_operator(
    node_id: &NodeId,
    operator_definition: OperatorDefinition,
    incoming_events: flume::Receiver<Event>,
    events_tx: Sender<OperatorEvent>,
    init_done: oneshot::Sender<Result<()>>,
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
        OperatorSource::Python(source) => {
            python::run(
                node_id,
                &operator_definition.id,
                source,
                events_tx,
                incoming_events,
                init_done,
            )
            .wrap_err_with(|| {
                format!(
                    "failed to spawn Python operator for {}",
                    operator_definition.id
                )
            })?;
        }
        OperatorSource::Wasm(_) => {
            tracing::error!("WASM operators are not supported yet");
        }
    }
    Ok(())
}

#[derive(Debug)]
pub enum OperatorEvent {
    Output {
        output_id: DataId,
        metadata: MetadataParameters<'static>,
        data: Vec<u8>,
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
