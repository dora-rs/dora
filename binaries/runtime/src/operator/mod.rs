use dora_core::{
    config::{DataId, NodeId},
    descriptor::{OperatorDefinition, OperatorSource},
};
use dora_message::{Metadata, MetadataParameters};
use eyre::Context;
#[cfg(feature = "tracing")]
use opentelemetry::sdk::trace::Tracer;
use std::any::Any;
use tokio::sync::mpsc::{Receiver, Sender};

#[cfg(not(feature = "tracing"))]
type Tracer = ();

mod python;
// mod shared_lib;

pub fn run_operator(
    node_id: &NodeId,
    operator_definition: OperatorDefinition,
    incoming_events: Receiver<IncomingEvent>,
    events_tx: Sender<OperatorEvent>,
) -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
    let tracer =
        dora_tracing::init_tracing(format!("{node_id}/{}", operator_definition.id).as_str())
            .wrap_err("could not initiate tracing for operator")?;
    #[cfg(not(feature = "tracing"))]
    #[allow(clippy::let_unit_value)]
    let tracer = ();

    match &operator_definition.config.source {
        OperatorSource::SharedLibrary(source) => {
            // shared_lib::spawn(
            //     node_id,
            //     &operator_definition.id,
            //     source,
            //     events_tx,
            //     input_events,
            //     publishers,
            //     tracer,
            // )
            // .wrap_err_with(|| {
            //     format!(
            //         "failed to spawn shared library operator for {}",
            //         operator_definition.id
            //     )
            // })?;
            todo!()
        }
        OperatorSource::Python(source) => {
            python::run(
                node_id,
                &operator_definition.id,
                source,
                events_tx,
                incoming_events,
                tracer,
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
pub enum IncomingEvent {
    Stop,
    Input {
        input_id: DataId,
        metadata: Metadata<'static>,
        data: Option<Vec<u8>>,
    },
}

#[derive(Debug)]
pub enum StopReason {
    InputsClosed,
    ExplicitStop,
    ExplicitStopAll,
}
