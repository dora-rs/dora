use dora_core::descriptor::{OperatorDefinition, OperatorSource};
use dora_node_api::{
    communication::{self, CommunicationLayer},
    config::NodeId,
};
use eyre::Context;
use std::any::Any;
use tokio::sync::mpsc::Sender;

mod python;
mod shared_lib;
use dora_tracing::init_tracing;

#[tracing::instrument(skip(communication))]
pub fn spawn_operator(
    node_id: &NodeId,
    operator_definition: OperatorDefinition,
    events_tx: Sender<OperatorEvent>,
    communication: &mut dyn CommunicationLayer,
) -> eyre::Result<()> {
    let inputs = communication::subscribe_all(communication, &operator_definition.config.inputs)
        .wrap_err_with(|| {
            format!(
                "failed to subscribe to inputs of operator {}",
                operator_definition.id
            )
        })?;

    let publishers = operator_definition
        .config
        .outputs
        .iter()
        .map(|output_id| {
            let topic = format!(
                "{node_id}/{operator_id}/{output_id}",
                operator_id = operator_definition.id
            );
            communication
                .publisher(&topic)
                .map_err(|err| eyre::eyre!(err))
                .wrap_err_with(|| format!("failed to create publisher for output {output_id}"))
                .map(|p| (output_id.to_owned(), p))
        })
        .collect::<Result<_, _>>()?;
    let tracer = init_tracing(&operator_definition.id.to_string())
        .wrap_err("could not initiate tracing for operator")?;

    match &operator_definition.config.source {
        OperatorSource::SharedLibrary(path) => {
            shared_lib::spawn(path, events_tx, inputs, publishers).wrap_err_with(|| {
                format!(
                    "failed to spawn shared library operator for {}",
                    operator_definition.id
                )
            })?;
        }
        OperatorSource::Python(path) => {
            python::spawn(path, events_tx, inputs, publishers, tracer).wrap_err_with(|| {
                format!(
                    "failed to spawn Python operator for {}",
                    operator_definition.id
                )
            })?;
        }
        OperatorSource::Wasm(_path) => {
            tracing::error!("WASM operators are not supported yet");
        }
    }
    Ok(())
}

pub enum OperatorEvent {
    Error(eyre::Error),
    Panic(Box<dyn Any + Send>),
    Finished,
}
