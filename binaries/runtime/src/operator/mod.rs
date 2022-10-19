use dora_core::{
    config::NodeId,
    descriptor::{OperatorDefinition, OperatorSource},
};
use dora_node_api::communication::{self, CommunicationLayer};
use eyre::Context;
#[cfg(feature = "tracing")]
use opentelemetry::sdk::trace::Tracer;
use std::{any::Any, io::Write};
use tokio::sync::mpsc::Sender;

#[cfg(not(feature = "tracing"))]
type Tracer = ();

mod python;
mod shared_lib;

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

    #[cfg(feature = "tracing")]
    let tracer =
        dora_tracing::init_tracing(format!("{node_id}/{}", operator_definition.id).as_str())
            .wrap_err("could not initiate tracing for operator")?;
    #[cfg(not(feature = "tracing"))]
    let tracer = ();

    match &operator_definition.config.source {
        OperatorSource::SharedLibrary(uri) => {
            shared_lib::spawn(uri, events_tx, inputs, publishers, tracer).wrap_err_with(|| {
                format!(
                    "failed to spawn shared library operator for {}",
                    operator_definition.id
                )
            })?;
        }
        OperatorSource::Python(uri) => {
            python::spawn(uri, events_tx, inputs, publishers, tracer).wrap_err_with(|| {
                format!(
                    "failed to spawn Python operator for {}",
                    operator_definition.id
                )
            })?;
        }
        OperatorSource::Wasm(_uri) => {
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

fn download_file(uri: &http::Uri) -> Result<tempfile::NamedTempFile, eyre::ErrReport> {
    let uri_str = uri.to_string();
    let response = tokio::runtime::Handle::current().block_on(async {
        reqwest::get(&uri_str)
            .await
            .wrap_err_with(|| format!("failed to request operator from `{uri_str}`"))?
            .bytes()
            .await
            .wrap_err("failed to read operator from `{uri}`")
    })?;
    let mut tmp =
        tempfile::NamedTempFile::new().wrap_err("failed to create temp file for operator")?;
    tmp.as_file_mut()
        .write_all(&response)
        .wrap_err("failed to write downloaded operator to file")?;
    Ok(tmp)
}
