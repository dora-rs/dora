use dora_core::{
    config::{DataId, NodeId},
    descriptor::{Descriptor, OperatorDefinition, OperatorSource},
};
use dora_node_api::{DataSample, Event, MetadataParameters, arrow::array::ArrayData};
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
            eyre::bail!(
                "operator `{}` uses a Python source, but this dora-runtime was \
                 built without the `python` feature",
                operator_definition.id
            );
        }
        OperatorSource::Wasm(_) => {
            eyre::bail!(
                "operator `{}` uses a WASM source, which is not supported yet",
                operator_definition.id
            );
        }
    }
    Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    /// An unsupported operator source must surface a descriptive error from
    /// `run_operator` rather than returning `Ok(())` while silently dropping
    /// the `init_done` sender — which would leave the runtime task blocked in
    /// `init_done.await` until it fails with the misleading "the `init_done`
    /// channel was closed unexpectedly".
    #[test]
    fn wasm_source_returns_descriptive_error() {
        let operator_definition: OperatorDefinition =
            serde_yaml::from_str("id: op\nwasm: model.wasm\n").expect("operator definition parses");
        let dataflow: Descriptor =
            serde_yaml::from_str("nodes:\n  - id: a\n").expect("descriptor parses");
        let (_events_in_tx, incoming_events) = flume::unbounded::<Event>();
        let (events_tx, _events_rx) = tokio::sync::mpsc::channel(1);
        let (init_done_tx, mut init_done_rx) = oneshot::channel();

        let err = run_operator(
            &NodeId::from("node".to_string()),
            operator_definition,
            incoming_events,
            events_tx,
            init_done_tx,
            &dataflow,
        )
        .expect_err("WASM operator source must return an error");
        assert!(
            err.to_string().contains("WASM"),
            "expected a descriptive WASM error, got: {err}"
        );
        // The init_done sender must not have signalled readiness.
        assert!(
            init_done_rx.try_recv().is_err(),
            "init_done must not receive a value for an unsupported source"
        );
    }
}
