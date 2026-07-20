//! Python (PyO3) operator runtime backend.
//!
//! Loads a Python module's `Operator` class and runs it on the
//! [`dora_runtime_api`] event loop. Shipped inside the Python wheel and
//! launched by the daemon via `python -uc "import dora; dora.start_runtime()"`.
//! This is the only runtime crate that links `pyo3`.

use dora_core::{
    config::NodeId,
    descriptor::{Descriptor, OperatorDefinition, OperatorSource},
};
use dora_node_api::Event;
use dora_runtime_api::{OperatorEvent, OperatorRunner};
use eyre::{Context, Result};
use tokio::sync::{mpsc::Sender, oneshot};

mod runner;

/// Runtime process entry point for Python operators.
pub fn main() -> eyre::Result<()> {
    dora_runtime_api::main(PythonRunner)
}

struct PythonRunner;

impl OperatorRunner for PythonRunner {
    fn run_operator(
        &self,
        node_id: &NodeId,
        operator: OperatorDefinition,
        incoming_events: flume::Receiver<Event>,
        events_tx: Sender<OperatorEvent>,
        init_done: oneshot::Sender<Result<()>>,
        dataflow_descriptor: &Descriptor,
    ) -> eyre::Result<()> {
        match &operator.config.source {
            OperatorSource::Python(source) => runner::run(
                node_id,
                &operator.id,
                source,
                events_tx,
                incoming_events,
                init_done,
                dataflow_descriptor,
            )
            .wrap_err_with(|| format!("failed to spawn Python operator for {}", operator.id)),
            // The daemon only ever routes Python operators to the Python
            // runtime (it forbids mixed-language runtime nodes), so these arms
            // are unreachable in practice. Fail loudly rather than silently.
            OperatorSource::SharedLibrary(_) => {
                tracing::error!(
                    "the Python runtime cannot host shared-library operators; \
                     this operator should be spawned by the shared-library runtime"
                );
                Ok(())
            }
            OperatorSource::Wasm(_) => {
                tracing::error!("WASM operators are not supported yet");
                Ok(())
            }
        }
    }
}
