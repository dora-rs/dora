//! Shared-library (C ABI) operator runtime backend.
//!
//! Loads `.so`/`.dll`/`.dylib` operators via `libloading` and runs them on the
//! [`dora_runtime_api`] event loop. Shipped inside the `dora` CLI and launched
//! by the daemon as the `dora runtime` subcommand.

use dora_core::{
    config::NodeId,
    descriptor::{Descriptor, OperatorDefinition, OperatorSource},
};
use dora_node_api::Event;
use dora_runtime_api::{OperatorEvent, OperatorRunner};
use eyre::{Context, Result};
use tokio::sync::{mpsc::Sender, oneshot};

mod runner;

/// Runtime process entry point for shared-library operators.
pub fn main() -> eyre::Result<()> {
    dora_runtime_api::main(SharedLibRunner)
}

struct SharedLibRunner;

impl OperatorRunner for SharedLibRunner {
    fn run_operator(
        &self,
        node_id: &NodeId,
        operator: OperatorDefinition,
        incoming_events: flume::Receiver<Event>,
        events_tx: Sender<OperatorEvent>,
        init_done: oneshot::Sender<Result<()>>,
        _dataflow_descriptor: &Descriptor,
    ) -> eyre::Result<()> {
        match &operator.config.source {
            OperatorSource::SharedLibrary(source) => runner::run(
                node_id,
                &operator.id,
                source,
                events_tx,
                incoming_events,
                init_done,
            )
            .wrap_err_with(|| {
                format!(
                    "failed to spawn shared library operator for {}",
                    operator.id
                )
            }),
            OperatorSource::Python(_) => {
                // Preserve the pre-split behavior of the native runtime built
                // without the `python` feature: log and let the dropped
                // `init_done` surface the failure to the event loop.
                tracing::error!(
                    "Dora runtime tried spawning Python Operator outside of python environment."
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
