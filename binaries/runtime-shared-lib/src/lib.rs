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
use dora_runtime_api::{OperatorEvent, OperatorRunner, RunnerGuard};
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
    ) -> eyre::Result<RunnerGuard> {
        match &operator.config.source {
            // The loaded library is handed back as the runner guard so it stays
            // mapped until the event loop has joined: the operator's Arrow
            // outputs carry FFI `release` callbacks that live inside this `.so`,
            // and freeing them after an unload SIGSEGVs.
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
            })
            .map(|library| Some(Box::new(library) as Box<dyn std::any::Any>)),
            // Unsupported sources must return a descriptive error rather than
            // `Ok(())` with a silently dropped `init_done` sender, which would
            // leave the runtime task blocked in `init_done.await` until it fails
            // with the misleading "the `init_done` channel was closed
            // unexpectedly" (#2595).
            OperatorSource::Python(_) => eyre::bail!(
                "operator `{}` uses a Python source, but this is the shared-library \
                 runtime; Python operators are spawned by the Python runtime \
                 (`dora-runtime-python`)",
                operator.id
            ),
            OperatorSource::Wasm(_) => eyre::bail!(
                "operator `{}` uses a WASM source, which is not supported yet",
                operator.id
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An unsupported operator source must surface a descriptive error from
    /// `run_operator` rather than returning `Ok(())` while silently dropping the
    /// `init_done` sender — which would leave the runtime task blocked in
    /// `init_done.await` until it fails with the misleading "the `init_done`
    /// channel was closed unexpectedly".
    #[test]
    fn wasm_source_returns_descriptive_error() {
        let operator: OperatorDefinition =
            serde_yaml::from_str("id: op\nwasm: model.wasm\n").expect("operator definition parses");
        let dataflow: Descriptor =
            serde_yaml::from_str("nodes:\n  - id: a\n").expect("descriptor parses");
        let (_events_in_tx, incoming_events) = flume::unbounded::<Event>();
        let (events_tx, _events_rx) = tokio::sync::mpsc::channel(1);
        let (init_done_tx, mut init_done_rx) = oneshot::channel();

        let err = SharedLibRunner
            .run_operator(
                &NodeId::from("node".to_string()),
                operator,
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
