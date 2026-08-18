//! Shared-library (C ABI) operator runtime backend.
//!
//! Loads `.so`/`.dll`/`.dylib` operators via `libloading` and runs them on the
//! [`dora_runtime_api`] event loop. Shipped inside the `dora` CLI and launched
//! by the daemon as the `dora runtime` subcommand.
//!
//! [`SharedLibRunner`] is public because the Python runtime embeds it too: a
//! daemon that is itself an embedded Python process routes *native* operators to
//! `python -uc "import dora; dora.start_runtime()"`, so the wheel's runtime has
//! to be able to host them. See `dora-runtime-python`.

use dora_core::{
    config::NodeId,
    descriptor::{Descriptor, OperatorDefinition, OperatorSource},
};
use dora_node_api::Event;
use dora_runtime_api::{OperatorRunner, RunnerGuard, RuntimeHandle};
use eyre::{Context, Result};
use tokio::sync::oneshot;

mod runner;

/// Runtime process entry point for shared-library operators.
pub fn main() -> eyre::Result<()> {
    dora_runtime_api::main(SharedLibRunner)
}

/// Backend hosting `dora_init_operator`/`dora_on_event` C-ABI operators.
pub struct SharedLibRunner;

impl OperatorRunner for SharedLibRunner {
    fn run_operator(
        &self,
        node_id: &NodeId,
        operator: OperatorDefinition,
        incoming_events: flume::Receiver<Event>,
        handle: RuntimeHandle,
        init_done: oneshot::Sender<Result<()>>,
        _dataflow_descriptor: &Descriptor,
    ) -> eyre::Result<RunnerGuard> {
        match &operator.config.source {
            // The loaded library is handed back as the runner guard so it stays
            // mapped until the event loop has joined: values whose vtable lives
            // in this `.so` (an `OperatorEvent::Panic` payload) can still be in
            // flight, and dropping them after an unload SIGSEGVs.
            OperatorSource::SharedLibrary(source) => runner::run(
                node_id,
                &operator.id,
                source,
                handle,
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
    use dora_runtime_api::SharedAllocator;

    /// Drives `run_operator` for a source this backend cannot host and returns
    /// the error plus the still-unsignalled `init_done` receiver.
    fn run_unsupported(yaml: &str) -> (eyre::Report, oneshot::Receiver<Result<()>>) {
        let operator: OperatorDefinition =
            serde_yaml::from_str(yaml).expect("operator definition parses");
        let dataflow: Descriptor =
            serde_yaml::from_str("nodes:\n  - id: a\n").expect("descriptor parses");
        let (_events_in_tx, incoming_events) = flume::unbounded::<Event>();
        let (events_tx, _events_rx) = tokio::sync::mpsc::channel(1);
        let (init_done_tx, init_done_rx) = oneshot::channel();

        let err = SharedLibRunner
            .run_operator(
                &NodeId::from("node".to_string()),
                operator,
                incoming_events,
                RuntimeHandle::new(events_tx, SharedAllocator::default()),
                init_done_tx,
                &dataflow,
            )
            .expect_err("unsupported operator source must return an error");
        (err, init_done_rx)
    }

    /// An unsupported operator source must surface a descriptive error from
    /// `run_operator` rather than returning `Ok(())` while silently dropping the
    /// `init_done` sender — which would leave the runtime task blocked in
    /// `init_done.await` until it fails with the misleading "the `init_done`
    /// channel was closed unexpectedly".
    #[test]
    fn wasm_source_returns_descriptive_error() {
        let (err, mut init_done_rx) = run_unsupported("id: op\nwasm: model.wasm\n");
        assert!(
            err.to_string().contains("WASM"),
            "expected a descriptive WASM error, got: {err}"
        );
        assert!(
            init_done_rx.try_recv().is_err(),
            "init_done must not receive a value for an unsupported source"
        );
    }

    /// The cross-language arm this split introduces: a Python operator reaching
    /// the shared-library runtime is a routing bug, and it must fail the same
    /// way — descriptive error, `init_done` left unsignalled — rather than hang
    /// the runtime task.
    #[test]
    fn python_source_returns_descriptive_error() {
        let (err, mut init_done_rx) = run_unsupported("id: op\npython: op.py\n");
        let msg = err.to_string();
        assert!(
            msg.contains("Python") && msg.contains("shared-library runtime"),
            "expected an error naming the wrong runtime, got: {err}"
        );
        assert!(
            init_done_rx.try_recv().is_err(),
            "init_done must not receive a value for a wrongly routed source"
        );
    }
}
