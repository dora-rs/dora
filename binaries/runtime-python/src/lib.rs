//! Python (PyO3) operator runtime backend.
//!
//! Loads a Python module's `Operator` class and runs it on the
//! [`dora_runtime_api`] event loop. Shipped inside the Python wheel and
//! launched by the daemon via `python -uc "import dora; dora.start_runtime()"`.
//! This is the only runtime crate that links `pyo3`.
//!
//! It also hosts shared-library operators, by delegating to
//! [`dora_runtime_shared_lib::SharedLibRunner`]. That is not incidental: when
//! the daemon is itself an embedded Python process (`current_exe` ends in
//! `python`/`python3`, e.g. `python -c "import dora; dora.start_daemon()"`), it
//! routes *native* runtime nodes here too — see `native_runtime_command` in the
//! daemon's `spawn::runtime_registry`. The pre-split `dora-runtime` handled
//! `SharedLibrary` unconditionally, so the wheel's runtime could always dlopen
//! them; keeping that arm wired preserves it.

use dora_core::{
    config::NodeId,
    descriptor::{Descriptor, OperatorDefinition, OperatorSource},
};
use dora_node_api::Event;
use dora_runtime_api::{OperatorRunner, RunnerGuard, RuntimeHandle};
use dora_runtime_shared_lib::SharedLibRunner;
use eyre::{Context, Result};
use tokio::sync::oneshot;

mod runner;

/// Runtime process entry point for Python operators.
pub fn main() -> eyre::Result<()> {
    dora_runtime_api::main(PythonRunner)
}

/// Backend hosting Python operators, and native ones by delegation.
pub struct PythonRunner;

impl OperatorRunner for PythonRunner {
    fn run_operator(
        &self,
        node_id: &NodeId,
        operator: OperatorDefinition,
        incoming_events: flume::Receiver<Event>,
        handle: RuntimeHandle,
        init_done: oneshot::Sender<Result<()>>,
        dataflow_descriptor: &Descriptor,
    ) -> eyre::Result<RunnerGuard> {
        match &operator.config.source {
            OperatorSource::Python(source) => runner::run(
                node_id,
                &operator.id,
                source,
                handle,
                incoming_events,
                init_done,
                dataflow_descriptor,
            )
            .wrap_err_with(|| format!("failed to spawn Python operator for {}", operator.id))
            // Nothing to keep alive: the Python interpreter outlives the process.
            .map(|()| None),
            // `SharedLibrary` is reachable when the daemon runs as an embedded
            // Python process (see the module docs): it sends native runtime
            // nodes to this runtime, so hosting them here is what keeps that
            // deployment working. `Wasm` goes the same way rather than growing a
            // second copy of the "not supported yet" arm — the shared-library
            // backend already owns that error, including the #2595 contract of
            // leaving `init_done` unsignalled.
            OperatorSource::SharedLibrary(_) | OperatorSource::Wasm(_) => SharedLibRunner
                .run_operator(
                    node_id,
                    operator,
                    incoming_events,
                    handle,
                    init_done,
                    dataflow_descriptor,
                ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_runtime_api::SharedAllocator;

    /// Drives `run_operator` and returns the error plus the `init_done`
    /// receiver, for sources that cannot start.
    fn run_failing(yaml: &str) -> (eyre::Report, oneshot::Receiver<Result<()>>) {
        let operator: OperatorDefinition =
            serde_yaml::from_str(yaml).expect("operator definition parses");
        let dataflow: Descriptor =
            serde_yaml::from_str("nodes:\n  - id: a\n").expect("descriptor parses");
        let (_events_in_tx, incoming_events) = flume::unbounded::<Event>();
        let (events_tx, _events_rx) = tokio::sync::mpsc::channel(1);
        let (init_done_tx, init_done_rx) = oneshot::channel();

        let err = PythonRunner
            .run_operator(
                &NodeId::from("node".to_string()),
                operator,
                incoming_events,
                RuntimeHandle::new(events_tx, SharedAllocator::default()),
                init_done_tx,
                &dataflow,
            )
            .expect_err("operator must fail to start");
        (err, init_done_rx)
    }

    /// An unsupported operator source must surface a descriptive error from
    /// `run_operator` rather than returning `Ok(())` while silently dropping the
    /// `init_done` sender — which would leave the runtime task blocked in
    /// `init_done.await` until it fails with the misleading "the `init_done`
    /// channel was closed unexpectedly".
    #[test]
    fn wasm_source_returns_descriptive_error() {
        let (err, mut init_done_rx) = run_failing("id: op\nwasm: model.wasm\n");
        assert!(
            err.to_string().contains("WASM"),
            "expected a descriptive WASM error, got: {err}"
        );
        assert!(
            init_done_rx.try_recv().is_err(),
            "init_done must not receive a value for an unsupported source"
        );
    }

    /// A shared-library operator must reach the shared-library backend rather
    /// than be rejected as "wrong runtime" — the embedded-Python-daemon path
    /// depends on it (see the module docs). A missing `.so` is the closest we
    /// can get without building one: the error has to come from *loading* the
    /// library, which only the delegated backend attempts.
    #[test]
    fn shared_library_source_is_delegated_not_rejected() {
        let (err, mut init_done_rx) = run_failing("id: op\nshared-library: /nonexistent/dora-op\n");
        let msg = format!("{err:?}");
        assert!(
            msg.contains("shared library"),
            "expected a load failure from the shared-library backend, got: {msg}"
        );
        assert!(
            !msg.contains("this is the Python runtime"),
            "shared-library operators must not be rejected by the Python runtime: {msg}"
        );
        assert!(
            init_done_rx.try_recv().is_err(),
            "init_done must not receive a value when the library fails to load"
        );
    }
}
