use super::{OperatorEvent, OperatorInput};
use dora_node_api::DoraInputContext;
use eyre::{bail, eyre, Context};
use pyo3::{pyclass, types::IntoPyDict, types::PyBytes, Py, Python};
use std::{
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    thread,
};
use tokio::sync::mpsc::{Receiver, Sender};

fn traceback(err: pyo3::PyErr) -> eyre::Report {
    Python::with_gil(|py| {
        eyre::Report::msg(format!(
            "{}\n{err}",
            err.traceback(py)
                .expect("PyError should have a traceback")
                .format()
                .expect("Traceback could not be formatted")
        ))
    })
}

pub fn spawn(
    path: &Path,
    events_tx: Sender<OperatorEvent>,
    mut inputs: Receiver<OperatorInput>,
) -> eyre::Result<()> {
    if !path.exists() {
        bail!("No python file exists at {}", path.display());
    }
    let path = path
        .canonicalize()
        .wrap_err_with(|| format!("no file found at `{}`", path.display()))?;
    let path_cloned = path.clone();

    let init_operator = move |py: Python| {
        if let Some(parent_path) = path.parent() {
            let parent_path = parent_path
                .to_str()
                .ok_or_else(|| eyre!("module path is not valid utf8"))?;
            let sys = py.import("sys").wrap_err("failed to import `sys` module")?;
            let sys_path = sys
                .getattr("path")
                .wrap_err("failed to import `sys.path` module")?;
            let sys_path_append = sys_path
                .getattr("append")
                .wrap_err("`sys.path.append` was not found")?;
            sys_path_append
                .call1((parent_path,))
                .wrap_err("failed to append module path to python search path")?;
        }

        let module_name = path
            .file_stem()
            .ok_or_else(|| eyre!("module path has no file stem"))?
            .to_str()
            .ok_or_else(|| eyre!("module file stem is not valid utf8"))?;
        let module = py.import(module_name).map_err(traceback)?;
        let operator_class = module
            .getattr("Operator")
            .wrap_err("no `Operator` class found in module")?;

        let locals = [("Operator", operator_class)].into_py_dict(py);
        let operator = py
            .eval("Operator()", None, Some(locals))
            .map_err(traceback)?;
        Result::<_, eyre::Report>::Ok(Py::from(operator))
    };

    let loop_tx = events_tx.clone();

    let python_runner = move || {
        let operator =
            Python::with_gil(init_operator).wrap_err("failed to init python operator")?;

        while let Some(input) = inputs.blocking_recv() {
            let dora_context = DoraContext {
                otel_context: input.dora_context,
                events_tx: loop_tx.clone(),
            };

            let status_enum = Python::with_gil(|py| {
                operator
                    .call_method1(
                        py,
                        "on_input",
                        (
                            input.id.to_string(),
                            PyBytes::new(py, &input.value),
                            dora_context,
                        ),
                    )
                    .map_err(traceback)
            })?;
            let status_val = Python::with_gil(|py| status_enum.getattr(py, "value"))
                .wrap_err("on_input must have enum return value")?;
            let status: i32 = Python::with_gil(|py| status_val.extract(py))
                .wrap_err("on_input has invalid return value")?;
            match status {
                0 => {}     // ok
                1 => break, // stop
                other => bail!("on_input returned invalid status {other}"),
            }
        }

        Python::with_gil(|py| {
            let operator = operator.as_ref(py);
            if operator
                .hasattr("drop_operator")
                .wrap_err("failed to look for drop_operator")?
            {
                operator.call_method0("drop_operator")?;
            }
            Result::<_, eyre::Report>::Ok(())
        })?;

        Result::<_, eyre::Report>::Ok(())
    };

    thread::spawn(move || {
        let closure = AssertUnwindSafe(|| {
            python_runner()
                .wrap_err_with(|| format!("error in Python module at {}", path_cloned.display()))
        });

        match catch_unwind(closure) {
            Ok(Ok(())) => {
                let _ = events_tx.blocking_send(OperatorEvent::Finished);
            }
            Ok(Err(err)) => {
                let _ = events_tx.blocking_send(OperatorEvent::Error(err));
            }
            Err(panic) => {
                let _ = events_tx.blocking_send(OperatorEvent::Panic(panic));
            }
        }
    });

    Ok(())
}

#[pyclass]
#[derive(Clone)]
struct DoraContext {
    events_tx: Sender<OperatorEvent>,
    otel_context: DoraInputContext,
}

#[allow(unsafe_op_in_unsafe_fn)]
mod callback_impl {
    use super::DoraContext;
    use crate::operator::OperatorEvent;
    use dora_message::serialize_message;
    use pyo3::{pymethods, PyResult};

    #[pymethods]
    impl DoraContext {
        fn __call__(&mut self, output: &str, data: &[u8]) -> PyResult<()> {
            let result = self.events_tx.blocking_send(OperatorEvent::Output {
                id: output.to_owned().into(),
                value: serialize_message(data, &self.otel_context.otel_context),
            });
            result
                .map_err(|_| eyre::eyre!("channel to dora runtime was closed unexpectedly").into())
        }
    }
}
