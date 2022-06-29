use super::{OperatorEvent, OperatorInput};
use eyre::{bail, eyre, Context};
use pyo3::{pyclass, types::IntoPyDict, Py, Python};
use std::{
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    thread,
};
use tokio::sync::mpsc::{Receiver, Sender};

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

    let send_output = SendOutputCallback {
        events_tx: events_tx.clone(),
    };

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
        let module = py
            .import(module_name)
            .wrap_err("failed to import Python module")?;
        let operator_class = module
            .getattr("Operator")
            .wrap_err("no `Operator` class found in module")?;

        let locals = [("Operator", operator_class)].into_py_dict(py);
        let operator = py
            .eval("Operator()", None, Some(locals))
            .wrap_err("failed to create Operator instance")?;
        Result::<_, eyre::Report>::Ok(Py::from(operator))
    };

    let python_runner = move || {
        let operator =
            Python::with_gil(init_operator).wrap_err("failed to init python operator")?;

        while let Some(input) = inputs.blocking_recv() {
            Python::with_gil(|py| {
                operator.call_method1(
                    py,
                    "on_input",
                    (input.id.to_string(), input.value, send_output.clone()),
                )
            })
            .wrap_err("on_input failed")?;
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
            Ok(Ok(())) => {}
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
struct SendOutputCallback {
    events_tx: Sender<OperatorEvent>,
}

#[allow(unsafe_op_in_unsafe_fn)]
mod callback_impl {
    use super::SendOutputCallback;
    use crate::operator::OperatorEvent;
    use pyo3::{pymethods, PyResult};

    #[pymethods]
    impl SendOutputCallback {
        fn __call__(&mut self, output: &str, data: &[u8]) -> PyResult<()> {
            println!("RUNTIME received python output `{output}` with value `{data:?}`");
            let result = self.events_tx.blocking_send(OperatorEvent::Output {
                id: output.to_owned().into(),
                value: data.to_owned(),
            });
            result
                .map_err(|_| eyre::eyre!("channel to dora runtime was closed unexpectedly").into())
        }
    }
}
