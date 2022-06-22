use super::{OperatorEvent, OperatorInput};
use eyre::{bail, eyre, Context};
use pyo3::{pyclass, Python};
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
    let events_tx_cloned = events_tx.clone();

    let python_runner = move |py: pyo3::Python| {
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

        let dora_init_operator = module
            .getattr("dora_init_operator")
            .wrap_err("`dora_init_operator` was not found")?;

        let operator_context = dora_init_operator
            .call0()
            .wrap_err("dora_init_operator failed")?;

        while let Some(input) = inputs.blocking_recv() {
            let events_tx = events_tx.clone();
            let send_output_callback = move |output_id: &str, data: &[u8]| {
                println!("RUNTIME received python output `{output_id}` with value `{data:?}`");
                let result = events_tx.blocking_send(OperatorEvent::Output {
                    id: output_id.to_owned().into(),
                    value: data.to_owned(),
                });
                if result.is_ok() {
                    0
                } else {
                    -1
                }
            };
            let send_output = SendOutputCallback {
                callback: Box::new(send_output_callback),
            };
            operator_context
                .call_method1(
                    "dora_on_input",
                    (input.id.to_string(), input.value, send_output),
                )
                .wrap_err("dora_on_input failed")?;
        }

        operator_context
            .call_method0("dora_drop_operator")
            .wrap_err("dora_drop_operator failed")?;

        Result::<_, eyre::Report>::Ok(())
    };

    thread::spawn(move || {
        let closure = AssertUnwindSafe(|| {
            let result = Python::with_gil(python_runner);
            result.wrap_err_with(|| format!("error in Python module at {}", path_cloned.display()))
        });

        match catch_unwind(closure) {
            Ok(Ok(())) => {}
            Ok(Err(err)) => {
                let _ = events_tx_cloned.blocking_send(OperatorEvent::Error(err));
            }
            Err(panic) => {
                let _ = events_tx_cloned.blocking_send(OperatorEvent::Panic(panic));
            }
        }
    });

    Ok(())
}

#[pyclass]
struct SendOutputCallback {
    callback: Box<dyn FnMut(&str, &[u8]) -> isize + Send>,
}

#[allow(unsafe_op_in_unsafe_fn)]
mod callback_impl {
    use super::SendOutputCallback;
    use pyo3::{pymethods, PyResult};

    #[pymethods]
    impl SendOutputCallback {
        fn __call__(&mut self, output: &str, data: &[u8]) -> PyResult<isize> {
            Ok((self.callback)(output, data))
        }
    }
}
