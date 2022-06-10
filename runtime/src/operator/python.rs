use super::{OperatorEvent, OperatorInput};
use eyre::{bail, eyre, Context};
use pyo3::Python;
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
    thread::spawn(move || {
        let closure = AssertUnwindSafe(|| {
            let result = Python::with_gil(|py| {
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
                    operator_context
                        .call_method1("dora_on_input", (input.id.to_string(), input.value))
                        .wrap_err("dora_on_input failed")?;
                }

                operator_context
                    .call_method0("dora_drop_operator")
                    .wrap_err("dora_drop_operator failed")?;

                Result::<_, eyre::Report>::Ok(())
            });

            result.wrap_err_with(|| format!("error in Python module at {}", path.display()))
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
