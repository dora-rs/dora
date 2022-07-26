use eyre::Context;
use pyo3::{
    prelude::*,
    types::{PyBytes, PyDict, PyString},
};
use std::{collections::BTreeMap, sync::Arc};

#[derive(Clone)]
pub struct PythonBinding {
    app: String,
    func: String,
    python_wrapper: Arc<Py<PyAny>>,
}

impl PythonBinding {
    pub fn try_new(app: &str, function: &str) -> eyre::Result<Self> {
        pyo3::prepare_freethreaded_python();
        Python::with_gil(|py| {
            let file = py
                .import(app)
                .wrap_err(format!("Importing '{app}' did not succeed."))?;
            // convert Function into a PyObject
            let python_function = file
                .getattr(function)
                .wrap_err(format!("'{function}' was not found in '{app}'."))?;
            Ok(Self {
                app: app.to_string(),
                func: function.to_string(),
                python_wrapper: Arc::new(python_function.to_object(py)),
            })
        })
    }

    pub fn call(
        &self,
        inputs: &BTreeMap<String, Vec<u8>>,
    ) -> eyre::Result<BTreeMap<String, Vec<u8>>> {
        Python::with_gil(|py| {
            let py_inputs = PyDict::new(py);

            for (k, value) in inputs.iter() {
                py_inputs.set_item(k, PyBytes::new(py, value))?;
            }

            let results = self
                .python_wrapper
                .call(py, (py_inputs,), None)
                .wrap_err(format!(
                    "'{}.{}' call did not succeed.",
                    self.app, self.func
                ))?;

            let py_outputs = results.cast_as::<PyDict>(py).unwrap();
            let mut outputs = BTreeMap::new();
            for (k, v) in py_outputs.into_iter() {
                let slice = v
                    .cast_as::<PyBytes>()
                    .or_else(|e| eyre::bail!("{e}"))?
                    .as_bytes()
                    .to_vec();
                let key = k
                    .cast_as::<PyString>()
                    .or_else(|e| eyre::bail!("{e}"))?
                    .to_string();
                outputs.insert(key, slice);
            }

            Ok(outputs)
        })
    }
}
