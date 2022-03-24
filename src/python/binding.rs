use eyre::Context;
use pyo3::prelude::*;
use std::collections::{BTreeMap, HashMap};

pub fn init(app: &str, function: &str) -> eyre::Result<Py<PyAny>> {
    pyo3::prepare_freethreaded_python();
    Python::with_gil(|py| {
        let file = py
            .import(app)
            .wrap_err("The import file was not found. Check your PYTHONPATH env variable.")?;
        // convert Function into a PyObject
        let identity = file
            .getattr(function)
            .wrap_err("The Function was not found in the imported file.")?;
        Ok(identity.to_object(py))
    })
}

pub fn call(
    py_function: &PyObject,
    states: BTreeMap<String, String>,
) -> eyre::Result<HashMap<String, String>> {
    Python::with_gil(|py| {
        let args = (states.into_py(py),);
        let result = py_function
            .call(py, args, None)
            .wrap_err("The Python function call did not succeed.")?;
        result
            .extract(py)
            .wrap_err("The Python function returned an error.")
    })
}
