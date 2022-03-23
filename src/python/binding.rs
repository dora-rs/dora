use eyre::Context;
use pyo3::prelude::*;
use std::collections::{BTreeMap, HashMap};

pub fn init(app: &str, function: &str) -> eyre::Result<Py<PyAny>> {
    pyo3::prepare_freethreaded_python();
    Ok(Python::with_gil(|py| {
        let file = py
            .import(app)
            .wrap_err("The import file was not found. Check your PYTHONPATH env variable.")
            .unwrap();
        // convert Function into a PyObject
        let identity = file
            .getattr(function)
            .wrap_err("The Function was not found in the imported file.")
            .unwrap();
        identity.to_object(py)
    }))
}

pub async fn call(
    py_function: &PyObject,
    states: BTreeMap<String, String>,
) -> eyre::Result<HashMap<String, String>> {
    Python::with_gil(|py| {
        let args = (states.clone().into_py(py),);
        let result = py_function
            .call(py, args, None)
            .wrap_err("The Python function call did not succeed.")
            .unwrap();
        result.extract(py)
    })
    .wrap_err("")
}
