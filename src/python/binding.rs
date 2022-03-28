use eyre::Context;
use pyo3::{
    buffer::PyBuffer,
    prelude::*,
    types::{PyByteArray, PyDict, PyString},
};
use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
};

pub fn init(app: &str, function: &str) -> eyre::Result<Py<PyAny>> {
    pyo3::prepare_freethreaded_python();
    Python::with_gil(|py| {
        let file = py
            .import(app)
            .wrap_err("Importing the Python file did not succeed.")?;
        // convert Function into a PyObject
        let identity = file
            .getattr(function)
            .wrap_err("The Function was not found in the imported file.")?;
        Ok(identity.to_object(py))
    })
}

pub fn call(
    py_function: Arc<PyObject>,
    states: &BTreeMap<String, Vec<u8>>,
) -> eyre::Result<HashMap<String, Vec<u8>>> {
    let outputs = Python::with_gil(|py| {
        let py_inputs = PyDict::new(py);
        for (k, v) in states.iter() {
            py_inputs.set_item(k, PyByteArray::new(py, v)).unwrap();
        }

        let results = py_function
            .call(py, (), Some(py_inputs))
            .wrap_err("The Python function call did not succeed.")
            .unwrap();

        let py_outputs = results.cast_as::<PyDict>(py).unwrap();
        let mut outputs = HashMap::new();
        for (k, v) in py_outputs.into_iter() {
            let values = PyBuffer::get(v)
                .wrap_err("Reading from Python Buffer failed")
                .unwrap()
                .to_vec(py)
                .unwrap();
            let key = k.cast_as::<PyString>().unwrap().to_string();
            outputs.insert(key, values);
        }

        outputs
    });
    Ok(outputs)
}
