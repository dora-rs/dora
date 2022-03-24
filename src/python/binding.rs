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
            .wrap_err("The import file was not found. Check your PYTHONPATH env variable.")?;
        // convert Function into a PyObject
        let identity = file
            .getattr(function)
            .wrap_err("The Function was not found in the imported file.")?;
        Ok(identity.to_object(py))
    })
}

pub fn call(
    py_function: Arc<PyObject>,
    states: BTreeMap<String, Vec<u8>>,
) -> eyre::Result<HashMap<String, Vec<u8>>> {
    let outputs = Python::with_gil(|py| {
        // let args = (states.into_py(py),);
        let dicts = PyDict::new(py);
        for (k, v) in states.into_iter() {
            let buffer = PyByteArray::new(py, v.as_slice());
            dicts.set_item(k, buffer).unwrap();
        }
        let result = py_function
            .call(py, (), Some(dicts))
            .wrap_err("The Python function call did not succeed.")
            .unwrap();
        let dicts = result.cast_as::<PyDict>(py).unwrap();
        let mut outputs = HashMap::new();
        for (k, v) in dicts.into_iter() {
            let values = PyBuffer::get(v).unwrap().to_vec(py).unwrap();
            let key = k.cast_as::<PyString>().unwrap().to_string();
            outputs.insert(key, values);
        }

        outputs
    });
    Ok(outputs)
}
