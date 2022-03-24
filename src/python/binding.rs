use eyre::Context;
use pyo3::{
    buffer::PyBuffer,
    prelude::*,
    types::{PyByteArray, PyDict},
};
use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
    time::Instant,
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
    let a = Python::with_gil(|py| {
        let time_generate_args = Instant::now();
        // let args = (states.into_py(py),);
        let dicts = PyDict::new(py);
        for (k, v) in states.into_iter() {
            let buffer = PyByteArray::new(py, v.as_slice());
            dicts.set_item(k, buffer).unwrap();
        }
        println!("time: generate_args: {:#?}", time_generate_args.elapsed());
        let result = py_function
            .call(py, (), Some(dicts))
            .wrap_err("The Python function call did not succeed.")
            .unwrap();
        let a = PyBuffer::get(result.as_ref(py)).unwrap().to_vec(py);
        //.wrap_err("The Python function returned an error.");

        a.unwrap()
    });
    let mut data = HashMap::new();
    data.insert("a".to_string(), a);
    Ok(data)
}
