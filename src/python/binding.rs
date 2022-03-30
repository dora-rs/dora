use eyre::eyre;
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

use super::server::PythonCommand;
use super::server::Workload;

fn init(app: &str, function: &str) -> eyre::Result<Py<PyAny>> {
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

fn call(
    py_function: Arc<PyObject>,
    states: &BTreeMap<String, Vec<u8>>,
    pulled_states: &Option<BTreeMap<String, Vec<u8>>>,
) -> eyre::Result<HashMap<String, Vec<u8>>> {
    Python::with_gil(|py| {
        let py_inputs = PyDict::new(py);
        for (k, v) in states.iter() {
            py_inputs.set_item(k, PyByteArray::new(py, v))?;
        }
        if let Some(pulled_states) = pulled_states {
            for (k, v) in pulled_states.iter() {
                py_inputs.set_item(k, PyByteArray::new(py, v))?;
            }
        }

        let results = py_function
            .call(py, (), Some(py_inputs))
            .wrap_err("The Python function call did not succeed.")?;

        let py_outputs = results.cast_as::<PyDict>(py).unwrap();
        let mut outputs = HashMap::new();
        for (k, v) in py_outputs.into_iter() {
            let values = PyBuffer::get(v)
                .wrap_err("Reading from Python Buffer failed")?
                .to_vec(py)?;
            let key = k
                .cast_as::<PyString>()
                .map_err(|e| eyre!("{e}"))?
                .to_string();
            outputs.insert(key, values);
        }

        Ok(outputs)
    })
}

pub fn python_compute_event_loop(
    mut input_receiver: tokio::sync::mpsc::Receiver<Workload>,
    output_sender: tokio::sync::mpsc::Sender<
        std::collections::HashMap<std::string::String, std::vec::Vec<u8>>,
    >,
    variables: PythonCommand,
) {
    tokio::spawn(async move {
        let py_function = Arc::new(
            init(&variables.app, &variables.function)
                .context("Failed to init the Python Function")
                .unwrap(),
        );

        while let Some(workload) = input_receiver.recv().await {
            let pyfunc = py_function.clone();
            let push_tx = output_sender.clone();
            let states = workload.states.read().await.clone(); // This is probably expensive.
            rayon::spawn(move || {
                push_tx
                    .blocking_send(
                        call(pyfunc, &states, &workload.pulled_states)
                            .wrap_err("Python binding call did not work")
                            .unwrap(),
                    )
                    .unwrap();
            });
        }
    });
}
