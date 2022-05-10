use eyre::Context;
use log::{debug, warn};
use opentelemetry::{
    global,
    trace::{TraceContextExt, Tracer},
    Context as OTelContext,
};
use pyo3::{
    buffer::PyBuffer,
    prelude::*,
    types::{PyByteArray, PyDict, PyString},
};
use std::{collections::BTreeMap, sync::Arc};

use super::server::Workload;
use super::server::{BatchMessages, PythonCommand};

fn init(app: &str, function: &str) -> eyre::Result<Py<PyAny>> {
    pyo3::prepare_freethreaded_python();
    Python::with_gil(|py| {
        let file = py
            .import(app)
            .wrap_err(format!("Importing '{app}' did not succeed."))?;
        // convert Function into a PyObject
        let identity = file
            .getattr(function)
            .wrap_err(format!("'{function}' was not found in '{app}'."))?;
        Ok(identity.to_object(py))
    })
}

fn call(
    py_function: Arc<PyObject>,
    function_name: &str,
    states: &BTreeMap<String, Vec<u8>>,
    pulled_states: &Option<BTreeMap<String, Vec<u8>>>,
) -> eyre::Result<BTreeMap<String, Vec<u8>>> {
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
            .call(py, (py_inputs,), None)
            .wrap_err(format!("'{function_name}' call did not succeed."))?;

        let py_outputs = results.cast_as::<PyDict>(py).unwrap();
        let mut outputs = BTreeMap::new();
        for (k, v) in py_outputs.into_iter() {
            let values = PyBuffer::get(v)
                .wrap_err("Reading from Python Buffer failed")?
                .to_vec(py)?;
            let key = k
                .cast_as::<PyString>()
                .or_else(|e| eyre::bail!("{e}"))?
                .to_string();
            outputs.insert(key, values);
        }

        Ok(outputs)
    })
}

pub fn python_compute_event_loop(
    mut input_receiver: tokio::sync::mpsc::Receiver<Workload>,
    output_sender: tokio::sync::mpsc::Sender<BatchMessages>,
    variables: PythonCommand,
) {
    tokio::spawn(async move {
        let app = &variables.app;
        let function_name = &variables.function;

        let py_function = Arc::new(
            init(app, function_name)
                .context(format!(
                    "Failed to init '{app}' with function '{function_name}'"
                ))
                .unwrap(),
        );
        let tracer = global::tracer("python-caller");

        while let Some(workload) = input_receiver.recv().await {
            let span = tracer.start_with_context(
                format!("wrapper-{app}-{function_name}"),
                &workload.otel_context,
            );
            let cx = OTelContext::current_with_span(span);
            let pyfunc = py_function.clone();
            let push_tx = output_sender.clone();
            let states = workload.states.read().await.clone(); // This is probably expensive.

            let outputs = call(pyfunc, function_name, &states, &workload.pulled_states)
                .unwrap_or_else(|err| {
                    warn!("App: '{app}', Function: '{function_name}', Error: {err}");
                    states
                });

            let batch_messages = BatchMessages {
                outputs,
                deadlines: 1,
                otel_context: cx.clone(),
                degree: workload.degree + 1,
            };
            push_tx.send(batch_messages).await.unwrap_or_else(|err| {
                debug!("App: '{app}', Function: '{function_name}', Sending Error: {err}")
            });
        }
    });
}
