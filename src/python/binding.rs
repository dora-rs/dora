use eyre::Context;
use log::debug;
#[cfg(feature = "opentelemetry_jaeger")]
use opentelemetry::{
    global,
    trace::{TraceContextExt, Tracer},
    Context as OTelContext,
};
use pyo3::{
    prelude::*,
    types::{PyBytes, PyDict, PyString},
};
use std::{collections::BTreeMap, sync::Arc};
use zenoh::buf::ZBuf;
use zenoh::prelude::SplitBuffer;

use crate::message::{message_capnp, serialize_message};
#[cfg(feature = "opentelemetry_jaeger")]
use crate::tracing::serialize_context;

use super::server::PythonCommand;

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
    pulled_states: &BTreeMap<String, ZBuf>,
) -> eyre::Result<BTreeMap<String, Vec<u8>>> {
    let mut string_context = "".to_string();
    let mut max_depth = 0;
    Python::with_gil(|py| {
        let py_inputs = PyDict::new(py);

        for (k, value) in pulled_states.iter() {
            let buffer = value.contiguous();
            let mut owned_buffer = &*buffer;
            let deserialized = capnp::serialize::read_message(
                &mut owned_buffer,
                capnp::message::ReaderOptions::new(),
            )
            .unwrap();
            let message = deserialized
                .get_root::<message_capnp::message::Reader>()
                .unwrap();
            let data = message.get_data().unwrap();
            let metadata = message.get_metadata().unwrap();
            let depth = metadata.get_depth();
            if max_depth <= depth {
                string_context = metadata.get_otel_context().unwrap().to_string();
                max_depth = depth
            }
            py_inputs.set_item(k, PyBytes::new(py, data))?;
        }

        #[cfg(feature = "opentelemetry_jaeger")]
        py_inputs.set_item("otel_context", serialize_context(&string_context))?;
        #[cfg(not(feature = "opentelemetry_jaeger"))]
        py_inputs.set_item("otel_context", &string_context)?;

        let results = py_function
            .call(py, (py_inputs,), None)
            .wrap_err(format!("'{function_name}' call did not succeed."))?;

        let py_outputs = results.cast_as::<PyDict>(py).unwrap();
        let mut outputs = BTreeMap::new();
        for (k, v) in py_outputs.into_iter() {
            let slice = v
                .cast_as::<PyBytes>()
                .or_else(|e| eyre::bail!("{e}"))?
                .as_bytes();
            let key = k
                .cast_as::<PyString>()
                .or_else(|e| eyre::bail!("{e}"))?
                .to_string();
            outputs.insert(
                key,
                serialize_message(&slice, &string_context, max_depth + 1),
            );
        }

        Ok(outputs)
    })
}

pub fn python_compute_event_loop(
    mut input_receiver: tokio::sync::mpsc::Receiver<BTreeMap<String, ZBuf>>,
    output_sender: tokio::sync::mpsc::Sender<BTreeMap<String, Vec<u8>>>,
    variables: PythonCommand,
) {
    rayon::spawn(move || {
        let app = &variables.app;
        let function_name = &variables.function;

        let py_function = Arc::new(
            init(app, function_name)
                .context(format!(
                    "Failed to init '{app}' with function '{function_name}'"
                ))
                .unwrap(),
        );
        #[cfg(feature = "opentelemetry_jaeger")]
        let tracer = global::tracer("python-caller");

        while let Some(workload) = input_receiver.blocking_recv() {
            let pyfunc = py_function.clone();
            let push_tx = output_sender.clone();

            let outputs = call(pyfunc, function_name, &workload)
                .context(format!("App: '{app}', Function: '{function_name}'"))
                .unwrap();

            let batch_messages = outputs;

            push_tx.blocking_send(batch_messages).unwrap_or_else(|err| {
                debug!("App: '{app}', Function: '{function_name}', Sending Error: {err}")
            });
        }
    });
}
