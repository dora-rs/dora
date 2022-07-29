use dora_node_api::config::DataId;
use dora_node_api::{DoraNode, Input};
use eyre::Context;
use futures::StreamExt;
use pyo3::prelude::*;
use pyo3::types::PyBytes;
use std::sync::Arc;
use std::thread;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};
#[pyclass]
// #[repr(transparent)]
pub struct PyDoraNode {
    // pub node: DoraNode,
    pub rx_input: Receiver<Input>,
    pub tx_output: Sender<(String, Vec<u8>)>,
}

pub struct PyInput(Input);

impl IntoPy<PyObject> for PyInput {
    fn into_py(self, py: Python) -> PyObject {
        (self.0.id.to_string(), PyBytes::new(py, &self.0.data)).into_py(py)
    }
}

#[pymethods]
impl PyDoraNode {
    #[staticmethod]
    pub fn init_from_env() -> Self {
        let (tx_input, rx_input) = mpsc::channel(10);
        let (tx_output, mut rx_output) = mpsc::channel::<(String, Vec<u8>)>(10);

        // Dispatching a tokio threadpool enables us to conveniently use Dora Future stream
        // through tokio channel.
        // It would have been difficult to expose the FutureStream of Dora directly.
        thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_multi_thread().build().unwrap();
            rt.block_on(async move {
                let node = Arc::new(DoraNode::init_from_env().await.unwrap());
                let _node = node.clone();
                let receive_handle = tokio::spawn(async move {
                    let mut inputs = _node.inputs().await.unwrap();
                    loop {
                        if let Some(input) = inputs.next().await {
                            tx_input.send(input).await.unwrap()
                        };
                    }
                });
                let send_handle = tokio::spawn(async move {
                    loop {
                        if let Some((output_str, data)) = rx_output.recv().await {
                            let output_id = DataId::from(output_str);
                            node.send_output(&output_id, data.as_slice()).await.unwrap()
                        };
                    }
                });
                let (_, _) = tokio::join!(receive_handle, send_handle);
            });
        });

        PyDoraNode {
            rx_input,
            tx_output,
        }
    }

    pub fn next(&mut self) -> PyResult<Option<PyInput>> {
        self.__next__()
    }

    pub fn __next__(&mut self) -> PyResult<Option<PyInput>> {
        if let Some(input) = self.rx_input.blocking_recv() {
            Ok(Some(PyInput(input)))
        } else {
            Ok(None)
        }
    }

    pub fn send_output(&self, output_str: String, data: Vec<u8>) -> () {
        self.tx_output
            .blocking_send((output_str, data))
            .wrap_err("Could not send output")
            .unwrap()
    }
}

/// This module is implemented in Rust.
#[pymodule]
fn dora(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyDoraNode>().unwrap();
    Ok(())
}
