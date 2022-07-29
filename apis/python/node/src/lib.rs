use dora_node_api::config::{DataId, NodeId};
use dora_node_api::{DoraNode, Input};
use eyre::{Context, Result};
use futures::StreamExt;
use pyo3::prelude::*;
use pyo3::types::PyBytes;
use std::sync::Arc;
use std::thread;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};

#[pyclass]
pub struct PyDoraNode {
    id: NodeId,
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
    #[new]
    pub fn new() -> Result<Self> {
        let id = {
            let raw =
                std::env::var("DORA_NODE_ID").wrap_err("env variable DORA_NODE_ID must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
        };

        let (tx_input, rx_input) = mpsc::channel(1);
        let (tx_output, mut rx_output) = mpsc::channel::<(String, Vec<u8>)>(1);

        // Dispatching a tokio threadpool enables us to conveniently use Dora Future stream
        // through tokio channel.
        // It would have been difficult to expose the FutureStream of Dora directly.
        thread::spawn(move || -> Result<()> {
            let rt = tokio::runtime::Builder::new_multi_thread().build()?;
            rt.block_on(async move {
                let node = Arc::new(DoraNode::init_from_env().await?);
                let _node = node.clone();
                let receive_handle = tokio::spawn(async move {
                    let mut inputs = _node.inputs().await.unwrap();
                    while let Some(input) = inputs.next().await {
                        tx_input.send(input).await?
                    }
                    Result::<_, eyre::Error>::Ok(())
                });
                let send_handle = tokio::spawn(async move {
                    while let Some((output_str, data)) = rx_output.recv().await {
                        let output_id = DataId::from(output_str);
                        node.send_output(&output_id, data.as_slice()).await?
                    }
                    Result::<_, eyre::Error>::Ok(())
                });
                let (receiver, sender) = tokio::join!(receive_handle, send_handle);
                receiver
                    .wrap_err("Handle to the receiver failed")?
                    .wrap_err("Receiving messages from receiver channel failed")?;
                sender
                    .wrap_err("Handle to the sender failed")?
                    .wrap_err("Sending messages using sender channel failed")?;
                Ok(())
            })
        });

        Ok(PyDoraNode {
            id,
            rx_input,
            tx_output,
        })
    }

    pub fn next(&mut self) -> PyResult<Option<PyInput>> {
        self.__next__()
    }

    pub fn __next__(&mut self) -> PyResult<Option<PyInput>> {
        Ok(self.rx_input.blocking_recv().map(PyInput))
    }

    fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    pub fn send_output(&self, output_str: String, data: Vec<u8>) -> Result<()> {
        Ok(self
            .tx_output
            .blocking_send((output_str, data))
            .wrap_err("Could not send output")?)
    }

    pub fn id(&self) -> String {
        self.id.to_string()
    }
}

#[pymodule]
fn dora(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyDoraNode>().unwrap();
    Ok(())
}
