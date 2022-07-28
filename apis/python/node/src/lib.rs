use dora_node_api::DoraNode;
use pyo3::prelude::*;

#[pyclass]
#[repr(transparent)]
pub struct PyDoraNode {
    pub node: DoraNode,
}

/// This module is implemented in Rust.
#[pymodule]
fn wonnx(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyDoraNode>().unwrap();
    Ok(())
}
