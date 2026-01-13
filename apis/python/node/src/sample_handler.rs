use dora_core::metadata::ArrowTypeInfoExt;
use dora_message::metadata::ArrowTypeInfo;
use dora_node_api::{DataSample, DoraNode, MetadataParameters, dora_core::config::DataId};
use dora_operator_api_python::DelayedCleanup;
use eyre;
use pyo3::{Bound, Py, PyAny, PyResult, Python, pyclass, pymethods};

#[pyclass]
pub struct SampleHandler {
    node: DelayedCleanup<DoraNode>,
    inner: Option<SampleInner>,
    memoryview: Option<Py<PyAny>>,
}

pub struct SampleInner {
    data_id: DataId,
    type_info: ArrowTypeInfo,
    parameters: MetadataParameters,
    sample: DataSample,
}

impl SampleHandler {
    pub fn new(
        data_length: usize,
        data_id: DataId,
        parameters: MetadataParameters,
        node: DelayedCleanup<DoraNode>,
    ) -> eyre::Result<Self> {
        let sample = node.get_mut().allocate_data_sample(data_length)?;
        let type_info = ArrowTypeInfo::byte_array(data_length);
        let inner = SampleInner {
            data_id,
            type_info,
            parameters,
            sample,
        };
        Ok(Self {
            node,
            inner: Some(inner),
            memoryview: None,
        })
    }
}

#[pymethods]
impl SampleHandler {
    /// Get a writable memoryview wrapping the buffer for zero-copy writing
    /// This returns a writable memoryview that Python can fill
    ///
    /// This method will raise an error if called after send()
    pub fn as_memoryview(&mut self, py: Python) -> PyResult<Py<PyAny>> {
        use pyo3::types::PyAnyMethods;
        use std::ops::DerefMut;

        // First check if the sample has already been sent
        if self.inner.is_none() {
            return Err(pyo3::exceptions::PyRuntimeError::new_err(
                "Cannot access buffer after send() - the sample has already been sent",
            ));
        }
        if let Some(mv) = &self.memoryview {
            // Return cached memoryview if already created
            Ok(mv.clone_ref(py))
        } else if let Some(inner) = &mut self.inner {
            let sample_slice: &mut [u8] = inner.sample.deref_mut();
            let len = sample_slice.len();
            let ptr = sample_slice.as_mut_ptr();

            // Import ctypes to create a writable buffer
            let ctypes = py.import("ctypes")?;

            // Create ctypes array type: ctypes.c_uint8 * len
            let c_uint8 = ctypes.getattr("c_uint8")?;
            let array_type = c_uint8.call_method1("__mul__", (len,))?;

            // Create array from address
            let c_array = array_type.call_method1("from_address", (ptr as usize,))?;

            // Create a memoryview from the ctypes array (this will be writable)
            let builtins = py.import("builtins")?;
            let memoryview = builtins.call_method1("memoryview", (c_array,))?;

            self.memoryview = Some(memoryview.clone().into());
            Ok(memoryview.into())
        } else {
            // This should never be reached due to the check at the beginning
            unreachable!()
        }
    }

    /// Enter the context manager - returns the memoryview
    fn __enter__(&mut self, py: Python) -> PyResult<Py<PyAny>> {
        self.as_memoryview(py)
    }

    /// Exit the context manager and send the data
    fn __exit__(
        &mut self,
        _exc_type: &Bound<'_, PyAny>,
        _exc_value: &Bound<'_, PyAny>,
        _traceback: &Bound<'_, PyAny>,
    ) -> PyResult<bool> {
        self.send().map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to send: {}", e))
        })?;
        Ok(false) // Don't suppress exceptions
    }

    /// Manually send the sample (alternative to using context manager)
    pub fn send(&mut self) -> eyre::Result<()> {
        let SampleInner {
            data_id,
            type_info,
            parameters,
            sample,
        } = self
            .inner
            .take()
            .ok_or_else(|| eyre::eyre!("Sample has already been sent"))?;

        if let Some(mv) = self.memoryview.take() {
            // Drop the memoryview to release the buffer
            Python::with_gil(|py| {
                mv.call_method0(py, "release").unwrap();
            });
        }

        self.node
            .get_mut()
            .send_output_sample(data_id, type_info, parameters, Some(sample))
    }
}
