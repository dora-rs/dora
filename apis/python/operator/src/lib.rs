use std::sync::Arc;

use arrow::pyarrow::{FromPyArrow, ToPyArrow};
use arrow_schema::DataType;
use dora_node_api::{merged::MergedEvent, Data, Event, Metadata, MetadataParameters};
use eyre::{bail, Context, ContextCompat, Result};
use pyo3::{
    exceptions::PyLookupError,
    prelude::*,
    types::{PyBytes, PyDict},
};

#[pyclass]
pub struct PyEvent {
    event: MergedEvent<PyObject>,
    data: Option<Arc<Data>>,
}

#[pymethods]
impl PyEvent {
    pub fn __getitem__(&self, key: &str, py: Python<'_>) -> PyResult<Option<PyObject>> {
        if key == "kind" {
            let kind = match &self.event {
                MergedEvent::Dora(_) => "dora",
                MergedEvent::External(_) => "external",
            };
            return Ok(Some(kind.to_object(py)));
        }
        match &self.event {
            MergedEvent::Dora(event) => {
                let value = match key {
                    "type" => Some(Self::ty(event).to_object(py)),
                    "id" => Self::id(event).map(|v| v.to_object(py)),
                    "data" => self.data(py),
                    "value" => self.value(py)?,
                    "metadata" => Self::metadata(event, py),
                    "error" => Self::error(event).map(|v| v.to_object(py)),
                    other => {
                        return Err(PyLookupError::new_err(format!(
                            "event has no property `{other}`"
                        )))
                    }
                };
                Ok(value)
            }
            MergedEvent::External(event) => event.call_method0(py, "__getitem__").map(Some),
        }
    }
}

impl PyEvent {
    fn ty(event: &Event) -> &str {
        match event {
            Event::Stop => "STOP",
            Event::Input { .. } => "INPUT",
            Event::InputClosed { .. } => "INPUT_CLOSED",
            Event::Error(_) => "ERROR",
            _other => "UNKNOWN",
        }
    }

    fn id(event: &Event) -> Option<&str> {
        match event {
            Event::Input { id, .. } => Some(id),
            Event::InputClosed { id } => Some(id),
            _ => None,
        }
    }

    /// Returns the payload of an input event as a `PyBytes` object (if any).
    fn data(&self, py: Python<'_>) -> Option<PyObject> {
        self.data.as_ref().map(|data| PyBytes::new(py, data).into())
    }

    /// Returns the payload of an input event as an arrow array (if any).
    fn value(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        if let Some(data) = &self.data {
            let data_type = match &self.event {
                MergedEvent::Dora(Event::Input { metadata, .. }) => match &metadata
                    .schema
                    .fields()
                    .first()
                    .context("no field in schema")?
                    .data_type()
                {
                    DataType::List(field) => field.data_type().clone(),
                    _ => todo!(),
                },
                _ => DataType::UInt8,
            };
            let array = data
                .clone()
                .into_arrow_array(data_type)
                .map_err(|err| arrow::pyarrow::PyArrowException::new_err(err.to_string()))?;
            // TODO: Does this call leak data?
            let array_data = array.to_pyarrow(py)?;
            return Ok(Some(array_data));
        }

        Ok(None)
    }

    fn metadata(event: &Event, py: Python<'_>) -> Option<PyObject> {
        match event {
            Event::Input { metadata, .. } => Some(metadata_to_pydict(metadata, py).to_object(py)),
            _ => None,
        }
    }

    fn error(event: &Event) -> Option<&str> {
        match event {
            Event::Error(error) => Some(error),
            _other => None,
        }
    }
}

impl From<Event> for PyEvent {
    fn from(event: Event) -> Self {
        Self::from(MergedEvent::Dora(event))
    }
}

impl From<MergedEvent<PyObject>> for PyEvent {
    fn from(mut event: MergedEvent<PyObject>) -> Self {
        let data = if let MergedEvent::Dora(Event::Input { data, .. }) = &mut event {
            data.take().map(Arc::new)
        } else {
            None
        };
        Self { event, data }
    }
}

pub fn pydict_to_metadata(dict: Option<&PyDict>) -> Result<MetadataParameters> {
    let mut default_metadata = MetadataParameters::default();
    if let Some(metadata) = dict {
        for (key, value) in metadata.iter() {
            match key.extract::<&str>().context("Parsing metadata keys")? {
                "watermark" => {
                    default_metadata.watermark =
                        value.extract().context("parsing watermark failed")?;
                }
                "deadline" => {
                    default_metadata.deadline =
                        value.extract().context("parsing deadline failed")?;
                }
                "open_telemetry_context" => {
                    let otel_context: &str = value
                        .extract()
                        .context("parsing open telemetry context failed")?;
                    default_metadata.open_telemetry_context = otel_context.to_string();
                }
                _ => (),
            }
        }
    }
    Ok(default_metadata)
}

pub fn metadata_to_pydict<'a>(metadata: &'a Metadata, py: Python<'a>) -> &'a PyDict {
    let dict = PyDict::new(py);
    dict.set_item(
        "open_telemetry_context",
        &metadata.parameters.open_telemetry_context,
    )
    .wrap_err("could not make metadata a python dictionary item")
    .unwrap();
    dict
}

pub fn python_output_len(data: &PyObject, py: Python) -> eyre::Result<usize> {
    if let Ok(py_bytes) = data.downcast::<PyBytes>(py) {
        py_bytes.len().wrap_err("failed to get length of PyBytes")
    } else if let Ok(arrow_array) = arrow::array::ArrayData::from_pyarrow(data.as_ref(py)) {
        if arrow_array.buffers().len() != 1 {
            eyre::bail!("output arrow array must contain a single buffer");
        }

        Ok(arrow_array.buffers()[0].len())
    } else {
        eyre::bail!("invalid `data` type, must by `PyBytes` or arrow array")
    }
}

pub fn process_python_output<T>(
    data: &PyObject,
    py: Python,
    callback: impl FnOnce(&[u8]) -> eyre::Result<T>,
) -> eyre::Result<T> {
    if let Ok(py_bytes) = data.downcast::<PyBytes>(py) {
        let data = py_bytes.as_bytes();
        callback(data)
    } else if let Ok(arrow_array) = arrow::array::ArrayData::from_pyarrow(data.as_ref(py)) {
        if arrow_array.buffers().len() != 1 {
            eyre::bail!("output arrow array must contain a single buffer");
        }

        let buffers = arrow_array.buffers();
        if buffers.len() != 1 {
            bail!("Arrow array must contain a single buffer");
        }
        let slice = buffers[0];

        callback(slice)
    } else {
        eyre::bail!("invalid `data` type, must by `PyBytes` or arrow array")
    }
}

pub fn process_python_type(data: &PyObject, py: Python) -> Result<DataType> {
    if let Ok(_py_bytes) = data.downcast::<PyBytes>(py) {
        Ok(DataType::UInt8)
    } else if let Ok(arrow_array) = arrow::array::ArrayData::from_pyarrow(data.as_ref(py)) {
        Ok(arrow_array.data_type().clone())
    } else {
        eyre::bail!("invalid `data` type, must by `PyBytes` or arrow array")
    }
}
