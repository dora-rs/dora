use arrow::{array::ArrayRef, pyarrow::ToPyArrow};
use dora_node_api::{merged::MergedEvent, Event, Metadata, MetadataParameters};
use eyre::{Context, Result};
use pyo3::{exceptions::PyLookupError, prelude::*, types::PyDict};

#[pyclass]
pub struct PyEvent {
    event: MergedEvent<PyObject>,
    data: Option<ArrayRef>,
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
            MergedEvent::External(event) => event.call_method1(py, "__getitem__", (key,)).map(Some),
        }
    }

    pub fn inner(&mut self) -> Option<&PyObject> {
        match &self.event {
            MergedEvent::Dora(_) => None,
            MergedEvent::External(event) => Some(event),
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

    /// Returns the payload of an input event as an arrow array (if any).
    fn value(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        match (&self.event, &self.data) {
            (MergedEvent::Dora(Event::Input { .. }), Some(data)) => {
                // TODO: Does this call leak data?
                let array_data = data.to_data().to_pyarrow(py)?;
                Ok(Some(array_data))
            }
            _ => Ok(None),
        }
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
            Some(data.clone())
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
