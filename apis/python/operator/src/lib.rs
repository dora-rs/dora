use std::sync::Arc;

use arrow::{array::ArrayData, pyarrow::ToPyArrow};
use dora_node_api::{
    dora_core::message::{ArrowTypeInfo, BufferOffset},
    merged::MergedEvent,
    Data, Event, Metadata, MetadataParameters,
};
use eyre::{Context, Result};
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

    /// Returns the payload of an input event as a `PyBytes` object (if any).
    fn data(&self, py: Python<'_>) -> Option<PyObject> {
        self.data.as_ref().map(|data| PyBytes::new(py, data).into())
    }

    /// Returns the payload of an input event as an arrow array (if any).
    fn value(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        match (&self.event, &self.data) {
            (MergedEvent::Dora(Event::Input { metadata, .. }), Some(data)) => {
                let array = data
                    .clone()
                    .into_arrow_array(&metadata.type_info)
                    .context("Could not create arrow array")?;
                // TODO: Does this call leak data?
                let array_data = array.to_pyarrow(py)?;
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

pub fn copy_array_into_sample(
    target_buffer: &mut [u8],
    arrow_array: &ArrayData,
) -> eyre::Result<ArrowTypeInfo> {
    let mut next_offset = 0;
    copy_array_into_sample_inner(target_buffer, &mut next_offset, arrow_array)
}

fn copy_array_into_sample_inner(
    target_buffer: &mut [u8],
    next_offset: &mut usize,
    arrow_array: &ArrayData,
) -> eyre::Result<ArrowTypeInfo> {
    let mut buffer_offsets = Vec::new();
    for buffer in arrow_array.buffers().iter() {
        let len = buffer.len();
        assert!(
            target_buffer[*next_offset..].len() >= len,
            "target buffer too small (total_len: {}, offset: {}, required_len: {len})",
            target_buffer.len(),
            *next_offset,
        );
        target_buffer[*next_offset..][..len].copy_from_slice(buffer.as_slice());
        buffer_offsets.push(BufferOffset {
            offset: *next_offset,
            len,
        });
        *next_offset += len;
    }

    let mut child_data = Vec::new();
    for child in arrow_array.child_data() {
        let child_type_info = copy_array_into_sample_inner(target_buffer, next_offset, child)?;
        child_data.push(child_type_info);
    }

    Ok(ArrowTypeInfo {
        data_type: arrow_array.data_type().clone(),
        len: arrow_array.len(),
        null_count: arrow_array.null_count(),
        validity: arrow_array.nulls().map(|b| b.validity().to_owned()),
        offset: arrow_array.offset(),
        buffer_offsets,
        child_data,
    })
}

pub fn required_data_size(array: &ArrayData) -> usize {
    let mut size = 0;
    for buffer in array.buffers() {
        size += buffer.len();
    }
    for child in array.child_data() {
        size += required_data_size(child);
    }
    size
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use arrow::{
        array::{
            ArrayData, ArrayRef, BooleanArray, Float64Array, Int32Array, Int64Array, Int8Array,
            ListArray, StructArray,
        },
        buffer::Buffer,
    };

    use arrow_schema::{DataType, Field};
    use dora_node_api::Data;
    use eyre::{Context, Result};

    use crate::{copy_array_into_sample, required_data_size};

    fn assert_roundtrip(arrow_array: &ArrayData) -> Result<()> {
        let size = required_data_size(&arrow_array);
        let mut sample: Vec<u8> = vec![0; size];

        let info = copy_array_into_sample(&mut sample, &arrow_array)?;

        let serialized_deserialized_arrow_array = Arc::new(Data::Vec(sample))
            .into_arrow_array(&info)
            .context("Could not create arrow array")?;
        assert_eq!(arrow_array, &serialized_deserialized_arrow_array);

        Ok(())
    }

    #[test]
    fn serialize_deserialize_arrow() -> Result<()> {
        // Int8
        let arrow_array = Int8Array::from(vec![1, -2, 3, 4]).into();
        assert_roundtrip(&arrow_array)?;

        // Int64
        let arrow_array = Int64Array::from(vec![1, -2, 3, 4]).into();
        assert_roundtrip(&arrow_array)?;

        // Float64
        let arrow_array = Float64Array::from(vec![1., -2., 3., 4.]).into();
        assert_roundtrip(&arrow_array)?;

        // Struct
        let boolean = Arc::new(BooleanArray::from(vec![false, false, true, true]));
        let int = Arc::new(Int32Array::from(vec![42, 28, 19, 31]));

        let struct_array = StructArray::from(vec![
            (
                Arc::new(Field::new("b", DataType::Boolean, false)),
                boolean.clone() as ArrayRef,
            ),
            (
                Arc::new(Field::new("c", DataType::Int32, false)),
                int.clone() as ArrayRef,
            ),
        ])
        .into();
        assert_roundtrip(&struct_array)?;

        // List
        let value_data = ArrayData::builder(DataType::Int32)
            .len(8)
            .add_buffer(Buffer::from_slice_ref([0, 1, 2, 3, 4, 5, 6, 7]))
            .build()
            .unwrap();

        // Construct a buffer for value offsets, for the nested array:
        //  [[0, 1, 2], [3, 4, 5], [6, 7]]
        let value_offsets = Buffer::from_slice_ref([0, 3, 6, 8]);

        // Construct a list array from the above two
        let list_data_type = DataType::List(Arc::new(Field::new("item", DataType::Int32, false)));
        let list_data = ArrayData::builder(list_data_type.clone())
            .len(3)
            .add_buffer(value_offsets.clone())
            .add_child_data(value_data.clone())
            .build()
            .unwrap();
        let list_array = ListArray::from(list_data).into();
        assert_roundtrip(&list_array)?;

        Ok(())
    }
}
