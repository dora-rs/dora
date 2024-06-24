use std::collections::HashMap;

use arrow::pyarrow::ToPyArrow;
use dora_node_api::{merged::MergedEvent, Event, Metadata, MetadataParameters};
use eyre::{Context, Result};
use pyo3::{
    prelude::*,
    pybacked::PyBackedStr,
    types::{IntoPyDict, PyDict},
};

/// Dora Event
#[derive(Debug)]
pub struct PyEvent {
    event: MergedEvent<PyObject>,
}

impl PyEvent {
    pub fn to_py_dict(self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        let mut pydict = HashMap::new();
        match &self.event {
            MergedEvent::Dora(_) => pydict.insert("kind", "dora".to_object(py)),
            MergedEvent::External(_) => pydict.insert("kind", "external".to_object(py)),
        };
        match &self.event {
            MergedEvent::Dora(event) => {
                if let Some(id) = Self::id(event) {
                    pydict.insert("id", id.into_py(py));
                }
                pydict.insert("type", Self::ty(event).to_object(py));

                if let Some(value) = self.value(py)? {
                    pydict.insert("value", value);
                }
                if let Some(metadata) = Self::metadata(event, py) {
                    pydict.insert("metadata", metadata);
                }
                if let Some(error) = Self::error(event) {
                    pydict.insert("error", error.to_object(py));
                }
            }
            MergedEvent::External(event) => {
                pydict.insert("value", event.clone());
            }
        }

        Ok(pydict.into_py_dict_bound(py).unbind())
    }

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
        match &self.event {
            MergedEvent::Dora(Event::Input { data, .. }) => {
                // TODO: Does this call leak data?&
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
    fn from(event: MergedEvent<PyObject>) -> Self {
        Self { event }
    }
}

pub fn pydict_to_metadata(dict: Option<Bound<'_, PyDict>>) -> Result<MetadataParameters> {
    let mut default_metadata = MetadataParameters::default();
    if let Some(metadata) = dict {
        for (key, value) in metadata.iter() {
            match key
                .extract::<PyBackedStr>()
                .context("Parsing metadata keys")?
                .as_ref()
            {
                "watermark" => {
                    default_metadata.watermark =
                        value.extract().context("parsing watermark failed")?;
                }
                "deadline" => {
                    default_metadata.deadline =
                        value.extract().context("parsing deadline failed")?;
                }
                "open_telemetry_context" => {
                    let otel_context: PyBackedStr = value
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

pub fn metadata_to_pydict<'a>(metadata: &'a Metadata, py: Python<'a>) -> pyo3::Bound<'a, PyDict> {
    let dict = PyDict::new_bound(py);
    dict.set_item(
        "open_telemetry_context",
        &metadata.parameters.open_telemetry_context,
    )
    .wrap_err("could not make metadata a python dictionary item")
    .unwrap();
    dict
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use aligned_vec::{AVec, ConstAlign};
    use arrow::{
        array::{
            ArrayData, ArrayRef, BooleanArray, Float64Array, Int32Array, Int64Array, Int8Array,
            ListArray, StructArray,
        },
        buffer::Buffer,
    };

    use arrow_schema::{DataType, Field};
    use dora_node_api::{
        arrow_utils::{copy_array_into_sample, required_data_size},
        RawData,
    };
    use eyre::{Context, Result};

    fn assert_roundtrip(arrow_array: &ArrayData) -> Result<()> {
        let size = required_data_size(arrow_array);
        let mut sample: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, size);

        let info = copy_array_into_sample(&mut sample, arrow_array);

        let serialized_deserialized_arrow_array = RawData::Vec(sample)
            .into_arrow_array(&info)
            .context("Could not create arrow array")?;
        assert_eq!(arrow_array, &serialized_deserialized_arrow_array);

        Ok(())
    }

    #[test]
    fn serialize_deserialize_arrow() -> Result<()> {
        // Int8
        let arrow_array = Int8Array::from(vec![1, -2, 3, 4]).into();
        assert_roundtrip(&arrow_array).context("Int8Array roundtrip failed")?;

        // Int64
        let arrow_array = Int64Array::from(vec![1, -2, 3, 4]).into();
        assert_roundtrip(&arrow_array).context("Int64Array roundtrip failed")?;

        // Float64
        let arrow_array = Float64Array::from(vec![1., -2., 3., 4.]).into();
        assert_roundtrip(&arrow_array).context("Float64Array roundtrip failed")?;

        // Struct
        let boolean = Arc::new(BooleanArray::from(vec![false, false, true, true]));
        let int = Arc::new(Int32Array::from(vec![42, 28, 19, 31]));

        let struct_array = StructArray::from(vec![
            (
                Arc::new(Field::new("b", DataType::Boolean, false)),
                boolean as ArrayRef,
            ),
            (
                Arc::new(Field::new("c", DataType::Int32, false)),
                int as ArrayRef,
            ),
        ])
        .into();
        assert_roundtrip(&struct_array).context("StructArray roundtrip failed")?;

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
        let list_data = ArrayData::builder(list_data_type)
            .len(3)
            .add_buffer(value_offsets)
            .add_child_data(value_data)
            .build()
            .unwrap();
        let list_array = ListArray::from(list_data).into();
        assert_roundtrip(&list_array).context("ListArray roundtrip failed")?;

        Ok(())
    }
}
