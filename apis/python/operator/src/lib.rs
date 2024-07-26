use std::{
    collections::{BTreeMap, HashMap},
    sync::{Arc, Mutex},
};

use arrow::pyarrow::ToPyArrow;
use dora_node_api::{
    merged::{MergeExternalSend, MergedEvent},
    DoraNode, Event, EventStream, Metadata, MetadataParameters, Parameter,
};
use eyre::{Context, Result};
use futures::{Stream, StreamExt};
use futures_concurrency::stream::Merge as _;
use pyo3::{
    prelude::*,
    types::{IntoPyDict, PyBool, PyDict, PyInt, PyString},
};

/// Dora Event
pub struct PyEvent {
    pub event: MergedEvent<PyObject>,
    pub _cleanup: Option<NodeCleanupHandle>,
}

/// Keeps the dora node alive until all event objects have been dropped.
#[derive(Clone)]
#[pyclass]
pub struct NodeCleanupHandle {
    pub _handles: Arc<(CleanupHandle<DoraNode>, CleanupHandle<EventStream>)>,
}

/// Owned type with delayed cleanup (using `handle` method).
pub struct DelayedCleanup<T>(Arc<Mutex<T>>);

impl<T> DelayedCleanup<T> {
    pub fn new(value: T) -> Self {
        Self(Arc::new(Mutex::new(value)))
    }

    pub fn handle(&self) -> CleanupHandle<T> {
        CleanupHandle(self.0.clone())
    }

    pub fn get_mut(&mut self) -> std::sync::MutexGuard<T> {
        self.0.try_lock().expect("failed to lock DelayedCleanup")
    }
}

impl Stream for DelayedCleanup<EventStream> {
    type Item = Event;

    fn poll_next(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        let mut inner: std::sync::MutexGuard<'_, EventStream> = self.get_mut().get_mut();
        inner.poll_next_unpin(cx)
    }
}

impl<'a, E> MergeExternalSend<'a, E> for DelayedCleanup<EventStream>
where
    E: 'static,
{
    type Item = MergedEvent<E>;

    fn merge_external_send(
        self,
        external_events: impl Stream<Item = E> + Unpin + Send + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + Send + 'a> {
        let dora = self.map(MergedEvent::Dora);
        let external = external_events.map(MergedEvent::External);
        Box::new((dora, external).merge())
    }
}

pub struct CleanupHandle<T>(Arc<Mutex<T>>);

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
                if let Some(metadata) = Self::metadata(event, py)? {
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

        if let Some(cleanup) = self._cleanup.clone() {
            pydict.insert("_cleanup", cleanup.into_py(py));
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

    fn metadata(event: &Event, py: Python<'_>) -> Result<Option<PyObject>> {
        match event {
            Event::Input { metadata, .. } => Ok(Some(
                metadata_to_pydict(metadata, py)
                    .context("Issue deserializing metadata")?
                    .to_object(py),
            )),
            _ => Ok(None),
        }
    }

    fn error(event: &Event) -> Option<&str> {
        match event {
            Event::Error(error) => Some(error),
            _other => None,
        }
    }
}

pub fn pydict_to_metadata(dict: Option<Bound<'_, PyDict>>) -> Result<MetadataParameters> {
    let mut parameters = BTreeMap::default();
    if let Some(pymetadata) = dict {
        for (key, value) in pymetadata.iter() {
            let key = key.extract::<String>().context("Parsing metadata keys")?;
            if value.is_exact_instance_of::<PyBool>() {
                parameters.insert(key, Parameter::Bool(value.extract()?))
            } else if value.is_instance_of::<PyInt>() {
                parameters.insert(key, Parameter::Integer(value.extract::<i64>()?))
            } else if value.is_instance_of::<PyString>() {
                parameters.insert(key, Parameter::String(value.extract()?))
            } else {
                println!("could not convert type {value}");
                parameters.insert(key, Parameter::String(value.str()?.to_string()))
            };
        }
    }
    Ok(parameters)
}

pub fn metadata_to_pydict<'a>(
    metadata: &'a Metadata,
    py: Python<'a>,
) -> Result<pyo3::Bound<'a, PyDict>> {
    let dict = PyDict::new_bound(py);
    for (k, v) in metadata.parameters.iter() {
        match v {
            Parameter::Bool(bool) => dict
                .set_item(k, bool)
                .context(format!("Could not insert metadata into python dictionary"))?,
            Parameter::Integer(int) => dict
                .set_item(k, int)
                .context(format!("Could not insert metadata into python dictionary"))?,
            Parameter::String(s) => dict
                .set_item(k, s)
                .context(format!("Could not insert metadata into python dictionary"))?,
        }
    }

    Ok(dict)
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
