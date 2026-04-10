use std::{
    collections::{BTreeMap, HashMap},
    sync::{Arc, Mutex},
};

use arrow::pyarrow::ToPyArrow;
use chrono::{DateTime, Utc};
use dora_node_api::{
    DoraNode, Event, EventStream, Metadata, MetadataParameters, Parameter, StopCause,
    merged::{MergeExternalSend, MergedEvent},
};
use eyre::{Context, Result};
use futures::{Stream, StreamExt};
use futures_concurrency::stream::Merge as _;
use pyo3::{
    prelude::*,
    sync::PyOnceLock,
    types::{IntoPyDict, PyBool, PyDict, PyFloat, PyInt, PyList, PyModule, PyString, PyTuple},
};
use std::time::UNIX_EPOCH;

/// Cached Python `datetime` module to avoid repeated `PyModule::import` on the hot path.
static DATETIME_MODULE: PyOnceLock<Py<PyModule>> = PyOnceLock::new();

fn datetime_module<'py>(py: Python<'py>) -> PyResult<&'py Bound<'py, PyModule>> {
    Ok(DATETIME_MODULE
        .get_or_try_init(py, || PyModule::import(py, "datetime").map(|m| m.unbind()))?
        .bind(py))
}

/// Dora Event
pub struct PyEvent {
    pub event: MergedEvent<Py<PyAny>>,
}

/// Keeps the dora node alive until all event objects have been dropped.
#[derive(Clone)]
#[pyclass(skip_from_py_object)]
pub struct NodeCleanupHandle {
    pub _handles: Arc<CleanupHandle<DoraNode>>,
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

    pub fn get_mut(&self) -> std::sync::MutexGuard<'_, T> {
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
        external_events: impl Stream<Item = E> + Unpin + Send + Sync + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + Send + Sync + 'a> {
        let dora = self.map(MergedEvent::Dora);
        let external = external_events.map(MergedEvent::External);
        Box::new((dora, external).merge())
    }
}

#[allow(dead_code)]
pub struct CleanupHandle<T>(Arc<Mutex<T>>);

impl PyEvent {
    pub fn to_py_dict(self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        let mut pydict = HashMap::new();
        match &self.event {
            MergedEvent::Dora(_) => pydict.insert(
                "kind",
                "dora"
                    .into_pyobject(py)
                    .context("Failed to create pystring")?
                    .unbind()
                    .into(),
            ),
            MergedEvent::External(_) => pydict.insert(
                "kind",
                "external"
                    .into_pyobject(py)
                    .context("Failed to create pystring")?
                    .unbind()
                    .into(),
            ),
        };
        match &self.event {
            MergedEvent::Dora(event) => {
                if let Some(id) = Self::id(event) {
                    pydict.insert(
                        "id",
                        id.into_pyobject(py)
                            .context("Failed to create id pyobject")?
                            .into(),
                    );
                }
                pydict.insert(
                    "type",
                    Self::ty(event)
                        .into_pyobject(py)
                        .context("Failed to create event pyobject")?
                        .unbind()
                        .into(),
                );

                if let Some(value) = self.value(py)? {
                    pydict.insert("value", value);
                }
                if let Some(metadata) = Self::metadata(event, py)? {
                    pydict.insert("metadata", metadata);
                }
                if let Some(error) = Self::error(event) {
                    pydict.insert(
                        "error",
                        error
                            .into_pyobject(py)
                            .context("Failed to create error pyobject")?
                            .unbind()
                            .into(),
                    );
                }
            }
            MergedEvent::External(event) => {
                pydict.insert("value", event.clone_ref(py));
            }
        }

        Ok(pydict
            .into_py_dict(py)
            .context("Failed to create py_dict")?
            .unbind())
    }

    fn ty(event: &Event) -> &'static str {
        match event {
            Event::Stop(_) => "STOP",
            Event::Input { .. } => "INPUT",
            Event::InputClosed { .. } => "INPUT_CLOSED",
            Event::InputRecovered { .. } => "INPUT_RECOVERED",
            Event::NodeRestarted { .. } => "NODE_RESTARTED",
            Event::Reload { .. } => "RELOAD",
            Event::ParamUpdate { .. } => "PARAM_UPDATE",
            Event::ParamDeleted { .. } => "PARAM_DELETED",
            Event::Error(_) => "ERROR",
            // `Event` is `#[non_exhaustive]`; surface genuinely new variants
            // as UNKNOWN rather than failing to build on future dora upgrades.
            _other => "UNKNOWN",
        }
    }

    fn id(event: &Event) -> Option<&str> {
        match event {
            Event::Input { id, .. } => Some(id.as_ref()),
            Event::InputClosed { id } => Some(id.as_ref()),
            Event::InputRecovered { id } => Some(id.as_ref()),
            Event::NodeRestarted { id } => Some(id.as_ref()),
            Event::Reload { operator_id } => operator_id.as_ref().map(|id| id.as_ref()),
            Event::ParamUpdate { key, .. } => Some(key.as_str()),
            Event::ParamDeleted { key } => Some(key.as_str()),
            Event::Stop(cause) => match cause {
                StopCause::Manual => Some("MANUAL"),
                StopCause::AllInputsClosed => Some("ALL_INPUTS_CLOSED"),
                &_ => None,
            },
            _ => None,
        }
    }

    /// Returns the payload of an event as a Python object (if any).
    ///
    /// - `Input`: the Arrow array payload.
    /// - `ParamUpdate`: the new parameter value, converted from JSON.
    fn value(&self, py: Python<'_>) -> PyResult<Option<Py<PyAny>>> {
        match &self.event {
            MergedEvent::Dora(Event::Input { data, .. }) => {
                // Ownership transfer: to_data() clones buffer Arcs, to_pyarrow()
                // creates an FFI_ArrowArray with a release callback. PyArrow's
                // _import_from_c takes ownership and calls release when the Python
                // object is GC'd, which drops the cloned Arcs. No leak.
                let array_data = data.to_data().to_pyarrow(py)?.unbind();
                Ok(Some(array_data))
            }
            MergedEvent::Dora(Event::ParamUpdate { value, .. }) => {
                // `pythonize` converts serde_json::Value recursively into native
                // Python types (None, bool, int, float, str, list, dict).
                let obj = pythonize::pythonize(py, value)
                    .context("failed to convert ParamUpdate value to Python")?
                    .unbind();
                Ok(Some(obj))
            }
            _ => Ok(None),
        }
    }

    fn metadata(event: &Event, py: Python<'_>) -> Result<Option<Py<PyAny>>> {
        match event {
            Event::Input { metadata, .. } => Ok(Some(
                metadata_to_pydict(metadata, py)
                    .context("Issue deserializing metadata")?
                    .into_pyobject(py)
                    .context("Failed to create metadata_to_pydice")?
                    .unbind()
                    .into(),
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
            } else if value.is_instance_of::<PyFloat>() {
                parameters.insert(key, Parameter::Float(value.extract::<f64>()?))
            } else if value.is_instance_of::<PyString>() {
                parameters.insert(key, Parameter::String(value.extract()?))
            } else if (value.is_instance_of::<PyTuple>() || value.is_instance_of::<PyList>())
                && value.len()? > 0
                && value.get_item(0)?.is_exact_instance_of::<PyInt>()
            {
                let list: Vec<i64> = value.extract()?;
                parameters.insert(key, Parameter::ListInt(list))
            } else if (value.is_instance_of::<PyTuple>() || value.is_instance_of::<PyList>())
                && value.len()? > 0
                && value.get_item(0)?.is_exact_instance_of::<PyFloat>()
            {
                let list: Vec<f64> = value.extract()?;
                parameters.insert(key, Parameter::ListFloat(list))
            } else if value.is_instance_of::<PyList>()
                && value.len()? > 0
                && value.get_item(0)?.is_exact_instance_of::<PyString>()
            {
                let list: Vec<String> = value.extract()?;
                parameters.insert(key, Parameter::ListString(list))
            } else {
                // Check if it's a datetime.datetime object
                let dt_module =
                    datetime_module(value.py()).context("Failed to import datetime module")?;
                let datetime_class = dt_module.getattr("datetime")?;

                if value.is_instance(datetime_class.as_ref())? {
                    // Extract timestamp using timestamp() method
                    let timestamp_float: f64 = value
                        .call_method0("timestamp")?
                        .extract()
                        .context("Failed to extract timestamp from datetime")?;

                    // Convert to chrono::DateTime<Utc>
                    // timestamp() returns seconds since epoch as float
                    // Convert to SystemTime first, then to DateTime<Utc>
                    let system_time = if timestamp_float >= 0.0 {
                        let duration = std::time::Duration::try_from_secs_f64(timestamp_float)
                            .context("Failed to convert timestamp to Duration")?;
                        UNIX_EPOCH + duration
                    } else {
                        let duration = std::time::Duration::try_from_secs_f64(-timestamp_float)
                            .context("Failed to convert timestamp to Duration")?;
                        UNIX_EPOCH.checked_sub(duration).unwrap_or(UNIX_EPOCH)
                    };

                    let dt = DateTime::<Utc>::from(system_time);

                    parameters.insert(key, Parameter::Timestamp(dt))
                } else {
                    println!("could not convert type {value}");
                    parameters.insert(key, Parameter::String(value.str()?.to_string()))
                }
            };
        }
    }
    Ok(parameters)
}

pub fn metadata_to_pydict<'a>(
    metadata: &'a Metadata,
    py: Python<'a>,
) -> Result<pyo3::Bound<'a, PyDict>> {
    let dict = PyDict::new(py);

    // Add timestamp as timezone-aware Python datetime (UTC)
    // Note: uhlc::Timestamp is a Hybrid Logical Clock. We use get_time().to_system_time()
    // which extracts the physical clock component. This pattern is used consistently
    // throughout the dora codebase (e.g., in binaries/daemon/src/log.rs, binaries/coordinator/src/lib.rs)
    // and assumes the physical time component represents UTC wall-clock time.
    let timestamp = metadata.timestamp();
    let system_time = timestamp.get_time().to_system_time();
    let duration_since_epoch = system_time
        .duration_since(UNIX_EPOCH)
        .context("Failed to calculate duration since epoch")?;

    // Extract seconds and microseconds (Python datetime supports microsecond precision)
    let seconds = duration_since_epoch.as_secs() as i64;
    let microseconds = duration_since_epoch.subsec_micros();

    // Get UTC timezone from Python's datetime module and create timezone-aware datetime
    // We use Python's datetime.fromtimestamp() to create a UTC-aware datetime object
    // This avoids float precision loss by using integer seconds and microseconds
    let dt_module = datetime_module(py).context("Failed to import datetime module")?;
    let datetime_class = dt_module.getattr("datetime")?;
    let utc_timezone = dt_module.getattr("timezone")?.getattr("utc")?;

    // Create timezone-aware datetime using fromtimestamp
    // We compute total_seconds as float (required by fromtimestamp) but preserve
    // precision by computing from integer seconds and microseconds separately
    let total_seconds = seconds as f64 + microseconds as f64 / 1_000_000.0;
    let py_datetime = datetime_class
        .call_method1("fromtimestamp", (total_seconds, utc_timezone))
        .context("Failed to create Python datetime from timestamp")?;

    dict.set_item("timestamp", py_datetime)
        .context("Could not insert timestamp into python dictionary")?;

    // Add existing parameters
    for (k, v) in metadata.parameters.iter() {
        match v {
            Parameter::Bool(bool) => dict
                .set_item(k, bool)
                .context("Could not insert metadata into python dictionary")?,
            Parameter::Integer(int) => dict
                .set_item(k, int)
                .context("Could not insert metadata into python dictionary")?,
            Parameter::Float(float) => dict
                .set_item(k, float)
                .context("Could not insert metadata into python dictionary")?,
            Parameter::String(s) => dict
                .set_item(k, s)
                .context("Could not insert metadata into python dictionary")?,
            Parameter::ListInt(l) => dict
                .set_item(k, l)
                .context("Could not insert metadata into python dictionary")?,
            Parameter::ListFloat(l) => dict
                .set_item(k, l)
                .context("Could not insert metadata into python dictionary")?,
            Parameter::ListString(l) => dict
                .set_item(k, l)
                .context("Could not insert metadata into python dictionary")?,
            Parameter::Timestamp(dt) => {
                // Convert chrono::DateTime<Utc> to Python datetime.datetime
                let timestamp = dt.timestamp();
                let microseconds = dt.timestamp_subsec_micros();

                // Get UTC timezone from Python's datetime module
                let dt_module = datetime_module(py).context("Failed to import datetime module")?;
                let datetime_class = dt_module.getattr("datetime")?;
                let utc_timezone = dt_module.getattr("timezone")?.getattr("utc")?;

                // Create timezone-aware datetime using fromtimestamp
                let total_seconds = timestamp as f64 + microseconds as f64 / 1_000_000.0;
                let py_datetime = datetime_class
                    .call_method1("fromtimestamp", (total_seconds, utc_timezone))
                    .context("Failed to create Python datetime from timestamp")?;

                dict.set_item(k, py_datetime)
                    .context("Could not insert timestamp into python dictionary")?
            }
        }
    }

    Ok(dict)
}

#[cfg(test)]
mod tests {
    use std::{ptr::NonNull, sync::Arc};

    use aligned_vec::{AVec, ConstAlign};
    use arrow::{
        array::{
            ArrayData, ArrayRef, BooleanArray, Float64Array, Int8Array, Int32Array, Int64Array,
            ListArray, StructArray,
        },
        buffer::Buffer,
    };

    use arrow_schema::{DataType, Field};
    use dora_node_api::arrow_utils::{
        buffer_into_arrow_array, copy_array_into_sample, required_data_size,
    };
    use eyre::{Context, Result};

    fn assert_roundtrip(arrow_array: &ArrayData) -> Result<()> {
        let size = required_data_size(arrow_array);
        let mut sample: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, size);

        let info = copy_array_into_sample(&mut sample, arrow_array);

        let serialized_deserialized_arrow_array = {
            let ptr = NonNull::new(sample.as_ptr() as *mut _).unwrap();
            let len = sample.len();

            let raw_buffer = unsafe {
                arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(sample))
            };
            buffer_into_arrow_array(&raw_buffer, &info)?
        };

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

    // Regression tests for dora-rs/adora#147: Python event conversion used
    // to return "UNKNOWN" with no id for five event types, making fault
    // tolerance and runtime parameters unusable from Python.
    mod py_event_types {
        use super::super::PyEvent;
        use dora_node_api::{
            Event,
            dora_core::config::{DataId, NodeId, OperatorId},
        };

        #[test]
        fn node_restarted_has_type_and_id() {
            let event = Event::NodeRestarted {
                id: NodeId::from("upstream".to_string()),
            };
            assert_eq!(PyEvent::ty(&event), "NODE_RESTARTED");
            assert_eq!(PyEvent::id(&event), Some("upstream"));
        }

        #[test]
        fn input_recovered_has_type_and_id() {
            let event = Event::InputRecovered {
                id: DataId::from("sensor".to_string()),
            };
            assert_eq!(PyEvent::ty(&event), "INPUT_RECOVERED");
            assert_eq!(PyEvent::id(&event), Some("sensor"));
        }

        #[test]
        fn reload_has_type_and_operator_id() {
            let event = Event::Reload {
                operator_id: Some(OperatorId::from("op-1".to_string())),
            };
            assert_eq!(PyEvent::ty(&event), "RELOAD");
            assert_eq!(PyEvent::id(&event), Some("op-1"));
        }

        #[test]
        fn reload_without_operator_has_no_id() {
            let event = Event::Reload { operator_id: None };
            assert_eq!(PyEvent::ty(&event), "RELOAD");
            assert_eq!(PyEvent::id(&event), None);
        }

        #[test]
        fn param_update_has_type_and_key() {
            let event = Event::ParamUpdate {
                key: "threshold".to_string(),
                value: serde_json::json!(0.85),
            };
            assert_eq!(PyEvent::ty(&event), "PARAM_UPDATE");
            assert_eq!(PyEvent::id(&event), Some("threshold"));
        }

        #[test]
        fn param_deleted_has_type_and_key() {
            let event = Event::ParamDeleted {
                key: "threshold".to_string(),
            };
            assert_eq!(PyEvent::ty(&event), "PARAM_DELETED");
            assert_eq!(PyEvent::id(&event), Some("threshold"));
        }
    }
}
