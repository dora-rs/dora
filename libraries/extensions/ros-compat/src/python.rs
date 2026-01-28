//! Python bindings for ROS message compatibility
//!
//! This module provides a minimal Python API for converting Dora Arrow arrays
//! to ROS message format dictionaries.
use pyo3::prelude::*;

use dora_ros2_bridge_msg_gen::types::{
    MemberType, Message,
    primitives::{BasicType, GenericString, NamedType, NamespacedType, NestableType},
};
use pyo3::{
    prelude::*,
    types::{PyList, PyModule},
};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

/// Converts Dora Arrow arrays to ROS message dictionaries
#[pyclass]
pub struct RosMessageConverter {
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
}

#[pymethods]
impl RosMessageConverter {
    #[new]
    #[pyo3(signature = (ros_paths=None))]
    pub fn new(ros_paths: Option<Vec<PathBuf>>) -> PyResult<Self> {
        let ament_prefix_path = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();

        let paths: Vec<PathBuf> = match &ros_paths {
            Some(paths) => paths.clone(),
            None => {
                if ament_prefix_path.is_empty() {
                    Vec::new()
                } else {
                    ament_prefix_path.split(':').map(PathBuf::from).collect()
                }
            }
        };

        let packages = if paths.is_empty() {
            Vec::new()
        } else {
            dora_ros2_bridge_msg_gen::get_packages(
                &paths.iter().map(|p| p.as_path()).collect::<Vec<_>>(),
            )
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!(
                    "Failed to parse ROS messages: {}",
                    e
                ))
            })?
        };

        let mut messages = HashMap::new();
        for message in packages.into_iter().flat_map(|p| p.messages.into_iter()) {
            let entry: &mut HashMap<String, Message> =
                messages.entry(message.package.clone()).or_default();
            entry.insert(message.name.clone(), message);
        }

        Ok(Self {
            messages: Arc::new(messages),
        })
    }

    /// Converts an Arrow array to a ROS message dictionary (or list of dictionaries)
    #[pyo3(signature = (data, msg_type=None))]
    pub fn to_ros(&self, data: Bound<'_, PyAny>, msg_type: Option<&str>) -> PyResult<PyObject> {
        let _ = msg_type;
        let pylist = data.call_method0("to_pylist")?;
        Ok(pylist.into())
    }

    /// Returns a PyArrow Schema for the given ROS message type
    ///
    /// :param msg_type: ROS message type, e.g., "geometry_msgs/Twist"
    /// :return: pyarrow.Schema
    pub fn get_schema(&self, py: Python, msg_type: &str) -> PyResult<PyObject> {
        let (package_name, message_name) = msg_type.split_once('/').ok_or_else(|| {
            PyErr::new::<pyo3::exceptions::PyValueError, _>("Invalid message type format")
        })?;

        let message = self
            .messages
            .get(package_name)
            .and_then(|m| m.get(message_name))
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Message type not found: {}",
                    msg_type
                ))
            })?;

        let pa = py.import("pyarrow")?;
        let fields: Vec<_> = message
            .members
            .iter()
            .map(|member| {
                let field_type = type_to_pyarrow(py, &member.r#type, &self.messages, &pa)?;
                pa.call_method1("field", (member.name.as_str(), field_type))
            })
            .collect::<PyResult<_>>()?;
        let fields = PyList::new(py, fields)?;

        Ok(pa.call_method1("schema", (fields,))?.into())
    }
}

fn type_to_pyarrow(
    py: Python,
    type_: &MemberType,
    messages: &HashMap<String, HashMap<String, Message>>,
    pa: &Bound<'_, PyModule>,
) -> PyResult<PyObject> {
    match type_ {
        MemberType::NestableType(t) => nestable_to_pyarrow(py, t, messages, pa),
        MemberType::Array(a) => {
            let inner = nestable_to_pyarrow(py, &a.value_type, messages, pa)?;
            Ok(pa.getattr("list_")?.call1((inner, a.size))?.into())
        }
        MemberType::Sequence(s) => {
            let inner = nestable_to_pyarrow(py, &s.value_type, messages, pa)?;
            Ok(pa.getattr("list_")?.call1((inner,))?.into())
        }
        MemberType::BoundedSequence(bs) => {
            let inner = nestable_to_pyarrow(py, &bs.value_type, messages, pa)?;
            Ok(pa.getattr("list_")?.call1((inner,))?.into())
        }
    }
}

fn nestable_to_pyarrow(
    py: Python,
    type_: &NestableType,
    messages: &HashMap<String, HashMap<String, Message>>,
    pa: &Bound<'_, PyModule>,
) -> PyResult<PyObject> {
    match type_ {
        NestableType::BasicType(t) => match t {
            BasicType::Bool => Ok(pa.getattr("bool_")?.call0()?.into()),
            BasicType::I8 => Ok(pa.getattr("int8")?.call0()?.into()),
            BasicType::Byte | BasicType::Char | BasicType::U8 => {
                Ok(pa.getattr("uint8")?.call0()?.into())
            } // Mapped to uint8
            BasicType::I16 => Ok(pa.getattr("int16")?.call0()?.into()),
            BasicType::U16 => Ok(pa.getattr("uint16")?.call0()?.into()),
            BasicType::I32 => Ok(pa.getattr("int32")?.call0()?.into()),
            BasicType::U32 => Ok(pa.getattr("uint32")?.call0()?.into()),
            BasicType::I64 => Ok(pa.getattr("int64")?.call0()?.into()),
            BasicType::U64 => Ok(pa.getattr("uint64")?.call0()?.into()),
            BasicType::F32 => Ok(pa.getattr("float32")?.call0()?.into()),
            BasicType::F64 => Ok(pa.getattr("float64")?.call0()?.into()),
        },
        NestableType::GenericString(_) => Ok(pa.getattr("string")?.call0()?.into()),
        NestableType::NamedType(NamedType(name)) => resolve_struct(py, name, "", messages, pa),
        NestableType::NamespacedType(NamespacedType { package, name, .. }) => {
            resolve_struct(py, name, package, messages, pa)
        }
    }
}

fn resolve_struct(
    py: Python,
    name: &str,
    package: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
    pa: &Bound<'_, PyModule>,
) -> PyResult<PyObject> {
    let msg_def = if !package.is_empty() {
        messages.get(package).and_then(|m| m.get(name))
    } else {
        if let Some((pkg, msg)) = name.split_once('/') {
            messages.get(pkg).and_then(|m| m.get(msg))
        } else {
            None
        }
    };

    if let Some(msg) = msg_def {
        let fields: Vec<_> = msg
            .members
            .iter()
            .map(|member| {
                let field_type = type_to_pyarrow(py, &member.r#type, messages, &pa)?;
                pa.call_method1("field", (member.name.as_str(), field_type))
            })
            .collect::<PyResult<_>>()?;
        let fields = PyList::new(py, fields)?;
        Ok(pa.call_method1("struct_", (fields,))?.into())
    } else {
        Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
            "Unknown message type: {}/{}",
            package, name
        )))
    }
}

/// Create Python module for ROS compatibility
pub fn create_ros_compat_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<RosMessageConverter>()?;
    Ok(())
}
