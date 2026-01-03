//! Python bindings for ROS message compatibility
//!
//! This module provides a minimal Python API for converting Dora Arrow arrays
//! to ROS message format dictionaries.

use arrow::array::ArrayRef;
use pyo3::prelude::*;
use pyo3::types::PyDict;

/// Converts Dora Arrow arrays to ROS message dictionaries
///
/// Right now only supports `geometry_msgs/Twist`, but it's easy to add more.
#[pyclass]
pub struct RosMessageConverter {}

#[pymethods]
impl RosMessageConverter {
    /// Converts an Arrow array to a ROS message dictionary
    ///
    /// ```python
    /// from dora import Node
    /// from dora.ros import RosMessageConverter
    ///
    /// node = Node()
    /// converter = RosMessageConverter()
    ///
    /// for event in node:
    ///     if event["type"] == "INPUT":
    ///         data = event["value"]
    ///         ros_msg = converter.to_ros(data, "geometry_msgs/Twist")
    ///         # ros_msg is a dict with 'linear' and 'angular' keys
    /// ```
    ///
    /// :param data: PyArrow array with the message data
    /// :param msg_type: ROS message type like "geometry_msgs/Twist"
    /// :return: Python dict with the ROS message
    pub fn to_ros(&self, data: Bound<'_, PyAny>, msg_type: &str) -> PyResult<PyObject> {
        let py = data.py();

        // Convert PyArrow array to Arrow ArrayRef
        use arrow::pyarrow::FromPyArrow;
        let array_data = arrow::array::ArrayData::from_pyarrow_bound(&data)?;
        let array = arrow::array::make_array(array_data);

        // Convert based on message type
        let result = match msg_type {
            "geometry_msgs/Twist" => convert_twist_to_ros(py, &array)?,
            _ => {
                return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Unsupported message type: {}. Currently only 'geometry_msgs/Twist' is supported.",
                    msg_type
                )));
            }
        };

        Ok(result)
    }
}

/// Convert geometry_msgs/Twist Arrow array to ROS message dict
fn convert_twist_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| {
            PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Expected struct array for geometry_msgs/Twist",
            )
        })?;

    let result = PyDict::new(py);

    // Extract linear velocity
    if let Some(linear_col) = struct_array.column_by_name("linear") {
        let linear_struct = linear_col
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected linear struct")
            })?;

        let linear = PyDict::new(py);
        if let Some(x) = linear_struct.column_by_name("x") {
            if let Some(x_arr) = x.as_any().downcast_ref::<arrow::array::Float64Array>() {
                linear.set_item("x", x_arr.value(0))?;
            }
        }
        if let Some(y) = linear_struct.column_by_name("y") {
            if let Some(y_arr) = y.as_any().downcast_ref::<arrow::array::Float64Array>() {
                linear.set_item("y", y_arr.value(0))?;
            }
        }
        if let Some(z) = linear_struct.column_by_name("z") {
            if let Some(z_arr) = z.as_any().downcast_ref::<arrow::array::Float64Array>() {
                linear.set_item("z", z_arr.value(0))?;
            }
        }
        result.set_item("linear", linear)?;
    }

    // Extract angular velocity
    if let Some(angular_col) = struct_array.column_by_name("angular") {
        let angular_struct = angular_col
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected angular struct")
            })?;

        let angular = PyDict::new(py);
        if let Some(x) = angular_struct.column_by_name("x") {
            if let Some(x_arr) = x.as_any().downcast_ref::<arrow::array::Float64Array>() {
                angular.set_item("x", x_arr.value(0))?;
            }
        }
        if let Some(y) = angular_struct.column_by_name("y") {
            if let Some(y_arr) = y.as_any().downcast_ref::<arrow::array::Float64Array>() {
                angular.set_item("y", y_arr.value(0))?;
            }
        }
        if let Some(z) = angular_struct.column_by_name("z") {
            if let Some(z_arr) = z.as_any().downcast_ref::<arrow::array::Float64Array>() {
                angular.set_item("z", z_arr.value(0))?;
            }
        }
        result.set_item("angular", angular)?;
    }

    Ok(result.into())
}

/// Create Python module for ROS compatibility
pub fn create_ros_compat_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<RosMessageConverter>()?;
    Ok(())
}
