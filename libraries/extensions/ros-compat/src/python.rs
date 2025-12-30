//! Python bindings for ROS message compatibility

use arrow::array::{ArrayRef, make_array};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList};

/// Python binding for converting Dora Arrow arrays to ROS messages
#[pyclass]
pub struct RosMessageConverter {
    // Message type information would be stored here
}

#[pymethods]
impl RosMessageConverter {
    /// Convert a Dora Arrow array to a ROS message
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
    ///         # Use ROS message...
    /// ```
    ///
    /// :type data: pyarrow.Array
    /// :type msg_type: str
    /// :rtype: dict
    pub fn to_ros(&self, data: Bound<'_, PyAny>, msg_type: &str) -> PyResult<PyObject> {
        let py = data.py();

        // Import pyarrow (not used directly but may be needed for type checking)
        let _pyarrow = PyModule::import(py, "pyarrow")?;

        // Convert Arrow array to Python dict
        // This is a simplified version - full implementation would parse
        // the message type and convert accordingly
        use arrow::pyarrow::FromPyArrow;
        let array_data = arrow::array::ArrayData::from_pyarrow_bound(&data)?;
        let array = arrow::array::make_array(array_data);

        // Convert to Python dict based on message type
        let result = match msg_type {
            "geometry_msgs/Twist" => convert_twist_to_ros(py, &array)?,
            "geometry_msgs/Pose" => convert_pose_to_ros(py, &array)?,
            "geometry_msgs/PoseStamped" => convert_pose_stamped_to_ros(py, &array)?,
            "geometry_msgs/TwistStamped" => convert_twist_stamped_to_ros(py, &array)?,
            "geometry_msgs/Transform" => convert_transform_to_ros(py, &array)?,
            "geometry_msgs/TransformStamped" => convert_transform_stamped_to_ros(py, &array)?,
            "sensor_msgs/Image" => convert_image_to_ros(py, &array)?,
            "sensor_msgs/CompressedImage" => convert_compressed_image_to_ros(py, &array)?,
            "sensor_msgs/LaserScan" => convert_laserscan_to_ros(py, &array)?,
            "sensor_msgs/Imu" => convert_imu_to_ros(py, &array)?,
            "nav_msgs/Odometry" => convert_odometry_to_ros(py, &array)?,
            "nav_msgs/Path" => convert_path_to_ros(py, &array)?,
            "std_msgs/Header" => convert_header_to_ros(py, &array)?,
            _ => {
                // Generic conversion for unknown types
                convert_generic_to_ros(py, &array)?
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
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    // Extract linear
    if let Some(linear_col) = struct_array.column_by_name("linear") {
        let linear_struct = linear_col
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected linear struct")
            })?;

        let mut linear = PyDict::new(py);
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

    // Extract angular
    if let Some(angular_col) = struct_array.column_by_name("angular") {
        let angular_struct = angular_col
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected angular struct")
            })?;

        let mut angular = PyDict::new(py);
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

/// Convert std_msgs/Header Arrow array to ROS message dict
fn convert_header_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(seq) = struct_array.column_by_name("seq") {
        if let Some(seq_arr) = seq.as_any().downcast_ref::<arrow::array::UInt32Array>() {
            result.set_item("seq", seq_arr.value(0))?;
        }
    }

    if let Some(stamp_col) = struct_array.column_by_name("stamp") {
        let stamp_struct = stamp_col
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected stamp struct")
            })?;

        let mut stamp = PyDict::new(py);
        if let Some(secs) = stamp_struct.column_by_name("secs") {
            if let Some(secs_arr) = secs.as_any().downcast_ref::<arrow::array::UInt32Array>() {
                stamp.set_item("secs", secs_arr.value(0))?;
            }
        }
        if let Some(nsecs) = stamp_struct.column_by_name("nsecs") {
            if let Some(nsecs_arr) = nsecs.as_any().downcast_ref::<arrow::array::UInt32Array>() {
                stamp.set_item("nsecs", nsecs_arr.value(0))?;
            }
        }
        result.set_item("stamp", stamp)?;
    }

    if let Some(frame_id) = struct_array.column_by_name("frame_id") {
        let frame_id_str = frame_id
            .as_any()
            .downcast_ref::<arrow::array::StringArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected string array")
            })?
            .value(0);
        result.set_item("frame_id", frame_id_str)?;
    }

    Ok(result.into())
}

/// Convert geometry_msgs/Pose Arrow array to ROS message dict
fn convert_pose_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(position_col) = struct_array.column_by_name("position") {
        let position_struct = position_col
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected position struct")
            })?;

        let mut position = PyDict::new(py);
        if let Some(x) = position_struct.column_by_name("x") {
            if let Some(x_arr) = x.as_any().downcast_ref::<arrow::array::Float64Array>() {
                position.set_item("x", x_arr.value(0))?;
            }
        }
        if let Some(y) = position_struct.column_by_name("y") {
            if let Some(y_arr) = y.as_any().downcast_ref::<arrow::array::Float64Array>() {
                position.set_item("y", y_arr.value(0))?;
            }
        }
        if let Some(z) = position_struct.column_by_name("z") {
            if let Some(z_arr) = z.as_any().downcast_ref::<arrow::array::Float64Array>() {
                position.set_item("z", z_arr.value(0))?;
            }
        }
        result.set_item("position", position)?;
    }

    if let Some(orientation_col) = struct_array.column_by_name("orientation") {
        let orientation_struct = orientation_col
            .as_any()
            .downcast_ref::<StructArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected orientation struct")
            })?;

        let mut orientation = PyDict::new(py);
        if let Some(x) = orientation_struct.column_by_name("x") {
            if let Some(x_arr) = x.as_any().downcast_ref::<arrow::array::Float64Array>() {
                orientation.set_item("x", x_arr.value(0))?;
            }
        }
        if let Some(y) = orientation_struct.column_by_name("y") {
            if let Some(y_arr) = y.as_any().downcast_ref::<arrow::array::Float64Array>() {
                orientation.set_item("y", y_arr.value(0))?;
            }
        }
        if let Some(z) = orientation_struct.column_by_name("z") {
            if let Some(z_arr) = z.as_any().downcast_ref::<arrow::array::Float64Array>() {
                orientation.set_item("z", z_arr.value(0))?;
            }
        }
        if let Some(w) = orientation_struct.column_by_name("w") {
            if let Some(w_arr) = w.as_any().downcast_ref::<arrow::array::Float64Array>() {
                orientation.set_item("w", w_arr.value(0))?;
            }
        }
        result.set_item("orientation", orientation)?;
    }

    Ok(result.into())
}

/// Convert geometry_msgs/PoseStamped Arrow array to ROS message dict
fn convert_pose_stamped_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    if let Some(pose_col) = struct_array.column_by_name("pose") {
        let pose = convert_pose_to_ros(py, pose_col)?;
        result.set_item("pose", pose)?;
    }

    Ok(result.into())
}

/// Convert geometry_msgs/TwistStamped Arrow array to ROS message dict
fn convert_twist_stamped_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    if let Some(twist_col) = struct_array.column_by_name("twist") {
        let twist = convert_twist_to_ros(py, twist_col)?;
        result.set_item("twist", twist)?;
    }

    Ok(result.into())
}

/// Convert geometry_msgs/Point Arrow array to ROS message dict
fn convert_point_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(x) = struct_array.column_by_name("x") {
        if let Some(x_arr) = x.as_any().downcast_ref::<arrow::array::Float64Array>() {
            result.set_item("x", x_arr.value(0))?;
        }
    }
    if let Some(y) = struct_array.column_by_name("y") {
        if let Some(y_arr) = y.as_any().downcast_ref::<arrow::array::Float64Array>() {
            result.set_item("y", y_arr.value(0))?;
        }
    }
    if let Some(z) = struct_array.column_by_name("z") {
        if let Some(z_arr) = z.as_any().downcast_ref::<arrow::array::Float64Array>() {
            result.set_item("z", z_arr.value(0))?;
        }
    }

    Ok(result.into())
}

/// Convert geometry_msgs/Quaternion Arrow array to ROS message dict
fn convert_quaternion_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(x) = struct_array.column_by_name("x") {
        if let Some(x_arr) = x.as_any().downcast_ref::<arrow::array::Float64Array>() {
            result.set_item("x", x_arr.value(0))?;
        }
    }
    if let Some(y) = struct_array.column_by_name("y") {
        if let Some(y_arr) = y.as_any().downcast_ref::<arrow::array::Float64Array>() {
            result.set_item("y", y_arr.value(0))?;
        }
    }
    if let Some(z) = struct_array.column_by_name("z") {
        if let Some(z_arr) = z.as_any().downcast_ref::<arrow::array::Float64Array>() {
            result.set_item("z", z_arr.value(0))?;
        }
    }
    if let Some(w) = struct_array.column_by_name("w") {
        if let Some(w_arr) = w.as_any().downcast_ref::<arrow::array::Float64Array>() {
            result.set_item("w", w_arr.value(0))?;
        }
    }

    Ok(result.into())
}

/// Convert geometry_msgs/Transform Arrow array to ROS message dict
fn convert_transform_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(translation_col) = struct_array.column_by_name("translation") {
        let translation = convert_point_to_ros(py, translation_col)?;
        result.set_item("translation", translation)?;
    }

    if let Some(rotation_col) = struct_array.column_by_name("rotation") {
        let rotation = convert_quaternion_to_ros(py, rotation_col)?;
        result.set_item("rotation", rotation)?;
    }

    Ok(result.into())
}

/// Convert geometry_msgs/TransformStamped Arrow array to ROS message dict
fn convert_transform_stamped_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    if let Some(child_frame_id) = struct_array.column_by_name("child_frame_id") {
        let child_frame_id_str = child_frame_id
            .as_any()
            .downcast_ref::<arrow::array::StringArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected string array")
            })?
            .value(0);
        result.set_item("child_frame_id", child_frame_id_str)?;
    }

    if let Some(transform_col) = struct_array.column_by_name("transform") {
        let transform = convert_transform_to_ros(py, transform_col)?;
        result.set_item("transform", transform)?;
    }

    Ok(result.into())
}

/// Convert sensor_msgs/Image Arrow array to ROS message dict
fn convert_image_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    if let Some(height) = struct_array.column_by_name("height") {
        if let Some(height_arr) = height.as_any().downcast_ref::<arrow::array::UInt32Array>() {
            result.set_item("height", height_arr.value(0))?;
        }
    }
    if let Some(width) = struct_array.column_by_name("width") {
        if let Some(width_arr) = width.as_any().downcast_ref::<arrow::array::UInt32Array>() {
            result.set_item("width", width_arr.value(0))?;
        }
    }
    if let Some(encoding) = struct_array.column_by_name("encoding") {
        let encoding_str = encoding
            .as_any()
            .downcast_ref::<arrow::array::StringArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected string array")
            })?
            .value(0);
        result.set_item("encoding", encoding_str)?;
    }
    if let Some(data) = struct_array.column_by_name("data") {
        // Convert binary array to Python bytes
        let binary_array = data
            .as_any()
            .downcast_ref::<arrow::array::BinaryArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected binary array")
            })?;
        let data_bytes = binary_array.value(0);
        result.set_item("data", PyBytes::new(py, data_bytes))?;
    }

    Ok(result.into())
}

/// Convert sensor_msgs/CompressedImage Arrow array to ROS message dict
fn convert_compressed_image_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    if let Some(format) = struct_array.column_by_name("format") {
        let format_str = format
            .as_any()
            .downcast_ref::<arrow::array::StringArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected string array")
            })?
            .value(0);
        result.set_item("format", format_str)?;
    }
    if let Some(data) = struct_array.column_by_name("data") {
        let binary_array = data
            .as_any()
            .downcast_ref::<arrow::array::BinaryArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected binary array")
            })?;
        let data_bytes = binary_array.value(0);
        result.set_item("data", PyBytes::new(py, data_bytes))?;
    }

    Ok(result.into())
}

/// Convert sensor_msgs/LaserScan Arrow array to ROS message dict
fn convert_laserscan_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    // Add other LaserScan fields (simplified - full implementation would handle all fields)
    if let Some(ranges) = struct_array.column_by_name("ranges") {
        // Convert list array to Python list
        use arrow::array::Array;
        let list_array = ranges
            .as_any()
            .downcast_ref::<arrow::array::ListArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected list array")
            })?;
        let ranges_list = PyList::empty(py);
        let list_len = list_array.len();
        for i in 0..list_len {
            let value = list_array.value(i);
            if let Some(float_array) = value.as_any().downcast_ref::<arrow::array::Float32Array>() {
                let len: usize = float_array.len();
                for j in 0..len {
                    ranges_list.append(float_array.value(j))?;
                }
            }
        }
        result.set_item("ranges", ranges_list)?;
    }

    Ok(result.into())
}

/// Convert sensor_msgs/Imu Arrow array to ROS message dict
fn convert_imu_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    // Add orientation, angular_velocity, linear_acceleration
    // (simplified - full implementation would handle all fields and covariance matrices)

    Ok(result.into())
}

/// Convert nav_msgs/Odometry Arrow array to ROS message dict
fn convert_odometry_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    if let Some(child_frame_id) = struct_array.column_by_name("child_frame_id") {
        let child_frame_id_str = child_frame_id
            .as_any()
            .downcast_ref::<arrow::array::StringArray>()
            .ok_or_else(|| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected string array")
            })?
            .value(0);
        result.set_item("child_frame_id", child_frame_id_str)?;
    }

    // Add pose and twist (simplified)

    Ok(result.into())
}

/// Convert nav_msgs/Path Arrow array to ROS message dict
fn convert_path_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    if let Some(header_col) = struct_array.column_by_name("header") {
        let header = convert_header_to_ros(py, header_col)?;
        result.set_item("header", header)?;
    }

    // Add poses list (simplified)

    Ok(result.into())
}

/// Generic conversion for unknown message types
fn convert_generic_to_ros(py: Python, array: &ArrayRef) -> PyResult<PyObject> {
    use arrow::array::StructArray;

    let struct_array = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>("Expected struct array"))?;

    let mut result = PyDict::new(py);

    // Convert all fields to dict
    for (i, field) in struct_array.fields().iter().enumerate() {
        let column = struct_array.column(i);
        let field_name = field.name();

        // Simple conversion - can be extended for nested types
        match column.data_type() {
            arrow::datatypes::DataType::Float64 => {
                if let Some(primitive) =
                    column.as_any().downcast_ref::<arrow::array::Float64Array>()
                {
                    result.set_item(field_name, primitive.value(0))?;
                }
            }
            arrow::datatypes::DataType::UInt32 => {
                if let Some(primitive) = column.as_any().downcast_ref::<arrow::array::UInt32Array>()
                {
                    result.set_item(field_name, primitive.value(0))?;
                }
            }
            arrow::datatypes::DataType::Utf8 => {
                if let Some(string_array) =
                    column.as_any().downcast_ref::<arrow::array::StringArray>()
                {
                    let val: &str = string_array.value(0);
                    result.set_item(field_name, val.to_string())?;
                }
            }
            _ => {
                // For complex types, skip for now (could implement recursive conversion)
                // result.set_item(field_name, PyObject::from(column.clone()))?;
            }
        }
    }

    Ok(result.into())
}

/// Create Python module for ROS compatibility
pub fn create_ros_compat_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<RosMessageConverter>()?;
    Ok(())
}
