// Re-export the Arrow<->ROS2 conversion types from the shared crate.
// The actual logic lives in `adora-ros2-bridge-arrow` to be reusable
// by the standalone bridge binary.
pub use adora_ros2_bridge_arrow::*;

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use crate::Ros2Context;
    use crate::typed::TypeInfo;
    use crate::typed::deserialize::StructDeserializer;
    use crate::typed::serialize;

    use arrow::array::make_array;
    use arrow::pyarrow::FromPyArrow;
    use arrow::pyarrow::ToPyArrow;

    use eyre::eyre;
    use pyo3::ffi::c_str;
    use pyo3::types::IntoPyDict;
    use pyo3::types::PyAnyMethods;
    use pyo3::types::PyDict;
    use pyo3::types::PyList;
    use pyo3::types::PyListMethods;
    use pyo3::types::PyModule;
    use pyo3::types::PyTuple;

    use pyo3::Python;
    use serde::Serialize;
    use serde::de::DeserializeSeed;

    use serde_assert::Serializer;
    use serialize::TypedValue;

    use eyre::{Context, Result};
    use serde_assert::Deserializer;
    #[test]
    fn test_python_array_code() -> Result<()> {
        pyo3::prepare_freethreaded_python();
        let context = Ros2Context::new(None).context("Could not create a context")?;
        let messages = context.messages.clone();
        let serializer = Serializer::builder().build();

        Python::attach(|py| -> Result<()> {
            let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")); //.join("test_utils.py"); // Adjust this path as needed

            // Add the Python module's directory to sys.path
            py.run(
                c_str!("import sys; sys.path.append(str(path))"),
                Some(
                    &[("path", path)]
                        .into_py_dict(py)
                        .context("Failed to create py_dict")?,
                ),
                None,
            )?;

            let my_module = PyModule::import(py, "test_utils")?;

            let arrays = my_module.getattr("TEST_ARRAYS")?;
            let arrays = arrays
                .downcast::<PyList>()
                .map_err(|err| eyre!("Could not downcast PyAny. Err: {}", err))?;
            for array_wrapper in arrays.iter() {
                let arrays = array_wrapper.downcast::<PyTuple>().map_err(|err| {
                    eyre!("Could not downcast expected tuple test array. Err: {}", err)
                })?;
                let package_name: String = arrays.get_item(0)?.extract()?;
                let message_name: String = arrays.get_item(1)?.extract()?;
                println!("Checking {}::{}", package_name, message_name);
                let in_pyarrow = arrays.get_item(2)?;

                let array = arrow::array::ArrayData::from_pyarrow_bound(&in_pyarrow.as_borrowed())?;
                let type_info = TypeInfo {
                    package_name: package_name.into(),
                    message_name: message_name.clone().into(),
                    messages: messages.clone(),
                };
                let typed_value = TypedValue {
                    value: &make_array(array.clone()),
                    type_info: &type_info.clone(),
                };

                let typed_deserializer =
                    StructDeserializer::new(std::borrow::Cow::Owned(type_info));
                let tokens = typed_value.serialize(&serializer)?;
                let mut deserializer = Deserializer::builder(tokens).build();

                let out_value = typed_deserializer
                    .deserialize(&mut deserializer)
                    .context("could not deserialize array")?;

                let out_pyarrow = out_value.to_pyarrow(py)?;

                let test_utils = PyModule::import(py, "test_utils")?;
                let context = PyDict::new(py);

                context.set_item("test_utils", test_utils)?;
                context.set_item("in_pyarrow", in_pyarrow)?;
                context.set_item("out_pyarrow", out_pyarrow)?;

                let _ = py
                    .eval(
                        c_str!("test_utils.is_subset(in_pyarrow, out_pyarrow)"),
                        Some(&context),
                        None,
                    )
                    .context("could not check if it is a subset")?;
            }
            Ok(())
        })
    }
}
