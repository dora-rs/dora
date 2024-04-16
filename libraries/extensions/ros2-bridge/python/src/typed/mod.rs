use dora_ros2_bridge_msg_gen::types::Message;
use std::{borrow::Cow, collections::HashMap, sync::Arc};

pub use serialize::TypedValue;

pub mod deserialize;
pub mod serialize;

#[derive(Debug, Clone)]
pub struct TypeInfo<'a> {
    pub package_name: Cow<'a, str>,
    pub message_name: Cow<'a, str>,
    pub messages: Arc<HashMap<String, HashMap<String, Message>>>,
}

/// Serde requires that struct and field names are known at
/// compile time with a `'static` lifetime, which is not
/// possible in this case. Thus, we need to use dummy names
/// instead.
///
/// The actual names do not really matter because
/// the CDR format of ROS2 does not encode struct or field
/// names.
const DUMMY_STRUCT_NAME: &str = "struct";

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use crate::typed::deserialize::StructDeserializer;
    use crate::typed::serialize;
    use crate::typed::TypeInfo;
    use crate::Ros2Context;

    use arrow::array::make_array;
    use arrow::pyarrow::FromPyArrow;
    use arrow::pyarrow::ToPyArrow;

    use pyo3::types::IntoPyDict;
    use pyo3::types::PyAnyMethods;
    use pyo3::types::PyDict;
    use pyo3::types::PyList;
    use pyo3::types::PyModule;
    use pyo3::types::PyTuple;
    use pyo3::PyNativeType;
    use pyo3::Python;
    use serde::de::DeserializeSeed;
    use serde::Serialize;

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

        Python::with_gil(|py| -> Result<()> {
            let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")); //.join("test_utils.py"); // Adjust this path as needed

            // Add the Python module's directory to sys.path
            py.run_bound(
                "import sys; sys.path.append(str(path))",
                Some(&[("path", path)].into_py_dict_bound(py)),
                None,
            )?;

            let my_module = PyModule::import_bound(py, "test_utils")?;

            let arrays: &PyList = my_module.getattr("TEST_ARRAYS")?.extract()?;
            for array_wrapper in arrays.iter() {
                let arrays: &PyTuple = array_wrapper.extract()?;
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

                let test_utils = PyModule::import_bound(py, "test_utils")?;
                let context = PyDict::new_bound(py);

                context.set_item("test_utils", test_utils)?;
                context.set_item("in_pyarrow", in_pyarrow)?;
                context.set_item("out_pyarrow", out_pyarrow)?;

                let _ = py
                    .eval_bound(
                        "test_utils.is_subset(in_pyarrow, out_pyarrow)",
                        Some(&context),
                        None,
                    )
                    .context("could not check if it is a subset")?;
            }
            Ok(())
        })
    }
}
