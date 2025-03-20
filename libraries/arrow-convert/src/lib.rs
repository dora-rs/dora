use arrow::array::{Array, Float32Array, Float64Array, Int32Array, Int64Array, UInt32Array};
use arrow::datatypes::DataType;
use eyre::{eyre, ContextCompat, Result};
use std::ops::{Deref, DerefMut};

mod from_impls;
mod into_impls;

pub trait IntoArrow {
    type A: Array;

    fn into_arrow(self) -> Self::A;
}

#[derive(Debug)]
pub struct ArrowData(pub arrow::array::ArrayRef);

impl Deref for ArrowData {
    type Target = arrow::array::ArrayRef;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ArrowData {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

macro_rules! register_array_handlers {
    ($(($variant:path, $array_type:ty, $type_name:expr)),* $(,)?) => {
        pub fn into_vec_f64(data: &ArrowData) -> Result<Vec<f64>> {
            match data.data_type() {
                $(
                    $variant => {
                        let buffer: &$array_type = data
                            .as_any()
                            .downcast_ref()
                            .context(concat!("series is not ", $type_name))?;
                        Ok(buffer.values().iter().map(|&v| v as f64).collect())
                    }
                ),*
                // Error handling for unsupported types
                unsupported_type => Err(eyre!("Unsupported data type for conversion: {:?}", unsupported_type))
            }
        }
    };
}

// Register all supported array types in one place
register_array_handlers! {
    (DataType::Float32, Float32Array, "float32"),
    (DataType::Float64, Float64Array, "float64"),
    (DataType::Int32, Int32Array, "int32"),
    (DataType::Int64, Int64Array, "int64"),
    (DataType::UInt32, UInt32Array, "uint32"),
}
