use arrow::array::{
    Array, Float32Array, Float64Array, Int16Array, Int32Array, Int64Array, Int8Array, UInt16Array,
    UInt32Array, UInt8Array,
};
use arrow::datatypes::DataType;
use eyre::{eyre, ContextCompat, Result};
use num::NumCast;
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
        pub fn into_vec<T>(data: &ArrowData) -> Result<Vec<T>>
        where
            T: Copy + NumCast + 'static,
        {
            match data.data_type() {
                $(
                    $variant => {
                        let buffer: &$array_type = data
                            .as_any()
                            .downcast_ref()
                            .context(concat!("series is not ", $type_name))?;

                        let mut result = Vec::with_capacity(buffer.len());
                        for &v in buffer.values() {
                            let converted = NumCast::from(v).context(format!("Failed to cast value from {} to target type",$type_name))?;
                            result.push(converted);
                        }
                        Ok(result)
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
    (DataType::Int8, Int8Array, "int8"),
    (DataType::Int16, Int16Array, "int16"),
    (DataType::Int32, Int32Array, "int32"),
    (DataType::Int64, Int64Array, "int64"),
    (DataType::UInt8, UInt8Array, "uint8"),
    (DataType::UInt16, UInt16Array, "uint16"),
    (DataType::UInt32, UInt32Array, "uint32"),
}
