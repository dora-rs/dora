//! Provides functions for converting between Apache Arrow arrays and Rust data types.

#![warn(missing_docs)]

use arrow::array::{
    Array, Float32Array, Float64Array, Int8Array, Int16Array, Int32Array, Int64Array, UInt8Array,
    UInt16Array, UInt32Array,
};
use arrow::datatypes::DataType;
use eyre::{ContextCompat, Result, eyre};
use num::NumCast;
use std::ops::{Deref, DerefMut};

mod from_impls;
mod into_impls;

/// Data that can be converted to an Arrow array.
///
/// This is the conversion that dora node APIs use to turn plain Rust values
/// into the [Apache Arrow](https://arrow.apache.org/) columnar format before
/// sending them as outputs. Implementations are provided for booleans,
/// strings, the primitive integer and float types, `Vec`s of those primitive
/// types, and a few `chrono` date/time types. The unit type `()` converts to
/// an empty [`NullArray`](arrow::array::NullArray), which is useful for
/// outputs that carry only metadata.
///
/// For the opposite direction (reading received Arrow data back into Rust
/// types), see the `TryFrom<&ArrowData>` implementations on [`ArrowData`].
///
/// # Example
///
/// ```
/// use arrow::array::Array;
/// use dora_arrow_convert::IntoArrow;
///
/// let array = vec![1.0_f32, 2.0, 3.0].into_arrow();
/// assert_eq!(array.len(), 3);
///
/// let single = 42_u8.into_arrow();
/// assert_eq!(single.len(), 1);
/// ```
pub trait IntoArrow {
    /// The Arrow array type that the data converts to.
    type A: Array;

    /// Convert the data into an Arrow array.
    fn into_arrow(self) -> Self::A;
}

/// Wrapper type for an Arrow [`ArrayRef`](arrow::array::ArrayRef).
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
        /// Tries to convert the given Arrow array into a `Vec` of integers or floats.
        ///
        /// Returns an error if the array contains any null values, consistent
        /// with every other [`TryFrom<&ArrowData>`] impl in this crate.
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

                        if buffer.null_count() != 0 {
                            eyre::bail!("array has nulls");
                        }

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
