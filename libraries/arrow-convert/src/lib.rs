//! Provides functions for converting between Apache Arrow arrays and Rust data types.

#![warn(missing_docs)]

use arrow::array::{
    Array, Float16Array, Float32Array, Float64Array, Int8Array, Int16Array, Int32Array, Int64Array,
    UInt8Array, UInt16Array, UInt32Array, UInt64Array,
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
    (DataType::UInt64, UInt64Array, "uint64"),
    (DataType::Float16, Float16Array, "float16"),
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::ArrayRef;
    use half::f16;
    use std::sync::Arc;

    /// Round-trips every type registered in `register_array_handlers!` so a
    /// future macro entry that gets dropped (the original cause of #2080) is
    /// caught by a failing test rather than a silent runtime error.
    #[test]
    fn into_vec_supports_all_registered_types() {
        macro_rules! assert_round_trip {
            ($array_type:ty, $rust_type:ty, $values:expr) => {{
                let values: Vec<$rust_type> = $values;
                let array: ArrayRef = Arc::new(<$array_type>::from(values.clone()));
                let data = ArrowData(array);
                let result: Vec<$rust_type> = into_vec(&data).unwrap();
                assert_eq!(result, values);
            }};
        }

        assert_round_trip!(Float32Array, f32, vec![1.0, 2.5, -3.0]);
        assert_round_trip!(Float64Array, f64, vec![1.0, 2.5, -3.0]);
        assert_round_trip!(Int8Array, i8, vec![-1, 2, 3]);
        assert_round_trip!(Int16Array, i16, vec![-1, 2, 3]);
        assert_round_trip!(Int32Array, i32, vec![-1, 2, 3]);
        assert_round_trip!(Int64Array, i64, vec![-1, 2, 3]);
        assert_round_trip!(UInt8Array, u8, vec![1, 2, 3]);
        assert_round_trip!(UInt16Array, u16, vec![1, 2, 3]);
        assert_round_trip!(UInt32Array, u32, vec![1, 2, 3]);
        assert_round_trip!(UInt64Array, u64, vec![1, 2, 3]);

        // Float16 needs explicit f16 construction; round-trip back to f16.
        let values = vec![f16::from_f32(1.0), f16::from_f32(2.5), f16::from_f32(-3.0)];
        let array: ArrayRef = Arc::new(Float16Array::from(values.clone()));
        let data = ArrowData(array);
        let result: Vec<f16> = into_vec(&data).unwrap();
        assert_eq!(result, values);
    }

    /// The case from the issue: a `UInt64` array previously errored with
    /// "Unsupported data type for conversion: UInt64".
    #[test]
    fn into_vec_handles_uint64() {
        let data = ArrowData(Arc::new(UInt64Array::from(vec![1u64, 2, 3])));
        let res: Vec<u64> = into_vec(&data).unwrap();
        assert_eq!(res, vec![1u64, 2, 3]);
    }

    #[test]
    fn into_vec_rejects_arrays_with_nulls() {
        let array: ArrayRef = Arc::new(UInt64Array::from(vec![Some(1u64), None, Some(3)]));
        let data = ArrowData(array);
        let res: Result<Vec<u64>> = into_vec(&data);
        assert!(res.is_err());
    }

    #[test]
    fn into_vec_rejects_unsupported_type() {
        let array: ArrayRef = Arc::new(arrow::array::BooleanArray::from(vec![true, false]));
        let data = ArrowData(array);
        let res: Result<Vec<u8>> = into_vec(&data);
        assert!(res.is_err());
    }
}
