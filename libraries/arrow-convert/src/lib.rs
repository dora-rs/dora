//! Provides functions for converting between Apache Arrow arrays and Rust data types.
//!
//! # Arrow version decoupling
//!
//! The types in this crate are re-exported from `dora-node-api`, whose public
//! API is frozen at 1.0. To keep dora free to bump its *internal* Arrow major
//! in a minor release, no Arrow type appears in the ungated public surface
//! here: received payloads are [`DoraArray`], a dora-owned newtype with a
//! private field, and [`IntoArrow::into_arrow`] returns that same type.
//!
//! Direct access to the underlying Arrow array is available, but only behind a
//! feature that names the major explicitly:
//!
//! | Feature | Effect |
//! |---|---|
//! | `arrow-v59` | dora's *internal* major. Adds the borrowing accessors [`DoraArray::as_array`] / [`DoraArray::into_inner`] and `From`/`Into` for `arrow::array::ArrayRef`. Pulls **no extra dependency** — it re-exports the copy of Arrow 59 dora already links. |
//! | `arrow-v58` | An *older* major. Adds an aliased `arrow58` dependency plus `TryFrom` impls in both directions, which hop across the Arrow C Data Interface (zero-copy, see the `ffi_bridge` module). The hop is fallible, hence `TryFrom` rather than `From`. |
//!
//! `default = []`, so neither is on unless asked for. See
//! `docs/plan-arrow-version-decoupling.md` and the support-window policy in
//! `docs/api-rust.md`.

#![warn(missing_docs)]

use arrow::array::{
    Array, Float16Array, Float32Array, Float64Array, Int8Array, Int16Array, Int32Array, Int64Array,
    UInt8Array, UInt16Array, UInt32Array, UInt64Array,
};
use arrow::datatypes::DataType;
use eyre::{ContextCompat, Result, eyre};
use num::NumCast;

#[cfg(feature = "arrow-v58")]
pub mod ffi_bridge;
mod from_impls;
pub mod internal;
mod into_impls;

/// Data that can be converted into a dora payload.
///
/// This is the conversion that dora node APIs use to turn plain Rust values
/// into the [Apache Arrow](https://arrow.apache.org/) columnar format before
/// sending them as outputs. Implementations are provided for booleans,
/// strings, the primitive integer and float types, `Vec`s of those primitive
/// types, and a few `chrono` date/time types. The unit type `()` converts to
/// an empty null array, which is useful for outputs that carry only metadata.
/// [`DoraArray`] itself implements the trait as the identity conversion, so
/// `send_output` accepts both plain Rust values and already-built payloads.
///
/// The trait deliberately has **no associated type**: an
/// `type A: arrow::array::Array` bound would put Arrow back into dora's frozen
/// public API and pin 1.x to a single Arrow major. `into_arrow` returns the
/// dora-owned [`DoraArray`] instead.
///
/// For the opposite direction (reading received Arrow data back into Rust
/// types), see the `TryFrom<&DoraArray>` implementations on [`DoraArray`].
///
/// # Example
///
/// ```
/// use dora_arrow_convert::IntoArrow;
///
/// let array = vec![1.0_f32, 2.0, 3.0].into_arrow();
/// assert_eq!(array.len(), 3);
///
/// let single = 42_u8.into_arrow();
/// assert_eq!(single.len(), 1);
/// ```
pub trait IntoArrow {
    /// Convert the data into a dora payload.
    fn into_arrow(self) -> DoraArray;
}

/// A dora payload: an Apache Arrow array owned by dora.
///
/// `DoraArray` is the counterpart to [`IntoArrow`]: dora node APIs hand
/// received outputs to nodes as `DoraArray`, which is read back into plain
/// Rust values through its `TryFrom<&DoraArray>` implementations.
///
/// The wrapped Arrow array is **private**. That is the whole point of the
/// type: it is what lets dora change its internal Arrow major without
/// breaking the 1.x contract. To reach the Arrow array itself, enable the
/// feature naming the major you want — see the crate-level docs.
///
/// Two conversion shapes are provided, with different length contracts:
///
/// - **Scalar** conversions (`bool`, the primitive integer/float types,
///   `String`, `&str`, and the `chrono` date/time types) require the array to
///   hold **exactly one element and no nulls**; any other length is an error.
/// - **Slice / `Vec`** conversions (`&[T]` and `Vec<T>` for the primitive
///   types) accept **any length** but still reject **any null values**.
///
/// # Example
///
/// ```
/// use dora_arrow_convert::{DoraArray, IntoArrow};
///
/// // Scalar: a single-element array converts to the value.
/// let data: DoraArray = 42_u8.into_arrow();
/// let scalar: u8 = (&data).try_into()?;
/// assert_eq!(scalar, 42);
///
/// // Strings follow the same single-element rule.
/// let data = "hello".to_string().into_arrow();
/// let text: String = (&data).try_into()?;
/// assert_eq!(text, "hello");
///
/// // Vec: any length, collected into an owned `Vec`.
/// let data = vec![1_i32, 2, 3].into_arrow();
/// let values: Vec<i32> = (&data).try_into()?;
/// assert_eq!(values, vec![1, 2, 3]);
///
/// // A multi-element array cannot be read as a scalar.
/// let data = vec![1_i32, 2, 3].into_arrow();
/// let scalar: Result<i32, _> = (&data).try_into();
/// assert!(scalar.is_err());
/// # Ok::<(), eyre::Report>(())
/// ```
#[derive(Debug, Clone)]
pub struct DoraArray(arrow::array::ArrayRef);

impl DoraArray {
    /// The number of elements in the payload.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether the payload holds no elements.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The number of null elements in the payload.
    pub fn null_count(&self) -> usize {
        self.0.null_count()
    }

    /// A human-readable name for the payload's Arrow type, e.g. `"UInt8"` or
    /// `"List(Field { name: \"item\", .. })"`.
    ///
    /// Returned as a `String` rather than an `arrow_schema::DataType` so that
    /// the ungated surface stays free of Arrow types. Use it for logging and
    /// error messages; to actually inspect the type, take the array through a
    /// version-gated accessor.
    pub fn type_name(&self) -> String {
        format!("{:?}", self.0.data_type())
    }
}

/// Borrowing access to the Arrow array, for callers on dora's **internal**
/// Arrow major.
///
/// This is free: no conversion, no allocation, just a reference to the array
/// dora already holds. When dora later moves internally to Arrow 60, this
/// borrowing pair re-gates behind `arrow-v60` and `arrow-v59` keeps a
/// *converting* `TryFrom` pair instead — which is exactly what `arrow-v58`
/// already looks like today (see the `ffi_bridge` module).
#[cfg(feature = "arrow-v59")]
impl DoraArray {
    /// Borrow the underlying Arrow 59 array.
    pub fn as_array(&self) -> &arrow::array::ArrayRef {
        &self.0
    }

    /// Take the underlying Arrow 59 array.
    pub fn into_inner(self) -> arrow::array::ArrayRef {
        self.0
    }

    /// Build a payload from any Arrow 59 array.
    pub fn from_array(array: impl arrow::array::Array + 'static) -> Self {
        Self(arrow::array::make_array(array.to_data()))
    }
}

#[cfg(feature = "arrow-v59")]
impl From<arrow::array::ArrayRef> for DoraArray {
    fn from(value: arrow::array::ArrayRef) -> Self {
        Self(value)
    }
}

#[cfg(feature = "arrow-v59")]
impl From<DoraArray> for arrow::array::ArrayRef {
    fn from(value: DoraArray) -> Self {
        value.0
    }
}

impl IntoArrow for DoraArray {
    fn into_arrow(self) -> DoraArray {
        self
    }
}

macro_rules! register_array_handlers {
    ($(($variant:path, $array_type:ty, $type_name:expr)),* $(,)?) => {
        /// Tries to convert the given payload into a `Vec` of integers or floats.
        ///
        /// The array's element type is cast to `T` per element via [`num::NumCast`],
        /// so the source and target types need not match (e.g. a `UInt64Array`
        /// into a `Vec<f64>`).
        ///
        /// # Errors
        ///
        /// Returns an error if the array contains any null values (consistent
        /// with every other [`TryFrom<&DoraArray>`] impl in this crate), if the
        /// array's data type is not a supported integer or float type, or if any
        /// element cannot be represented in `T` (an out-of-range cast).
        ///
        /// ```
        /// use dora_arrow_convert::{IntoArrow, into_vec};
        ///
        /// // Values are cast element-wise to the requested target type.
        /// let data = vec![1u64, 2, 3].into_arrow();
        /// assert_eq!(into_vec::<u64>(&data).ok(), Some(vec![1, 2, 3]));
        /// assert_eq!(into_vec::<f64>(&data).ok(), Some(vec![1.0, 2.0, 3.0]));
        ///
        /// // Unsupported (non-numeric) array types are rejected.
        /// let strings = vec!["a".to_string(), "b".to_string()].into_arrow();
        /// assert!(into_vec::<u64>(&strings).is_err());
        /// ```
        pub fn into_vec<T>(data: &DoraArray) -> Result<Vec<T>>
        where
            T: Copy + NumCast + 'static,
        {
            match data.0.data_type() {
                $(
                    $variant => {
                        let buffer: &$array_type = data
                            .0
                            .as_any()
                            .downcast_ref()
                            .context(concat!("series is not ", $type_name))?;

                        if buffer.null_count() != 0 {
                            eyre::bail!("array has nulls");
                        }

                        let mut result = Vec::with_capacity(buffer.len());
                        for &v in buffer.values() {
                            // `with_context` defers the `format!` to the error
                            // path: `context(format!(...))` would heap-allocate a
                            // fresh error String on every element even on the
                            // (overwhelmingly common) success path.
                            let converted = NumCast::from(v).with_context(|| {
                                format!("Failed to cast value from {} to target type", $type_name)
                            })?;
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

    fn wrap(array: ArrayRef) -> DoraArray {
        internal::from_array_ref(array)
    }

    /// Round-trips every type registered in `register_array_handlers!` so a
    /// future macro entry that gets dropped (the original cause of #2080) is
    /// caught by a failing test rather than a silent runtime error.
    #[test]
    fn into_vec_supports_all_registered_types() {
        macro_rules! assert_round_trip {
            ($array_type:ty, $rust_type:ty, $values:expr) => {{
                let values: Vec<$rust_type> = $values;
                let array: ArrayRef = Arc::new(<$array_type>::from(values.clone()));
                let data = wrap(array);
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
        let data = wrap(array);
        let result: Vec<f16> = into_vec(&data).unwrap();
        assert_eq!(result, values);
    }

    /// The case from the issue: a `UInt64` array previously errored with
    /// "Unsupported data type for conversion: UInt64".
    #[test]
    fn into_vec_handles_uint64() {
        let data = wrap(Arc::new(UInt64Array::from(vec![1u64, 2, 3])));
        let res: Vec<u64> = into_vec(&data).unwrap();
        assert_eq!(res, vec![1u64, 2, 3]);
    }

    #[test]
    fn into_vec_rejects_arrays_with_nulls() {
        let array: ArrayRef = Arc::new(UInt64Array::from(vec![Some(1u64), None, Some(3)]));
        let data = wrap(array);
        let res: Result<Vec<u64>> = into_vec(&data);
        assert!(res.is_err());
    }

    #[test]
    fn into_vec_rejects_unsupported_type() {
        let array: ArrayRef = Arc::new(arrow::array::BooleanArray::from(vec![true, false]));
        let data = wrap(array);
        let res: Result<Vec<u8>> = into_vec(&data);
        assert!(res.is_err());
    }

    /// `DoraArray`'s ungated inspection helpers must not require any Arrow
    /// feature — they are the whole reason the common receive path compiles
    /// with `default = []`.
    #[test]
    fn ungated_accessors() {
        let data = vec![1u64, 2, 3].into_arrow();
        assert_eq!(data.len(), 3);
        assert!(!data.is_empty());
        assert_eq!(data.null_count(), 0);
        assert_eq!(data.type_name(), "UInt64");
    }
}
