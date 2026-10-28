use arrow::{
    array::{Array, ArrayAccessor, AsArray, PrimitiveArray},
    datatypes::{ArrowPrimitiveType, ArrowTemporalType},
};
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use eyre::ContextCompat;
use half::f16;

use crate::{DoraArray, internal::array_ref};

impl TryFrom<&DoraArray> for bool {
    type Error = eyre::Report;
    fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
        let bool_array = array_ref(value)
            .as_boolean_opt()
            .context("not a bool array")?;
        ensure_single(bool_array)?;
        Ok(bool_array.value(0))
    }
}

/// Any length, no nulls: the reverse of `IntoArrow for Vec<bool>`.
impl TryFrom<&DoraArray> for Vec<bool> {
    type Error = eyre::Report;
    fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
        let bool_array = array_ref(value)
            .as_boolean_opt()
            .context("not a bool array")?;
        ensure_no_nulls(bool_array)?;
        Ok(bool_array.values().iter().collect())
    }
}

macro_rules! impl_try_from_arrow_data {
    ($($t:ty => $arrow_type:ident),*) => {
        $(
            impl TryFrom<&DoraArray> for $t {
                type Error = eyre::Report;

                fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
                    let array = array_ref(value).as_primitive_opt::<arrow::datatypes::$arrow_type>()
                        .context(concat!("not a primitive ", stringify!($arrow_type), " array"))?;
                    extract_single_primitive(array)
                }
            }
        )*

        $(
            impl<'a> TryFrom<&'a DoraArray> for &'a [$t] {
                type Error = eyre::Report;

                fn try_from(value: &'a DoraArray) -> Result<Self, Self::Error> {
                    let array: &PrimitiveArray<arrow::datatypes::$arrow_type> = array_ref(value).as_primitive_opt()
                        .wrap_err(concat!("not a primitive ", stringify!($arrow_type), " array"))?;
                    ensure_no_nulls(array)?;
                    Ok(array.values())
                }
            }
        )*

        $(
            impl<'a> TryFrom<&'a DoraArray> for Vec<$t> {
                type Error = eyre::Report;

                fn try_from(value: &'a DoraArray) -> Result<Self, Self::Error> {
                    value
                        .try_into()
                        .map(|slice: &'a [$t]| slice.to_vec())
                }
            }
        )*
    };
}

impl_try_from_arrow_data!(
    u8 => UInt8Type,
    u16 => UInt16Type,
    u32 => UInt32Type,
    u64 => UInt64Type,
    i8 => Int8Type,
    i16 => Int16Type,
    i32 => Int32Type,
    i64 => Int64Type,
    f16 => Float16Type,
    f32 => Float32Type,
    f64 => Float64Type
);

/// Apply the generic `$read` function to `$value` as whichever of the three
/// Arrow string encodings it holds: `Utf8`, `LargeUtf8` or `Utf8View`.
/// Senders outside Rust (Polars, pyarrow) commonly produce the latter two, and
/// this crate's own `IntoArrow` builds them too.
macro_rules! read_any_string_array {
    ($value:expr, $read:ident) => {{
        let array = array_ref($value);
        if let Some(array) = array.as_string_opt::<i32>() {
            $read(array)
        } else if let Some(array) = array.as_string_opt::<i64>() {
            $read(array)
        } else if let Some(array) = array.as_string_view_opt() {
            $read(array)
        } else {
            eyre::bail!(
                "not a string array (expected Utf8, LargeUtf8 or Utf8View, got {})",
                array.data_type()
            )
        }
    }};
}

impl<'a> TryFrom<&'a DoraArray> for &'a str {
    type Error = eyre::Report;
    /// Accepts all three Arrow string encodings: `Utf8`, `LargeUtf8` and
    /// `Utf8View`.
    fn try_from(value: &'a DoraArray) -> Result<Self, Self::Error> {
        read_any_string_array!(value, single_string)
    }
}

fn single_string<'a, A>(array: &'a A) -> eyre::Result<&'a str>
where
    A: Array,
    &'a A: ArrayAccessor<Item = &'a str>,
{
    ensure_single(array)?;
    Ok(array.value(0))
}

/// Any length, no nulls: the reverse of `IntoArrow for Vec<String>`. Like
/// the `&str` impl, accepts `Utf8`, `LargeUtf8` and `Utf8View`.
impl TryFrom<&DoraArray> for Vec<String> {
    type Error = eyre::Report;
    fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
        read_any_string_array!(value, all_strings)
    }
}

fn all_strings<'a, A>(array: &'a A) -> eyre::Result<Vec<String>>
where
    A: Array,
    &'a A: ArrayAccessor<Item = &'a str>,
{
    ensure_no_nulls(array)?;
    Ok((0..array.len())
        .map(|i| array.value(i).to_owned())
        .collect())
}

impl TryFrom<&DoraArray> for String {
    type Error = eyre::Report;
    fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
        // Delegate to the `&str` impl so the single-element validation lives in
        // one place and the two conversions can never drift apart.
        let s: &str = value.try_into()?;
        Ok(s.to_string())
    }
}

impl TryFrom<&DoraArray> for NaiveDate {
    type Error = eyre::Report;
    fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
        const CTX: &str = "data type cannot be converted to NaiveDate";
        if let Some(array) = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::Date32Array>()
        {
            return single_temporal(array, |a, i| a.value_as_date(i), CTX);
        }
        let array = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::Date64Array>()
            .context("Reference is neither to a Date32Array nor a Date64Array")?;
        single_temporal(array, |a, i| a.value_as_date(i), CTX)
    }
}

impl TryFrom<&DoraArray> for NaiveTime {
    type Error = eyre::Report;
    fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
        const CTX: &str = "data type cannot be converted to NaiveTime";
        if let Some(array) = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::Time32SecondArray>()
        {
            return single_temporal(array, |a, i| a.value_as_time(i), CTX);
        }
        if let Some(array) = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::Time32MillisecondArray>()
        {
            return single_temporal(array, |a, i| a.value_as_time(i), CTX);
        }
        if let Some(array) = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::Time64MicrosecondArray>()
        {
            return single_temporal(array, |a, i| a.value_as_time(i), CTX);
        }
        let array = array_ref(value)
            .as_primitive_opt::<arrow::datatypes::Time64NanosecondType>()
            .context("not any of the primitive Time arrays")?;
        single_temporal(array, |a, i| a.value_as_time(i), CTX)
    }
}

impl TryFrom<&DoraArray> for NaiveDateTime {
    type Error = eyre::Report;
    fn try_from(value: &DoraArray) -> Result<Self, Self::Error> {
        const CTX: &str = "data type cannot be converted to NaiveDateTime";
        if let Some(array) = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::TimestampSecondArray>()
        {
            return single_temporal(array, |a, i| a.value_as_datetime(i), CTX);
        }
        if let Some(array) = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::TimestampMillisecondArray>()
        {
            return single_temporal(array, |a, i| a.value_as_datetime(i), CTX);
        }
        if let Some(array) = array_ref(value)
            .as_any()
            .downcast_ref::<arrow::array::TimestampMicrosecondArray>()
        {
            return single_temporal(array, |a, i| a.value_as_datetime(i), CTX);
        }
        let array = array_ref(value)
            .as_primitive_opt::<arrow::datatypes::TimestampNanosecondType>()
            .context("not any of the primitive Timestamp arrays")?;
        single_temporal(array, |a, i| a.value_as_datetime(i), CTX)
    }
}

/// Extract the single scalar out of a length-1, non-null temporal array.
///
/// Every temporal `TryFrom<&DoraArray>` arm above shares the exact same shape:
/// validate that the array holds exactly one non-null element, then convert
/// element 0 via one of arrow's `value_as_{date,time,datetime}` accessors.
/// Centralizing it here removes ~8 near-verbatim copies and keeps the
/// validation and error text from drifting between them.
fn single_temporal<T, R>(
    array: &PrimitiveArray<T>,
    convert: impl Fn(&PrimitiveArray<T>, usize) -> Option<R>,
    context: &'static str,
) -> Result<R, eyre::Error>
where
    T: ArrowTemporalType,
{
    ensure_single(array)?;
    convert(array, 0).context(context)
}

fn extract_single_primitive<T>(array: &PrimitiveArray<T>) -> Result<T::Native, eyre::Error>
where
    T: ArrowPrimitiveType,
{
    ensure_single(array)?;
    Ok(array.value(0))
}

/// Check that a scalar conversion's source holds exactly one non-null
/// element, with the error text every scalar `TryFrom<&DoraArray>` shares.
fn ensure_single(array: &dyn Array) -> eyre::Result<()> {
    if array.is_empty() {
        eyre::bail!("empty array");
    }
    if array.len() != 1 {
        eyre::bail!("expected length 1");
    }
    ensure_no_nulls(array)
}

/// Check that a conversion's source holds no null element.
fn ensure_no_nulls(array: &dyn Array) -> eyre::Result<()> {
    if array.null_count() != 0 {
        eyre::bail!("array has nulls");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use arrow::array::{Array, PrimitiveArray, make_array};

    use crate::{DoraArray, internal::from_array_ref};

    #[test]
    fn test_u8() {
        let array =
            make_array(PrimitiveArray::<arrow::datatypes::UInt8Type>::from(vec![42]).into());
        let data: DoraArray = from_array_ref(array);
        let value: u8 = (&data).try_into().unwrap();
        assert_eq!(value, 42);
    }

    /// #3661: every Arrow string encoding converts to `&str` and `String`,
    /// not only `Utf8`.
    #[test]
    fn test_string_encodings() {
        use arrow::array::{ArrayRef, LargeStringArray, StringArray, StringViewArray};
        use std::sync::Arc;

        let arrays: [ArrayRef; 3] = [
            Arc::new(StringArray::from(vec!["hello"])),
            Arc::new(LargeStringArray::from(vec!["hello"])),
            Arc::new(StringViewArray::from(vec!["hello"])),
        ];
        for array in arrays {
            let data_type = array.data_type().clone();
            let data: DoraArray = from_array_ref(array);
            let s: &str = (&data).try_into().unwrap();
            assert_eq!(s, "hello", "{data_type}");
            let s: String = (&data).try_into().unwrap();
            assert_eq!(s, "hello", "{data_type}");
        }
    }

    /// The single-element / no-null checks apply to every encoding.
    #[test]
    fn test_string_encodings_reject_invalid_shapes() {
        use arrow::array::{ArrayRef, LargeStringArray, StringViewArray};
        use std::sync::Arc;

        let cases: [(ArrayRef, &str); 6] = [
            (
                Arc::new(LargeStringArray::from(Vec::<&str>::new())),
                "empty",
            ),
            (Arc::new(LargeStringArray::from(vec!["a", "b"])), "length 1"),
            (
                Arc::new(LargeStringArray::from(vec![None::<&str>])),
                "nulls",
            ),
            (Arc::new(StringViewArray::from(Vec::<&str>::new())), "empty"),
            (Arc::new(StringViewArray::from(vec!["a", "b"])), "length 1"),
            (Arc::new(StringViewArray::from(vec![None::<&str>])), "nulls"),
        ];
        for (array, expected) in cases {
            let data: DoraArray = from_array_ref(array);
            let err = <&str>::try_from(&data).unwrap_err().to_string();
            assert!(err.contains(expected), "{err}");
        }
    }

    #[test]
    fn test_string_rejects_non_string() {
        let array =
            make_array(PrimitiveArray::<arrow::datatypes::UInt8Type>::from(vec![42]).into());
        let data: DoraArray = from_array_ref(array);
        let err = String::try_from(&data).unwrap_err().to_string();
        assert!(err.contains("not a string array"), "{err}");
        assert!(err.contains("UInt8"), "{err}");
    }

    /// The temporal and `bool` scalars reject a bad shape with the same three
    /// messages as the other scalars, not a catch-all "Not a valid array".
    #[test]
    fn test_scalar_shape_errors_are_specific() {
        use arrow::array::{BooleanArray, Date32Array, Time64NanosecondArray};
        use chrono::{NaiveDate, NaiveTime};

        fn err<T: for<'a> TryFrom<&'a DoraArray, Error = eyre::Report>>(
            array: impl Array + 'static,
        ) -> String {
            match T::try_from(&from_array_ref(make_array(array.into_data()))) {
                Ok(_) => panic!("expected an error"),
                Err(e) => e.to_string(),
            }
        }

        assert_eq!(
            err::<NaiveDate>(Date32Array::from(Vec::<i32>::new())),
            "empty array"
        );
        assert_eq!(
            err::<NaiveDate>(Date32Array::from(vec![1, 2])),
            "expected length 1"
        );
        assert_eq!(
            err::<NaiveDate>(Date32Array::from(vec![None])),
            "array has nulls"
        );
        assert_eq!(
            err::<NaiveTime>(Time64NanosecondArray::from(vec![None])),
            "array has nulls"
        );
        assert_eq!(
            err::<bool>(BooleanArray::from(vec![true, false])),
            "expected length 1"
        );
        assert_eq!(
            err::<bool>(BooleanArray::from(vec![None])),
            "array has nulls"
        );
    }
}
