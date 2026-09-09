use arrow::{
    array::{Array, AsArray, PrimitiveArray, StringArray},
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
        if bool_array.is_empty() {
            eyre::bail!("empty array");
        }
        if bool_array.len() != 1 {
            eyre::bail!("expected length 1");
        }
        if bool_array.null_count() != 0 {
            eyre::bail!("bool array has nulls");
        }
        Ok(bool_array.value(0))
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
                    if array.null_count() != 0 {
                        eyre::bail!("array has nulls");
                    }
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

impl<'a> TryFrom<&'a DoraArray> for &'a str {
    type Error = eyre::Report;
    fn try_from(value: &'a DoraArray) -> Result<Self, Self::Error> {
        let array: &StringArray = array_ref(value)
            .as_string_opt()
            .wrap_err("not a string array")?;
        if array.is_empty() {
            eyre::bail!("empty array");
        }
        if array.len() != 1 {
            eyre::bail!("expected length 1");
        }
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.value(0))
    }
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
    if check_single_datetime(array) {
        eyre::bail!("Not a valid array");
    }
    convert(array, 0).context(context)
}

fn check_single_datetime<T>(array: &PrimitiveArray<T>) -> bool
where
    T: ArrowTemporalType,
{
    // `len() != 1` already rejects the empty (`len() == 0`) case, so no separate
    // `is_empty()` term is needed — unlike the primitive/`&str`/`bool` helpers,
    // which branch on `is_empty()` to emit a distinct "empty array" message.
    array.len() != 1 || array.null_count() != 0
}
fn extract_single_primitive<T>(array: &PrimitiveArray<T>) -> Result<T::Native, eyre::Error>
where
    T: ArrowPrimitiveType,
{
    if array.is_empty() {
        eyre::bail!("empty array");
    }
    if array.len() != 1 {
        eyre::bail!("expected length 1");
    }
    if array.null_count() != 0 {
        eyre::bail!("array has nulls");
    }
    Ok(array.value(0))
}

#[cfg(test)]
mod tests {
    use arrow::array::{PrimitiveArray, make_array};

    use crate::{DoraArray, internal::from_array_ref};

    #[test]
    fn test_u8() {
        let array =
            make_array(PrimitiveArray::<arrow::datatypes::UInt8Type>::from(vec![42]).into());
        let data: DoraArray = from_array_ref(array);
        let value: u8 = (&data).try_into().unwrap();
        assert_eq!(value, 42);
    }
}
