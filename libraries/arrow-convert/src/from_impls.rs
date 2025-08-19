use arrow::{
    array::{Array, AsArray, PrimitiveArray, StringArray},
    datatypes::{ArrowPrimitiveType, ArrowTemporalType},
};
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use eyre::ContextCompat;
use half::f16;

use crate::ArrowData;

impl From<ArrowData> for arrow::array::ArrayRef {
    fn from(value: ArrowData) -> Self {
        value.0
    }
}

impl From<arrow::array::ArrayRef> for ArrowData {
    fn from(value: arrow::array::ArrayRef) -> Self {
        Self(value)
    }
}

impl TryFrom<&ArrowData> for bool {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let bool_array = value.as_boolean_opt().context("not a bool array")?;
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
            impl TryFrom<&ArrowData> for $t {
                type Error = eyre::Report;

                fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
                    let array = value
                        .as_primitive_opt::<arrow::datatypes::$arrow_type>()
                        .context(concat!("not a primitive ", stringify!($arrow_type), " array"))?;
                    extract_single_primitive(array)
                }
            }
        )*

        $(
            impl<'a> TryFrom<&'a ArrowData> for &'a [$t] {
                type Error = eyre::Report;

                fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
                    let array: &PrimitiveArray<arrow::datatypes::$arrow_type> = value
                        .as_primitive_opt()
                        .wrap_err(concat!("not a primitive ", stringify!($arrow_type), " array"))?;
                    if array.null_count() != 0 {
                        eyre::bail!("array has nulls");
                    }
                    Ok(array.values())
                }
            }
        )*

        $(
            impl<'a> TryFrom<&'a ArrowData> for Vec<$t> {
                type Error = eyre::Report;

                fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
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

impl<'a> TryFrom<&'a ArrowData> for &'a str {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &StringArray = value.as_string_opt().wrap_err("not a string array")?;
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

impl<'a> TryFrom<&'a ArrowData> for String {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &StringArray = value.as_string_opt().wrap_err("not a string array")?;
        if array.is_empty() {
            eyre::bail!("empty array");
        }
        if array.len() != 1 {
            eyre::bail!("expected length 1");
        }
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.value(0).to_string())
    }
}

impl TryFrom<&ArrowData> for NaiveDate {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        if let Some(array) = value.as_any().downcast_ref::<arrow::array::Date32Array>() {
            if check_single_datetime(array) {
                eyre::bail!("Not a valid array");
            }
            return array
                .value_as_date(0)
                .context("data type cannot be converted to NaiveDate");
        }
        let array = value
            .as_any()
            .downcast_ref::<arrow::array::Date64Array>()
            .context("Reference is neither to a Date32Array nor a Date64Array")?;
        if check_single_datetime(array) {
            eyre::bail!("Not a valid array");
        }
        array
            .value_as_date(0)
            .context("data type cannot be converted to NaiveDate")
    }
}

impl TryFrom<&ArrowData> for NaiveTime {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        if let Some(array) = value
            .as_any()
            .downcast_ref::<arrow::array::Time32SecondArray>()
        {
            if check_single_datetime(array) {
                eyre::bail!("Not a valid array");
            }
            return array
                .value_as_time(0)
                .context("data type cannot be converted to NaiveTime");
        }
        if let Some(array) = value
            .as_any()
            .downcast_ref::<arrow::array::Time32MillisecondArray>()
        {
            if check_single_datetime(array) {
                eyre::bail!("Not a valid array");
            }
            return array
                .value_as_time(0)
                .context("data type cannot be converted to NaiveTime");
        }
        if let Some(array) = value
            .as_any()
            .downcast_ref::<arrow::array::Time64MicrosecondArray>()
        {
            if check_single_datetime(array) {
                eyre::bail!("Not a valid array");
            }
            return array
                .value_as_time(0)
                .context("data type cannot be converted to NaiveTime");
        }
        let array = value
            .as_primitive_opt::<arrow::datatypes::Time64NanosecondType>()
            .context("not any of the primitive Time arrays")?;
        if check_single_datetime(array) {
            eyre::bail!("Not a valid array");
        }
        array
            .value_as_time(0)
            .context("data type cannot be converted to NaiveTime")
    }
}

impl TryFrom<&ArrowData> for NaiveDateTime {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        if let Some(array) = value
            .as_any()
            .downcast_ref::<arrow::array::TimestampSecondArray>()
        {
            if check_single_datetime(array) {
                eyre::bail!("Not a valid array");
            }
            return array
                .value_as_datetime(0)
                .context("data type cannot be converted to NaiveDateTime");
        }
        if let Some(array) = value
            .as_any()
            .downcast_ref::<arrow::array::TimestampMillisecondArray>()
        {
            if check_single_datetime(array) {
                eyre::bail!("Not a valid array");
            }
            return array
                .value_as_datetime(0)
                .context("data type cannot be converted to NaiveDateTime");
        }
        if let Some(array) = value
            .as_any()
            .downcast_ref::<arrow::array::TimestampMicrosecondArray>()
        {
            if check_single_datetime(array) {
                eyre::bail!("Not a valid array");
            }
            return array
                .value_as_datetime(0)
                .context("data type cannot be converted to NaiveDateTime");
        }
        let array = value
            .as_primitive_opt::<arrow::datatypes::TimestampNanosecondType>()
            .context("not any of the primitive Time arrays")?;
        if check_single_datetime(array) {
            eyre::bail!("Not a valid array");
        }
        array
            .value_as_datetime(0)
            .context("data type cannot be converted to NaiveDateTime")
    }
}

fn check_single_datetime<T>(array: &PrimitiveArray<T>) -> bool
where
    T: ArrowTemporalType,
{
    array.is_empty() || array.len() != 1 || array.null_count() != 0
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

    use crate::ArrowData;

    #[test]
    fn test_u8() {
        let array =
            make_array(PrimitiveArray::<arrow::datatypes::UInt8Type>::from(vec![42]).into());
        let data: ArrowData = array.into();
        let value: u8 = (&data).try_into().unwrap();
        assert_eq!(value, 42);
    }
}
