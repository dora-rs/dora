use arrow::{
    array::{Array, AsArray, PrimitiveArray, StringArray},
    datatypes::ArrowPrimitiveType,
};
use chrono::{NaiveDate, NaiveTime};

use arrow_convert::deserialize::TryIntoCollection;
use eyre::ContextCompat;
use std::sync::Arc;

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
impl TryFrom<&ArrowData> for u8 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::UInt8Type>()
            .context("not a primitive UInt8Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for u16 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::UInt16Type>()
            .context("not a primitive UInt16Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for u32 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::UInt32Type>()
            .context("not a primitive UInt32Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for u64 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::UInt64Type>()
            .context("not a primitive UInt64Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for i8 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Int8Type>()
            .context("not a primitive Int8Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for i16 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Int16Type>()
            .context("not a primitive Int16Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for i32 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Int32Type>()
            .context("not a primitive Int32Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for i64 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Int64Type>()
            .context("not a primitive Int64Type array")?;
        extract_single_primitive(array)
    }
}

impl TryFrom<&ArrowData> for f32 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Float32Type>()
            .context("not a primitive Float32Type array")?;
        extract_single_primitive(array)
    }
}
impl TryFrom<&ArrowData> for f64 {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Float64Type>()
            .context("not a primitive Float64Type array")?;
        extract_single_primitive(array)
    }
}

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

impl<'a> TryFrom<&'a ArrowData> for &'a [u8] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::UInt8Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive UInt8Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<u8> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [u8]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [u16] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::UInt16Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive UInt16Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<u16> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [u16]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [u32] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::UInt32Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive UInt32Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<u32> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [u32]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [u64] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::UInt64Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive UInt64Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<u64> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [u64]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [i8] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::Int8Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive Int8Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<i8> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [i8]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [i16] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::Int16Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive Int16Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<i16> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [i16]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [i32] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::Int32Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive Int32Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<i32> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [i32]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [i64] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::Int64Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive Int64Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<i64> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [i64]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [f32] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::Float32Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive Float32Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<f32> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [f32]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for &'a [f64] {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let array: &PrimitiveArray<arrow::datatypes::Float64Type> = value
            .as_primitive_opt()
            .wrap_err("not a primitive Float64Type array")?;
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.values())
    }
}

impl<'a> TryFrom<&'a ArrowData> for Vec<f64> {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        value.try_into().map(|slice: &'a [f64]| slice.to_vec())
    }
}

impl<'a> TryFrom<&'a ArrowData> for String {
    type Error = eyre::Report;
    fn try_from(value: &'a ArrowData) -> Result<Self, Self::Error> {
        let string_array: Vec<String> =
            <Arc<dyn arrow::array::Array> as Clone>::clone(&value).try_into_collection()?;
        return Ok(string_array[0].clone());
    }
}


impl TryFrom<&ArrowData> for NaiveDate {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Date32Type>()
            .context("not a primitive Date32Type array")?;
        if array.is_empty() {
            eyre::bail!("empty array");
        }
        if array.len() != 1 {
            eyre::bail!("expected length 1");
        }
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.value_as_date(0).context("data type cannot be converted to NaiveDate")?)
    }
}

impl TryFrom<&ArrowData> for NaiveTime {
    type Error = eyre::Report;
    fn try_from(value: &ArrowData) -> Result<Self, Self::Error> {
        let array = value
            .as_primitive_opt::<arrow::datatypes::Time64NanosecondType>()
            .context("not a primitive Time64NanosecondType array")?;
        if array.is_empty() {
            eyre::bail!("empty array");
        }
        if array.len() != 1 {
            eyre::bail!("expected length 1");
        }
        if array.null_count() != 0 {
            eyre::bail!("array has nulls");
        }
        Ok(array.value_as_time(0).context("data type cannot be converted to NaiveTime")?)
    }
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
    use arrow::array::{make_array, PrimitiveArray};

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
