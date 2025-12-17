use arrow::{
    array::{ArrayRef, AsArray},
    datatypes::{self, ArrowPrimitiveType},
};
use dora_ros2_bridge_msg_gen::types::primitives::BasicType;

pub struct SerializeWrapper<'a> {
    pub t: &'a BasicType,
    pub column: &'a ArrayRef,
}

impl serde::Serialize for SerializeWrapper<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match self.t {
            BasicType::I8 => {
                serializer.serialize_i8(as_single_primitive::<datatypes::Int8Type, _>(self.column)?)
            }
            BasicType::I16 => serializer
                .serialize_i16(as_single_primitive::<datatypes::Int16Type, _>(self.column)?),
            BasicType::I32 => serializer
                .serialize_i32(as_single_primitive::<datatypes::Int32Type, _>(self.column)?),
            BasicType::I64 => serializer
                .serialize_i64(as_single_primitive::<datatypes::Int64Type, _>(self.column)?),
            BasicType::U8 | BasicType::Char | BasicType::Byte => serializer
                .serialize_u8(as_single_primitive::<datatypes::UInt8Type, _>(self.column)?),
            BasicType::U16 => serializer
                .serialize_u16(as_single_primitive::<datatypes::UInt16Type, _>(
                    self.column,
                )?),
            BasicType::U32 => serializer
                .serialize_u32(as_single_primitive::<datatypes::UInt32Type, _>(
                    self.column,
                )?),
            BasicType::U64 => serializer
                .serialize_u64(as_single_primitive::<datatypes::UInt64Type, _>(
                    self.column,
                )?),
            BasicType::F32 => serializer
                .serialize_f32(as_single_primitive::<datatypes::Float32Type, _>(
                    self.column,
                )?),
            BasicType::F64 => serializer
                .serialize_f64(as_single_primitive::<datatypes::Float64Type, _>(
                    self.column,
                )?),
            BasicType::Bool => {
                let array = self.column.as_boolean_opt().ok_or_else(|| {
                    serde::ser::Error::custom(
                        "value is not compatible with expected `BooleanArray` type",
                    )
                })?;
                // should match the length of the outer struct
                assert_eq!(array.len(), 1);
                let field_value = array.value(0);
                serializer.serialize_bool(field_value)
            }
        }
    }
}

fn as_single_primitive<T, E>(column: &ArrayRef) -> Result<T::Native, E>
where
    T: ArrowPrimitiveType,
    E: serde::ser::Error,
{
    let array: &arrow::array::PrimitiveArray<T> = column.as_primitive_opt().ok_or_else(|| {
        serde::ser::Error::custom(format!(
            "value is not compatible with expected `{}` type",
            std::any::type_name::<T::Native>()
        ))
    })?;
    // should match the length of the outer struct
    assert_eq!(array.len(), 1);
    let number = array.value(0);
    Ok(number)
}
