use std::marker::PhantomData;

use arrow::{
    array::{Array, ArrayRef, AsArray, PrimitiveArray},
    datatypes::{self, ArrowPrimitiveType},
};
use dora_ros2_bridge_msg_gen::types::{
    primitives::{BasicType, NestableType},
    sequences,
};
use serde::ser::SerializeTuple;

use super::error;

/// Serialize an array with known size as tuple.
pub struct ArraySerializeWrapper<'a> {
    pub array_info: &'a sequences::Array,
    pub column: &'a ArrayRef,
}

impl serde::Serialize for ArraySerializeWrapper<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let entry = if let Some(list) = self.column.as_list_opt::<i32>() {
            // should match the length of the outer struct
            assert_eq!(list.len(), 1);
            list.value(0)
        } else {
            // try as large list
            let list = self
                .column
                .as_list_opt::<i64>()
                .ok_or_else(|| error("value is not compatible with expected Array type"))?;
            // should match the length of the outer struct
            assert_eq!(list.len(), 1);
            list.value(0)
        };

        match &self.array_info.value_type {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int8Type>,
                }
                .serialize(serializer),
                BasicType::I16 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int16Type>,
                }
                .serialize(serializer),
                BasicType::I32 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int32Type>,
                }
                .serialize(serializer),
                BasicType::I64 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int64Type>,
                }
                .serialize(serializer),
                BasicType::U8 | BasicType::Char | BasicType::Byte => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt8Type>,
                }
                .serialize(serializer),
                BasicType::U16 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt16Type>,
                }
                .serialize(serializer),
                BasicType::U32 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt32Type>,
                }
                .serialize(serializer),
                BasicType::U64 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt64Type>,
                }
                .serialize(serializer),
                BasicType::F32 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Float32Type>,
                }
                .serialize(serializer),
                BasicType::F64 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Float64Type>,
                }
                .serialize(serializer),
                BasicType::Bool => BoolArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                }
                .serialize(serializer),
            },
            NestableType::NamedType(_) => todo!("serializing arrays of NestableType::NamedType"),
            NestableType::NamespacedType(_) => {
                todo!("serializing arrays of NestableType::NamespacedType")
            }
            NestableType::GenericString(_) => {
                todo!("serializing arrays of NestableType::GenericString")
            }
        }
    }
}

/// Serializes a primitive array with known size as tuple.
struct BasicArrayAsTuple<'a, T> {
    len: usize,
    value: &'a ArrayRef,
    ty: PhantomData<T>,
}

impl<T> serde::Serialize for BasicArrayAsTuple<'_, T>
where
    T: ArrowPrimitiveType,
    T::Native: serde::Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut seq = serializer.serialize_tuple(self.len)?;
        let array: &PrimitiveArray<T> = self
            .value
            .as_primitive_opt()
            .ok_or_else(|| error("not a primitive array"))?;
        if array.len() != self.len {
            return Err(error(format!(
                "expected array with length {}, got length {}",
                self.len,
                array.len()
            )));
        }

        for value in array.values() {
            seq.serialize_element(value)?;
        }

        seq.end()
    }
}

struct BoolArrayAsTuple<'a> {
    len: usize,
    value: &'a ArrayRef,
}

impl serde::Serialize for BoolArrayAsTuple<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut seq = serializer.serialize_tuple(self.len)?;
        let array = self
            .value
            .as_boolean_opt()
            .ok_or_else(|| error("not a boolean array"))?;
        if array.len() != self.len {
            return Err(error(format!(
                "expected array with length {}, got length {}",
                self.len,
                array.len()
            )));
        }

        for value in array.values() {
            seq.serialize_element(&value)?;
        }

        seq.end()
    }
}
