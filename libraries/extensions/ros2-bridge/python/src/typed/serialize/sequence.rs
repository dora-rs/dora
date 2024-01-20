use std::marker::PhantomData;

use arrow::{
    array::{Array, ArrayRef, AsArray, PrimitiveArray},
    datatypes::{self, ArrowPrimitiveType},
};
use dora_ros2_bridge_msg_gen::types::primitives::{BasicType, NestableType};
use serde::ser::{SerializeSeq, SerializeTuple};

use super::error;

/// Serialize a variable-sized sequence.
pub struct SequenceSerializeWrapper<'a> {
    pub item_type: &'a NestableType,
    pub column: &'a ArrayRef,
}

impl serde::Serialize for SequenceSerializeWrapper<'_> {
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
        match &self.item_type {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int8Type>,
                }
                .serialize(serializer),
                BasicType::I16 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int16Type>,
                }
                .serialize(serializer),
                BasicType::I32 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int32Type>,
                }
                .serialize(serializer),
                BasicType::I64 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int64Type>,
                }
                .serialize(serializer),
                BasicType::U8 | BasicType::Char | BasicType::Byte => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt8Type>,
                }
                .serialize(serializer),
                BasicType::U16 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt16Type>,
                }
                .serialize(serializer),
                BasicType::U32 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt32Type>,
                }
                .serialize(serializer),
                BasicType::U64 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt64Type>,
                }
                .serialize(serializer),
                BasicType::F32 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Float32Type>,
                }
                .serialize(serializer),
                BasicType::F64 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Float64Type>,
                }
                .serialize(serializer),
                BasicType::Bool => BoolArray { value: &entry }.serialize(serializer),
            },
            NestableType::NamedType(_) => todo!("serializing NestableType::NamedType sequences"),
            NestableType::NamespacedType(_) => {
                todo!("serializing NestableType::NamespacedType sequences")
            }
            NestableType::GenericString(_) => {
                todo!("serializing NestableType::Genericstring sequences")
            }
        }
    }
}

struct BasicSequence<'a, T> {
    value: &'a ArrayRef,
    ty: PhantomData<T>,
}

impl<T> serde::Serialize for BasicSequence<'_, T>
where
    T: ArrowPrimitiveType,
    T::Native: serde::Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let array: &PrimitiveArray<T> = self
            .value
            .as_primitive_opt()
            .ok_or_else(|| error("not a primitive array"))?;

        let mut seq = serializer.serialize_seq(Some(array.len()))?;

        for value in array.values() {
            seq.serialize_element(value)?;
        }

        seq.end()
    }
}

struct BoolArray<'a> {
    value: &'a ArrayRef,
}

impl serde::Serialize for BoolArray<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let array = self
            .value
            .as_boolean_opt()
            .ok_or_else(|| error("not a boolean array"))?;
        let mut seq = serializer.serialize_tuple(array.len())?;

        for value in array.values() {
            seq.serialize_element(&value)?;
        }

        seq.end()
    }
}
