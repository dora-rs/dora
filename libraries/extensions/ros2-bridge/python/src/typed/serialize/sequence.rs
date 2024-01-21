use std::{any::type_name, borrow::Cow, marker::PhantomData, sync::Arc};

use arrow::{
    array::{Array, ArrayRef, AsArray, OffsetSizeTrait, PrimitiveArray},
    datatypes::{self, ArrowPrimitiveType, UInt8Type},
};
use dora_ros2_bridge_msg_gen::types::primitives::{BasicType, GenericString, NestableType};
use serde::ser::{SerializeSeq, SerializeTuple};

use crate::typed::TypeInfo;

use super::{error, TypedValue};

/// Serialize a variable-sized sequence.
pub struct SequenceSerializeWrapper<'a> {
    pub item_type: &'a NestableType,
    pub column: &'a ArrayRef,
    pub type_info: &'a TypeInfo<'a>,
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
        } else if let Some(list) = self.column.as_list_opt::<i64>() {
            // should match the length of the outer struct
            assert_eq!(list.len(), 1);
            list.value(0)
        } else if let Some(list) = self.column.as_binary_opt::<i32>() {
            // should match the length of the outer struct
            assert_eq!(list.len(), 1);
            Arc::new(list.slice(0, 1)) as ArrayRef
        } else if let Some(list) = self.column.as_binary_opt::<i64>() {
            // should match the length of the outer struct
            assert_eq!(list.len(), 1);
            Arc::new(list.slice(0, 1)) as ArrayRef
        } else {
            return Err(error(format!(
                "value is not compatible with expected sequence type: {:?}",
                self.column
            )));
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
                BasicType::U8 | BasicType::Char | BasicType::Byte => {
                    ByteSequence { value: &entry }.serialize(serializer)
                }
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
            NestableType::NamedType(name) => {
                let array = entry
                    .as_struct_opt()
                    .ok_or_else(|| error("not a struct array"))?;
                let mut seq = serializer.serialize_seq(Some(array.len()))?;
                for i in 0..array.len() {
                    let row = array.slice(i, 1);
                    seq.serialize_element(&TypedValue {
                        value: &(Arc::new(row) as ArrayRef),
                        type_info: &crate::typed::TypeInfo {
                            package_name: Cow::Borrowed(&self.type_info.package_name),
                            message_name: Cow::Borrowed(&name.0),
                            messages: self.type_info.messages.clone(),
                        },
                    })?;
                }
                seq.end()
            }
            NestableType::NamespacedType(reference) => {
                if reference.namespace != "msg" {
                    return Err(error(format!(
                        "sequence references non-message type {reference:?}"
                    )));
                }

                let array = entry
                    .as_struct_opt()
                    .ok_or_else(|| error("not a struct array"))?;
                let mut seq = serializer.serialize_seq(Some(array.len()))?;
                for i in 0..array.len() {
                    let row = array.slice(i, 1);
                    seq.serialize_element(&TypedValue {
                        value: &(Arc::new(row) as ArrayRef),
                        type_info: &crate::typed::TypeInfo {
                            package_name: Cow::Borrowed(&reference.package),
                            message_name: Cow::Borrowed(&reference.name),
                            messages: self.type_info.messages.clone(),
                        },
                    })?;
                }
                seq.end()
            }
            NestableType::GenericString(s) => match s {
                GenericString::String | GenericString::BoundedString(_) => {
                    match entry.as_string_opt::<i32>() {
                        Some(array) => serialize_arrow_string(serializer, array),
                        None => {
                            let array = entry
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array"))?;
                            serialize_arrow_string(serializer, array)
                        }
                    }
                }
                GenericString::WString => {
                    todo!("serializing WString sequences")
                }
                GenericString::BoundedWString(_) => todo!("serializing BoundedWString sequences"),
            },
        }
    }
}

fn serialize_arrow_string<S, O>(
    serializer: S,
    array: &arrow::array::GenericByteArray<datatypes::GenericStringType<O>>,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    let mut seq = serializer.serialize_seq(Some(array.len()))?;
    for s in array.iter() {
        seq.serialize_element(s.unwrap_or_default())?;
    }
    seq.end()
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
            .ok_or_else(|| error(format!("not a primitive {} array", type_name::<T>())))?;

        let mut seq = serializer.serialize_seq(Some(array.len()))?;

        for value in array.values() {
            seq.serialize_element(value)?;
        }

        seq.end()
    }
}

struct ByteSequence<'a> {
    value: &'a ArrayRef,
}

impl serde::Serialize for ByteSequence<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        if let Some(binary) = self.value.as_binary_opt::<i32>() {
            serialize_binary(serializer, binary)
        } else if let Some(binary) = self.value.as_binary_opt::<i64>() {
            serialize_binary(serializer, binary)
        } else {
            BasicSequence {
                value: self.value,
                ty: PhantomData::<UInt8Type>,
            }
            .serialize(serializer)
        }
    }
}

fn serialize_binary<S, O>(
    serializer: S,
    binary: &arrow::array::GenericByteArray<datatypes::GenericBinaryType<O>>,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    let mut seq = serializer.serialize_seq(Some(binary.len()))?;

    for value in binary.iter() {
        seq.serialize_element(value.unwrap_or_default())?;
    }

    seq.end()
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
