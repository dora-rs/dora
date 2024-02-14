use std::{any::type_name, borrow::Cow, marker::PhantomData, sync::Arc};

use arrow::{
    array::{Array, ArrayRef, AsArray, OffsetSizeTrait, PrimitiveArray},
    datatypes::{self, ArrowPrimitiveType},
};
use dora_ros2_bridge_msg_gen::types::{
    primitives::{BasicType, GenericString, NestableType},
    sequences,
};
use serde::ser::SerializeTuple;

use crate::typed::TypeInfo;

use super::{error, TypedValue};

/// Serialize an array with known size as tuple.
pub struct ArraySerializeWrapper<'a> {
    pub array_info: &'a sequences::Array,
    pub column: &'a ArrayRef,
    pub type_info: &'a TypeInfo<'a>,
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
                .ok_or_else(|| error("value is not compatible with expected array type"))?;
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
            NestableType::NamedType(name) => {
                let array = entry
                    .as_struct_opt()
                    .ok_or_else(|| error("not a struct array"))?;
                let mut seq = serializer.serialize_tuple(self.array_info.size)?;
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
                let mut seq = serializer.serialize_tuple(self.array_info.size)?;
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
                        Some(array) => {
                            serialize_arrow_string(serializer, array, self.array_info.size)
                        }
                        None => {
                            let array = entry
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array"))?;
                            serialize_arrow_string(serializer, array, self.array_info.size)
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
            .ok_or_else(|| error(format!("not a primitive {} array", type_name::<T>())))?;
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

fn serialize_arrow_string<S, O>(
    serializer: S,
    array: &arrow::array::GenericByteArray<datatypes::GenericStringType<O>>,
    array_len: usize,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    let mut seq = serializer.serialize_tuple(array_len)?;
    for s in array.iter() {
        seq.serialize_element(s.unwrap_or_default())?;
    }
    seq.end()
}
