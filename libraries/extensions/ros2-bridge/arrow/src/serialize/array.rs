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

use crate::TypeInfo;

use super::{TypedValue, check_array_len, error};

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
            if list.len() != 1 {
                return Err(error(format!(
                    "expected single-element list, got length {}",
                    list.len()
                )));
            }
            list.value(0)
        } else {
            let list = self
                .column
                .as_list_opt::<i64>()
                .ok_or_else(|| error("value is not compatible with expected array type"))?;
            if list.len() != 1 {
                return Err(error(format!(
                    "expected single-element large list, got length {}",
                    list.len()
                )));
            }
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
                check_array_len(array.len(), self.array_info.size)?;
                let mut seq = serializer.serialize_tuple(self.array_info.size)?;
                for i in 0..array.len() {
                    let row = array.slice(i, 1);
                    seq.serialize_element(&TypedValue {
                        value: &(Arc::new(row) as ArrayRef),
                        type_info: &TypeInfo {
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
                check_array_len(array.len(), self.array_info.size)?;
                let mut seq = serializer.serialize_tuple(self.array_info.size)?;
                for i in 0..array.len() {
                    let row = array.slice(i, 1);
                    seq.serialize_element(&TypedValue {
                        value: &(Arc::new(row) as ArrayRef),
                        type_info: &TypeInfo {
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
                GenericString::WString | GenericString::BoundedWString(_) => {
                    match entry.as_string_opt::<i32>() {
                        Some(array) => {
                            serialize_arrow_wstring(serializer, array, self.array_info.size)
                        }
                        None => {
                            let array = entry
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array for WString"))?;
                            serialize_arrow_wstring(serializer, array, self.array_info.size)
                        }
                    }
                }
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
        check_array_len(array.len(), self.len)?;

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
        check_array_len(array.len(), self.len)?;

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
    check_array_len(array.len(), array_len)?;
    let mut seq = serializer.serialize_tuple(array_len)?;
    for s in array.iter() {
        seq.serialize_element(s.unwrap_or_default())?;
    }
    seq.end()
}

fn serialize_arrow_wstring<S, O>(
    serializer: S,
    array: &arrow::array::GenericByteArray<datatypes::GenericStringType<O>>,
    array_len: usize,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    check_array_len(array.len(), array_len)?;
    let mut seq = serializer.serialize_tuple(array_len)?;
    for s in array.iter() {
        let utf16: Vec<u16> = s.unwrap_or_default().encode_utf16().collect();
        seq.serialize_element(&utf16)?;
    }
    seq.end()
}

#[cfg(test)]
mod tests {
    use std::{borrow::Cow, collections::HashMap, sync::Arc};

    use arrow::{
        array::{ArrayRef, ListArray, StringArray, StructArray},
        buffer::OffsetBuffer,
        datatypes::{DataType, Field},
    };
    use byteorder::LittleEndian;
    use dora_ros2_bridge_msg_gen::types::{
        Member, MemberType, Message,
        primitives::{GenericString, NestableType},
        sequences::Array as ArrayType,
    };

    use super::*;
    use crate::TypeInfo;

    /// A message with a single fixed-size `<field_type>[3]` array field.
    fn fixed_array_message(
        field_type: NestableType,
    ) -> Arc<HashMap<String, HashMap<String, Message>>> {
        let message = Message {
            package: "test_msgs".to_string(),
            name: "ArrMsg".to_string(),
            members: vec![Member {
                name: "names".to_string(),
                r#type: MemberType::Array(ArrayType {
                    value_type: field_type,
                    size: 3,
                }),
                default: None,
            }],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("ArrMsg".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_msgs".to_string(), package);
        Arc::new(messages)
    }

    fn serializes_ok(
        messages: &Arc<HashMap<String, HashMap<String, Message>>>,
        names: &[&str],
    ) -> bool {
        // An array field's column is a single-element `List` whose inner value
        // holds the array elements (mirrors how dora wraps message fields).
        let item = Arc::new(Field::new("item", DataType::Utf8, true));
        let list = ListArray::new(
            item.clone(),
            OffsetBuffer::from_lengths([names.len()]),
            Arc::new(StringArray::from(names.to_vec())),
            None,
        );
        let struct_array = StructArray::from(vec![(
            Arc::new(Field::new("names", DataType::List(item), false)),
            Arc::new(list) as ArrayRef,
        )]);
        let value = Arc::new(struct_array) as ArrayRef;
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ArrMsg"),
            messages: messages.clone(),
        };
        let typed = TypedValue {
            value: &value,
            type_info: &type_info,
        };
        cdr_encoding::to_vec::<_, LittleEndian>(&typed).is_ok()
    }

    /// #2027: a fixed-size `string[3]` field given the wrong number of elements
    /// must error rather than silently emit a wrong-sized (length-prefix-less)
    /// CDR tuple.
    #[test]
    fn fixed_string_array_length_is_checked() {
        let messages = fixed_array_message(NestableType::GenericString(GenericString::String));
        assert!(
            !serializes_ok(&messages, &["a", "b"]),
            "size-3 field with 2 elements must error"
        );
        assert!(
            serializes_ok(&messages, &["a", "b", "c"]),
            "size-3 field with 3 elements must serialize"
        );
    }

    /// Same guard on the `wstring[N]` path.
    #[test]
    fn fixed_wstring_array_length_is_checked() {
        let messages = fixed_array_message(NestableType::GenericString(GenericString::WString));
        assert!(
            !serializes_ok(&messages, &["a", "b", "c", "d"]),
            "size-3 field with 4 elements must error"
        );
        assert!(
            serializes_ok(&messages, &["a", "b", "c"]),
            "size-3 field with 3 elements must serialize"
        );
    }
}
