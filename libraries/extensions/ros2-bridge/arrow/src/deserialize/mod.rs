use super::{DUMMY_STRUCT_NAME, TypeInfo};
use arrow::{
    array::{ArrayData, StructArray, make_array},
    datatypes::{DataType, Field},
};
use core::fmt;
use dora_ros2_bridge_msg_gen::types::Message;
use std::{borrow::Cow, fmt::Display, sync::Arc};

mod array;
mod primitive;
mod sequence;
mod string;

#[derive(Debug, Clone)]
pub struct StructDeserializer<'a> {
    type_info: Cow<'a, TypeInfo<'a>>,
}

impl<'a> StructDeserializer<'a> {
    pub fn new(type_info: Cow<'a, TypeInfo<'a>>) -> Self {
        Self { type_info }
    }

    /// Computes the Arrow [`DataType`] this deserializer produces, without
    /// requiring any input bytes.
    ///
    /// Used to build a length-0 child array for an empty `sequence<struct>`,
    /// where there is no element to infer the struct type from.
    pub(crate) fn struct_data_type(&self) -> Result<DataType, String> {
        struct_data_type(self.type_info.as_ref())
    }
}

/// Resolves the message definition referenced by `type_info`.
fn lookup_message<'a>(type_info: &'a TypeInfo<'a>) -> Result<&'a Message, String> {
    type_info
        .messages
        .get(type_info.package_name.as_ref())
        .and_then(|pkg| pkg.get(type_info.message_name.as_ref()))
        .ok_or_else(|| {
            format!(
                "could not find message type {}::{}",
                type_info.package_name, type_info.message_name
            )
        })
}

/// Builds the `DataType::Struct` produced when deserializing the message
/// referenced by `type_info`. The field types mirror the mapping used by the
/// deserializers in this module exactly, so a zero-row array built from this
/// type matches what an equivalent non-empty sequence would produce.
fn struct_data_type(type_info: &TypeInfo<'_>) -> Result<DataType, String> {
    use dora_ros2_bridge_msg_gen::types::{
        MemberType,
        primitives::{BasicType, NestableType},
    };

    fn nestable_data_type(t: &NestableType, type_info: &TypeInfo<'_>) -> Result<DataType, String> {
        Ok(match t {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => DataType::Int8,
                BasicType::I16 => DataType::Int16,
                BasicType::I32 => DataType::Int32,
                BasicType::I64 => DataType::Int64,
                BasicType::U8 | BasicType::Char | BasicType::Byte => DataType::UInt8,
                BasicType::U16 => DataType::UInt16,
                BasicType::U32 => DataType::UInt32,
                BasicType::U64 => DataType::UInt64,
                BasicType::F32 => DataType::Float32,
                BasicType::F64 => DataType::Float64,
                BasicType::Bool => DataType::Boolean,
            },
            NestableType::GenericString(_) => DataType::Utf8,
            NestableType::NamedType(name) => struct_data_type(&TypeInfo {
                package_name: type_info.package_name.clone(),
                message_name: Cow::Owned(name.0.clone()),
                messages: type_info.messages.clone(),
            })?,
            NestableType::NamespacedType(reference) => {
                if reference.namespace != "msg" {
                    return Err(format!("field references non-message type {reference:?}"));
                }
                struct_data_type(&TypeInfo {
                    package_name: Cow::Owned(reference.package.clone()),
                    message_name: Cow::Owned(reference.name.clone()),
                    messages: type_info.messages.clone(),
                })?
            }
        })
    }

    fn list_data_type(
        value_type: &NestableType,
        type_info: &TypeInfo<'_>,
    ) -> Result<DataType, String> {
        let item = nestable_data_type(value_type, type_info)?;
        Ok(DataType::List(Arc::new(Field::new("item", item, true))))
    }

    let message = lookup_message(type_info)?;
    let mut fields = Vec::with_capacity(message.members.len());
    for member in &message.members {
        let data_type = match &member.r#type {
            MemberType::NestableType(t) => nestable_data_type(t, type_info)?,
            MemberType::Array(a) => list_data_type(&a.value_type, type_info)?,
            MemberType::Sequence(s) => list_data_type(&s.value_type, type_info)?,
            MemberType::BoundedSequence(s) => list_data_type(&s.value_type, type_info)?,
        };
        fields.push(Field::new(&member.name, data_type, true));
    }
    Ok(DataType::Struct(fields.into()))
}

impl<'de> serde::de::DeserializeSeed<'de> for StructDeserializer<'_> {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let type_info = self.type_info.as_ref();
        let message = lookup_message(type_info).map_err(error)?;

        let visitor = StructVisitor { type_info, message };
        deserializer.deserialize_tuple_struct(DUMMY_STRUCT_NAME, message.members.len(), visitor)
    }
}

struct StructVisitor<'a> {
    type_info: &'a TypeInfo<'a>,
    /// Message definition for `type_info`, resolved once in `deserialize` and
    /// reused here so `visit_seq` does not repeat the `messages` lookup (and the
    /// throwaway `HashMap` allocation) for every struct — including every
    /// element of a `sequence<struct>` / `struct[N]`.
    message: &'a Message,
}

impl<'de> serde::de::Visitor<'de> for StructVisitor<'_> {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a struct encoded as TupleStruct")
    }

    fn visit_seq<A>(self, mut data: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        let message = self.message;

        let mut fields = vec![];
        for member in &message.members {
            let value = match &member.r#type {
                dora_ros2_bridge_msg_gen::types::MemberType::NestableType(t) => match t {
                    dora_ros2_bridge_msg_gen::types::primitives::NestableType::BasicType(t) => {
                        data.next_element_seed(primitive::PrimitiveDeserializer(t))?
                    }
                    dora_ros2_bridge_msg_gen::types::primitives::NestableType::NamedType(name) => {
                        data.next_element_seed(StructDeserializer {
                            type_info: Cow::Owned(TypeInfo {
                                package_name: Cow::Borrowed(&self.type_info.package_name),
                                message_name: Cow::Borrowed(&name.0),
                                messages: self.type_info.messages.clone(),
                            }),
                        })?
                    }
                    dora_ros2_bridge_msg_gen::types::primitives::NestableType::NamespacedType(
                        reference,
                    ) => {
                        if reference.namespace != "msg" {
                            return Err(error(format!(
                                "struct field {} references non-message type {reference:?}",
                                member.name
                            )));
                        }
                        data.next_element_seed(StructDeserializer {
                            type_info: Cow::Owned(TypeInfo {
                                package_name: Cow::Borrowed(&reference.package),
                                message_name: Cow::Borrowed(&reference.name),
                                messages: self.type_info.messages.clone(),
                            }),
                        })?
                    }
                    dora_ros2_bridge_msg_gen::types::primitives::NestableType::GenericString(t) => {
                        match t {
                            dora_ros2_bridge_msg_gen::types::primitives::GenericString::String | dora_ros2_bridge_msg_gen::types::primitives::GenericString::BoundedString(_)=> {
                                data.next_element_seed(string::StringDeserializer)?
                            },
                            dora_ros2_bridge_msg_gen::types::primitives::GenericString::WString | dora_ros2_bridge_msg_gen::types::primitives::GenericString::BoundedWString(_) => {
                                data.next_element_seed(string::WStringDeserializer)?
                            },
                        }
                    }
                },
                dora_ros2_bridge_msg_gen::types::MemberType::Array(a) => {
                    data.next_element_seed(array::ArrayDeserializer{ array_type : a, type_info: self.type_info})?
                },
                dora_ros2_bridge_msg_gen::types::MemberType::Sequence(s) => {
                    data.next_element_seed(sequence::SequenceDeserializer{item_type: &s.value_type, type_info: self.type_info})?
                },
                dora_ros2_bridge_msg_gen::types::MemberType::BoundedSequence(s) => {
                    data.next_element_seed(sequence::SequenceDeserializer{ item_type: &s.value_type, type_info: self.type_info})?
                },
            };

            let value = value.ok_or_else(|| {
                error(format!(
                    "struct member {} not present in message",
                    member.name
                ))
            })?;

            fields.push((
                Arc::new(Field::new(&member.name, value.data_type().clone(), true)),
                make_array(value),
            ));
        }

        let struct_array: StructArray = fields.into();

        Ok(struct_array.into())
    }
}

fn error<E, T>(e: T) -> E
where
    T: Display,
    E: serde::de::Error,
{
    serde::de::Error::custom(e)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{Array, AsArray};
    use byteorder::LittleEndian;
    use dora_ros2_bridge_msg_gen::types::{
        Member, MemberType, Message,
        primitives::{BasicType, NamedType, NestableType},
        sequences::Sequence,
    };
    use std::collections::HashMap;

    fn member(name: &str, r#type: MemberType) -> Member {
        Member {
            name: name.to_string(),
            r#type,
            default: None,
        }
    }

    /// Builds a `test_pkg` with a `Point { int32 x }` message and a
    /// `Container { int32 count; Point[] points }` message.
    fn container_messages() -> Arc<HashMap<String, HashMap<String, Message>>> {
        let point = Message {
            package: "test_pkg".to_string(),
            name: "Point".to_string(),
            members: vec![member(
                "x",
                MemberType::NestableType(NestableType::BasicType(BasicType::I32)),
            )],
            constants: vec![],
        };
        let container = Message {
            package: "test_pkg".to_string(),
            name: "Container".to_string(),
            members: vec![
                member(
                    "count",
                    MemberType::NestableType(NestableType::BasicType(BasicType::I32)),
                ),
                member(
                    "points",
                    MemberType::Sequence(Sequence {
                        value_type: NestableType::NamedType(NamedType("Point".to_string())),
                    }),
                ),
            ],
            constants: vec![],
        };

        let mut package = HashMap::new();
        package.insert("Point".to_string(), point);
        package.insert("Container".to_string(), container);
        let mut messages = HashMap::new();
        messages.insert("test_pkg".to_string(), package);
        Arc::new(messages)
    }

    /// Regression test for #2033: an empty `sequence<struct>` field must not
    /// fail the whole message deserialize. Previously this hit
    /// `arrow::compute::concat` on an empty slice, which errors out.
    #[test]
    fn empty_struct_sequence_deserializes() {
        let messages = container_messages();
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_pkg"),
            message_name: Cow::Borrowed("Container"),
            messages,
        };

        // CDR (little-endian): int32 count = 42, then a sequence with a u32
        // length prefix of 0 (no elements).
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&42i32.to_le_bytes());
        bytes.extend_from_slice(&0u32.to_le_bytes());

        let seed = StructDeserializer::new(Cow::Owned(type_info));
        let mut deserializer = cdr_encoding::CdrDeserializer::<LittleEndian>::new(&bytes);
        let data = serde::de::DeserializeSeed::deserialize(seed, &mut deserializer)
            .expect("empty struct-sequence message must deserialize successfully");

        let array = make_array(data);
        let container = array.as_struct();
        let count = container
            .column_by_name("count")
            .unwrap()
            .as_primitive::<arrow::datatypes::Int32Type>();
        assert_eq!(count.value(0), 42);

        let points = container.column_by_name("points").unwrap().as_list::<i32>();
        // The outer list wraps the whole sequence in a single row.
        assert_eq!(points.len(), 1);
        let inner = points.value(0);
        // The sequence itself is empty.
        assert_eq!(inner.len(), 0);
        // ...but the child type is the correct `Point` struct.
        assert!(matches!(inner.data_type(), DataType::Struct(fields)
            if fields.len() == 1 && fields[0].name() == "x"));
    }
}
