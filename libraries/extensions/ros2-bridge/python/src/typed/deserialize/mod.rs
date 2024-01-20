use super::{TypeInfo, DUMMY_STRUCT_NAME};
use arrow::{
    array::{make_array, ArrayData, StructArray},
    datatypes::Field,
};
use core::fmt;
use std::{borrow::Cow, collections::HashMap, fmt::Display, sync::Arc};

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
}

impl<'de> serde::de::DeserializeSeed<'de> for StructDeserializer<'_> {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let empty = HashMap::new();
        let package_messages = self
            .type_info
            .messages
            .get(self.type_info.package_name.as_ref())
            .unwrap_or(&empty);
        let message = package_messages
            .get(self.type_info.message_name.as_ref())
            .ok_or_else(|| {
                error(format!(
                    "could not find message type {}::{}",
                    self.type_info.package_name, self.type_info.message_name
                ))
            })?;

        let visitor = StructVisitor {
            type_info: self.type_info.as_ref(),
        };
        deserializer.deserialize_tuple_struct(DUMMY_STRUCT_NAME, message.members.len(), visitor)
    }
}

struct StructVisitor<'a> {
    type_info: &'a TypeInfo<'a>,
}

impl<'a, 'de> serde::de::Visitor<'de> for StructVisitor<'a> {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a struct encoded as sequence")
    }

    fn visit_seq<A>(self, mut data: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        let empty = HashMap::new();
        let package_messages = self
            .type_info
            .messages
            .get(self.type_info.package_name.as_ref())
            .unwrap_or(&empty);
        let message = package_messages
            .get(self.type_info.message_name.as_ref())
            .ok_or_else(|| {
                error(format!(
                    "could not find message type {}::{}",
                    self.type_info.package_name, self.type_info.message_name
                ))
            })?;

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
                            dora_ros2_bridge_msg_gen::types::primitives::GenericString::WString => todo!("deserialize WString"),
                            dora_ros2_bridge_msg_gen::types::primitives::GenericString::BoundedWString(_) => todo!("deserialize BoundedWString"),
                        }
                    }
                },
                dora_ros2_bridge_msg_gen::types::MemberType::Array(a) => {
                    data.next_element_seed(array::ArrayDeserializer(a))?
                },
                dora_ros2_bridge_msg_gen::types::MemberType::Sequence(s) => {
                    data.next_element_seed(sequence::SequenceDeserializer(&s.value_type))?
                },
                dora_ros2_bridge_msg_gen::types::MemberType::BoundedSequence(s) => {
                    data.next_element_seed(sequence::SequenceDeserializer(&s.value_type))?
                },
            };

            let value = value.ok_or_else(|| {
                error(format!(
                    "struct member {} not present in message",
                    member.name
                ))
            })?;

            fields.push((
                // Recreate a new field as List(UInt8) can be converted to UInt8
                Arc::new(Field::new(&member.name, value.data_type().clone(), true)),
                make_array(value),
            ));
        }

        let struct_array: StructArray = fields.into();

        Ok(struct_array.into())
    }
}

// struct ListVisitor {
//     field: Arc<Field>,
//     defaults: ArrayData,
// }

// impl<'de> serde::de::Visitor<'de> for ListVisitor {
//     type Value = ArrayData;

//     fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
//         formatter.write_str("an array encoded as sequence")
//     }

//     fn visit_seq<A>(self, mut data: A) -> Result<Self::Value, A::Error>
//     where
//         A: serde::de::SeqAccess<'de>,
//     {
//         let data = match self.field.data_type().clone() {
//             DataType::UInt8 => {
//                 let mut array = UInt8Builder::new();
//                 while let Some(value) = data.next_element::<u8>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::UInt16 => {
//                 let mut array = UInt16Builder::new();
//                 while let Some(value) = data.next_element::<u16>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::UInt32 => {
//                 let mut array = UInt32Builder::new();
//                 while let Some(value) = data.next_element::<u32>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::UInt64 => {
//                 let mut array = UInt64Builder::new();
//                 while let Some(value) = data.next_element::<u64>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::Int8 => {
//                 let mut array = Int8Builder::new();
//                 while let Some(value) = data.next_element::<i8>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::Int16 => {
//                 let mut array = Int16Builder::new();
//                 while let Some(value) = data.next_element::<i16>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::Int32 => {
//                 let mut array = Int32Builder::new();
//                 while let Some(value) = data.next_element::<i32>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::Int64 => {
//                 let mut array = Int64Builder::new();
//                 while let Some(value) = data.next_element::<i64>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::Float32 => {
//                 let mut array = Float32Builder::new();
//                 while let Some(value) = data.next_element::<f32>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::Float64 => {
//                 let mut array = Float64Builder::new();
//                 while let Some(value) = data.next_element::<f64>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             DataType::Utf8 => {
//                 let mut array = StringBuilder::new();
//                 while let Some(value) = data.next_element::<String>()? {
//                     array.append_value(value);
//                 }
//                 Ok(array.finish().into())
//             }
//             _ => {
//                 let mut buffer = vec![];
//                 while let Some(value) = data.next_element_seed(StructDeserializer {
//                     type_info: TypeInfo {
//                         data_type: self.field.data_type().clone(),
//                         defaults: self.defaults.clone(),
//                     },
//                 })? {
//                     let element = make_array(value);
//                     buffer.push(element);
//                 }

//                 concat(
//                     buffer
//                         .iter()
//                         .map(|data| data.as_ref())
//                         .collect::<Vec<_>>()
//                         .as_slice(),
//                 )
//                 .map(|op| op.to_data())
//             }
//         };

//         if let Ok(values) = data {
//             let offsets = OffsetBuffer::new(vec![0, values.len() as i32].into());

//             let array = ListArray::new(self.field, offsets, make_array(values), None).to_data();
//             Ok(array)
//         } else {
//             Ok(self.defaults) // TODO: Better handle deserialization error
//         }
//     }
// }

fn error<E, T>(e: T) -> E
where
    T: Display,
    E: serde::de::Error,
{
    serde::de::Error::custom(e)
}
