use arrow::{
    array::{ArrayData, BooleanBuilder, PrimitiveBuilder, StringBuilder},
    datatypes::{self, ArrowPrimitiveType},
};
use core::fmt;
use dora_ros2_bridge_msg_gen::types::primitives::{self, BasicType, NestableType};
use serde::Deserialize;
use std::{borrow::Cow, ops::Deref};

use crate::typed::TypeInfo;

use super::{error, StructDeserializer};

pub struct SequenceDeserializer<'a> {
    pub item_type: &'a NestableType,
    pub type_info: &'a TypeInfo<'a>,
}

impl<'de> serde::de::DeserializeSeed<'de> for SequenceDeserializer<'_> {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_seq(SequenceVisitor {
            item_type: self.item_type,
            type_info: self.type_info,
        })
    }
}

pub struct SequenceVisitor<'a> {
    pub item_type: &'a NestableType,
    pub type_info: &'a TypeInfo<'a>,
}

impl<'de> serde::de::Visitor<'de> for SequenceVisitor<'_> {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(formatter, "a sequence")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        match &self.item_type {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => deserialize_primitive_seq::<_, datatypes::Int8Type>(seq),
                BasicType::I16 => deserialize_primitive_seq::<_, datatypes::Int16Type>(seq),
                BasicType::I32 => deserialize_primitive_seq::<_, datatypes::Int32Type>(seq),
                BasicType::I64 => deserialize_primitive_seq::<_, datatypes::Int64Type>(seq),
                BasicType::U8 | BasicType::Char | BasicType::Byte => {
                    deserialize_primitive_seq::<_, datatypes::UInt8Type>(seq)
                }
                BasicType::U16 => deserialize_primitive_seq::<_, datatypes::UInt16Type>(seq),
                BasicType::U32 => deserialize_primitive_seq::<_, datatypes::UInt32Type>(seq),
                BasicType::U64 => deserialize_primitive_seq::<_, datatypes::UInt64Type>(seq),
                BasicType::F32 => deserialize_primitive_seq::<_, datatypes::Float32Type>(seq),
                BasicType::F64 => deserialize_primitive_seq::<_, datatypes::Float64Type>(seq),
                BasicType::Bool => {
                    let mut array = BooleanBuilder::new();
                    while let Some(value) = seq.next_element()? {
                        array.append_value(value);
                    }
                    Ok(array.finish().into())
                }
            },
            NestableType::NamedType(name) => {
                let deserializer = StructDeserializer {
                    type_info: Cow::Owned(TypeInfo {
                        package_name: Cow::Borrowed(&self.type_info.package_name),
                        message_name: Cow::Borrowed(&name.0),
                        messages: self.type_info.messages.clone(),
                    }),
                };
                deserialize_struct_array(&mut seq, deserializer)
            }
            NestableType::NamespacedType(reference) => {
                if reference.namespace != "msg" {
                    return Err(error(format!(
                        "sequence item references non-message type {reference:?}",
                    )));
                }
                let deserializer = StructDeserializer {
                    type_info: Cow::Owned(TypeInfo {
                        package_name: Cow::Borrowed(&reference.package),
                        message_name: Cow::Borrowed(&reference.name),
                        messages: self.type_info.messages.clone(),
                    }),
                };
                deserialize_struct_array(&mut seq, deserializer)
            }
            NestableType::GenericString(t) => match t {
                primitives::GenericString::String | primitives::GenericString::BoundedString(_) => {
                    let mut array = StringBuilder::new();
                    while let Some(value) = seq.next_element::<String>()? {
                        array.append_value(value);
                    }
                    Ok(array.finish().into())
                }
                primitives::GenericString::WString => todo!("deserialize sequence of WString"),
                primitives::GenericString::BoundedWString(_) => {
                    todo!("deserialize sequence of BoundedWString")
                }
            },
        }
    }
}

fn deserialize_struct_array<'de, A>(
    seq: &mut A,
    deserializer: StructDeserializer<'_>,
) -> Result<ArrayData, <A as serde::de::SeqAccess<'de>>::Error>
where
    A: serde::de::SeqAccess<'de>,
{
    let mut values = Vec::new();
    while let Some(value) = seq.next_element_seed(deserializer.clone())? {
        values.push(arrow::array::make_array(value));
    }
    let refs: Vec<_> = values.iter().map(|a| a.deref()).collect();
    arrow::compute::concat(&refs)
        .map(|a| a.to_data())
        .map_err(super::error)
}

fn deserialize_primitive_seq<'de, S, T>(
    mut seq: S,
) -> Result<ArrayData, <S as serde::de::SeqAccess<'de>>::Error>
where
    S: serde::de::SeqAccess<'de>,
    T: ArrowPrimitiveType,
    T::Native: Deserialize<'de>,
{
    let mut array = PrimitiveBuilder::<T>::new();
    while let Some(value) = seq.next_element::<T::Native>()? {
        array.append_value(value);
    }
    Ok(array.finish().into())
}
