use std::{borrow::Cow, collections::HashMap, fmt::Display};

use arrow::{
    array::{Array, ArrayRef, AsArray},
    error,
};
use dora_ros2_bridge_msg_gen::types::{
    primitives::{GenericString, NestableType},
    MemberType,
};
use eyre::Context;
use serde::ser::SerializeTupleStruct;

use super::{TypeInfo, DUMMY_STRUCT_NAME};

mod array;
mod defaults;
mod primitive;
mod sequence;

#[derive(Debug, Clone)]
pub struct TypedValue<'a> {
    pub value: &'a ArrayRef,
    pub type_info: &'a TypeInfo<'a>,
}

impl serde::Serialize for TypedValue<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
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

        let input = self.value.as_struct_opt().ok_or_else(|| {
            error(format!(
                "expected struct array for message: {}, with following format: {:#?} \n But, got type: {:#?}",
               self.type_info.message_name, message, self.value.data_type()
            ))
        })?;
        for column_name in input.column_names() {
            if !message.members.iter().any(|m| m.name == column_name) {
                return Err(error(format!(
                    "given struct has unknown field {column_name}"
                )))?;
            }
        }
        if input.is_empty() {
            // TODO: publish default value
            return Err(error("given struct is empty"))?;
        }
        if input.len() > 1 {
            return Err(error(format!(
                "expected single struct instance, got struct array with {} entries",
                input.len()
            )))?;
        }
        let mut s = serializer.serialize_tuple_struct(DUMMY_STRUCT_NAME, message.members.len())?;
        for field in message.members.iter() {
            let column: Cow<_> = match input.column_by_name(&field.name) {
                Some(input) => Cow::Borrowed(input),
                None => {
                    let default = defaults::default_for_member(
                        field,
                        &self.type_info.package_name,
                        &self.type_info.messages,
                    )
                    .with_context(|| {
                        format!(
                            "failed to calculate default value for field {}.{}",
                            message.name, field.name
                        )
                    })
                    .map_err(|e| error(format!("{e:?}")))?;
                    Cow::Owned(arrow::array::make_array(default))
                }
            };

            self.serialize_field::<S>(field, column, &mut s)
                .map_err(|e| {
                    error(format!(
                        "failed to serialize field {}.{}: {e}",
                        message.name, field.name
                    ))
                })?;
        }
        s.end()
    }
}

impl<'a> TypedValue<'a> {
    fn serialize_field<S>(
        &self,
        field: &dora_ros2_bridge_msg_gen::types::Member,
        column: Cow<'_, std::sync::Arc<dyn Array>>,
        s: &mut S::SerializeTupleStruct,
    ) -> Result<(), S::Error>
    where
        S: serde::Serializer,
    {
        match &field.r#type {
            MemberType::NestableType(t) => match t {
                NestableType::BasicType(t) => {
                    s.serialize_field(&primitive::SerializeWrapper {
                        t,
                        column: column.as_ref(),
                    })?;
                }
                NestableType::NamedType(name) => {
                    let referenced_value = &TypedValue {
                        value: column.as_ref(),
                        type_info: &TypeInfo {
                            package_name: Cow::Borrowed(&self.type_info.package_name),
                            message_name: Cow::Borrowed(&name.0),
                            messages: self.type_info.messages.clone(),
                        },
                    };
                    s.serialize_field(&referenced_value)?;
                }
                NestableType::NamespacedType(reference) => {
                    if reference.namespace != "msg" {
                        return Err(error(format!(
                            "struct field {} references non-message type {reference:?}",
                            field.name
                        )));
                    }

                    let referenced_value: &TypedValue<'_> = &TypedValue {
                        value: column.as_ref(),
                        type_info: &TypeInfo {
                            package_name: Cow::Borrowed(&reference.package),
                            message_name: Cow::Borrowed(&reference.name),
                            messages: self.type_info.messages.clone(),
                        },
                    };
                    s.serialize_field(&referenced_value)?;
                }
                NestableType::GenericString(t) => match t {
                    GenericString::String | GenericString::BoundedString(_) => {
                        let string = if let Some(string_array) = column.as_string_opt::<i32>() {
                            // should match the length of the outer struct array
                            assert_eq!(string_array.len(), 1);
                            string_array.value(0)
                        } else {
                            // try again with large offset type
                            let string_array = column
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array"))?;
                            // should match the length of the outer struct array
                            assert_eq!(string_array.len(), 1);
                            string_array.value(0)
                        };
                        s.serialize_field(string)?;
                    }
                    GenericString::WString => todo!("serializing WString types"),
                    GenericString::BoundedWString(_) => {
                        todo!("serializing BoundedWString types")
                    }
                },
            },
            dora_ros2_bridge_msg_gen::types::MemberType::Array(a) => {
                s.serialize_field(&array::ArraySerializeWrapper {
                    array_info: a,
                    column: column.as_ref(),
                    type_info: self.type_info,
                })?;
            }
            dora_ros2_bridge_msg_gen::types::MemberType::Sequence(v) => {
                s.serialize_field(&sequence::SequenceSerializeWrapper {
                    item_type: &v.value_type,
                    column: column.as_ref(),
                    type_info: self.type_info,
                })?;
            }
            dora_ros2_bridge_msg_gen::types::MemberType::BoundedSequence(v) => {
                s.serialize_field(&sequence::SequenceSerializeWrapper {
                    item_type: &v.value_type,
                    column: column.as_ref(),
                    type_info: self.type_info,
                })?;
            }
        }
        Ok(())
    }
}

fn error<E, T>(e: T) -> E
where
    T: Display,
    E: serde::ser::Error,
{
    serde::ser::Error::custom(e)
}
