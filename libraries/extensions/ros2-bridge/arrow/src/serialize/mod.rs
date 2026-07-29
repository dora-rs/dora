use std::{borrow::Cow, collections::HashMap, fmt::Display};

use arrow::{
    array::{Array, ArrayRef, AsArray},
    error,
};
use dora_ros2_bridge_msg_gen::types::{
    MemberType,
    primitives::{GenericString, NestableType},
};
use eyre::Context;
use serde::ser::SerializeTupleStruct;

use super::{DUMMY_STRUCT_NAME, TypeInfo};

mod array;
pub mod defaults;
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
                Err(error(format!(
                    "given struct has unknown field {column_name}"
                )))?;
            }
        }
        if input.len() > 1 {
            Err(error(format!(
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

impl TypedValue<'_> {
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
                            check_single_element(string_array.len(), "StringArray")?;
                            check_not_null(string_array, "StringArray")?;
                            string_array.value(0)
                        } else {
                            let string_array = column
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array"))?;
                            check_single_element(string_array.len(), "LargeStringArray")?;
                            check_not_null(string_array, "LargeStringArray")?;
                            string_array.value(0)
                        };
                        if let GenericString::BoundedString(max) = t
                            && string.len() > *max
                        {
                            return Err(error(format!(
                                "string length {} exceeds BoundedString max {}",
                                string.len(),
                                max
                            )));
                        }
                        s.serialize_field(string)?;
                    }
                    GenericString::WString | GenericString::BoundedWString(_) => {
                        let string = if let Some(string_array) = column.as_string_opt::<i32>() {
                            check_single_element(string_array.len(), "StringArray (WString)")?;
                            check_not_null(string_array, "StringArray (WString)")?;
                            string_array.value(0)
                        } else {
                            let string_array = column
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array for WString"))?;
                            check_single_element(string_array.len(), "LargeStringArray (WString)")?;
                            check_not_null(string_array, "LargeStringArray (WString)")?;
                            string_array.value(0)
                        };
                        if let GenericString::BoundedWString(max) = t {
                            let utf16_len = string.encode_utf16().count();
                            if utf16_len > *max {
                                return Err(error(format!(
                                    "wstring length {} exceeds BoundedWString max {}",
                                    utf16_len, max
                                )));
                            }
                        }
                        let utf16: Vec<u16> = string.encode_utf16().collect();
                        s.serialize_field(&utf16)?;
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
                    max_size: None,
                })?;
            }
            dora_ros2_bridge_msg_gen::types::MemberType::BoundedSequence(v) => {
                s.serialize_field(&sequence::SequenceSerializeWrapper {
                    item_type: &v.value_type,
                    column: column.as_ref(),
                    type_info: self.type_info,
                    max_size: Some(v.max_size),
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

fn check_single_element<E: serde::ser::Error>(len: usize, type_name: &str) -> Result<(), E> {
    if len != 1 {
        return Err(serde::ser::Error::custom(format!(
            "expected single-element {type_name}, got length {len}"
        )));
    }
    Ok(())
}

/// Reject a null in a single-element scalar/string field column.
///
/// ROS2 messages (and the CDR wire format) have no null concept, but
/// `Array::value(0)` returns the physical buffer slot regardless of the
/// validity bit — for a null it silently yields a default (`0`/`false`/`""`).
/// Serializing that would invent a value the producer never sent, so reject it
/// instead, matching the sibling `arrow-convert` and `mavlink2-bridge` layers
/// which also refuse nulls rather than substitute a default. Callers must have
/// already checked that the column has exactly one element.
fn check_not_null<E: serde::ser::Error>(
    array: &dyn arrow::array::Array,
    type_name: &str,
) -> Result<(), E> {
    if array.is_null(0) {
        return Err(serde::ser::Error::custom(format!(
            "{type_name} field is null at row 0, but ROS2 messages cannot represent null"
        )));
    }
    Ok(())
}

/// Guard for fixed-size array fields: a ROS2 `T[N]` array has no length prefix
/// on the CDR wire, so the Arrow column must have exactly `expected` elements.
/// A mismatch would otherwise emit the wrong element count into a fixed tuple
/// and silently corrupt the message (dora-rs/dora#2027).
fn check_array_len<E: serde::ser::Error>(actual: usize, expected: usize) -> Result<(), E> {
    if actual != expected {
        return Err(serde::ser::Error::custom(format!(
            "expected array with length {expected}, got length {actual}"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::{collections::HashMap, sync::Arc};

    use arrow::{
        array::{ArrayRef, Int32Array, StringArray, StructArray},
        datatypes::{DataType, Field},
    };
    use byteorder::LittleEndian;
    use dora_ros2_bridge_msg_gen::types::{
        Member, MemberType, Message,
        primitives::{BasicType, GenericString, NestableType},
    };

    use super::*;

    /// A one-field message whose single member has the given name/type.
    fn single_field_message(
        field: &str,
        ty: NestableType,
    ) -> Arc<HashMap<String, HashMap<String, Message>>> {
        let message = Message {
            package: "test_msgs".to_string(),
            name: "OneField".to_string(),
            members: vec![Member {
                name: field.to_string(),
                r#type: MemberType::NestableType(ty),
                default: None,
            }],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("OneField".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_msgs".to_string(), package);
        Arc::new(messages)
    }

    fn struct_of(field: &str, data_type: DataType, column: ArrayRef) -> ArrayRef {
        Arc::new(StructArray::from(vec![(
            Arc::new(Field::new(field, data_type, true)),
            column,
        )])) as ArrayRef
    }

    fn serialize(
        messages: &Arc<HashMap<String, HashMap<String, Message>>>,
        value: &ArrayRef,
    ) -> Result<Vec<u8>, String> {
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("OneField"),
            messages: messages.clone(),
        };
        let typed = TypedValue {
            value,
            type_info: &type_info,
        };
        cdr_encoding::to_vec::<_, LittleEndian>(&typed).map_err(|e| e.to_string())
    }

    #[test]
    fn null_scalar_field_is_rejected() {
        let messages = single_field_message("x", NestableType::BasicType(BasicType::I32));
        // A present value serializes fine...
        let ok = struct_of(
            "x",
            DataType::Int32,
            Arc::new(Int32Array::from(vec![Some(7)])),
        );
        assert!(serialize(&messages, &ok).is_ok());
        // ...but a null must be rejected rather than silently encoded as 0.
        let null = struct_of("x", DataType::Int32, Arc::new(Int32Array::from(vec![None])));
        let err = serialize(&messages, &null).unwrap_err();
        assert!(
            err.to_string().contains("null"),
            "expected a null-rejection error, got: {err}"
        );
    }

    #[test]
    fn null_string_field_is_rejected() {
        let messages =
            single_field_message("s", NestableType::GenericString(GenericString::String));
        let ok = struct_of(
            "s",
            DataType::Utf8,
            Arc::new(StringArray::from(vec![Some("hi")])),
        );
        assert!(serialize(&messages, &ok).is_ok());
        let null = struct_of(
            "s",
            DataType::Utf8,
            Arc::new(StringArray::from(vec![Option::<&str>::None])),
        );
        let err = serialize(&messages, &null).unwrap_err();
        assert!(
            err.to_string().contains("null"),
            "expected a null-rejection error, got: {err}"
        );
    }
}
