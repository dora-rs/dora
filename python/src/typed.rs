use std::collections::HashMap;

use dora_ros2_bridge_msg_gen::types::{
    primitives::{BasicType, NestableType},
    MemberType, Message,
};
use eyre::{Context, ContextCompat};
use serde::ser::SerializeStruct;

#[derive(Debug, Clone, PartialEq)]
pub struct TypedValue<'a> {
    pub value: &'a serde_yaml::Value,
    pub type_info: &'a TypeInfo,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TypeInfo {
    Struct {
        name: String,
        fields: Vec<(String, TypeInfo)>,
    },
    I32,
    F32,
}

impl TypeInfo {
    pub fn for_message(
        messages: &HashMap<String, HashMap<String, Message>>,
        message_type: &str,
    ) -> eyre::Result<Self> {
        let (package_name, type_name) = message_type
            .split_once("::")
            .context("message type must be of form `package::type`")?;
        let empty = HashMap::new();
        let package_messages = messages.get(package_name).unwrap_or(&empty);
        let message = package_messages
            .get(type_name)
            .context("unknown type name")?;
        Ok(Self::Struct {
            name: message.name.clone(),
            fields: message
                .members
                .iter()
                .map(|m| {
                    Result::<_, eyre::Report>::Ok((
                        m.name.clone(),
                        type_info_for_member(m, package_messages, messages)?,
                    ))
                })
                .collect::<Result<_, _>>()?,
        })
    }
}

fn type_info_for_member(
    m: &dora_ros2_bridge_msg_gen::types::Member,
    package_messages: &HashMap<String, Message>,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<TypeInfo> {
    Ok(match &m.r#type {
        MemberType::NestableType(t) => match t {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => todo!(),
                BasicType::I16 => todo!(),
                BasicType::I32 => TypeInfo::I32,
                BasicType::I64 => todo!(),
                BasicType::U8 => todo!(),
                BasicType::U16 => todo!(),
                BasicType::U32 => todo!(),
                BasicType::U64 => todo!(),
                BasicType::F32 => TypeInfo::F32,
                BasicType::F64 => todo!(),
                BasicType::Bool => todo!(),
                BasicType::Char => todo!(),
                BasicType::Byte => todo!(),
            },
            NestableType::NamedType(name) => {
                let referenced_message = package_messages
                    .get(&name.0)
                    .context("unknown referenced message")?;
                TypeInfo::for_message(messages, &referenced_message.name)?
            }
            NestableType::NamespacedType(t) => {
                let referenced_message = messages
                    .get(&t.namespace)
                    .and_then(|n| n.get(&t.name))
                    .context("unknown referenced message")?;
                TypeInfo::for_message(messages, &referenced_message.name)?
            }
            NestableType::GenericString(_) => todo!(),
        },
        MemberType::Array(_) => todo!(),
        MemberType::Sequence(_) => todo!(),
        MemberType::BoundedSequence(_) => {
            todo!()
        }
    })
}

impl serde::Serialize for TypedValue<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match &self.type_info {
            TypeInfo::I32 => {
                let number = self
                    .value
                    .as_i64()
                    .context("expected i32 value")
                    .and_then(|n| i32::try_from(n).context("value too large"))
                    .map_err(serde::ser::Error::custom)?;
                serializer.serialize_i32(number)
            }
            TypeInfo::F32 => {
                let number = self
                    .value
                    .as_f64()
                    .context("expected f32 value")
                    .map_err(serde::ser::Error::custom)? as f32;
                serializer.serialize_f32(number)
            }
            TypeInfo::Struct { name: _, fields } => {
                /// Serde requires that struct and field names are known at
                /// compile time with a `'static` lifetime, which is not
                /// possible in this case. Thus, we need to use dummy names
                /// instead.
                ///
                /// The actual names do not really matter because
                /// the CDR format of ROS2 does not encode struct or field
                /// names.
                const DUMMY_STRUCT_NAME: &str = "struct";
                const DUMMY_FIELD_NAME: &str = "field";

                let mut s = serializer.serialize_struct(DUMMY_STRUCT_NAME, fields.len())?;
                for (field_name, field_type) in fields {
                    let field_value = self
                        .value
                        .get(field_name)
                        .with_context(|| format!("value has no field `{field_name}`"))
                        .map_err(serde::ser::Error::custom)?;
                    s.serialize_field(
                        DUMMY_FIELD_NAME,
                        &TypedValue {
                            value: field_value,
                            type_info: field_type,
                        },
                    )?;
                }
                s.end()
            }
        }
    }
}
