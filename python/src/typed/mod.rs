use dora_ros2_bridge_msg_gen::types::{
    primitives::{BasicType, NestableType},
    MemberType, Message,
};
use eyre::{Context, ContextCompat};
use std::collections::HashMap;

pub use serialize::TypedValue;

pub mod deserialize;
pub mod serialize;

#[derive(Debug, Clone, PartialEq)]
pub enum TypeInfo {
    Struct {
        name: String,
        fields: Vec<StructField>,
    },
    I32,
    F32,
    F64,
}

impl TypeInfo {
    pub fn for_message(
        messages: &HashMap<String, HashMap<String, Message>>,
        package_name: &str,
        message_name: &str,
    ) -> eyre::Result<Self> {
        let empty = HashMap::new();
        let package_messages = messages.get(package_name).unwrap_or(&empty);
        let message = package_messages
            .get(message_name)
            .context("unknown type name")?;
        Ok(Self::Struct {
            name: message.name.clone(),
            fields: message
                .members
                .iter()
                .map(|m| {
                    Result::<_, eyre::Report>::Ok(StructField {
                        name: m.name.clone(),
                        ty: type_info_for_member(m, package_name, messages)?,
                        default: default_for_member(m, package_name, messages)?,
                    })
                })
                .collect::<Result<_, _>>()?,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct StructField {
    pub name: String,
    pub ty: TypeInfo,
    pub default: Option<serde_yaml::Value>,
}

fn type_info_for_member(
    m: &dora_ros2_bridge_msg_gen::types::Member,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<TypeInfo> {
    let empty = HashMap::new();
    let package_messages = messages.get(package_name).unwrap_or(&empty);
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
                BasicType::F64 => TypeInfo::F64,
                BasicType::Bool => todo!(),
                BasicType::Char => todo!(),
                BasicType::Byte => todo!(),
            },
            NestableType::NamedType(name) => {
                let referenced_message = package_messages
                    .get(&name.0)
                    .context("unknown referenced message")?;
                TypeInfo::for_message(messages, package_name, &referenced_message.name)?
            }
            NestableType::NamespacedType(t) => {
                TypeInfo::for_message(messages, &t.package, &t.name)?
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

fn default_for_member(
    m: &dora_ros2_bridge_msg_gen::types::Member,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<Option<serde_yaml::Value>> {
    let empty = HashMap::new();
    let package_messages = messages.get(package_name).unwrap_or(&empty);

    let value = match &m.r#type {
        MemberType::NestableType(t) => match t {
            t @ NestableType::BasicType(_) | t @ NestableType::GenericString(_) => {
                match &m.default.as_deref() {
                    Some([]) => eyre::bail!("empty default value not supported"),
                    Some([default]) => serde_yaml::from_str(default).with_context(|| {
                        format!("failed to parse default value for `{}`", m.name)
                    })?,
                    Some(_) => eyre::bail!(
                        "there should be only a single default value for non-sequence types"
                    ),
                    None => default_for_basic_type(t),
                }
            }
            NestableType::NamedType(name) => {
                if m.default.is_some() {
                    eyre::bail!("default values for nested types are not supported")
                } else {
                    let referenced_message = package_messages
                        .get(&name.0)
                        .context("unknown referenced message")?;

                    default_for_referenced_message(referenced_message, package_name, messages)?
                }
            }
            NestableType::NamespacedType(t) => {
                let referenced_package_messages = messages.get(&t.package).unwrap_or(&empty);
                let referenced_message = referenced_package_messages
                    .get(&t.name)
                    .context("unknown referenced message")?;
                default_for_referenced_message(referenced_message, &t.package, messages)?
            }
        },
        MemberType::Array(_) | MemberType::Sequence(_) | MemberType::BoundedSequence(_) => m
            .default
            .as_ref()
            .map(|default| {
                Result::<_, eyre::Report>::Ok(serde_yaml::Value::Sequence(
                    default
                        .iter()
                        .map(|v| serde_yaml::from_str(v))
                        .collect::<Result<_, _>>()?,
                ))
            })
            .transpose()?,
    };
    Ok(value)
}

fn default_for_basic_type(t: &NestableType) -> Option<serde_yaml::Value> {
    match t {
        NestableType::BasicType(t) => match t {
            BasicType::I8
            | BasicType::I16
            | BasicType::I32
            | BasicType::I64
            | BasicType::U8
            | BasicType::U16
            | BasicType::U32
            | BasicType::U64
            | BasicType::F32
            | BasicType::F64
            | BasicType::Char
            | BasicType::Byte => Some(serde_yaml::Value::Number(0u32.into())),
            BasicType::Bool => Some(serde_yaml::Value::Bool(false)),
        },
        NestableType::GenericString(_) => Some(serde_yaml::Value::String(String::new())),
        _ => None,
    }
}

fn default_for_referenced_message(
    referenced_message: &Message,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<Option<serde_yaml::Value>> {
    let mapping: Option<_> = referenced_message
        .members
        .iter()
        .map(|m| {
            let default = default_for_member(m, package_name, messages)?;
            Ok(default.map(|d| (serde_yaml::Value::String(m.name.clone()), d)))
        })
        .collect::<Result<_, eyre::Report>>()?;
    Ok(mapping.map(serde_yaml::Value::Mapping))
}
