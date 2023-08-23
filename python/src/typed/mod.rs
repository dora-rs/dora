use arrow::{
    array::{
        make_array, Array, ArrayData, BooleanArray, Float32Array, Float64Array, Int16Array,
        Int32Array, Int64Array, Int8Array, StringArray, StructArray, UInt16Array, UInt32Array,
        UInt64Array, UInt8Array,
    },
    datatypes::{DataType, Field},
};
use dora_ros2_bridge_msg_gen::types::{
    primitives::{BasicType, NestableType},
    MemberType, Message,
};
use eyre::{Context, ContextCompat, Result};
use std::{collections::HashMap, sync::Arc};

pub use serialize::TypedValue;

pub mod deserialize;
pub mod serialize;

#[derive(Debug, Clone, PartialEq)]
pub struct TypeInfo {
    fields: DataType,
    defaults: ArrayData,
}

pub fn for_message(
    messages: &HashMap<String, HashMap<String, Message>>,
    package_name: &str,
    message_name: &str,
) -> eyre::Result<TypeInfo> {
    let empty = HashMap::new();
    let package_messages = messages.get(package_name).unwrap_or(&empty);
    let message = package_messages
        .get(message_name)
        .context("unknown type name")?;
    let default_struct_vec: Vec<(Arc<Field>, Arc<dyn Array>)> = message
        .members
        .iter()
        .map(|m| {
            Result::<_, eyre::Report>::Ok((
                Arc::new(Field::new(
                    m.name.clone(),
                    type_info_for_member(m, package_name, messages)?,
                    false,
                )),
                make_array(default_for_member(m, package_name, messages)?),
            ))
        })
        .collect::<Result<_, _>>()?;
    let default_struct: StructArray = default_struct_vec.into();
    Ok(TypeInfo {
        fields: default_struct.data_type().clone(),
        defaults: default_struct.into(),
    })
}

fn type_info_for_member(
    m: &dora_ros2_bridge_msg_gen::types::Member,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<DataType> {
    let empty = HashMap::new();
    let package_messages = messages.get(package_name).unwrap_or(&empty);
    Ok(match &m.r#type {
        MemberType::NestableType(t) => match t {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => todo!(),
                BasicType::I16 => todo!(),
                BasicType::I32 => DataType::Int32,
                BasicType::I64 => todo!(),
                BasicType::U8 => todo!(),
                BasicType::U16 => todo!(),
                BasicType::U32 => todo!(),
                BasicType::U64 => todo!(),
                BasicType::F32 => DataType::Float32,
                BasicType::F64 => DataType::Float64,
                BasicType::Bool => todo!(),
                BasicType::Char => todo!(),
                BasicType::Byte => todo!(),
            },
            NestableType::NamedType(name) => {
                let referenced_message = package_messages
                    .get(&name.0)
                    .context("unknown referenced message")?;
                for_message(messages, package_name, &referenced_message.name)?.fields
            }
            NestableType::NamespacedType(t) => for_message(messages, &t.package, &t.name)?.fields,
            NestableType::GenericString(_) => DataType::Utf8,
        },
        MemberType::Array(array) => match array.value_type {
            _ => todo!(),
        },
        MemberType::Sequence(sequence) => match &sequence.value_type {
            NestableType::NamedType(name) => {
                let referenced_message = package_messages
                    .get(&name.0)
                    .context("unknown referenced message")?;
                for_message(messages, package_name, &referenced_message.name)?.fields
            }
            _ => todo!(),
        },
        MemberType::BoundedSequence(_) => {
            todo!()
        }
    })
}

pub fn default_for_member(
    m: &dora_ros2_bridge_msg_gen::types::Member,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<ArrayData> {
    let empty = HashMap::new();
    let package_messages = messages.get(package_name).unwrap_or(&empty);

    let value = match &m.r#type {
        MemberType::NestableType(t) => match t {
            t @ NestableType::BasicType(_) | t @ NestableType::GenericString(_) => match &m
                .default
                .as_deref()
            {
                Some([]) => eyre::bail!("empty default value not supported"),
                Some([default]) => preset_default_for_basic_type(t, &default)
                    .with_context(|| format!("failed to parse default value for `{}`", m.name))?,
                Some(_) => eyre::bail!(
                    "there should be only a single default value for non-sequence types"
                ),
                None => default_for_basic_type(t),
            },
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
        MemberType::Array(_) | MemberType::Sequence(_) | MemberType::BoundedSequence(_) => {
            todo!()
        }
    };
    Ok(value)
}

fn default_for_basic_type(t: &NestableType) -> ArrayData {
    match t {
        NestableType::BasicType(t) => match t {
            BasicType::I8 => Int8Array::from(vec![0]).into(),
            BasicType::I16 => Int16Array::from(vec![0]).into(),
            BasicType::I32 => Int32Array::from(vec![0]).into(),
            BasicType::I64 => Int64Array::from(vec![0]).into(),
            BasicType::U8 => UInt8Array::from(vec![0]).into(),
            BasicType::U16 => UInt16Array::from(vec![0]).into(),
            BasicType::U32 => UInt32Array::from(vec![0]).into(),
            BasicType::U64 => UInt64Array::from(vec![0]).into(),
            BasicType::F32 => Float32Array::from(vec![0.]).into(),
            BasicType::F64 => Float64Array::from(vec![0.]).into(),
            BasicType::Char => StringArray::from(vec![""]).into(),
            BasicType::Byte => UInt8Array::from(vec![] as Vec<u8>).into(),
            BasicType::Bool => BooleanArray::from(vec![false]).into(),
        },
        NestableType::GenericString(_) => StringArray::from(vec![""]).into(),
        _ => todo!(),
    }
}
fn preset_default_for_basic_type(t: &NestableType, preset: &str) -> Result<ArrayData> {
    Ok(match t {
        NestableType::BasicType(t) => match t {
            BasicType::I8 => Int8Array::from(vec![preset
                .parse::<i8>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::I16 => Int16Array::from(vec![preset
                .parse::<i16>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::I32 => Int32Array::from(vec![preset
                .parse::<i32>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::I64 => Int64Array::from(vec![preset
                .parse::<i64>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::U8 => UInt8Array::from(vec![preset
                .parse::<u8>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::U16 => UInt16Array::from(vec![preset
                .parse::<u16>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::U32 => UInt32Array::from(vec![preset
                .parse::<u32>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::U64 => UInt64Array::from(vec![preset
                .parse::<u64>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::F32 => Float32Array::from(vec![preset
                .parse::<f32>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::F64 => Float64Array::from(vec![preset
                .parse::<f64>()
                .context("Could not parse preset default value")?])
            .into(),
            BasicType::Char => StringArray::from(vec![preset]).into(),
            BasicType::Byte => UInt8Array::from(preset.as_bytes().to_owned()).into(),
            BasicType::Bool => BooleanArray::from(vec![preset
                .parse::<bool>()
                .context("could not parse preset default value")?])
            .into(),
        },
        NestableType::GenericString(_) => StringArray::from(vec![preset]).into(),
        _ => todo!(),
    })
}

fn default_for_referenced_message(
    referenced_message: &Message,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<ArrayData> {
    let fields: Vec<(Arc<Field>, Arc<dyn Array>)> = referenced_message
        .members
        .iter()
        .map(|m| {
            let default = default_for_member(m, package_name, messages)?;
            Result::<_, eyre::Report>::Ok((
                Arc::new(Field::new(
                    m.name.clone(),
                    default.data_type().clone(),
                    false,
                )),
                make_array(default),
            ))
        })
        .collect::<Result<_, _>>()?;

    let struct_array: StructArray = fields.into();
    Ok(struct_array.into())
}
