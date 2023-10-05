use arrow::{
    array::{
        make_array, Array, ArrayData, BooleanArray, Float32Array, Float64Array, Int16Array,
        Int32Array, Int64Array, Int8Array, StringArray, StructArray, UInt16Array, UInt32Array,
        UInt64Array, UInt8Array,
    },
    buffer::Buffer,
    compute::concat,
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
    data_type: DataType,
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
            let default = make_array(default_for_member(m, package_name, messages)?);
            Result::<_, eyre::Report>::Ok((
                Arc::new(Field::new(
                    m.name.clone(),
                    default.data_type().clone(),
                    true,
                )),
                default,
            ))
        })
        .collect::<Result<_, _>>()?;

    let default_struct: StructArray = default_struct_vec.into();

    Ok(TypeInfo {
        data_type: default_struct.data_type().clone(),
        defaults: default_struct.into(),
    })
}

pub fn default_for_member(
    m: &dora_ros2_bridge_msg_gen::types::Member,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> eyre::Result<ArrayData> {
    let value = match &m.r#type {
        MemberType::NestableType(t) => match t {
            NestableType::BasicType(_) | NestableType::GenericString(_) => match &m
                .default
                .as_deref()
            {
                Some([]) => eyre::bail!("empty default value not supported"),
                Some([default]) => preset_default_for_basic_type(t, default)
                    .with_context(|| format!("failed to parse default value for `{}`", m.name))?,
                Some(_) => eyre::bail!(
                    "there should be only a single default value for non-sequence types"
                ),
                None => default_for_nestable_type(t, package_name, messages)?,
            },
            NestableType::NamedType(_) => {
                if m.default.is_some() {
                    eyre::bail!("default values for nested types are not supported")
                } else {
                    default_for_nestable_type(t, package_name, messages)?
                }
            }
            NestableType::NamespacedType(_) => {
                default_for_nestable_type(t, package_name, messages)?
            }
        },
        MemberType::Array(array) => {
            list_default_values(m, &array.value_type, package_name, messages)?
        }
        MemberType::Sequence(seq) => {
            list_default_values(m, &seq.value_type, package_name, messages)?
        }
        MemberType::BoundedSequence(seq) => {
            list_default_values(m, &seq.value_type, package_name, messages)?
        }
    };
    Ok(value)
}

fn default_for_nestable_type(
    t: &NestableType,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> Result<ArrayData> {
    let empty = HashMap::new();
    let package_messages = messages.get(package_name).unwrap_or(&empty);
    let array = match t {
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
            BasicType::Byte => UInt8Array::from(vec![0u8] as Vec<u8>).into(),
            BasicType::Bool => BooleanArray::from(vec![false]).into(),
        },
        NestableType::GenericString(_) => StringArray::from(vec![""]).into(),
        NestableType::NamedType(name) => {
            let referenced_message = package_messages
                .get(&name.0)
                .context("unknown referenced message")?;

            default_for_referenced_message(referenced_message, package_name, messages)?
        }
        NestableType::NamespacedType(t) => {
            let referenced_package_messages = messages.get(&t.package).unwrap_or(&empty);
            let referenced_message = referenced_package_messages
                .get(&t.name)
                .context("unknown referenced message")?;
            default_for_referenced_message(referenced_message, &t.package, messages)?
        }
    };
    Ok(array)
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
                    true,
                )),
                make_array(default),
            ))
        })
        .collect::<Result<_, _>>()?;

    let struct_array: StructArray = fields.into();
    Ok(struct_array.into())
}

fn list_default_values(
    m: &dora_ros2_bridge_msg_gen::types::Member,
    value_type: &NestableType,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
) -> Result<ArrayData> {
    let defaults = match &m.default.as_deref() {
        Some([]) => eyre::bail!("empty default value not supported"),
        Some(defaults) => {
            let raw_array: Vec<Arc<dyn Array>> = defaults
                .iter()
                .map(|default| {
                    preset_default_for_basic_type(value_type, default)
                        .with_context(|| format!("failed to parse default value for `{}`", m.name))
                        .map(make_array)
                })
                .collect::<Result<_, _>>()?;
            let default_values = concat(
                raw_array
                    .iter()
                    .map(|data| data.as_ref())
                    .collect::<Vec<_>>()
                    .as_slice(),
            )
            .context("Failed to concatenate default list value")?;
            default_values.to_data()
        }
        None => {
            let default_nested_type =
                default_for_nestable_type(value_type, package_name, messages)?;

            let value_offsets = Buffer::from_slice_ref([0i64, 1]);

            let list_data_type = DataType::List(Arc::new(Field::new(
                &m.name,
                default_nested_type.data_type().clone(),
                true,
            )));
            // Construct a list array from the above two
            ArrayData::builder(list_data_type)
                .len(1)
                .add_buffer(value_offsets)
                .add_child_data(default_nested_type)
                .build()
                .context("Failed to build default list value")?
        }
    };

    Ok(defaults)
}
