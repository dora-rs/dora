use arrow::{
    array::{
        Array, ArrayData, BooleanArray, Float32Array, Float64Array, Int8Array, Int16Array,
        Int32Array, Int64Array, ListArray, StringArray, StructArray, UInt8Array, UInt16Array,
        UInt32Array, UInt64Array, make_array,
    },
    buffer::{OffsetBuffer, ScalarBuffer},
    compute::concat,
    datatypes::Field,
};
use dora_ros2_bridge_msg_gen::types::{
    MemberType, Message,
    primitives::{BasicType, NestableType},
};
use eyre::{Context, ContextCompat, Result};
use std::{collections::HashMap, sync::Arc, vec};

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
                None => default_for_nestable_type(t, package_name, messages, 1)?,
            },
            NestableType::NamedType(_) => {
                if m.default.is_some() {
                    eyre::bail!("default values for nested types are not supported")
                } else {
                    default_for_nestable_type(t, package_name, messages, 1)?
                }
            }
            NestableType::NamespacedType(_) => {
                default_for_nestable_type(t, package_name, messages, 1)?
            }
        },
        MemberType::Array(array) => list_default_values(
            m,
            &array.value_type,
            package_name,
            messages,
            Some(array.size),
        )?,
        MemberType::Sequence(seq) => {
            list_default_values(m, &seq.value_type, package_name, messages, None)?
        }
        MemberType::BoundedSequence(seq) => list_default_values(
            m,
            &seq.value_type,
            package_name,
            messages,
            Some(seq.max_size),
        )?,
    };
    Ok(value)
}

fn default_for_nestable_type(
    t: &NestableType,
    package_name: &str,
    messages: &HashMap<String, HashMap<String, Message>>,
    size: usize,
) -> Result<ArrayData> {
    let empty = HashMap::new();
    let package_messages = messages.get(package_name).unwrap_or(&empty);
    let array = match t {
        NestableType::BasicType(t) => match t {
            BasicType::I8 => Int8Array::from(vec![0; size]).into(),
            BasicType::I16 => Int16Array::from(vec![0; size]).into(),
            BasicType::I32 => Int32Array::from(vec![0; size]).into(),
            BasicType::I64 => Int64Array::from(vec![0; size]).into(),
            BasicType::U8 => UInt8Array::from(vec![0; size]).into(),
            BasicType::U16 => UInt16Array::from(vec![0; size]).into(),
            BasicType::U32 => UInt32Array::from(vec![0; size]).into(),
            BasicType::U64 => UInt64Array::from(vec![0; size]).into(),
            BasicType::F32 => Float32Array::from(vec![0.; size]).into(),
            BasicType::F64 => Float64Array::from(vec![0.; size]).into(),
            // `Char` is serialized/deserialized as a `UInt8` primitive (see
            // `array.rs` / `primitive.rs`), so its default must be a UInt8
            // array, not a `StringArray` (which the serialize path rejects).
            BasicType::Char => UInt8Array::from(vec![0u8; size]).into(),
            BasicType::Byte => UInt8Array::from(vec![0u8; size]).into(),
            BasicType::Bool => BooleanArray::from(vec![false; size]).into(),
        },
        NestableType::GenericString(_) => StringArray::from(vec![""; size]).into(),
        NestableType::NamedType(name) => {
            let referenced_message = package_messages
                .get(&name.0)
                .context("unknown referenced message")?;

            let one = default_for_referenced_message(referenced_message, package_name, messages)?;
            repeat_default(one, size)?
        }
        NestableType::NamespacedType(t) => {
            let referenced_package_messages = messages.get(&t.package).unwrap_or(&empty);
            let referenced_message = referenced_package_messages
                .get(&t.name)
                .context("unknown referenced message")?;
            let one = default_for_referenced_message(referenced_message, &t.package, messages)?;
            repeat_default(one, size)?
        }
    };
    Ok(array)
}

/// Repeat a length-1 default `size` times so a fixed-size `T[N]` (or N-element
/// sequence) default has a backing array of the right length. Without this,
/// `list_default_values` builds a `[0, N]` offset over a length-1 backing and
/// `ListArray::new` panics for `N > 1` (dora-rs/dora#2027).
fn repeat_default(one: ArrayData, size: usize) -> Result<ArrayData> {
    let array = make_array(one);
    if size == 0 {
        return Ok(arrow::array::new_empty_array(array.data_type()).to_data());
    }
    let copies = vec![array.as_ref(); size];
    Ok(concat(&copies)
        .context("failed to build repeated default value")?
        .to_data())
}

fn preset_default_for_basic_type(t: &NestableType, preset: &str) -> Result<ArrayData> {
    Ok(match t {
        NestableType::BasicType(t) => match t {
            BasicType::I8 => Int8Array::from(vec![
                preset
                    .parse::<i8>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::I16 => Int16Array::from(vec![
                preset
                    .parse::<i16>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::I32 => Int32Array::from(vec![
                preset
                    .parse::<i32>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::I64 => Int64Array::from(vec![
                preset
                    .parse::<i64>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::U8 => UInt8Array::from(vec![
                preset
                    .parse::<u8>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::U16 => UInt16Array::from(vec![
                preset
                    .parse::<u16>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::U32 => UInt32Array::from(vec![
                preset
                    .parse::<u32>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::U64 => UInt64Array::from(vec![
                preset
                    .parse::<u64>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::F32 => Float32Array::from(vec![
                preset
                    .parse::<f32>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::F64 => Float64Array::from(vec![
                preset
                    .parse::<f64>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            // `char` is a `UInt8` on the wire (see `primitive.rs`); its preset
            // default is the numeric byte value, not a string.
            BasicType::Char => UInt8Array::from(vec![
                preset
                    .parse::<u8>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::Byte => UInt8Array::from(vec![
                preset
                    .parse::<u8>()
                    .context("Could not parse preset default value")?,
            ])
            .into(),
            BasicType::Bool => BooleanArray::from(vec![
                preset
                    .parse::<bool>()
                    .context("could not parse preset default value")?,
            ])
            .into(),
        },
        NestableType::GenericString(_) => StringArray::from(vec![preset]).into(),
        _ => eyre::bail!(
            "preset default values for named/namespaced types are not supported in ROS2"
        ),
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
    size: Option<usize>,
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
            // Wrap the preset elements in a single-element `List`, exactly like
            // the no-default arm below. `ArraySerializeWrapper` /
            // `SequenceSerializeWrapper` unwrap the column via `as_list_opt`, so
            // returning the bare concatenated array would fail to serialize
            // ("value is not compatible with expected array type") for an
            // omitted field with a preset array/sequence default
            // (dora-rs/dora#2027 review).
            let len = default_values.len();
            let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, len as i32]));
            let field = Arc::new(Field::new("item", default_values.data_type().clone(), true));
            let list = ListArray::new(field, offsets, default_values, None);
            list.to_data()
        }
        None => {
            let size = size.unwrap_or(1);
            let default_nested_type =
                default_for_nestable_type(value_type, package_name, messages, size)?;
            let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, size as i32]));

            let field = Arc::new(Field::new(
                "item",
                default_nested_type.data_type().clone(),
                true,
            ));
            let list = ListArray::new(field, offsets, make_array(default_nested_type), None);
            list.to_data()
        }
    };

    Ok(defaults)
}

#[cfg(test)]
mod tests {
    use arrow::array::AsArray;
    use dora_ros2_bridge_msg_gen::types::{
        Member, MemberType, Message,
        primitives::{GenericString, NamedType, NestableType},
        sequences::Array as ArrayType,
    };

    use super::*;

    fn array_member(value_type: NestableType, size: usize) -> Member {
        Member {
            name: "field".to_string(),
            r#type: MemberType::Array(ArrayType { value_type, size }),
            default: None,
        }
    }

    /// #2027: an absent fixed-size `string[N]` (N>1) default must not panic in
    /// `ListArray::new` (the backing array previously had length 1 while the
    /// list offset declared N). The default list element must hold N strings.
    #[test]
    fn absent_fixed_string_array_default_has_correct_length() {
        let member = array_member(NestableType::GenericString(GenericString::String), 3);
        let messages = HashMap::new();
        let data = default_for_member(&member, "test_pkg", &messages).unwrap();
        let list = make_array(data);
        let list = list.as_list::<i32>();
        assert_eq!(list.len(), 1, "fixed array is a single-element list");
        let inner = list.value(0);
        assert_eq!(inner.len(), 3, "inner default must hold N elements");
        assert_eq!(
            inner.data_type(),
            &arrow::datatypes::DataType::Utf8,
            "string default element type"
        );
    }

    /// `char[N]` is serialized as `UInt8`, so its default element type must be
    /// `UInt8` (not `Utf8`) — otherwise the serialize path rejects it
    /// (dora-rs/dora#2027 review).
    #[test]
    fn absent_fixed_char_array_default_is_uint8() {
        use dora_ros2_bridge_msg_gen::types::primitives::BasicType;
        let member = array_member(NestableType::BasicType(BasicType::Char), 3);
        let messages = HashMap::new();
        let data = default_for_member(&member, "test_pkg", &messages).unwrap();
        let list = make_array(data);
        let list = list.as_list::<i32>();
        let inner = list.value(0);
        assert_eq!(inner.len(), 3);
        assert_eq!(
            inner.data_type(),
            &arrow::datatypes::DataType::UInt8,
            "char default element type must be UInt8"
        );
    }

    /// A preset `char` default (`char foo 100`) is the numeric byte value and
    /// must produce a `UInt8` array, not a `StringArray` the serialize path
    /// would reject (dora-rs/dora#2027 review).
    #[test]
    fn preset_char_default_is_uint8() {
        use dora_ros2_bridge_msg_gen::types::primitives::BasicType;
        let member = Member {
            name: "c".to_string(),
            r#type: MemberType::NestableType(NestableType::BasicType(BasicType::Char)),
            default: Some(vec!["100".to_string()]),
        };
        let messages = HashMap::new();
        let data = default_for_member(&member, "test_pkg", &messages).unwrap();
        let array = make_array(data);
        assert_eq!(array.data_type(), &arrow::datatypes::DataType::UInt8);
        assert_eq!(
            array.as_primitive::<arrow::datatypes::UInt8Type>().value(0),
            100
        );
    }

    /// Same for a preset `char[N]` default (`char[3] [0, 1, 127]`): every
    /// element must be `UInt8`. (The preset-array path returns the concatenated
    /// element array directly rather than list-wrapping it — a separate
    /// no-default path) so `ArraySerializeWrapper` can unwrap it, and each
    /// element must be `UInt8`.
    #[test]
    fn preset_char_array_default_is_uint8() {
        use dora_ros2_bridge_msg_gen::types::primitives::BasicType;
        let mut member = array_member(NestableType::BasicType(BasicType::Char), 3);
        member.default = Some(vec!["0".to_string(), "1".to_string(), "127".to_string()]);
        let messages = HashMap::new();
        let data = default_for_member(&member, "test_pkg", &messages).unwrap();
        let list = make_array(data);
        let list = list.as_list::<i32>();
        assert_eq!(list.len(), 1, "preset fixed array is a single-element list");
        let inner = list.value(0);
        assert_eq!(inner.data_type(), &arrow::datatypes::DataType::UInt8);
        assert_eq!(inner.len(), 3);
    }

    /// End-to-end: an Arrow input that OMITS a `char[3]` field with a preset
    /// default must serialize using that default (it previously errored with
    /// "value is not compatible with expected array type" because the preset
    /// default was a bare array, not a single-element list).
    #[test]
    fn preset_char_array_default_serializes() {
        use std::borrow::Cow;

        use arrow::array::{ArrayRef, StructArray};
        use byteorder::LittleEndian;
        use dora_ros2_bridge_msg_gen::types::primitives::BasicType;

        let mut member = array_member(NestableType::BasicType(BasicType::Char), 3);
        member.name = "c".to_string();
        member.default = Some(vec!["0".to_string(), "1".to_string(), "127".to_string()]);
        let message = Message {
            package: "test_pkg".to_string(),
            name: "M".to_string(),
            members: vec![member],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("M".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_pkg".to_string(), package);

        // Length-1 struct with no columns => every field uses its default.
        let value: ArrayRef = Arc::new(StructArray::new_empty_fields(1, None));
        let type_info = crate::TypeInfo {
            package_name: Cow::Borrowed("test_pkg"),
            message_name: Cow::Borrowed("M"),
            messages: Arc::new(messages),
        };
        let typed = crate::serialize::TypedValue {
            value: &value,
            type_info: &type_info,
        };
        let bytes = cdr_encoding::to_vec::<_, LittleEndian>(&typed)
            .expect("preset char[3] default must serialize");
        // Fixed `char[3]` => 3 raw UInt8 bytes, no length prefix.
        assert_eq!(bytes, vec![0u8, 1, 127]);
    }

    /// Same guard for a fixed-size array of a referenced struct type.
    #[test]
    fn absent_fixed_struct_array_default_has_correct_length() {
        let inner = Message {
            package: "test_pkg".to_string(),
            name: "Inner".to_string(),
            members: vec![Member {
                name: "x".to_string(),
                r#type: MemberType::NestableType(NestableType::BasicType(
                    dora_ros2_bridge_msg_gen::types::primitives::BasicType::I32,
                )),
                default: None,
            }],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("Inner".to_string(), inner);
        let mut messages = HashMap::new();
        messages.insert("test_pkg".to_string(), package);

        let member = array_member(NestableType::NamedType(NamedType("Inner".to_string())), 2);
        let data = default_for_member(&member, "test_pkg", &messages).unwrap();
        let list = make_array(data);
        let list = list.as_list::<i32>();
        assert_eq!(list.len(), 1);
        assert_eq!(
            list.value(0).len(),
            2,
            "inner struct default must hold N rows"
        );
    }
}
