use std::{any::type_name, borrow::Cow, marker::PhantomData, sync::Arc};

use arrow::{
    array::{Array, ArrayRef, AsArray, OffsetSizeTrait, PrimitiveArray},
    datatypes::{self, ArrowPrimitiveType, UInt8Type},
};
use dora_ros2_bridge_msg_gen::types::primitives::{BasicType, GenericString, NestableType};
use serde::ser::SerializeSeq;

use crate::TypeInfo;

use super::{TypedValue, error};

/// Serialize a variable-sized sequence.
pub struct SequenceSerializeWrapper<'a> {
    pub item_type: &'a NestableType,
    pub column: &'a ArrayRef,
    pub type_info: &'a TypeInfo<'a>,
    /// Maximum number of elements (for BoundedSequence). None = unbounded.
    pub max_size: Option<usize>,
}

impl serde::Serialize for SequenceSerializeWrapper<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        // `entry` is the array passed to the element serializers; `seq_len` is
        // the true number of sequence elements used for the BoundedSequence
        // bound check. For `List`/`LargeList` these coincide (`entry` is the
        // inner element array). For `Binary`/`LargeBinary`, `entry` is a 1-row
        // binary array — its `len()` is the row count (always 1), so the
        // element count must be read from the row's byte length instead
        // (see #2816).
        let (entry, seq_len) = if let Some(list) = self.column.as_list_opt::<i32>() {
            if list.len() != 1 {
                return Err(error(format!(
                    "expected single-element list, got length {}",
                    list.len()
                )));
            }
            let entry = list.value(0);
            let seq_len = entry.len();
            (entry, seq_len)
        } else if let Some(list) = self.column.as_list_opt::<i64>() {
            if list.len() != 1 {
                return Err(error(format!(
                    "expected single-element large list, got length {}",
                    list.len()
                )));
            }
            let entry = list.value(0);
            let seq_len = entry.len();
            (entry, seq_len)
        } else if let Some(list) = self.column.as_binary_opt::<i32>() {
            if list.len() != 1 {
                return Err(error(format!(
                    "expected single-element binary, got length {}",
                    list.len()
                )));
            }
            let seq_len = list.value(0).len();
            (Arc::new(list.slice(0, 1)) as ArrayRef, seq_len)
        } else if let Some(list) = self.column.as_binary_opt::<i64>() {
            if list.len() != 1 {
                return Err(error(format!(
                    "expected single-element large binary, got length {}",
                    list.len()
                )));
            }
            let seq_len = list.value(0).len();
            (Arc::new(list.slice(0, 1)) as ArrayRef, seq_len)
        } else {
            return Err(error(format!(
                "value is not compatible with expected sequence type: {:?}",
                self.column
            )));
        };
        // Enforce BoundedSequence max_size
        if let Some(max) = self.max_size
            && seq_len > max
        {
            return Err(error(format!(
                "sequence length {seq_len} exceeds BoundedSequence max_size {max}"
            )));
        }
        match &self.item_type {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int8Type>,
                }
                .serialize(serializer),
                BasicType::I16 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int16Type>,
                }
                .serialize(serializer),
                BasicType::I32 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int32Type>,
                }
                .serialize(serializer),
                BasicType::I64 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Int64Type>,
                }
                .serialize(serializer),
                BasicType::U8 | BasicType::Char | BasicType::Byte => {
                    ByteSequence { value: &entry }.serialize(serializer)
                }
                BasicType::U16 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt16Type>,
                }
                .serialize(serializer),
                BasicType::U32 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt32Type>,
                }
                .serialize(serializer),
                BasicType::U64 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt64Type>,
                }
                .serialize(serializer),
                BasicType::F32 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Float32Type>,
                }
                .serialize(serializer),
                BasicType::F64 => BasicSequence {
                    value: &entry,
                    ty: PhantomData::<datatypes::Float64Type>,
                }
                .serialize(serializer),
                BasicType::Bool => BoolArray { value: &entry }.serialize(serializer),
            },
            NestableType::NamedType(name) => {
                let array = entry
                    .as_struct_opt()
                    .ok_or_else(|| error("not a struct array"))?;
                let mut seq = serializer.serialize_seq(Some(array.len()))?;
                for i in 0..array.len() {
                    let row = array.slice(i, 1);
                    seq.serialize_element(&TypedValue {
                        value: &(Arc::new(row) as ArrayRef),
                        type_info: &TypeInfo {
                            package_name: Cow::Borrowed(&self.type_info.package_name),
                            message_name: Cow::Borrowed(&name.0),
                            messages: self.type_info.messages.clone(),
                        },
                    })?;
                }
                seq.end()
            }
            NestableType::NamespacedType(reference) => {
                if reference.namespace != "msg" {
                    return Err(error(format!(
                        "sequence references non-message type {reference:?}"
                    )));
                }

                let array = entry
                    .as_struct_opt()
                    .ok_or_else(|| error("not a struct array"))?;
                let mut seq = serializer.serialize_seq(Some(array.len()))?;
                for i in 0..array.len() {
                    let row = array.slice(i, 1);
                    seq.serialize_element(&TypedValue {
                        value: &(Arc::new(row) as ArrayRef),
                        type_info: &TypeInfo {
                            package_name: Cow::Borrowed(&reference.package),
                            message_name: Cow::Borrowed(&reference.name),
                            messages: self.type_info.messages.clone(),
                        },
                    })?;
                }
                seq.end()
            }
            NestableType::GenericString(s) => match s {
                GenericString::String | GenericString::BoundedString(_) => {
                    match entry.as_string_opt::<i32>() {
                        Some(array) => serialize_arrow_string(serializer, array),
                        None => {
                            let array = entry
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array"))?;
                            serialize_arrow_string(serializer, array)
                        }
                    }
                }
                GenericString::WString | GenericString::BoundedWString(_) => {
                    match entry.as_string_opt::<i32>() {
                        Some(array) => serialize_arrow_wstring(serializer, array),
                        None => {
                            let array = entry
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array for WString"))?;
                            serialize_arrow_wstring(serializer, array)
                        }
                    }
                }
            },
        }
    }
}

fn serialize_arrow_string<S, O>(
    serializer: S,
    array: &arrow::array::GenericByteArray<datatypes::GenericStringType<O>>,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    let mut seq = serializer.serialize_seq(Some(array.len()))?;
    for s in array.iter() {
        seq.serialize_element(s.unwrap_or_default())?;
    }
    seq.end()
}

fn serialize_arrow_wstring<S, O>(
    serializer: S,
    array: &arrow::array::GenericByteArray<datatypes::GenericStringType<O>>,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    let mut seq = serializer.serialize_seq(Some(array.len()))?;
    for s in array.iter() {
        let utf16: Vec<u16> = s.unwrap_or_default().encode_utf16().collect();
        seq.serialize_element(&utf16)?;
    }
    seq.end()
}

struct BasicSequence<'a, T> {
    value: &'a ArrayRef,
    ty: PhantomData<T>,
}

impl<T> serde::Serialize for BasicSequence<'_, T>
where
    T: ArrowPrimitiveType,
    T::Native: serde::Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let array: &PrimitiveArray<T> = self
            .value
            .as_primitive_opt()
            .ok_or_else(|| error(format!("not a primitive {} array", type_name::<T>())))?;

        let mut seq = serializer.serialize_seq(Some(array.len()))?;

        for value in array.values() {
            seq.serialize_element(value)?;
        }

        seq.end()
    }
}

struct ByteSequence<'a> {
    value: &'a ArrayRef,
}

impl serde::Serialize for ByteSequence<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        if let Some(binary) = self.value.as_binary_opt::<i32>() {
            serialize_binary(serializer, binary)
        } else if let Some(binary) = self.value.as_binary_opt::<i64>() {
            serialize_binary(serializer, binary)
        } else {
            BasicSequence {
                value: self.value,
                ty: PhantomData::<UInt8Type>,
            }
            .serialize(serializer)
        }
    }
}

fn serialize_binary<S, O>(
    serializer: S,
    binary: &arrow::array::GenericByteArray<datatypes::GenericBinaryType<O>>,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    // `binary` is always a 1-row array (sliced in `SequenceSerializeWrapper`),
    // so the CDR sequence is the *bytes of that single row*, not the rows. The
    // previous `binary.iter()` iterated rows (`binary.len()` == 1) and handed
    // each row `&[u8]` to `serialize_element`, which routes through serde's
    // `impl Serialize for [u8]` and emits *another* `serialize_seq`. Net wire
    // output for a byte field of length N was `[u32 1][u32 N][N bytes]` instead
    // of `[u32 N][N bytes]` — an extra length prefix and one nesting level too
    // deep, so a real ROS2 peer reads the length as 1 and misaligns every
    // field after the sequence. Serialize the row's bytes directly instead.
    let bytes: &[u8] = if binary.is_empty() {
        &[]
    } else {
        binary.value(0)
    };

    let mut seq = serializer.serialize_seq(Some(bytes.len()))?;

    for b in bytes {
        seq.serialize_element(b)?;
    }

    seq.end()
}

struct BoolArray<'a> {
    value: &'a ArrayRef,
}

impl serde::Serialize for BoolArray<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let array = self
            .value
            .as_boolean_opt()
            .ok_or_else(|| error("not a boolean array"))?;
        // Variable-length `sequence<bool>` / `bool[]` must be encoded with a
        // serde sequence so the CDR codec emits the mandatory u32 length
        // prefix. Using `serialize_tuple` here omits that prefix, which makes
        // the deserializer (and every field after it) decode garbage.
        let mut seq = serializer.serialize_seq(Some(array.len()))?;

        for value in array.values() {
            seq.serialize_element(&value)?;
        }

        seq.end()
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use arrow::{
        array::{
            ArrayRef, BinaryArray, BooleanArray, Int32Array, ListArray, StructArray, UInt8Array,
        },
        buffer::OffsetBuffer,
        datatypes::{DataType, Field},
    };
    use byteorder::LittleEndian;
    use dora_ros2_bridge_msg_gen::types::{
        Member, MemberType, Message,
        primitives::{BasicType, NestableType},
        sequences::{BoundedSequence, Sequence},
    };

    use super::*;

    /// Builds a message type with a `sequence<bool>` field followed by an
    /// `int32` field, so we can verify the trailing field survives.
    fn bool_seq_then_int32_message() -> Arc<HashMap<String, HashMap<String, Message>>> {
        let message = Message {
            package: "test_msgs".to_string(),
            name: "BoolSeqMsg".to_string(),
            members: vec![
                Member {
                    name: "flags".to_string(),
                    r#type: MemberType::Sequence(Sequence {
                        value_type: NestableType::BasicType(BasicType::Bool),
                    }),
                    default: None,
                },
                Member {
                    name: "tail".to_string(),
                    r#type: MemberType::NestableType(NestableType::BasicType(BasicType::I32)),
                    default: None,
                },
            ],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("BoolSeqMsg".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_msgs".to_string(), package);
        Arc::new(messages)
    }

    fn build_value(flags: &[bool], tail: i32) -> ArrayRef {
        let bool_array = BooleanArray::from(flags.to_vec());
        let list = ListArray::new(
            Arc::new(Field::new("item", DataType::Boolean, true)),
            OffsetBuffer::from_lengths([flags.len()]),
            Arc::new(bool_array),
            None,
        );
        let struct_array = StructArray::from(vec![
            (
                Arc::new(Field::new(
                    "flags",
                    DataType::List(Arc::new(Field::new("item", DataType::Boolean, true))),
                    false,
                )),
                Arc::new(list) as ArrayRef,
            ),
            (
                Arc::new(Field::new("tail", DataType::Int32, false)),
                Arc::new(Int32Array::from(vec![tail])) as ArrayRef,
            ),
        ]);
        Arc::new(struct_array) as ArrayRef
    }

    /// Regression test for #2032: a variable-length `sequence<bool>` must be
    /// CDR-encoded with a u32 length prefix. Without it, an actual CDR reader
    /// misparses both the sequence and every following field.
    #[test]
    fn bool_sequence_round_trips_through_real_cdr() {
        // Choose flags whose first element is `true` so that, under the buggy
        // (no length prefix) encoding, the reader would interpret the leading
        // `0x01` byte as a length of 1 and stop after a single element.
        let flags = vec![true, false, true, true];
        let tail = 0x1234_5678_i32;

        let messages = bool_seq_then_int32_message();
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("BoolSeqMsg"),
            messages,
        };
        let value = build_value(&flags, tail);
        let typed = TypedValue {
            value: &value,
            type_info: &type_info,
        };

        let bytes = cdr_encoding::to_vec::<_, LittleEndian>(&typed).expect("serialize to CDR");

        // The first 4 bytes must be the little-endian u32 sequence length.
        let prefix = u32::from_le_bytes(bytes[..4].try_into().unwrap());
        assert_eq!(
            prefix,
            flags.len() as u32,
            "missing/incorrect CDR length prefix for sequence<bool>"
        );

        // Decode with a real CDR reader as `(Vec<bool>, i32)` and confirm both
        // the sequence values and the trailing field are intact.
        let (decoded, _consumed): ((Vec<bool>, i32), _) =
            cdr_encoding::from_bytes::<(Vec<bool>, i32), LittleEndian>(&bytes)
                .expect("deserialize from CDR");
        assert_eq!(decoded.0, flags, "bool sequence values corrupted");
        assert_eq!(
            decoded.1, tail,
            "trailing field after bool sequence corrupted"
        );
    }

    /// Builds a message type with a `sequence<uint8>` (`uint8[]`) field
    /// followed by an `int32` field, so we can verify the trailing field
    /// survives the byte-sequence encoding.
    fn byte_seq_then_int32_message() -> Arc<HashMap<String, HashMap<String, Message>>> {
        let message = Message {
            package: "test_msgs".to_string(),
            name: "ByteSeqMsg".to_string(),
            members: vec![
                Member {
                    name: "data".to_string(),
                    r#type: MemberType::Sequence(Sequence {
                        value_type: NestableType::BasicType(BasicType::U8),
                    }),
                    default: None,
                },
                Member {
                    name: "tail".to_string(),
                    r#type: MemberType::NestableType(NestableType::BasicType(BasicType::I32)),
                    default: None,
                },
            ],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("ByteSeqMsg".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_msgs".to_string(), package);
        Arc::new(messages)
    }

    /// The `data` field as an Arrow `List<UInt8>` (the representation the
    /// deserializer emits).
    fn build_byte_value_list(data: &[u8], tail: i32) -> ArrayRef {
        let values = UInt8Array::from(data.to_vec());
        let list = ListArray::new(
            Arc::new(Field::new("item", DataType::UInt8, true)),
            OffsetBuffer::from_lengths([data.len()]),
            Arc::new(values),
            None,
        );
        let struct_array = StructArray::from(vec![
            (
                Arc::new(Field::new(
                    "data",
                    DataType::List(Arc::new(Field::new("item", DataType::UInt8, true))),
                    false,
                )),
                Arc::new(list) as ArrayRef,
            ),
            (
                Arc::new(Field::new("tail", DataType::Int32, false)),
                Arc::new(Int32Array::from(vec![tail])) as ArrayRef,
            ),
        ]);
        Arc::new(struct_array) as ArrayRef
    }

    /// The same `data` field as an Arrow `Binary` array (the idiomatic Arrow
    /// encoding for a bytes field, e.g. what a `pyarrow` producer yields).
    fn build_byte_value_binary(data: &[u8], tail: i32) -> ArrayRef {
        let binary = BinaryArray::from_vec(vec![data]);
        let struct_array = StructArray::from(vec![
            (
                Arc::new(Field::new("data", DataType::Binary, false)),
                Arc::new(binary) as ArrayRef,
            ),
            (
                Arc::new(Field::new("tail", DataType::Int32, false)),
                Arc::new(Int32Array::from(vec![tail])) as ArrayRef,
            ),
        ]);
        Arc::new(struct_array) as ArrayRef
    }

    /// Regression test for #2507: a byte sequence supplied as an Arrow `Binary`
    /// column must CDR-encode identically to the same value supplied as a
    /// `List<UInt8>`. The buggy encoder emitted an extra outer `u32` length
    /// prefix (`[u32 1][u32 N][N bytes]`), so a real ROS2 peer read the length
    /// as 1 and misaligned every following field.
    #[test]
    fn byte_sequence_binary_matches_list_through_real_cdr() {
        let data = vec![10u8, 20, 30];
        let tail = 0x1234_5678_i32;

        let messages = byte_seq_then_int32_message();
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ByteSeqMsg"),
            messages,
        };

        let list_value = build_byte_value_list(&data, tail);
        let list_bytes = cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &list_value,
            type_info: &type_info,
        })
        .expect("serialize List<UInt8> to CDR");

        let binary_value = build_byte_value_binary(&data, tail);
        let binary_bytes = cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &binary_value,
            type_info: &type_info,
        })
        .expect("serialize Binary to CDR");

        assert_eq!(
            binary_bytes, list_bytes,
            "Binary byte-sequence encoding diverges from List<UInt8>"
        );

        // The length prefix must be the byte count, not the row count (1).
        let prefix = u32::from_le_bytes(binary_bytes[..4].try_into().unwrap());
        assert_eq!(
            prefix,
            data.len() as u32,
            "missing/incorrect CDR length prefix for byte sequence"
        );

        // Decode with a real CDR reader and confirm the trailing field aligns.
        let (decoded, _consumed): ((Vec<u8>, i32), _) =
            cdr_encoding::from_bytes::<(Vec<u8>, i32), LittleEndian>(&binary_bytes)
                .expect("deserialize from CDR");
        assert_eq!(decoded.0, data, "byte sequence values corrupted");
        assert_eq!(
            decoded.1, tail,
            "trailing field after byte sequence corrupted"
        );
    }

    /// Builds a message type with a bounded `uint8[<=max_size]` field followed
    /// by an `int32` field.
    fn bounded_byte_seq_then_int32_message(
        max_size: usize,
    ) -> Arc<HashMap<String, HashMap<String, Message>>> {
        let message = Message {
            package: "test_msgs".to_string(),
            name: "ByteSeqMsg".to_string(),
            members: vec![
                Member {
                    name: "data".to_string(),
                    r#type: MemberType::BoundedSequence(BoundedSequence {
                        value_type: NestableType::BasicType(BasicType::U8),
                        max_size,
                    }),
                    default: None,
                },
                Member {
                    name: "tail".to_string(),
                    r#type: MemberType::NestableType(NestableType::BasicType(BasicType::I32)),
                    default: None,
                },
            ],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("ByteSeqMsg".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_msgs".to_string(), package);
        Arc::new(messages)
    }

    /// Regression test for #2816: a `BoundedSequence` (`uint8[<=N]`) supplied as
    /// an Arrow `Binary` column must enforce `max_size` on the byte count, just
    /// like the `List<UInt8>` representation. The buggy check compared against
    /// the 1-row array length (always 1), so an over-capacity Binary payload was
    /// silently accepted.
    #[test]
    fn bounded_byte_sequence_binary_enforces_max_size() {
        let max_size = 10;
        let over: Vec<u8> = (0..100u32).map(|i| i as u8).collect();
        let tail = 0x1234_5678_i32;

        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ByteSeqMsg"),
            messages: bounded_byte_seq_then_int32_message(max_size),
        };

        // Over-capacity as a `List<UInt8>` column: rejected (the already-working
        // representation).
        let list_err = cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &build_byte_value_list(&over, tail),
            type_info: &type_info,
        })
        .expect_err("List over max_size must be rejected");
        assert!(
            list_err
                .to_string()
                .contains("exceeds BoundedSequence max_size"),
            "unexpected error for List: {list_err}"
        );

        // Over-capacity as a `Binary` column: must be rejected identically.
        let binary_err = cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &build_byte_value_binary(&over, tail),
            type_info: &type_info,
        })
        .expect_err("Binary over max_size must be rejected");
        assert!(
            binary_err
                .to_string()
                .contains("sequence length 100 exceeds BoundedSequence max_size 10"),
            "unexpected error for Binary: {binary_err}"
        );

        // Within capacity as a `Binary` column: must still serialize fine.
        let ok: Vec<u8> = vec![1, 2, 3];
        cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &build_byte_value_binary(&ok, tail),
            type_info: &type_info,
        })
        .expect("Binary within max_size must serialize");
    }
}
