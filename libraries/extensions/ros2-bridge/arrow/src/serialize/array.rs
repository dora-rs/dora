use std::{any::type_name, borrow::Cow, marker::PhantomData, sync::Arc};

use arrow::{
    array::{Array, ArrayRef, AsArray, OffsetSizeTrait, PrimitiveArray},
    datatypes::{self, ArrowPrimitiveType},
};
use dora_ros2_bridge_msg_gen::types::{
    primitives::{BasicType, GenericString, NestableType},
    sequences,
};
use serde::ser::SerializeTuple;

use crate::TypeInfo;

use super::{
    TypedValue, check_array_len, check_string_bound, encode_wstring, error, reject_null_element,
    reject_null_string_element, string_bound,
};

/// Serialize an array with known size as tuple.
pub struct ArraySerializeWrapper<'a> {
    pub array_info: &'a sequences::Array,
    pub column: &'a ArrayRef,
    pub type_info: &'a TypeInfo<'a>,
}

impl serde::Serialize for ArraySerializeWrapper<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        // A byte-typed fixed array (`uint8[N]`, `char[N]`, `byte[N]`) may be
        // supplied as an Arrow `Binary`/`LargeBinary` column — the idiomatic
        // representation a `pyarrow` producer uses for a byte buffer. The
        // variable-length sequence path already accepts this (#2507/#2816); the
        // fixed-array path must be consistent, otherwise the same 16 bytes that
        // serialize fine as `uint8[]` fail as `uint8[16]`. The row's *bytes* are
        // the array elements, so handle it here before the generic `List`
        // extraction below.
        // `FixedSizeBinary(N)` is the Arrow type made for exactly this shape
        // (e.g. `pa.binary(16)` for a `uint8[16]` UUID, #4149).
        let value_type = &self.array_info.value_type;
        let size = self.array_info.size;
        if let Some(binary) = self.column.as_binary_opt::<i32>() {
            return serialize_bytes_fixed_array(
                serializer,
                binary.len(),
                || binary.value(0),
                value_type,
                size,
            );
        } else if let Some(binary) = self.column.as_binary_opt::<i64>() {
            return serialize_bytes_fixed_array(
                serializer,
                binary.len(),
                || binary.value(0),
                value_type,
                size,
            );
        } else if let Some(binary) = self.column.as_fixed_size_binary_opt() {
            return serialize_bytes_fixed_array(
                serializer,
                binary.len(),
                || binary.value(0),
                value_type,
                size,
            );
        }

        // `FixedSizeList<T, N>` is what `pa.list_(t, N)`, a 2-D numpy array or
        // a Polars `Array` dtype produce for `T[N]` (#4149); its length is
        // validated by `check_array_len` below like any other list.
        let entry = if let Some(list) = self.column.as_list_opt::<i32>() {
            single_row(list.len(), "list")?;
            list.value(0)
        } else if let Some(list) = self.column.as_list_opt::<i64>() {
            single_row(list.len(), "large list")?;
            list.value(0)
        } else if let Some(list) = self.column.as_fixed_size_list_opt() {
            single_row(list.len(), "fixed-size list")?;
            list.value(0)
        } else {
            return Err(error("value is not compatible with expected array type"));
        };

        match &self.array_info.value_type {
            NestableType::BasicType(t) => match t {
                BasicType::I8 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int8Type>,
                }
                .serialize(serializer),
                BasicType::I16 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int16Type>,
                }
                .serialize(serializer),
                BasicType::I32 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int32Type>,
                }
                .serialize(serializer),
                BasicType::I64 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Int64Type>,
                }
                .serialize(serializer),
                BasicType::U8 | BasicType::Char | BasicType::Byte => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt8Type>,
                }
                .serialize(serializer),
                BasicType::U16 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt16Type>,
                }
                .serialize(serializer),
                BasicType::U32 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt32Type>,
                }
                .serialize(serializer),
                BasicType::U64 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::UInt64Type>,
                }
                .serialize(serializer),
                BasicType::F32 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Float32Type>,
                }
                .serialize(serializer),
                BasicType::F64 => BasicArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                    ty: PhantomData::<datatypes::Float64Type>,
                }
                .serialize(serializer),
                BasicType::Bool => BoolArrayAsTuple {
                    len: self.array_info.size,
                    value: &entry,
                }
                .serialize(serializer),
            },
            NestableType::NamedType(name) => {
                let array = entry
                    .as_struct_opt()
                    .ok_or_else(|| error("not a struct array"))?;
                check_array_len(array.len(), self.array_info.size)?;
                let mut seq = serializer.serialize_tuple(self.array_info.size)?;
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
                check_array_len(array.len(), self.array_info.size)?;
                let mut seq = serializer.serialize_tuple(self.array_info.size)?;
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
                    let bound = string_bound(s);
                    match entry.as_string_opt::<i32>() {
                        Some(array) => {
                            serialize_arrow_string(serializer, array, self.array_info.size, bound)
                        }
                        None => {
                            let array = entry
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array"))?;
                            serialize_arrow_string(serializer, array, self.array_info.size, bound)
                        }
                    }
                }
                GenericString::WString | GenericString::BoundedWString(_) => {
                    let bound = string_bound(s);
                    match entry.as_string_opt::<i32>() {
                        Some(array) => {
                            serialize_arrow_wstring(serializer, array, self.array_info.size, bound)
                        }
                        None => {
                            let array = entry
                                .as_string_opt::<i64>()
                                .ok_or_else(|| error("expected string array for WString"))?;
                            serialize_arrow_wstring(serializer, array, self.array_info.size, bound)
                        }
                    }
                }
            },
        }
    }
}

/// Serialize a byte-typed fixed-size array (`uint8[N]`, `char[N]`, `byte[N]`)
/// supplied as a single-row Arrow `Binary`/`LargeBinary` column.
///
/// The single row's bytes are the array elements, so the fixed-size length
/// check (`check_array_len`, #2027) is applied to the byte count. Only byte
/// element types can originate from a `Binary` column; any other element type
/// is a genuine type mismatch and is rejected.
fn single_row<E: serde::ser::Error>(len: usize, kind: &str) -> Result<(), E> {
    if len != 1 {
        return Err(error(format!(
            "expected single-element {kind}, got length {len}"
        )));
    }
    Ok(())
}

/// Serializes the single row of a byte column (`Binary`, `LargeBinary` or
/// `FixedSizeBinary`) as a byte-typed fixed array.
fn serialize_bytes_fixed_array<'a, S>(
    serializer: S,
    rows: usize,
    row: impl FnOnce() -> &'a [u8],
    value_type: &NestableType,
    size: usize,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
{
    match value_type {
        NestableType::BasicType(BasicType::U8 | BasicType::Char | BasicType::Byte) => {}
        other => {
            return Err(error(format!(
                "Binary column is only valid for a byte-typed fixed array, not {other:?}"
            )));
        }
    }
    single_row(rows, "binary")?;
    let bytes = row();
    check_array_len(bytes.len(), size)?;
    let mut seq = serializer.serialize_tuple(size)?;
    for b in bytes {
        seq.serialize_element(b)?;
    }
    seq.end()
}

/// Serializes a primitive array with known size as tuple.
struct BasicArrayAsTuple<'a, T> {
    len: usize,
    value: &'a ArrayRef,
    ty: PhantomData<T>,
}

impl<T> serde::Serialize for BasicArrayAsTuple<'_, T>
where
    T: ArrowPrimitiveType,
    T::Native: serde::Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut seq = serializer.serialize_tuple(self.len)?;
        let array: &PrimitiveArray<T> = self
            .value
            .as_primitive_opt()
            .ok_or_else(|| error(format!("not a primitive {} array", type_name::<T>())))?;
        check_array_len(array.len(), self.len)?;
        reject_null_element(array, type_name::<T>())?;

        for value in array.values() {
            seq.serialize_element(value)?;
        }

        seq.end()
    }
}

struct BoolArrayAsTuple<'a> {
    len: usize,
    value: &'a ArrayRef,
}

impl serde::Serialize for BoolArrayAsTuple<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut seq = serializer.serialize_tuple(self.len)?;
        let array = self
            .value
            .as_boolean_opt()
            .ok_or_else(|| error("not a boolean array"))?;
        check_array_len(array.len(), self.len)?;
        reject_null_element(array, "bool")?;

        for value in array.values() {
            seq.serialize_element(&value)?;
        }

        seq.end()
    }
}

fn serialize_arrow_string<S, O>(
    serializer: S,
    array: &arrow::array::GenericByteArray<datatypes::GenericStringType<O>>,
    array_len: usize,
    bound: Option<usize>,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    check_array_len(array.len(), array_len)?;
    let mut seq = serializer.serialize_tuple(array_len)?;
    for (i, s) in array.iter().enumerate() {
        let s = reject_null_string_element(s, i)?;
        check_string_bound(s, bound)?;
        seq.serialize_element(s)?;
    }
    seq.end()
}

fn serialize_arrow_wstring<S, O>(
    serializer: S,
    array: &arrow::array::GenericByteArray<datatypes::GenericStringType<O>>,
    array_len: usize,
    bound: Option<usize>,
) -> Result<<S as serde::Serializer>::Ok, <S as serde::Serializer>::Error>
where
    S: serde::Serializer,
    O: OffsetSizeTrait,
{
    check_array_len(array.len(), array_len)?;
    let mut seq = serializer.serialize_tuple(array_len)?;
    for (i, s) in array.iter().enumerate() {
        seq.serialize_element(&encode_wstring(reject_null_string_element(s, i)?, bound)?)?;
    }
    seq.end()
}

#[cfg(test)]
mod tests {
    use std::{borrow::Cow, collections::HashMap, sync::Arc};

    use arrow::{
        array::{
            ArrayRef, BinaryArray, BooleanArray, FixedSizeBinaryArray, FixedSizeListArray,
            Float64Array, Int32Array, ListArray, StringArray, StructArray, UInt8Array,
        },
        buffer::{NullBuffer, OffsetBuffer},
        datatypes::{DataType, Field},
    };
    use byteorder::LittleEndian;
    use dora_ros2_bridge_msg_gen::types::{
        Member, MemberType, Message,
        primitives::{BasicType, GenericString, NestableType},
        sequences::Array as ArrayType,
    };

    use super::*;
    use crate::TypeInfo;

    /// A message with a single fixed-size `<field_type>[3]` array field.
    fn fixed_array_message(
        field_type: NestableType,
    ) -> Arc<HashMap<String, HashMap<String, Message>>> {
        let message = Message {
            package: "test_msgs".to_string(),
            name: "ArrMsg".to_string(),
            members: vec![Member {
                name: "names".to_string(),
                r#type: MemberType::Array(ArrayType {
                    value_type: field_type,
                    size: 3,
                }),
                default: None,
            }],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("ArrMsg".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_msgs".to_string(), package);
        Arc::new(messages)
    }

    fn serializes_ok(
        messages: &Arc<HashMap<String, HashMap<String, Message>>>,
        names: &[&str],
    ) -> bool {
        // All-present case: delegate to the nullable helper below, which handles
        // the (superset) `Vec<Option<&str>>` shape.
        serializes_ok_with_nulls(messages, names.iter().map(|s| Some(*s)).collect())
    }

    /// #2027: a fixed-size `string[3]` field given the wrong number of elements
    /// must error rather than silently emit a wrong-sized (length-prefix-less)
    /// CDR tuple.
    #[test]
    fn fixed_string_array_length_is_checked() {
        let messages = fixed_array_message(NestableType::GenericString(GenericString::String));
        assert!(
            !serializes_ok(&messages, &["a", "b"]),
            "size-3 field with 2 elements must error"
        );
        assert!(
            serializes_ok(&messages, &["a", "b", "c"]),
            "size-3 field with 3 elements must serialize"
        );
    }

    /// Same guard on the `wstring[N]` path.
    #[test]
    fn fixed_wstring_array_length_is_checked() {
        let messages = fixed_array_message(NestableType::GenericString(GenericString::WString));
        assert!(
            !serializes_ok(&messages, &["a", "b", "c", "d"]),
            "size-3 field with 4 elements must error"
        );
        assert!(
            serializes_ok(&messages, &["a", "b", "c"]),
            "size-3 field with 3 elements must serialize"
        );
    }

    /// Each element of a `string<=N>[M]` field is bounded like a scalar
    /// `string<=N>` field.
    #[test]
    fn fixed_bounded_string_array_element_length_is_checked() {
        let messages =
            fixed_array_message(NestableType::GenericString(GenericString::BoundedString(2)));
        assert!(
            !serializes_ok(&messages, &["ab", "abc", "a"]),
            "a 3-byte element of a string<=2[3] field must error"
        );
        assert!(serializes_ok(&messages, &["ab", "", "a"]));
    }

    /// Same for `wstring<=N>[M]`.
    #[test]
    fn fixed_bounded_wstring_array_element_length_is_checked() {
        let messages = fixed_array_message(NestableType::GenericString(
            GenericString::BoundedWString(2),
        ));
        assert!(
            !serializes_ok(&messages, &["ab", "abc", "a"]),
            "a 3-unit element of a wstring<=2[3] field must error"
        );
        assert!(serializes_ok(&messages, &["ab", "", "a"]));
    }

    /// Like `serializes_ok`, but the inner `StringArray` may contain nulls so we
    /// can exercise the null-element path of a fixed `string[N]` field.
    fn serializes_ok_with_nulls(
        messages: &Arc<HashMap<String, HashMap<String, Message>>>,
        names: Vec<Option<&str>>,
    ) -> bool {
        let item = Arc::new(Field::new("item", DataType::Utf8, true));
        let list = ListArray::new(
            item.clone(),
            OffsetBuffer::from_lengths([names.len()]),
            Arc::new(StringArray::from(names)),
            None,
        );
        let struct_array = StructArray::from(vec![(
            Arc::new(Field::new("names", DataType::List(item), false)),
            Arc::new(list) as ArrayRef,
        )]);
        let value = Arc::new(struct_array) as ArrayRef;
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ArrMsg"),
            messages: messages.clone(),
        };
        let typed = TypedValue {
            value: &value,
            type_info: &type_info,
        };
        cdr_encoding::to_vec::<_, LittleEndian>(&typed).is_ok()
    }

    /// A null element in a fixed `string[N]` field must error rather than be
    /// silently encoded as `""`. The scalar-string path already rejects nulls
    /// (ROS2/CDR has no null concept); the array path must honor the same
    /// invariant instead of inventing an empty string the producer never sent.
    #[test]
    fn fixed_string_array_null_element_is_rejected() {
        let messages = fixed_array_message(NestableType::GenericString(GenericString::String));
        assert!(
            !serializes_ok_with_nulls(&messages, vec![Some("a"), None, Some("c")]),
            "a null element in a string[3] field must error, not encode as \"\""
        );
        assert!(
            serializes_ok_with_nulls(&messages, vec![Some("a"), Some("b"), Some("c")]),
            "an all-present string[3] field must still serialize"
        );
    }

    /// Same null guard on the `wstring[N]` path.
    #[test]
    fn fixed_wstring_array_null_element_is_rejected() {
        let messages = fixed_array_message(NestableType::GenericString(GenericString::WString));
        assert!(
            !serializes_ok_with_nulls(&messages, vec![Some("a"), None, Some("c")]),
            "a null element in a wstring[3] field must error, not encode as \"\""
        );
        assert!(
            serializes_ok_with_nulls(&messages, vec![Some("a"), Some("b"), Some("c")]),
            "an all-present wstring[3] field must still serialize"
        );
    }

    /// A message with a single fixed-size `uint8[<size>]` field followed by an
    /// `int32` field, so we can verify the trailing field survives the byte
    /// encoding (mirrors the `unique_identifier_msgs/msg/UUID` shape from #3065).
    fn byte_array_then_int32_message(
        size: usize,
    ) -> Arc<HashMap<String, HashMap<String, Message>>> {
        let message = Message {
            package: "test_msgs".to_string(),
            name: "ByteArrMsg".to_string(),
            members: vec![
                Member {
                    name: "data".to_string(),
                    r#type: MemberType::Array(ArrayType {
                        value_type: NestableType::BasicType(BasicType::U8),
                        size,
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
        package.insert("ByteArrMsg".to_string(), message);
        let mut messages = HashMap::new();
        messages.insert("test_msgs".to_string(), package);
        Arc::new(messages)
    }

    /// The `data` field as an Arrow `List<UInt8>` (the representation dora's
    /// deserializer emits).
    fn build_byte_array_list(data: &[u8], tail: i32) -> ArrayRef {
        let list = ListArray::new(
            Arc::new(Field::new("item", DataType::UInt8, true)),
            OffsetBuffer::from_lengths([data.len()]),
            Arc::new(UInt8Array::from(data.to_vec())),
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

    /// The same `data` field as an Arrow `Binary` column (what a `pyarrow`
    /// producer idiomatically yields for a byte buffer).
    fn build_byte_array_binary(data: &[u8], tail: i32) -> ArrayRef {
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

    /// #3065: a byte-valued fixed-size array (`uint8[N]`) supplied as an Arrow
    /// `Binary` column must serialize, and encode identically to the same value
    /// supplied as `List<UInt8>`. A fixed array has no CDR length prefix, so the
    /// trailing field must stay byte-aligned across both representations.
    #[test]
    fn fixed_byte_array_binary_matches_list() {
        let data = vec![10u8, 20, 30];
        let tail = 0x1234_5678_i32;

        let messages = byte_array_then_int32_message(data.len());
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ByteArrMsg"),
            messages,
        };

        let list_bytes = cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &build_byte_array_list(&data, tail),
            type_info: &type_info,
        })
        .expect("serialize List<UInt8> fixed array to CDR");

        let binary_bytes = cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &build_byte_array_binary(&data, tail),
            type_info: &type_info,
        })
        .expect("serialize Binary fixed array to CDR");

        assert_eq!(
            binary_bytes, list_bytes,
            "Binary fixed-array encoding diverges from List<UInt8>"
        );

        // Decode with a real CDR reader to confirm the trailing field aligns.
        // A `uint8[3]` field is a bare 3-tuple (no length prefix) on the wire.
        let (decoded, _consumed): (([u8; 3], i32), _) =
            cdr_encoding::from_bytes::<([u8; 3], i32), LittleEndian>(&binary_bytes)
                .expect("deserialize fixed array from CDR");
        assert_eq!(
            decoded.0,
            data.as_slice(),
            "fixed byte-array values corrupted"
        );
        assert_eq!(
            decoded.1, tail,
            "trailing field after fixed byte array corrupted"
        );
    }

    /// #3065: the fixed-size length check (#2027) must also apply to the `Binary`
    /// representation — a wrong-length byte buffer must error, not silently emit
    /// a wrong-sized tuple.
    #[test]
    fn fixed_byte_array_binary_length_is_checked() {
        let type_info = |size| TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ByteArrMsg"),
            messages: byte_array_then_int32_message(size),
        };

        // 3 bytes into a `uint8[3]` field: serializes.
        cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &build_byte_array_binary(&[1, 2, 3], 0),
            type_info: &type_info(3),
        })
        .expect("size-3 field with 3 bytes must serialize");

        // 2 bytes into a `uint8[3]` field: must error.
        cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &build_byte_array_binary(&[1, 2], 0),
            type_info: &type_info(3),
        })
        .expect_err("size-3 field with 2 bytes must error");
    }

    /// Build a struct with a `names` field wrapping the given `Int32` list, so
    /// we can drive an `int32[3]` message column with per-element nulls.
    fn build_int32_array(values: Vec<Option<i32>>) -> ArrayRef {
        let item = Arc::new(Field::new("item", DataType::Int32, true));
        let list = ListArray::new(
            item.clone(),
            OffsetBuffer::from_lengths([values.len()]),
            Arc::new(Int32Array::from(values)),
            None,
        );
        let struct_array = StructArray::from(vec![(
            Arc::new(Field::new("names", DataType::List(item), false)),
            Arc::new(list) as ArrayRef,
        )]);
        Arc::new(struct_array) as ArrayRef
    }

    /// Same shape for a `Bool` list, so we can drive a `bool[3]` column.
    fn build_bool_array(values: Vec<Option<bool>>) -> ArrayRef {
        let item = Arc::new(Field::new("item", DataType::Boolean, true));
        let list = ListArray::new(
            item.clone(),
            OffsetBuffer::from_lengths([values.len()]),
            Arc::new(BooleanArray::from(values)),
            None,
        );
        let struct_array = StructArray::from(vec![(
            Arc::new(Field::new("names", DataType::List(item), false)),
            Arc::new(list) as ArrayRef,
        )]);
        Arc::new(struct_array) as ArrayRef
    }

    fn serializes_primitive_array(value: &ArrayRef, field_type: NestableType) -> bool {
        let messages = fixed_array_message(field_type);
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ArrMsg"),
            messages,
        };
        cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value,
            type_info: &type_info,
        })
        .is_ok()
    }

    /// #3271: a null element in a fixed `int32[N]` field must be rejected. The
    /// scalar and string-array paths already reject nulls; the primitive-array
    /// path previously iterated `array.values()`, which ignores the validity
    /// bitmap and silently encodes the raw buffer slot (`0`) — inventing a
    /// value the producer never sent.
    #[test]
    fn fixed_int32_array_null_element_is_rejected() {
        let with_null = build_int32_array(vec![Some(1), None, Some(3)]);
        assert!(
            !serializes_primitive_array(&with_null, NestableType::BasicType(BasicType::I32)),
            "a null element in an int32[3] field must error, not encode as 0"
        );
        let all_present = build_int32_array(vec![Some(1), Some(2), Some(3)]);
        assert!(
            serializes_primitive_array(&all_present, NestableType::BasicType(BasicType::I32)),
            "an all-present int32[3] field must still serialize"
        );
    }

    /// #3271: same guard on the fixed `bool[N]` path.
    #[test]
    fn fixed_bool_array_null_element_is_rejected() {
        let with_null = build_bool_array(vec![Some(true), None, Some(false)]);
        assert!(
            !serializes_primitive_array(&with_null, NestableType::BasicType(BasicType::Bool)),
            "a null element in a bool[3] field must error, not encode as false"
        );
        let all_present = build_bool_array(vec![Some(true), Some(false), Some(true)]);
        assert!(
            serializes_primitive_array(&all_present, NestableType::BasicType(BasicType::Bool)),
            "an all-present bool[3] field must still serialize"
        );
    }

    /// A null row of a fixed `uint8[3]` field — as a `List` or as a
    /// `Binary` column — must error. Both used to publish the row's masked
    /// values as real data.
    #[test]
    fn null_fixed_array_row_is_rejected() {
        let messages = fixed_array_message(NestableType::BasicType(BasicType::U8));
        let serializes_ok = |column: ArrayRef| {
            let value = Arc::new(StructArray::from(vec![(
                Arc::new(Field::new("names", column.data_type().clone(), true)),
                column,
            )])) as ArrayRef;
            let type_info = TypeInfo {
                package_name: Cow::Borrowed("test_msgs"),
                message_name: Cow::Borrowed("ArrMsg"),
                messages: messages.clone(),
            };
            cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
                value: &value,
                type_info: &type_info,
            })
            .is_ok()
        };
        let list = |nulls| {
            Arc::new(ListArray::new(
                Arc::new(Field::new("item", DataType::UInt8, true)),
                OffsetBuffer::from_lengths([3usize]),
                Arc::new(UInt8Array::from(vec![1, 2, 3])),
                nulls,
            )) as ArrayRef
        };
        let binary = |nulls| {
            Arc::new(BinaryArray::new(
                OffsetBuffer::from_lengths([3usize]),
                vec![1u8, 2, 3].into(),
                nulls,
            )) as ArrayRef
        };
        assert!(!serializes_ok(list(Some(NullBuffer::new_null(1)))));
        assert!(!serializes_ok(binary(Some(NullBuffer::new_null(1)))));
        assert!(
            serializes_ok(list(None)),
            "a valid row must still serialize"
        );
        assert!(
            serializes_ok(binary(None)),
            "a valid row must still serialize"
        );
    }
    /// CDR bytes of a single-field `ArrMsg { names: <field_type>[3] }` whose
    /// `names` column is `column`.
    fn encode_fixed_array(field_type: NestableType, column: ArrayRef) -> Result<Vec<u8>, String> {
        let value = Arc::new(StructArray::from(vec![(
            Arc::new(Field::new("names", column.data_type().clone(), true)),
            column,
        )])) as ArrayRef;
        let type_info = TypeInfo {
            package_name: Cow::Borrowed("test_msgs"),
            message_name: Cow::Borrowed("ArrMsg"),
            messages: fixed_array_message(field_type),
        };
        cdr_encoding::to_vec::<_, LittleEndian>(&TypedValue {
            value: &value,
            type_info: &type_info,
        })
        .map_err(|e| e.to_string())
    }

    fn list_of(field: DataType, values: ArrayRef) -> ArrayRef {
        let len = values.len();
        Arc::new(ListArray::new(
            Arc::new(Field::new("item", field, true)),
            OffsetBuffer::from_lengths([len]),
            values,
            None,
        ))
    }

    fn fixed_size_list_of(field: DataType, size: i32, values: ArrayRef) -> ArrayRef {
        Arc::new(FixedSizeListArray::new(
            Arc::new(Field::new("item", field, true)),
            size,
            values,
            None,
        ))
    }

    /// #4149: `FixedSizeList<T, N>` (e.g. `pa.list_(pa.float64(), 9)` for a
    /// covariance) serializes into `T[N]` exactly like `List<T>`.
    #[test]
    fn fixed_size_list_matches_list() {
        let f64_type = NestableType::BasicType(BasicType::F64);
        let values = || Arc::new(Float64Array::from(vec![1.0, 2.0, 3.0])) as ArrayRef;
        let expected = encode_fixed_array(f64_type.clone(), list_of(DataType::Float64, values()))
            .expect("List<Float64> into float64[3]");
        let actual = encode_fixed_array(
            f64_type.clone(),
            fixed_size_list_of(DataType::Float64, 3, values()),
        )
        .expect("FixedSizeList<Float64, 3> into float64[3]");
        assert_eq!(actual, expected);

        let u8_type = NestableType::BasicType(BasicType::U8);
        let bytes = || Arc::new(UInt8Array::from(vec![7, 8, 9])) as ArrayRef;
        assert_eq!(
            encode_fixed_array(
                u8_type.clone(),
                fixed_size_list_of(DataType::UInt8, 3, bytes())
            )
            .expect("FixedSizeList<UInt8, 3> into uint8[3]"),
            encode_fixed_array(u8_type, list_of(DataType::UInt8, bytes())).unwrap(),
        );
    }

    /// The `T[N]` length check also applies to `FixedSizeList`.
    #[test]
    fn fixed_size_list_length_is_checked() {
        let f64_type = NestableType::BasicType(BasicType::F64);
        let two = Arc::new(Float64Array::from(vec![1.0, 2.0])) as ArrayRef;
        assert!(
            encode_fixed_array(
                f64_type.clone(),
                fixed_size_list_of(DataType::Float64, 2, two)
            )
            .is_err()
        );
        // A wider `N` is rejected as well.
        let four = Arc::new(Float64Array::from(vec![0.0; 4])) as ArrayRef;
        assert!(
            encode_fixed_array(f64_type, fixed_size_list_of(DataType::Float64, 4, four)).is_err()
        );
    }

    /// #4149: `FixedSizeBinary(N)` (e.g. `pa.binary(16)` for a UUID) serializes
    /// into `uint8[N]` exactly like `List<UInt8>` and `Binary`.
    #[test]
    fn fixed_size_binary_matches_list() {
        let u8_type = NestableType::BasicType(BasicType::U8);
        let fsb = FixedSizeBinaryArray::try_from_iter(vec![vec![1u8, 2, 3]].into_iter()).unwrap();
        let actual = encode_fixed_array(u8_type.clone(), Arc::new(fsb))
            .expect("FixedSizeBinary(3) into uint8[3]");
        let expected = encode_fixed_array(
            u8_type,
            list_of(DataType::UInt8, Arc::new(UInt8Array::from(vec![1, 2, 3]))),
        )
        .unwrap();
        assert_eq!(actual, expected);
    }

    /// `FixedSizeBinary` keeps the `Binary` rules: byte element types only, and
    /// the length must match `N`.
    #[test]
    fn fixed_size_binary_is_checked() {
        let fsb = |row: Vec<u8>| {
            Arc::new(FixedSizeBinaryArray::try_from_iter(vec![row].into_iter()).unwrap())
                as ArrayRef
        };
        assert!(
            encode_fixed_array(NestableType::BasicType(BasicType::U8), fsb(vec![1, 2])).is_err()
        );
        assert!(
            encode_fixed_array(NestableType::BasicType(BasicType::I32), fsb(vec![1, 2, 3]))
                .is_err()
        );
    }
}
