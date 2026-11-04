use dora_ros2_bridge_msg_gen::types::Message;
use std::{borrow::Cow, cell::RefCell, collections::HashMap, sync::Arc};

pub use serialize::TypedValue;

const CDR_LE_ENCAPSULATION: [u8; 4] = [0, 1, 0, 0];

pub fn serialize_cdr<T: serde::Serialize>(value: &T) -> eyre::Result<Vec<u8>> {
    let body = cdr_encoding::to_vec::<_, byteorder::LittleEndian>(value)
        .map_err(|error| eyre::eyre!("failed to serialize ROS2 CDR: {error}"))?;
    let mut payload = Vec::with_capacity(CDR_LE_ENCAPSULATION.len() + body.len());
    payload.extend_from_slice(&CDR_LE_ENCAPSULATION);
    payload.extend_from_slice(&body);
    Ok(payload)
}

pub fn deserialize_cdr<T: serde::de::DeserializeOwned>(bytes: &[u8]) -> eyre::Result<T> {
    let body = cdr_little_endian_body(bytes)?;
    let (value, consumed) = cdr_encoding::from_bytes::<T, byteorder::LittleEndian>(body)
        .map_err(|error| eyre::eyre!("failed to deserialize ROS2 CDR: {error}"))?;
    if consumed != body.len() {
        eyre::bail!(
            "ROS2 CDR payload has {} trailing bytes",
            body.len() - consumed
        );
    }
    Ok(value)
}

pub fn serialize_raw_cdr(value: &TypedValue<'_>) -> eyre::Result<Vec<u8>> {
    serialize_cdr(value)
}

pub fn deserialize_raw_cdr(
    bytes: &[u8],
    type_info: TypeInfo<'static>,
    max_payload_size: usize,
) -> eyre::Result<arrow::array::ArrayData> {
    if bytes.len() > max_payload_size {
        return Err(eyre::eyre!(
            "ROS2 CDR payload is {} bytes, limit is {max_payload_size}",
            bytes.len()
        ));
    }
    let body = cdr_little_endian_body(bytes)?;
    let seed = deserialize::StructDeserializer::new(Cow::Owned(type_info));
    let mut deserializer = cdr_encoding::CdrDeserializer::<byteorder::LittleEndian>::new(body);
    let value = serde::de::DeserializeSeed::deserialize(seed, &mut deserializer)
        .map_err(|error| eyre::eyre!("failed to deserialize ROS2 CDR: {error}"))?;
    let consumed = deserializer.bytes_consumed();
    if consumed != body.len() {
        return Err(eyre::eyre!(
            "ROS2 CDR payload has {} trailing bytes",
            body.len() - consumed
        ));
    }
    Ok(value)
}

fn cdr_little_endian_body(bytes: &[u8]) -> eyre::Result<&[u8]> {
    let header = bytes
        .get(..CDR_LE_ENCAPSULATION.len())
        .ok_or_else(|| eyre::eyre!("ROS2 CDR payload is missing its encapsulation header"))?;
    if header != CDR_LE_ENCAPSULATION {
        eyre::bail!("unsupported ROS2 CDR encapsulation header {header:02x?}");
    }
    Ok(&bytes[CDR_LE_ENCAPSULATION.len()..])
}

pub mod deserialize;
pub mod serialize;

#[derive(Debug, Clone)]
pub struct TypeInfo<'a> {
    pub package_name: Cow<'a, str>,
    pub message_name: Cow<'a, str>,
    pub messages: Arc<HashMap<String, HashMap<String, Message>>>,
}

/// Serde requires that struct and field names are known at
/// compile time with a `'static` lifetime, which is not
/// possible in this case. Thus, we need to use dummy names
/// instead.
///
/// The actual names do not really matter because
/// the CDR format of ROS2 does not encode struct or field
/// names.
const DUMMY_STRUCT_NAME: &str = "struct";

// ---------------------------------------------------------------------------
// BridgeMessage: dynamic-type wrapper for ros2_client Service/Action traits
// ---------------------------------------------------------------------------
//
// ros2_client requires `T: Message` (= Serialize + DeserializeOwned) for
// service request/response and action goal/result/feedback types.
// The bridge uses dynamic types, so we use thread-local TypeInfo storage
// to pass type information into Serialize/Deserialize impls.
//
// SAFETY: The bridge event loop is single-threaded per service/action, so
// thread-local access is race-free.

thread_local! {
    static SERIALIZE_TYPE_INFO: RefCell<Option<TypeInfo<'static>>> = const { RefCell::new(None) };
    static DESERIALIZE_TYPE_INFO: RefCell<Option<TypeInfo<'static>>> = const { RefCell::new(None) };
}

/// Set the TypeInfo used by the next `BridgeMessage::serialize` call.
///
/// Uses `try_borrow_mut` to avoid panicking if called during unwind while
/// the RefCell is already borrowed (e.g. from a TypeInfoGuard drop during panic).
pub fn set_serialize_type_info(info: Option<TypeInfo<'static>>) {
    SERIALIZE_TYPE_INFO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            *slot = info;
        }
    });
}

/// Set the TypeInfo used by the next `BridgeMessage::deserialize` call.
///
/// Uses `try_borrow_mut` to avoid panicking if called during unwind while
/// the RefCell is already borrowed.
pub fn set_deserialize_type_info(info: Option<TypeInfo<'static>>) {
    DESERIALIZE_TYPE_INFO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            *slot = info;
        }
    });
}

/// RAII guard that clears its thread-local TypeInfo slot on drop.
///
/// Each guard tracks which slot (serialize or deserialize) it owns and
/// only clears that slot. This is safe even if multiple guards coexist.
pub struct TypeInfoGuard {
    kind: GuardKind,
}

enum GuardKind {
    Serialize,
    Deserialize,
}

impl TypeInfoGuard {
    /// Set serialize TypeInfo and return a guard that clears the serialize slot on drop.
    pub fn serialize(info: TypeInfo<'static>) -> Self {
        set_serialize_type_info(Some(info));
        Self {
            kind: GuardKind::Serialize,
        }
    }

    /// Set deserialize TypeInfo and return a guard that clears the deserialize slot on drop.
    pub fn deserialize(info: TypeInfo<'static>) -> Self {
        set_deserialize_type_info(Some(info));
        Self {
            kind: GuardKind::Deserialize,
        }
    }
}

impl Drop for TypeInfoGuard {
    fn drop(&mut self) {
        match self.kind {
            GuardKind::Serialize => set_serialize_type_info(None),
            GuardKind::Deserialize => set_deserialize_type_info(None),
        }
    }
}

/// A dynamically-typed message wrapper that implements `Serialize + DeserializeOwned + Clone`.
///
/// This allows using dynamic Arrow-based messages with `ros2_client`'s
/// `Service` and `ActionTypes` traits, which require `Message` bounds.
#[derive(Clone, Debug)]
pub struct BridgeMessage(pub Option<arrow::array::ArrayData>);

impl serde::Serialize for BridgeMessage {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let type_info = SERIALIZE_TYPE_INFO
            .with(|cell| cell.borrow().clone())
            .ok_or_else(|| {
                serde::ser::Error::custom("BridgeMessage serialize: SERIALIZE_TYPE_INFO not set")
            })?;
        let data = self
            .0
            .as_ref()
            .ok_or_else(|| serde::ser::Error::custom("BridgeMessage: no data to serialize"))?;
        let array_ref = arrow::array::make_array(data.clone());
        let typed_value = TypedValue {
            value: &array_ref,
            type_info: &type_info,
        };
        typed_value.serialize(serializer)
    }
}

impl<'de> serde::Deserialize<'de> for BridgeMessage {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let type_info = DESERIALIZE_TYPE_INFO
            .with(|cell| cell.borrow().clone())
            .ok_or_else(|| {
                serde::de::Error::custom("BridgeMessage deserialize: DESERIALIZE_TYPE_INFO not set")
            })?;
        let seed = deserialize::StructDeserializer::new(Cow::Owned(type_info));
        serde::de::DeserializeSeed::deserialize(seed, deserializer)
            .map(|data| BridgeMessage(Some(data)))
    }
}

impl ros2_client::Message for BridgeMessage {}

/// Service type descriptor for dynamic bridge services.
#[derive(Debug)]
pub struct BridgeServiceType {
    pub request_type_name: String,
    pub response_type_name: String,
}

impl ros2_client::Service for BridgeServiceType {
    type Request = BridgeMessage;
    type Response = BridgeMessage;

    fn request_type_name(&self) -> &str {
        &self.request_type_name
    }

    fn response_type_name(&self) -> &str {
        &self.response_type_name
    }
}

#[cfg(test)]
mod raw_cdr_tests {
    use arrow::array::{ArrayRef, Int32Array, StructArray};
    use arrow::datatypes::{DataType, Field, Fields};
    use dora_ros2_bridge_msg_gen::types::{Member, Message, primitives::BasicType};
    use std::{borrow::Cow, collections::HashMap, sync::Arc};

    use super::{
        CDR_LE_ENCAPSULATION, TypeInfo, TypedValue, deserialize_raw_cdr, serialize_raw_cdr,
    };

    fn fixture() -> (ArrayRef, TypeInfo<'static>) {
        let message = Message {
            package: "test_pkg".into(),
            name: "Primitive".into(),
            members: vec![Member {
                name: "value".into(),
                r#type: BasicType::I32.into(),
                default: None,
            }],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("Primitive".into(), message);
        let mut messages = HashMap::new();
        messages.insert("test_pkg".into(), package);
        let fields = Fields::from(vec![Field::new("value", DataType::Int32, true)]);
        let value: ArrayRef = Arc::new(StructArray::new(
            fields,
            vec![Arc::new(Int32Array::from(vec![42]))],
            None,
        ));
        (
            value,
            TypeInfo {
                package_name: Cow::Borrowed("test_pkg"),
                message_name: Cow::Borrowed("Primitive"),
                messages: Arc::new(messages),
            },
        )
    }

    #[test]
    fn raw_cdr_uses_rustdds_little_endian_layout() {
        let (value, type_info) = fixture();
        let bytes = serialize_raw_cdr(&TypedValue {
            value: &value,
            type_info: &type_info,
        })
        .unwrap();
        assert_eq!(
            bytes,
            [CDR_LE_ENCAPSULATION.as_slice(), &42_i32.to_le_bytes()].concat()
        );
        let decoded = deserialize_raw_cdr(&bytes, type_info, 1024).unwrap();
        assert_eq!(decoded, value.to_data());
    }

    #[test]
    fn raw_cdr_rejects_payload_limit_truncation_and_trailing_bytes() {
        let (_, type_info) = fixture();
        assert!(deserialize_raw_cdr(&[0; 5], type_info.clone(), 4).is_err());
        assert!(deserialize_raw_cdr(&[0; 3], type_info.clone(), 4).is_err());
        assert!(deserialize_raw_cdr(&[0, 0, 0, 0, 42, 0, 0, 0], type_info.clone(), 8).is_err());
        assert!(deserialize_raw_cdr(&[0, 1, 0, 0, 42, 0, 0, 0, 1], type_info, 9).is_err());
    }

    /// `{ uint8 tag; <element>[] data; uint32 after }`, for checking the byte
    /// sequence wire layout including the alignment padding around it.
    fn byte_sequence_fixture(element: BasicType) -> TypeInfo<'static> {
        use dora_ros2_bridge_msg_gen::types::{
            MemberType, primitives::NestableType, sequences::Sequence,
        };
        let message = Message {
            package: "test_pkg".into(),
            name: "Bytes".into(),
            members: vec![
                Member {
                    name: "tag".into(),
                    r#type: BasicType::U8.into(),
                    default: None,
                },
                Member {
                    name: "data".into(),
                    r#type: MemberType::Sequence(Sequence {
                        value_type: NestableType::BasicType(element),
                    }),
                    default: None,
                },
                Member {
                    name: "after".into(),
                    r#type: BasicType::U32.into(),
                    default: None,
                },
            ],
            constants: vec![],
        };
        let mut package = HashMap::new();
        package.insert("Bytes".into(), message);
        let mut messages = HashMap::new();
        messages.insert("test_pkg".into(), package);
        TypeInfo {
            package_name: Cow::Borrowed("test_pkg"),
            message_name: Cow::Borrowed("Bytes"),
            messages: Arc::new(messages),
        }
    }

    fn byte_sequence_value(data: ArrayRef) -> ArrayRef {
        use arrow::array::{UInt8Array, UInt32Array};
        let fields = Fields::from(vec![
            Field::new("tag", DataType::UInt8, true),
            Field::new("data", data.data_type().clone(), true),
            Field::new("after", DataType::UInt32, true),
        ]);
        Arc::new(StructArray::new(
            fields,
            vec![
                Arc::new(UInt8Array::from(vec![7])),
                data,
                Arc::new(UInt32Array::from(vec![0xAABBCCDD])),
            ],
            None,
        ))
    }

    /// What rustdds / rmw put on the wire for the fixture with `data = [1..=5]`:
    /// the odd length makes the `uint32` after it need 3 bytes of padding.
    fn byte_sequence_wire() -> Vec<u8> {
        [
            CDR_LE_ENCAPSULATION.as_slice(),
            &[7, 0, 0, 0],
            &5u32.to_le_bytes(),
            &[1, 2, 3, 4, 5, 0, 0, 0],
            &0xAABBCCDDu32.to_le_bytes(),
        ]
        .concat()
    }

    #[test]
    fn byte_sequences_keep_the_cdr_wire_layout() {
        use arrow::array::{BinaryArray, ListBuilder, UInt8Builder};
        for element in [BasicType::U8, BasicType::Byte, BasicType::Char] {
            let type_info = byte_sequence_fixture(element);
            let mut list = ListBuilder::new(UInt8Builder::new());
            list.values().append_slice(&[1, 2, 3, 4, 5]);
            list.append(true);
            let value = byte_sequence_value(Arc::new(list.finish()));
            let bytes = serialize_raw_cdr(&TypedValue {
                value: &value,
                type_info: &type_info,
            })
            .unwrap();
            assert_eq!(bytes, byte_sequence_wire(), "{element:?} serialize");
            let decoded = deserialize_raw_cdr(&bytes, type_info.clone(), 1024).unwrap();
            assert_eq!(decoded, value.to_data(), "{element:?} round trip");

            // A `binary` column (one row) encodes the same bytes.
            let binary =
                byte_sequence_value(Arc::new(BinaryArray::from(vec![&[1u8, 2, 3, 4, 5][..]])));
            let bytes = serialize_raw_cdr(&TypedValue {
                value: &binary,
                type_info: &type_info,
            })
            .unwrap();
            assert_eq!(bytes, byte_sequence_wire(), "{element:?} binary serialize");
        }
    }

    #[test]
    fn empty_byte_sequence_round_trips() {
        use arrow::array::{ListBuilder, UInt8Builder};
        let type_info = byte_sequence_fixture(BasicType::U8);
        let mut list = ListBuilder::new(UInt8Builder::new());
        list.append(true);
        let value = byte_sequence_value(Arc::new(list.finish()));
        let bytes = serialize_raw_cdr(&TypedValue {
            value: &value,
            type_info: &type_info,
        })
        .unwrap();
        assert_eq!(
            bytes,
            [
                CDR_LE_ENCAPSULATION.as_slice(),
                &[7, 0, 0, 0],
                &0u32.to_le_bytes(),
                &0xAABBCCDDu32.to_le_bytes()
            ]
            .concat()
        );
        assert_eq!(
            deserialize_raw_cdr(&bytes, type_info, 1024).unwrap(),
            value.to_data()
        );
    }

    /// Byte sequences go through `serialize_bytes` / `deserialize_bytes`; a
    /// non-CDR format (the Python bridge round-trips through `serde_assert`)
    /// must still get the same array back.
    #[test]
    fn byte_sequences_round_trip_through_a_token_format() {
        use arrow::array::{ListBuilder, UInt8Builder};
        use serde::{Serialize, de::DeserializeSeed};
        let type_info = byte_sequence_fixture(BasicType::U8);
        let mut list = ListBuilder::new(UInt8Builder::new());
        list.values().append_slice(&[1, 2, 3, 4, 5]);
        list.append(true);
        let value = byte_sequence_value(Arc::new(list.finish()));
        let tokens = TypedValue {
            value: &value,
            type_info: &type_info,
        }
        .serialize(&serde_assert::Serializer::builder().build())
        .unwrap();
        let mut deserializer = serde_assert::Deserializer::builder(tokens).build();
        let decoded = crate::deserialize::StructDeserializer::new(Cow::Owned(type_info))
            .deserialize(&mut deserializer)
            .unwrap();
        assert_eq!(decoded, value.to_data());
    }

    #[test]
    fn byte_sequence_length_beyond_the_payload_is_rejected() {
        let type_info = byte_sequence_fixture(BasicType::U8);
        let mut bytes = byte_sequence_wire();
        bytes[8..12].copy_from_slice(&1_000u32.to_le_bytes());
        assert!(deserialize_raw_cdr(&bytes, type_info, 1 << 20).is_err());
    }
}

/// Action type descriptor for dynamic bridge actions.
#[derive(Debug)]
pub struct BridgeActionType {
    pub goal_type_name: String,
    pub result_type_name: String,
    pub feedback_type_name: String,
}

impl ros2_client::ActionTypes for BridgeActionType {
    type GoalType = BridgeMessage;
    type ResultType = BridgeMessage;
    type FeedbackType = BridgeMessage;

    fn goal_type_name(&self) -> &str {
        &self.goal_type_name
    }

    fn result_type_name(&self) -> &str {
        &self.result_type_name
    }

    fn feedback_type_name(&self) -> &str {
        &self.feedback_type_name
    }
}
