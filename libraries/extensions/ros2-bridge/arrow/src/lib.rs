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
