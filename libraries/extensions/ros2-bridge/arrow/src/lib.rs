use adora_ros2_bridge_msg_gen::types::Message;
use std::{borrow::Cow, cell::RefCell, collections::HashMap, sync::Arc};

pub use serialize::TypedValue;

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
pub fn set_serialize_type_info(info: Option<TypeInfo<'static>>) {
    SERIALIZE_TYPE_INFO.with(|cell| *cell.borrow_mut() = info);
}

/// Set the TypeInfo used by the next `BridgeMessage::deserialize` call.
pub fn set_deserialize_type_info(info: Option<TypeInfo<'static>>) {
    DESERIALIZE_TYPE_INFO.with(|cell| *cell.borrow_mut() = info);
}

/// RAII guard that clears both thread-local TypeInfo slots on drop.
///
/// Ensures cleanup even on error returns or panics.
pub struct TypeInfoGuard {
    _private: (),
}

impl TypeInfoGuard {
    /// Set serialize TypeInfo and return a guard that clears both slots on drop.
    pub fn serialize(info: TypeInfo<'static>) -> Self {
        set_serialize_type_info(Some(info));
        Self { _private: () }
    }

    /// Set deserialize TypeInfo and return a guard that clears both slots on drop.
    pub fn deserialize(info: TypeInfo<'static>) -> Self {
        set_deserialize_type_info(Some(info));
        Self { _private: () }
    }
}

impl Drop for TypeInfoGuard {
    fn drop(&mut self) {
        set_serialize_type_info(None);
        set_deserialize_type_info(None);
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
