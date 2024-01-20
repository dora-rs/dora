use dora_ros2_bridge_msg_gen::types::Message;
use std::{borrow::Cow, collections::HashMap, sync::Arc};

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
const DUMMY_FIELD_NAME: &str = "field";
