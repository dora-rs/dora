use std::collections::BTreeMap;

use arrow_schema::DataType;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Additional data that is sent as part of output messages.
///
/// Includes a timestamp, type information, and additional user-provided parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    metadata_version: u16,
    timestamp: uhlc::Timestamp,
    pub type_info: ArrowTypeInfo,
    pub parameters: MetadataParameters,
}

impl Metadata {
    pub fn new(timestamp: uhlc::Timestamp, type_info: ArrowTypeInfo) -> Self {
        Self::from_parameters(timestamp, type_info, Default::default())
    }

    pub fn from_parameters(
        timestamp: uhlc::Timestamp,
        type_info: ArrowTypeInfo,
        parameters: MetadataParameters,
    ) -> Self {
        Self {
            metadata_version: 0,
            timestamp,
            parameters,
            type_info,
        }
    }

    pub fn timestamp(&self) -> uhlc::Timestamp {
        self.timestamp
    }

    pub fn open_telemetry_context(&self) -> String {
        get_string_param(&self.parameters, "open_telemetry_context")
            .unwrap_or("")
            .to_string()
    }
}

/// Additional metadata that can be sent as part of output messages.
pub type MetadataParameters = BTreeMap<String, Parameter>;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArrowTypeInfo {
    pub data_type: DataType,
    pub len: usize,
    pub null_count: usize,
    pub validity: Option<Vec<u8>>,
    pub offset: usize,
    pub buffer_offsets: Vec<BufferOffset>,
    pub child_data: Vec<ArrowTypeInfo>,
    /// Optional field names for struct types (enables schema introspection
    /// without full Arrow IPC framing).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub field_names: Option<Vec<String>>,
    /// Hash of the full Arrow schema for fast type matching (future use).
    /// Currently always `None`. When populated, receivers can compare
    /// this O(1) before doing a full type check.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schema_hash: Option<u64>,
}

/// A metadata parameter that can be sent as part of output messages.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum Parameter {
    Bool(bool),
    Integer(i64),
    String(String),
    ListInt(Vec<i64>),
    Float(f64),
    ListFloat(Vec<f64>),
    ListString(Vec<String>),
    Timestamp(DateTime<Utc>),
}

/// Extract a string parameter from metadata, returning `None` if missing or
/// not a `Parameter::String`.
pub fn get_string_param<'a>(params: &'a MetadataParameters, key: &str) -> Option<&'a str> {
    params.get(key).and_then(|p| match p {
        Parameter::String(s) => Some(s.as_str()),
        _ => None,
    })
}

/// Extract an integer parameter from metadata, returning `None` if missing or
/// not a `Parameter::Integer`.
pub fn get_integer_param(params: &MetadataParameters, key: &str) -> Option<i64> {
    params.get(key).and_then(|p| match p {
        Parameter::Integer(n) => Some(*n),
        _ => None,
    })
}

/// Extract a bool parameter from metadata, returning `None` if missing or
/// not a `Parameter::Bool`.
pub fn get_bool_param(params: &MetadataParameters, key: &str) -> Option<bool> {
    params.get(key).and_then(|p| match p {
        Parameter::Bool(b) => Some(*b),
        _ => None,
    })
}

// ---------------------------------------------------------------------------
// Well-known metadata parameter keys for service and action patterns
// ---------------------------------------------------------------------------

/// Metadata key for correlating a service request with its response.
pub const REQUEST_ID: &str = "request_id";

/// Metadata key for identifying an action goal across feedback/result messages.
pub const GOAL_ID: &str = "goal_id";

/// Metadata key for the completion status of an action goal.
pub const GOAL_STATUS: &str = "goal_status";

/// Goal completed successfully.
pub const GOAL_STATUS_SUCCEEDED: &str = "succeeded";

/// Goal was aborted by the server.
pub const GOAL_STATUS_ABORTED: &str = "aborted";

/// Goal was canceled by the client.
pub const GOAL_STATUS_CANCELED: &str = "canceled";

// ---------------------------------------------------------------------------
// Well-known metadata parameter keys for the streaming pattern
// ---------------------------------------------------------------------------

/// Metadata key identifying the conversation/session.
pub const SESSION_ID: &str = "session_id";

/// Metadata key for the logical segment within a session (e.g. one utterance).
pub const SEGMENT_ID: &str = "segment_id";

/// Metadata key for chunk sequence number within a segment.
pub const SEQ: &str = "seq";

/// Metadata key marking the last chunk of a segment (`true` on final chunk).
pub const FIN: &str = "fin";

/// Metadata key to discard older queued messages on this input (`true` to flush).
pub const FLUSH: &str = "flush";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BufferOffset {
    pub offset: usize,
    pub len: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn well_known_keys_have_stable_values() {
        // These string values are part of the cross-language protocol
        // (Python nodes use the same literal strings). Do not change.
        assert_eq!(REQUEST_ID, "request_id");
        assert_eq!(GOAL_ID, "goal_id");
        assert_eq!(GOAL_STATUS, "goal_status");
        assert_eq!(GOAL_STATUS_SUCCEEDED, "succeeded");
        assert_eq!(GOAL_STATUS_ABORTED, "aborted");
        assert_eq!(GOAL_STATUS_CANCELED, "canceled");
        assert_eq!(SESSION_ID, "session_id");
        assert_eq!(SEGMENT_ID, "segment_id");
        assert_eq!(SEQ, "seq");
        assert_eq!(FIN, "fin");
        assert_eq!(FLUSH, "flush");
    }

    #[test]
    fn well_known_keys_are_distinct() {
        let keys = [
            REQUEST_ID,
            GOAL_ID,
            GOAL_STATUS,
            SESSION_ID,
            SEGMENT_ID,
            SEQ,
            FIN,
            FLUSH,
        ];
        for (i, a) in keys.iter().enumerate() {
            for b in &keys[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn goal_status_values_are_distinct() {
        let vals = [
            GOAL_STATUS_SUCCEEDED,
            GOAL_STATUS_ABORTED,
            GOAL_STATUS_CANCELED,
        ];
        for (i, a) in vals.iter().enumerate() {
            for b in &vals[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn get_string_param_extracts_string() {
        let mut params = MetadataParameters::default();
        params.insert("key".to_string(), Parameter::String("value".to_string()));
        assert_eq!(get_string_param(&params, "key"), Some("value"));
        assert_eq!(get_string_param(&params, "missing"), None);
    }

    #[test]
    fn get_string_param_returns_none_for_non_string() {
        let mut params = MetadataParameters::default();
        params.insert("num".to_string(), Parameter::Integer(42));
        assert_eq!(get_string_param(&params, "num"), None);
    }

    #[test]
    fn get_integer_param_extracts_integer() {
        let mut params = MetadataParameters::default();
        params.insert("key".to_string(), Parameter::Integer(42));
        assert_eq!(get_integer_param(&params, "key"), Some(42));
        assert_eq!(get_integer_param(&params, "missing"), None);
    }

    #[test]
    fn get_integer_param_returns_none_for_non_integer() {
        let mut params = MetadataParameters::default();
        params.insert("s".to_string(), Parameter::String("hello".to_string()));
        assert_eq!(get_integer_param(&params, "s"), None);
    }

    #[test]
    fn get_bool_param_extracts_bool() {
        let mut params = MetadataParameters::default();
        params.insert("key".to_string(), Parameter::Bool(true));
        assert_eq!(get_bool_param(&params, "key"), Some(true));
        assert_eq!(get_bool_param(&params, "missing"), None);
    }

    #[test]
    fn get_bool_param_returns_none_for_non_bool() {
        let mut params = MetadataParameters::default();
        params.insert("n".to_string(), Parameter::Integer(1));
        assert_eq!(get_bool_param(&params, "n"), None);
    }
}
