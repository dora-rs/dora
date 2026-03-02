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
        if let Some(Parameter::String(otel)) = self.parameters.get("open_telemetry_context") {
            otel.to_string()
        } else {
            "".to_string()
        }
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
    }

    #[test]
    fn well_known_keys_are_distinct() {
        let keys = [REQUEST_ID, GOAL_ID, GOAL_STATUS];
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
}
