use std::convert::TryFrom;
use std::{collections::BTreeMap, fmt};

use arrow_schema::DataType;
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

    /// Returns a raw metadata parameter by key.
    ///
    /// This method only accesses user-provided metadata parameters.
    /// It does **not** return fields such as the timestamp or type information.
    /// For example, `get("timestamp")` is not equivalent to [`Metadata::timestamp`].
    pub fn get(&self, key: &str) -> Option<&Parameter> {
        self.parameters.get(key)
    }

    pub fn open_telemetry_context(&self) -> String {
        self.get("open_telemetry_context")
            .and_then(|p| String::try_from(p).ok())
            .unwrap_or_default()
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

#[derive(Debug, Clone)]
pub struct TryFromParameterError {
    pub expected: &'static str,
    pub found: &'static str,
}

impl fmt::Display for TryFromParameterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "expected {}, found {}", self.expected, self.found)
    }
}

impl std::error::Error for TryFromParameterError {}
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
}

impl Parameter {
    pub(crate) fn variant_name(&self) -> &'static str {
        match self {
            Parameter::Bool(_) => "bool",
            Parameter::Integer(_) => "integer",
            Parameter::String(_) => "string",
            Parameter::ListInt(_) => "list<i64>",
            Parameter::Float(_) => "float",
            Parameter::ListFloat(_) => "list<f64>",
            Parameter::ListString(_) => "list<string>",
        }
    }
}

impl TryFrom<&Parameter> for bool {
    type Error = TryFromParameterError;

    fn try_from(value: &Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::Bool(value) => Ok(*value),
            other => Err(TryFromParameterError {
                expected: "bool",
                found: other.variant_name(),
            }),
        }
    }
}

impl TryFrom<&Parameter> for String {
    type Error = TryFromParameterError;

    fn try_from(value: &Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::String(val) => Ok(val.clone()),
            other => Err(TryFromParameterError {
                expected: "string",
                found: other.variant_name(),
            }),
        }
    }
}

impl<'a> TryFrom<&'a Parameter> for &'a str {
    type Error = TryFromParameterError;

    fn try_from(value: &'a Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::String(v) => Ok(v.as_str()),
            other => Err(TryFromParameterError {
                expected: "&str",
                found: other.variant_name(),
            }),
        }
    }
}

impl TryFrom<&Parameter> for i64 {
    type Error = TryFromParameterError;

    fn try_from(value: &Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::Integer(v) => Ok(*v),
            other => Err(TryFromParameterError {
                expected: "i64",
                found: other.variant_name(),
            }),
        }
    }
}

impl TryFrom<&Parameter> for f64 {
    type Error = TryFromParameterError;

    fn try_from(value: &Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::Float(val) => Ok(*val),
            other => Err(TryFromParameterError {
                expected: "f64",
                found: other.variant_name(),
            }),
        }
    }
}

impl TryFrom<&Parameter> for Vec<i64> {
    type Error = TryFromParameterError;

    fn try_from(value: &Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::ListInt(v) => Ok(v.clone()),
            other => Err(TryFromParameterError {
                expected: "list<i64>",
                found: other.variant_name(),
            }),
        }
    }
}

impl<'a> TryFrom<&'a Parameter> for &'a [i64] {
    type Error = TryFromParameterError;

    fn try_from(value: &'a Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::ListInt(v) => Ok(v.as_slice()),
            other => Err(TryFromParameterError {
                expected: "&[i64]",
                found: other.variant_name(),
            }),
        }
    }
}

impl TryFrom<&Parameter> for Vec<f64> {
    type Error = TryFromParameterError;

    fn try_from(value: &Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::ListFloat(val) => Ok(val.clone()),
            other => Err(TryFromParameterError {
                expected: "list<f64>",
                found: other.variant_name(),
            }),
        }
    }
}

impl<'a> TryFrom<&'a Parameter> for &'a [f64] {
    type Error = TryFromParameterError;

    fn try_from(value: &'a Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::ListFloat(v) => Ok(v.as_slice()),
            other => Err(TryFromParameterError {
                expected: "&[f64]",
                found: other.variant_name(),
            }),
        }
    }
}

impl TryFrom<&Parameter> for Vec<String> {
    type Error = TryFromParameterError;

    fn try_from(value: &Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::ListString(v) => Ok(v.clone()),
            other => Err(TryFromParameterError {
                expected: "list<string>",
                found: other.variant_name(),
            }),
        }
    }
}

impl<'a> TryFrom<&'a Parameter> for &'a [String] {
    type Error = TryFromParameterError;

    fn try_from(value: &'a Parameter) -> Result<Self, Self::Error> {
        match value {
            Parameter::ListString(v) => Ok(v.as_slice()),
            other => Err(TryFromParameterError {
                expected: "&[String]",
                found: other.variant_name(),
            }),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BufferOffset {
    pub offset: usize,
    pub len: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::convert::TryFrom;

    #[test]
    fn try_from_bool_ok() {
        let p = Parameter::Bool(true);
        let v = bool::try_from(&p).unwrap();
        assert!(v);
    }

    #[test]
    fn try_from_bool_type_mismatch() {
        let p = Parameter::Integer(1);
        let err = bool::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("expected bool"));
    }

    #[test]
    fn try_from_i64_ok() {
        let p = Parameter::Integer(42);
        let v = i64::try_from(&p).unwrap();
        assert_eq!(v, 42);
    }

    #[test]
    fn try_from_i64_type_mismatch() {
        let p = Parameter::Float(1.0);
        let err = i64::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("expected i64"));
    }

    #[test]
    fn try_from_f64_ok() {
        let p = Parameter::Float(1.0);
        let val = f64::try_from(&p).unwrap();
        assert_eq!(val, 1.0);
    }

    #[test]
    fn try_from_f64_type_mismatch() {
        let p = Parameter::Integer(50);
        let err = f64::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("expected f64"));
    }

    #[test]
    fn try_from_string_ok() {
        let p = Parameter::String(String::from("welcome"));
        let val = String::try_from(&p).unwrap();
        assert_eq!(val, String::from("welcome"));
    }

    #[test]
    fn try_from_string_type_mismatch() {
        let p = Parameter::Integer(5);
        let err = String::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("expected string"));
    }

    #[test]
    fn try_from_str_ok() {
        let p = Parameter::String("welcome".into());
        let v: &str = <&str>::try_from(&p).unwrap();
        assert_eq!(v, "welcome");
    }

    #[test]
    fn try_from_str_type_mismatch() {
        let p = Parameter::Integer(5);
        let err = <&str>::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("&str"));
    }

    #[test]
    fn try_from_vec_i64_ok() {
        let p = Parameter::ListInt(vec![1, 2, 3]);
        let v = Vec::<i64>::try_from(&p).unwrap();
        assert_eq!(v, vec![1, 2, 3]);
    }

    #[test]
    fn try_from_vec_i64_type_mismatch() {
        let p = Parameter::ListFloat(vec![1.0]);
        let err = Vec::<i64>::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("list<i64>"));
    }

    #[test]
    fn try_from_vec_f64_ok() {
        let p = Parameter::ListFloat(vec![1.0, 2.0]);
        let v = Vec::<f64>::try_from(&p).unwrap();
        assert_eq!(v, vec![1.0, 2.0]);
    }

    #[test]
    fn try_from_vec_f64_type_mismatch() {
        let p = Parameter::ListInt(vec![1, 2]);
        let err = Vec::<f64>::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("list<f64>"));
    }

    #[test]
    fn try_from_vec_string_ok() {
        let p = Parameter::ListString(vec!["a".into(), "b".into()]);
        let v = Vec::<String>::try_from(&p).unwrap();
        assert_eq!(v, vec!["a", "b"]);
    }

    #[test]
    fn try_from_vec_string_type_mismatch() {
        let p = Parameter::String("x".into());
        let err = Vec::<String>::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("list<string>"));
    }

    #[test]
    fn try_from_slice_i64_ok() {
        let p = Parameter::ListInt(vec![1, 2, 3]);
        let v: &[i64] = <&[i64]>::try_from(&p).unwrap();
        assert_eq!(v, &[1, 2, 3]);
    }

    #[test]
    fn try_from_slice_i64_type_mismatch() {
        let p = Parameter::ListFloat(vec![1.0]);
        let err = <&[i64]>::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("&[i64]"));
    }

    #[test]
    fn try_from_slice_f64_ok() {
        let p = Parameter::ListFloat(vec![1.0, 2.0]);
        let v: &[f64] = <&[f64]>::try_from(&p).unwrap();
        assert_eq!(v, &[1.0, 2.0]);
    }

    #[test]
    fn try_from_slice_f64_type_mismatch() {
        let p = Parameter::ListInt(vec![1, 2]);
        let err = <&[f64]>::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("&[f64]"));
    }
    #[test]
    fn try_from_slice_string_ok() {
        let p = Parameter::ListString(vec!["a".into(), "b".into()]);
        let v: &[String] = <&[String]>::try_from(&p).unwrap();
        assert_eq!(v, &["a", "b"]);
    }

    #[test]
    fn try_from_slice_string_type_mismatch() {
        let p = Parameter::String("x".into());
        let err = <&[String]>::try_from(&p).unwrap_err();
        assert!(err.to_string().contains("&[String]"));
    }
}
