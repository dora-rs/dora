use std::convert::TryFrom;

use anyhow::Result;

use super::RclParameterType;
use crate::error::RclRustError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterType {
    NotSet,
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
}

impl TryFrom<u8> for ParameterType {
    type Error = anyhow::Error;

    fn try_from(v: u8) -> Result<Self> {
        Ok(match v {
            RclParameterType::PARAMETER_NOT_SET => Self::NotSet,
            RclParameterType::PARAMETER_BOOL => Self::Bool,
            RclParameterType::PARAMETER_INTEGER => Self::Integer,
            RclParameterType::PARAMETER_DOUBLE => Self::Double,
            RclParameterType::PARAMETER_STRING => Self::String,
            RclParameterType::PARAMETER_BYTE_ARRAY => Self::ByteArray,
            RclParameterType::PARAMETER_BOOL_ARRAY => Self::BoolArray,
            RclParameterType::PARAMETER_INTEGER_ARRAY => Self::IntegerArray,
            RclParameterType::PARAMETER_DOUBLE_ARRAY => Self::DoubleArray,
            RclParameterType::PARAMETER_STRING_ARRAY => Self::StringArray,
            _ => {
                return Err(RclRustError::OutOfRange(format!(
                    "{} cannot be converted into RclParameterType",
                    v
                ))
                .into())
            }
        })
    }
}
