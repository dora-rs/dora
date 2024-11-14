use std::collections::BTreeMap;

use arrow_schema::DataType;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
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

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
pub enum Parameter {
    Bool(bool),
    Integer(i64),
    String(String),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BufferOffset {
    pub offset: usize,
    pub len: usize,
}
