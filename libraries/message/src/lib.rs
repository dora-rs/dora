//! Enable serialisation and deserialisation of capnproto messages
//!

use arrow_schema::{DataType, Field, Schema};
use serde::{Deserialize, Serialize};
pub use uhlc;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Metadata {
    metadata_version: u16,
    timestamp: uhlc::Timestamp,
    pub schema: Schema,
    pub parameters: MetadataParameters,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct MetadataParameters {
    pub watermark: u64,
    pub deadline: u64,
    pub open_telemetry_context: String,
}

impl MetadataParameters {
    pub fn into_owned(self) -> MetadataParameters {
        MetadataParameters {
            open_telemetry_context: self.open_telemetry_context,
            ..self
        }
    }
}

impl Metadata {
    pub fn new(timestamp: uhlc::Timestamp, data_type: DataType) -> Self {
        Self::from_parameters(timestamp, data_type, Default::default())
    }

    pub fn from_parameters(
        timestamp: uhlc::Timestamp,
        data_type: DataType,
        parameters: MetadataParameters,
    ) -> Self {
        let schema = Schema::new(vec![Field::new_list(
            "value",
            Field::new("item", data_type, false),
            true,
        )]);

        Self {
            metadata_version: 0,
            timestamp,
            parameters,
            schema,
        }
    }

    pub fn timestamp(&self) -> uhlc::Timestamp {
        self.timestamp
    }
}
