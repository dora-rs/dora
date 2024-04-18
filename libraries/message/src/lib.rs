//! Enable serialisation and deserialisation of capnproto messages
//!

#![allow(clippy::missing_safety_doc)]

use arrow_data::ArrayData;
use arrow_schema::DataType;
use eyre::Context;
use serde::{Deserialize, Serialize};
pub use uhlc;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Metadata {
    metadata_version: u16,
    timestamp: uhlc::Timestamp,
    pub type_info: ArrowTypeInfo,
    pub parameters: MetadataParameters,
}

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

impl ArrowTypeInfo {
    pub const fn empty() -> Self {
        Self {
            data_type: DataType::Null,
            len: 0,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: Vec::new(),
            child_data: Vec::new(),
        }
    }

    pub fn byte_array(data_len: usize) -> Self {
        Self {
            data_type: DataType::UInt8,
            len: data_len,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![BufferOffset {
                offset: 0,
                len: data_len,
            }],
            child_data: Vec::new(),
        }
    }

    pub unsafe fn from_array(
        array: &ArrayData,
        region_start: *const u8,
        region_len: usize,
    ) -> eyre::Result<Self> {
        Ok(Self {
            data_type: array.data_type().clone(),
            len: array.len(),
            null_count: array.null_count(),
            validity: array.nulls().map(|b| b.validity().to_owned()),
            offset: array.offset(),
            buffer_offsets: array
                .buffers()
                .iter()
                .map(|b| {
                    let ptr = b.as_ptr();
                    if ptr as usize <= region_start as usize {
                        eyre::bail!("ptr {ptr:p} starts before region {region_start:p}");
                    }
                    if ptr as usize >= region_start as usize + region_len {
                        eyre::bail!("ptr {ptr:p} starts after region {region_start:p}");
                    }
                    if ptr as usize + b.len() > region_start as usize + region_len {
                        eyre::bail!("ptr {ptr:p} ends after region {region_start:p}");
                    }
                    let offset = usize::try_from(unsafe { ptr.offset_from(region_start) })
                        .context("offset_from is negative")?;

                    Result::<_, eyre::Report>::Ok(BufferOffset {
                        offset,
                        len: b.len(),
                    })
                })
                .collect::<Result<_, _>>()?,
            child_data: array
                .child_data()
                .iter()
                .map(|c| unsafe { Self::from_array(c, region_start, region_len) })
                .collect::<Result<_, _>>()?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BufferOffset {
    pub offset: usize,
    pub len: usize,
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
}
