use crate::{BridgeError, BridgeResult};
use arrow::array::{Array, AsArray, RecordBatch, UInt8Array, UInt32Array};
use arrow::datatypes::{DataType, Field, Schema, UInt8Type, UInt32Type};
use mavlink::common;
use num_traits::FromPrimitive;
use std::sync::Arc;

/// Bidirectional conversion between a MAVLink 2 message struct and a 1-row
/// Apache Arrow `RecordBatch`. Implemented per message type; for batched
/// telemetry, callers stream individual batches and concatenate downstream.
pub trait MavlinkArrow: Sized {
    /// Arrow schema describing this message's fields.
    fn schema() -> Schema;

    /// Encode `self` into a 1-row Arrow `RecordBatch`.
    fn to_record_batch(&self) -> BridgeResult<RecordBatch>;

    /// Decode the first row of a `RecordBatch` back into the message struct.
    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self>;
}

fn read_u32(batch: &RecordBatch, name: &str) -> BridgeResult<u32> {
    let col = batch
        .column_by_name(name)
        .ok_or_else(|| BridgeError::Config(format!("missing column: {name}")))?;
    Ok(col.as_primitive::<UInt32Type>().value(0))
}

fn read_u8(batch: &RecordBatch, name: &str) -> BridgeResult<u8> {
    let col = batch
        .column_by_name(name)
        .ok_or_else(|| BridgeError::Config(format!("missing column: {name}")))?;
    Ok(col.as_primitive::<UInt8Type>().value(0))
}

fn decode_enum<E: FromPrimitive>(v: u8, name: &str) -> BridgeResult<E> {
    E::from_u8(v).ok_or_else(|| BridgeError::Mavlink(format!("invalid {name} discriminant: {v}")))
}

impl MavlinkArrow for common::HEARTBEAT_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("custom_mode", DataType::UInt32, false),
            Field::new("mavtype", DataType::UInt8, false),
            Field::new("autopilot", DataType::UInt8, false),
            Field::new("base_mode", DataType::UInt8, false),
            Field::new("system_status", DataType::UInt8, false),
            Field::new("mavlink_version", DataType::UInt8, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        let schema = Arc::new(Self::schema());
        let columns: Vec<Arc<dyn Array>> = vec![
            Arc::new(UInt32Array::from(vec![self.custom_mode])),
            Arc::new(UInt8Array::from(vec![self.mavtype as u8])),
            Arc::new(UInt8Array::from(vec![self.autopilot as u8])),
            Arc::new(UInt8Array::from(vec![self.base_mode.bits()])),
            Arc::new(UInt8Array::from(vec![self.system_status as u8])),
            Arc::new(UInt8Array::from(vec![self.mavlink_version])),
        ];
        Ok(RecordBatch::try_new(schema, columns)?)
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            custom_mode: read_u32(batch, "custom_mode")?,
            mavtype: decode_enum(read_u8(batch, "mavtype")?, "mavtype")?,
            autopilot: decode_enum(read_u8(batch, "autopilot")?, "autopilot")?,
            base_mode: common::MavModeFlag::from_bits_truncate(read_u8(batch, "base_mode")?),
            system_status: decode_enum(read_u8(batch, "system_status")?, "system_status")?,
            mavlink_version: read_u8(batch, "mavlink_version")?,
        })
    }
}
