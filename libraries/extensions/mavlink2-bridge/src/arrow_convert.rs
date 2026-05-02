//! `MavlinkArrow` trait and per-message impls for the common dialect.
//!
//! Arrow-side conventions used by every impl:
//!
//! * One row per call. `to_record_batch` returns a 1-row `RecordBatch`;
//!   `from_record_batch` reads row 0.
//! * All MAVLink enums (regardless of wire width) are stored as `UInt32`
//!   to match the Rust `repr(u32)` of the generated enum types.
//! * Bitflag fields are stored at their bitflag's natural width
//!   (`MavModeFlag` is `u8`, `MavSysStatusSensor` is `u32`).
//! * Primitive fields keep their wire width unchanged.
//! * Fixed-size arrays (e.g. `STATUSTEXT::text[50]`,
//!   `BATTERY_STATUS::voltages[10]`) are deferred to a follow-up.

use crate::{BridgeError, BridgeResult};
use arrow::array::{
    ArrayRef, AsArray, Float32Array, Int8Array, Int16Array, Int32Array, RecordBatch, UInt8Array,
    UInt16Array, UInt32Array, UInt64Array,
};
use arrow::datatypes::{
    DataType, Field, Float32Type, Int8Type, Int16Type, Int32Type, Schema, UInt8Type, UInt16Type,
    UInt32Type, UInt64Type,
};
use mavlink::common;
use num_traits::FromPrimitive;
use std::sync::Arc;

/// Bidirectional conversion between a MAVLink 2 message struct and a 1-row
/// Apache Arrow `RecordBatch`. Each impl is symmetric: the schema returned
/// by [`schema`] is what [`to_record_batch`] emits and what
/// [`from_record_batch`] expects.
///
/// [`schema`]: MavlinkArrow::schema
/// [`to_record_batch`]: MavlinkArrow::to_record_batch
/// [`from_record_batch`]: MavlinkArrow::from_record_batch
pub trait MavlinkArrow: Sized {
    fn schema() -> Schema;
    fn to_record_batch(&self) -> BridgeResult<RecordBatch>;
    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self>;
}

// -----------------------------------------------------------------------------
// helpers
// -----------------------------------------------------------------------------

fn missing(name: &str) -> BridgeError {
    BridgeError::Config(format!("missing column: {name}"))
}

fn build(fields: Vec<Field>, columns: Vec<ArrayRef>) -> BridgeResult<RecordBatch> {
    Ok(RecordBatch::try_new(
        Arc::new(Schema::new(fields)),
        columns,
    )?)
}

// readers
fn read_u8(batch: &RecordBatch, name: &str) -> BridgeResult<u8> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<UInt8Type>()
        .value(0))
}
fn read_u16(batch: &RecordBatch, name: &str) -> BridgeResult<u16> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<UInt16Type>()
        .value(0))
}
fn read_u32(batch: &RecordBatch, name: &str) -> BridgeResult<u32> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<UInt32Type>()
        .value(0))
}
fn read_u64(batch: &RecordBatch, name: &str) -> BridgeResult<u64> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<UInt64Type>()
        .value(0))
}
fn read_i8(batch: &RecordBatch, name: &str) -> BridgeResult<i8> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<Int8Type>()
        .value(0))
}
fn read_i16(batch: &RecordBatch, name: &str) -> BridgeResult<i16> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<Int16Type>()
        .value(0))
}
fn read_i32(batch: &RecordBatch, name: &str) -> BridgeResult<i32> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<Int32Type>()
        .value(0))
}
fn read_f32(batch: &RecordBatch, name: &str) -> BridgeResult<f32> {
    Ok(batch
        .column_by_name(name)
        .ok_or_else(|| missing(name))?
        .as_primitive::<Float32Type>()
        .value(0))
}
// builders (single-row Arrow arrays from a single value)
fn arr_u8(v: u8) -> ArrayRef {
    Arc::new(UInt8Array::from(vec![v]))
}
fn arr_u16(v: u16) -> ArrayRef {
    Arc::new(UInt16Array::from(vec![v]))
}
fn arr_u32(v: u32) -> ArrayRef {
    Arc::new(UInt32Array::from(vec![v]))
}
fn arr_u64(v: u64) -> ArrayRef {
    Arc::new(UInt64Array::from(vec![v]))
}
fn arr_i8(v: i8) -> ArrayRef {
    Arc::new(Int8Array::from(vec![v]))
}
fn arr_i16(v: i16) -> ArrayRef {
    Arc::new(Int16Array::from(vec![v]))
}
fn arr_i32(v: i32) -> ArrayRef {
    Arc::new(Int32Array::from(vec![v]))
}
fn arr_f32(v: f32) -> ArrayRef {
    Arc::new(Float32Array::from(vec![v]))
}
/// Decode a MAVLink enum value (always stored as `u32` in our Arrow
/// schema) into the strongly-typed enum. All common-dialect enums are
/// `repr(u32)` and derive `num_traits::FromPrimitive`.
fn decode_enum<E: FromPrimitive>(v: u32, name: &str) -> BridgeResult<E> {
    E::from_u32(v).ok_or_else(|| BridgeError::Mavlink(format!("invalid {name} discriminant: {v}")))
}

// -----------------------------------------------------------------------------
// HEARTBEAT
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::HEARTBEAT_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("custom_mode", DataType::UInt32, false),
            Field::new("mavtype", DataType::UInt32, false),
            Field::new("autopilot", DataType::UInt32, false),
            Field::new("base_mode", DataType::UInt8, false),
            Field::new("system_status", DataType::UInt32, false),
            Field::new("mavlink_version", DataType::UInt8, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.custom_mode),
                arr_u32(self.mavtype as u32),
                arr_u32(self.autopilot as u32),
                arr_u8(self.base_mode.bits()),
                arr_u32(self.system_status as u32),
                arr_u8(self.mavlink_version),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            custom_mode: read_u32(batch, "custom_mode")?,
            mavtype: decode_enum(read_u32(batch, "mavtype")?, "mavtype")?,
            autopilot: decode_enum(read_u32(batch, "autopilot")?, "autopilot")?,
            base_mode: common::MavModeFlag::from_bits_truncate(read_u8(batch, "base_mode")?),
            system_status: decode_enum(read_u32(batch, "system_status")?, "system_status")?,
            mavlink_version: read_u8(batch, "mavlink_version")?,
        })
    }
}

// -----------------------------------------------------------------------------
// SYS_STATUS
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::SYS_STATUS_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("onboard_control_sensors_present", DataType::UInt32, false),
            Field::new("onboard_control_sensors_enabled", DataType::UInt32, false),
            Field::new("onboard_control_sensors_health", DataType::UInt32, false),
            Field::new("load", DataType::UInt16, false),
            Field::new("voltage_battery", DataType::UInt16, false),
            Field::new("current_battery", DataType::Int16, false),
            Field::new("drop_rate_comm", DataType::UInt16, false),
            Field::new("errors_comm", DataType::UInt16, false),
            Field::new("errors_count1", DataType::UInt16, false),
            Field::new("errors_count2", DataType::UInt16, false),
            Field::new("errors_count3", DataType::UInt16, false),
            Field::new("errors_count4", DataType::UInt16, false),
            Field::new("battery_remaining", DataType::Int8, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.onboard_control_sensors_present.bits()),
                arr_u32(self.onboard_control_sensors_enabled.bits()),
                arr_u32(self.onboard_control_sensors_health.bits()),
                arr_u16(self.load),
                arr_u16(self.voltage_battery),
                arr_i16(self.current_battery),
                arr_u16(self.drop_rate_comm),
                arr_u16(self.errors_comm),
                arr_u16(self.errors_count1),
                arr_u16(self.errors_count2),
                arr_u16(self.errors_count3),
                arr_u16(self.errors_count4),
                arr_i8(self.battery_remaining),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            onboard_control_sensors_present: common::MavSysStatusSensor::from_bits_truncate(
                read_u32(batch, "onboard_control_sensors_present")?,
            ),
            onboard_control_sensors_enabled: common::MavSysStatusSensor::from_bits_truncate(
                read_u32(batch, "onboard_control_sensors_enabled")?,
            ),
            onboard_control_sensors_health: common::MavSysStatusSensor::from_bits_truncate(
                read_u32(batch, "onboard_control_sensors_health")?,
            ),
            load: read_u16(batch, "load")?,
            voltage_battery: read_u16(batch, "voltage_battery")?,
            current_battery: read_i16(batch, "current_battery")?,
            drop_rate_comm: read_u16(batch, "drop_rate_comm")?,
            errors_comm: read_u16(batch, "errors_comm")?,
            errors_count1: read_u16(batch, "errors_count1")?,
            errors_count2: read_u16(batch, "errors_count2")?,
            errors_count3: read_u16(batch, "errors_count3")?,
            errors_count4: read_u16(batch, "errors_count4")?,
            battery_remaining: read_i8(batch, "battery_remaining")?,
        })
    }
}

// -----------------------------------------------------------------------------
// SYSTEM_TIME
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::SYSTEM_TIME_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("time_unix_usec", DataType::UInt64, false),
            Field::new("time_boot_ms", DataType::UInt32, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![arr_u64(self.time_unix_usec), arr_u32(self.time_boot_ms)],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_unix_usec: read_u64(batch, "time_unix_usec")?,
            time_boot_ms: read_u32(batch, "time_boot_ms")?,
        })
    }
}

// -----------------------------------------------------------------------------
// ATTITUDE
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::ATTITUDE_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("time_boot_ms", DataType::UInt32, false),
            Field::new("roll", DataType::Float32, false),
            Field::new("pitch", DataType::Float32, false),
            Field::new("yaw", DataType::Float32, false),
            Field::new("rollspeed", DataType::Float32, false),
            Field::new("pitchspeed", DataType::Float32, false),
            Field::new("yawspeed", DataType::Float32, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.time_boot_ms),
                arr_f32(self.roll),
                arr_f32(self.pitch),
                arr_f32(self.yaw),
                arr_f32(self.rollspeed),
                arr_f32(self.pitchspeed),
                arr_f32(self.yawspeed),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_boot_ms: read_u32(batch, "time_boot_ms")?,
            roll: read_f32(batch, "roll")?,
            pitch: read_f32(batch, "pitch")?,
            yaw: read_f32(batch, "yaw")?,
            rollspeed: read_f32(batch, "rollspeed")?,
            pitchspeed: read_f32(batch, "pitchspeed")?,
            yawspeed: read_f32(batch, "yawspeed")?,
        })
    }
}

// -----------------------------------------------------------------------------
// ATTITUDE_QUATERNION
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::ATTITUDE_QUATERNION_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("time_boot_ms", DataType::UInt32, false),
            Field::new("q1", DataType::Float32, false),
            Field::new("q2", DataType::Float32, false),
            Field::new("q3", DataType::Float32, false),
            Field::new("q4", DataType::Float32, false),
            Field::new("rollspeed", DataType::Float32, false),
            Field::new("pitchspeed", DataType::Float32, false),
            Field::new("yawspeed", DataType::Float32, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.time_boot_ms),
                arr_f32(self.q1),
                arr_f32(self.q2),
                arr_f32(self.q3),
                arr_f32(self.q4),
                arr_f32(self.rollspeed),
                arr_f32(self.pitchspeed),
                arr_f32(self.yawspeed),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_boot_ms: read_u32(batch, "time_boot_ms")?,
            q1: read_f32(batch, "q1")?,
            q2: read_f32(batch, "q2")?,
            q3: read_f32(batch, "q3")?,
            q4: read_f32(batch, "q4")?,
            rollspeed: read_f32(batch, "rollspeed")?,
            pitchspeed: read_f32(batch, "pitchspeed")?,
            yawspeed: read_f32(batch, "yawspeed")?,
        })
    }
}

// -----------------------------------------------------------------------------
// LOCAL_POSITION_NED
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::LOCAL_POSITION_NED_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("time_boot_ms", DataType::UInt32, false),
            Field::new("x", DataType::Float32, false),
            Field::new("y", DataType::Float32, false),
            Field::new("z", DataType::Float32, false),
            Field::new("vx", DataType::Float32, false),
            Field::new("vy", DataType::Float32, false),
            Field::new("vz", DataType::Float32, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.time_boot_ms),
                arr_f32(self.x),
                arr_f32(self.y),
                arr_f32(self.z),
                arr_f32(self.vx),
                arr_f32(self.vy),
                arr_f32(self.vz),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_boot_ms: read_u32(batch, "time_boot_ms")?,
            x: read_f32(batch, "x")?,
            y: read_f32(batch, "y")?,
            z: read_f32(batch, "z")?,
            vx: read_f32(batch, "vx")?,
            vy: read_f32(batch, "vy")?,
            vz: read_f32(batch, "vz")?,
        })
    }
}

// -----------------------------------------------------------------------------
// GLOBAL_POSITION_INT
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::GLOBAL_POSITION_INT_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("time_boot_ms", DataType::UInt32, false),
            Field::new("lat", DataType::Int32, false),
            Field::new("lon", DataType::Int32, false),
            Field::new("alt", DataType::Int32, false),
            Field::new("relative_alt", DataType::Int32, false),
            Field::new("vx", DataType::Int16, false),
            Field::new("vy", DataType::Int16, false),
            Field::new("vz", DataType::Int16, false),
            Field::new("hdg", DataType::UInt16, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.time_boot_ms),
                arr_i32(self.lat),
                arr_i32(self.lon),
                arr_i32(self.alt),
                arr_i32(self.relative_alt),
                arr_i16(self.vx),
                arr_i16(self.vy),
                arr_i16(self.vz),
                arr_u16(self.hdg),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_boot_ms: read_u32(batch, "time_boot_ms")?,
            lat: read_i32(batch, "lat")?,
            lon: read_i32(batch, "lon")?,
            alt: read_i32(batch, "alt")?,
            relative_alt: read_i32(batch, "relative_alt")?,
            vx: read_i16(batch, "vx")?,
            vy: read_i16(batch, "vy")?,
            vz: read_i16(batch, "vz")?,
            hdg: read_u16(batch, "hdg")?,
        })
    }
}

// -----------------------------------------------------------------------------
// GPS_RAW_INT
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::GPS_RAW_INT_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("time_usec", DataType::UInt64, false),
            Field::new("lat", DataType::Int32, false),
            Field::new("lon", DataType::Int32, false),
            Field::new("alt", DataType::Int32, false),
            Field::new("eph", DataType::UInt16, false),
            Field::new("epv", DataType::UInt16, false),
            Field::new("vel", DataType::UInt16, false),
            Field::new("cog", DataType::UInt16, false),
            Field::new("fix_type", DataType::UInt32, false),
            Field::new("satellites_visible", DataType::UInt8, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u64(self.time_usec),
                arr_i32(self.lat),
                arr_i32(self.lon),
                arr_i32(self.alt),
                arr_u16(self.eph),
                arr_u16(self.epv),
                arr_u16(self.vel),
                arr_u16(self.cog),
                arr_u32(self.fix_type as u32),
                arr_u8(self.satellites_visible),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_usec: read_u64(batch, "time_usec")?,
            lat: read_i32(batch, "lat")?,
            lon: read_i32(batch, "lon")?,
            alt: read_i32(batch, "alt")?,
            eph: read_u16(batch, "eph")?,
            epv: read_u16(batch, "epv")?,
            vel: read_u16(batch, "vel")?,
            cog: read_u16(batch, "cog")?,
            fix_type: decode_enum(read_u32(batch, "fix_type")?, "fix_type")?,
            satellites_visible: read_u8(batch, "satellites_visible")?,
        })
    }
}

// -----------------------------------------------------------------------------
// RC_CHANNELS (18 raw channels)
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::RC_CHANNELS_DATA {
    fn schema() -> Schema {
        let mut fields = vec![Field::new("time_boot_ms", DataType::UInt32, false)];
        for n in 1..=18 {
            fields.push(Field::new(format!("chan{n}_raw"), DataType::UInt16, false));
        }
        fields.push(Field::new("chancount", DataType::UInt8, false));
        fields.push(Field::new("rssi", DataType::UInt8, false));
        Schema::new(fields)
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.time_boot_ms),
                arr_u16(self.chan1_raw),
                arr_u16(self.chan2_raw),
                arr_u16(self.chan3_raw),
                arr_u16(self.chan4_raw),
                arr_u16(self.chan5_raw),
                arr_u16(self.chan6_raw),
                arr_u16(self.chan7_raw),
                arr_u16(self.chan8_raw),
                arr_u16(self.chan9_raw),
                arr_u16(self.chan10_raw),
                arr_u16(self.chan11_raw),
                arr_u16(self.chan12_raw),
                arr_u16(self.chan13_raw),
                arr_u16(self.chan14_raw),
                arr_u16(self.chan15_raw),
                arr_u16(self.chan16_raw),
                arr_u16(self.chan17_raw),
                arr_u16(self.chan18_raw),
                arr_u8(self.chancount),
                arr_u8(self.rssi),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_boot_ms: read_u32(batch, "time_boot_ms")?,
            chan1_raw: read_u16(batch, "chan1_raw")?,
            chan2_raw: read_u16(batch, "chan2_raw")?,
            chan3_raw: read_u16(batch, "chan3_raw")?,
            chan4_raw: read_u16(batch, "chan4_raw")?,
            chan5_raw: read_u16(batch, "chan5_raw")?,
            chan6_raw: read_u16(batch, "chan6_raw")?,
            chan7_raw: read_u16(batch, "chan7_raw")?,
            chan8_raw: read_u16(batch, "chan8_raw")?,
            chan9_raw: read_u16(batch, "chan9_raw")?,
            chan10_raw: read_u16(batch, "chan10_raw")?,
            chan11_raw: read_u16(batch, "chan11_raw")?,
            chan12_raw: read_u16(batch, "chan12_raw")?,
            chan13_raw: read_u16(batch, "chan13_raw")?,
            chan14_raw: read_u16(batch, "chan14_raw")?,
            chan15_raw: read_u16(batch, "chan15_raw")?,
            chan16_raw: read_u16(batch, "chan16_raw")?,
            chan17_raw: read_u16(batch, "chan17_raw")?,
            chan18_raw: read_u16(batch, "chan18_raw")?,
            chancount: read_u8(batch, "chancount")?,
            rssi: read_u8(batch, "rssi")?,
        })
    }
}

// -----------------------------------------------------------------------------
// RC_CHANNELS_OVERRIDE — used by missions that drive vehicles in MANUAL
// mode by emulating an RC transmitter. Required for ArduRover 2.50 which
// does not accept COMMAND_LONG ARM / DO_REPOSITION / NAV_WAYPOINT.
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::RC_CHANNELS_OVERRIDE_DATA {
    fn schema() -> Schema {
        let mut fields = vec![
            Field::new("target_system", DataType::UInt8, false),
            Field::new("target_component", DataType::UInt8, false),
        ];
        for n in 1..=8 {
            fields.push(Field::new(format!("chan{n}_raw"), DataType::UInt16, false));
        }
        Schema::new(fields)
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u8(self.target_system),
                arr_u8(self.target_component),
                arr_u16(self.chan1_raw),
                arr_u16(self.chan2_raw),
                arr_u16(self.chan3_raw),
                arr_u16(self.chan4_raw),
                arr_u16(self.chan5_raw),
                arr_u16(self.chan6_raw),
                arr_u16(self.chan7_raw),
                arr_u16(self.chan8_raw),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            target_system: read_u8(batch, "target_system")?,
            target_component: read_u8(batch, "target_component")?,
            chan1_raw: read_u16(batch, "chan1_raw")?,
            chan2_raw: read_u16(batch, "chan2_raw")?,
            chan3_raw: read_u16(batch, "chan3_raw")?,
            chan4_raw: read_u16(batch, "chan4_raw")?,
            chan5_raw: read_u16(batch, "chan5_raw")?,
            chan6_raw: read_u16(batch, "chan6_raw")?,
            chan7_raw: read_u16(batch, "chan7_raw")?,
            chan8_raw: read_u16(batch, "chan8_raw")?,
        })
    }
}

// -----------------------------------------------------------------------------
// SERVO_OUTPUT_RAW
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::SERVO_OUTPUT_RAW_DATA {
    fn schema() -> Schema {
        let mut fields = vec![Field::new("time_usec", DataType::UInt32, false)];
        for n in 1..=8 {
            fields.push(Field::new(format!("servo{n}_raw"), DataType::UInt16, false));
        }
        fields.push(Field::new("port", DataType::UInt8, false));
        Schema::new(fields)
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.time_usec),
                arr_u16(self.servo1_raw),
                arr_u16(self.servo2_raw),
                arr_u16(self.servo3_raw),
                arr_u16(self.servo4_raw),
                arr_u16(self.servo5_raw),
                arr_u16(self.servo6_raw),
                arr_u16(self.servo7_raw),
                arr_u16(self.servo8_raw),
                arr_u8(self.port),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            time_usec: read_u32(batch, "time_usec")?,
            servo1_raw: read_u16(batch, "servo1_raw")?,
            servo2_raw: read_u16(batch, "servo2_raw")?,
            servo3_raw: read_u16(batch, "servo3_raw")?,
            servo4_raw: read_u16(batch, "servo4_raw")?,
            servo5_raw: read_u16(batch, "servo5_raw")?,
            servo6_raw: read_u16(batch, "servo6_raw")?,
            servo7_raw: read_u16(batch, "servo7_raw")?,
            servo8_raw: read_u16(batch, "servo8_raw")?,
            port: read_u8(batch, "port")?,
        })
    }
}

// -----------------------------------------------------------------------------
// COMMAND_LONG
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::COMMAND_LONG_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("param1", DataType::Float32, false),
            Field::new("param2", DataType::Float32, false),
            Field::new("param3", DataType::Float32, false),
            Field::new("param4", DataType::Float32, false),
            Field::new("param5", DataType::Float32, false),
            Field::new("param6", DataType::Float32, false),
            Field::new("param7", DataType::Float32, false),
            Field::new("command", DataType::UInt32, false),
            Field::new("target_system", DataType::UInt8, false),
            Field::new("target_component", DataType::UInt8, false),
            Field::new("confirmation", DataType::UInt8, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_f32(self.param1),
                arr_f32(self.param2),
                arr_f32(self.param3),
                arr_f32(self.param4),
                arr_f32(self.param5),
                arr_f32(self.param6),
                arr_f32(self.param7),
                arr_u32(self.command as u32),
                arr_u8(self.target_system),
                arr_u8(self.target_component),
                arr_u8(self.confirmation),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            param1: read_f32(batch, "param1")?,
            param2: read_f32(batch, "param2")?,
            param3: read_f32(batch, "param3")?,
            param4: read_f32(batch, "param4")?,
            param5: read_f32(batch, "param5")?,
            param6: read_f32(batch, "param6")?,
            param7: read_f32(batch, "param7")?,
            command: decode_enum(read_u32(batch, "command")?, "command")?,
            target_system: read_u8(batch, "target_system")?,
            target_component: read_u8(batch, "target_component")?,
            confirmation: read_u8(batch, "confirmation")?,
        })
    }
}

// -----------------------------------------------------------------------------
// COMMAND_ACK
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::COMMAND_ACK_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("command", DataType::UInt32, false),
            Field::new("result", DataType::UInt32, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![arr_u32(self.command as u32), arr_u32(self.result as u32)],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            command: decode_enum(read_u32(batch, "command")?, "command")?,
            result: decode_enum(read_u32(batch, "result")?, "result")?,
        })
    }
}

// -----------------------------------------------------------------------------
// SET_MODE — used by mission drivers to request a flight mode on autopilots
// that don't accept MAV_CMD_DO_SET_MODE via COMMAND_LONG (e.g. legacy
// ArduCopter <= 3.5).
//
// Known limitation (mavlink-rust 0.13): the spec defines `base_mode` as a
// uint8 bitfield, but mavlink-rust generates `MavMode` as a strict enum with
// only 11 named variants {0, 64, 66, 80, 88, 92, 192, 194, 208, 216, 220}.
// None has bit 0x01 (CUSTOM_MODE_ENABLED) set, so any ArduPilot custom-mode
// entry (base_mode=0x01 + custom_mode=N for GUIDED/AUTO/RTL/...) cannot be
// encoded or decoded through this path. For ArduPilot custom modes, prefer
// MAV_CMD_DO_SET_MODE via the `command_long_cmd` input on autopilots that
// support it (ArduCopter >= 3.6, recent PX4); on older firmware the bridge
// can't speak custom-mode SET_MODE until mavlink-rust accepts base_mode as
// raw u8 (tracked at https://github.com/mavlink/rust-mavlink).
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::SET_MODE_DATA {
    fn schema() -> Schema {
        Schema::new(vec![
            Field::new("custom_mode", DataType::UInt32, false),
            Field::new("target_system", DataType::UInt8, false),
            Field::new("base_mode", DataType::UInt8, false),
        ])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![
                arr_u32(self.custom_mode),
                arr_u8(self.target_system),
                arr_u8(self.base_mode as u8),
            ],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        let raw_base_mode = read_u8(batch, "base_mode")?;
        // Surface the mavlink-rust 0.13 enum-vs-bitfield mismatch with a
        // pointer to the workaround instead of the bare "invalid base_mode
        // discriminant" that decode_enum would produce, since this hits any
        // user trying to enter an ArduPilot custom mode.
        let base_mode = decode_enum(raw_base_mode as u32, "base_mode").map_err(|_| {
            BridgeError::Mavlink(format!(
                "SET_MODE base_mode={raw_base_mode} (0x{raw_base_mode:02x}) is a valid \
                 uint8 per MAVLink spec but not representable as mavlink-rust 0.13's \
                 strict MavMode enum (variants: 0, 64, 66, 80, 88, 92, 192, 194, 208, \
                 216, 220). For ArduPilot custom-mode entry (base_mode=0x01 + \
                 custom_mode=N), use MAV_CMD_DO_SET_MODE via the `command_long_cmd` \
                 input on autopilots that support it (ArduCopter >= 3.6, recent PX4). \
                 Tracked upstream at https://github.com/mavlink/rust-mavlink."
            ))
        })?;
        Ok(Self {
            custom_mode: read_u32(batch, "custom_mode")?,
            target_system: read_u8(batch, "target_system")?,
            base_mode,
        })
    }
}

// -----------------------------------------------------------------------------
// MISSION_CURRENT
// -----------------------------------------------------------------------------

impl MavlinkArrow for common::MISSION_CURRENT_DATA {
    fn schema() -> Schema {
        Schema::new(vec![Field::new("seq", DataType::UInt16, false)])
    }

    fn to_record_batch(&self) -> BridgeResult<RecordBatch> {
        build(
            Self::schema()
                .fields()
                .iter()
                .map(|f| f.as_ref().clone())
                .collect(),
            vec![arr_u16(self.seq)],
        )
    }

    fn from_record_batch(batch: &RecordBatch) -> BridgeResult<Self> {
        Ok(Self {
            seq: read_u16(batch, "seq")?,
        })
    }
}
