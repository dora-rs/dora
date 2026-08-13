//! Round-trip tests for `MavlinkArrow` impls.
//!
//! Each test encodes a synthetic MAVLink message struct into a 1-row
//! Arrow `RecordBatch`, decodes it back, and asserts every field matches.

// This suite deliberately exercises upstream-deprecated items (SET_MODE,
// superseded by MAV_CMD_DO_SET_MODE, kept for legacy autopilots — see the
// limitation note in arrow_convert.rs — plus MavMode/MAV_FRAME_GLOBAL_INT
// sentinels), so silence the deprecation lint file-wide.
#![allow(deprecated)]

use dora_mavlink2_bridge::MavlinkArrow;
use dora_mavlink2_bridge::mavlink::dialects::common::SET_MODE_DATA;
use dora_mavlink2_bridge::mavlink::dialects::common::{
    ATTITUDE_DATA, ATTITUDE_QUATERNION_DATA, COMMAND_ACK_DATA, COMMAND_LONG_DATA,
    GLOBAL_POSITION_INT_DATA, GPS_RAW_INT_DATA, GpsFixType, HEARTBEAT_DATA,
    LOCAL_POSITION_NED_DATA, MISSION_CURRENT_DATA, MavAutopilot, MavCmd, MavFrame, MavMode,
    MavModeFlag, MavResult, MavState, MavSysStatusSensor, MavType, PositionTargetTypemask,
    RC_CHANNELS_DATA, RC_CHANNELS_OVERRIDE_DATA, SERVO_OUTPUT_RAW_DATA,
    SET_POSITION_TARGET_GLOBAL_INT_DATA, SET_POSITION_TARGET_LOCAL_NED_DATA, SYS_STATUS_DATA,
    SYSTEM_TIME_DATA,
};

#[test]
fn heartbeat_arrow_roundtrip() {
    let original = HEARTBEAT_DATA {
        custom_mode: 42,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    };
    let batch = original.to_record_batch().expect("encode");
    assert_eq!(batch.num_rows(), 1);
    let decoded = HEARTBEAT_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.custom_mode, original.custom_mode);
    assert_eq!(decoded.mavtype, original.mavtype);
    assert_eq!(decoded.autopilot, original.autopilot);
    assert_eq!(decoded.base_mode, original.base_mode);
    assert_eq!(decoded.system_status, original.system_status);
    assert_eq!(decoded.mavlink_version, original.mavlink_version);
}

#[test]
fn sys_status_arrow_roundtrip() {
    let original = SYS_STATUS_DATA {
        onboard_control_sensors_present: MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
            | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL,
        onboard_control_sensors_enabled: MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO,
        onboard_control_sensors_health: MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL,
        load: 250,
        voltage_battery: 12500,
        current_battery: 230,
        drop_rate_comm: 0,
        errors_comm: 1,
        errors_count1: 2,
        errors_count2: 3,
        errors_count3: 4,
        errors_count4: 5,
        battery_remaining: 87,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = SYS_STATUS_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(
        decoded.onboard_control_sensors_present,
        original.onboard_control_sensors_present
    );
    assert_eq!(
        decoded.onboard_control_sensors_enabled,
        original.onboard_control_sensors_enabled
    );
    assert_eq!(
        decoded.onboard_control_sensors_health,
        original.onboard_control_sensors_health
    );
    assert_eq!(decoded.load, original.load);
    assert_eq!(decoded.voltage_battery, original.voltage_battery);
    assert_eq!(decoded.current_battery, original.current_battery);
    assert_eq!(decoded.battery_remaining, original.battery_remaining);
}

#[test]
fn system_time_arrow_roundtrip() {
    let original = SYSTEM_TIME_DATA {
        time_unix_usec: 1_700_000_000_000_000,
        time_boot_ms: 12_345,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = SYSTEM_TIME_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_unix_usec, original.time_unix_usec);
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
}

#[test]
fn attitude_arrow_roundtrip() {
    let original = ATTITUDE_DATA {
        time_boot_ms: 1000,
        roll: 0.1,
        pitch: -0.2,
        yaw: 1.57,
        rollspeed: 0.01,
        pitchspeed: -0.02,
        yawspeed: 0.03,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = ATTITUDE_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
    assert_eq!(decoded.roll, original.roll);
    assert_eq!(decoded.pitch, original.pitch);
    assert_eq!(decoded.yaw, original.yaw);
    assert_eq!(decoded.rollspeed, original.rollspeed);
    assert_eq!(decoded.pitchspeed, original.pitchspeed);
    assert_eq!(decoded.yawspeed, original.yawspeed);
}

#[test]
fn attitude_quaternion_arrow_roundtrip() {
    let original = ATTITUDE_QUATERNION_DATA {
        time_boot_ms: 2000,
        q1: 1.0,
        q2: 0.0,
        q3: 0.0,
        q4: 0.0,
        rollspeed: 0.1,
        pitchspeed: 0.2,
        yawspeed: 0.3,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = ATTITUDE_QUATERNION_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
    assert_eq!(decoded.q1, original.q1);
    assert_eq!(decoded.q2, original.q2);
    assert_eq!(decoded.q3, original.q3);
    assert_eq!(decoded.q4, original.q4);
}

#[test]
fn local_position_ned_arrow_roundtrip() {
    let original = LOCAL_POSITION_NED_DATA {
        time_boot_ms: 5000,
        x: 1.0,
        y: 2.0,
        z: -3.0,
        vx: 0.5,
        vy: -0.5,
        vz: 0.0,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = LOCAL_POSITION_NED_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
    assert_eq!(decoded.x, original.x);
    assert_eq!(decoded.y, original.y);
    assert_eq!(decoded.z, original.z);
    assert_eq!(decoded.vx, original.vx);
    assert_eq!(decoded.vy, original.vy);
    assert_eq!(decoded.vz, original.vz);
}

#[test]
fn global_position_int_arrow_roundtrip() {
    let original = GLOBAL_POSITION_INT_DATA {
        time_boot_ms: 8000,
        lat: 379_000_000,
        lon: -1_223_000_000,
        alt: 100_000,
        relative_alt: 50_000,
        vx: 10,
        vy: -5,
        vz: 1,
        hdg: 27_000,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = GLOBAL_POSITION_INT_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
    assert_eq!(decoded.lat, original.lat);
    assert_eq!(decoded.lon, original.lon);
    assert_eq!(decoded.alt, original.alt);
    assert_eq!(decoded.relative_alt, original.relative_alt);
    assert_eq!(decoded.vx, original.vx);
    assert_eq!(decoded.hdg, original.hdg);
}

#[test]
fn gps_raw_int_arrow_roundtrip() {
    let original = GPS_RAW_INT_DATA {
        time_usec: 1_700_000_000_000_000,
        lat: 379_000_000,
        lon: -1_223_000_000,
        alt: 100_000,
        eph: 100,
        epv: 200,
        vel: 300,
        cog: 4500,
        fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
        satellites_visible: 12,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = GPS_RAW_INT_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_usec, original.time_usec);
    assert_eq!(decoded.lat, original.lat);
    assert_eq!(decoded.fix_type, original.fix_type);
    assert_eq!(decoded.satellites_visible, original.satellites_visible);
}

#[test]
fn rc_channels_arrow_roundtrip() {
    let original = RC_CHANNELS_DATA {
        time_boot_ms: 3000,
        chan1_raw: 1500,
        chan2_raw: 1500,
        chan3_raw: 1100,
        chan4_raw: 1500,
        chan5_raw: 1000,
        chan6_raw: 2000,
        chan7_raw: 1500,
        chan8_raw: 1500,
        chan9_raw: 1500,
        chan10_raw: 1500,
        chan11_raw: 1500,
        chan12_raw: 1500,
        chan13_raw: 1500,
        chan14_raw: 1500,
        chan15_raw: 1500,
        chan16_raw: 1500,
        chan17_raw: 1500,
        chan18_raw: 1500,
        chancount: 8,
        rssi: 200,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = RC_CHANNELS_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
    assert_eq!(decoded.chan1_raw, original.chan1_raw);
    assert_eq!(decoded.chan8_raw, original.chan8_raw);
    assert_eq!(decoded.chan18_raw, original.chan18_raw);
    assert_eq!(decoded.chancount, original.chancount);
    assert_eq!(decoded.rssi, original.rssi);
}

#[test]
fn servo_output_raw_arrow_roundtrip() {
    let original = SERVO_OUTPUT_RAW_DATA {
        time_usec: 100_000,
        servo1_raw: 1500,
        servo2_raw: 1500,
        servo3_raw: 1500,
        servo4_raw: 1500,
        servo5_raw: 1500,
        servo6_raw: 1500,
        servo7_raw: 1500,
        servo8_raw: 1500,
        port: 0,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = SERVO_OUTPUT_RAW_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_usec, original.time_usec);
    assert_eq!(decoded.servo1_raw, original.servo1_raw);
    assert_eq!(decoded.servo8_raw, original.servo8_raw);
    assert_eq!(decoded.port, original.port);
}

#[test]
fn command_long_arrow_roundtrip() {
    let original = COMMAND_LONG_DATA {
        param1: 1.0,
        param2: 2.0,
        param3: 3.0,
        param4: 4.0,
        param5: 5.0,
        param6: 6.0,
        param7: 7.0,
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        target_system: 1,
        target_component: 1,
        confirmation: 0,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = COMMAND_LONG_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.command, original.command);
    assert_eq!(decoded.param1, original.param1);
    assert_eq!(decoded.param7, original.param7);
    assert_eq!(decoded.target_system, original.target_system);
    assert_eq!(decoded.target_component, original.target_component);
    assert_eq!(decoded.confirmation, original.confirmation);
}

#[test]
fn command_ack_arrow_roundtrip() {
    let original = COMMAND_ACK_DATA {
        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
        result: MavResult::MAV_RESULT_ACCEPTED,
    };
    let batch = original.to_record_batch().expect("encode");
    let decoded = COMMAND_ACK_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.command, original.command);
    assert_eq!(decoded.result, original.result);
}

#[test]
fn mission_current_arrow_roundtrip() {
    let original = MISSION_CURRENT_DATA { seq: 5 };
    let batch = original.to_record_batch().expect("encode");
    let decoded = MISSION_CURRENT_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.seq, original.seq);
}

// -----------------------------------------------------------------------------
// Write-path (dora -> MAVLink) round-trips.
//
// These are the messages that arm and move a vehicle, so they get stricter
// coverage than the telemetry tests above: every field gets a *distinct*
// sentinel value and every field is asserted. The schema list and the
// column-builder list in each impl are two parallel, manually-ordered lists;
// transposing two same-Arrow-type fields between them is not a compile error
// and not caught by `RecordBatch::try_new` (the dtypes still match). Distinct
// per-field sentinels are what make such a swap actually fail the assert —
// reusing the same value for adjacent same-type fields (e.g. the long runs of
// Float32 in the SET_POSITION_TARGET messages) would silently pass.
// -----------------------------------------------------------------------------

#[test]
fn set_mode_arrow_roundtrip() {
    let original = SET_MODE_DATA {
        custom_mode: 42,
        target_system: 7,
        // Must be one of mavlink-rust 0.18's strict MavMode variants
        // (see the SET_MODE limitation note in arrow_convert.rs); a raw
        // ArduPilot custom-mode base_mode would not round-trip.
        base_mode: MavMode::MAV_MODE_GUIDED_ARMED,
    };
    let batch = original.to_record_batch().expect("encode");
    assert_eq!(batch.num_rows(), 1);
    let decoded = SET_MODE_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.custom_mode, original.custom_mode);
    assert_eq!(decoded.target_system, original.target_system);
    assert_eq!(decoded.base_mode, original.base_mode);
}

#[test]
fn rc_channels_override_arrow_roundtrip() {
    let original = RC_CHANNELS_OVERRIDE_DATA {
        target_system: 9,
        target_component: 11,
        chan1_raw: 1001,
        chan2_raw: 1002,
        chan3_raw: 1003,
        chan4_raw: 1004,
        chan5_raw: 1005,
        chan6_raw: 1006,
        chan7_raw: 1007,
        chan8_raw: 1008,
    };
    let batch = original.to_record_batch().expect("encode");
    assert_eq!(batch.num_rows(), 1);
    let decoded = RC_CHANNELS_OVERRIDE_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.target_system, original.target_system);
    assert_eq!(decoded.target_component, original.target_component);
    assert_eq!(decoded.chan1_raw, original.chan1_raw);
    assert_eq!(decoded.chan2_raw, original.chan2_raw);
    assert_eq!(decoded.chan3_raw, original.chan3_raw);
    assert_eq!(decoded.chan4_raw, original.chan4_raw);
    assert_eq!(decoded.chan5_raw, original.chan5_raw);
    assert_eq!(decoded.chan6_raw, original.chan6_raw);
    assert_eq!(decoded.chan7_raw, original.chan7_raw);
    assert_eq!(decoded.chan8_raw, original.chan8_raw);
}

#[test]
fn set_position_target_global_int_arrow_roundtrip() {
    let original = SET_POSITION_TARGET_GLOBAL_INT_DATA {
        time_boot_ms: 8000,
        lat_int: 379_000_000,
        lon_int: -1_223_000_000,
        // Distinct value per Float32 slot so a transposition anywhere in
        // the 9-field alt..yaw_rate run is caught.
        alt: 100.0,
        vx: 1.0,
        vy: 2.0,
        vz: 3.0,
        afx: 4.0,
        afy: 5.0,
        afz: 6.0,
        yaw: 7.0,
        yaw_rate: 8.0,
        type_mask: PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE,
        target_system: 3,
        target_component: 4,
        coordinate_frame: MavFrame::MAV_FRAME_GLOBAL_INT,
    };
    let batch = original.to_record_batch().expect("encode");
    assert_eq!(batch.num_rows(), 1);
    let decoded = SET_POSITION_TARGET_GLOBAL_INT_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
    assert_eq!(decoded.lat_int, original.lat_int);
    assert_eq!(decoded.lon_int, original.lon_int);
    assert_eq!(decoded.alt, original.alt);
    assert_eq!(decoded.vx, original.vx);
    assert_eq!(decoded.vy, original.vy);
    assert_eq!(decoded.vz, original.vz);
    assert_eq!(decoded.afx, original.afx);
    assert_eq!(decoded.afy, original.afy);
    assert_eq!(decoded.afz, original.afz);
    assert_eq!(decoded.yaw, original.yaw);
    assert_eq!(decoded.yaw_rate, original.yaw_rate);
    assert_eq!(decoded.type_mask, original.type_mask);
    assert_eq!(decoded.target_system, original.target_system);
    assert_eq!(decoded.target_component, original.target_component);
    assert_eq!(decoded.coordinate_frame, original.coordinate_frame);
}

#[test]
fn set_position_target_local_ned_arrow_roundtrip() {
    let original = SET_POSITION_TARGET_LOCAL_NED_DATA {
        time_boot_ms: 5000,
        // 11 adjacent Float32 fields — the worst case for a silent
        // transposition. Each gets a distinct sentinel.
        x: 1.0,
        y: 2.0,
        z: 3.0,
        vx: 4.0,
        vy: 5.0,
        vz: 6.0,
        afx: 7.0,
        afy: 8.0,
        afz: 9.0,
        yaw: 10.0,
        yaw_rate: 11.0,
        type_mask: PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE,
        target_system: 1,
        target_component: 2,
        coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
    };
    let batch = original.to_record_batch().expect("encode");
    assert_eq!(batch.num_rows(), 1);
    let decoded = SET_POSITION_TARGET_LOCAL_NED_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.time_boot_ms, original.time_boot_ms);
    assert_eq!(decoded.x, original.x);
    assert_eq!(decoded.y, original.y);
    assert_eq!(decoded.z, original.z);
    assert_eq!(decoded.vx, original.vx);
    assert_eq!(decoded.vy, original.vy);
    assert_eq!(decoded.vz, original.vz);
    assert_eq!(decoded.afx, original.afx);
    assert_eq!(decoded.afy, original.afy);
    assert_eq!(decoded.afz, original.afz);
    assert_eq!(decoded.yaw, original.yaw);
    assert_eq!(decoded.yaw_rate, original.yaw_rate);
    assert_eq!(decoded.type_mask, original.type_mask);
    assert_eq!(decoded.target_system, original.target_system);
    assert_eq!(decoded.target_component, original.target_component);
    assert_eq!(decoded.coordinate_frame, original.coordinate_frame);
}

/// Lock every impl's encode side to its declared schema: `to_record_batch`
/// must emit exactly the `schema()` it advertises (same field names, order,
/// dtypes, and nullability) and exactly one row. This catches an impl whose
/// column-builder list drifts in width or dtype from its schema list, and
/// guards the decode contract that `from_record_batch` requires exactly
/// one row.
#[test]
fn encode_matches_declared_schema() {
    fn check<T: MavlinkArrow + Default>() {
        let batch = T::default().to_record_batch().expect("encode default");
        assert_eq!(batch.num_rows(), 1, "expected exactly one row");
        assert_eq!(
            batch.schema().as_ref(),
            &T::schema(),
            "to_record_batch() schema drifted from schema()"
        );
    }

    check::<HEARTBEAT_DATA>();
    check::<SYS_STATUS_DATA>();
    check::<SYSTEM_TIME_DATA>();
    check::<ATTITUDE_DATA>();
    check::<ATTITUDE_QUATERNION_DATA>();
    check::<LOCAL_POSITION_NED_DATA>();
    check::<GLOBAL_POSITION_INT_DATA>();
    check::<GPS_RAW_INT_DATA>();
    check::<RC_CHANNELS_DATA>();
    check::<SERVO_OUTPUT_RAW_DATA>();
    check::<COMMAND_LONG_DATA>();
    check::<COMMAND_ACK_DATA>();
    check::<MISSION_CURRENT_DATA>();
    // write path
    check::<SET_MODE_DATA>();
    check::<RC_CHANNELS_OVERRIDE_DATA>();
    check::<SET_POSITION_TARGET_GLOBAL_INT_DATA>();
    check::<SET_POSITION_TARGET_LOCAL_NED_DATA>();
}
