//! Round-trip tests for `MavlinkArrow` impls.
//!
//! Each test encodes a synthetic MAVLink message struct into a 1-row
//! Arrow `RecordBatch`, decodes it back, and asserts every field matches.

use dora_mavlink2_bridge::MavlinkArrow;
use dora_mavlink2_bridge::mavlink::common::{
    HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType,
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

    let batch = original.to_record_batch().expect("encode failed");
    assert_eq!(batch.num_rows(), 1);

    let decoded = HEARTBEAT_DATA::from_record_batch(&batch).expect("decode failed");

    assert_eq!(decoded.custom_mode, original.custom_mode);
    assert_eq!(decoded.mavtype, original.mavtype);
    assert_eq!(decoded.autopilot, original.autopilot);
    assert_eq!(decoded.base_mode, original.base_mode);
    assert_eq!(decoded.system_status, original.system_status);
    assert_eq!(decoded.mavlink_version, original.mavlink_version);
}
