//! Verifies the crate builds with the workspace dependency graph and
//! exposes the expected public surface. Real conversion tests land in
//! the next PR (#1786).

use dora_mavlink2_bridge::{MAVLINK_VERSION, MavlinkVersion};

#[test]
fn targets_mavlink_v2_only() {
    assert!(matches!(MAVLINK_VERSION, MavlinkVersion::V2));
}
