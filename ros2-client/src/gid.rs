use serde::{Deserialize, Serialize};
use rustdds::*;

/// ROS2 equivalent for DDS GUID
///
/// See https://github.com/ros2/rmw_dds_common/blob/master/rmw_dds_common/msg/Gid.msg
#[derive(
  Debug, Copy, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, CdrEncodingSize,
)]
pub struct Gid {
  data: [u8; 24],
  // TODO: ROS2 seems to be changing this to 16 bytes.
  // https://github.com/ros2/rmw_dds_common/commit/5ab4f5944e4442fe0188e15b10cf11377fb45801
  // As of Jan 2023
}

impl Gid {
  pub fn from_guid(guid: GUID) -> Gid {
    let mut data: [u8; 24] = [0; 24];
    data[..12].clone_from_slice(guid.prefix.as_ref());
    data[12..15].clone_from_slice(&guid.entity_id.entity_key);
    data[15] = u8::from(guid.entity_id.entity_kind);
    Gid { data }
  }
}

impl Key for Gid {}
