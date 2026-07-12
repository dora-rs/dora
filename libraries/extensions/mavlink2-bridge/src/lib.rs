//! MAVLink 2 ↔ Apache Arrow bridge for the dora dataflow runtime.
//!
//! This crate is the scaffold for a MAVLink 2 bridge that mirrors the
//! `dora-ros2-bridge` extension. Subsequent PRs add Arrow conversion,
//! transport adapters, and the daemon-spawnable bridge node.
//!
//! See <https://github.com/dora-rs/dora/issues/1786> for the design RFC.

mod arrow_convert;
mod error;
pub mod transport;

pub use arrow_convert::MavlinkArrow;
pub use error::{BridgeError, BridgeResult};
// mavlink 0.18 moved the generated dialect modules under
// `mavlink::dialects`, so the common dialect is at
// `dora_mavlink2_bridge::mavlink::dialects::common`.
pub use mavlink::{self, MavlinkVersion};

/// Compile-time guarantee that this bridge targets MAVLink 2.
pub const MAVLINK_VERSION: MavlinkVersion = MavlinkVersion::V2;
