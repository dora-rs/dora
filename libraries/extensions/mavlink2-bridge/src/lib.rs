//! MAVLink 2 ↔ Apache Arrow bridge for the dora dataflow runtime.
//!
//! This crate mirrors the `dora-ros2-bridge` extension for MAVLink 2. It
//! provides:
//!
//! - Arrow conversion for common-dialect messages via the [`MavlinkArrow`]
//!   trait.
//! - Transport builders for TCP, UDP, and serial links (see [`transport`]).
//! - [`MAVLINK_VERSION`], the protocol version a transport uses unless its
//!   URL selects another with `?proto=`.
//!
//! The daemon-spawnable bridge node built on top of this crate lives in the
//! `dora-mavlink2-bridge-node` binary.
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

/// Default MAVLink protocol version of a [`transport::connect`]ed link.
///
/// A `?proto=v1` query on the transport URL overrides it per link.
pub const MAVLINK_VERSION: MavlinkVersion = MavlinkVersion::V2;
