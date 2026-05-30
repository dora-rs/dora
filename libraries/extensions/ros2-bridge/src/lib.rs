#![allow(clippy::missing_safety_doc)]

pub mod prelude {
    pub use crate::_core;
    pub use flume;
    pub use futures;
    pub use futures_timer;
    pub use ros2_client;
    pub use rustdds;
    pub use tracing;
}

pub use prelude::*;

/// Pick the [`ros2_client::ServiceMapping`] variant that matches the user's ROS2
/// middleware, based on `RMW_IMPLEMENTATION` (primary) and `ROS_DISTRO`
/// (fallback). Falls back to `Enhanced` with a `tracing::warn!` if neither env
/// var gives a usable answer — `Enhanced` is the historical default and keeps
/// existing setups working when env is unset.
///
/// Shared by the standalone bridge daemon and the Python bindings so their
/// service/action wire mapping can never drift. See dora-rs/dora#449.
pub fn detect_service_mapping() -> ros2_client::ServiceMapping {
    match std::env::var("RMW_IMPLEMENTATION").ok().as_deref() {
        Some("rmw_fastrtps_cpp") => return ros2_client::ServiceMapping::Enhanced,
        Some("rmw_cyclonedds_cpp") => return ros2_client::ServiceMapping::Cyclone,
        Some(other) => {
            tracing::warn!(
                "unknown RMW_IMPLEMENTATION `{other}`, falling back to \
                 ServiceMapping::Enhanced (see dora-rs/dora#449)"
            );
            return ros2_client::ServiceMapping::Enhanced;
        }
        None => {}
    }
    match std::env::var("ROS_DISTRO").ok().as_deref() {
        Some("humble" | "iron" | "jazzy" | "kilted" | "rolling") => {
            ros2_client::ServiceMapping::Enhanced
        }
        Some("galactic") => ros2_client::ServiceMapping::Cyclone,
        Some(other) => {
            tracing::warn!(
                "unknown ROS_DISTRO `{other}`, falling back to \
                 ServiceMapping::Enhanced (see dora-rs/dora#449)"
            );
            ros2_client::ServiceMapping::Enhanced
        }
        None => ros2_client::ServiceMapping::Enhanced,
    }
}

#[cfg(feature = "generate-messages")]
pub mod messages {
    include!(env!("MESSAGES_PATH"));
}

pub mod _core;
