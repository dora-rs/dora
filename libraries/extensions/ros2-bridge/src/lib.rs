#![allow(clippy::missing_safety_doc)]

extern crate self as dora_ros2_bridge;

pub mod prelude {
    pub use crate::_core;
    pub use flume;
    pub use futures;
    pub use futures_timer;
    pub use ros2_client;
    pub use rustdds;
    pub use tracing;
}

pub use dora_message;
pub use prelude::*;

pub mod transport;
pub use transport::dds::detect_service_mapping;

#[cfg(feature = "generate-messages")]
pub mod messages {
    include!(env!("MESSAGES_PATH"));
}

pub mod _core;

#[cfg(test)]
mod ros2_distro {
    use crate::ros2_client::{COMPILED_ROS_DISTRO, RosDistro};

    /// The `ros2-*` features must reach `ros2-client`; `COMPILED_ROS_DISTRO`
    /// is the newest one it sees, and it is what selects the `Gid` layout.
    /// Keep the branches in step with the feature table in `Cargo.toml`.
    #[test]
    fn compiled_distro_matches_the_enabled_feature() {
        let expected = if cfg!(feature = "ros2-kilted") {
            RosDistro::Kilted
        } else if cfg!(feature = "ros2-jazzy") {
            RosDistro::Jazzy
        } else if cfg!(feature = "ros2-iron") {
            RosDistro::Iron
        } else if cfg!(feature = "ros2-humble") {
            RosDistro::Humble
        } else {
            RosDistro::Galactic
        };
        assert_eq!(COMPILED_ROS_DISTRO, expected);
    }
}
