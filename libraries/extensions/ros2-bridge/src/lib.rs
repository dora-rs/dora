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
