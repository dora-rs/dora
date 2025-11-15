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

#[cfg(feature = "generate-messages")]
pub mod messages {
    include!(env!("MESSAGES_PATH"));
}

pub mod _core;
