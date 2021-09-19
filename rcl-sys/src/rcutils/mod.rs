//! Wrapper of [rcutils](https://github.com/ros2/rcutils)

pub mod allocator;
pub use allocator::*;

pub mod error_handling;
pub use error_handling::*;

pub mod logging;
pub use logging::*;

pub mod types;
pub use types::*;

pub mod time {
    //! API in rcutils/time.h

    /// A single point in time, measured in nanoseconds since the Unix epoch.
    pub type rcutils_time_point_value_t = i64;
    /// A duration of time, measured in nanoseconds.
    pub type rcutils_duration_value_t = i64;
}
pub use time::*;
