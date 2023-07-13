use serde::{Deserialize, Serialize};

use crate::message::Message;

#[derive(Clone, Copy, Serialize, Deserialize, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct Time {
  pub sec: i32,
  pub nanosec: u32,
}
impl Message for Time {}

impl Time {
  pub const ZERO: Time = Time { sec: 0, nanosec: 0 };

  pub const DUMMY: Time = Time {
    sec: 1234567890,
    nanosec: 1234567890,
  };

  pub fn now() -> Self {
    Self::from_nanos(chrono::Utc::now().timestamp_nanos() as u64)
  }

  fn from_nanos(nanos_since_epoch: u64) -> Self {
    Self {
      sec: (nanos_since_epoch / 1_000_000_000) as i32,
      nanosec: (nanos_since_epoch % 1_000_000_000) as u32,
    }
  }
}

// TODO: Implement constructors and conversions to/from usual Rust time formats
// Note that this type does not specifiy a zero point in time.

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Duration {
  pub sec: i32, // ROS2: Seconds component, range is valid over any possible int32 value.
  pub nanosec: u32, /* ROS2:  Nanoseconds component in the range of [0, 10e9). */

                /* TODO: How does the nanoseconds component work with negative seconds? */
}
impl Message for Duration {}

// TODO: Implement the usual time arithmetic for Time and Duration, i.e.
// Time - Time = Duration
// Time + Duration = Time
// Time - Duration = time
// Duration + Duration = Duration
// Duration - Duration = Duration
// Implement a "zero" Duration value
// Implement conversions to/from Rust's usual Duration types.
