use std::convert::TryInto;
use std::time::Duration;

use rclrust_msg::builtin_interfaces;

use crate::clock::ClockType;

const S_TO_NS: i64 = 1_000_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Time {
    pub nanosecs: i64,
    pub clock_type: ClockType,
}

impl Time {
    /// # Examples
    ///
    /// ```
    /// use rclrust::{ClockType, Time};
    ///
    /// let time = Time::new(10, 100000, ClockType::RosTime);
    /// ```
    pub fn new(secs: i32, nanosecs: u32, clock_type: ClockType) -> Self {
        Self {
            nanosecs: i64::from(secs) * S_TO_NS + i64::from(nanosecs),
            clock_type,
        }
    }

    /// # Examples
    ///
    /// ```
    /// use rclrust::{ClockType, Time};
    ///
    /// let time = Time::from_nanosecs(100000, ClockType::RosTime);
    /// ```
    pub const fn from_nanosecs(nanosecs: i64, clock_type: ClockType) -> Self {
        Self {
            nanosecs,
            clock_type,
        }
    }

    /// # Examples
    ///
    /// ```
    /// use rclrust::{ClockType, Time};
    /// use rclrust_msg::builtin_interfaces;
    ///
    /// let time_msg = builtin_interfaces::msg::Time {
    ///     sec: 5,
    ///     nanosec: 10,
    /// };
    /// let time = Time::from_ros_msg(&time_msg, ClockType::RosTime);
    /// assert_eq!(time, Time::new(5, 10, ClockType::RosTime));
    /// ```
    pub fn from_ros_msg(time_msg: &builtin_interfaces::msg::Time, clock_type: ClockType) -> Self {
        Self::new(time_msg.sec, time_msg.nanosec, clock_type)
    }

    /// # Examples
    ///
    /// ```
    /// use rclrust::{ClockType, Time};
    /// use rclrust_msg::builtin_interfaces;
    ///
    /// let time = Time::new(5, 10, ClockType::RosTime);
    /// let time_msg = builtin_interfaces::msg::Time {
    ///     sec: 5,
    ///     nanosec: 10,
    /// };
    /// assert_eq!(time.to_ros_msg(), time_msg);
    /// ```
    pub fn to_ros_msg(self) -> builtin_interfaces::msg::Time {
        let sec = self.nanosecs / S_TO_NS;
        let nanosec = self.nanosecs % S_TO_NS;

        let (sec, nanosec) = if self.nanosecs >= 0 {
            (sec, nanosec)
        } else {
            (sec - 1, nanosec + S_TO_NS)
        };
        builtin_interfaces::msg::Time {
            sec: sec.try_into().unwrap(),
            nanosec: nanosec.try_into().unwrap(),
        }
    }
}

pub(crate) trait RclDurationT {
    fn to_rmw_time_t(&self) -> rcl_sys::rmw_time_t;

    fn from_rmw_time_t(duration: &rcl_sys::rmw_time_t) -> Self;
}

impl RclDurationT for Duration {
    fn to_rmw_time_t(&self) -> rcl_sys::rmw_time_t {
        rcl_sys::rmw_time_t {
            sec: self.as_secs(),
            nsec: self.subsec_nanos().into(),
        }
    }

    fn from_rmw_time_t(duration: &rcl_sys::rmw_time_t) -> Self {
        Self::new(duration.sec, duration.nsec.try_into().unwrap())
    }
}
