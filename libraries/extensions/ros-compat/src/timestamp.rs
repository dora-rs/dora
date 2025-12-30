//! Timestamp conversion utilities for ROS and Dora
//!
//! Note: Full timestamp conversion requires a uhlc::Clock instance.
//! For now, this provides basic conversion utilities that can be extended
//! when integrating with actual ROS messages.

use std::time::{SystemTime, UNIX_EPOCH, Duration};

/// ROS 1 timestamp (secs, nsecs)
#[derive(Debug, Clone, Copy)]
pub struct Ros1Time {
    pub secs: u32,
    pub nsecs: u32,
}

/// ROS 2 timestamp (sec, nanosec)
#[derive(Debug, Clone, Copy)]
pub struct Ros2Time {
    pub sec: i32,
    pub nanosec: u32,
}

impl Ros1Time {
    /// Convert ROS 1 time to SystemTime
    pub fn to_system_time(&self) -> SystemTime {
        let duration = Duration::new(self.secs as u64, self.nsecs);
        UNIX_EPOCH + duration
    }

    /// Create from SystemTime
    pub fn from_system_time(time: SystemTime) -> eyre::Result<Self> {
        let duration = time.duration_since(UNIX_EPOCH)?;
        Ok(Self {
            secs: duration.as_secs() as u32,
            nsecs: duration.subsec_nanos(),
        })
    }

    /// Get total nanoseconds since epoch
    pub fn as_nanos(&self) -> u64 {
        (self.secs as u64) * 1_000_000_000 + (self.nsecs as u64)
    }
}

impl Ros2Time {
    /// Convert ROS 2 time to SystemTime
    pub fn to_system_time(&self) -> SystemTime {
        if self.sec >= 0 {
            let duration = Duration::new(self.sec as u64, self.nanosec);
            UNIX_EPOCH + duration
        } else {
            // Handle negative seconds (before epoch)
            let abs_secs = (-self.sec) as u64;
            let duration = Duration::new(abs_secs, self.nanosec);
            UNIX_EPOCH - duration
        }
    }

    /// Create from SystemTime
    pub fn from_system_time(time: SystemTime) -> eyre::Result<Self> {
        let duration = time.duration_since(UNIX_EPOCH)?;
        Ok(Self {
            sec: duration.as_secs() as i32,
            nanosec: duration.subsec_nanos(),
        })
    }

    /// Get total nanoseconds since epoch
    pub fn as_nanos(&self) -> i64 {
        if self.sec >= 0 {
            (self.sec as i64) * 1_000_000_000 + (self.nanosec as i64)
        } else {
            -(((-self.sec) as i64) * 1_000_000_000 + (self.nanosec as i64))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ros1_to_dora() {
        let ros_time = Ros1Time {
            secs: 1234567890,
            nsecs: 123456789,
        };
        let system_time = ros_time.to_system_time();
        let back = Ros1Time::from_system_time(system_time).unwrap();
        assert_eq!(ros_time.secs, back.secs);
        assert_eq!(ros_time.nsecs, back.nsecs);
    }

    #[test]
    fn test_ros2_to_dora() {
        let ros_time = Ros2Time {
            sec: 1234567890,
            nanosec: 123456789,
        };
        let system_time = ros_time.to_system_time();
        let back = Ros2Time::from_system_time(system_time).unwrap();
        assert_eq!(ros_time.sec, back.sec);
        assert_eq!(ros_time.nanosec, back.nanosec);
    }
}

