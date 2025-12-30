//! Common ROS message type definitions and conversions

use arrow::array::ArrayRef;
use eyre::Result;

/// Common ROS message types that can be converted to/from Dora format
pub mod std_msgs {
    use super::*;

    /// std_msgs/Header
    pub struct Header {
        pub seq: u32,
        pub stamp: Time,
        pub frame_id: String,
    }

    /// ROS Time (compatible with both ROS 1 and ROS 2)
    pub struct Time {
        pub secs: u32,
        pub nsecs: u32,
    }

    impl Header {
        pub fn to_dora(&self) -> Result<ArrayRef> {
            crate::converter::common_types::header_to_arrow(
                self.seq,
                self.stamp.secs,
                self.stamp.nsecs,
                &self.frame_id,
            )
        }
    }
}

/// Geometry message types
pub mod geometry_msgs {
    use super::*;

    /// geometry_msgs/Twist
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }

    /// geometry_msgs/Vector3
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl Twist {
        pub fn to_dora(&self) -> Result<ArrayRef> {
            crate::converter::common_types::twist_to_arrow(
                self.linear.x,
                self.linear.y,
                self.linear.z,
                self.angular.x,
                self.angular.y,
                self.angular.z,
            )
        }
    }
}

