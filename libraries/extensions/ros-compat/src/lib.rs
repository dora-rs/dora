//! ROS Message Format Compatibility Layer
//!
//! This library provides interoperability between Dora and ROS message formats.
//! It supports both ROS 1 and ROS 2 message types and provides automatic
//! conversion to/from Dora's Arrow-based data format.

pub mod converter;
pub mod message_parser;
pub mod types;
pub mod timestamp;
pub mod bridge_config;
#[cfg(any(feature = "ros1", feature = "ros2"))]
pub mod bridge;

// Re-export bridge for easier access
#[cfg(any(feature = "ros1", feature = "ros2"))]
pub use bridge::RosBridge;
#[cfg(feature = "python")]
pub mod python;

#[cfg(test)]
mod tests;

use arrow::array::ArrayRef;
use eyre::Result;

/// Convert a ROS message to Dora Arrow format
pub trait ToDora {
    /// Convert this ROS message to an Arrow array
    fn to_dora(&self) -> Result<ArrayRef>;
}

/// Convert from Dora Arrow format to a ROS message
pub trait FromDora {
    /// Convert from an Arrow array to this ROS message type
    fn from_dora(array: &ArrayRef) -> Result<Self>
    where
        Self: Sized;
}

/// ROS version enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RosVersion {
    Ros1,
    Ros2,
}

impl RosVersion {
    pub fn from_str(s: &str) -> Result<Self> {
        match s {
            "1" | "ros1" => Ok(Self::Ros1),
            "2" | "ros2" => Ok(Self::Ros2),
            _ => eyre::bail!("Invalid ROS version: {}. Use '1' or '2'", s),
        }
    }
}

impl std::fmt::Display for RosVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Ros1 => write!(f, "ROS 1"),
            Self::Ros2 => write!(f, "ROS 2"),
        }
    }
}

