//! ROS Message Format Compatibility Layer
//!
//! This library provides a minimal Python API for converting Dora Arrow arrays
//! to ROS message format dictionaries.
//!
//! # Example
//!
//! ```python
//! from dora import Node
//! from dora.ros import RosMessageConverter
//!
//! node = Node()
//! converter = RosMessageConverter()
//!
//! for event in node:
//!     if event["type"] == "INPUT":
//!         ros_msg = converter.to_ros(event["value"], "geometry_msgs/Twist")
//!         print(f"Linear: {ros_msg['linear']}")
//! ```

#[cfg(feature = "python")]
pub mod python;
