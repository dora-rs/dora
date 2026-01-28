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
//!         # Converts to a list of dicts (one per message in the batch)
//!         ros_msgs = converter.to_ros(event["value"])
//!         for msg in ros_msgs:
//!             print(f"Linear: {msg['linear']}")
//! ```

#[cfg(feature = "python")]
pub mod python;
