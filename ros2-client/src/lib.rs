//! ROS2 interface using DDS module
//!
//! # Examples
//!
//! ```
//! use rustdds::*;
//! use ros2_client::*;
//! use ros2_client::node_entities_info::NodeEntitiesInfo;
//!
//!
//! let mut ros_context = Context::new().unwrap();
//!
//!
//! let mut ros_node = ros_context.new_node(
//!   "some_node_name",
//!   "/some_namespace",
//!   NodeOptions::new().enable_rosout(true),
//!   ).unwrap();
//!
//! let some_topic = ros_node.create_topic(
//!     "some_topic_name",
//!     "NodeEntitiesInfo".to_string(),
//!     &QosPolicies::builder().build() )
//!   .unwrap();
//!
//! // declaring some writer that use non keyed types
//! let some_writer = ros_node
//!   .create_publisher::<NodeEntitiesInfo>(&some_topic, None)
//!   .unwrap();
//!
//! // Publisher and subscription implement [`mio::Evented`], so thay can be polled.
//! ```

#[macro_use]
extern crate lazy_static;

/// Some builtin datatypes needed for ROS2 communication
/// Some convenience topic infos for ROS2 communication
pub mod builtin_topics;

pub mod action_msgs;
/// Some builtin interfaces for ROS2 communication
pub mod builtin_interfaces;
#[doc(hidden)]
pub mod context;
pub mod unique_identifier_msgs;

pub mod interfaces;

pub mod action;
mod gid;
pub mod log;
pub mod message;
pub mod node_entities_info;
pub mod parameters;
pub mod participant_entities_info;
#[doc(hidden)]
pub mod pubsub;
pub mod service;

#[doc(hidden)]
pub(crate) mod node;

// Re-exports from crate root to simplify usage
#[doc(inline)]
pub use context::*;
#[doc(inline)]
pub use message::{Message, MessageTypeName};
#[doc(inline)]
pub use node::*;
#[doc(inline)]
pub use pubsub::*;
#[doc(inline)]
pub use service::{AService, Client, Server, Service, ServiceMapping};
#[doc(inline)]
pub use action::{Action, ActionTypes};

/// Module for stuff we do not want to export from top level;
pub mod ros2 {
  pub use rustdds::{Duration, Timestamp};
}
