//! ROS2 interface using DDS module - DO NOT USE - Use [ros2-client](https://crates.io/crates/ros2-client) instead.
//!
//! # Examples
//!
//! ```
//! use rustdds::*;
//! use rustdds::ros2::RosParticipant;
//! use rustdds::ros2::NodeOptions;
//! use rustdds::ros2::RosNode;
//! use rustdds::ros2::builtin_datatypes::NodeInfo;
//! use rustdds::serialization::CDRSerializerAdapter;
//!
//!
//!
//! // RosParticipant is needed for defined RosNodes to be visible in ROS2 network.
//! let mut ros_participant = RosParticipant::new().unwrap();
//!
//!
//! // declaring ros node
//! let mut ros_node = ros_participant.new_ros_node(
//!   "some_node_name",
//!   "/some_namespace",
//!   NodeOptions::new(false), // enable rosout?
//!   ).unwrap();
//!
//! // Creating some topic for RosNode
//! let some_topic = ros_node.create_ros_topic(
//!     "some_topic_name",
//!     "NodeInfo".to_string(),
//!     &QosPolicies::builder().build(),
//!     TopicKind::NoKey)
//!   .unwrap();
//!
//! // declaring some writer that use non keyed types
//! let some_writer = ros_node
//!   .create_ros_no_key_publisher::<NodeInfo, CDRSerializerAdapter<_>>(
//!     &some_topic, None)
//!   .unwrap();
//!
//! // Readers and RosParticipant implement mio Evented trait and thus function the same way as
//! // std::sync::mpcs and can be handled the same way for reading the data
//! ```

/// Some builtin datatypes needed for ROS2 communication
pub mod builtin_datatypes;
/// Some convenience topic infos for ROS2 communication
pub mod builtin_topics;

pub(crate) mod ros_node;

pub use ros_node::*;

pub type RosSubscriber<D, DA> = crate::dds::no_key::datareader::DataReader<D, DA>;

pub type KeyedRosSubscriber<D, DA> = crate::dds::with_key::datareader::DataReader<D, DA>;

pub type RosPublisher<D, SA> = crate::dds::no_key::datawriter::DataWriter<D, SA>;

pub type KeyedRosPublisher<D, SA> = crate::dds::with_key::datawriter::DataWriter<D, SA>;

// Short-hand notation for CDR serialization

pub type RosSubscriberCdr<D> =
  crate::dds::no_key::datareader::DataReader<D, crate::serialization::CDRDeserializerAdapter<D>>;

pub type KeyedRosSubscriberCdr<D> =
  crate::dds::with_key::datareader::DataReader<D, crate::serialization::CDRDeserializerAdapter<D>>;

pub type RosPublisherCdr<D> =
  crate::dds::no_key::datawriter::DataWriter<D, crate::serialization::CDRSerializerAdapter<D>>;

pub type KeyedRosPublisherCdr<D> =
  crate::dds::with_key::datawriter::DataWriter<D, crate::serialization::CDRSerializerAdapter<D>>;
