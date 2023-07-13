//! DDS interface - Most commonly needed items should be re-exported directly to
//! crate top level and modules [`no_key`](crate::no_key) and
//! [`with_key`](crate::with_key).

mod helpers;

pub(crate) mod participant;
pub use participant::DomainParticipant;

pub(crate) mod dds_entity;
pub(crate) mod ddsdata;
pub(crate) mod pubsub;
pub(crate) mod readcondition;
pub(crate) mod topic;
pub(crate) mod typedesc;

pub mod result;
pub use result::{Error, Result};

// Public interface

/// DDS Quality of Service policies
pub mod qos;

/// Events that report other things than data samples received, e.g. new
/// endpoints matched or communication errors.
pub mod statusevents;

/// DDS Sample metadata
pub mod sampleinfo;

/// Defines instance Keys that are needed to access WITH_KEY topics.
pub mod key;

/// Participating to NoKey topics.
pub mod no_key;
/// Participating to WithKey topics.
pub mod with_key;

/// Serializer/deserializer adapters to connect serialization to RTPS.
pub mod adapters;
