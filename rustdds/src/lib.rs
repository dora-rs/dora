//! A pure Rust implementation of Data Distribution Service (DDS).
//!
//! DDS is an object-oriented API [specified](https://www.omg.org/spec/DDS/) by
//! the Object Management Group.
//!
//! DDS communicates over the network using the [RTPS](https://www.omg.org/spec/DDSI-RTPS/)
//! protocol, which by default runs over UDP/IP.
//!
//! This implementation does not attempt to make an accurate implementation of
//! the DDS object API, as it would be quite unnatural to use in Rust as such.
//! However, we aim for functional compatibility, while at the same time using
//! Rust techniques and conventions.
//!
//! Additionally, there is a [ROS2](https://index.ros.org/doc/ros2/) interface, that is simpler to use than DDS
//! when communicating to ROS2 components. See package [ros2-client](https://crates.io/crates/ros2-client).
//! Note: Do not use module [`ros2`] contained within RustDDS. It is no longer
//! being developed.
//!
//! # DDS usage summary
//!
//! * Create a [`DomainParticipant`]. You have to choose a domain id. The
//!   default value is zero.
//! * Create or find a [`Topic`] from the [`DomainParticipant`]. Topics have a
//!   name and a type.
//! * Create a [`Publisher`] and/or [`Subscriber`] from the
//!   [`DomainParticipant`].
//! * To receive data, create a [`DataReader`](with_key::DataReader) from
//!   [`Subscriber`] and [`Topic`].
//! * To send data, create a [`DataWriter`](with_key::DataWriter) from
//!   [`Publisher`] and [`Topic`].
//! * Data from `DataReader` can be read or taken. Taking removes the data
//!   samples from the DataReader, whereas reading only marks them as read.
//!
//! # Concepts
//!
//! * Data is sent and received in consecutive *samples*. When read, a sample is
//!   accompanied with [`SampleInfo`], which contains DDS-generated metadata.
//! * Topics are either With_Key or No_Key.
//!   * With_Key topics are like map data
//!   structures, containing multiple *instances* (map entries), identified by a
//!   *key*. The key must be something that can be extracted from the data
//!   samples. Instances can be created (published) and deleted (disposed).
//!   * No_Key topics have always only one instance, which cannot be disposed.
//!   * Many types and traits in RustDDS have both with_key and no_key versions.
//!     This is
//!   because with_key communication must be able to access keys from data
//! samples, so it is required in type signatures. Such requirement makes no
//! sense for no_key communication, so signature must be different.
//!
//!
//! # Interfacing Rust data types to DDS
//!
//! * DDS, as specified, takes care of data serialization and deserialization.
//! In order for RustDDS to do this, the payload data must be [Serde](https://serde.rs/)
//! serializable/deserializable.
//! * If your data is to be communicated over a WithKey topic, the payload data
//!   type must implement [`Keyed`] trait from this crate.
//! * If you are using [CDR serialization](https://en.wikipedia.org/wiki/Common_Data_Representation)
//!   ([specification, Section 15.3](https://www.omg.org/cgi-bin/doc?formal/02-06-51)) , which
//!   is the DDS default, then use [`CDRSerializerAdapter`] and
//!   [`CDRDeserializerAdapter`]
//!   when such adapters are required. If you need to use another serialization format, then you should find or write
//!   a [Serde data format](https://serde.rs/data-format.html) implementation and wrap it as a (De)SerializerAdaper.
//!
//! # Polling multiple DataReaders
//!
//! There are three alternative methods to poll DataReaders (and DataWriters):
//! mio-0.6, mio-0.8, and async. Use only one of these!
//!
//! ## `mio-0.6`
//!
//! RustDDS is designed to used with [mio](mio_06) version 0.6.x. DataReaders
//! implement [`Evented`](mio_06::event::Evented) so that they can be directly
//! registered to a [`poll`](mio_06::Poll). See example `shapes_demo`.
//!
//! ## `mio-0.8`
//!
//! RustDDS DataReaders implement [`mio_08::event::Source`] for registering with
//! mio-0.8. See example `shapes_demo_mio_08`
//!
//!
//! ## `async`
//!
//! DataReader and DataWriter can do Rust async I/O by converting themselves to
//! [`futures::stream::Stream`]s.
//! * [`crate::dds::with_key::DataReader::async_sample_stream`] to get data
//! * [`crate::dds::with_key::DataReaderStream::async_event_stream`] to get
//!   status events
//!
//! See exampe `async_shapes_demo`.
//!
//! # Usage Example
//!
//! ```
//! use rustdds::*;
//! use rustdds::no_key::{DataReader, DataWriter, DataSample}; // We use a NO_KEY topic here
//! use serde::{Serialize, Deserialize};
//!
//! // DomainParticipant is always necessary
//! let domain_participant = DomainParticipant::new(0).unwrap();
//!
//! let qos = QosPolicyBuilder::new()
//!   .reliability(policy::Reliability::Reliable { max_blocking_time: rustdds::Duration::DURATION_ZERO })
//!   .build();
//!
//! // DDS Subscriber, only one is necessary for each thread (slight difference to
//! // DDS specification)
//! let subscriber = domain_participant.create_subscriber(&qos).unwrap();
//!
//! // DDS Publisher, only one is necessary for each thread (slight difference to
//! // DDS specification)
//! let publisher = domain_participant.create_publisher(&qos).unwrap();
//!
//! // Some DDS Topic that we can write and read from (basically only binds readers
//! // and writers together)
//! let some_topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
//!
//! // Used type needs Serialize for writers and Deserialize for readers
//! #[derive(Serialize, Deserialize)]
//! struct SomeType {
//!   a: i32
//! }
//!
//! // Creating DataReader requires type and deserializer adapter (which is recommended to be CDR).
//! // Reader needs to be mutable if any operations are used.
//! let mut reader = subscriber
//!   .create_datareader_no_key::<SomeType, CDRDeserializerAdapter<SomeType>>(
//!     &some_topic,
//!     None)
//!   .unwrap();
//!
//! // Creating DataWriter required type and serializer adapter (which is recommended to be CDR).
//! let writer = publisher
//!   .create_datawriter_no_key::<SomeType, CDRSerializerAdapter<SomeType>>(
//!     &some_topic,
//!     None)
//!   .unwrap();
//!
//! // Readers implement mio Evented trait and thus function the same way as
//! // std::sync::mpcs and can be handled the same way for reading the data
//!
//! let some_data = SomeType { a: 1 };
//!
//! // This should send the data to all who listen "some_topic" topic.
//! writer.write(some_data, None).unwrap();
//!
//! // ... Some data has arrived at some point for the reader
//! let data_sample = if let Ok(Some(value)) = reader.take_next_sample() {
//!   value
//! } else {
//!   // no data has arrived
//!   return;
//! };
//!
//! // Getting reference to actual data from the data sample
//! let actual_data = data_sample.value();
//! ```
#![deny(clippy::all)]
#![warn(clippy::needless_pass_by_value, clippy::semicolon_if_nothing_returned)]
#![allow(
  // option_map_unit_fn suggests changing option.map( ) with () return value to if let -construct,
  // but that may break code flow.
  clippy::option_map_unit_fn,
)]

#[macro_use]
mod serialization_test;
#[macro_use]
mod checked_impl;
#[doc(hidden)]
pub mod discovery; // to access some Discovered data in e.g. ros2-client crate
mod messages;
mod network;
mod rtps;
mod security;
pub(crate) mod structure;

#[cfg(test)]
mod test;

mod mio_source;

// Public modules
pub mod dds; // this is public, but not advertised
pub mod ros2;
/// Helpers for (De)serialization and definitions of (De)serializer adapters
pub mod serialization;

// Re-exports from crate root to simplify usage
#[doc(inline)]
pub use dds::{
  key::{Key, Keyed},
  participant::DomainParticipant,
  pubsub::{Publisher, Subscriber},
  qos,
  qos::{policy, QosPolicies, QosPolicyBuilder},
  readcondition::ReadCondition,
  sampleinfo::{InstanceState, NotAliveGenerationCounts, SampleInfo, SampleState, ViewState},
  statusevents::StatusEvented,
  topic::{Topic, TopicDescription, TopicKind},
  typedesc::TypeDesc,
  with_key::{datareader::SelectByKey, WriteOptions, WriteOptionsBuilder},
};
/// Needed to specify serialized data representation in case it is other than
/// CDR.
pub use serialization::representation_identifier::RepresentationIdentifier;
#[doc(inline)]
pub use serialization::{
  CDRDeserializerAdapter, CDRSerializerAdapter, CdrDeserializer, CdrSerializer,
};
pub use structure::{
  duration::Duration, entity::RTPSEntity, guid::GUID, sequence_number::SequenceNumber,
  time::Timestamp,
};
// re-export from a helper crate
/// Helper trait to compute the CDR-serialized size of data
pub use cdr_encoding_size::CdrEncodingSize;

/// Components used to access NO_KEY Topics
pub mod no_key {
  pub use crate::dds::{adapters::no_key::*, no_key::*};
}

/// Components used to access WITH_KEY Topics
pub mod with_key {
  pub use crate::dds::{adapters::with_key::*, with_key::*};
}

pub mod rpc {
  pub use crate::structure::rpc::*;
}
