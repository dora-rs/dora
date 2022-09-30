#![warn(missing_docs)]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]

//! Abstraction of various publisher/subscriber communication backends.
//!
//! Provides a [`CommunicationLayer`] trait as an abstraction for different publisher/subscriber
//! systems. The following set of backends are currently supported:
//!
//! - **[Zenoh](https://zenoh.io/):** The zenoh project implements a distributed
//!   publisher/subscriber system with automated routing. To use zenoh, use the
//!   [`ZenohCommunicationLayer`][zenoh::ZenohCommunicationLayer] struct.
//! - **[Iceoryx](https://iceoryx.io/):** The Eclipse iceoryx™ project provides an IPC middleware
//!   based on shared memory. It is very fast, but it only supports local communication. To use
//!   iceoryx, use the [`IceoryxCommunicationLayer`][iceoryx::IceoryxCommunicationLayer] struct.

use std::{borrow::Cow, fmt::Debug};

#[cfg(all(unix, feature = "iceoryx"))]
pub mod iceoryx;
#[cfg(feature = "zenoh")]
pub mod zenoh;

type BoxError = Box<dyn std::error::Error + Send + Sync + 'static>;

/// Abstraction trait for different publisher/subscriber implementations.
pub trait CommunicationLayer: Send + Sync {
    /// Creates a publisher for the given topic.
    fn publisher(&mut self, topic: &str) -> Result<Box<dyn Publisher>, BoxError>;

    /// Subscribe to the given topic.
    fn subscribe(&mut self, topic: &str) -> Result<Box<dyn Subscriber>, BoxError>;
}

/// Allows publishing messages to subscribers.
///
/// The messages is published to the topic that was used to create the publisher
/// (see [`CommunicationLayer::publisher`]).
pub trait Publisher: Send + Sync {
    /// Prepare memory for publishing a message with the given length.
    ///
    /// This function makes it possible to construct messages without
    /// any additional copying. The returned [`Sample`] is initialized
    /// with zeros.
    fn prepare(&self, len: usize) -> Result<Box<dyn PublishSample + '_>, BoxError>;

    /// Clone this publisher, returning the clone as a
    /// [trait object](https://doc.rust-lang.org/book/ch17-02-trait-objects.html).
    fn dyn_clone(&self) -> Box<dyn Publisher>;

    /// Publishes the gives message to subscribers.
    ///
    /// Depending on the backend, this method might need to copy the data, which can
    /// decrease performance. To avoid this, the [`prepare`](Publisher::prepare) function
    /// can be used to construct the message in-place.
    fn publish(&self, data: &[u8]) -> Result<(), BoxError> {
        let mut sample = self.prepare(data.len())?;
        sample.as_mut_slice().copy_from_slice(&data);
        sample.publish()?;
        Ok(())
    }
}

/// A prepared message constructed by [`Publisher::prepare`].
pub trait PublishSample<'a>: Send + Sync {
    /// Gets a reference to the prepared message.
    ///
    /// Makes it possible to construct the message in-place.
    fn as_mut_slice(&mut self) -> &mut [u8];

    /// Publish this sample to subscribers.
    ///
    /// The sample is published to the topic that was used to create the corresponding publisher
    /// (see [`CommunicationLayer::publisher`]).
    fn publish(self: Box<Self>) -> Result<(), BoxError>;
}

/// Allows receiving messages published on a topic.
pub trait Subscriber: Send + Sync {
    /// Receives the next message.
    ///
    /// Blocks until the next message is available.
    ///
    /// Depending on the chosen communication backend, some messages might be dropped if
    /// the publisher is faster than the subscriber.
    fn recv(&mut self) -> Result<Option<Box<dyn ReceivedSample>>, BoxError>;
}

pub trait ReceivedSample: Send + Sync {
    fn get(&self) -> Cow<[u8]>;
}
