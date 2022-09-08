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
pub trait Publisher: Send + Sync {
    /// Publish the given data to subscribers.
    ///
    /// The data is published to the topic that was used to create this publisher
    /// (see [`CommunicationLayer::publisher`]).
    fn publish(&self, data: &[u8]) -> Result<(), BoxError>;

    /// Clone this publisher, returning the clone as a
    /// [trait object](https://doc.rust-lang.org/book/ch17-02-trait-objects.html).
    fn dyn_clone(&self) -> Box<dyn Publisher>;
}

/// Allows receiving messages published on a topic.
pub trait Subscriber: Send + Sync {
    /// Receives the next message.
    ///
    /// Blocks until the next message is available.
    ///
    /// Depending on the chosen communication backend, some messages might be dropped if
    /// the publisher is faster than the subscriber.
    fn recv(&mut self) -> Result<Option<Vec<u8>>, BoxError>;
}
