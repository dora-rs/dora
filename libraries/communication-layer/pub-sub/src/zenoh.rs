//! Provides [`ZenohCommunicationLayer`] to communicate over `zenoh`.

use super::{CommunicationLayer, Publisher, Subscriber};
use crate::{BoxError, ReceivedSample};
use std::{borrow::Cow, sync::Arc, time::Duration};
use zenoh::{
    prelude::{Config, Priority, SessionDeclarations, SplitBuffer, sync::SyncResolve},
    publication::CongestionControl,
};

/// Allows communication over `zenoh`.
pub struct ZenohCommunicationLayer {
    zenoh: Arc<zenoh::Session>,
    topic_prefix: String,
}

impl ZenohCommunicationLayer {
    /// Initializes a new `zenoh` session with the given configuration.
    ///
    /// The `prefix` is added to all topic names when using the [`publisher`][Self::publisher]
    /// and [`subscriber`][Self::subscribe] methods. Pass an empty string if no prefix is
    /// desired.
    pub fn init(config: Config, prefix: String) -> Result<Self, BoxError> {
        let zenoh = ::zenoh::open(config).res_sync()?.into_arc();
        Ok(Self {
            zenoh,
            topic_prefix: prefix,
        })
    }

    fn prefixed(&self, topic: &str) -> String {
        format!("{}/{topic}", self.topic_prefix)
    }
}

impl CommunicationLayer for ZenohCommunicationLayer {
    fn publisher(&mut self, topic: &str) -> Result<Box<dyn Publisher>, BoxError> {
        let publisher = self
            .zenoh
            .declare_publisher(self.prefixed(topic))
            .congestion_control(CongestionControl::Block)
            .priority(Priority::RealTime)
            .res_sync()?;

        Ok(Box::new(ZenohPublisher { publisher }))
    }

    fn subscribe(&mut self, topic: &str) -> Result<Box<dyn Subscriber>, BoxError> {
        let subscriber = self
            .zenoh
            .declare_subscriber(self.prefixed(topic))
            .reliable()
            .res_sync()?;

        Ok(Box::new(ZenohReceiver(subscriber)))
    }
}

impl Drop for ZenohCommunicationLayer {
    fn drop(&mut self) {
        // wait a bit before closing to ensure that remaining published
        // messages are sent out
        //
        // TODO: create a minimal example to reproduce the dropped messages
        // and report this issue in the zenoh repo
        std::thread::sleep(Duration::from_secs_f32(2.0));
    }
}

#[derive(Clone)]
struct ZenohPublisher {
    publisher: zenoh::publication::Publisher<'static>,
}

impl Publisher for ZenohPublisher {
    fn prepare(&self, len: usize) -> Result<Box<dyn crate::PublishSample>, BoxError> {
        Ok(Box::new(ZenohPublishSample {
            sample: vec![0; len],
            publisher: self.publisher.clone(),
        }))
    }

    fn dyn_clone(&self) -> Box<dyn Publisher> {
        Box::new(self.clone())
    }
}

#[derive(Clone)]
struct ZenohPublishSample {
    sample: Vec<u8>,
    publisher: zenoh::publication::Publisher<'static>,
}

impl<'a> crate::PublishSample<'a> for ZenohPublishSample {
    fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.sample
    }

    fn publish(self: Box<Self>) -> Result<(), BoxError> {
        self.publisher.put(self.sample).res_sync()
    }
}

struct ZenohReceiver(
    zenoh::subscriber::Subscriber<'static, flume::Receiver<zenoh::sample::Sample>>,
);

impl Subscriber for ZenohReceiver {
    fn recv(&mut self) -> Result<Option<Box<dyn ReceivedSample>>, BoxError> {
        match self.0.recv() {
            Ok(sample) => Ok(Some(Box::new(ZenohReceivedSample {
                sample: sample.value.payload,
            }))),
            Err(_) => Ok(None),
        }
    }
}

struct ZenohReceivedSample {
    sample: zenoh::buffers::ZBuf,
}

impl ReceivedSample for ZenohReceivedSample {
    fn get(&self) -> Cow<[u8]> {
        self.sample.contiguous()
    }
}
