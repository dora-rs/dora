//! Provides [`IceoryxCommunicationLayer`] to communicate over `iceoryx`.

use super::{CommunicationLayer, Publisher, Subscriber};
use crate::{BoxError, ReceivedSample};
use eyre::Context;
use std::{borrow::Cow, collections::HashMap, sync::Arc, time::Duration};

/// Enables local communication based on `iceoryx`.
pub struct IceoryxCommunicationLayer {
    group_name: String,
    instance_name: String,
    publishers: HashMap<String, Arc<iceoryx_rs::Publisher<[u8]>>>,
}

impl IceoryxCommunicationLayer {
    /// Initializes a new `iceoryx` connection with default configuration.
    ///
    /// The given `app_name` must be unique. The `group_name` and
    /// `instance_name` arguments are used to create an `iceoryx`
    /// `ServiceDescription` in combination wiith topic names given to the
    /// [`publisher`][Self::publisher] and [`subscriber`][Self::subscribe]
    /// methods. See the
    /// [`iceoryx` documentation](https://iceoryx.io/v2.0.1/getting-started/overview/#creating-service-descriptions-for-topics)
    /// for details.
    ///
    /// Note: In order to use iceoryx, you need to start its broker deamon called
    /// [_RouDi_](https://iceoryx.io/v2.0.2/getting-started/overview/#roudi) first.
    /// Its executable name is `iox-roudi`. See the
    /// [`iceoryx` installation chapter](https://iceoryx.io/v2.0.2/getting-started/installation/)
    /// for ways to install it.
    pub fn init(
        app_name: String,
        group_name: String,
        instance_name: String,
    ) -> Result<Self, BoxError> {
        iceoryx_rs::Runtime::init(&app_name);

        Ok(Self {
            group_name,
            instance_name,
            publishers: Default::default(),
        })
    }
}

impl IceoryxCommunicationLayer {
    fn get_or_create_publisher(
        &mut self,
        topic: &str,
    ) -> eyre::Result<Arc<iceoryx_rs::Publisher<[u8]>>> {
        match self.publishers.get(topic) {
            Some(p) => Ok(p.clone()),
            None => {
                let publisher =
                    Self::create_publisher(&self.group_name, &self.instance_name, topic)
                        .context("failed to create iceoryx publisher")?;

                let publisher = Arc::new(publisher);
                self.publishers.insert(topic.to_owned(), publisher.clone());
                Ok(publisher)
            }
        }
    }

    fn create_publisher(
        group: &str,
        instance: &str,
        topic: &str,
    ) -> Result<iceoryx_rs::Publisher<[u8]>, iceoryx_rs::IceoryxError> {
        iceoryx_rs::PublisherBuilder::new(group, instance, topic).create()
    }
}

impl CommunicationLayer for IceoryxCommunicationLayer {
    fn publisher(&mut self, topic: &str) -> Result<Box<dyn Publisher>, crate::BoxError> {
        let publisher = self
            .get_or_create_publisher(topic)
            .map_err(BoxError::from)?;

        Ok(Box::new(IceoryxPublisher { publisher }))
    }

    fn subscribe(&mut self, topic: &str) -> Result<Box<dyn Subscriber>, crate::BoxError> {
        let (subscriber, token) =
            iceoryx_rs::SubscriberBuilder::new(&self.group_name, &self.instance_name, topic)
                .queue_capacity(5)
                .create_mt()
                .context("failed to create iceoryx subscriber")
                .map_err(BoxError::from)?;
        let receiver = subscriber.get_sample_receiver(token);

        Ok(Box::new(IceoryxReceiver { receiver }))
    }
}

#[derive(Clone)]
struct IceoryxPublisher {
    publisher: Arc<iceoryx_rs::Publisher<[u8]>>,
}

impl Publisher for IceoryxPublisher {
    fn prepare(&self, len: usize) -> Result<Box<dyn crate::PublishSample>, BoxError> {
        let sample = self
            .publisher
            .loan_slice_arc(len)
            .context("failed to loan iceoryx slice for publishing")
            .map_err(BoxError::from)?;
        Ok(Box::new(IceoryxPublishSample {
            sample,
            publisher: self.publisher.clone(),
        }))
    }

    fn dyn_clone(&self) -> Box<dyn Publisher> {
        Box::new(self.clone())
    }
}

struct IceoryxPublishSample {
    publisher: Arc<iceoryx_rs::Publisher<[u8]>>,
    sample: iceoryx_rs::SampleMutArc<[u8]>,
}

impl crate::PublishSample for IceoryxPublishSample {
    fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.sample
    }

    fn publish(self: Box<Self>) -> Result<(), BoxError> {
        self.publisher.publish_arc(self.sample);
        Ok(())
    }
}

struct IceoryxReceiver {
    receiver: iceoryx_rs::mt::SampleReceiver<[u8]>,
}

impl Subscriber for IceoryxReceiver {
    fn recv(&mut self) -> Result<Option<Box<dyn ReceivedSample>>, BoxError> {
        self.receiver
            .wait_for_samples(Duration::from_secs(u64::MAX));
        match self.receiver.take() {
            Some(sample) => Ok(Some(Box::new(IceoryxReceivedSample { sample }))),
            None => Ok(None),
        }
    }
}

struct IceoryxReceivedSample {
    sample: iceoryx_rs::Sample<[u8], iceoryx_sys::SubscriberArc>,
}

impl ReceivedSample for IceoryxReceivedSample {
    fn get(&self) -> Cow<[u8]> {
        Cow::Borrowed(&self.sample)
    }
}
