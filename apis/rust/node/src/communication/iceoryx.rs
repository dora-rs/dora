use eyre::Context;
use std::{collections::HashMap, sync::Arc, time::Duration};
use uuid::Uuid;

use crate::BoxError;

use super::{CommunicationLayer, Publisher, Subscriber};

pub struct IceoryxCommunicationLayer {
    topic_prefix: Arc<String>,
    publishers: HashMap<String, Arc<iceoryx_rs::Publisher<[u8]>>>,
}

impl IceoryxCommunicationLayer {
    pub fn init(app_name_prefix: String, topic_prefix: String) -> eyre::Result<Self> {
        let app_name = format!("{app_name_prefix}-{}", Uuid::new_v4());
        iceoryx_rs::Runtime::init(&app_name);

        Ok(Self {
            topic_prefix: Arc::new(topic_prefix.clone()),
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
                let publisher = Self::create_publisher(&self.topic_prefix, topic)
                    .context("failed to create iceoryx publisher")?;

                let publisher = Arc::new(publisher);
                self.publishers.insert(topic.to_owned(), publisher.clone());
                Ok(publisher)
            }
        }
    }

    fn create_publisher(
        topic_prefix: &str,
        topic: &str,
    ) -> Result<iceoryx_rs::Publisher<[u8]>, iceoryx_rs::IceoryxError> {
        iceoryx_rs::PublisherBuilder::new("dora", &topic_prefix, &topic).create()
    }
}

impl CommunicationLayer for IceoryxCommunicationLayer {
    fn publisher(&mut self, topic: &str) -> Result<Box<dyn Publisher>, crate::BoxError> {
        let publisher = self
            .get_or_create_publisher(topic)
            .map_err(|err| BoxError(err.into()))?;

        Ok(Box::new(IceoryxPublisher { publisher }))
    }

    fn subscribe(&mut self, topic: &str) -> Result<Box<dyn Subscriber>, crate::BoxError> {
        let (subscriber, token) =
            iceoryx_rs::SubscriberBuilder::new("dora", &self.topic_prefix, topic)
                .queue_capacity(5)
                .create_mt()
                .context("failed to create iceoryx subscriber")
                .map_err(|err| BoxError(err.into()))?;
        let receiver = subscriber.get_sample_receiver(token);

        Ok(Box::new(IceoryxReceiver { receiver }))
    }
}

#[derive(Clone)]
pub struct IceoryxPublisher {
    publisher: Arc<iceoryx_rs::Publisher<[u8]>>,
}

impl Publisher for IceoryxPublisher {
    fn publish(&self, data: &[u8]) -> Result<(), crate::BoxError> {
        let mut sample = self
            .publisher
            .loan_slice(data.len())
            .context("failed to loan iceoryx slice for publishing")
            .map_err(|err| BoxError(err.into()))?;
        sample.copy_from_slice(data);
        self.publisher.publish(sample);
        Ok(())
    }

    fn boxed_clone(&self) -> Box<dyn Publisher> {
        Box::new(self.clone())
    }
}

pub struct IceoryxReceiver {
    receiver: iceoryx_rs::mt::SampleReceiver<[u8]>,
}

impl Subscriber for IceoryxReceiver {
    fn recv(&mut self) -> Result<Option<Vec<u8>>, crate::BoxError> {
        self.receiver
            .wait_for_samples(Duration::from_secs(u64::MAX));
        match self.receiver.take() {
            Some(sample) => Ok(Some(sample.to_owned())),
            None => Ok(None),
        }
    }
}
