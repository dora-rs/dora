use async_trait::async_trait;
use futures::StreamExt;
use paho_mqtt as mqtt;
use std::{collections::HashMap, pin::Pin};
use zenoh::{
    prelude::{Priority, SplitBuffer, ZFuture},
    publication::CongestionControl,
};

use crate::BoxError;

#[async_trait]
pub trait CommunicationLayer {
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>>, BoxError>;

    async fn publish(&self, topic: &str, data: &[u8]) -> Result<(), BoxError>;

    fn publish_sync(&self, topic: &str, data: &[u8]) -> Result<(), BoxError>;
}

#[async_trait]
impl CommunicationLayer
    for HashMap<
        String,
        (
            mqtt::AsyncClient,
            paho_mqtt::AsyncReceiver<std::option::Option<paho_mqtt::Message>>,
        ),
    >
{
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>>, BoxError> {
        let (_, stream) = self.get(topic).unwrap();
        Ok(Box::pin(
            stream.clone().map(|x| x.unwrap().payload().to_vec()),
        ))
    }

    async fn publish(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        let msg = mqtt::Message::new(topic, data, mqtt::QOS_1);
        self.get("publish").unwrap().0.publish(msg).await.unwrap();
        Ok(())
    }

    fn publish_sync(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        let msg = mqtt::Message::new(topic, data, mqtt::QOS_1);
        self.get("publish").unwrap().0.publish(msg).wait().unwrap();
        Ok(())
    }
}

#[async_trait]
impl CommunicationLayer for zenoh::Session {
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>>, BoxError> {
        zenoh::Session::subscribe(self, topic)
            .await
            .map(|s| {
                let trait_object: Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>> =
                    Box::pin(s.map(|s| s.value.payload.contiguous().into_owned()));
                trait_object
            })
            .map_err(BoxError)
    }

    async fn publish(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        let writer = self
            .put(topic, data)
            .congestion_control(CongestionControl::Block)
            .priority(Priority::RealTime);

        let result = writer.await.map_err(BoxError);
        result
    }

    fn publish_sync(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        let writer = self
            .put(topic, data)
            .congestion_control(CongestionControl::Block)
            .priority(Priority::RealTime);

        writer.wait().map_err(BoxError)
    }
}
