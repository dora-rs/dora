use async_trait::async_trait;
use futures::StreamExt;
use futures_time::future::FutureExt;
use std::pin::Pin;
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

    async fn close(self: Box<Self>) -> Result<(), BoxError>;
}

#[async_trait]
impl CommunicationLayer for zenoh::Session {
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>>, BoxError> {
        zenoh::Session::subscribe(self, topic)
            .reliable()
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

    async fn close(self: Box<Self>) -> Result<(), BoxError> {
        zenoh::Session::close(*self)
            .await
            .map_err(BoxError)
    }
}
