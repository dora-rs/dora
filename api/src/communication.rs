use async_trait::async_trait;
use futures::StreamExt;
use std::pin::Pin;
use zenoh::prelude::{SplitBuffer, ZFuture};

use crate::BoxError;

#[async_trait]
pub(crate) trait CommunicationLayer {
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>>, BoxError>;

    async fn publish(&self, topic: &str, data: &[u8]) -> Result<(), BoxError>;

    fn publish_sync(&self, topic: &str, data: &[u8]) -> Result<(), BoxError>;
}

#[async_trait]
impl CommunicationLayer for zenoh::Session {
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>>, BoxError> {
        self.subscribe(topic)
            .await
            .map(|s| {
                let trait_object: Pin<Box<dyn futures::Stream<Item = Vec<u8>> + 'a>> =
                    Box::pin(s.map(|s| s.value.payload.contiguous().into_owned()));
                trait_object
            })
            .map_err(BoxError)
    }

    async fn publish(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        self.put(topic, data).await.map_err(BoxError)
    }

    fn publish_sync(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        self.put(topic, data).wait().map_err(BoxError)
    }
}
