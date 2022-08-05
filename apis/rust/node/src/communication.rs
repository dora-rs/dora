use async_trait::async_trait;
use eyre::Context;
use futures::StreamExt;
use futures_time::future::FutureExt;
use std::pin::Pin;
use zenoh::{
    prelude::{Priority, SplitBuffer, ZFuture},
    publication::CongestionControl,
};

use crate::{config::CommunicationConfig, BoxError};

pub async fn init(
    communication_config: &CommunicationConfig,
) -> eyre::Result<Box<dyn CommunicationLayer>> {
    match communication_config {
        CommunicationConfig::Zenoh {
            config: zenoh_config,
            prefix: zenoh_prefix,
        } => {
            let zenoh = zenoh::open(zenoh_config.clone())
                .await
                .map_err(BoxError)
                .wrap_err("failed to create zenoh session")?;
            let layer = ZenohCommunicationLayer {
                zenoh,
                topic_prefix: zenoh_prefix.clone(),
            };
            Ok(Box::new(layer))
        }
    }
}

#[async_trait]
pub trait CommunicationLayer: Send + Sync {
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + Send + 'a>>, BoxError>;

    async fn publish(&self, topic: &str, data: &[u8]) -> Result<(), BoxError>;

    fn publish_sync(&self, topic: &str, data: &[u8]) -> Result<(), BoxError>;

    async fn close(self: Box<Self>) -> Result<(), BoxError>;
}

struct ZenohCommunicationLayer {
    zenoh: zenoh::Session,
    topic_prefix: String,
}

impl ZenohCommunicationLayer {
    fn prefixed(&self, topic: &str) -> String {
        format!("{}/{topic}", self.topic_prefix)
    }
}

#[async_trait]
impl CommunicationLayer for ZenohCommunicationLayer {
    async fn subscribe<'a>(
        &'a self,
        topic: &str,
    ) -> Result<Pin<Box<dyn futures::Stream<Item = Vec<u8>> + Send + 'a>>, BoxError> {
        zenoh::Session::subscribe(&self.zenoh, self.prefixed(topic))
            .reliable()
            .await
            .map(|s| {
                let trait_object: Pin<Box<dyn futures::Stream<Item = Vec<u8>> + Send + 'a>> =
                    Box::pin(s.map(|s| s.value.payload.contiguous().into_owned()));
                trait_object
            })
            .map_err(BoxError)
    }

    async fn publish(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        let writer = self
            .zenoh
            .put(self.prefixed(topic), data)
            .congestion_control(CongestionControl::Block)
            .priority(Priority::RealTime);

        let result = writer.await.map_err(BoxError);
        result
    }

    fn publish_sync(&self, topic: &str, data: &[u8]) -> Result<(), BoxError> {
        let writer = self
            .zenoh
            .put(self.prefixed(topic), data)
            .congestion_control(CongestionControl::Block)
            .priority(Priority::RealTime);

        writer.wait().map_err(BoxError)
    }

    async fn close(self: Box<Self>) -> Result<(), BoxError> {
        zenoh::Session::close(self.zenoh)
            // wait a bit before closing to ensure that remaining published
            // messages are sent out
            //
            // TODO: create a minimal example to reproduce the dropped messages
            // and report this issue in the zenoh repo
            .delay(futures_time::time::Duration::from_secs_f32(0.5))
            .await
            .map_err(BoxError)
    }
}
