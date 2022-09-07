use super::{CommunicationLayer, Publisher, Subscriber};
use crate::BoxError;
use std::{sync::Arc, time::Duration};
use zenoh::{
    prelude::{EntityFactory, Priority, Receiver as _, SplitBuffer, ZFuture},
    publication::CongestionControl,
};

pub struct ZenohCommunicationLayer {
    zenoh: Arc<zenoh::Session>,
    topic_prefix: String,
}

impl ZenohCommunicationLayer {
    pub fn init(config: zenoh_config::Config, prefix: String) -> eyre::Result<Self> {
        let zenoh = ::zenoh::open(config)
            .wait()
            .map_err(|err| BoxError(err.into()))?
            .into_arc();
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
    fn publisher(&mut self, topic: &str) -> Result<Box<dyn Publisher>, crate::BoxError> {
        let publisher = self
            .zenoh
            .publish(self.prefixed(topic))
            .congestion_control(CongestionControl::Block)
            .priority(Priority::RealTime)
            .wait()
            .map_err(|err| BoxError(err.into()))?;

        Ok(Box::new(ZenohPublisher { publisher }))
    }

    fn subscribe(&mut self, topic: &str) -> Result<Box<dyn Subscriber>, BoxError> {
        let subscriber = self
            .zenoh
            .subscribe(self.prefixed(topic))
            .reliable()
            .wait()
            .map_err(|err| BoxError(err.into()))?;

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
pub struct ZenohPublisher {
    publisher: zenoh::publication::Publisher<'static>,
}

impl Publisher for ZenohPublisher {
    fn publish(&self, data: &[u8]) -> Result<(), crate::BoxError> {
        self.publisher
            .send(data)
            .map_err(|err| BoxError(err.into()))
    }

    fn boxed_clone(&self) -> Box<dyn Publisher> {
        Box::new(self.clone())
    }
}

pub struct ZenohReceiver(zenoh::subscriber::Subscriber<'static>);

impl Subscriber for ZenohReceiver {
    fn recv(&mut self) -> Result<Option<Vec<u8>>, crate::BoxError> {
        match self.0.recv() {
            Ok(sample) => Ok(Some(sample.value.payload.contiguous().into_owned())),
            Err(flume::RecvError::Disconnected) => Ok(None),
        }
    }
}
