use crate::message::message_capnp;

#[cfg(feature = "opentelemetry_jaeger")]
use crate::tracing::{serialize_context, tracing_init};
#[cfg(feature = "opentelemetry_jaeger")]
use opentelemetry::{
    trace::{TraceContextExt, Tracer},
    Context,
};
use std::{collections::BTreeMap, sync::Arc, time::Duration};

use eyre::Result;
use futures::{future::join_all, prelude::*};
use log::warn;
use tokio::{
    sync::mpsc::{Receiver, Sender},
    time::timeout,
};
use zenoh::{buf::ZBuf, config::Config};
use zenoh::{subscriber::SampleReceiver, Session};

static PULL_WAIT_PERIOD: std::time::Duration = Duration::from_millis(100);
static QUEUE_WAIT_PERIOD: std::time::Duration = Duration::from_millis(50);
static PUSH_WAIT_PERIOD: std::time::Duration = Duration::from_millis(100);

#[derive(Clone, Debug)]
pub struct ZenohClient {
    session: Arc<Session>,
    name: String,
    subscriptions: Vec<String>,
}

impl ZenohClient {
    pub async fn try_new(subscriptions: Vec<String>, name: String) -> Result<Self> {
        let session = Arc::new(
            zenoh::open(Config::default())
                .await
                .or_else(|e| eyre::bail!("{name}, Error: {e}"))?,
        );

        Ok(ZenohClient {
            session,
            name,
            subscriptions,
        })
    }

    pub async fn push(&self, batch_messages: BTreeMap<String, Vec<u8>>) {
        join_all(
            batch_messages
                .iter()
                .map(|(key, data)| self.session.put(key, data.as_slice())),
        )
        .await;
    }

    pub async fn pull(&self, receivers: &mut [&mut SampleReceiver]) -> BTreeMap<String, ZBuf> {
        let fetched_data = join_all(
            receivers
                .iter_mut()
                .map(|receiver| timeout(PULL_WAIT_PERIOD, receiver.next())),
        )
        .await;

        let mut pulled_states = BTreeMap::new();

        for (result, subscription) in fetched_data.into_iter().zip(&self.subscriptions) {
            if let Ok(Some(data)) = result {
                let value = data.value.payload;

                pulled_states.insert(subscription.clone().to_string(), value);
            }
        }

        pulled_states
    }

    pub fn push_event_loop(self, mut receiver: Receiver<BTreeMap<String, Vec<u8>>>) {
        tokio::spawn(async move {
            while let Some(batch_messages) = receiver.recv().await {
                self.push(batch_messages).await;
            }
        });
    }

    pub async fn pull_event_loop(self, sender: Sender<BTreeMap<String, ZBuf>>) -> eyre::Result<()> {
        let mut subscribers = Vec::new();
        for subscription in self.subscriptions.iter() {
            subscribers.push(self.session.subscribe(subscription).await.or_else(|err| {
                eyre::bail!(
                    "Could not subscribe to the given subscription key expression. Error: {err}"
                )
            })?);
        }
        let mut receivers: Vec<_> = subscribers.iter_mut().map(|sub| sub.receiver()).collect();
        let is_source = self.subscriptions.is_empty();
        #[cfg(feature = "opentelemetry_jaeger")]
        let tracer = tracing_init()?;
        #[cfg(feature = "opentelemetry_jaeger")]
        let name = &self.name;

        if is_source {
            loop {
                #[cfg(feature = "opentelemetry_jaeger")]
                let span = tracer.start(format!("{name}-pushing"));
                #[cfg(feature = "opentelemetry_jaeger")]
                let cx = Context::current_with_span(span);
                #[cfg(not(feature = "opentelemetry_jaeger"))]
                let cx = "".to_string();

                let mut message = ::capnp::message::Builder::new_default();
                let mut metadata = message.init_root::<message_capnp::metadata::Builder>();
                #[cfg(not(feature = "opentelemetry_jaeger"))]
                metadata.set_otel_context(&cx);
                #[cfg(feature = "opentelemetry_jaeger")]
                metadata.set_otel_context(&serialize_context(&cx));

                let sent_workload = sender.send_timeout(BTreeMap::new(), QUEUE_WAIT_PERIOD);

                if let Err(err) = sent_workload.await {
                    let context = &self.name;
                    warn!("{context}, Sending Error: {err}");
                }
                tokio::time::sleep(PUSH_WAIT_PERIOD).await;
            }
        } else {
            loop {
                let workload = self.pull(&mut receivers).await;

                let sent_workload = sender.send_timeout(workload, QUEUE_WAIT_PERIOD);
                if let Err(err) = sent_workload.await {
                    let context = &self.name;
                    warn!("{context}, Sending Error: {err}");
                }
            }
        }
    }
}
