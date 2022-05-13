use std::{
    collections::BTreeMap,
    sync::{Arc, RwLock},
    time::Duration,
};

use crate::message::{message_capnp, serialize_message};
#[cfg(feature = "opentelemetry_jaeger")]
use crate::tracing::{deserialize_context, serialize_context, tracing_init};
#[cfg(feature = "opentelemetry_jaeger")]
use opentelemetry::{
    global,
    sdk::{propagation::TraceContextPropagator, trace as sdktrace},
    trace::{TraceContextExt, TraceError, Tracer},
    Context, KeyValue,
};

use eyre::Result;
use futures::{future::join_all, prelude::*};
use log::warn;
use tokio::{
    sync::mpsc::{Receiver, Sender},
    time::timeout,
};
use zenoh::{config::Config, prelude::SplitBuffer};
use zenoh::{subscriber::SampleReceiver, Session};

use crate::python::server::{BatchMessages, Workload};

static PULL_WAIT_PERIOD: std::time::Duration = Duration::from_millis(100);
static QUEUE_WAIT_PERIOD: std::time::Duration = Duration::from_millis(50);
static PUSH_WAIT_PERIOD: std::time::Duration = Duration::from_millis(100);

#[derive(Clone, Debug)]
pub struct ZenohClient {
    session: Arc<Session>,
    name: String,
    states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
    subscriptions: Vec<String>,
}

impl ZenohClient {
    pub async fn try_new(subscriptions: Vec<String>, name: String) -> Result<Self> {
        let session = Arc::new(
            zenoh::open(Config::default())
                .await
                .or_else(|e| eyre::bail!("{name}, Error: {e}"))?,
        );
        let states = Arc::new(RwLock::new(BTreeMap::new()));

        Ok(ZenohClient {
            session,
            name,
            states,
            subscriptions,
        })
    }

    pub async fn push(&self, batch_messages: BatchMessages) {
        #[cfg(feature = "opentelemetry_jaeger")]
        let tracer = global::tracer("pusher");
        #[cfg(feature = "opentelemetry_jaeger")]
        let span = tracer.start_with_context("result-pushing", &batch_messages.otel_context);
        #[cfg(feature = "opentelemetry_jaeger")]
        let cx = Context::current_with_span(span);
        #[cfg(feature = "opentelemetry_jaeger")]
        let string_context = serialize_context(&cx);

        #[cfg(not(feature = "opentelemetry_jaeger"))]
        let string_context = "".to_string();

        join_all(batch_messages.outputs.iter().map(|(key, data)| {
            let buffer = serialize_message(data, &string_context, batch_messages.degree);
            self.session.put(key, buffer)
        }))
        .await;
        self.states.write().unwrap().extend(batch_messages.outputs);
    }

    pub async fn pull(&self, receivers: &mut [&mut SampleReceiver]) -> Option<Workload> {
        let fetched_data = join_all(
            receivers
                .iter_mut()
                .map(|reciever| timeout(PULL_WAIT_PERIOD, reciever.next())),
        )
        .await;

        let mut pulled_states = BTreeMap::new();
        let mut string_context = "".to_string();
        let mut max_depth = 0;

        for (result, subscription) in fetched_data.into_iter().zip(&self.subscriptions) {
            if let Ok(Some(data)) = result {
                let value = data.value.payload;
                let buffer = value.contiguous();
                let mut owned_buffer = &*buffer;
                let deserialized = capnp::serialize::read_message(
                    &mut owned_buffer,
                    capnp::message::ReaderOptions::new(),
                )
                .unwrap();
                let message = deserialized
                    .get_root::<message_capnp::message::Reader>()
                    .unwrap();
                let data = message.get_data().unwrap().to_vec();
                let metadata = message.get_metadata().unwrap();
                let depth = metadata.get_depth();
                if max_depth <= depth {
                    string_context = metadata.get_otel_context().unwrap().to_string();
                    max_depth = depth
                }

                pulled_states.insert(subscription.clone().to_string(), data);
            }
        }

        #[cfg(feature = "opentelemetry_jaeger")]
        let cx = deserialize_context(string_context.as_str());
        #[cfg(not(feature = "opentelemetry_jaeger"))]
        let cx = string_context;

        if pulled_states.is_empty() {
            None
        } else {
            Some(Workload {
                pulled_states: Some(pulled_states),
                states: self.states.clone(),
                otel_context: cx, // TODO: Better telemetry management
                degree: max_depth,
            })
        }
    }

    pub fn push_event_loop(self, mut receiver: Receiver<BatchMessages>) {
        tokio::spawn(async move {
            while let Some(batch_messages) = receiver.recv().await {
                self.push(batch_messages).await;
            }
        });
    }

    pub async fn pull_event_loop(self, sender: Sender<Workload>) -> eyre::Result<()> {
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
                let states = self.states.clone();
                #[cfg(feature = "opentelemetry_jaeger")]
                let span = tracer.start(format!("{name}-pushing"));
                #[cfg(feature = "opentelemetry_jaeger")]
                let cx = Context::current_with_span(span);
                #[cfg(not(feature = "opentelemetry_jaeger"))]
                let cx = "".to_string();

                let sent_workload = sender.send_timeout(
                    Workload {
                        states,
                        pulled_states: None,
                        otel_context: cx,
                        degree: 0,
                    },
                    QUEUE_WAIT_PERIOD,
                );

                if let Err(err) = sent_workload.await {
                    let context = &self.name;
                    warn!("{context}, Sending Error: {err}");
                }
                tokio::time::sleep(PUSH_WAIT_PERIOD).await;
            }
        } else {
            loop {
                if let Some(workload) = self.pull(&mut receivers).await {
                    #[cfg(feature = "opentelemetry_jaeger")]
                    let span = tracer
                        .start_with_context(format!("{name}-pulling"), &workload.otel_context);
                    #[cfg(feature = "opentelemetry_jaeger")]
                    let cx = Context::current_with_span(span);

                    let sent_workload = sender.send_timeout(workload, QUEUE_WAIT_PERIOD);
                    if let Err(err) = sent_workload.await {
                        let context = &self.name;
                        warn!("{context}, Sending Error: {err}");
                        #[cfg(feature = "opentelemetry_jaeger")]
                        cx.span().add_event(
                            "Sending Error",
                            vec![KeyValue::new("err", format!("{err}"))],
                        )
                    }
                }
            }
        }
    }
}
