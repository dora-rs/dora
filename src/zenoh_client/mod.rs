use std::{collections::BTreeMap, sync::Arc, time::Duration};

use opentelemetry::{
    sdk::{propagation::TraceContextPropagator, trace as sdktrace},
    trace::{FutureExt, TraceError},
};

use eyre::Result;
use futures::{future::join_all, prelude::*};
use log::warn;
use opentelemetry::{
    global,
    trace::{TraceContextExt, Tracer},
    Context,
};
use tokio::{
    sync::{
        mpsc::{Receiver, Sender},
        RwLock,
    },
    time::timeout,
};
use zenoh::{config::Config, prelude::SplitBuffer};
use zenoh::{subscriber::SampleReceiver, Session};

use crate::{
    message::{deserialize_context, message_capnp, serialize_context, serialize_message},
    python::server::{BatchMessages, Workload},
};

static PULL_WAIT_PERIOD: std::time::Duration = Duration::from_millis(100);
static PUSH_WAIT_PERIOD: std::time::Duration = Duration::from_millis(100);

#[derive(Clone, Debug)]
pub struct ZenohClient {
    session: Arc<Session>,
    context: String,
    states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
    subscriptions: Vec<String>,
}

impl ZenohClient {
    pub async fn try_new(subscriptions: Vec<String>, context: String) -> Result<Self> {
        let session = Arc::new(
            zenoh::open(Config::default())
                .await
                .or_else(|e| eyre::bail!("{context}, Error: {e}"))?,
        );
        let states = Arc::new(RwLock::new(BTreeMap::new()));

        Ok(ZenohClient {
            session,
            context,
            states,
            subscriptions,
        })
    }

    pub async fn push(&self, batch_messages: BatchMessages) {
        let tracer = global::tracer("pusher");
        let span = tracer.start_with_context("result-pushing", &batch_messages.otel_context);
        let cx = Context::current_with_span(span);
        let string_context = serialize_context(&cx);
        join_all(batch_messages.outputs.iter().map(|(key, data)| {
            let buffer = serialize_message(data, &string_context);
            self.session.put(key, buffer)
        }))
        .with_context(cx)
        .await;
        self.states.write().await.extend(batch_messages.outputs);
    }

    pub async fn pull(&self, receivers: &mut Vec<&mut SampleReceiver>) -> Option<Workload> {
        let fetched_data = join_all(
            receivers
                .iter_mut()
                .map(|reciever| timeout(PULL_WAIT_PERIOD, reciever.next())),
        )
        .await;

        let mut pulled_states = BTreeMap::new();
        let mut otel_contexts = vec![];
        for (result, subscription) in fetched_data.into_iter().zip(&self.subscriptions) {
            if let Ok(Some(data)) = result {
                let value = data.value.payload;
                let buffer = value.contiguous();
                let owned_buffer = buffer.into_owned();
                let deserialized = capnp::serialize::read_message(
                    &mut owned_buffer.as_slice(),
                    capnp::message::ReaderOptions::new(),
                )
                .unwrap();
                let message = deserialized
                    .get_root::<message_capnp::message::Reader>()
                    .unwrap();
                let data = message.get_data().unwrap().to_vec();
                let string_context = message.get_metadata().unwrap().get_otel_context().unwrap();
                let cx = deserialize_context(string_context);
                otel_contexts.push(cx);

                pulled_states.insert(subscription.clone().to_string(), data);
            }
        }
        if pulled_states.is_empty() {
            None
        } else {
            Some(Workload {
                pulled_states: Some(pulled_states),
                states: self.states.clone(),
                otel_context: otel_contexts.pop().unwrap(), // TODO: Better telemetry management
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
        let tracer = tracing_init()?;
        let name = &self.context;
        //let span = tracer.start("client-root");
        //let cx = Context::current_with_span(span);
        if is_source {
            loop {
                let states = self.states.clone();
                let span = tracer.start(format!("{name}-pushing"));
                let cx = Context::current_with_span(span);
                if let Err(err) = timeout(
                    PULL_WAIT_PERIOD,
                    sender.send(Workload {
                        states,
                        pulled_states: None,
                        otel_context: cx,
                    }),
                )
                .await
                {
                    let context = &self.context;
                    warn!("{context}, Sending Error: {err}");
                }
                tokio::time::sleep(PUSH_WAIT_PERIOD).await;
            }
        } else {
            loop {
                if let Some(workload) = self.pull(&mut receivers).await {
                    if let Err(err) = timeout(PULL_WAIT_PERIOD, sender.send(workload)).await {
                        let context = &self.context;
                        warn!("{context}, Sending Error: {err}");
                    }
                }
            }
        }
    }
}

fn tracing_init() -> Result<sdktrace::Tracer, TraceError> {
    global::set_text_map_propagator(TraceContextPropagator::new());
    opentelemetry_jaeger::new_agent_pipeline()
        .with_endpoint("172.17.0.1:6831")
        .with_service_name("test-client")
        .install_batch(opentelemetry::runtime::Tokio)
}
