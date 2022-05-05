use std::{collections::BTreeMap, sync::Arc, time::Duration};

use eyre::Result;
use futures::{future::join_all, prelude::*};
use log::debug;
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
    message::{deserialize_message, serialize_message},
    python::server::Workload,
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

    pub async fn push(&self, outputs: BTreeMap<String, Vec<u8>>) {
        join_all(outputs.iter().map(|(key, data)| {
            let buffer = serialize_message(data);
            self.session.put(key, buffer)
        }))
        .await;
        self.states.write().await.extend(outputs);
    }

    pub async fn pull(
        &self,
        receivers: &mut [&mut SampleReceiver],
    ) -> Option<BTreeMap<String, Vec<u8>>> {
        let fetched_data = join_all(
            receivers
                .iter_mut()
                .map(|reciever| timeout(PULL_WAIT_PERIOD, reciever.next())),
        )
        .await;

        let mut pulled_states = BTreeMap::new();
        for (result, subscription) in fetched_data.into_iter().zip(&self.subscriptions) {
            if let Ok(Some(data)) = result {
                let value = data.value.payload;
                let buffer = value.contiguous();
                let owned_buffer = buffer.into_owned();
                let data = deserialize_message(owned_buffer);
                pulled_states.insert(subscription.clone().to_string(), data);
            }
        }
        if pulled_states.is_empty() {
            None
        } else {
            Some(pulled_states)
        }
    }
    pub fn push_event_loop(self, mut receiver: Receiver<BTreeMap<String, Vec<u8>>>) {
        tokio::spawn(async move {
            while let Some(outputs) = receiver.recv().await {
                self.push(outputs).await;
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
        if is_source {
            loop {
                let states = self.states.clone();
                if let Err(err) = timeout(
                    PULL_WAIT_PERIOD,
                    sender.send(Workload {
                        states,
                        pulled_states: None,
                    }),
                )
                .await
                {
                    let context = &self.context;
                    debug!("{context}, Sending Error: {err}");
                }
                tokio::time::sleep(PUSH_WAIT_PERIOD).await;
            }
        } else {
            loop {
                if let Some(pulled_states) = self.pull(&mut receivers).await {
                    let states = self.states.clone();
                    if let Err(err) = timeout(
                        PULL_WAIT_PERIOD,
                        sender.send(Workload {
                            states,
                            pulled_states: Some(pulled_states),
                        }),
                    )
                    .await
                    {
                        let context = &self.context;
                        debug!("{context}, Sending Error: {err}");
                    }
                }
            }
        }
    }
}
