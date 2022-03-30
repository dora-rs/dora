use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
    time::Duration,
};

use eyre::eyre;
use futures::{future::join_all, prelude::*};
use tokio::{
    sync::{
        mpsc::{Receiver, Sender},
        RwLock,
    },
    time::timeout,
};
use zenoh::prelude::SplitBuffer;
use zenoh::{subscriber::SampleReceiver, Session};

use crate::python::server::Workload;

static PULL_WAIT_PERIOD: std::time::Duration = Duration::from_millis(5);
static PUSH_WAIT_PERIOD: std::time::Duration = Duration::from_millis(40);

pub async fn pull(
    receivers: &mut Vec<&mut SampleReceiver>,
    subs: &[String],
) -> Option<BTreeMap<String, Vec<u8>>> {
    let fetched_data = join_all(
        receivers
            .iter_mut()
            .map(|reciever| timeout(PULL_WAIT_PERIOD, reciever.next())),
    )
    .await;

    let mut pulled_states = BTreeMap::new();
    for (result, subscription) in fetched_data.into_iter().zip(subs) {
        if let Ok(Some(data)) = result {
            let value = data.value.payload;
            let binary = value.contiguous();
            pulled_states.insert(subscription.clone().to_string(), binary.to_vec());
        }
    }
    if pulled_states.is_empty() {
        None
    } else {
        Some(pulled_states)
    }
}

pub async fn push(
    session: Arc<Session>,
    states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
    outputs: HashMap<String, Vec<u8>>,
) {
    join_all(
        outputs
            .iter()
            .map(|(key, value)| session.put(key, value.clone())),
    )
    .await;
    states.write().await.extend(outputs);
}

pub fn push_event_loop(
    mut receiver: Receiver<HashMap<String, Vec<u8>>>,
    session: Arc<Session>,
    states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
) {
    let session_push = session.clone();
    let states_push = states.clone();
    tokio::spawn(async move {
        while let Some(outputs) = receiver.recv().await {
            let states = states_push.clone();
            let session = session_push.clone();
            push(session, states, outputs).await;
        }
    });
}

pub async fn pull_event_loop(
    sender: Sender<Workload>,
    subscriptions: &[String],
    session: Arc<Session>,
    states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
) -> eyre::Result<()> {
    let mut subscribers = Vec::new();
    for subscription in subscriptions.iter() {
        subscribers.push(session.subscribe(subscription).await.map_err(|err| {
            eyre!("Could not subscribe to the given subscription key expression. Error: {err}")
        })?);
    }
    let mut receivers: Vec<_> = subscribers.iter_mut().map(|sub| sub.receiver()).collect();
    let is_source = subscriptions.is_empty();
    if is_source {
        loop {
            let states = states.clone();
            if let Err(err) = sender
                .send(Workload {
                    states,
                    pulled_states: None,
                })
                .await
            {
                eyre!("{err}");
            }
            tokio::time::sleep(PUSH_WAIT_PERIOD).await;
        }
    } else {
        loop {
            if let Some(pulled_states) = pull(&mut receivers, &subscriptions).await {
                let states = states.clone();
                if let Err(err) = sender
                    .send(Workload {
                        states,
                        pulled_states: Some(pulled_states),
                    })
                    .await
                {
                    eyre!("{err}");
                }
            }
        }
    }
}
