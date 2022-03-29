use super::binding;
use eyre::eyre;
use eyre::Context;
use eyre::Result;
use eyre::WrapErr;
use futures::future::join_all;
use futures::prelude::*;
use serde::Deserialize;
use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::time::Duration;
use structopt::StructOpt;
use tokio::sync::mpsc;
use tokio::sync::RwLock;
use tokio::time::timeout;
use zenoh::config::Config;
use zenoh::prelude::SplitBuffer;
use zenoh::subscriber::SampleReceiver;
use zenoh::Session;

static PULL_WAIT_PERIOD: std::time::Duration = Duration::from_millis(5);
static PUSH_WAIT_PERIOD: std::time::Duration = Duration::from_millis(40);

#[derive(Deserialize, Debug, Clone, StructOpt)]
pub struct PythonCommand {
    pub app: String,
    pub function: String,
    pub subscriptions: Vec<String>,
}

pub struct Workload {
    states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
    pulled_states: Option<BTreeMap<String, Vec<u8>>>,
}

#[tokio::main]
pub async fn run(variables: PythonCommand) -> Result<()> {
    // Store the latest value of all subscription as well as the output of the function.
    //  let states = Arc::new(RwLock::new(BTreeMap::new()));
    let (push_tx, mut push_rx) = mpsc::channel::<HashMap<String, Vec<u8>>>(8);
    let (rayon_tx, mut rayon_rx) = mpsc::channel::<Workload>(8);

    // Computation Event Loop
    tokio::spawn(async move {
        let py_function = Arc::new(
            binding::init(&variables.app, &variables.function)
                .context("Failed to init the Python Function")
                .unwrap(),
        );

        while let Some(workload) = rayon_rx.recv().await {
            let pyfunc = py_function.clone();
            let push_tx = push_tx.clone();
            let states = workload.states.read().await.clone(); // This is probably expensive.
            rayon::spawn(move || {
                push_tx
                    .blocking_send(
                        binding::call(pyfunc, &states, &workload.pulled_states)
                            .wrap_err("Python binding call did not work")
                            .unwrap(),
                    )
                    .unwrap();
            });
        }
    });

    // Push Event Loop
    let session = Arc::new(zenoh::open(Config::default()).await.unwrap());
    let states = Arc::new(RwLock::new(BTreeMap::new()));
    let session_push = session.clone();
    let states_push = states.clone();
    tokio::spawn(async move {
        while let Some(outputs) = push_rx.recv().await {
            let states = states_push.clone();
            let session = session_push.clone();
            push(session, states, outputs).await;
        }
    });

    // Pull Event Loop
    let mut subscribers = Vec::new();
    for subscription in variables.subscriptions.iter() {
        subscribers.push(session.subscribe(subscription).await.map_err(|err| {
            eyre!("Could not subscribe to the given subscription key expression. Error: {err}")
        })?);
    }
    let mut receivers: Vec<_> = subscribers.iter_mut().map(|sub| sub.receiver()).collect();
    let is_source = variables.subscriptions.is_empty();
    if is_source {
        loop {
            let states = states.clone();
            if let Err(_) = rayon_tx
                .send(Workload {
                    states,
                    pulled_states: None,
                })
                .await
            {
                println!("Dropped Messages")
            }
            tokio::time::sleep(PUSH_WAIT_PERIOD).await;
        }
    } else {
        loop {
            if let Some(pulled_states) = pull(&mut receivers, &variables.subscriptions).await {
                let states = states.clone();
                if let Err(_) = rayon_tx
                    .send(Workload {
                        states,
                        pulled_states: Some(pulled_states),
                    })
                    .await
                {
                    println!("Dropped Messages")
                }
            }
        }
    }
}

async fn pull(
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

async fn push(
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
