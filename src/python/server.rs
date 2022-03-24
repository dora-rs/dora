use super::binding;
use eyre::eyre;
use eyre::WrapErr;
use futures::future::join_all;
use futures::lock::Mutex;
use futures::prelude::*;
use pyo3::prelude::*;
use serde::Deserialize;
use std::collections::BTreeMap;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use structopt::StructOpt;
use tokio::time::error::Elapsed;
use tokio::time::timeout;
use zenoh::config::Config;
use zenoh::prelude::Sample;
use zenoh::prelude::SplitBuffer;
use zenoh::Session;

static DURATION_MILLIS: u64 = 1;

#[derive(Deserialize, Debug, Clone, StructOpt)]
pub struct PythonCommand {
    pub app: String,
    pub function: String,
    pub subscriptions: Vec<String>,
}

#[tokio::main]
pub async fn run(variables: PythonCommand) -> PyResult<()> {
    let duration = Duration::from_millis(DURATION_MILLIS);
    // Subscribe
    // Create a hashmap of all subscriptions.

    // Store the latest value of all subscription as well as the output of the function. hash the state to easily check if the state has changed.
    let states = Arc::new(Mutex::new(BTreeMap::new()));

    let py_function = Arc::new(
        binding::init(&variables.app, &variables.function)
            .wrap_err("Failed to init the Python Function")
            .unwrap(),
    );

    let session = Arc::new(zenoh::open(Config::default()).await.unwrap());
    let is_source = variables.subscriptions.is_empty();

    let mut subscribers = Vec::new();
    for subscription in variables.subscriptions.iter() {
        subscribers.push( session
            .subscribe(subscription)
            .await
            .map_err(|err| {
                eyre!("Could not subscribe to the given subscription key expression. Error: {err}")
            })
            .unwrap());
    }
    let mut receivers: Vec<_> = subscribers.iter_mut().map(|sub| sub.receiver()).collect();

    loop {
        let subs = variables.subscriptions.clone();
        let mut states_copy = (*states.lock().await).clone();
        let states = states.clone();
        let session = session.clone();
        let pyfunc = py_function.clone();
        let inputs = join_all(
            receivers
                .iter_mut()
                .map(|reciever| timeout(duration, reciever.next())),
        )
        .await;

        tokio::task::spawn_blocking(move || {
            let mutation = if !is_source {
                update(inputs, &subs, &mut states_copy)
            } else {
                true
            };

            if mutation {
                {
                    let loop_start = Instant::now();

                    let outputs = binding::call(pyfunc, states_copy)
                        .wrap_err("Python binding call did not work")
                        .unwrap();
                    println!("loop {:#?}", loop_start.elapsed());

                    push(session, states, outputs);
                }
            }
        });
        if is_source {
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }
}

fn update(
    results: Vec<Result<Option<Sample>, Elapsed>>,
    subs: &Vec<String>,
    states_copy: &mut BTreeMap<String, Vec<u8>>,
) -> bool {
    let mut mutation = false;

    for (result, subscription) in results.into_iter().zip(subs) {
        if let Ok(Some(data)) = result {
            let value = data.value.payload;
            let binary = value.contiguous();
            states_copy.insert(subscription.clone().to_string(), binary.to_vec());
            mutation = true;
        }
    }
    mutation
}

fn push(
    session: Arc<Session>,
    states: Arc<Mutex<BTreeMap<String, Vec<u8>>>>,
    outputs: HashMap<String, Vec<u8>>,
) {
    tokio::spawn(async move {
        let mut futures = vec![];
        let mut states = states.lock().await;
        for (key, value) in outputs {
            states.insert(key.clone(), value.clone());
            futures.push(session.put(key, value));
        }
        join_all(futures).await;
    });
}
