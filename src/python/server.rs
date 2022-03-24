use super::binding;
use eyre::eyre;
use eyre::WrapErr;
use futures::future::join_all;
use futures::lock::Mutex;
use futures::prelude::*;
use pyo3::prelude::*;
use serde::Deserialize;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use structopt::StructOpt;
use tokio::time::timeout;
use zenoh::config::Config;
use zenoh::prelude::SplitBuffer;

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

    let subs = variables.subscriptions.clone();
    let session = Arc::new(zenoh::open(Config::default()).await.unwrap());
    let mut subscribers = Vec::new();
    for subscription in subs {
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
        let states = states.clone();
        let subs = variables.subscriptions.clone();
        let mut states_copy = BTreeMap::new(); // (*states.lock().await).clone();
        let session = session.clone();
        let pyfunc = py_function.clone();
        let results = join_all(
            receivers
                .iter_mut()
                .map(|reciever| timeout(duration, reciever.next())),
        )
        .await;
        tokio::task::spawn_blocking(move || {
            let mut mutation = false;
            for (result, subscription) in results.into_iter().zip(&subs) {
                if let Ok(Some(data)) = result {
                    let value = data.value.payload;
                    let binary = value.contiguous();
                    states_copy.insert(
                        subscription.clone().to_string(),
                        String::from_utf8(binary.to_vec()).unwrap(),
                    );
                    mutation = true;
                }
            }

            if mutation {
                {
                    let loop_start = Instant::now();
                    let states = states.clone();

                    // let new_hash = hash(&states);

                    let outputs = binding::call(&pyfunc, states_copy)
                        .wrap_err("Python binding call did not work")
                        .unwrap();

                    tokio::spawn(async move {
                        let session = session.clone();
                        let mut futures = vec![];
                        let mut states = states.lock().await;
                        for (key, value) in outputs {
                            states.insert(key.clone(), value.clone());
                            futures.push(session.put(key, value));
                        }
                        join_all(futures).await;
                    });
                    println!("loop {:#?}", loop_start.elapsed());
                }
            }
        });
    }
}
