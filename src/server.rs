use eyre::{eyre, Context};
use futures::future::join_all;
use futures::stream::Next;
use futures::{join, prelude::*};
use pyo3::prelude::*;
use std::collections::hash_map::DefaultHasher;
use std::collections::{BTreeMap, HashMap};
use std::hash::Hash;
use std::hash::Hasher;
use std::time::{Duration, Instant};
use tokio::time::timeout;
use zenoh::config::Config;
use zenoh::prelude::SplitBuffer;
static DURATION_MILLIS: u64 = 1;
use serde::Deserialize;

#[derive(Deserialize, Debug)]
struct ConfigVariables {
    subscriptions: Vec<String>,
    app: String,
    function: String,
}

#[pyo3_asyncio::tokio::main]
pub async fn main() -> PyResult<()> {
    // Subscribe
    let variables = envy::from_env::<ConfigVariables>().unwrap();

    env_logger::init();
    let config = Config::default();
    let session = zenoh::open(config).await.unwrap();

    // Create a hashmap of all subscriptions.
    let mut subscribers = HashMap::new();
    let subs = variables.subscriptions.clone();

    for subscription in &subs {
        subscribers.insert(subscription.clone(), session
            .subscribe(subscription)
            .await
            .map_err(|err| {
                eyre!("Could not subscribe to the given subscription key expression. Error: {err}")
            })
            .unwrap());
    }

    // Store the latest value of all subscription as well as the output of the function. hash the state to easily check if the state has changed.
    let mut states = BTreeMap::new();
    let mut hasher = DefaultHasher::new();
    states.hash(&mut hasher);
    let mut state_hash = hasher.finish();

    let py_function = initialize(variables.app, variables.function).unwrap();
    let dur = Duration::from_millis(DURATION_MILLIS);
    let mut futures_put = vec![];

    loop {
        let now = Instant::now();
        let mut futures = vec![];
        for (_, v) in subscribers.iter_mut() {
            futures.push(timeout(dur, v.next()));
        }

        let results = join_all(futures).await;

        for (result, subscription) in results.into_iter().zip(&subs) {
            if let Ok(Some(data)) = result {
                let value = data.value.payload;
                let binary = value.contiguous();
                states.insert(
                    subscription.clone().to_string(),
                    String::from_utf8(binary.to_vec()).unwrap(),
                );
            }
        }

        let mut hasher = DefaultHasher::new();
        states.hash(&mut hasher);
        let new_hash = hasher.finish();
        if state_hash == new_hash {
            continue;
        }

        let result = Python::with_gil(|py| {
            let args = (states.clone().into_py(py),);
            pyo3_asyncio::tokio::into_future(
                py_function
                    .call(py, args, None)
                    .wrap_err("The Python function call did not succeed.")
                    .unwrap()
                    .as_ref(py),
            )
        })
        .wrap_err("Could not create future of python function call.")
        .unwrap()
        .await
        .wrap_err("Could not await the python future.")
        .unwrap();

        let outputs: HashMap<String, String> = Python::with_gil(|py| result.extract(py))
            .wrap_err("Could not retrieve the python result.")
            .unwrap();

        for (key, value) in outputs {
            states.insert(key.clone(), value.clone());
            futures_put.push(timeout(dur, session.put(key, value)));
        }

        let mut hasher = DefaultHasher::new();
        states.hash(&mut hasher);
        state_hash = hasher.finish();
        println!("loop {:#?}", now.elapsed());
    }
}

pub fn initialize(file: String, app: String) -> eyre::Result<Py<PyAny>> {
    Ok(Python::with_gil(|py| {
        let file = py
            .import(&file)
            .wrap_err("The import file was not found. Check your PYTHONPATH env variable.")
            .unwrap();
        // convert Function into a PyObject
        let identity = file
            .getattr(app)
            .wrap_err("The Function was not found in the imported file.")
            .unwrap();
        identity.to_object(py)
    }))
}
