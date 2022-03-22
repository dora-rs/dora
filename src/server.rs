use eyre::{eyre, Context};
use futures::prelude::*;
use pyo3::prelude::*;
use std::collections::hash_map::DefaultHasher;
use std::collections::{BTreeMap, HashMap};
use std::env;
use std::hash::Hash;
use std::hash::Hasher;
use std::time::Duration;
use tokio::time::timeout;
use zenoh::config::Config;
use zenoh::prelude::SplitBuffer;
static DURATION_MILLIS: u64 = 5;

#[pyo3_asyncio::tokio::main]
pub async fn main() -> PyResult<()> {
    // Subscribe
    env_logger::init();
    let config = Config::default();
    let session = zenoh::open(config).await.unwrap();
    let subscriptions = env::var("SRC_LABELS")
        .wrap_err("Env variable not set")
        .unwrap();
    let subscriptions = subscriptions.split(":");

    let mut subscribers = HashMap::new();

    for subscription in subscriptions {
        subscribers.insert(subscription, session
            .subscribe(subscription)
            .await
            .map_err(|err| {
                eyre!("Could not subscribe to the given subscription key expression. Error: {err}")
            })
            .unwrap());
    }

    let mut states = BTreeMap::new();
    let mut hasher = DefaultHasher::new();
    states.hash(&mut hasher);
    let mut state_hash = hasher.finish();

    let identity = initialize("app".to_string(), "return_1".to_string()).unwrap();
    let dur = Duration::from_millis(DURATION_MILLIS);

    loop {
        for (subscription, subscriber) in subscribers.iter_mut() {
            let result = timeout(dur, subscriber.next()).await;
            if let Ok(Some(data)) = result {
                let value = data.value.payload;
                let binary = value.contiguous();
                states.insert(subscription.clone(), binary.to_vec());
            }
        }

        let mut hasher = DefaultHasher::new();
        states.hash(&mut hasher);

        if state_hash == hasher.finish() {
            continue;
        } else {
            state_hash = hasher.finish();
        }

        let result = Python::with_gil(|py| {
            let args = (states.clone().into_py(py),);
            pyo3_asyncio::tokio::into_future(
                identity
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
            session
                .put(key, value)
                .await
                .map_err(|err| {
                    eyre!("Could not put the output within the chosen key expression topic. Error: {err}")
                })
                .unwrap();
        }
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
