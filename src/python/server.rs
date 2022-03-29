use super::binding;
use eyre::eyre;
use eyre::Result;
use eyre::WrapErr;
use futures::future::join_all;
use futures::prelude::*;
use serde::Deserialize;
use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use std::time::Duration;
use structopt::StructOpt;
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

#[tokio::main]
pub async fn run(variables: PythonCommand) -> Result<()> {
    // Store the latest value of all subscription as well as the output of the function.
    let states = Arc::new(RwLock::new(BTreeMap::new()));
    let pool = Arc::new(
        rayon::ThreadPoolBuilder::new()
            .num_threads(8)
            .build()
            .unwrap(),
    );
    let session = Arc::new(zenoh::open(Config::default()).await.unwrap());
    let py_function = Arc::new(
        binding::init(&variables.app, &variables.function)
            .wrap_err("Failed to init the Python Function")
            .unwrap(),
    );

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
        let pool = pool.clone();
        let session = session.clone();
        let pyfunc = py_function.clone();

        let pulled_states = if !is_source {
            let pulled_states = pull(&mut receivers, &variables.subscriptions).await;

            if pulled_states.is_empty() {
                continue;
            }
            Some(pulled_states)
        } else {
            None
        };

        let states = states.clone();
        tokio::spawn(async move {
            let states_read = states.read().await;
            let outputs = pool.install(move || {
                Some(
                    binding::call(pyfunc, &states_read, &pulled_states)
                        .wrap_err("Python binding call did not work")
                        .unwrap(),
                )
            });
            if let Some(outputs) = outputs {
                push(session, states, outputs).await;
            }
        });

        if is_source {
            tokio::time::sleep(PUSH_WAIT_PERIOD).await;
        }
    }
}

async fn pull(
    receivers: &mut Vec<&mut SampleReceiver>,
    subs: &[String],
) -> BTreeMap<String, Vec<u8>> {
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
    pulled_states
}

async fn push(
    session: Arc<Session>,
    states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
    outputs: HashMap<String, Vec<u8>>,
) {
    let mut futures = vec![];
    let mut states = states.write().await;
    for (key, value) in outputs {
        states.insert(key.clone(), value.clone());
        futures.push(session.put(key, value));
    }
    join_all(futures).await;
}
