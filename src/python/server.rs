use crate::zenoh_client::pull_event_loop;
use crate::zenoh_client::push_event_loop;

use super::binding::python_compute_event_loop;
use eyre::eyre;
use eyre::Result;
use serde::Deserialize;
use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;
use structopt::StructOpt;
use tokio::sync::mpsc;
use tokio::sync::RwLock;
use zenoh::config::Config;

#[derive(Deserialize, Debug, Clone, StructOpt)]
pub struct PythonCommand {
    pub app: String,
    pub function: String,
    pub subscriptions: Vec<String>,
}

pub struct Workload {
    pub states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
    pub pulled_states: Option<BTreeMap<String, Vec<u8>>>,
}

#[tokio::main]
pub async fn run(variables: PythonCommand) -> Result<()> {
    // Store the latest value of all subscription as well as the output of the function.
    //  let states = Arc::new(RwLock::new(BTreeMap::new()));
    let (push_sender, push_receiver) = mpsc::channel::<HashMap<String, Vec<u8>>>(8);
    let (python_sender, python_receiver) = mpsc::channel::<Workload>(8);
    // Push Event Loop
    let session = Arc::new(
        zenoh::open(Config::default())
            .await
            .map_err(|e| eyre!("{e}"))?,
    );
    let states = Arc::new(RwLock::new(BTreeMap::new()));
    let subscriptions = variables.subscriptions.clone();

    python_compute_event_loop(python_receiver, push_sender, variables);
    push_event_loop(push_receiver, session.clone(), states.clone());
    pull_event_loop(python_sender, &subscriptions, session, states).await?;

    Ok(())
}
