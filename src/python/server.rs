use crate::zenoh_client::ZenohClient;

use super::binding::python_compute_event_loop;
use eyre::Result;
use opentelemetry::Context;
use serde::Deserialize;
use std::collections::BTreeMap;
use std::sync::{Arc, RwLock};
use structopt::StructOpt;
use tokio::sync::mpsc;

#[derive(Deserialize, Debug, Clone, StructOpt)]
pub struct PythonCommand {
    pub app: String,
    pub function: String,
    subscriptions: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct Workload {
    pub states: Arc<RwLock<BTreeMap<String, Vec<u8>>>>,
    pub pulled_states: Option<BTreeMap<String, Vec<u8>>>,
    pub otel_context: Context,
    pub degree: u32,
}

pub struct BatchMessages {
    pub outputs: BTreeMap<String, Vec<u8>>,
    pub deadlines: u64,
    pub otel_context: Context,
    pub degree: u32,
}

#[tokio::main]
pub async fn run(variables: PythonCommand) -> Result<()> {
    let (push_sender, push_receiver) = mpsc::channel::<BatchMessages>(1);
    let (python_sender, python_receiver) = mpsc::channel::<Workload>(1);

    let app_function_name = format!("{}-{}", &variables.app, &variables.function);

    let zenoh_client =
        ZenohClient::try_new(variables.subscriptions.clone(), app_function_name).await?;
    zenoh_client.clone().push_event_loop(push_receiver);
    python_compute_event_loop(python_receiver, push_sender, variables);
    zenoh_client.pull_event_loop(python_sender).await
}
