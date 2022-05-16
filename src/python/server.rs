use super::binding::python_compute_event_loop;
use crate::zenoh_client::ZenohClient;
use eyre::Result;
#[cfg(feature = "opentelemetry_jaeger")]
use opentelemetry::Context;
use serde::Deserialize;
use std::collections::BTreeMap;
use structopt::StructOpt;
use tokio::sync::mpsc;
use zenoh::buf::ZBuf;

#[derive(Deserialize, Debug, Clone, StructOpt)]
pub struct PythonCommand {
    pub app: String,
    pub function: String,
    subscriptions: Vec<String>,
}

#[tokio::main]
pub async fn run(variables: PythonCommand) -> Result<()> {
    let (push_sender, push_receiver) = mpsc::channel::<BTreeMap<String, Vec<u8>>>(1);
    let (python_sender, python_receiver) = mpsc::channel::<BTreeMap<String, ZBuf>>(1);

    let app_function_name = format!("{}-{}", &variables.app, &variables.function);

    let zenoh_client =
        ZenohClient::try_new(variables.subscriptions.clone(), app_function_name).await?;
    zenoh_client.clone().push_event_loop(push_receiver);
    python_compute_event_loop(python_receiver, push_sender, variables);
    zenoh_client.pull_event_loop(python_sender).await
}
