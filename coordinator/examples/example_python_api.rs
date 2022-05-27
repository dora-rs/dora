use dora_api::{self, config::DataId, DoraNode};
use eyre::Context;
use futures::StreamExt;
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};
use tokio::time::sleep;

static ATOMIC_TIMEOUT: Duration = Duration::from_millis(20);
static BATCH_TIMEOUT: Duration = Duration::from_millis(100);
use dora_rs::python::binding::PythonBinding;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let node = DoraNode::init_from_env().await?;

    let app_ = node.env;
    let func = std::env::var("DORA_NODE_PYTHON_FUNCTION")
        .wrap_err("env variable DORA_NODE_PYTHON_FUNCTION must be set")?;
    let is_source = std::env::var("IS_SOURCE").wrap_err("env variable IS_SOURCE must be set")?;

    let py_function = PythonBinding::try_new(&app, &func).unwrap();

    let mut inputs = node.inputs().await?;

    loop {
        let mut workload = BTreeMap::new();
        let time_at_start = Instant::now();
        let py_function = py_function.clone();

        // Retrieve several inputs within a time frame
        if is_source != "true" {
            while time_at_start.elapsed() < BATCH_TIMEOUT {
                let input = match tokio::time::timeout(ATOMIC_TIMEOUT, inputs.next()).await {
                    Ok(Some(input)) => input,
                    Ok(None) => continue,
                    Err(_) => continue,
                };
                workload.insert(input.id.to_string(), input.data);
            }
        } else {
            sleep(BATCH_TIMEOUT).await;
        }

        // Call the function
        let batch_messages = py_function.call(&workload).unwrap();

        // Send the data one by one.
        for (k, v) in batch_messages {
            node.send_output(&DataId::from(k), &v).await.unwrap();
        }
    }
}
