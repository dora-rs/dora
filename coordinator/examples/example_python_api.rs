use dora_api::{self, config::DataId, DoraNode};
use eyre::{bail, Context};
use futures::StreamExt;
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

static ATOMIC_TIMEOUT: Duration = Duration::from_millis(20);
static BATCH_TIMEOUT: Duration = Duration::from_millis(100);
use dora_rs::python::binding::PythonBinding;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let Node = DoraNode::init_from_env().await?;

    let app = std::env::var("DORA_NODE_PYTHON_FILE")
        .wrap_err("env variable DORA_NODE_RUN_CONFIG must be set")?;
    let func = std::env::var("DORA_NODE_PYTHON_FUNCTION")
        .wrap_err("env variable DORA_NODE_RUN_CONFIG must be set")?;

    let py_function = PythonBinding::try_new(&app, &func).unwrap();

    let mut inputs = Node.inputs().await?;

    loop {
        let mut workload = BTreeMap::new();
        let time_at_start = Instant::now();
        let py_function = py_function.clone();

        // Retrieve several inputs within a time frame
        while time_at_start.elapsed() < BATCH_TIMEOUT {
            let input = match tokio::time::timeout(ATOMIC_TIMEOUT, inputs.next()).await {
                Ok(Some(input)) => input,
                Ok(None) => continue,
                Err(_) => continue,
            };
            workload.insert(input.id.to_string(), input.data);
        }

        // Call the function
        let batch_messages = py_function.call(&workload).unwrap();

        // Send the data one by one.
        for (k, v) in batch_messages {
            Node.send_output(&DataId::from(k), &v).await.unwrap();
        }
    }
}
