use dora_api::{self, config::DataId, DoraOperator};
use eyre::bail;
use futures::StreamExt;
use std::{
    collections::BTreeMap,
    sync::Arc,
    time::{Duration, Instant},
};

static ATOMIC_TIMEOUT: Duration = Duration::from_millis(20);
static BATCH_TIMEOUT: Duration = Duration::from_millis(100);
use dora_rs::python::binding::{call, init};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let operator = DoraOperator::init_from_args().await?;

    let app = "app";
    let function_name = "function";

    let py_function = Arc::new(init("app", "func").unwrap());

    let mut inputs = operator.inputs().await?;

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
        let batch_messages = call(py_function, function_name, &workload).unwrap();

        // Send the data one by one.
        for (k, v) in batch_messages {
            operator.send_output(&DataId::from(k), &v).await.unwrap();
        }
    }
    Ok(())
}
