use eyre::{eyre, Context};
use futures::prelude::*;
use pyo3::prelude::*;
use std::collections::HashMap;
use std::env;
use zenoh::config::Config;
use zenoh::net::protocol::io::SplitBuffer;

pub async fn start_server(file: &str, app: &str) -> eyre::Result<()> {
    // Subscribe
    env_logger::init();
    let config = Config::default();
    let session = zenoh::open(config).await.unwrap();
    let mut subscriber = session
        .subscribe(env::var("SRC_LABELS").unwrap())
        .await
        .map_err(|err| {
            eyre!("Could not subscribe to the given subscription key expression. Error: {err}")
        })
        .unwrap();
    let identity = initialize(file, app).await.unwrap();

    loop {
        let data = subscriber.next().await.unwrap().value.payload;
        let binary = data.contiguous();
        println!("recieved data");

        let result = Python::with_gil(|py| {
            let args = (binary.into_py(py),);
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

pub async fn initialize(file: &str, app: &str) -> eyre::Result<Py<PyAny>> {
    Ok(Python::with_gil(|py| {
        let file = py
            .import(file)
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
