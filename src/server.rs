use futures::prelude::*;
use pyo3::prelude::*;
use zenoh::config::Config;
use zenoh::net::protocol::io::SplitBuffer;

pub async fn start_server(file: &str, app: &str) -> eyre::Result<()> {
    // Subscribe
    env_logger::init();
    let config = Config::default();
    let session = zenoh::open(config).await.unwrap();
    let mut subscriber = session.subscribe("a").await.unwrap();
    let identity = initialize(file, app).await.unwrap();

    loop {
        let data = subscriber.next().await.unwrap().value.payload;
        let binary = data.contiguous();
        println!("recieved data");

        let result = Python::with_gil(|py| {
            let a = (binary.into_py(py),);
            pyo3_asyncio::tokio::into_future(identity.call(py, a, None).unwrap().as_ref(py))
        })
        .unwrap()
        .await
        .unwrap();
    }
}

pub async fn initialize(file: &str, app: &str) -> PyResult<Py<PyAny>> {
    Ok(Python::with_gil(|py| {
        let file = py.import(file).unwrap();
        // convert Function into a PyObject
        let identity = file.getattr(app).unwrap();
        identity.to_object(py)
    }))
}
