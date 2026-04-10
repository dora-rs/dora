use std::net::SocketAddr;
use std::sync::Arc;

/// Start a coordinator on a random port and return (port, background_task).
pub async fn start_test_coordinator() -> (u16, tokio::task::JoinHandle<()>) {
    start_test_coordinator_with_store(Arc::new(dora_coordinator::InMemoryStore::new())).await
}

/// Start a coordinator with a caller-provided store.
pub async fn start_test_coordinator_with_store(
    store: Arc<dyn dora_coordinator::CoordinatorStore>,
) -> (u16, tokio::task::JoinHandle<()>) {
    let bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
    let (port, future) =
        dora_coordinator::start_testing_with_store(bind, futures::stream::empty(), store)
            .await
            .expect("failed to start coordinator");
    let handle = tokio::spawn(async move {
        let _ = future.await;
    });
    // Poll until the server is accepting connections (up to 2s)
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        if tokio::net::TcpStream::connect(format!("127.0.0.1:{port}"))
            .await
            .is_ok()
        {
            break;
        }
        if tokio::time::Instant::now() > deadline {
            panic!("coordinator did not become ready within 2s");
        }
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
    }
    (port, handle)
}
