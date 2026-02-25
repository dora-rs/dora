use std::net::SocketAddr;

/// Start a coordinator on a random port and return (port, background_task).
pub async fn start_test_coordinator() -> (u16, tokio::task::JoinHandle<()>) {
    let bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
    let (port, future) = adora_coordinator::start_testing(bind, futures::stream::empty())
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
