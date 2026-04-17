use std::net::SocketAddr;
use std::sync::{Arc, OnceLock};

fn coordinator_test_lock() -> &'static Arc<tokio::sync::Mutex<()>> {
    static LOCK: OnceLock<Arc<tokio::sync::Mutex<()>>> = OnceLock::new();
    LOCK.get_or_init(|| Arc::new(tokio::sync::Mutex::new(())))
}

pub struct TestCoordinatorHandle {
    handle: tokio::task::JoinHandle<()>,
    _guard: tokio::sync::OwnedMutexGuard<()>,
}

impl Drop for TestCoordinatorHandle {
    fn drop(&mut self) {
        self.handle.abort();
    }
}

/// Start a coordinator on a random port and return (port, background_task).
pub async fn start_test_coordinator() -> (u16, TestCoordinatorHandle) {
    start_test_coordinator_with_store(Arc::new(dora_coordinator::InMemoryStore::new())).await
}

/// Start a coordinator with a caller-provided store.
pub async fn start_test_coordinator_with_store(
    store: Arc<dyn dora_coordinator::CoordinatorStore>,
) -> (u16, TestCoordinatorHandle) {
    let guard = coordinator_test_lock().clone().lock_owned().await;
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
    (
        port,
        TestCoordinatorHandle {
            handle,
            _guard: guard,
        },
    )
}
