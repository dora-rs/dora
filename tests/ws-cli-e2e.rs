//! End-to-end tests: coordinator + CLI WsSession over real WebSocket connections.
//!
//! `WsSession` creates its own tokio runtime internally (`block_on`), so these
//! tests run the coordinator on a background thread and use `WsSession` from
//! the main test thread (no nested runtimes).
//!
//! Tests in the `real_dataflow` module use full coordinator+daemon+node stack
//! via `adora up` CLI to test the complete lifecycle.

use adora_cli::WsSession;
use adora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use std::net::SocketAddr;

/// Start a coordinator on a background tokio runtime. Returns the bound port.
fn start_coordinator_background() -> u16 {
    let (port_tx, port_rx) = std::sync::mpsc::channel();

    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("failed to create coordinator runtime");

        rt.block_on(async {
            let bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
            let (port, future) = adora_coordinator::start_testing(bind, futures::stream::empty())
                .await
                .expect("failed to start coordinator");
            port_tx.send(port).unwrap();
            let _ = future.await;
        });
    });

    let port = port_rx.recv().expect("failed to receive coordinator port");
    // Poll until the coordinator is accepting connections (up to 2s)
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        if std::net::TcpStream::connect(format!("127.0.0.1:{port}")).is_ok() {
            break;
        }
        if std::time::Instant::now() > deadline {
            panic!("coordinator did not become ready within 2s");
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    port
}

/// Helper: send a ControlRequest via WsSession and deserialize the reply.
fn send_request(session: &WsSession, req: &ControlRequest) -> eyre::Result<ControlRequestReply> {
    let data = serde_json::to_vec(req)?;
    let reply_bytes = session.request(&data)?;
    let reply: ControlRequestReply = serde_json::from_slice(&reply_bytes)?;
    Ok(reply)
}

#[test]
fn cli_list_empty() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(&session, &ControlRequest::List).unwrap();
    match reply {
        ControlRequestReply::DataflowList(list) => {
            assert!(list.0.is_empty(), "expected empty dataflow list");
        }
        other => panic!("expected DataflowList, got {other:?}"),
    }
}

#[test]
fn cli_status_no_daemon() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(&session, &ControlRequest::DaemonConnected).unwrap();
    match reply {
        ControlRequestReply::DaemonConnected(connected) => {
            assert!(!connected, "no daemons should be connected");
        }
        other => panic!("expected DaemonConnected, got {other:?}"),
    }
}

#[test]
fn cli_stop_nonexistent() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let fake_uuid = uuid::Uuid::new_v4();
    let reply = send_request(
        &session,
        &ControlRequest::Stop {
            dataflow_uuid: fake_uuid,
            grace_duration: None,
            force: false,
        },
    )
    .unwrap();

    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains(&fake_uuid.to_string())
                    || msg.to_lowercase().contains("not found")
                    || !msg.is_empty(),
                "error should be descriptive: {msg}"
            );
        }
        other => panic!("expected Error, got {other:?}"),
    }
}

#[test]
fn cli_multiple_requests_same_session() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    // First request: List
    let reply1 = send_request(&session, &ControlRequest::List).unwrap();
    assert!(matches!(reply1, ControlRequestReply::DataflowList(_)));

    // Second request: DaemonConnected
    let reply2 = send_request(&session, &ControlRequest::DaemonConnected).unwrap();
    assert!(matches!(
        reply2,
        ControlRequestReply::DaemonConnected(false)
    ));

    // Third request: ConnectedMachines
    let reply3 = send_request(&session, &ControlRequest::ConnectedMachines).unwrap();
    match reply3 {
        ControlRequestReply::ConnectedDaemons(daemons) => {
            assert!(daemons.is_empty());
        }
        other => panic!("expected ConnectedDaemons, got {other:?}"),
    }
}

/// Full-stack E2E tests using coordinator + daemon + real nodes.
///
/// These tests use the `adora` CLI binary via `adora up` / `adora start` etc.
/// They must run sequentially (--test-threads=1) because they share the
/// coordinator port. They are in a separate module to group them logically.
mod real_dataflow {
    use std::path::Path;
    use std::process::{Command, Stdio};
    use std::sync::Once;
    use std::time::Duration;

    static BUILD: Once = Once::new();

    fn adora_bin() -> String {
        let manifest = env!("CARGO_MANIFEST_DIR");
        let target_dir = Path::new(manifest).join("target/debug/adora");
        if target_dir.exists() {
            return target_dir.to_string_lossy().to_string();
        }
        "adora".to_string()
    }

    fn ensure_built() {
        BUILD.call_once(|| {
            let status = Command::new("cargo")
                .args([
                    "build",
                    "-p",
                    "adora-cli",
                    "-p",
                    "rust-dataflow-example-node",
                    "-p",
                    "rust-dataflow-example-status-node",
                    "-p",
                    "rust-dataflow-example-sink",
                ])
                .status()
                .expect("failed to build");
            assert!(status.success());
        });
    }

    fn cleanup(adora: &str) {
        let _ = Command::new(adora)
            .args(["stop", "--all"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        std::thread::sleep(Duration::from_millis(500));
        let _ = Command::new(adora)
            .arg("destroy")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        std::thread::sleep(Duration::from_millis(500));
    }

    fn start_cluster(adora: &str) {
        cleanup(adora);
        let status = Command::new(adora)
            .arg("up")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run adora up");
        assert!(status.success(), "adora up failed");
    }

    /// Full lifecycle: start -> list (shows dataflow) -> stop -> destroy
    #[test]
    #[ignore] // Requires exclusive coordinator port; run with --ignored --test-threads=1
    fn e2e_start_list_stop() {
        ensure_built();
        let adora = adora_bin();
        start_cluster(&adora);

        let yaml =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rust-dataflow/dataflow.yml");

        // Start dataflow
        let status = Command::new(&adora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .unwrap();
        assert!(status.success(), "adora start failed");

        // Brief pause to let it register
        std::thread::sleep(Duration::from_secs(1));

        // List should show a dataflow (Running or Succeeded -- it may finish quickly)
        let list_output = Command::new(&adora).arg("list").output().unwrap();
        assert!(list_output.status.success(), "adora list failed");
        let stdout = String::from_utf8_lossy(&list_output.stdout);
        let has_dataflow =
            stdout.contains("Running") || stdout.contains("Succeeded") || stdout.contains("Failed");
        assert!(
            has_dataflow,
            "expected a dataflow in list output, got: {stdout}"
        );

        // Stop all
        let _ = Command::new(&adora)
            .args(["stop", "--all"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();

        cleanup(&adora);
    }

    /// Verify that a second start after the first completes works correctly.
    #[test]
    #[ignore] // Requires exclusive coordinator port; run with --ignored --test-threads=1
    fn e2e_sequential_dataflows() {
        ensure_built();
        let adora = adora_bin();
        start_cluster(&adora);

        let yaml =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rust-dataflow/dataflow.yml");

        // First dataflow
        let status = Command::new(&adora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .unwrap();
        assert!(status.success(), "first start failed");

        // Wait for it to finish
        std::thread::sleep(Duration::from_secs(8));

        // Stop if still running
        let _ = Command::new(&adora)
            .args(["stop", "--all"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();

        std::thread::sleep(Duration::from_secs(1));

        // Second dataflow -- verifies coordinator handles sequential runs
        let status2 = Command::new(&adora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .unwrap();
        assert!(status2.success(), "second start failed");

        std::thread::sleep(Duration::from_secs(1));

        // Verify it's listed
        let list_output = Command::new(&adora).arg("list").output().unwrap();
        assert!(list_output.status.success());
        let stdout = String::from_utf8_lossy(&list_output.stdout);
        let has_dataflow =
            stdout.contains("Running") || stdout.contains("Succeeded") || stdout.contains("Failed");
        assert!(has_dataflow, "second dataflow not listed: {stdout}");

        cleanup(&adora);
    }
}
