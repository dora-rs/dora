//! End-to-end tests: coordinator + CLI WsSession over real WebSocket connections.
//!
//! `WsSession` creates its own tokio runtime internally, so these tests run the
//! coordinator on a background thread and use `WsSession` from the main test
//! thread (no nested runtimes).
//!
//! Tests in the `real_dataflow` module use full coordinator+daemon+node stack
//! via `dora up` CLI to test the complete lifecycle.

use dora_cli::WsSession;
use dora_coordinator::dora_coordinator_store::{DataflowRecord, DataflowStatus};
use dora_coordinator::{CoordinatorStore, InMemoryStore};
use dora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
    current_crate_version, ws_protocol::WsRequest,
};
use futures::{SinkExt, StreamExt};
use std::{collections::BTreeMap, net::SocketAddr, sync::Arc};
use tokio_tungstenite::accept_async;
use uuid::Uuid;

/// Start a coordinator on a background tokio runtime. Returns the bound port.
fn start_coordinator_background() -> u16 {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    start_coordinator_background_with_store(store)
}

/// Start a coordinator with caller-provided store.
fn start_coordinator_background_with_store(store: Arc<dyn CoordinatorStore>) -> u16 {
    let (port_tx, port_rx) = std::sync::mpsc::channel();

    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("failed to create coordinator runtime");

        rt.block_on(async {
            let bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
            let (port, future) =
                dora_coordinator::start_testing_with_store(bind, futures::stream::empty(), store)
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

fn seed_dataflow_record(store: &dyn CoordinatorStore, dataflow_id: uuid::Uuid, node_ids: &[&str]) {
    let descriptor_json = serde_json::json!({
        "nodes": node_ids.iter().map(|id| serde_json::json!({ "id": id })).collect::<Vec<_>>()
    })
    .to_string();
    let record = DataflowRecord {
        uuid: dataflow_id,
        name: Some("seeded-dataflow".to_string()),
        descriptor_json,
        status: DataflowStatus::Succeeded,
        daemon_ids: Vec::new(),
        node_to_daemon: BTreeMap::new(),
        uv: false,
        generation: 1,
        created_at: 0,
        updated_at: 0,
    };
    store.put_dataflow(&record).expect("seed dataflow record");
}

/// Start a minimal control websocket server that acks `TopicSubscribe` and
/// later pushes a binary frame on the same connection.
fn start_mock_topic_server(subscription_id: Uuid, payload: Vec<u8>) -> u16 {
    async fn handle_mock_control_ws<S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin>(
        socket: tokio_tungstenite::WebSocketStream<S>,
        subscription_id: Uuid,
        payload: Vec<u8>,
    ) {
        use tokio_tungstenite::tungstenite::Message;

        let (mut ws_tx, mut ws_rx) = socket.split();
        while let Some(message) = ws_rx.next().await {
            let Ok(Message::Text(text)) = message else {
                continue;
            };
            let request: WsRequest = serde_json::from_str(&text).expect("parse WsRequest");
            let control_request: ControlRequest =
                serde_json::from_value(request.params).expect("parse control request");
            match control_request {
                ControlRequest::Hello { .. } => {
                    let reply = ControlRequestReply::HelloOk {
                        dora_version: current_crate_version(),
                    };
                    let response = serde_json::json!({
                        "id": request.id,
                        "result": reply,
                    });
                    ws_tx
                        .send(Message::Text(response.to_string().into()))
                        .await
                        .expect("send hello reply");
                }
                ControlRequest::TopicSubscribe { .. } => {
                    let reply = ControlRequestReply::TopicSubscribed { subscription_id };
                    let response = serde_json::json!({
                        "id": request.id,
                        "result": reply,
                    });
                    ws_tx
                        .send(Message::Text(response.to_string().into()))
                        .await
                        .expect("send topic subscribe reply");
                    let mut frame = subscription_id.as_bytes().to_vec();
                    frame.extend_from_slice(&payload);
                    ws_tx
                        .send(Message::Binary(frame.into()))
                        .await
                        .expect("send binary frame");
                    break;
                }
                other => panic!("unexpected control request: {other:?}"),
            }
        }
    }

    let (port_tx, port_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("failed to create mock topic runtime");

        rt.block_on(async move {
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
                .await
                .expect("bind mock topic server");
            let port = listener.local_addr().expect("mock local addr").port();
            port_tx.send(port).expect("send mock server port");
            loop {
                let (stream, _) = listener.accept().await.expect("accept mock topic client");
                let payload = payload.clone();
                tokio::spawn(async move {
                    let ws_stream = accept_async(stream).await.expect("accept websocket");
                    handle_mock_control_ws(ws_stream, subscription_id, payload).await;
                });
            }
        });
    });

    let port = port_rx.recv().expect("receive mock topic port");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        if std::net::TcpStream::connect(format!("127.0.0.1:{port}")).is_ok() {
            break;
        }
        if std::time::Instant::now() > deadline {
            panic!("mock topic server did not become ready within 2s");
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

#[test]
fn cli_topic_subscription_receives_binary_frames_immediately_after_subscribe_ack() {
    let subscription_id = Uuid::new_v4();
    let expected_payload = b"topic-payload".to_vec();
    let port = start_mock_topic_server(subscription_id, expected_payload.clone());
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let (received_subscription_id, data_rx) = session
        .subscribe_topics(
            Uuid::new_v4(),
            vec![("node".to_string().into(), "output".to_string().into())],
        )
        .expect("subscribe topics");

    assert_eq!(received_subscription_id, subscription_id);
    let payload = data_rx
        .recv_timeout(std::time::Duration::from_secs(2))
        .expect("receive topic payload")
        .expect("topic payload should be ok");
    assert_eq!(payload, expected_payload);
}

// -- Phase 2: Node control error paths via WsSession --

#[test]
fn cli_restart_node_nonexistent() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(
        &session,
        &ControlRequest::RestartNode {
            dataflow_id: uuid::Uuid::new_v4(),
            node_id: "camera".to_string().into(),
            grace_duration: None,
        },
    )
    .unwrap();

    assert!(
        matches!(reply, ControlRequestReply::Error(_)),
        "expected Error, got {reply:?}"
    );
}

#[test]
fn cli_stop_node_nonexistent() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(
        &session,
        &ControlRequest::StopNode {
            dataflow_id: uuid::Uuid::new_v4(),
            node_id: "sensor".to_string().into(),
            grace_duration: None,
        },
    )
    .unwrap();

    assert!(
        matches!(reply, ControlRequestReply::Error(_)),
        "expected Error, got {reply:?}"
    );
}

// -- Phase 3: Parameter operations via WsSession (E2E) --

#[test]
fn cli_param_set_rejects_unknown_target() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let df_id = uuid::Uuid::new_v4();
    let node_id: dora_message::id::NodeId = "sensor".to_string().into();

    let reply = send_request(
        &session,
        &ControlRequest::SetParam {
            dataflow_id: df_id,
            node_id: node_id.clone(),
            key: "rate".into(),
            value: serde_json::json!(100),
        },
    )
    .unwrap();
    assert!(
        matches!(reply, ControlRequestReply::Error(_)),
        "expected Error for unknown param target, got {reply:?}"
    );
}

#[test]
fn cli_param_get_nonexistent() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(
        &session,
        &ControlRequest::GetParam {
            dataflow_id: uuid::Uuid::new_v4(),
            node_id: "ghost".to_string().into(),
            key: "missing".into(),
        },
    )
    .unwrap();

    assert!(
        matches!(reply, ControlRequestReply::Error(_)),
        "expected Error for missing param, got {reply:?}"
    );
}

#[test]
fn cli_param_delete_rejects_unknown_target() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let df_id = uuid::Uuid::new_v4();
    let node_id: dora_message::id::NodeId = "node".to_string().into();

    let reply = send_request(
        &session,
        &ControlRequest::DeleteParam {
            dataflow_id: df_id,
            node_id,
            key: "rate".into(),
        },
    )
    .unwrap();
    assert!(
        matches!(reply, ControlRequestReply::Error(_)),
        "expected Error for unknown param target, got {reply:?}"
    );
}

#[test]
fn cli_param_set_get_list_delete() {
    let store_impl = Arc::new(InMemoryStore::new());
    let dataflow_id = uuid::Uuid::new_v4();
    seed_dataflow_record(store_impl.as_ref(), dataflow_id, &["sensor"]);
    let store: Arc<dyn CoordinatorStore> = store_impl;
    let port = start_coordinator_background_with_store(store);
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");
    let node_id: dora_message::id::NodeId = "sensor".to_string().into();

    // Set params.
    for (key, value) in [
        ("rate", serde_json::json!(100)),
        ("enabled", serde_json::json!(true)),
    ] {
        let reply = send_request(
            &session,
            &ControlRequest::SetParam {
                dataflow_id,
                node_id: node_id.clone(),
                key: key.into(),
                value,
            },
        )
        .unwrap();
        assert!(matches!(reply, ControlRequestReply::ParamSet));
    }

    // Get one param back.
    let reply = send_request(
        &session,
        &ControlRequest::GetParam {
            dataflow_id,
            node_id: node_id.clone(),
            key: "rate".into(),
        },
    )
    .unwrap();
    match reply {
        ControlRequestReply::ParamValue { key, value } => {
            assert_eq!(key, "rate");
            assert_eq!(value, serde_json::json!(100));
        }
        other => panic!("expected ParamValue, got {other:?}"),
    }

    // List all params.
    let reply = send_request(
        &session,
        &ControlRequest::GetParams {
            dataflow_id,
            node_id: node_id.clone(),
        },
    )
    .unwrap();
    match reply {
        ControlRequestReply::ParamList { params } => {
            assert_eq!(params.len(), 2);
            assert!(
                params
                    .iter()
                    .any(|(k, v)| k == "rate" && *v == serde_json::json!(100))
            );
            assert!(
                params
                    .iter()
                    .any(|(k, v)| k == "enabled" && *v == serde_json::json!(true))
            );
        }
        other => panic!("expected ParamList, got {other:?}"),
    }

    // Delete one param.
    let reply = send_request(
        &session,
        &ControlRequest::DeleteParam {
            dataflow_id,
            node_id: node_id.clone(),
            key: "rate".into(),
        },
    )
    .unwrap();
    assert!(matches!(reply, ControlRequestReply::ParamDeleted));

    // Confirm only one remains.
    let reply = send_request(
        &session,
        &ControlRequest::GetParams {
            dataflow_id,
            node_id,
        },
    )
    .unwrap();
    match reply {
        ControlRequestReply::ParamList { params } => {
            assert_eq!(params.len(), 1);
            assert_eq!(params[0].0, "enabled");
            assert_eq!(params[0].1, serde_json::json!(true));
        }
        other => panic!("expected ParamList, got {other:?}"),
    }
}

#[test]
fn cli_param_set_json_types() {
    let store_impl = Arc::new(InMemoryStore::new());
    let dataflow_id = uuid::Uuid::new_v4();
    seed_dataflow_record(store_impl.as_ref(), dataflow_id, &["sensor"]);
    let store: Arc<dyn CoordinatorStore> = store_impl;
    let port = start_coordinator_background_with_store(store);
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");
    let node_id: dora_message::id::NodeId = "sensor".to_string().into();

    let test_cases = [
        ("int", serde_json::json!(42)),
        ("float", serde_json::json!(1.23)),
        ("string", serde_json::json!("hello")),
        ("bool", serde_json::json!(true)),
        ("array", serde_json::json!([1, 2, 3])),
        ("object", serde_json::json!({"k": "v"})),
    ];

    for (key, value) in test_cases {
        let reply = send_request(
            &session,
            &ControlRequest::SetParam {
                dataflow_id,
                node_id: node_id.clone(),
                key: key.to_string(),
                value: value.clone(),
            },
        )
        .unwrap();
        assert!(
            matches!(reply, ControlRequestReply::ParamSet),
            "set failed for {key}"
        );

        let reply = send_request(
            &session,
            &ControlRequest::GetParam {
                dataflow_id,
                node_id: node_id.clone(),
                key: key.to_string(),
            },
        )
        .unwrap();
        match reply {
            ControlRequestReply::ParamValue {
                key: got_key,
                value: got_value,
            } => {
                assert_eq!(got_key, key);
                assert_eq!(got_value, value);
            }
            other => panic!("expected ParamValue for key {key}, got {other:?}"),
        }
    }
}

/// Full-stack E2E tests using coordinator + daemon + real nodes.
///
/// These tests use the `dora` CLI binary via `dora up` / `dora start` etc.
/// They must run sequentially (--test-threads=1) because they share the
/// coordinator port. They are in a separate module to group them logically.
mod real_dataflow {
    use dora_cli::WsSession;
    use dora_core::topics::{DORA_COORDINATOR_PORT_WS_DEFAULT, LOCALHOST};
    use dora_message::{
        cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
    };
    use std::net::SocketAddr;
    use std::path::Path;
    use std::process::{Command, Stdio};
    use std::sync::Once;
    use std::time::Duration;
    use uuid::Uuid;

    static BUILD: Once = Once::new();

    fn dora_bin() -> String {
        let manifest = env!("CARGO_MANIFEST_DIR");
        let target_dir = Path::new(manifest).join("target/debug/dora");
        if target_dir.exists() {
            return target_dir.to_string_lossy().to_string();
        }
        "dora".to_string()
    }

    fn ensure_built() {
        BUILD.call_once(|| {
            let status = Command::new("cargo")
                .args([
                    "build",
                    "-p",
                    "dora-cli",
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

    fn cleanup(dora: &str) {
        let _ = Command::new(dora)
            .args(["stop", "--all"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        std::thread::sleep(Duration::from_millis(500));
        let _ = Command::new(dora)
            .arg("down")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        std::thread::sleep(Duration::from_millis(500));
    }

    fn start_cluster(dora: &str) {
        cleanup(dora);
        let status = Command::new(dora)
            .arg("up")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora up");
        assert!(status.success(), "dora up failed");
    }

    fn connect_session() -> WsSession {
        let addr: SocketAddr = (LOCALHOST, DORA_COORDINATOR_PORT_WS_DEFAULT).into();
        WsSession::connect(addr).expect("failed to connect ws session to local coordinator")
    }

    fn start_rust_dataflow_detached(dora: &str) -> Uuid {
        let yaml =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rust-dataflow/dataflow.yml");
        let status = Command::new(dora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora start");
        assert!(status.success(), "dora start failed");
        std::thread::sleep(Duration::from_secs(1));

        let session = connect_session();
        let reply = super::send_request(&session, &ControlRequest::List).unwrap();
        match reply {
            ControlRequestReply::DataflowList(list) => list
                .0
                .first()
                .map(|entry| entry.id.uuid)
                .expect("expected a started dataflow in list"),
            other => panic!("unexpected list reply: {other:?}"),
        }
    }

    fn set_param_with_retry(
        session: &WsSession,
        dataflow_id: Uuid,
        node_id: dora_message::id::NodeId,
        key: String,
        value: serde_json::Value,
    ) {
        let deadline = std::time::Instant::now() + Duration::from_secs(8);
        loop {
            let reply = super::send_request(
                session,
                &ControlRequest::SetParam {
                    dataflow_id,
                    node_id: node_id.clone(),
                    key: key.clone(),
                    value: value.clone(),
                },
            )
            .unwrap();
            match reply {
                ControlRequestReply::ParamSet => return,
                ControlRequestReply::Error(msg)
                    if msg.contains("not connected")
                        || msg.contains("channel full")
                        || msg.contains("failed to apply SetParam") =>
                {
                    assert!(
                        std::time::Instant::now() <= deadline,
                        "set failed after retries for key `{key}`: {msg}"
                    );
                    std::thread::sleep(Duration::from_millis(100));
                }
                other => panic!("set failed for key `{key}`: {other:?}"),
            }
        }
    }

    /// Full lifecycle: start -> list (shows dataflow) -> stop -> destroy
    #[test]
    fn e2e_start_list_stop() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);

        let yaml =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rust-dataflow/dataflow.yml");

        // Start dataflow
        let status = Command::new(&dora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .unwrap();
        assert!(status.success(), "dora start failed");

        // Brief pause to let it register
        std::thread::sleep(Duration::from_secs(1));

        // List should show a dataflow (Running or Succeeded -- it may finish quickly)
        let list_output = Command::new(&dora).arg("list").output().unwrap();
        assert!(list_output.status.success(), "dora list failed");
        let stdout = String::from_utf8_lossy(&list_output.stdout);
        let has_dataflow = stdout.contains("Running")
            || stdout.contains("Succeeded")
            || stdout.contains("Finished")
            || stdout.contains("Failed");
        assert!(
            has_dataflow,
            "expected a dataflow in list output, got: {stdout}"
        );

        // Stop all
        let _ = Command::new(&dora)
            .args(["stop", "--all"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();

        cleanup(&dora);
    }

    /// Verify that a second start after the first completes works correctly.
    #[test]
    fn e2e_sequential_dataflows() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);

        let yaml =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/rust-dataflow/dataflow.yml");

        // First dataflow
        let status = Command::new(&dora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .unwrap();
        assert!(status.success(), "first start failed");

        // Wait for it to finish
        std::thread::sleep(Duration::from_secs(8));

        // Stop if still running and wait for full teardown before restarting.
        let _ = Command::new(&dora)
            .args(["stop", "--all"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();

        std::thread::sleep(Duration::from_secs(3));

        // Second dataflow -- verifies coordinator handles sequential runs
        let output2 = Command::new(&dora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .output()
            .unwrap();
        assert!(
            output2.status.success(),
            "second start failed: {}",
            String::from_utf8_lossy(&output2.stderr)
        );

        std::thread::sleep(Duration::from_secs(1));

        // Verify it's listed
        let list_output = Command::new(&dora).arg("list").output().unwrap();
        assert!(list_output.status.success());
        let stdout = String::from_utf8_lossy(&list_output.stdout);
        let has_dataflow = stdout.contains("Running")
            || stdout.contains("Succeeded")
            || stdout.contains("Finished")
            || stdout.contains("Failed");
        assert!(has_dataflow, "second dataflow not listed: {stdout}");

        cleanup(&dora);
    }

    #[test]
    fn e2e_param_set_get_list_delete_running_dataflow() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let dataflow_id = start_rust_dataflow_detached(&dora);
        let node_id: dora_message::id::NodeId = "rust-node".to_string().into();
        let session = connect_session();

        set_param_with_retry(
            &session,
            dataflow_id,
            node_id.clone(),
            "rate".into(),
            serde_json::json!(100),
        );

        let reply = super::send_request(
            &session,
            &ControlRequest::GetParam {
                dataflow_id,
                node_id: node_id.clone(),
                key: "rate".into(),
            },
        )
        .unwrap();
        match reply {
            ControlRequestReply::ParamValue { key, value } => {
                assert_eq!(key, "rate");
                assert_eq!(value, serde_json::json!(100));
            }
            other => panic!("expected ParamValue, got {other:?}"),
        }

        let reply = super::send_request(
            &session,
            &ControlRequest::GetParams {
                dataflow_id,
                node_id: node_id.clone(),
            },
        )
        .unwrap();
        match reply {
            ControlRequestReply::ParamList { params } => {
                assert!(
                    params
                        .iter()
                        .any(|(k, v)| k == "rate" && *v == serde_json::json!(100))
                );
            }
            other => panic!("expected ParamList, got {other:?}"),
        }

        let reply = super::send_request(
            &session,
            &ControlRequest::DeleteParam {
                dataflow_id,
                node_id: node_id.clone(),
                key: "rate".into(),
            },
        )
        .unwrap();
        assert!(matches!(reply, ControlRequestReply::ParamDeleted));

        let reply = super::send_request(
            &session,
            &ControlRequest::GetParam {
                dataflow_id,
                node_id,
                key: "rate".into(),
            },
        )
        .unwrap();
        assert!(matches!(reply, ControlRequestReply::Error(_)));

        cleanup(&dora);
    }

    #[test]
    fn e2e_param_set_json_types_running_dataflow() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let dataflow_id = start_rust_dataflow_detached(&dora);
        let node_id: dora_message::id::NodeId = "rust-node".to_string().into();
        let session = connect_session();

        let test_cases: Vec<(&str, serde_json::Value)> = vec![
            ("int", serde_json::json!(42)),
            ("float", serde_json::json!(1.23)),
            ("string", serde_json::json!("hello")),
            ("bool", serde_json::json!(true)),
            ("null", serde_json::json!(null)),
            ("array", serde_json::json!([1, 2, 3])),
            ("object", serde_json::json!({"a": 1})),
        ];

        for (key, value) in &test_cases {
            set_param_with_retry(
                &session,
                dataflow_id,
                node_id.clone(),
                key.to_string(),
                value.clone(),
            );
        }

        for (key, expected) in &test_cases {
            let reply = super::send_request(
                &session,
                &ControlRequest::GetParam {
                    dataflow_id,
                    node_id: node_id.clone(),
                    key: key.to_string(),
                },
            )
            .unwrap();
            match reply {
                ControlRequestReply::ParamValue { value, .. } => {
                    assert_eq!(&value, expected, "roundtrip failed for {key}");
                }
                other => panic!("expected ParamValue for {key}, got {other:?}"),
            }
        }

        cleanup(&dora);
    }
}
