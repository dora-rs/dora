//! End-to-end tests: coordinator + CLI WsSession over real WebSocket connections.
//!
//! `WsSession` creates its own tokio runtime internally, so these tests run the
//! coordinator on a background thread and use `WsSession` from the main test
//! thread (no nested runtimes).
//!
//! Tests in the `real_dataflow` module use full coordinator+daemon+node stack
//! via `dora up` CLI to test the complete lifecycle.

use dora_cli::WsSession;
use dora_coordinator::dora_coordinator_store::{
    BuildRecord, DaemonInfo, DataflowRecord, DataflowStatus,
};
use dora_coordinator::{CoordinatorStore, InMemoryStore};
use dora_message::{
    cli_to_coordinator::ControlRequest, common::DaemonId, coordinator_to_cli::ControlRequestReply,
    current_crate_version, id::NodeId, ws_protocol::WsRequest,
};
use futures::{SinkExt, StreamExt};
use std::{
    collections::BTreeMap,
    net::SocketAddr,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};
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
        ready_barrier_released: false,
        barrier_exited_before_subscribe: Vec::new(),
        generation: 1,
        created_at: 0,
        updated_at: 0,
    };
    store.put_dataflow(&record).expect("seed dataflow record");
}

/// Test wrapper around `InMemoryStore` that can be flipped to make
/// `list_dataflows` return an error after the coordinator has finished
/// starting up. Used to exercise `dora clean`'s hard-fail-on-enumeration
/// error path without having to wire a real broken redb file.
struct FailingListStore {
    inner: InMemoryStore,
    fail_list: AtomicBool,
}

impl FailingListStore {
    fn new() -> Self {
        Self {
            inner: InMemoryStore::new(),
            fail_list: AtomicBool::new(false),
        }
    }

    fn enable_failure(&self) {
        self.fail_list.store(true, Ordering::Release);
    }
}

impl CoordinatorStore for FailingListStore {
    fn register_daemon(&self, info: DaemonInfo) -> eyre::Result<()> {
        self.inner.register_daemon(info)
    }
    fn unregister_daemon(&self, id: &DaemonId) -> eyre::Result<()> {
        self.inner.unregister_daemon(id)
    }
    fn list_daemons(&self) -> eyre::Result<Vec<DaemonInfo>> {
        self.inner.list_daemons()
    }
    fn get_daemon(&self, id: &DaemonId) -> eyre::Result<Option<DaemonInfo>> {
        self.inner.get_daemon(id)
    }
    fn get_daemon_by_machine(&self, machine_id: &str) -> eyre::Result<Option<DaemonId>> {
        self.inner.get_daemon_by_machine(machine_id)
    }
    fn put_dataflow(&self, record: &DataflowRecord) -> eyre::Result<()> {
        self.inner.put_dataflow(record)
    }
    fn get_dataflow(&self, uuid: &Uuid) -> eyre::Result<Option<DataflowRecord>> {
        self.inner.get_dataflow(uuid)
    }
    fn list_dataflows(&self) -> eyre::Result<Vec<DataflowRecord>> {
        if self.fail_list.load(Ordering::Acquire) {
            Err(eyre::eyre!("injected list_dataflows failure for test"))
        } else {
            self.inner.list_dataflows()
        }
    }
    fn delete_dataflow(&self, uuid: &Uuid) -> eyre::Result<()> {
        self.inner.delete_dataflow(uuid)
    }
    fn put_build(&self, record: &BuildRecord) -> eyre::Result<()> {
        self.inner.put_build(record)
    }
    fn get_build(&self, build_id: &Uuid) -> eyre::Result<Option<BuildRecord>> {
        self.inner.get_build(build_id)
    }
    fn list_builds(&self) -> eyre::Result<Vec<BuildRecord>> {
        self.inner.list_builds()
    }
    fn delete_build(&self, build_id: &Uuid) -> eyre::Result<()> {
        self.inner.delete_build(build_id)
    }
    fn put_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
        value: &[u8],
    ) -> eyre::Result<()> {
        self.inner.put_node_param(dataflow_id, node_id, key, value)
    }
    fn get_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
    ) -> eyre::Result<Option<Vec<u8>>> {
        self.inner.get_node_param(dataflow_id, node_id, key)
    }
    fn list_node_params(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
    ) -> eyre::Result<Vec<(String, Vec<u8>)>> {
        self.inner.list_node_params(dataflow_id, node_id)
    }
    fn delete_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
    ) -> eyre::Result<()> {
        self.inner.delete_node_param(dataflow_id, node_id, key)
    }
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
fn cli_clean_reaps_persisted_only_record() {
    // Regression for #1835 review round 6: after a coordinator restart,
    // historical Succeeded/Failed rows live in the persisted store but
    // are NOT reloaded into `dataflow_results` (the recovery loop
    // intentionally skips them). `dora clean` must still be able to
    // reap them, otherwise the on-disk redb state grows unbounded.
    //
    // The seeded record stands in for the post-restart state: empty
    // in-memory map + a row in the store.
    let store_impl = Arc::new(InMemoryStore::new());
    let dataflow_id = uuid::Uuid::new_v4();
    seed_dataflow_record(store_impl.as_ref(), dataflow_id, &["sensor"]);
    let store: Arc<dyn CoordinatorStore> = store_impl.clone();
    let port = start_coordinator_background_with_store(store);
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(&session, &ControlRequest::Clean).unwrap();
    match reply {
        ControlRequestReply::CleanResult { cleaned, failed } => {
            assert!(
                failed.is_empty(),
                "expected no failed entries, got {failed:?}"
            );
            assert_eq!(
                cleaned.0.len(),
                1,
                "expected the seeded record to be cleaned, got {:?}",
                cleaned.0
            );
            assert_eq!(cleaned.0[0].id.uuid, dataflow_id);
        }
        other => panic!("expected CleanResult, got {other:?}"),
    }

    // The persisted store row must be gone after a successful clean.
    let surviving = store_impl
        .get_dataflow(&dataflow_id)
        .expect("get_dataflow after clean");
    assert!(
        surviving.is_none(),
        "persisted record should be deleted after clean, got {surviving:?}"
    );
}

#[test]
fn cli_clean_hard_fails_when_persisted_enumeration_fails() {
    // Regression for #1835 review round 7: if `store.list_dataflows()`
    // errors during clean, the coordinator can't honor the "trim disk
    // state" contract because it can't even see what's on disk. Previous
    // behavior silently degraded to in-memory-only and reported success;
    // that lets the CLI claim "nothing to clean" while historical rows
    // sit untouched. The fixed behavior is to return a request error so
    // the CLI exits non-zero and the operator notices.
    let store_impl = Arc::new(FailingListStore::new());
    let store: Arc<dyn CoordinatorStore> = store_impl.clone();
    let port = start_coordinator_background_with_store(store);
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    // Flip the failure switch only AFTER startup has finished using the
    // store — startup calls list_dataflows once and we don't want to
    // sabotage it.
    store_impl.enable_failure();

    let reply = send_request(&session, &ControlRequest::Clean).unwrap();
    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains("failed to enumerate persisted dataflows"),
                "expected enumeration-failure error message, got: {msg}"
            );
            assert!(
                msg.contains("No state was modified"),
                "error message should reassure the operator that state is intact: {msg}"
            );
        }
        other => {
            panic!("expected Error reply when persisted-store enumeration fails, got {other:?}")
        }
    }
}

#[test]
fn cli_clean_empty() {
    // Sanity check: `dora clean` against a coordinator with no finished/failed
    // dataflows must return both lists empty (not panic, not error). This
    // catches protocol regressions on the new `ControlRequest::Clean` variant
    // and the matching `CleanResult` reply — the CLI message round-trips and
    // the coordinator's match arm runs cleanly even when dataflow_results is
    // empty, and the failed list is empty in the absence of store errors.
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(&session, &ControlRequest::Clean).unwrap();
    match reply {
        ControlRequestReply::CleanResult { cleaned, failed } => {
            assert!(
                cleaned.0.is_empty(),
                "expected empty cleaned list, got {:?}",
                cleaned.0
            );
            assert!(
                failed.is_empty(),
                "expected empty failed list, got {failed:?}"
            );
        }
        other => panic!("expected CleanResult, got {other:?}"),
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
                msg.contains(&fake_uuid.to_string()) || msg.to_lowercase().contains("not found"),
                "error must name the unknown UUID {fake_uuid}: {msg}"
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

    let fake_dataflow_id = uuid::Uuid::new_v4();
    let reply = send_request(
        &session,
        &ControlRequest::RestartNode {
            dataflow_id: fake_dataflow_id,
            node_id: "camera".to_string().into(),
            grace_duration: None,
        },
    )
    .unwrap();

    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains(&fake_dataflow_id.to_string())
                    || msg.to_lowercase().contains("not found"),
                "error must name the unknown dataflow {fake_dataflow_id}: {msg}"
            );
        }
        other => panic!("expected Error, got {other:?}"),
    }
}

#[test]
fn cli_stop_node_nonexistent() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let fake_dataflow_id = uuid::Uuid::new_v4();
    let reply = send_request(
        &session,
        &ControlRequest::StopNode {
            dataflow_id: fake_dataflow_id,
            node_id: "sensor".to_string().into(),
            grace_duration: None,
        },
    )
    .unwrap();

    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains(&fake_dataflow_id.to_string())
                    || msg.to_lowercase().contains("not found"),
                "error must name the unknown dataflow {fake_dataflow_id}: {msg}"
            );
        }
        other => panic!("expected Error, got {other:?}"),
    }
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
    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains(&df_id.to_string()) || msg.to_lowercase().contains("not found"),
                "error must name the unknown dataflow {df_id}: {msg}"
            );
        }
        other => panic!("expected Error for unknown param target, got {other:?}"),
    }
}

#[test]
fn cli_param_get_nonexistent() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let df_id = uuid::Uuid::new_v4();
    let reply = send_request(
        &session,
        &ControlRequest::GetParam {
            dataflow_id: df_id,
            node_id: "ghost".to_string().into(),
            key: "missing".into(),
        },
    )
    .unwrap();

    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains(&df_id.to_string()) || msg.to_lowercase().contains("not found"),
                "error must name the unknown dataflow {df_id}: {msg}"
            );
        }
        other => panic!("expected Error for missing param, got {other:?}"),
    }
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
    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains(&df_id.to_string()) || msg.to_lowercase().contains("not found"),
                "error must name the unknown dataflow {df_id}: {msg}"
            );
        }
        other => panic!("expected Error for unknown param target, got {other:?}"),
    }
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

// -- Phase 3.1: Parameter operations via the dora CLI binary (#1655) --
//
// The WsSession tests above cover the control-plane protocol. These tests
// exercise the actual `dora param` subcommand code path
// (binaries/cli/src/command/param/) so a regression in argument parsing,
// JSON deserialization, or output formatting is caught alongside
// protocol-layer regressions.

mod param_cli {
    use super::{seed_dataflow_record, start_coordinator_background_with_store};
    use dora_coordinator::{CoordinatorStore, InMemoryStore};
    use std::path::Path;
    use std::process::{Command, Stdio};
    use std::sync::{Arc, Once};

    static BUILD_CLI: Once = Once::new();

    fn dora_bin() -> String {
        let manifest = env!("CARGO_MANIFEST_DIR");
        // Honor $CARGO_TARGET_DIR + EXE_SUFFIX so Windows finds
        // `dora.exe` instead of falling back to a stale globally-
        // installed `dora` on PATH — that fallback ran the pre-fix
        // CLI/daemon against new-coordinator code and caused
        // test-windows to fail with the silent-reply timeout this PR
        // is meant to fix. Same shape as the example-smoke.rs fix
        // for #1701.
        let target_root = std::env::var("CARGO_TARGET_DIR")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| Path::new(manifest).join("target"));
        let exe_name = format!("dora{}", std::env::consts::EXE_SUFFIX);
        let candidate = target_root.join("debug").join(&exe_name);
        if candidate.exists() {
            return candidate.to_string_lossy().to_string();
        }
        panic!(
            "dora binary not found at {} after ensure_cli_built(); \
             if you use .cargo/config.toml or CARGO_BUILD_TARGET, ensure \
             CARGO_TARGET_DIR points at the resolved artifact directory",
            candidate.display()
        );
    }

    fn ensure_cli_built() {
        BUILD_CLI.call_once(|| {
            let status = Command::new("cargo")
                .args(["build", "-p", "dora-cli"])
                .status()
                .expect("failed to build dora-cli");
            assert!(status.success(), "failed to build dora-cli");
        });
    }

    fn run_dora(
        dora: &str,
        port: u16,
        args: &[&str],
    ) -> (std::process::ExitStatus, String, String) {
        let mut cmd = Command::new(dora);
        cmd.args(args);
        cmd.arg("--coordinator-port").arg(port.to_string());
        let out = cmd
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .expect("failed to spawn dora");
        let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
        let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
        (out.status, stdout, stderr)
    }

    /// Full CRUD via the CLI subprocess: `dora param set`, `get`, `list`,
    /// `delete`. Each step asserts the specific observable output — exit
    /// code alone wouldn't catch a regression that, say, dropped the value
    /// or printed a wrong key.
    #[test]
    fn cli_param_crud_via_subprocess() {
        ensure_cli_built();
        let dora = dora_bin();

        let store_impl = Arc::new(InMemoryStore::new());
        let dataflow_id = uuid::Uuid::new_v4();
        seed_dataflow_record(store_impl.as_ref(), dataflow_id, &["sensor"]);
        let store: Arc<dyn CoordinatorStore> = store_impl;
        let port = start_coordinator_background_with_store(store);

        let df = dataflow_id.to_string();

        // set → "Set `rate` = 100 on node `sensor`"
        let (status, stdout, stderr) = run_dora(
            &dora,
            port,
            &["param", "set", "sensor", "rate", "100", "-d", &df],
        );
        assert!(
            status.success(),
            "dora param set failed: status={status:?}\n\
             stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            stdout.contains("Set `rate` = 100 on node `sensor`"),
            "unexpected set stdout:\n{stdout}"
        );

        // get → the JSON value line
        let (status, stdout, _) =
            run_dora(&dora, port, &["param", "get", "sensor", "rate", "-d", &df]);
        assert!(status.success(), "dora param get failed: {stdout}");
        assert_eq!(stdout.trim(), "100", "get stdout not the JSON value");

        // list → shows "rate = 100"
        let (status, stdout, _) = run_dora(&dora, port, &["param", "list", "sensor", "-d", &df]);
        assert!(status.success(), "dora param list failed: {stdout}");
        assert!(
            stdout.contains("rate = 100"),
            "list didn't show `rate = 100`:\n{stdout}"
        );

        // list --format json → parseable JSON with the key
        let (status, stdout, _) = run_dora(
            &dora,
            port,
            &["param", "list", "sensor", "--format", "json", "-d", &df],
        );
        assert!(status.success());
        let parsed: serde_json::Value =
            serde_json::from_str(stdout.trim()).expect("list --format json output is not JSON");
        assert_eq!(parsed["rate"], serde_json::json!(100));

        // delete → confirmation line
        let (status, stdout, _) = run_dora(
            &dora,
            port,
            &["param", "delete", "sensor", "rate", "-d", &df],
        );
        assert!(status.success(), "dora param delete failed: {stdout}");
        assert!(
            stdout.contains("Deleted `rate` from node `sensor`"),
            "unexpected delete stdout:\n{stdout}"
        );

        // get after delete → error (the key is gone)
        let (status, _, stderr) =
            run_dora(&dora, port, &["param", "get", "sensor", "rate", "-d", &df]);
        assert!(
            !status.success(),
            "dora param get should fail after delete, but exit was success. stderr:\n{stderr}"
        );
    }

    /// Regression guard for the "invalid JSON value" parse path in
    /// `binaries/cli/src/command/param/set.rs` — a bare word like `hello`
    /// is not valid JSON (needs quotes) and the CLI must reject it
    /// cleanly instead of succeeding or panicking.
    #[test]
    fn cli_param_set_rejects_non_json_value() {
        ensure_cli_built();
        let dora = dora_bin();
        let store_impl = Arc::new(InMemoryStore::new());
        let dataflow_id = uuid::Uuid::new_v4();
        seed_dataflow_record(store_impl.as_ref(), dataflow_id, &["sensor"]);
        let store: Arc<dyn CoordinatorStore> = store_impl;
        let port = start_coordinator_background_with_store(store);

        let df = dataflow_id.to_string();

        let (status, stdout, stderr) = run_dora(
            &dora,
            port,
            &["param", "set", "sensor", "mode", "hello", "-d", &df],
        );
        assert!(
            !status.success(),
            "bare-word value should fail JSON parse, but succeeded. \
             stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            stderr.contains("invalid JSON"),
            "stderr should mention invalid JSON, got:\n{stderr}"
        );
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
    use fs2::FileExt as _;
    use redb::{Database, ReadableDatabase, TableDefinition};
    use std::net::SocketAddr;
    use std::path::Path;
    use std::process::{Command, Stdio};
    use std::sync::Once;
    use std::time::{Duration, Instant};
    use tempfile::tempdir;
    use uuid::Uuid;

    static BUILD: Once = Once::new();

    fn dora_bin() -> String {
        let manifest = env!("CARGO_MANIFEST_DIR");
        // Honor $CARGO_TARGET_DIR + EXE_SUFFIX. See the matching copy
        // of this helper earlier in the file for the rationale.
        let target_root = std::env::var("CARGO_TARGET_DIR")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| Path::new(manifest).join("target"));
        let exe_name = format!("dora{}", std::env::consts::EXE_SUFFIX);
        let candidate = target_root.join("debug").join(&exe_name);
        if candidate.exists() {
            return candidate.to_string_lossy().to_string();
        }
        panic!(
            "dora binary not found at {} after ensure_built(); \
             if you use .cargo/config.toml or CARGO_BUILD_TARGET, ensure \
             CARGO_TARGET_DIR points at the resolved artifact directory",
            candidate.display()
        );
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

    fn write_incompatible_coordinator_store(home: &Path) {
        let path = home.join(".dora").join("coordinator.redb");
        std::fs::create_dir_all(path.parent().expect("store parent"))
            .expect("create store directory");
        let db = Database::create(path).expect("create redb store");
        let txn = db.begin_write().expect("begin redb transaction");
        {
            let mut meta: redb::Table<'_, &str, u32> = txn
                .open_table(TableDefinition::new("meta"))
                .expect("open metadata table");
            meta.insert("schema_version", u32::MAX)
                .expect("write incompatible schema version");
        }
        txn.commit().expect("commit incompatible store");
    }

    fn unused_port() -> u16 {
        std::net::TcpListener::bind(("127.0.0.1", 0))
            .expect("bind ephemeral port")
            .local_addr()
            .expect("read ephemeral port")
            .port()
    }

    fn run_dora_isolated(
        dora: &str,
        home: &Path,
        port: u16,
        args: &[&str],
    ) -> (std::process::ExitStatus, String, String) {
        let capture_name = args.join("-");
        let (child, stdout_path, stderr_path) =
            spawn_dora_isolated(dora, home, port, args, &capture_name);
        wait_for_dora_isolated(child, stdout_path, stderr_path)
    }

    fn spawn_dora_isolated(
        dora: &str,
        home: &Path,
        port: u16,
        args: &[&str],
        capture_name: &str,
    ) -> (std::process::Child, std::path::PathBuf, std::path::PathBuf) {
        // Files avoid waiting for pipe EOF on Windows when a detached child
        // inherits a standard-stream handle from `dora up`.
        let stdout_path = home.join(format!("{capture_name}-stdout.txt"));
        let stderr_path = home.join(format!("{capture_name}-stderr.txt"));
        let stdout = std::fs::File::create(&stdout_path).expect("create stdout capture");
        let stderr = std::fs::File::create(&stderr_path).expect("create stderr capture");
        let child = Command::new(dora)
            .args(args)
            .env("HOME", home)
            .env("USERPROFILE", home)
            .env("DORA_COORDINATOR_PORT", port.to_string())
            // Complete the isolation: an inherited coordinator address would
            // make these tests poll a foreign host after mutating the store.
            .env_remove("DORA_COORDINATOR_ADDR")
            .env_remove("DORA_COORDINATOR_INTERFACE")
            .stdout(stdout)
            .stderr(stderr)
            .spawn()
            .expect("spawn isolated dora command");
        (child, stdout_path, stderr_path)
    }

    /// Kills the wrapped `dora` CLI process on drop so assertion failures
    /// cannot leak children blocked on the recreation lock — once the test's
    /// lock is released by the panic, such a child would spawn a detached
    /// coordinator + daemon pointing at the already-deleted temp HOME and
    /// pollute the rest of the serially-run suite.
    struct KillOnDrop(Option<std::process::Child>);

    impl KillOnDrop {
        fn child(&mut self) -> &mut std::process::Child {
            self.0.as_mut().expect("child already taken")
        }

        fn into_inner(mut self) -> std::process::Child {
            self.0.take().expect("child already taken")
        }
    }

    impl Drop for KillOnDrop {
        fn drop(&mut self) {
            if let Some(mut child) = self.0.take() {
                let _ = child.kill();
                let _ = child.wait();
            }
        }
    }

    fn wait_for_dora_isolated(
        mut child: std::process::Child,
        stdout_path: std::path::PathBuf,
        stderr_path: std::path::PathBuf,
    ) -> (std::process::ExitStatus, String, String) {
        let deadline = Instant::now() + Duration::from_secs(15);
        let status = loop {
            if let Some(status) = child.try_wait().expect("poll isolated dora command") {
                break status;
            }
            if Instant::now() >= deadline {
                let _ = child.kill();
                let _ = child.wait();
                panic!("isolated dora command did not exit within 15 seconds");
            }
            std::thread::sleep(Duration::from_millis(20));
        };
        let stdout = std::fs::read_to_string(stdout_path).unwrap_or_default();
        let stderr = std::fs::read_to_string(stderr_path).unwrap_or_default();
        (status, stdout, stderr)
    }

    #[test]
    fn e2e_up_reports_incompatible_coordinator_store() {
        ensure_built();
        let dora = dora_bin();
        let home = tempdir().expect("create isolated home");
        write_incompatible_coordinator_store(home.path());

        let (status, stdout, stderr) =
            run_dora_isolated(&dora, home.path(), unused_port(), &["up"]);

        assert!(
            !status.success(),
            "dora up unexpectedly succeeded; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            stderr.contains("redb schema version mismatch"),
            "coordinator startup error was not surfaced; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            stderr.contains("dora up --recreate-store"),
            "recovery hint was not surfaced; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
    }

    #[test]
    fn e2e_up_recreate_store_archives_incompatible_store() {
        ensure_built();
        let dora = dora_bin();
        let home = tempdir().expect("create isolated home");
        write_incompatible_coordinator_store(home.path());
        let port = unused_port();

        let (status, stdout, stderr) =
            run_dora_isolated(&dora, home.path(), port, &["up", "--recreate-store"]);
        let (down_status, down_stdout, down_stderr) =
            run_dora_isolated(&dora, home.path(), port, &["down"]);

        assert!(
            status.success(),
            "dora up --recreate-store failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            down_status.success(),
            "failed to stop isolated coordinator; stdout:\n{down_stdout}\nstderr:\n{down_stderr}"
        );
        assert!(
            stdout.contains("archived coordinator store"),
            "store archive was not announced; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        let store_dir = home.path().join(".dora");
        assert!(
            !backup_files(&store_dir).is_empty(),
            "incompatible store was not archived"
        );
        assert!(
            store_dir.join("coordinator.redb").exists(),
            "fresh coordinator store was not created"
        );
    }

    /// All `coordinator.redb.backup*` files in the store directory (the
    /// naming scheme of `available_backup_path` in dora-cli).
    fn backup_files(store_dir: &Path) -> Vec<std::path::PathBuf> {
        std::fs::read_dir(store_dir)
            .expect("read store directory")
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| {
                path.file_name().is_some_and(|name| {
                    name.to_string_lossy()
                        .starts_with("coordinator.redb.backup")
                })
            })
            .collect()
    }

    #[test]
    fn e2e_concurrent_up_recreate_store_preserves_original_backup() {
        ensure_built();
        let dora = dora_bin();
        let home = tempdir().expect("create isolated home");
        write_incompatible_coordinator_store(home.path());
        let port = unused_port();
        let store_dir = home.path().join(".dora");
        let lock_path = store_dir.join("coordinator.redb.recreate.lock");
        let lock = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(lock_path)
            .expect("open recreation lock");
        lock.lock_exclusive().expect("hold recreation lock");

        let (first, first_stdout, first_stderr) = spawn_dora_isolated(
            &dora,
            home.path(),
            port,
            &["up", "--recreate-store"],
            "first-recreate",
        );
        let mut first = KillOnDrop(Some(first));
        let (second, second_stdout, second_stderr) = spawn_dora_isolated(
            &dora,
            home.path(),
            port,
            &["up", "--recreate-store"],
            "second-recreate",
        );
        let mut second = KillOnDrop(Some(second));
        std::thread::sleep(Duration::from_millis(200));
        assert!(
            first
                .child()
                .try_wait()
                .expect("poll first recreation")
                .is_none(),
            "first recreation ignored the held store lock"
        );
        assert!(
            second
                .child()
                .try_wait()
                .expect("poll second recreation")
                .is_none(),
            "second recreation ignored the held store lock"
        );
        // Neither invocation may touch the store before holding the lock; if
        // the lock acquisition is ever removed from `up()`, these assertions
        // fail deterministically (the liveness checks above pass either way,
        // and the final single-backup count is reachable without locking).
        assert!(
            store_dir.join("coordinator.redb").exists(),
            "a recreation archived the store without holding the lock"
        );
        assert!(
            backup_files(&store_dir).is_empty(),
            "backup created while the external lock was held"
        );
        fs2::FileExt::unlock(&lock).expect("release recreation lock");

        let (first_status, first_stdout, first_stderr) =
            wait_for_dora_isolated(first.into_inner(), first_stdout, first_stderr);
        let (second_status, second_stdout, second_stderr) =
            wait_for_dora_isolated(second.into_inner(), second_stdout, second_stderr);
        let (down_status, down_stdout, down_stderr) =
            run_dora_isolated(&dora, home.path(), port, &["down"]);

        assert!(
            first_status.success(),
            "first recreation failed; stdout:\n{first_stdout}\nstderr:\n{first_stderr}"
        );
        assert!(
            second_status.success(),
            "second recreation failed; stdout:\n{second_stdout}\nstderr:\n{second_stderr}"
        );
        assert!(
            down_status.success(),
            "failed to stop isolated coordinator; stdout:\n{down_stdout}\nstderr:\n{down_stderr}"
        );

        let backups = backup_files(&store_dir);
        assert_eq!(
            backups.len(),
            1,
            "concurrent recreation should archive the original store exactly once"
        );
        let backup = Database::open(&backups[0]).expect("open archived store");
        let txn = backup.begin_read().expect("read archived store");
        let meta = txn
            .open_table::<&str, u32>(TableDefinition::new("meta"))
            .expect("open archived metadata table");
        let archived_version = meta
            .get("schema_version")
            .expect("read archived schema version")
            .expect("archived schema version exists")
            .value();
        assert_eq!(
            archived_version,
            u32::MAX,
            "the original incompatible store backup was overwritten"
        );
    }

    #[test]
    fn e2e_up_recreate_store_skips_archive_when_coordinator_running() {
        ensure_built();
        let dora = dora_bin();
        let home = tempdir().expect("create isolated home");
        let port = unused_port();

        let (up_status, up_stdout, up_stderr) =
            run_dora_isolated(&dora, home.path(), port, &["up"]);
        let (status, stdout, stderr) =
            run_dora_isolated(&dora, home.path(), port, &["up", "--recreate-store"]);
        let (down_status, down_stdout, down_stderr) =
            run_dora_isolated(&dora, home.path(), port, &["down"]);

        assert!(
            up_status.success(),
            "initial dora up failed; stdout:\n{up_stdout}\nstderr:\n{up_stderr}"
        );
        assert!(
            status.success(),
            "recreate against a running coordinator failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            down_status.success(),
            "failed to stop isolated coordinator; stdout:\n{down_stdout}\nstderr:\n{down_stderr}"
        );
        assert!(
            stdout.contains("--recreate-store skipped"),
            "skip notice was not printed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        let store_dir = home.path().join(".dora");
        assert!(
            backup_files(&store_dir).is_empty(),
            "live store was archived despite a running coordinator"
        );
        assert!(
            store_dir.join("coordinator.redb").exists(),
            "live coordinator store went missing"
        );
    }

    #[test]
    fn e2e_up_recreate_store_with_no_existing_store_succeeds() {
        ensure_built();
        let dora = dora_bin();
        let home = tempdir().expect("create isolated home");
        let port = unused_port();

        let (status, stdout, stderr) =
            run_dora_isolated(&dora, home.path(), port, &["up", "--recreate-store"]);
        let (down_status, down_stdout, down_stderr) =
            run_dora_isolated(&dora, home.path(), port, &["down"]);

        assert!(
            status.success(),
            "dora up --recreate-store failed on an empty home; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            down_status.success(),
            "failed to stop isolated coordinator; stdout:\n{down_stdout}\nstderr:\n{down_stderr}"
        );
        assert!(
            stdout.contains("nothing to archive"),
            "missing-store notice was not printed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        let store_dir = home.path().join(".dora");
        assert!(
            backup_files(&store_dir).is_empty(),
            "backup created although no store existed"
        );
        assert!(
            store_dir.join("coordinator.redb").exists(),
            "fresh coordinator store was not created"
        );
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

    struct ClusterGuard<'a>(&'a str);
    impl<'a> Drop for ClusterGuard<'a> {
        fn drop(&mut self) {
            cleanup(self.0);
        }
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

    /// dora-rs/dora#2919: `dora start --env KEY=VAL` must reach
    /// daemon-spawned nodes with the documented precedence:
    /// node `env:` > `--env` > dataflow-level `env:`.
    ///
    /// Under `dora start` nodes inherit the DAEMON's environment, not
    /// the CLI invocation's, so `--env` is the only run-parameterization
    /// channel that doesn't require editing the YAML. One probe pins the
    /// whole chain: the dataflow YAML sets `E2919_VAR: from-yaml`
    /// (--env must beat it) and the node sets `E2919_NODE: node-wins`
    /// (--env must lose to it).
    ///
    /// The probe is a plain python script, not a dora node — it writes
    /// the two variables to a marker and idles. It never subscribes, so
    /// the dataflow sits in startup; the marker is written at spawn,
    /// which is all this test observes, and `ClusterGuard` tears the
    /// dataflow down. The idle is bounded (120s) so a probe orphaned by
    /// teardown ordering self-reaps.
    ///
    /// Unix-only: relies on `python3` on PATH and on direct exec of a
    /// shebang-less `.py` probe, both of which differ on Windows.
    #[test]
    #[cfg(unix)]
    fn e2e_start_env_flag_reaches_daemon_spawned_nodes() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let _guard = ClusterGuard(&dora);

        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        // pid-suffixed artifacts are never reused — reclaim stale ones
        if let Ok(entries) = std::fs::read_dir(&target) {
            for entry in entries.flatten() {
                if entry
                    .file_name()
                    .to_string_lossy()
                    .starts_with("dora-env-2919-")
                {
                    let _ = std::fs::remove_file(entry.path());
                }
            }
        }
        let stem = format!("dora-env-2919-{}", std::process::id());
        let probe = target.join(format!("{stem}-probe.py"));
        let marker = target.join(format!("{stem}-out.txt"));
        let yaml = target.join(format!("{stem}.yml"));
        // concat!, not a `\`-continued literal: Rust's line continuation
        // strips leading whitespace, which would flatten the indented
        // python block and crash the probe with an IndentationError.
        std::fs::write(
            &probe,
            concat!(
                "import os, time\n",
                "out = os.environ['PROBE_OUT']\n",
                // write-then-rename: the poller checks existence, so the
                // final name must never be observable half-written
                "with open(out + '.tmp', 'w') as f:\n",
                "    f.write(os.environ.get('E2919_VAR', '<unset>') + '\\n')\n",
                "    f.write(os.environ.get('E2919_NODE', '<unset>') + '\\n')\n",
                "os.replace(out + '.tmp', out)\n",
                "time.sleep(120)\n",
            ),
        )
        .expect("failed to write probe");
        std::fs::write(
            &yaml,
            format!(
                "env:\n  \
                   E2919_VAR: from-yaml\n\
                 nodes:\n  \
                 - id: probe\n    \
                   path: {probe}\n    \
                   env:\n      \
                     PROBE_OUT: {marker}\n      \
                     E2919_NODE: node-wins\n    \
                   outputs:\n      - value\n",
                probe = probe.display(),
                marker = marker.display(),
            ),
        )
        .expect("failed to write dataflow yaml");

        let status = Command::new(&dora)
            .args([
                "start",
                yaml.to_str().unwrap(),
                "--detach",
                "--env",
                "E2919_VAR=from-cli",
                "--env",
                "E2919_NODE=cli-loses",
            ])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora start");
        assert!(status.success(), "dora start --env failed");

        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        while !marker.exists() {
            assert!(
                std::time::Instant::now() < deadline,
                "probe never wrote its marker — the node was not spawned \
                 (is `python3` installed? the probe is a python script the \
                 daemon spawns)"
            );
            std::thread::sleep(Duration::from_millis(200));
        }
        let content = std::fs::read_to_string(&marker).expect("failed to read marker");
        let lines: Vec<&str> = content.lines().collect();
        assert_eq!(
            lines.first().copied(),
            Some("from-cli"),
            "--env must override the dataflow-level env: block; got {content:?}"
        );
        assert_eq!(
            lines.get(1).copied(),
            Some("node-wins"),
            "node-level env: must still win over --env; got {content:?}"
        );
    }

    /// dora-rs/dora#2920: `dora run --exit-when-nodes-finish` must let a
    /// timer-driven graph terminate once its work is done.
    ///
    /// A timer input is registered like any other but never closes —
    /// there is no upstream node to finish — so by default a node
    /// consuming `dora/timer/...` is never told its inputs are closed
    /// and never exits. The reported shape: every worker finished, and
    /// the dataflow still sat for 9+ minutes because one node was
    /// discarding timer ticks.
    ///
    /// `producer` emits once on its first tick and exits; `consumer` has
    /// that data input plus a timer of its own. Without the flag this
    /// run never returns (asserted first, with a bounded wait); with it,
    /// the dataflow completes on its own.
    ///
    /// Local-only: no coordinator, no daemon process, no port 6013.
    /// Unix-only for the same reasons as the sibling run tests.
    #[test]
    #[cfg(unix)]
    fn e2e_run_exit_when_nodes_finish_terminates_timer_driven_graph() {
        ensure_built();
        let dora = dora_bin();
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "emit-then-exit-source-node",
                "-p",
                "drain-recording-node",
            ])
            .status()
            .expect("failed to build fixtures");
        assert!(status.success(), "failed to build fixtures");

        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        // Sweep stale artifacts from earlier runs, as the sibling tests do.
        if let Ok(entries) = std::fs::read_dir(&target) {
            for entry in entries.flatten() {
                let name = entry.file_name();
                if name.to_string_lossy().starts_with("dora-timer-2920-") {
                    let _ = std::fs::remove_file(entry.path());
                }
            }
        }
        let yaml = target.join(format!("dora-timer-2920-{}.yml", std::process::id()));
        let record = target.join(format!("dora-timer-2920-{}.record", std::process::id()));
        let _ = std::fs::remove_file(&record);
        std::fs::write(
            &yaml,
            format!(
                "nodes:\n  \
                 - id: producer\n    \
                   path: {producer}\n    \
                   inputs:\n      \
                     tick: dora/timer/millis/100\n    \
                   outputs:\n      - value\n  \
                 - id: consumer\n    \
                   path: {consumer}\n    \
                   env:\n      \
                     DORA_TEST_DRAIN_RECORD: {record}\n    \
                   inputs:\n      \
                     value: producer/value\n      \
                     tick: dora/timer/millis/200\n",
                producer = target.join("debug/emit-then-exit-source-node").display(),
                consumer = target.join("debug/drain-recording-node").display(),
                record = record.display(),
            ),
        )
        .expect("failed to write dataflow yaml");

        // Premise: without the flag this graph does not terminate. Bound
        // it so a regression that makes it exit on its own shows up here
        // rather than silently making the flag look unnecessary.
        let mut without = Command::new(&dora)
            .args(["run", yaml.to_str().unwrap()])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("failed to spawn dora run");
        let deadline = std::time::Instant::now() + Duration::from_secs(20);
        let exited_on_its_own = loop {
            match without.try_wait().expect("try_wait failed") {
                Some(_) => break true,
                None if std::time::Instant::now() >= deadline => break false,
                None => std::thread::sleep(Duration::from_millis(200)),
            }
        };
        // SIGTERM, not `kill()`. `Child::kill` sends SIGKILL, which
        // `dora run` cannot handle, so its spawned nodes are orphaned and
        // keep running for the rest of the CI job — verified: the
        // consumer survives indefinitely. SIGTERM gives the run a chance
        // to tear its nodes down (dora-rs/dora#2949), and we only escalate
        // if it ignores that.
        let _ = Command::new("kill")
            .args(["-TERM", &without.id().to_string()])
            .status();
        let term_deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            match without.try_wait().expect("try_wait failed") {
                Some(_) => break,
                None if std::time::Instant::now() >= term_deadline => {
                    let _ = without.kill();
                    break;
                }
                None => std::thread::sleep(Duration::from_millis(100)),
            }
        }
        let _ = without.wait();
        assert!(
            !exited_on_its_own,
            "test premise broken: this graph terminated WITHOUT \
             --exit-when-nodes-finish, so the flag is not what this test thinks \
             it is measuring"
        );

        // With the flag it must finish on its own, and cleanly. Bounded
        // rather than a plain `.output()`: if the flag regresses, this
        // run never returns, and a hanging test is far worse in CI than a
        // failing one — it burns the job's whole budget and reports
        // nothing useful.
        // Clear the record the premise run may have written: its consumer
        // can outlive the `dora run` we just reaped, and a late write of
        // its own result would otherwise be read as this run's.
        let _ = std::fs::remove_file(&record);
        let mut with = Command::new(&dora)
            .args(["run", yaml.to_str().unwrap(), "--exit-when-nodes-finish"])
            .stdout(Stdio::null())
            // Null, not piped: nothing reads this pipe, and a `dora run`
            // that fills the buffer would then block on write and look
            // like the hang this test is trying to detect.
            .stderr(Stdio::null())
            .spawn()
            .expect("failed to spawn dora run");
        let deadline = std::time::Instant::now() + Duration::from_secs(60);
        let status = loop {
            match with.try_wait().expect("try_wait failed") {
                Some(status) => break Some(status),
                None if std::time::Instant::now() >= deadline => break None,
                None => std::thread::sleep(Duration::from_millis(200)),
            }
        };
        let Some(status) = status else {
            let _ = with.kill();
            let _ = with.wait();
            panic!(
                "`dora run --exit-when-nodes-finish` never terminated — the timer \
                 input is still gating the drain (#2920)"
            );
        };
        assert!(
            status.success(),
            "`dora run --exit-when-nodes-finish` exited with {status}"
        );

        // Terminating is necessary but not sufficient: draining the
        // consumer at startup, before the producer ever sent anything,
        // would also terminate and also exit 0. Assert it did its work
        // first, and that it left because it was told all inputs were
        // closed rather than because its event stream happened to end.
        let recorded = std::fs::read_to_string(&record).unwrap_or_else(|e| {
            panic!(
                "consumer wrote no drain record at {} ({e}) — it did not reach \
                 a clean exit",
                record.display()
            )
        });
        let (inputs, drained) = recorded
            .split_once(' ')
            .unwrap_or_else(|| panic!("malformed drain record {recorded:?}"));
        assert_eq!(
            drained, "true",
            "consumer exited without receiving AllInputsClosed (record: {recorded:?}) \
             — the dataflow ended for some other reason, so this test would pass \
             even if the drain opt-in did nothing"
        );
        assert!(
            inputs.parse::<u64>().expect("input count") > 0,
            "consumer drained without ever receiving `value` (record: {recorded:?}) \
             — it was stopped at startup, not after finishing its work"
        );

        let _ = std::fs::remove_file(&yaml);
        let _ = std::fs::remove_file(&record);
    }

    /// dora-rs/dora#2920: `dora start --exit-when-nodes-finish` must let
    /// a timer-driven dataflow finish on its own.
    ///
    /// The `dora run` half of this shipped first; `start` goes through a
    /// completely different path — CLI to coordinator to daemon — so it
    /// needs its own proof. The setting travels on the descriptor rather
    /// than the wire precisely so it survives that trip (and later,
    /// auto-recovery and restart), which is exactly what this checks.
    ///
    /// Status, not absence: a finished dataflow REMAINS in `dora list`
    /// with status `Finished`, so "no longer listed" would never become
    /// true and would make this pass for the wrong reason.
    #[test]
    #[cfg(unix)]
    fn e2e_start_exit_when_nodes_finish_terminates_timer_driven_graph() {
        ensure_built();
        let dora = dora_bin();
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "emit-then-exit-source-node",
                "-p",
                "drain-recording-node",
            ])
            .status()
            .expect("failed to build fixtures");
        assert!(status.success(), "failed to build fixtures");

        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        let yaml = target.join(format!("dora-start-2920-{}.yml", std::process::id()));
        std::fs::write(
            &yaml,
            format!(
                "nodes:\n  \
                 - id: producer\n    \
                   path: {producer}\n    \
                   inputs:\n      \
                     tick: dora/timer/millis/100\n    \
                   outputs:\n      - value\n  \
                 - id: consumer\n    \
                   path: {consumer}\n    \
                   inputs:\n      \
                     value: producer/value\n      \
                     tick: dora/timer/millis/200\n",
                producer = target.join("debug/emit-then-exit-source-node").display(),
                consumer = target.join("debug/drain-recording-node").display(),
            ),
        )
        .expect("failed to write dataflow yaml");

        start_cluster(&dora);
        // Tear the cluster down on EVERY exit path. The premise assertion
        // below fires before the explicit cleanup, so without this a
        // broken premise would strand a coordinator, a daemon and this
        // dataflow's nodes for the rest of the suite.
        let _guard = ClusterGuard(&dora);

        let status_of = |name: &str| -> String {
            let out = Command::new(&dora)
                .arg("list")
                .output()
                .expect("failed to run dora list");
            String::from_utf8_lossy(&out.stdout)
                .lines()
                .find(|l| l.split_whitespace().nth(1) == Some(name))
                .and_then(|l| l.split_whitespace().nth(2).map(str::to_owned))
                .unwrap_or_default()
        };
        let settle = |name: &str, secs: u64| -> String {
            let deadline = std::time::Instant::now() + Duration::from_secs(secs);
            loop {
                let s = status_of(name);
                if s == "Finished" || std::time::Instant::now() >= deadline {
                    break s;
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        };

        // Premise: without the flag this graph does not finish on its own.
        let started = Command::new(&dora)
            .args([
                "start",
                yaml.to_str().unwrap(),
                "--detach",
                "--name",
                "e2e2920noflag",
            ])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora start");
        assert!(started.success(), "dora start (no flag) failed");
        let without = settle("e2e2920noflag", 15);
        let _ = Command::new(&dora)
            .args(["stop", "--name", "e2e2920noflag"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        assert_ne!(
            without, "Finished",
            "test premise broken: this graph finished WITHOUT \
             --exit-when-nodes-finish, so the flag is not what this test \
             thinks it is measuring"
        );

        // With the flag it must reach `Finished` on its own.
        let started = Command::new(&dora)
            .args([
                "start",
                yaml.to_str().unwrap(),
                "--detach",
                "--name",
                "e2e2920flag",
                "--exit-when-nodes-finish",
            ])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora start");
        assert!(started.success(), "dora start (with flag) failed");
        let with = settle("e2e2920flag", 60);

        let _ = Command::new(&dora)
            .args(["stop", "--name", "e2e2920flag"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        let _ = std::fs::remove_file(&yaml);

        assert_eq!(
            with, "Finished",
            "`dora start --exit-when-nodes-finish` left the dataflow in state \
             `{with}` — the timer is still gating the drain across the \
             coordinator path (#2920)"
        );
    }

    /// #2919 P2: a node added to a RUNNING dataflow must inherit that
    /// dataflow's env, including anything set via `dora start --env`.
    ///
    /// `dora node add` resolves the incoming node through a temporary
    /// single-node descriptor in the coordinator; that descriptor was
    /// built with `env: None`, so the dataflow-into-node merge had
    /// nothing to merge and an added node silently saw none of the
    /// dataflow's environment. Reproduced as `<unset>` before the fix.
    ///
    /// Unix-only for the same reasons as the sibling env tests.
    #[test]
    #[cfg(unix)]
    fn e2e_dynamically_added_node_inherits_dataflow_env() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let _guard = ClusterGuard(&dora);

        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        if let Ok(entries) = std::fs::read_dir(&target) {
            for entry in entries.flatten() {
                if entry
                    .file_name()
                    .to_string_lossy()
                    .starts_with("dora-addenv-2919-")
                {
                    let _ = std::fs::remove_file(entry.path());
                }
            }
        }
        let stem = format!("dora-addenv-2919-{}", std::process::id());
        let probe = target.join(format!("{stem}-probe.py"));
        let base_marker = target.join(format!("{stem}-base.txt"));
        let added_marker = target.join(format!("{stem}-added.txt"));
        let yaml = target.join(format!("{stem}.yml"));
        let added_yaml = target.join(format!("{stem}-added.yml"));
        std::fs::write(
            &probe,
            concat!(
                "import os, time\n",
                "out = os.environ['PROBE_OUT']\n",
                "with open(out + '.tmp', 'w') as f:\n",
                "    f.write(os.environ.get('E2919_VAR', '<unset>') + '\\n')\n",
                "    f.write(os.environ.get('E2919_NODE', '<unset>') + '\\n')\n",
                "os.replace(out + '.tmp', out)\n",
                "time.sleep(120)\n",
            ),
        )
        .expect("failed to write probe");
        std::fs::write(
            &yaml,
            format!(
                "nodes:\n  \
                 - id: base\n    \
                   path: {probe}\n    \
                   env:\n      \
                     PROBE_OUT: {base_marker}\n    \
                   outputs:\n      - value\n",
                probe = probe.display(),
                base_marker = base_marker.display(),
            ),
        )
        .expect("failed to write dataflow yaml");
        // The added node sets its own E2919_NODE, so this also pins that
        // node-level env still wins for dynamically added nodes.
        std::fs::write(
            &added_yaml,
            format!(
                "id: added\n\
                 path: {probe}\n\
                 env:\n  \
                   PROBE_OUT: {added_marker}\n  \
                   E2919_NODE: node-wins\n\
                 outputs:\n  - value\n",
                probe = probe.display(),
                added_marker = added_marker.display(),
            ),
        )
        .expect("failed to write added node yaml");

        let status = Command::new(&dora)
            .args([
                "start",
                yaml.to_str().unwrap(),
                "--detach",
                "--name",
                "addenv",
                "--env",
                "E2919_VAR=from-cli",
                "--env",
                "E2919_NODE=cli-loses",
            ])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora start");
        assert!(status.success(), "dora start --env failed");

        // Wait for the base node so the dataflow is up before adding.
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        while !base_marker.exists() {
            assert!(
                std::time::Instant::now() < deadline,
                "base node never spawned (is `python3` installed?)"
            );
            std::thread::sleep(Duration::from_millis(200));
        }

        let output = Command::new(&dora)
            .args([
                "node",
                "add",
                "--dataflow",
                "addenv",
                "--from-yaml",
                added_yaml.to_str().unwrap(),
            ])
            .output()
            .expect("failed to run dora node add");
        assert!(
            output.status.success(),
            "dora node add failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );

        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        while !added_marker.exists() {
            assert!(
                std::time::Instant::now() < deadline,
                "added node never spawned"
            );
            std::thread::sleep(Duration::from_millis(200));
        }
        let content = std::fs::read_to_string(&added_marker).expect("failed to read marker");
        let lines: Vec<&str> = content.lines().collect();
        assert_eq!(
            lines.first().copied(),
            Some("from-cli"),
            "a node added to a running dataflow must inherit its env \
             (including `--env`); got {content:?}"
        );
        assert_eq!(
            lines.get(1).copied(),
            Some("node-wins"),
            "node-level env: must still win for added nodes; got {content:?}"
        );
    }

    /// The `dora run` half of #2919, which reaches the daemon by a
    /// different route: `run` builds a `descriptor_override` and hands it
    /// to the in-process `Daemon::run_dataflow`, rather than serializing
    /// a `ControlRequest::Start`. That wiring — the empty-env
    /// passthrough, the hub-resolved-vs-disk base, and the merge itself —
    /// is not touched by the `start` test above.
    ///
    /// Runs local-only: no coordinator, no daemon process, no port 6013,
    /// so it cannot contend with the networked tests in this file.
    /// `--stop-after` bounds it; the probe writes its marker at spawn and
    /// then idles until stopped.
    ///
    /// Unix-only for the same reasons as the `start` test.
    #[test]
    #[cfg(unix)]
    fn e2e_run_env_flag_reaches_locally_spawned_nodes() {
        ensure_built();
        let dora = dora_bin();

        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        if let Ok(entries) = std::fs::read_dir(&target) {
            for entry in entries.flatten() {
                if entry
                    .file_name()
                    .to_string_lossy()
                    .starts_with("dora-runenv-2919-")
                {
                    let _ = std::fs::remove_file(entry.path());
                }
            }
        }
        let stem = format!("dora-runenv-2919-{}", std::process::id());
        let probe = target.join(format!("{stem}-probe.py"));
        let marker = target.join(format!("{stem}-out.txt"));
        let yaml = target.join(format!("{stem}.yml"));
        std::fs::write(
            &probe,
            concat!(
                "import os, time\n",
                "out = os.environ['PROBE_OUT']\n",
                "with open(out + '.tmp', 'w') as f:\n",
                "    f.write(os.environ.get('E2919_VAR', '<unset>') + '\\n')\n",
                "    f.write(os.environ.get('E2919_NODE', '<unset>') + '\\n')\n",
                "os.replace(out + '.tmp', out)\n",
                "time.sleep(120)\n",
            ),
        )
        .expect("failed to write probe");
        std::fs::write(
            &yaml,
            format!(
                "env:\n  \
                   E2919_VAR: from-yaml\n\
                 nodes:\n  \
                 - id: probe\n    \
                   path: {probe}\n    \
                   env:\n      \
                     PROBE_OUT: {marker}\n      \
                     E2919_NODE: node-wins\n    \
                   outputs:\n      - value\n",
                probe = probe.display(),
                marker = marker.display(),
            ),
        )
        .expect("failed to write dataflow yaml");

        let output = Command::new(&dora)
            .args([
                "run",
                yaml.to_str().unwrap(),
                "--stop-after",
                "12s",
                "--env",
                "E2919_VAR=from-cli",
                "--env",
                "E2919_NODE=cli-loses",
            ])
            .output()
            .expect("failed to run dora run");

        let content = std::fs::read_to_string(&marker).unwrap_or_else(|e| {
            panic!(
                "probe never wrote its marker ({e}) — is `python3` installed? \
                 dora run stderr:\n{}",
                String::from_utf8_lossy(&output.stderr)
            )
        });
        let lines: Vec<&str> = content.lines().collect();
        assert_eq!(
            lines.first().copied(),
            Some("from-cli"),
            "--env must override the dataflow-level env: block under `dora run`; got {content:?}"
        );
        assert_eq!(
            lines.get(1).copied(),
            Some("node-wins"),
            "node-level env: must still win over --env under `dora run`; got {content:?}"
        );
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

    fn start_lifecycle_dataflow_detached(dora: &str) -> Uuid {
        // Assert no stale dataflows before we start — if the coordinator
        // already has an entry, first() would return the wrong UUID and every
        // subsequent node-command assertion would silently target it instead.
        {
            let session = connect_session();
            if let ControlRequestReply::DataflowList(list) =
                super::send_request(&session, &ControlRequest::List).unwrap()
            {
                assert!(
                    list.0.is_empty(),
                    "stale dataflow(s) in coordinator before start: {:?}",
                    list.0.iter().map(|e| e.id.uuid).collect::<Vec<_>>()
                );
            }
        }

        let yaml = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/node-lifecycle.yml");
        let status = Command::new(dora)
            .args(["start", yaml.to_str().unwrap(), "--detach"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora start");
        assert!(status.success(), "dora start failed");

        // Poll until the coordinator records the dataflow (max 10s).
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        let dataflow_id = loop {
            let session = connect_session();
            let reply = super::send_request(&session, &ControlRequest::List).unwrap();
            if let ControlRequestReply::DataflowList(list) = reply
                && let Some(entry) = list.0.first()
            {
                break entry.id.uuid;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "dataflow never appeared in list"
            );
            std::thread::sleep(Duration::from_millis(300));
        };

        // Wait until all three nodes show up in the daemon's running_nodes
        // (queried via `dora node info`). This avoids the race where the
        // dataflow is registered at coordinator level but the daemon hasn't
        // yet spawned+registered individual nodes.
        let expected_nodes = ["rust-node", "rust-status-node", "rust-sink"];
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        loop {
            let all_up = expected_nodes.iter().all(|node| {
                Command::new(dora)
                    .args(["node", "info", node, "--dataflow", &dataflow_id.to_string()])
                    .stdout(Stdio::null())
                    .stderr(Stdio::null())
                    .status()
                    .map(|s| s.success())
                    .unwrap_or(false)
            });
            if all_up {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "nodes never registered with daemon"
            );
            std::thread::sleep(Duration::from_millis(300));
        }

        dataflow_id
    }

    /// `dora node list --dataflow` lists all nodes in the running dataflow.
    #[test]
    fn e2e_node_list_shows_running_nodes() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let _guard = ClusterGuard(&dora);
        let dataflow_id = start_lifecycle_dataflow_detached(&dora);

        let output = Command::new(&dora)
            .args(["node", "list", "--dataflow", &dataflow_id.to_string()])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "dora node list failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            stdout.contains("rust-node"),
            "rust-node missing from list: {stdout}"
        );
        assert!(
            stdout.contains("rust-status-node"),
            "rust-status-node missing: {stdout}"
        );
        assert!(stdout.contains("rust-sink"), "rust-sink missing: {stdout}");

        cleanup(&dora);
    }

    /// `dora node info` returns the descriptor with inputs and outputs.
    #[test]
    fn e2e_node_info_returns_descriptor() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let _guard = ClusterGuard(&dora);
        let dataflow_id = start_lifecycle_dataflow_detached(&dora);

        let output = Command::new(&dora)
            .args([
                "node",
                "info",
                "rust-node",
                "--dataflow",
                &dataflow_id.to_string(),
            ])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "dora node info failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            stdout.contains("rust-node"),
            "node id missing in info: {stdout}"
        );
        assert!(
            stdout.contains("tick"),
            "tick input missing in info: {stdout}"
        );
        assert!(
            stdout.contains("random"),
            "random output missing in info: {stdout}"
        );

        cleanup(&dora);
    }

    /// `dora node stop` stops one node, leaving the rest of the dataflow running.
    #[test]
    fn e2e_node_stop_exits_cleanly() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let _guard = ClusterGuard(&dora);
        let dataflow_id = start_lifecycle_dataflow_detached(&dora);

        let output = Command::new(&dora)
            .args([
                "node",
                "stop",
                "rust-sink",
                "--dataflow",
                &dataflow_id.to_string(),
            ])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "dora node stop failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            stdout.contains("stopped") && stdout.contains("rust-sink"),
            "expected stop confirmation for rust-sink, got: {stdout}"
        );

        cleanup(&dora);
    }

    /// `dora node restart` re-spawns a node; daemon increments restart_count.
    #[test]
    fn e2e_node_restart_increments_counter() {
        ensure_built();
        let dora = dora_bin();
        start_cluster(&dora);
        let _guard = ClusterGuard(&dora);
        let dataflow_id = start_lifecycle_dataflow_detached(&dora);

        let output = Command::new(&dora)
            .args([
                "node",
                "restart",
                "rust-node",
                "--dataflow",
                &dataflow_id.to_string(),
            ])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "dora node restart failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(
            stdout.to_lowercase().contains("restart"),
            "expected restart confirmation, got: {stdout}"
        );

        // Give the daemon time to kill and re-spawn the node, then poll
        // `dora node list` until rust-node re-appears. This confirms the
        // restart loop actually ran — if the node simply stopped and was
        // never re-spawned it would not show up in the list.
        let deadline = std::time::Instant::now() + Duration::from_secs(20);
        loop {
            let list_out = Command::new(&dora)
                .args(["node", "list", "--dataflow", &dataflow_id.to_string()])
                .output()
                .unwrap();
            if list_out.status.success() {
                let stdout = String::from_utf8_lossy(&list_out.stdout);
                if stdout.contains("rust-node") {
                    break;
                }
            }
            assert!(
                std::time::Instant::now() < deadline,
                "rust-node never re-appeared in node list within 20s after restart"
            );
            std::thread::sleep(Duration::from_millis(300));
        }

        cleanup(&dora);
    }

    /// dora-rs/dora#2980: `Destroy` must not outrun the stop it just asked
    /// for.
    ///
    /// `handle_destroy` stops every dataflow and then tears the daemons down
    /// ~200ms later, but `stop_dataflow` returns once the daemons acknowledge
    /// the *request* — the grace → SIGTERM → SIGKILL ladder runs afterwards,
    /// inside the daemon. A node that ignores the cooperative `Stop`
    /// therefore used to survive the teardown with `ppid 1`, holding whatever
    /// it held, while the command reported success.
    ///
    /// Driven over the raw control protocol rather than `dora down`: the
    /// #2924 guard makes the CLI stop each dataflow through a path that
    /// already waits, which masks this. `Destroy` is also reached by
    /// `dora cluster down`, by Ctrl-C on the coordinator, and by any direct
    /// control-protocol client — this is that route.
    ///
    /// The fixture ignores both the cooperative `Stop` and SIGTERM, so only a
    /// SIGKILL can end it; a node that died to SIGTERM would be cleaned up
    /// either way and could not tell a working teardown from a broken one.
    ///
    /// Unix-only: it inspects process state.
    #[test]
    #[cfg(unix)]
    fn e2e_destroy_does_not_orphan_a_node_that_ignores_stop() {
        ensure_built();
        let dora = dora_bin();
        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        let status = Command::new("cargo")
            .args(["build", "-p", "sigterm-ignoring-node"])
            .arg("--target-dir")
            .arg(&target)
            .status()
            .expect("failed to build sigterm-ignoring-node");
        assert!(status.success(), "failed to build sigterm-ignoring-node");

        let home = tempdir().expect("create isolated home");
        let port = unused_port();
        let pid_file = home.path().join("stubborn.pid");
        let yaml = home.path().join("stubborn.yml");
        std::fs::write(
            &yaml,
            format!(
                "nodes:\n  \
                 - id: stubborn\n    \
                   path: \"{node}\"\n    \
                   env:\n      \
                     DORA_TEST_PID_FILE: \"{pid_file}\"\n    \
                   inputs:\n      \
                     tick: dora/timer/millis/100\n",
                node = target.join("debug/sigterm-ignoring-node").display(),
                pid_file = pid_file.display(),
            ),
        )
        .expect("write dataflow yaml");

        let (status, stdout, stderr) = run_dora_isolated(&dora, home.path(), port, &["up"]);
        assert!(
            status.success(),
            "dora up failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        // Explicit capture name: the default is the joined argv, and this
        // command's argv holds a path, which cannot be a file name.
        let (child, out_path, err_path) = spawn_dora_isolated(
            &dora,
            home.path(),
            port,
            &["start", yaml.to_str().unwrap(), "--detach"],
            "start",
        );
        let (status, stdout, stderr) = wait_for_dora_isolated(child, out_path, err_path);
        assert!(
            status.success(),
            "dora start failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );

        // The fixture publishes its own pid, so the assertions below are about
        // THIS process rather than a pattern match on the process table, which
        // would also catch a leftover from an earlier run.
        let node_pid = wait_for_pid_file(&pid_file, Duration::from_secs(30));

        // RAII: the fixture ignores SIGTERM and outlives a failed assertion by
        // design, so every bail-out path must still clear it off the runner.
        struct Reaper(u32);
        impl Drop for Reaper {
            fn drop(&mut self) {
                if fixture_alive(self.0) {
                    let _ = Command::new("kill")
                        .args(["-9", &self.0.to_string()])
                        .status();
                }
            }
        }
        let _reaper = Reaper(node_pid);
        assert!(
            fixture_alive(node_pid),
            "fixture {node_pid} was not running before destroy"
        );

        let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
        let session = WsSession::connect(addr).expect("failed to connect WsSession");
        let reply =
            super::send_request(&session, &ControlRequest::Destroy).expect("destroy request");
        assert!(
            matches!(reply, ControlRequestReply::DestroyOk),
            "expected DestroyOk, got {reply:?}"
        );

        // No polling window: the point of the fix is that `Destroy` does not
        // return until the daemon has dealt with its children, so the node
        // must already be gone when the reply lands.
        assert!(
            !fixture_alive(node_pid),
            "node {node_pid} outlived the destroy that reported success \
             — it is now orphaned to ppid 1"
        );
    }

    /// #3004 review: the same orphan, one level down.
    ///
    /// A node's tracked process is often a wrapper — `uv run python node.py`,
    /// a shell launcher — and the ladder's SIGTERM kills the wrapper while
    /// the process it spawned ignores the signal and carries on in its
    /// process group. Watching only the tracked pid calls the node gone and
    /// lets the destroy finish, orphaning the real one.
    ///
    /// Unix-only: it uses process groups and inspects process state.
    #[test]
    #[cfg(unix)]
    fn e2e_destroy_does_not_orphan_a_node_behind_an_exiting_wrapper() {
        ensure_built();
        let dora = dora_bin();
        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        let status = Command::new("cargo")
            .args(["build", "-p", "sigterm-ignoring-node"])
            .arg("--target-dir")
            .arg(&target)
            .status()
            .expect("failed to build sigterm-ignoring-node");
        assert!(status.success(), "failed to build sigterm-ignoring-node");

        let home = tempdir().expect("create isolated home");
        let port = unused_port();
        let pid_file = home.path().join("stubborn.pid");
        let yaml = home.path().join("wrapped.yml");
        // `sh` waits, so the daemon's entry stays while the real node runs.
        // On SIGTERM `sh` dies and the fixture — which ignores it — does not.
        std::fs::write(
            &yaml,
            format!(
                "nodes:\n  \
                 - id: wrapped\n    \
                   path: /bin/sh\n    \
                   args: \"-c '{node} & wait'\"\n    \
                   env:\n      \
                     DORA_TEST_PID_FILE: \"{pid_file}\"\n    \
                   inputs:\n      \
                     tick: dora/timer/millis/100\n",
                node = target.join("debug/sigterm-ignoring-node").display(),
                pid_file = pid_file.display(),
            ),
        )
        .expect("write dataflow yaml");

        let (status, stdout, stderr) = run_dora_isolated(&dora, home.path(), port, &["up"]);
        assert!(
            status.success(),
            "dora up failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        let (child, out_path, err_path) = spawn_dora_isolated(
            &dora,
            home.path(),
            port,
            &["start", yaml.to_str().unwrap(), "--detach"],
            "start",
        );
        let (status, stdout, stderr) = wait_for_dora_isolated(child, out_path, err_path);
        assert!(
            status.success(),
            "dora start failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );

        let node_pid = wait_for_pid_file(&pid_file, Duration::from_secs(30));
        struct Reaper(u32);
        impl Drop for Reaper {
            fn drop(&mut self) {
                if fixture_alive(self.0) {
                    let _ = Command::new("kill")
                        .args(["-9", &self.0.to_string()])
                        .status();
                }
            }
        }
        let _reaper = Reaper(node_pid);
        assert!(
            fixture_alive(node_pid),
            "fixture {node_pid} was not running before destroy"
        );

        let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
        let session = WsSession::connect(addr).expect("failed to connect WsSession");
        let reply = super::send_request(&session, &ControlRequest::Destroy).expect("destroy");
        assert!(
            matches!(reply, ControlRequestReply::DestroyOk),
            "expected DestroyOk, got {reply:?}"
        );

        assert!(
            !fixture_alive(node_pid),
            "node {node_pid} outlived the destroy behind its wrapper — the \
             wrapper's exit must not be mistaken for the node's"
        );
    }

    /// A destroy must not make a healthy shutdown slower or dirtier.
    ///
    /// The wait added for #2980 cannot be taken inline in the daemon's event
    /// loop: a node on its way out ends by asking that same loop to close its
    /// outputs and report them done, so a loop parked in the wait deadlocks
    /// every well-behaved node against it until the deadline, and they get
    /// signal-killed instead of exiting. That regression is invisible to the
    /// orphan test above — its fixture never talks to the daemon — so this
    /// pins the other side: real nodes, and a destroy that returns in about
    /// the time it took before the wait existed.
    #[test]
    fn e2e_destroy_does_not_delay_healthy_nodes() {
        ensure_built();
        let dora = dora_bin();
        let home = tempdir().expect("create isolated home");
        let port = unused_port();
        let yaml = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/node-lifecycle.yml");

        let (status, stdout, stderr) = run_dora_isolated(&dora, home.path(), port, &["up"]);
        assert!(
            status.success(),
            "dora up failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );
        let (child, out_path, err_path) = spawn_dora_isolated(
            &dora,
            home.path(),
            port,
            &["start", yaml.to_str().unwrap(), "--detach"],
            "start",
        );
        let (status, stdout, stderr) = wait_for_dora_isolated(child, out_path, err_path);
        assert!(
            status.success(),
            "dora start failed; stdout:\n{stdout}\nstderr:\n{stderr}"
        );

        let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
        let session = WsSession::connect(addr).expect("failed to connect WsSession");

        // Destroying mid-startup is a different scenario with its own timing:
        // a node that has not subscribed yet cannot answer the cooperative
        // stop, so the teardown waits out the startup barrier. This test is
        // about the steady state, so let the dataflow reach it first.
        let deadline = Instant::now() + Duration::from_secs(30);
        loop {
            let reply = super::send_request(&session, &ControlRequest::List).expect("list");
            if let ControlRequestReply::DataflowList(list) = reply
                && list.0.iter().any(|entry| {
                    matches!(
                        entry.status,
                        dora_message::coordinator_to_cli::DataflowStatus::Running
                    )
                })
            {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "dataflow never reached Running within 30s"
            );
            std::thread::sleep(Duration::from_millis(100));
        }

        let reply = super::send_request(&session, &ControlRequest::Destroy).expect("destroy");
        assert!(
            matches!(reply, ControlRequestReply::DestroyOk),
            "expected DestroyOk, got {reply:?}"
        );

        // The signal, rather than wall-clock timing: the daemon only kills
        // what is left when its deadline expires, and it says so. A healthy
        // node reaches that deadline exactly when the loop it needs was
        // parked in the wait — so this line appearing IS the regression,
        // whatever the machine's timing.
        let killed = daemon_log_lines(home.path())
            .into_iter()
            .find(|line| line.contains("still running at destroy"));
        assert!(
            killed.is_none(),
            "nodes that answer the cooperative stop must exit on their own, \
             but the daemon had to kill them: {killed:?}"
        );
    }

    /// Everything the daemon logged, across the isolated home's log files.
    fn daemon_log_lines(home: &Path) -> Vec<String> {
        let mut lines = Vec::new();
        let mut dirs = vec![home.join(".dora")];
        while let Some(dir) = dirs.pop() {
            let Ok(entries) = std::fs::read_dir(&dir) else {
                continue;
            };
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    dirs.push(path);
                } else if let Ok(contents) = std::fs::read_to_string(&path) {
                    lines.extend(contents.lines().map(str::to_owned));
                }
            }
        }
        lines
    }

    /// Read the pid the fixture published, waiting for it to appear.
    #[cfg(unix)]
    fn wait_for_pid_file(path: &Path, timeout: Duration) -> u32 {
        let deadline = Instant::now() + timeout;
        loop {
            if let Ok(contents) = std::fs::read_to_string(path)
                && let Ok(pid) = contents.trim().parse::<u32>()
            {
                return pid;
            }
            assert!(
                Instant::now() < deadline,
                "node never reported its pid at {}",
                path.display()
            );
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// Whether `pid` is still the fixture.
    ///
    /// Matches the binary path, not merely liveness: once the node exits its
    /// pid is free for reuse, and a bare liveness check would eventually
    /// report an unrelated process as the surviving node. `args=`, not
    /// `comm=`, because Linux caps `comm` at 15 characters and the fixture
    /// name is longer (dora-rs/dora#2961).
    #[cfg(unix)]
    fn fixture_alive(pid: u32) -> bool {
        Command::new("ps")
            .args(["-ww", "-o", "args=", "-p", &pid.to_string()])
            .output()
            .ok()
            .is_some_and(|out| {
                String::from_utf8_lossy(&out.stdout).contains("/sigterm-ignoring-node")
            })
    }
}
