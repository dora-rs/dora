//! Integration tests for the coordinator WebSocket control plane.
//!
//! Each test spins up a real coordinator, connects via tokio-tungstenite,
//! and exercises the /api/control and /health endpoints.

mod common;

use dora_coordinator_store::{CoordinatorStore, DataflowRecord, DataflowStatus};
use dora_message::{
    BuildId,
    cli_to_coordinator::ControlRequest,
    ws_protocol::{WsRequest, WsResponse},
};
use futures::{SinkExt, StreamExt};
use serde_json::json;
use std::{collections::BTreeMap, sync::Arc};
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

/// Connect a tokio-tungstenite client to the control endpoint.
async fn connect_control(
    port: u16,
) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
    let url = format!("ws://127.0.0.1:{port}/api/control");
    let (ws, _) = tokio_tungstenite::connect_async(&url)
        .await
        .expect("failed to connect to control WS");
    ws
}

/// Send a WsRequest and read one WsResponse back.
async fn request_reply(
    ws: &mut (
             impl SinkExt<Message, Error = tokio_tungstenite::tungstenite::Error>
             + StreamExt<Item = Result<Message, tokio_tungstenite::tungstenite::Error>>
             + Unpin
         ),
    method: &str,
    params: serde_json::Value,
) -> WsResponse {
    let id = Uuid::new_v4();
    let req = WsRequest {
        id,
        method: method.into(),
        params,
    };
    let json = serde_json::to_string(&req).unwrap();
    ws.send(Message::Text(json.into())).await.unwrap();

    // Read next text message
    loop {
        let msg = ws.next().await.expect("stream ended").expect("ws error");
        if let Message::Text(text) = msg {
            let resp: WsResponse = serde_json::from_str(&text).unwrap();
            assert_eq!(resp.id, id, "response id should match request id");
            return resp;
        }
        // Skip non-text frames (pong, etc.)
    }
}

fn seed_dataflow_record(store: &dyn CoordinatorStore, dataflow_id: Uuid, node_ids: &[&str]) {
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

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn connect_and_health_check() {
    let (port, _handle) = common::start_test_coordinator().await;

    // Use raw TCP + HTTP/1.1 request (no reqwest dependency needed)
    let mut stream = tokio::net::TcpStream::connect(format!("127.0.0.1:{port}"))
        .await
        .unwrap();
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    stream
        .write_all(b"GET /health HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
        .await
        .unwrap();
    let mut buf = Vec::new();
    stream.read_to_end(&mut buf).await.unwrap();
    let response = String::from_utf8_lossy(&buf);
    assert!(response.contains("200"));
    assert!(response.ends_with("ok") || response.contains("\r\n\r\nok"));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_list_empty() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::List).unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    assert!(
        resp.error.is_none(),
        "expected ok, got error: {:?}",
        resp.error
    );
    let result = resp.result.unwrap();
    // DataflowList is serialized as {"DataflowList": [...]}
    let list = result
        .get("DataflowList")
        .expect("expected DataflowList key");
    assert!(list.as_array().unwrap().is_empty());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_invalid_json() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    ws.send(Message::Text("not json at all".into()))
        .await
        .unwrap();

    let msg = ws.next().await.expect("stream ended").expect("ws error");
    if let Message::Text(text) = msg {
        let resp: WsResponse = serde_json::from_str(&text).unwrap();
        assert!(resp.error.is_some());
        assert!(resp.error.unwrap().contains("invalid request"));
    } else {
        panic!("expected text frame");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_invalid_params() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let resp = request_reply(&mut ws, "control", json!({"not_a_variant": true})).await;
    assert!(resp.error.is_some());
    assert!(resp.error.unwrap().contains("invalid params"));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_destroy_nonexistent() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::Destroy).unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    // Destroy with no daemons connected returns DestroyOk
    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    assert_eq!(result, json!("DestroyOk"));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_status() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::DaemonConnected).unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    // DaemonConnected(false) when no daemons connected
    let connected = result.get("DaemonConnected").unwrap();
    assert_eq!(connected, &json!(false));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_ping_pong() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    ws.send(Message::Ping(vec![1, 2, 3].into())).await.unwrap();

    let msg = ws.next().await.expect("stream ended").expect("ws error");
    match msg {
        Message::Pong(data) => assert_eq!(data.as_ref(), &[1, 2, 3]),
        other => panic!("expected Pong, got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_concurrent_requests() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let mut ids = Vec::new();
    let params = serde_json::to_value(&ControlRequest::List).unwrap();

    // Send 10 requests without reading responses
    for _ in 0..10 {
        let id = Uuid::new_v4();
        let req = WsRequest {
            id,
            method: "control".into(),
            params: params.clone(),
        };
        ws.send(Message::Text(serde_json::to_string(&req).unwrap().into()))
            .await
            .unwrap();
        ids.push(id);
    }

    // Read all 10 responses (order may differ)
    let mut received_ids = std::collections::HashSet::new();
    for _ in 0..10 {
        let msg = ws.next().await.expect("stream ended").expect("ws error");
        if let Message::Text(text) = msg {
            let resp: WsResponse = serde_json::from_str(&text).unwrap();
            assert!(resp.error.is_none());
            received_ids.insert(resp.id);
        }
    }

    for id in &ids {
        assert!(
            received_ids.contains(id),
            "missing response for request {id}"
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_connection_close() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    // Verify connection works
    let params = serde_json::to_value(&ControlRequest::List).unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());

    // Send close
    ws.send(Message::Close(None)).await.unwrap();

    // Read until stream ends
    while let Some(msg) = ws.next().await {
        match msg {
            Ok(Message::Close(_)) => break,
            Ok(_) => continue,
            Err(_) => break,
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn log_subscribe_nonexistent() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let fake_id = Uuid::new_v4();
    let params = serde_json::to_value(&ControlRequest::LogSubscribe {
        dataflow_id: fake_id,
        level: log::LevelFilter::Info,
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    assert!(resp.error.is_some());
    let err = resp.error.unwrap();
    assert!(
        err.contains(&fake_id.to_string()),
        "error should mention dataflow id"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn control_error_does_not_leak_internal_chain() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    // LogSubscribe with a fake dataflow_id triggers an eyre error internally.
    // The sanitized error should NOT contain eyre chain markers.
    let fake_id = Uuid::new_v4();
    let params = serde_json::to_value(&ControlRequest::LogSubscribe {
        dataflow_id: fake_id,
        level: log::LevelFilter::Info,
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    let err = resp.error.expect("should be an error");
    // Sanitized errors should not contain eyre chain separators or newlines
    assert!(
        !err.contains('\n'),
        "error should not contain newlines (eyre chain): {err}"
    );
    assert!(
        !err.contains("Caused by:"),
        "error should not expose internal cause chain: {err}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn build_log_subscribe_nonexistent() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let fake_id = BuildId::generate();
    let params = serde_json::to_value(&ControlRequest::BuildLogSubscribe {
        build_id: fake_id,
        level: log::LevelFilter::Debug,
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    assert!(resp.error.is_some());
    let err = resp.error.unwrap();
    assert!(!err.is_empty(), "error should be descriptive: {err}");
}

// -- Phase 2: Node restart/stop error paths --

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn restart_node_nonexistent_dataflow() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let fake_id = Uuid::new_v4();
    let params = serde_json::to_value(&ControlRequest::RestartNode {
        dataflow_id: fake_id,
        node_id: "camera".to_string().into(),
        grace_duration: None,
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    // Error comes back as ControlRequestReply::Error in the `result` field
    // (not WsResponse `error`), because the coordinator wraps eyre errors
    // into ControlRequestReply::Error via format_response_json.
    let result = resp.result.expect("expected a result");
    let err = result
        .get("Error")
        .expect("expected Error variant in result");
    assert!(
        err.as_str().unwrap().contains(&fake_id.to_string())
            || err.as_str().unwrap().to_lowercase().contains("not found"),
        "error should mention the dataflow: {err}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn stop_node_nonexistent_dataflow() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let fake_id = Uuid::new_v4();
    let params = serde_json::to_value(&ControlRequest::StopNode {
        dataflow_id: fake_id,
        node_id: "sensor".to_string().into(),
        grace_duration: None,
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    let result = resp.result.expect("expected a result");
    assert!(result.get("Error").is_some(), "expected Error variant");
}

// -- Phase 3: Parameter CRUD via coordinator --

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_list_empty() {
    let store_impl = Arc::new(dora_coordinator::InMemoryStore::new());
    let dataflow_id = Uuid::new_v4();
    seed_dataflow_record(store_impl.as_ref(), dataflow_id, &["camera"]);
    let store: Arc<dyn CoordinatorStore> = store_impl;
    let (port, _handle) = common::start_test_coordinator_with_store(store).await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::GetParams {
        dataflow_id,
        node_id: "camera".to_string().into(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;

    assert!(resp.error.is_none(), "expected ok: {:?}", resp.error);
    let result = resp.result.unwrap();
    let param_list = result.get("ParamList").expect("expected ParamList key");
    let params_arr = param_list.get("params").unwrap().as_array().unwrap();
    assert!(params_arr.is_empty());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_set_then_get() {
    let store_impl = Arc::new(dora_coordinator::InMemoryStore::new());
    let df_id = Uuid::new_v4();
    seed_dataflow_record(store_impl.as_ref(), df_id, &["sensor"]);
    let store: Arc<dyn CoordinatorStore> = store_impl;
    let (port, _handle) = common::start_test_coordinator_with_store(store).await;
    let mut ws = connect_control(port).await;

    let node_id: dora_message::id::NodeId = "sensor".to_string().into();

    let params = serde_json::to_value(&ControlRequest::SetParam {
        dataflow_id: df_id,
        node_id: node_id.clone(),
        key: "threshold".into(),
        value: json!(42),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none(), "set failed: {:?}", resp.error);
    assert_eq!(resp.result.unwrap(), json!("ParamSet"));

    let params = serde_json::to_value(&ControlRequest::GetParam {
        dataflow_id: df_id,
        node_id,
        key: "threshold".into(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none(), "get failed: {:?}", resp.error);
    let result = resp.result.unwrap();
    let pv = result.get("ParamValue").expect("expected ParamValue key");
    assert_eq!(pv.get("key").unwrap(), "threshold");
    assert_eq!(pv.get("value").unwrap(), &json!(42));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_set_list_delete() {
    let store_impl = Arc::new(dora_coordinator::InMemoryStore::new());
    let df_id = Uuid::new_v4();
    seed_dataflow_record(store_impl.as_ref(), df_id, &["camera"]);
    let store: Arc<dyn CoordinatorStore> = store_impl;
    let (port, _handle) = common::start_test_coordinator_with_store(store).await;
    let mut ws = connect_control(port).await;

    let node_id: dora_message::id::NodeId = "camera".to_string().into();

    for (key, value) in [("fps", json!(30)), ("resolution", json!("1080p"))] {
        let params = serde_json::to_value(&ControlRequest::SetParam {
            dataflow_id: df_id,
            node_id: node_id.clone(),
            key: key.into(),
            value,
        })
        .unwrap();
        let resp = request_reply(&mut ws, "control", params).await;
        assert!(
            resp.error.is_none(),
            "set failed for {key}: {:?}",
            resp.error
        );
        assert_eq!(resp.result.unwrap(), json!("ParamSet"));
    }

    let params = serde_json::to_value(&ControlRequest::GetParams {
        dataflow_id: df_id,
        node_id: node_id.clone(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    let arr = result
        .get("ParamList")
        .unwrap()
        .get("params")
        .unwrap()
        .as_array()
        .unwrap();
    assert_eq!(arr.len(), 2);

    let params = serde_json::to_value(&ControlRequest::DeleteParam {
        dataflow_id: df_id,
        node_id: node_id.clone(),
        key: "fps".into(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());
    assert_eq!(resp.result.unwrap(), json!("ParamDeleted"));

    let params = serde_json::to_value(&ControlRequest::GetParams {
        dataflow_id: df_id,
        node_id: node_id.clone(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    let result = resp.result.unwrap();
    let arr = result
        .get("ParamList")
        .unwrap()
        .get("params")
        .unwrap()
        .as_array()
        .unwrap();
    assert_eq!(arr.len(), 1);
    assert_eq!(arr[0].as_array().unwrap()[0], "resolution");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_set_rejects_unknown_target() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::SetParam {
        dataflow_id: Uuid::new_v4(),
        node_id: "sensor".to_string().into(),
        key: "threshold".into(),
        value: json!(42),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    assert!(result.get("Error").is_some());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_delete_rejects_unknown_target() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::DeleteParam {
        dataflow_id: Uuid::new_v4(),
        node_id: "camera".to_string().into(),
        key: "fps".into(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    assert!(result.get("Error").is_some());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_get_nonexistent() {
    let store_impl = Arc::new(dora_coordinator::InMemoryStore::new());
    let dataflow_id = Uuid::new_v4();
    seed_dataflow_record(store_impl.as_ref(), dataflow_id, &["node"]);
    let store: Arc<dyn CoordinatorStore> = store_impl;
    let (port, _handle) = common::start_test_coordinator_with_store(store).await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::GetParam {
        dataflow_id,
        node_id: "node".to_string().into(),
        key: "missing".into(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    assert!(
        result.get("Error").is_some(),
        "expected Error for missing param, got {result:?}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_get_rejects_unknown_target() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::GetParam {
        dataflow_id: Uuid::new_v4(),
        node_id: "ghost".to_string().into(),
        key: "missing".into(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    assert!(
        result.get("Error").is_some(),
        "expected Error for unknown target, got {result:?}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn param_list_rejects_unknown_target() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let params = serde_json::to_value(&ControlRequest::GetParams {
        dataflow_id: Uuid::new_v4(),
        node_id: "ghost".to_string().into(),
    })
    .unwrap();
    let resp = request_reply(&mut ws, "control", params).await;
    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    assert!(
        result.get("Error").is_some(),
        "expected Error for unknown target, got {result:?}"
    );
}

// NOTE: TopicPublish integration testing requires Zenoh with a multi-thread
// tokio runtime, which is incompatible with the test coordinator's current
// thread runtime. TopicPublish is tested via the ws-cli-e2e tests instead.
