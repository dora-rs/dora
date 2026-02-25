//! Integration tests for the coordinator WebSocket control plane.
//!
//! Each test spins up a real coordinator, connects via tokio-tungstenite,
//! and exercises the /api/control and /health endpoints.

mod common;

use adora_message::{
    BuildId,
    cli_to_coordinator::ControlRequest,
    ws_protocol::{WsRequest, WsResponse},
};
use futures::{SinkExt, StreamExt};
use serde_json::json;
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

#[tokio::test]
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

#[tokio::test]
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

#[tokio::test]
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

#[tokio::test]
async fn control_invalid_params() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_control(port).await;

    let resp = request_reply(&mut ws, "control", json!({"not_a_variant": true})).await;
    assert!(resp.error.is_some());
    assert!(resp.error.unwrap().contains("invalid params"));
}

#[tokio::test]
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

#[tokio::test]
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

#[tokio::test]
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

#[tokio::test]
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

#[tokio::test]
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

#[tokio::test]
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

#[tokio::test]
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
