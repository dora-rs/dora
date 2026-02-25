//! Integration tests for the coordinator WebSocket daemon endpoint (/api/daemon).
//!
//! Simulates a daemon connecting, registering, and disconnecting.

mod common;

use adora_message::{
    daemon_to_coordinator::{CoordinatorRequest, DaemonRegisterRequest, Timestamped},
    ws_protocol::{WsRequest, WsResponse},
};
use futures::{SinkExt, StreamExt};
use serde_json::json;
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

/// Connect a client to the daemon endpoint.
async fn connect_daemon(
    port: u16,
) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
    let url = format!("ws://127.0.0.1:{port}/api/daemon");
    let (ws, _) = tokio_tungstenite::connect_async(&url)
        .await
        .expect("failed to connect to daemon WS");
    ws
}

/// Connect a client to the control endpoint.
async fn connect_control(
    port: u16,
) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
    let url = format!("ws://127.0.0.1:{port}/api/control");
    let (ws, _) = tokio_tungstenite::connect_async(&url)
        .await
        .expect("failed to connect to control WS");
    ws
}

/// Build a daemon Register WsRequest JSON string.
///
/// Constructs the full JSON string directly (bypassing `serde_json::Value`)
/// because `uhlc::ID(NonZeroU128)` can be serialized to a JSON string via
/// `serde_json::to_string` but not via `serde_json::to_value` (u128 out of range
/// for the `Value::Number` type). The coordinator deserializes from the raw
/// JSON text (`serde_json::from_str`), so this matches the real wire format.
fn make_register_request() -> (Uuid, String) {
    let id = Uuid::new_v4();
    let register =
        CoordinatorRequest::Register(DaemonRegisterRequest::new(Some("test-machine".into())));
    let timestamped = Timestamped {
        inner: register,
        timestamp: adora_message::uhlc::HLC::default().new_timestamp(),
    };
    // Build WsRequest-shaped JSON manually, embedding the Timestamped params
    // as a raw JSON fragment to preserve u128 fidelity.
    let params_json = serde_json::to_string(&timestamped).unwrap();
    let full_json = format!(r#"{{"id":"{id}","method":"daemon_request","params":{params_json}}}"#,);
    (id, full_json)
}

/// Send a control request and read the response.
async fn control_request_reply(
    ws: &mut (
             impl SinkExt<Message, Error = tokio_tungstenite::tungstenite::Error>
             + StreamExt<Item = Result<Message, tokio_tungstenite::tungstenite::Error>>
             + Unpin
         ),
    params: serde_json::Value,
) -> WsResponse {
    let id = Uuid::new_v4();
    let req = WsRequest {
        id,
        method: "control".into(),
        params,
    };
    ws.send(Message::Text(serde_json::to_string(&req).unwrap().into()))
        .await
        .unwrap();
    loop {
        let msg = ws.next().await.expect("stream ended").expect("ws error");
        if let Message::Text(text) = msg {
            let resp: WsResponse = serde_json::from_str(&text).unwrap();
            if resp.id == id {
                return resp;
            }
        }
    }
}

/// Send daemon register and poll DaemonConnected until it returns true (up to 2s).
async fn register_daemon_and_wait(
    port: u16,
) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
    let mut ws = connect_daemon(port).await;
    let (_id, json) = make_register_request();
    ws.send(Message::Text(json.into())).await.unwrap();

    // Poll until coordinator has processed the registration
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        let mut ctrl = connect_control(port).await;
        let params = serde_json::to_value(
            &adora_message::cli_to_coordinator::ControlRequest::DaemonConnected,
        )
        .unwrap();
        let resp = control_request_reply(&mut ctrl, params).await;
        if let Some(result) = &resp.result {
            if result.get("DaemonConnected") == Some(&json!(true)) {
                break;
            }
        }
        if tokio::time::Instant::now() > deadline {
            panic!("daemon did not register within 2s");
        }
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;
    }
    ws
}

#[tokio::test]
async fn daemon_register_success() {
    let (port, _handle) = common::start_test_coordinator().await;
    let _ws = register_daemon_and_wait(port).await;

    // Verify registration via a control connection
    let mut ctrl = connect_control(port).await;
    let params =
        serde_json::to_value(&adora_message::cli_to_coordinator::ControlRequest::DaemonConnected)
            .unwrap();
    let resp = control_request_reply(&mut ctrl, params).await;

    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    let connected = result.get("DaemonConnected").unwrap();
    assert_eq!(connected, &json!(true));
}

#[tokio::test]
async fn daemon_register_then_status() {
    let (port, _handle) = common::start_test_coordinator().await;
    let _ws = register_daemon_and_wait(port).await;

    let mut ctrl = connect_control(port).await;
    let params =
        serde_json::to_value(&adora_message::cli_to_coordinator::ControlRequest::ConnectedMachines)
            .unwrap();
    let resp = control_request_reply(&mut ctrl, params).await;

    assert!(resp.error.is_none());
    let result = resp.result.unwrap();
    let daemons = result.get("ConnectedDaemons").unwrap().as_array().unwrap();
    assert_eq!(daemons.len(), 1);
}

#[tokio::test]
async fn daemon_disconnect_cleanup() {
    let (port, _handle) = common::start_test_coordinator().await;

    // Register daemon, then close the connection
    {
        let mut ws = register_daemon_and_wait(port).await;
        ws.send(Message::Close(None)).await.unwrap();
    }

    // Poll until DaemonConnected returns false (up to 2s)
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        let mut ctrl = connect_control(port).await;
        let params = serde_json::to_value(
            &adora_message::cli_to_coordinator::ControlRequest::DaemonConnected,
        )
        .unwrap();
        let resp = control_request_reply(&mut ctrl, params).await;
        if let Some(result) = &resp.result {
            if result.get("DaemonConnected") == Some(&json!(false)) {
                break;
            }
        }
        if tokio::time::Instant::now() > deadline {
            panic!("daemon did not disconnect within 2s");
        }
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;
    }
}

#[tokio::test]
async fn daemon_heartbeat_pong() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_daemon(port).await;

    ws.send(Message::Ping(vec![42].into())).await.unwrap();

    let msg = ws.next().await.expect("stream ended").expect("ws error");
    match msg {
        Message::Pong(data) => assert_eq!(data.as_ref(), &[42]),
        other => panic!("expected Pong, got {other:?}"),
    }
}
