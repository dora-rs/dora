//! Integration tests for the coordinator WebSocket daemon endpoint (/api/daemon).
//!
//! Simulates a daemon connecting, registering, and disconnecting.

mod common;

use dora_message::{
    coordinator_to_daemon::RegisterResult,
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
/// Constructs the full JSON string directly, exactly as the daemon does in
/// `dora_daemon::coordinator::register`, so the test exercises the real wire
/// format rather than a `serde_json::Value` re-encoding of it.
fn make_register_request() -> (Uuid, String) {
    make_register_request_for("test-machine", None)
}

fn make_register_request_for(machine: &str, zenoh_endpoint: Option<&str>) -> (Uuid, String) {
    let id = Uuid::new_v4();
    let register = CoordinatorRequest::Register(DaemonRegisterRequest::with_zenoh_endpoint(
        Some(machine.into()),
        Default::default(),
        zenoh_endpoint.map(str::to_owned),
    ));
    let timestamped = Timestamped {
        inner: register,
        timestamp: dora_message::uhlc::HLC::default().new_timestamp(),
    };
    // Build WsRequest-shaped JSON manually, embedding the Timestamped params
    // as a raw JSON fragment.
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

/// Register a daemon and return the peer zenoh endpoints its register reply
/// carries — what the daemon would dial.
async fn register_and_read_peers(
    port: u16,
    machine: &str,
    zenoh_endpoint: &str,
) -> (
    tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>,
    Vec<String>,
) {
    let mut ws = connect_daemon(port).await;
    let (_id, json) = make_register_request_for(machine, Some(zenoh_endpoint));
    ws.send(Message::Text(json.into())).await.unwrap();
    // The reply is a `daemon_event` request carrying `Timestamped<RegisterResult>`,
    // as `dora_daemon::coordinator::register` reads it.
    loop {
        let msg = ws.next().await.expect("stream ended").expect("ws error");
        let Message::Text(text) = msg else { continue };
        let Ok(req) = serde_json::from_str::<WsRequest>(&text) else {
            continue;
        };
        if req.method != "daemon_event" {
            continue;
        }
        let reply: Timestamped<RegisterResult> = serde_json::from_value(req.params).unwrap();
        let (_daemon_id, peers) = reply.inner.into_parts().expect("registration accepted");
        return (ws, peers);
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
            &dora_message::cli_to_coordinator::ControlRequest::DaemonConnected,
        )
        .unwrap();
        let resp = control_request_reply(&mut ctrl, params).await;
        if let Some(result) = &resp.result
            && result.get("DaemonConnected") == Some(&json!(true))
        {
            break;
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
        serde_json::to_value(&dora_message::cli_to_coordinator::ControlRequest::DaemonConnected)
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
        serde_json::to_value(&dora_message::cli_to_coordinator::ControlRequest::ConnectedMachines)
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
            &dora_message::cli_to_coordinator::ControlRequest::DaemonConnected,
        )
        .unwrap();
        let resp = control_request_reply(&mut ctrl, params).await;
        if let Some(result) = &resp.result
            && result.get("DaemonConnected") == Some(&json!(false))
        {
            break;
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

/// Two daemons that both reach the coordinator over loopback run on its
/// host, so the second one's register reply must carry the first one's
/// loopback zenoh endpoint: that is how two same-host daemons link without
/// multicast (the `multiple-daemons` example, `dora up` plus a second daemon).
/// The first daemon, registering into an empty registry, is handed nothing.
#[tokio::test]
async fn same_host_daemons_are_handed_each_others_loopback_endpoints() {
    let (port, _handle) = common::start_test_coordinator().await;

    let (_ws_a, peers_a) = register_and_read_peers(port, "A", "tcp/127.0.0.1:45001").await;
    assert!(
        peers_a.is_empty(),
        "first daemon has no peers yet: {peers_a:?}"
    );

    let (_ws_b, peers_b) = register_and_read_peers(port, "B", "tcp/127.0.0.1:45002").await;
    assert_eq!(peers_b, ["tcp/127.0.0.1:45001"]);

    let (_ws_c, mut peers_c) = register_and_read_peers(port, "C", "tcp/127.0.0.1:45003").await;
    peers_c.sort();
    assert_eq!(peers_c, ["tcp/127.0.0.1:45001", "tcp/127.0.0.1:45002"]);
}

/// Read frames until the register reply (`params.inner.Ok`) arrives.
async fn read_register_reply(
    ws: &mut (impl StreamExt<Item = Result<Message, tokio_tungstenite::tungstenite::Error>> + Unpin),
) -> serde_json::Value {
    let read = async {
        loop {
            let msg = ws.next().await.expect("stream ended").expect("ws error");
            if let Message::Text(text) = msg {
                let value: serde_json::Value = serde_json::from_str(&text).unwrap();
                if let Some(ok) = value.pointer("/params/inner/Ok") {
                    return ok.clone();
                }
            }
        }
    };
    tokio::time::timeout(std::time::Duration::from_secs(5), read)
        .await
        .expect("no register reply within 5s")
}

/// The coordinator offers binary topic debug frames at registration, and a
/// binary frame from the registered daemon is accepted without disturbing the
/// connection (dora-rs/dora#3535).
#[tokio::test]
async fn daemon_register_offers_binary_debug_frames_and_accepts_them() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = connect_daemon(port).await;
    let (_id, json) = make_register_request();
    ws.send(Message::Text(json.into())).await.unwrap();

    let reply = read_register_reply(&mut ws).await;
    assert_eq!(reply.get("binary_debug_frames"), Some(&json!(true)));

    // A frame for a dataflow that is not running is simply dropped by the
    // coordinator; what matters is that the socket stays usable afterwards.
    // 4 MiB: a camera-sized output, well past the 1 MiB control-message limit
    // the daemon socket used to apply to every message.
    let frame = dora_message::daemon_to_coordinator::encode_topic_debug_frame(
        Uuid::new_v4(),
        &[Uuid::new_v4()],
        &vec![0u8; 4 * 1024 * 1024],
    )
    .unwrap();
    ws.send(Message::Binary(frame.into())).await.unwrap();

    ws.send(Message::Ping(vec![7].into())).await.unwrap();
    let pong = tokio::time::timeout(std::time::Duration::from_secs(5), async {
        loop {
            match ws.next().await.expect("stream ended").expect("ws error") {
                Message::Pong(data) => return data,
                Message::Close(_) => panic!("coordinator closed the connection on a binary frame"),
                _ => continue,
            }
        }
    })
    .await
    .expect("no pong within 5s");
    assert_eq!(pong.as_ref(), &[7]);
}

/// Raising the daemon socket's size limit for binary frames must not raise it
/// for text: an oversized text message still closes the connection.
#[tokio::test]
async fn daemon_oversized_text_message_still_closes_the_connection() {
    let (port, _handle) = common::start_test_coordinator().await;
    let mut ws = register_daemon_and_wait(port).await;

    let oversized =
        "x".repeat(dora_message::daemon_to_coordinator::MAX_DAEMON_TEXT_MESSAGE_BYTES + 1);
    // The coordinator may reset the socket while this is still being written.
    let _ = ws.send(Message::Text(oversized.into())).await;

    let closed = tokio::time::timeout(std::time::Duration::from_secs(5), async {
        loop {
            match ws.next().await {
                None | Some(Err(_)) | Some(Ok(Message::Close(_))) => return,
                Some(Ok(_)) => continue,
            }
        }
    })
    .await;
    assert!(
        closed.is_ok(),
        "coordinator kept the connection open after an oversized text message"
    );
}
