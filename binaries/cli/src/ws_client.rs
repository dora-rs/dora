//! WebSocket client for CLI-to-coordinator communication.
//!
//! Replaces `TcpRequestReplyConnection` with a single WS connection that handles
//! both request-reply and log streaming.

use adora_message::ws_protocol::WsRequest;
use eyre::{Context, eyre};
use futures::{SinkExt, StreamExt};
use std::{collections::HashMap, net::SocketAddr, sync::mpsc as std_mpsc};
use tokio::sync::{mpsc, oneshot};
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

/// Helper for deserializing incoming WS frames without going through
/// `serde_json::Value` for the result/payload fields. This preserves
/// u128 fidelity for uhlc::ID inside timestamps.
#[derive(serde::Deserialize)]
struct IncomingFrame {
    #[serde(default)]
    id: Option<Uuid>,
    #[serde(default)]
    event: Option<String>,
    #[serde(default)]
    result: Option<Box<serde_json::value::RawValue>>,
    #[serde(default)]
    error: Option<String>,
    #[serde(default)]
    payload: Option<Box<serde_json::value::RawValue>>,
}

/// A WebSocket session to the coordinator.
///
/// Provides synchronous `request()` for request-reply and `subscribe_logs()`
/// for streaming log events, both over the same WS connection.
pub struct WsSession {
    rt: tokio::runtime::Runtime,
    cmd_tx: mpsc::UnboundedSender<SessionCommand>,
}

enum SessionCommand {
    /// Send a request and wait for a response.
    Request {
        data: Vec<u8>,
        reply: oneshot::Sender<eyre::Result<Vec<u8>>>,
    },
    /// Subscribe to log/build-log events.
    SubscribeLogs {
        request: Vec<u8>,
        log_tx: std_mpsc::Sender<eyre::Result<Vec<u8>>>,
        ack_tx: oneshot::Sender<eyre::Result<()>>,
    },
    /// Subscribe to topic data via binary WS frames.
    SubscribeTopics {
        request: Vec<u8>,
        data_tx: std_mpsc::Sender<eyre::Result<Vec<u8>>>,
        ack_tx: oneshot::Sender<eyre::Result<Uuid>>,
    },
}

impl WsSession {
    /// Connect to the coordinator via WebSocket.
    ///
    /// If called from within an existing tokio runtime, uses that runtime.
    /// Otherwise creates a new single-threaded runtime.
    pub fn connect(addr: SocketAddr) -> eyre::Result<Self> {
        if tokio::runtime::Handle::try_current().is_ok() {
            eyre::bail!(
                "WsSession::connect must not be called from within an async context; \
                 use an async-native client instead"
            );
        }
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .context("failed to create tokio runtime for WS session")?;

        let ws_url = format!("ws://{addr}/api/control");
        let ws_stream = rt
            .block_on(async {
                use tokio_tungstenite::tungstenite;
                let mut request = tungstenite::http::Request::builder()
                    .uri(&ws_url)
                    .header("Host", addr.to_string())
                    .header("Connection", "Upgrade")
                    .header("Upgrade", "websocket")
                    .header(
                        "Sec-WebSocket-Key",
                        tungstenite::handshake::client::generate_key(),
                    )
                    .header("Sec-WebSocket-Version", "13");
                if let Some(token) = adora_message::auth::discover_token() {
                    request = request.header("Authorization", format!("Bearer {}", token.as_hex()));
                }
                let request = request.body(()).expect("failed to build WS request");
                tokio_tungstenite::connect_async(request).await
            })
            .map_err(|e| {
                let msg = e.to_string();
                if msg.to_lowercase().contains("connection refused")
                    || msg.contains("No connection could be made")
                {
                    eyre!(
                        "cannot connect to coordinator at {addr}: {msg}\n\n  \
                         hint: is the coordinator running? Start it with `adora up`"
                    )
                } else if msg.contains("401") || msg.contains("Unauthorized") {
                    eyre!(
                        "authentication failed connecting to coordinator at {addr}: {msg}\n\n  \
                         The coordinator was started with --auth and requires a valid token.\n  \
                         The token is stored in ~/.config/adora/.adora-token\n\n  \
                         Possible fixes:\n  \
                         - Run `adora down && adora up` to regenerate the token\n  \
                         - Ensure you're using the same user that started the coordinator\n  \
                         - Set ADORA_AUTH_TOKEN env var to match the coordinator's token\n  \
                         - Restart without --auth to disable authentication"
                    )
                } else {
                    eyre!("failed to connect to coordinator at {addr}: {msg}")
                }
            })?
            .0;

        let (cmd_tx, cmd_rx) = mpsc::unbounded_channel();
        rt.spawn(session_loop(ws_stream, cmd_rx));

        Ok(Self { rt, cmd_tx })
    }

    /// Send a request and wait synchronously for the reply.
    ///
    /// `data` should be a serialized `ControlRequest`.
    /// Returns the serialized `ControlRequestReply`.
    pub fn request(&self, data: &[u8]) -> eyre::Result<Vec<u8>> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.cmd_tx
            .send(SessionCommand::Request {
                data: data.to_vec(),
                reply: reply_tx,
            })
            .map_err(|_| eyre!("WS session closed"))?;

        self.rt
            .block_on(reply_rx)
            .map_err(|_| eyre!("WS session dropped reply"))?
    }

    /// Subscribe to topic data via the coordinator's Zenoh proxy.
    ///
    /// Sends a `TopicSubscribe` request, waits for the ack, then returns
    /// a `(subscription_id, receiver)` pair. Binary WS frames with matching
    /// subscription UUID prefix are dispatched to the receiver.
    pub fn subscribe_topics(
        &self,
        dataflow_id: Uuid,
        topics: Vec<(adora_message::id::NodeId, adora_message::id::DataId)>,
    ) -> eyre::Result<(Uuid, std_mpsc::Receiver<eyre::Result<Vec<u8>>>)> {
        let request = serde_json::to_vec(
            &adora_message::cli_to_coordinator::ControlRequest::TopicSubscribe {
                dataflow_id,
                topics,
            },
        )
        .map_err(|e| eyre!("failed to serialize TopicSubscribe: {e}"))?;

        let (data_tx, data_rx) = std_mpsc::channel();
        let (ack_tx, ack_rx) = oneshot::channel();
        self.cmd_tx
            .send(SessionCommand::SubscribeTopics {
                request,
                data_tx,
                ack_tx,
            })
            .map_err(|_| eyre!("WS session closed"))?;

        let subscription_id = self
            .rt
            .block_on(ack_rx)
            .map_err(|_| eyre!("WS session dropped ack"))??;

        Ok((subscription_id, data_rx))
    }

    /// Subscribe to log events on this connection.
    ///
    /// Sends the subscribe request (LogSubscribe or BuildLogSubscribe),
    /// waits for the ack, then returns a receiver for log event payloads.
    ///
    /// Each received item is the serialized `LogMessage`.
    pub fn subscribe_logs(
        &self,
        request: &[u8],
    ) -> eyre::Result<std_mpsc::Receiver<eyre::Result<Vec<u8>>>> {
        let (log_tx, log_rx) = std_mpsc::channel();
        let (ack_tx, ack_rx) = oneshot::channel();
        self.cmd_tx
            .send(SessionCommand::SubscribeLogs {
                request: request.to_vec(),
                log_tx,
                ack_tx,
            })
            .map_err(|_| eyre!("WS session closed"))?;

        self.rt
            .block_on(ack_rx)
            .map_err(|_| eyre!("WS session dropped ack"))??;

        Ok(log_rx)
    }
}

type WsStream =
    tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;
type PendingRequests = HashMap<Uuid, oneshot::Sender<eyre::Result<Vec<u8>>>>;
type PendingSubscribes = HashMap<
    Uuid,
    (
        oneshot::Sender<eyre::Result<()>>,
        std_mpsc::Sender<eyre::Result<Vec<u8>>>,
    ),
>;
type PendingTopicSubscribes = HashMap<
    Uuid,
    (
        oneshot::Sender<eyre::Result<Uuid>>,
        std_mpsc::Sender<eyre::Result<Vec<u8>>>,
    ),
>;
type TopicSubscribers = HashMap<Uuid, std_mpsc::Sender<eyre::Result<Vec<u8>>>>;

async fn session_loop(ws_stream: WsStream, mut cmd_rx: mpsc::UnboundedReceiver<SessionCommand>) {
    let (mut ws_tx, mut ws_rx) = ws_stream.split();
    let mut pending_requests: PendingRequests = HashMap::new();
    let mut pending_subscribes: PendingSubscribes = HashMap::new();
    let mut log_subscribers: Vec<std_mpsc::Sender<eyre::Result<Vec<u8>>>> = Vec::new();
    let mut pending_topic_subscribes: PendingTopicSubscribes = HashMap::new();
    let mut topic_subscribers: TopicSubscribers = HashMap::new();

    loop {
        tokio::select! {
            Some(cmd) = cmd_rx.recv() => {
                match cmd {
                    SessionCommand::Request { data, reply } => {
                        let id = Uuid::new_v4();
                        let params = match serde_json::from_slice(&data) {
                            Ok(v) => v,
                            Err(e) => {
                                let _ = reply.send(Err(eyre!("failed to parse request: {e}")));
                                continue;
                            }
                        };
                        let req = WsRequest {
                            id,
                            method: "control".to_string(),
                            params,
                        };
                        let json = match serde_json::to_string(&req) {
                            Ok(j) => j,
                            Err(e) => {
                                let _ = reply.send(Err(eyre!("failed to serialize WsRequest: {e}")));
                                continue;
                            }
                        };
                        pending_requests.insert(id, reply);
                        if ws_tx.send(Message::Text(json.into())).await.is_err() {
                            break;
                        }
                    }
                    SessionCommand::SubscribeLogs { request, log_tx, ack_tx } => {
                        let id = Uuid::new_v4();
                        let params = match serde_json::from_slice(&request) {
                            Ok(v) => v,
                            Err(e) => {
                                let _ = ack_tx.send(Err(eyre!("failed to parse subscribe request: {e}")));
                                continue;
                            }
                        };
                        let req = WsRequest {
                            id,
                            method: "control".to_string(),
                            params,
                        };
                        let json = match serde_json::to_string(&req) {
                            Ok(j) => j,
                            Err(e) => {
                                let _ = ack_tx.send(Err(eyre!("failed to serialize WsRequest: {e}")));
                                continue;
                            }
                        };
                        pending_subscribes.insert(id, (ack_tx, log_tx));
                        if ws_tx.send(Message::Text(json.into())).await.is_err() {
                            break;
                        }
                    }
                    SessionCommand::SubscribeTopics { request, data_tx, ack_tx } => {
                        let id = Uuid::new_v4();
                        let params = match serde_json::from_slice(&request) {
                            Ok(v) => v,
                            Err(e) => {
                                let _ = ack_tx.send(Err(eyre!("failed to parse topic subscribe request: {e}")));
                                continue;
                            }
                        };
                        let req = WsRequest {
                            id,
                            method: "control".to_string(),
                            params,
                        };
                        let json = match serde_json::to_string(&req) {
                            Ok(j) => j,
                            Err(e) => {
                                let _ = ack_tx.send(Err(eyre!("failed to serialize WsRequest: {e}")));
                                continue;
                            }
                        };
                        pending_topic_subscribes.insert(id, (ack_tx, data_tx));
                        if ws_tx.send(Message::Text(json.into())).await.is_err() {
                            break;
                        }
                    }
                }
            }
            msg = ws_rx.next() => {
                let Some(msg) = msg else { break };
                match msg {
                    Ok(Message::Text(text)) => {
                        let frame: IncomingFrame = match serde_json::from_str(&text) {
                            Ok(m) => m,
                            Err(e) => {
                                tracing::warn!("failed to parse WS message: {e}");
                                continue;
                            }
                        };

                        if let Some(event_name) = &frame.event {
                            if event_name == "log" {
                                if let Some(payload) = &frame.payload {
                                    let bytes = payload.get().as_bytes().to_vec();
                                    log_subscribers.retain(|tx| tx.send(Ok(bytes.clone())).is_ok());
                                }
                            }
                        } else if let Some(id) = frame.id {
                            handle_response(
                                id,
                                frame.result,
                                frame.error,
                                &mut pending_requests,
                                &mut pending_subscribes,
                                &mut log_subscribers,
                                &mut pending_topic_subscribes,
                                &mut topic_subscribers,
                            );
                        }
                    }
                    Ok(Message::Binary(data)) => {
                        // Binary frame: first 16 bytes = subscription UUID, rest = payload
                        if data.len() < 16 {
                            tracing::warn!("binary WS frame too short ({} bytes)", data.len());
                            continue;
                        }
                        let Ok(sub_id_bytes): Result<[u8; 16], _> = data[..16].try_into() else {
                            continue;
                        };
                        let sub_id = Uuid::from_bytes(sub_id_bytes);
                        let payload = data[16..].to_vec();
                        if let Some(tx) = topic_subscribers.get(&sub_id) {
                            if tx.send(Ok(payload)).is_err() {
                                topic_subscribers.remove(&sub_id);
                            }
                        }
                    }
                    Ok(Message::Close(_)) => break,
                    Ok(Message::Ping(data)) => {
                        let _ = ws_tx.send(Message::Pong(data)).await;
                    }
                    Ok(other) => {
                        tracing::trace!("ignoring unexpected WS message type: {other:?}");
                    }
                    Err(_) => break,
                }
            }
        }
    }

    // Clean up: notify pending requests of disconnect
    for (_, reply) in pending_requests.drain() {
        let _ = reply.send(Err(eyre!("WS connection closed")));
    }
    for (_, (ack, _)) in pending_subscribes.drain() {
        let _ = ack.send(Err(eyre!("WS connection closed")));
    }
    for (_, (ack, _)) in pending_topic_subscribes.drain() {
        let _ = ack.send(Err(eyre!("WS connection closed")));
    }
}

#[allow(clippy::too_many_arguments)]
fn handle_response(
    id: Uuid,
    result: Option<Box<serde_json::value::RawValue>>,
    error: Option<String>,
    pending_requests: &mut PendingRequests,
    pending_subscribes: &mut PendingSubscribes,
    log_subscribers: &mut Vec<std_mpsc::Sender<eyre::Result<Vec<u8>>>>,
    pending_topic_subscribes: &mut PendingTopicSubscribes,
    topic_subscribers: &mut TopicSubscribers,
) {
    // Check if this is a log subscribe ack
    if let Some((ack_tx, log_tx)) = pending_subscribes.remove(&id) {
        if let Some(error) = error {
            let _ = ack_tx.send(Err(eyre!("{error}")));
        } else {
            log_subscribers.push(log_tx);
            let _ = ack_tx.send(Ok(()));
        }
        return;
    }

    // Check if this is a topic subscribe ack
    if let Some((ack_tx, data_tx)) = pending_topic_subscribes.remove(&id) {
        if let Some(error) = error {
            let _ = ack_tx.send(Err(eyre!("{error}")));
        } else if let Some(raw) = &result {
            // Parse TopicSubscribed { subscription_id } from the result
            let reply: Result<adora_message::coordinator_to_cli::ControlRequestReply, _> =
                serde_json::from_str(raw.get());
            match reply {
                Ok(adora_message::coordinator_to_cli::ControlRequestReply::TopicSubscribed {
                    subscription_id,
                }) => {
                    topic_subscribers.insert(subscription_id, data_tx);
                    let _ = ack_tx.send(Ok(subscription_id));
                }
                Ok(adora_message::coordinator_to_cli::ControlRequestReply::Error(e)) => {
                    let _ = ack_tx.send(Err(eyre!("{e}")));
                }
                _ => {
                    let _ = ack_tx.send(Err(eyre!("unexpected topic subscribe reply")));
                }
            }
        } else {
            let _ = ack_tx.send(Err(eyre!("empty topic subscribe reply")));
        }
        return;
    }

    // Normal request-reply
    if let Some(reply_tx) = pending_requests.remove(&id) {
        let reply = if let Some(error) = error {
            // Map WS error to ControlRequestReply::Error for compatibility
            let err_reply = adora_message::coordinator_to_cli::ControlRequestReply::Error(error);
            Ok(serde_json::to_vec(&err_reply).unwrap_or_default())
        } else if let Some(raw) = result {
            // Preserve raw JSON bytes to maintain u128 fidelity for uhlc::ID
            Ok(raw.get().as_bytes().to_vec())
        } else {
            Err(eyre!("empty WS response"))
        };
        let _ = reply_tx.send(reply);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use serde_json::value::RawValue;

    fn raw(val: serde_json::Value) -> Box<RawValue> {
        serde_json::value::to_raw_value(&val).unwrap()
    }

    #[test]
    fn handle_response_routes_to_pending() {
        let id = Uuid::new_v4();
        let (tx, rx) = oneshot::channel();
        let mut pending = HashMap::new();
        pending.insert(id, tx);
        let mut subscribes = HashMap::new();
        let mut subs = Vec::new();
        let mut topic_pending = HashMap::new();
        let mut topic_subs = HashMap::new();

        handle_response(
            id,
            Some(raw(json!({"List": []}))),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
            &mut topic_pending,
            &mut topic_subs,
        );

        let mut rx = rx;
        let result = rx.try_recv().unwrap().unwrap();
        let val: serde_json::Value = serde_json::from_slice(&result).unwrap();
        assert_eq!(val, json!({"List": []}));
    }

    #[test]
    fn handle_response_orphan_response() {
        let id = Uuid::new_v4();
        let mut pending = HashMap::new();
        let mut subscribes = HashMap::new();
        let mut subs = Vec::new();
        let mut topic_pending = HashMap::new();
        let mut topic_subs = HashMap::new();

        // Response with unknown id should be dropped without panic
        handle_response(
            id,
            Some(raw(json!("ignored"))),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
            &mut topic_pending,
            &mut topic_subs,
        );
    }

    #[test]
    fn handle_response_routes_event_to_subscriber() {
        let id = Uuid::new_v4();
        let (ack_tx, mut ack_rx) = oneshot::channel();
        let (log_tx, log_rx) = std_mpsc::channel();
        let mut pending = HashMap::new();
        let mut subscribes = HashMap::new();
        subscribes.insert(id, (ack_tx, log_tx));
        let mut subs = Vec::new();
        let mut topic_pending = HashMap::new();
        let mut topic_subs = HashMap::new();

        // Successful subscribe ack
        handle_response(
            id,
            Some(raw(json!({"subscribed": true}))),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
            &mut topic_pending,
            &mut topic_subs,
        );

        // ack should succeed
        assert!(ack_rx.try_recv().unwrap().is_ok());
        // log_tx should have been moved to log_subscribers
        assert_eq!(subs.len(), 1);

        // Verify the subscriber receives data by simulating what session_loop does
        let payload = json!({"message": "test log"});
        let bytes = serde_json::to_vec(&payload).unwrap();
        subs[0].send(Ok(bytes.clone())).unwrap();
        let received = log_rx.recv().unwrap().unwrap();
        assert_eq!(received, bytes);
    }

    #[test]
    fn handle_response_event_no_subscriber() {
        // Simulate a subscribe error: ack gets error, no log_tx promoted
        let id = Uuid::new_v4();
        let (ack_tx, mut ack_rx) = oneshot::channel();
        let (log_tx, _log_rx) = std_mpsc::channel();
        let mut pending = HashMap::new();
        let mut subscribes = HashMap::new();
        subscribes.insert(id, (ack_tx, log_tx));
        let mut subs = Vec::new();
        let mut topic_pending = HashMap::new();
        let mut topic_subs = HashMap::new();

        handle_response(
            id,
            None,
            Some("not found".into()),
            &mut pending,
            &mut subscribes,
            &mut subs,
            &mut topic_pending,
            &mut topic_subs,
        );

        assert!(ack_rx.try_recv().unwrap().is_err());
        assert!(subs.is_empty());
    }

    #[test]
    fn handle_response_topic_subscribe_ack() {
        let id = Uuid::new_v4();
        let sub_id = Uuid::new_v4();
        let (ack_tx, mut ack_rx) = oneshot::channel();
        let (data_tx, _data_rx) = std_mpsc::channel();
        let mut pending = HashMap::new();
        let mut subscribes = HashMap::new();
        let mut subs = Vec::new();
        let mut topic_pending = HashMap::new();
        topic_pending.insert(id, (ack_tx, data_tx));
        let mut topic_subs = HashMap::new();

        handle_response(
            id,
            Some(raw(json!({"TopicSubscribed": {"subscription_id": sub_id}}))),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
            &mut topic_pending,
            &mut topic_subs,
        );

        let result_id = ack_rx.try_recv().unwrap().unwrap();
        assert_eq!(result_id, sub_id);
        assert!(topic_subs.contains_key(&sub_id));
    }

    #[tokio::test]
    async fn connect_rejects_from_async_context() {
        let addr: std::net::SocketAddr = "127.0.0.1:0".parse().unwrap();
        match WsSession::connect(addr) {
            Err(err) => assert!(
                format!("{err}").contains("async context"),
                "expected 'async context' in error, got: {err}"
            ),
            Ok(_) => panic!("expected error from async context"),
        }
    }

    #[tokio::test]
    async fn sender_drop_signals_receiver_error() {
        // Verify that dropping the oneshot sender (simulating session close)
        // causes the receiver to get a RecvError.
        let (tx, rx) = oneshot::channel::<eyre::Result<Vec<u8>>>();
        drop(tx);
        let result = rx.await;
        assert!(result.is_err()); // RecvError = sender dropped
    }
}
