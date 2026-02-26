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
}

impl WsSession {
    /// Connect to the coordinator via WebSocket.
    pub fn connect(addr: SocketAddr) -> eyre::Result<Self> {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .context("failed to create tokio runtime for WS session")?;

        let ws_url = match adora_message::auth::discover_token() {
            Some(token) => format!("ws://{addr}/api/control?token={}", token.as_hex()),
            None => format!("ws://{addr}/api/control"),
        };
        let ws_stream = rt
            .block_on(async { tokio_tungstenite::connect_async(&ws_url).await })
            .map_err(|e| eyre!("failed to connect to coordinator at {}: {e}", addr))?
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

async fn session_loop(ws_stream: WsStream, mut cmd_rx: mpsc::UnboundedReceiver<SessionCommand>) {
    let (mut ws_tx, mut ws_rx) = ws_stream.split();
    let mut pending_requests: PendingRequests = HashMap::new();
    let mut pending_subscribes: PendingSubscribes = HashMap::new();
    let mut log_subscribers: Vec<std_mpsc::Sender<eyre::Result<Vec<u8>>>> = Vec::new();

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
                }
            }
            msg = ws_rx.next() => {
                let Some(msg) = msg else { break };
                let text = match msg {
                    Ok(Message::Text(text)) => text,
                    Ok(Message::Close(_)) => break,
                    Ok(Message::Ping(data)) => {
                        let _ = ws_tx.send(Message::Pong(data)).await;
                        continue;
                    }
                    Ok(_) => continue,
                    Err(_) => break,
                };

                let frame: IncomingFrame = match serde_json::from_str(&text) {
                    Ok(m) => m,
                    Err(e) => {
                        tracing::warn!("failed to parse WS message: {e}");
                        continue;
                    }
                };

                if let Some(event_name) = &frame.event {
                    // Event frame (e.g. log push)
                    if event_name == "log" {
                        if let Some(payload) = &frame.payload {
                            let bytes = payload.get().as_bytes().to_vec();
                            log_subscribers.retain(|tx| tx.send(Ok(bytes.clone())).is_ok());
                        }
                    }
                } else if let Some(id) = frame.id {
                    // Response frame
                    handle_response(
                        id,
                        frame.result,
                        frame.error,
                        &mut pending_requests,
                        &mut pending_subscribes,
                        &mut log_subscribers,
                    );
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
}

fn handle_response(
    id: Uuid,
    result: Option<Box<serde_json::value::RawValue>>,
    error: Option<String>,
    pending_requests: &mut PendingRequests,
    pending_subscribes: &mut PendingSubscribes,
    log_subscribers: &mut Vec<std_mpsc::Sender<eyre::Result<Vec<u8>>>>,
) {
    // Check if this is a subscribe ack
    if let Some((ack_tx, log_tx)) = pending_subscribes.remove(&id) {
        if let Some(error) = error {
            let _ = ack_tx.send(Err(eyre!("{error}")));
        } else {
            log_subscribers.push(log_tx);
            let _ = ack_tx.send(Ok(()));
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

        handle_response(
            id,
            Some(raw(json!({"List": []}))),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
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

        // Response with unknown id should be dropped without panic
        handle_response(
            id,
            Some(raw(json!("ignored"))),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
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

        // Successful subscribe ack
        handle_response(
            id,
            Some(raw(json!({"subscribed": true}))),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
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

        handle_response(
            id,
            None,
            Some("not found".into()),
            &mut pending,
            &mut subscribes,
            &mut subs,
        );

        assert!(ack_rx.try_recv().unwrap().is_err());
        assert!(subs.is_empty());
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
