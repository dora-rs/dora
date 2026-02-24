//! WebSocket client for CLI-to-coordinator communication.
//!
//! Replaces `TcpRequestReplyConnection` with a single WS connection that handles
//! both request-reply and log streaming.

use adora_message::ws_protocol::{WsMessage, WsRequest, WsResponse};
use eyre::{Context, eyre};
use futures::{SinkExt, StreamExt};
use std::{
    collections::HashMap,
    net::SocketAddr,
    sync::mpsc as std_mpsc,
};
use tokio::sync::{mpsc, oneshot};
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

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
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .context("failed to create tokio runtime for WS session")?;

        let ws_url = format!("ws://{addr}/api/control");
        let ws_stream = rt
            .block_on(async { tokio_tungstenite::connect_async(&ws_url).await })
            .map_err(|e| eyre!("failed to connect to coordinator at {ws_url}: {e}"))?
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

type WsStream = tokio_tungstenite::WebSocketStream<
    tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
>;

async fn session_loop(
    ws_stream: WsStream,
    mut cmd_rx: mpsc::UnboundedReceiver<SessionCommand>,
) {
    let (mut ws_tx, mut ws_rx) = ws_stream.split();
    let mut pending_requests: HashMap<Uuid, oneshot::Sender<eyre::Result<Vec<u8>>>> =
        HashMap::new();
    let mut pending_subscribes: HashMap<
        Uuid,
        (
            oneshot::Sender<eyre::Result<()>>,
            std_mpsc::Sender<eyre::Result<Vec<u8>>>,
        ),
    > = HashMap::new();
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

                let parsed: WsMessage = match serde_json::from_str(&text) {
                    Ok(m) => m,
                    Err(_) => continue,
                };

                match parsed {
                    WsMessage::Response(resp) => {
                        handle_response(
                            resp,
                            &mut pending_requests,
                            &mut pending_subscribes,
                            &mut log_subscribers,
                        );
                    }
                    WsMessage::Event(event) if event.event == "log" => {
                        let bytes = match serde_json::to_vec(&event.payload) {
                            Ok(b) => b,
                            Err(_) => continue,
                        };
                        log_subscribers.retain(|tx| tx.send(Ok(bytes.clone())).is_ok());
                    }
                    _ => {}
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
    resp: WsResponse,
    pending_requests: &mut HashMap<Uuid, oneshot::Sender<eyre::Result<Vec<u8>>>>,
    pending_subscribes: &mut HashMap<
        Uuid,
        (
            oneshot::Sender<eyre::Result<()>>,
            std_mpsc::Sender<eyre::Result<Vec<u8>>>,
        ),
    >,
    log_subscribers: &mut Vec<std_mpsc::Sender<eyre::Result<Vec<u8>>>>,
) {
    // Check if this is a subscribe ack
    if let Some((ack_tx, log_tx)) = pending_subscribes.remove(&resp.id) {
        if let Some(error) = resp.error {
            let _ = ack_tx.send(Err(eyre!("{error}")));
        } else {
            log_subscribers.push(log_tx);
            let _ = ack_tx.send(Ok(()));
        }
        return;
    }

    // Normal request-reply
    if let Some(reply_tx) = pending_requests.remove(&resp.id) {
        let result = if let Some(error) = resp.error {
            // Map WS error to ControlRequestReply::Error for compatibility
            let err_reply =
                adora_message::coordinator_to_cli::ControlRequestReply::Error(error);
            Ok(serde_json::to_vec(&err_reply).unwrap_or_default())
        } else if let Some(result) = resp.result {
            serde_json::to_vec(&result).map_err(|e| eyre!("failed to serialize response: {e}"))
        } else {
            Err(eyre!("empty WS response"))
        };
        let _ = reply_tx.send(result);
    }
}
