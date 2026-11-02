//! WebSocket client for CLI-to-coordinator communication.
//!
//! Replaces `TcpRequestReplyConnection` with a single WS connection that handles
//! both request-reply and log streaming.

use dora_message::ws_protocol::WsRequest;
use eyre::{Context, eyre};
use futures::{SinkExt, StreamExt};
use std::{
    collections::HashMap,
    net::SocketAddr,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
        mpsc as std_mpsc,
    },
    time::{Duration, Instant},
};
use tokio::sync::{mpsc, oneshot};
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

/// A coordinator answered at the address but turned this CLI away: it
/// rejected the `--auth` token, or it refused the protocol version handshake.
///
/// Carried as the root of the `eyre::Report` from [`WsSession::connect`] so
/// callers can tell "a coordinator is running here" from "nothing is
/// listening". Commands that spawn a coordinator, or build locally, when none
/// is reachable must not do so for this error. See
/// [`crate::common::coordinator_rejected`].
#[derive(Debug)]
pub(crate) struct CoordinatorRejected(String);

impl std::fmt::Display for CoordinatorRejected {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for CoordinatorRejected {}

/// Helper for deserializing incoming WS frames without going through
/// `serde_json::Value` for the result/payload fields: they are handed to the
/// caller as raw bytes to be parsed into their real type, so they are never
/// re-serialized from an intermediate `Value`.
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
        data_tx: TopicFrameSender,
        ack_tx: oneshot::Sender<eyre::Result<Uuid>>,
    },
}

impl WsSession {
    /// Connect to the coordinator via WebSocket.
    ///
    /// If called from within an existing tokio runtime, uses that runtime.
    /// Otherwise creates a dedicated runtime with one worker thread so the WS
    /// receive loop keeps running after synchronous API calls return.
    pub fn connect(addr: SocketAddr) -> eyre::Result<Self> {
        if tokio::runtime::Handle::try_current().is_ok() {
            eyre::bail!(
                "WsSession::connect must not be called from within an async context; \
                 use an async-native client instead"
            );
        }
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
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
                if let Some(token) = dora_message::auth::discover_token() {
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
                         hint: is the coordinator running? Start it with `dora up`"
                    )
                } else if msg.contains("401") || msg.contains("Unauthorized") {
                    eyre::Report::new(CoordinatorRejected(format!(
                        "authentication failed connecting to coordinator at {addr}: {msg}\n\n  \
                         The coordinator was started with --auth and requires a valid token.\n  \
                         The token is stored in ~/.config/dora/.dora-token\n\n  \
                         Possible fixes:\n  \
                         - Run `dora down && dora up` to regenerate the token\n  \
                         - Ensure you're using the same user that started the coordinator\n  \
                         - Set DORA_AUTH_TOKEN env var to match the coordinator's token\n  \
                         - Restart without --auth to disable authentication"
                    )))
                } else {
                    eyre!("failed to connect to coordinator at {addr}: {msg}")
                }
            })?
            .0;

        let (cmd_tx, cmd_rx) = mpsc::unbounded_channel();
        rt.spawn(session_loop(ws_stream, cmd_rx));

        let session = Self { rt, cmd_tx };

        // Protocol version handshake — sent before any other request.
        // Fails fast on version mismatch so the CLI never silently
        // exchanges incompatible messages with the coordinator
        // (dora-rs/adora#151).
        session.handshake_hello()?;

        Ok(session)
    }

    /// Send a `ControlRequest::Hello` stamped with the CLI's dora
    /// crate version and verify the coordinator accepts it. Fails with
    /// a clear error on version mismatch.
    fn handshake_hello(&self) -> eyre::Result<()> {
        use dora_message::{
            cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
        };
        let req = serde_json::to_vec(&ControlRequest::hello())
            .map_err(|e| eyre!("failed to serialize Hello: {e}"))?;
        let raw_reply = self.request(&req).wrap_err(
            "protocol version handshake with coordinator failed \
             (could not send or receive Hello)",
        )?;
        let reply: ControlRequestReply = serde_json::from_slice(&raw_reply)
            .map_err(|e| eyre!("failed to parse Hello reply: {e}"))?;
        match reply {
            ControlRequestReply::HelloOk { dora_version } => {
                tracing::debug!(
                    coordinator_version = %dora_version,
                    "protocol version handshake OK"
                );
                Ok(())
            }
            ControlRequestReply::Error(msg) => {
                Err(eyre::Report::new(CoordinatorRejected(format!(
                    "coordinator rejected CLI: {msg}\n\n  \
                     hint: the CLI and coordinator binaries must share a \
                     semver-compatible dora version. Upgrade the component \
                     that is behind."
                ))))
            }
            other => Err(eyre::Report::new(CoordinatorRejected(format!(
                "unexpected reply to Hello: {other:?} — \
                 coordinator may be too old to understand the handshake"
            )))),
        }
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

    /// Subscribe to topic data via the coordinator's topic inspection stream.
    ///
    /// Sends a `TopicSubscribe` request, waits for the ack, then returns
    /// a `(subscription_id, receiver)` pair. Binary WS frames with matching
    /// subscription UUID prefix are dispatched to the receiver.
    pub fn subscribe_topics(
        &self,
        dataflow_id: Uuid,
        topics: Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    ) -> eyre::Result<(Uuid, TopicFrameReceiver)> {
        let request = serde_json::to_vec(
            &dora_message::cli_to_coordinator::ControlRequest::topic_subscribe(dataflow_id, topics),
        )
        .map_err(|e| eyre!("failed to serialize TopicSubscribe: {e}"))?;

        let (data_tx, data_rx) = topic_frame_channel(TOPIC_FRAME_QUEUE_BYTES);
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

/// Payload bytes the frames queued for one topic subscription may hold in this
/// CLI before newer ones are dropped: two of the largest payloads, as the
/// coordinator lets be in flight to one connection.
///
/// The session task reads frames off the socket as fast as they arrive, so
/// the coordinator's byte budget only ever throttles a slow network. Without a
/// bound here, a command slower than its topic — `dora topic echo` rendering a
/// camera frame as text, say — would grow this queue, and the CLI's memory,
/// without limit (dora-rs/dora#3636 review).
pub(crate) const TOPIC_FRAME_QUEUE_BYTES: usize =
    2 * dora_message::daemon_to_coordinator::MAX_TOPIC_DEBUG_PAYLOAD_BYTES;

/// Shortest gap between warnings about topic frames this CLI dropped.
const TOPIC_FRAME_DROP_LOG_INTERVAL: Duration = Duration::from_secs(5);

/// A topic subscription's frames, in arrival order. Taking a frame gives its
/// bytes back to the subscription's [`TOPIC_FRAME_QUEUE_BYTES`].
pub struct TopicFrameReceiver {
    rx: std_mpsc::Receiver<eyre::Result<Vec<u8>>>,
    queued: Arc<AtomicUsize>,
}

impl TopicFrameReceiver {
    /// Like [`std_mpsc::Receiver::recv`].
    pub fn recv(&self) -> Result<eyre::Result<Vec<u8>>, std_mpsc::RecvError> {
        self.rx.recv().map(|frame| self.taken(frame))
    }

    /// Like [`std_mpsc::Receiver::recv_timeout`].
    pub fn recv_timeout(
        &self,
        timeout: Duration,
    ) -> Result<eyre::Result<Vec<u8>>, std_mpsc::RecvTimeoutError> {
        self.rx.recv_timeout(timeout).map(|frame| self.taken(frame))
    }

    fn taken(&self, frame: eyre::Result<Vec<u8>>) -> eyre::Result<Vec<u8>> {
        if let Ok(payload) = &frame {
            self.queued.fetch_sub(payload.len(), Ordering::Relaxed);
        }
        frame
    }
}

/// The session task's end of a topic subscription. It queues frames up to the
/// subscription's byte budget and drops the rest, never waiting: waiting would
/// stop the task reading the socket, and with it the replies to this CLI's
/// requests. Topic frames are droppable; the coordinator drops them too when
/// this connection falls behind.
pub(crate) struct TopicFrameSender {
    tx: std_mpsc::Sender<eyre::Result<Vec<u8>>>,
    queued: Arc<AtomicUsize>,
    capacity: usize,
    dropped: u64,
    last_warning: Option<Instant>,
}

impl TopicFrameSender {
    /// Queue `payload`, or drop it if it does not fit the byte budget. An
    /// empty queue takes any frame, so a frame of the largest size still gets
    /// through. Returns false once the receiver is gone.
    fn send(&mut self, payload: Vec<u8>) -> bool {
        // Only this task adds and the receiver only subtracts, so the count
        // can only have shrunk since it was read.
        let queued = self.queued.load(Ordering::Relaxed);
        if queued > 0 && queued + payload.len() > self.capacity {
            self.dropped += 1;
            let now = Instant::now();
            if self
                .last_warning
                .is_none_or(|last| now - last >= TOPIC_FRAME_DROP_LOG_INTERVAL)
            {
                tracing::warn!(
                    "dropped {} topic frame(s): they arrive faster than this command \
                     processes them",
                    self.dropped
                );
                self.dropped = 0;
                self.last_warning = Some(now);
            }
            return true;
        }
        self.queued.fetch_add(payload.len(), Ordering::Relaxed);
        self.tx.send(Ok(payload)).is_ok()
    }
}

/// A topic subscription's queue, holding up to `capacity` payload bytes.
pub(crate) fn topic_frame_channel(capacity: usize) -> (TopicFrameSender, TopicFrameReceiver) {
    let (tx, rx) = std_mpsc::channel();
    let queued = Arc::new(AtomicUsize::new(0));
    (
        TopicFrameSender {
            tx,
            queued: queued.clone(),
            capacity,
            dropped: 0,
            last_warning: None,
        },
        TopicFrameReceiver { rx, queued },
    )
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
type PendingTopicSubscribes =
    HashMap<Uuid, (oneshot::Sender<eyre::Result<Uuid>>, TopicFrameSender)>;
type TopicSubscribers = HashMap<Uuid, TopicFrameSender>;

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
                            if event_name == "log"
                                && let Some(payload) = &frame.payload {
                                    let bytes = payload.get().as_bytes().to_vec();
                                    log_subscribers.retain(|tx| tx.send(Ok(bytes.clone())).is_ok());
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
                        if let Some(tx) = topic_subscribers.get_mut(&sub_id)
                            && !tx.send(payload) {
                                topic_subscribers.remove(&sub_id);
                            }
                    }
                    Ok(Message::Close(_)) => break,
                    Ok(Message::Ping(data)) => {
                        let _ = ws_tx.send(Message::Pong(data)).await;
                    }
                    Ok(other) => {
                        tracing::trace!("ignoring unexpected WS message type: {other:?}");
                    }
                    Err(e) => {
                        if is_connection_drop(&e) {
                            tracing::debug!("coordinator WebSocket session ended: {e}");
                        } else {
                            tracing::warn!("coordinator WebSocket session ended: {e}");
                        }
                        break;
                    }
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

/// Whether a WebSocket read error is the connection simply going away.
///
/// The coordinator exits after `dora down` without a closing handshake, so a
/// reset is the normal end of that session and must not print a warning.
/// Anything else (a capacity limit, a protocol violation) is worth a warning.
fn is_connection_drop(e: &tokio_tungstenite::tungstenite::Error) -> bool {
    use tokio_tungstenite::tungstenite::error::{Error, ProtocolError};
    match e {
        Error::ConnectionClosed
        | Error::AlreadyClosed
        | Error::Protocol(ProtocolError::ResetWithoutClosingHandshake) => true,
        Error::Io(io) => matches!(
            io.kind(),
            std::io::ErrorKind::ConnectionReset
                | std::io::ErrorKind::ConnectionAborted
                | std::io::ErrorKind::BrokenPipe
                | std::io::ErrorKind::UnexpectedEof
        ),
        _ => false,
    }
}

/// Error for a `TopicSubscribed` ack whose binary-frame encoding does not match
/// ours. Wraps the shared message so both sides of the handshake explain a
/// mismatch identically.
fn topic_protocol_mismatch_error(coordinator_version: Option<u16>) -> eyre::Report {
    eyre!(dora_message::topic_protocol_mismatch_message(
        "coordinator",
        coordinator_version
    ))
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
            let reply: Result<dora_message::coordinator_to_cli::ControlRequestReply, _> =
                serde_json::from_str(raw.get());
            match reply {
                Ok(dora_message::coordinator_to_cli::ControlRequestReply::TopicSubscribed {
                    subscription_id,
                    protocol_version,
                }) => {
                    // A coordinator on the other encoding would send frames we
                    // would misparse rather than fail on, so refuse the
                    // subscription instead of consuming them (dora-rs/dora#3153).
                    //
                    // We deliberately don't send `TopicUnsubscribe` here. Only
                    // a *pre-handshake* coordinator reaches this branch having
                    // actually created a subscription (a current one rejects
                    // before creating anything), and every `subscribe_topics`
                    // caller propagates this error with `?`, dropping the
                    // `WsSession` — which closes the connection and makes the
                    // coordinator tear its subscriptions down. Threading an
                    // outgoing sender into `handle_response` to save that
                    // already-closing subscription would not earn its keep.
                    if protocol_version != Some(dora_message::TOPIC_DATA_PROTOCOL_VERSION) {
                        let _ = ack_tx.send(Err(topic_protocol_mismatch_error(protocol_version)));
                    } else {
                        topic_subscribers.insert(subscription_id, data_tx);
                        let _ = ack_tx.send(Ok(subscription_id));
                    }
                }
                Ok(dora_message::coordinator_to_cli::ControlRequestReply::Error(e)) => {
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
            let err_reply = dora_message::coordinator_to_cli::ControlRequestReply::Error(error);
            Ok(serde_json::to_vec(&err_reply).unwrap_or_default())
        } else if let Some(raw) = result {
            // Hand the raw JSON bytes to the caller, which parses them into the
            // concrete reply type.
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

    /// The receiver takes back each frame's bytes, and the budget admits a
    /// frame into an empty queue whatever its size.
    #[test]
    fn topic_frame_queue_is_bounded_by_bytes() {
        let (mut tx, rx) = topic_frame_channel(10);
        assert!(tx.send(vec![0; 6]));
        assert!(tx.send(vec![0; 4]), "exactly at the budget still fits");
        assert!(
            tx.send(vec![0; 1]),
            "a dropped frame keeps the subscription"
        );
        assert_eq!(rx.recv().unwrap().unwrap().len(), 6);
        assert_eq!(rx.recv().unwrap().unwrap().len(), 4);
        assert!(
            rx.recv_timeout(Duration::from_millis(10)).is_err(),
            "the third was dropped"
        );
        assert_eq!(rx.queued.load(Ordering::Relaxed), 0);

        assert!(
            tx.send(vec![0; 25]),
            "an empty queue takes a frame past the budget"
        );
        assert_eq!(rx.recv().unwrap().unwrap().len(), 25);
        drop(rx);
        assert!(
            !tx.send(vec![0; 1]),
            "a closed receiver ends the subscription"
        );
    }

    /// Serve one control connection that acks `TopicSubscribe` and then sends
    /// `count` binary frames of `len` payload bytes. Signals on the returned
    /// channel once the client has read them all: it answers a ping sent
    /// after the frames only once it has read them.
    fn serve_topic_frames(
        subscription_id: Uuid,
        len: usize,
        count: usize,
    ) -> (u16, std_mpsc::Receiver<()>) {
        use dora_message::{
            cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
        };
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let (done_tx, done_rx) = std_mpsc::channel();
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap();
            rt.block_on(async move {
                listener.set_nonblocking(true).unwrap();
                let listener = tokio::net::TcpListener::from_std(listener).unwrap();
                let (stream, _) = listener.accept().await.unwrap();
                let mut ws = tokio_tungstenite::accept_async(stream).await.unwrap();
                while let Some(Ok(msg)) = ws.next().await {
                    let text = match msg {
                        Message::Text(text) => text,
                        Message::Pong(_) => {
                            let _ = done_tx.send(());
                            continue;
                        }
                        _ => continue,
                    };
                    let request: WsRequest = serde_json::from_str(&text).unwrap();
                    let reply = match serde_json::from_value(request.params).unwrap() {
                        ControlRequest::Hello { .. } => ControlRequestReply::HelloOk {
                            dora_version: dora_message::current_crate_version(),
                        },
                        ControlRequest::TopicSubscribe { .. } => {
                            ControlRequestReply::TopicSubscribed {
                                subscription_id,
                                protocol_version: Some(dora_message::TOPIC_DATA_PROTOCOL_VERSION),
                            }
                        }
                        other => panic!("unexpected request {other:?}"),
                    };
                    let subscribed = matches!(reply, ControlRequestReply::TopicSubscribed { .. });
                    let response = serde_json::json!({ "id": request.id, "result": reply });
                    ws.send(Message::Text(response.to_string().into()))
                        .await
                        .unwrap();
                    if subscribed {
                        for _ in 0..count {
                            let mut frame = subscription_id.as_bytes().to_vec();
                            frame.resize(16 + len, 0);
                            ws.send(Message::Binary(frame.into())).await.unwrap();
                        }
                        ws.send(Message::Ping(Vec::new().into())).await.unwrap();
                    }
                }
            });
        });
        (port, done_rx)
    }

    /// A command that is not taking frames must not make the session task
    /// queue them without bound: past the subscription's byte budget, frames
    /// are dropped, and taking the queued ones frees the budget again
    /// (dora-rs/dora#3636 review).
    #[test]
    fn frames_past_the_queue_budget_are_dropped_while_the_command_is_behind() {
        let subscription_id = Uuid::new_v4();
        let (port, done) = serve_topic_frames(subscription_id, TOPIC_FRAME_QUEUE_BYTES / 2, 4);
        let session = WsSession::connect(SocketAddr::from(([127, 0, 0, 1], port))).unwrap();
        let (_, frames) = session
            .subscribe_topics(
                Uuid::new_v4(),
                vec![("node".to_string().into(), "out".to_string().into())],
            )
            .unwrap();

        // Take nothing until the session task has read all four frames.
        done.recv_timeout(Duration::from_secs(20))
            .expect("the client must read every frame off the socket");

        let mut delivered = 0;
        while let Ok(frame) = frames.recv_timeout(Duration::from_millis(200)) {
            assert_eq!(frame.unwrap().len(), TOPIC_FRAME_QUEUE_BYTES / 2);
            delivered += 1;
        }
        assert_eq!(
            delivered, 2,
            "two frames fill the budget; the other two are dropped"
        );
        assert_eq!(frames.queued.load(Ordering::Relaxed), 0);
    }
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
        let (data_tx, _data_rx) = topic_frame_channel(TOPIC_FRAME_QUEUE_BYTES);
        let mut pending = HashMap::new();
        let mut subscribes = HashMap::new();
        let mut subs = Vec::new();
        let mut topic_pending = HashMap::new();
        topic_pending.insert(id, (ack_tx, data_tx));
        let mut topic_subs = HashMap::new();

        handle_response(
            id,
            Some(raw(json!({"TopicSubscribed": {
                "subscription_id": sub_id,
                "protocol_version": dora_message::TOPIC_DATA_PROTOCOL_VERSION,
            }}))),
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

    /// Binary topic frames are positionally encoded, so a coordinator on a
    /// different encoding produces plausible garbage rather than a decode
    /// error. The ack must be refused, and the subscription must not be
    /// registered — otherwise frames would be dispatched to a consumer that
    /// cannot read them (dora-rs/dora#3153).
    fn ack_with_protocol_version(version: Option<u16>) -> (eyre::Result<Uuid>, bool) {
        let id = Uuid::new_v4();
        let sub_id = Uuid::new_v4();
        let (ack_tx, mut ack_rx) = oneshot::channel();
        let (data_tx, _data_rx) = topic_frame_channel(TOPIC_FRAME_QUEUE_BYTES);
        let mut pending = HashMap::new();
        let mut subscribes = HashMap::new();
        let mut subs = Vec::new();
        let mut topic_pending = HashMap::new();
        topic_pending.insert(id, (ack_tx, data_tx));
        let mut topic_subs = HashMap::new();

        let result = match version {
            Some(version) => json!({"TopicSubscribed": {
                "subscription_id": sub_id, "protocol_version": version,
            }}),
            // A pre-handshake coordinator omits the field entirely.
            None => json!({"TopicSubscribed": {"subscription_id": sub_id}}),
        };

        handle_response(
            id,
            Some(raw(result)),
            None,
            &mut pending,
            &mut subscribes,
            &mut subs,
            &mut topic_pending,
            &mut topic_subs,
        );

        (ack_rx.try_recv().unwrap(), topic_subs.contains_key(&sub_id))
    }

    #[test]
    fn topic_subscribe_ack_rejects_older_protocol_version() {
        let (ack, registered) = ack_with_protocol_version(Some(1));
        let err = ack.expect_err("an older protocol version must be rejected");
        let msg = format!("{err}");
        assert!(
            msg.contains("version 1") && msg.contains("misparse"),
            "error should name the peer's version and why it is fatal, got: {msg}"
        );
        assert!(
            !registered,
            "a rejected subscription must not be registered for frame dispatch"
        );
    }

    #[test]
    fn topic_subscribe_ack_rejects_missing_protocol_version() {
        let (ack, registered) = ack_with_protocol_version(None);
        let err = ack.expect_err("a pre-handshake coordinator must be rejected");
        assert!(
            format!("{err}").contains("predates"),
            "error should say the coordinator predates the handshake, got: {err}"
        );
        assert!(
            !registered,
            "a rejected subscription must not be registered for frame dispatch"
        );
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

    #[test]
    fn connection_drop_is_not_a_warning() {
        use tokio_tungstenite::tungstenite::error::{CapacityError, Error, ProtocolError};
        // `dora down`: the coordinator exits without a closing handshake.
        assert!(is_connection_drop(&Error::Protocol(
            ProtocolError::ResetWithoutClosingHandshake
        )));
        assert!(is_connection_drop(&Error::ConnectionClosed));
        assert!(is_connection_drop(&Error::Io(std::io::Error::from(
            std::io::ErrorKind::ConnectionReset
        ))));
        // A real failure still warns.
        assert!(!is_connection_drop(&Error::Capacity(
            CapacityError::MessageTooLong {
                size: 17 << 20,
                max_size: 16 << 20,
            }
        )));
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
