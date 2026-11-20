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
        atomic::{AtomicU64, AtomicUsize, Ordering},
        mpsc as std_mpsc,
    },
};
use tokio::sync::{mpsc, oneshot};
use tokio_tungstenite::tungstenite::{self, Message};
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

/// Where the CLI looks for the `--auth` token, in the order
/// [`dora_message::auth::discover_token`] tries them. The config directory
/// is the platform's (`~/.config` only on Linux; `~/Library/Application
/// Support` on macOS, `%APPDATA%` on Windows), so it is printed resolved.
fn token_locations() -> String {
    let config = dora_message::auth::config_token_path()
        .map(|path| format!(", then {}", path.display()))
        .unwrap_or_default();
    format!(
        "The CLI reads it from DORA_AUTH_TOKEN, then .dora-token in the current \
         directory{config}. `dora up --auth` writes both files."
    )
}

/// Explains why connecting to the coordinator at `addr` failed.
///
/// A refused connection is recognized by its `io::ErrorKind`, not by the OS
/// message: that is localized, so no English text matches it everywhere.
fn connect_failure(addr: SocketAddr, error: tungstenite::Error) -> eyre::Report {
    let refused = matches!(
        &error,
        tungstenite::Error::Io(io) if io.kind() == std::io::ErrorKind::ConnectionRefused
    );
    let msg = error.to_string();
    if refused {
        eyre!(
            "cannot connect to coordinator at {addr}: {msg}\n\n  \
             hint: is the coordinator running? Start it with `dora up`"
        )
    } else if msg.contains("401") || msg.contains("Unauthorized") {
        let locations = token_locations();
        eyre::Report::new(CoordinatorRejected(format!(
            "authentication failed connecting to coordinator at {addr}: {msg}\n\n  \
             The coordinator was started with --auth and requires a valid token.\n  \
             {locations}\n\n  \
             Possible fixes:\n  \
             - Run `dora down && dora up` to regenerate the token\n  \
             - Ensure you're using the same user that started the coordinator\n  \
             - Set DORA_AUTH_TOKEN env var to match the coordinator's token\n  \
             - Restart without --auth to disable authentication"
        )))
    } else {
        eyre!("failed to connect to coordinator at {addr}: {msg}")
    }
}

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
        subscriber: TopicSubscriber,
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
            .map_err(|e| connect_failure(addr, e))?
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
    ///
    /// The receiver's backlog is bounded by `limits`; frames that arrive while
    /// it is full are dropped and counted (see `dispatch_topic_frame` and
    /// [`TopicReceiver::dropped`]).
    pub fn subscribe_topics(
        &self,
        dataflow_id: Uuid,
        topics: Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
        limits: TopicBufferLimits,
    ) -> eyre::Result<(Uuid, TopicReceiver)> {
        let request = serde_json::to_vec(
            &dora_message::cli_to_coordinator::ControlRequest::topic_subscribe(dataflow_id, topics),
        )
        .map_err(|e| eyre!("failed to serialize TopicSubscribe: {e}"))?;

        let (subscriber, data_rx) = topic_channel(limits);
        let (ack_tx, ack_rx) = oneshot::channel();
        self.cmd_tx
            .send(SessionCommand::SubscribeTopics {
                request,
                subscriber,
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
type PendingTopicSubscribes = HashMap<Uuid, (oneshot::Sender<eyre::Result<Uuid>>, TopicSubscriber)>;
type TopicSubscribers = HashMap<Uuid, TopicSubscriber>;
/// How much a topic subscription's backlog holds before newer frames are
/// dropped: at most `frames` frames and `bytes` payload bytes. A frame for an
/// empty backlog is always taken, whatever its size (the coordinator caps
/// debug payloads at 64 MiB).
#[derive(Debug, Clone, Copy)]
pub struct TopicBufferLimits {
    pub frames: usize,
    pub bytes: usize,
}

impl TopicBufferLimits {
    /// For live views (`topic echo`, `hz`, `info`): enough to ride out a short
    /// consumer stall, small enough that a slow terminal can't hold more than
    /// 64 MiB of camera frames.
    pub const LIVE: Self = Self {
        frames: 1024,
        bytes: 64 * 1024 * 1024,
    };

    /// For `dora record --proxy`, where every lost frame is a gap in the
    /// recording: a backlog that absorbs a disk stall of seconds even for
    /// large frames (~170 1080p RGB frames), while still bounding memory.
    pub const RECORD: Self = Self {
        frames: 64 * 1024,
        bytes: 1024 * 1024 * 1024,
    };
}

/// State shared by the two ends of a topic subscription.
#[derive(Default)]
struct TopicBacklog {
    /// Payload bytes sent but not yet received.
    bytes: AtomicUsize,
    /// Frames dropped because the backlog was full.
    dropped: AtomicU64,
}

/// The session's end of a topic subscription.
pub(crate) struct TopicSubscriber {
    tx: std_mpsc::SyncSender<eyre::Result<Vec<u8>>>,
    max_bytes: usize,
    backlog: Arc<TopicBacklog>,
}

/// A handle to a [`TopicReceiver`]'s drop count that outlives the receiver,
/// e.g. after it was moved into a worker thread.
#[derive(Clone)]
pub struct DroppedFrames(Arc<TopicBacklog>);

impl DroppedFrames {
    /// How many frames were dropped so far because the receiver fell behind.
    pub fn get(&self) -> u64 {
        self.0.dropped.load(Ordering::Relaxed)
    }

    /// Tell the user, on stderr, how many frames were dropped, if any.
    pub fn report(&self) {
        let dropped = self.get();
        if dropped > 0 {
            eprintln!(
                "note: {dropped} messages were dropped because they arrived faster \
                 than they could be processed, and are not included above."
            );
        }
    }
}

/// Receives the payloads of one topic subscription's frames
/// (see [`WsSession::subscribe_topics`]).
pub struct TopicReceiver {
    rx: std_mpsc::Receiver<eyre::Result<Vec<u8>>>,
    backlog: Arc<TopicBacklog>,
}

impl TopicReceiver {
    /// Like [`std_mpsc::Receiver::recv`].
    pub fn recv(&self) -> Result<eyre::Result<Vec<u8>>, std_mpsc::RecvError> {
        self.rx.recv().map(|frame| self.taken(frame))
    }

    /// Like [`std_mpsc::Receiver::recv_timeout`].
    pub fn recv_timeout(
        &self,
        timeout: std::time::Duration,
    ) -> Result<eyre::Result<Vec<u8>>, std_mpsc::RecvTimeoutError> {
        self.rx.recv_timeout(timeout).map(|frame| self.taken(frame))
    }

    /// How many frames were dropped so far because this receiver fell behind.
    pub fn dropped(&self) -> u64 {
        self.dropped_frames().get()
    }

    /// A handle to the drop count that can be kept after `self` is moved.
    pub fn dropped_frames(&self) -> DroppedFrames {
        DroppedFrames(self.backlog.clone())
    }

    fn taken(&self, frame: eyre::Result<Vec<u8>>) -> eyre::Result<Vec<u8>> {
        if let Ok(payload) = &frame {
            self.backlog
                .bytes
                .fetch_sub(payload.len(), Ordering::Relaxed);
        }
        frame
    }
}

pub(crate) fn topic_channel(limits: TopicBufferLimits) -> (TopicSubscriber, TopicReceiver) {
    let (tx, rx) = std_mpsc::sync_channel(limits.frames);
    let backlog = Arc::new(TopicBacklog::default());
    let subscriber = TopicSubscriber {
        tx,
        max_bytes: limits.bytes,
        backlog: backlog.clone(),
    };
    (subscriber, TopicReceiver { rx, backlog })
}

/// Hand one binary WS frame (`subscription id ++ payload`) to its
/// subscriber, without waiting.
///
/// The session loop also reads request replies, so it must never block on a
/// slow consumer — and it must not buffer for one without limit either: it
/// keeps the socket drained, so the coordinator's own drop-when-full policy
/// never sees the slow consumer, and an unbounded backlog here grows until
/// the CLI runs out of memory (e.g. `dora topic echo` on a large camera
/// topic). A frame that would overfill the backlog — by count or by bytes —
/// is dropped instead, before its payload is copied.
fn dispatch_topic_frame(topic_subscribers: &mut TopicSubscribers, data: &[u8]) {
    let Some((sub_id_bytes, payload)) = data.split_first_chunk::<16>() else {
        tracing::warn!("binary WS frame too short ({} bytes)", data.len());
        return;
    };
    let sub_id = Uuid::from_bytes(*sub_id_bytes);
    let Some(subscriber) = topic_subscribers.get(&sub_id) else {
        return;
    };
    let backlog = &subscriber.backlog;
    let queued = backlog.bytes.load(Ordering::Relaxed);
    let fits = queued == 0 || queued.saturating_add(payload.len()) <= subscriber.max_bytes;
    if fits {
        // Counted before the send, so the receiver never subtracts first.
        backlog.bytes.fetch_add(payload.len(), Ordering::Relaxed);
        match subscriber.tx.try_send(Ok(payload.to_vec())) {
            Ok(()) => return,
            Err(std_mpsc::TrySendError::Full(_)) => {
                backlog.bytes.fetch_sub(payload.len(), Ordering::Relaxed);
            }
            Err(std_mpsc::TrySendError::Disconnected(_)) => {
                topic_subscribers.remove(&sub_id);
                return;
            }
        }
    }
    // Only `debug`: the CLI's console logger writes to stdout, which is the
    // data stream of `topic echo`. Consumers report the count on stderr.
    if backlog.dropped.fetch_add(1, Ordering::Relaxed) == 0 {
        tracing::debug!(
            "topic data arrives faster than it is processed; \
             dropping frames for subscription {sub_id}"
        );
    }
}

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
                    SessionCommand::SubscribeTopics { request, subscriber, ack_tx } => {
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
                        pending_topic_subscribes.insert(id, (ack_tx, subscriber));
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
                        dispatch_topic_frame(&mut topic_subscribers, &data);
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
    if let Some((ack_tx, subscriber)) = pending_topic_subscribes.remove(&id) {
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
                        topic_subscribers.insert(subscription_id, subscriber);
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
    use serde_json::json;
    use serde_json::value::RawValue;

    fn raw(val: serde_json::Value) -> Box<RawValue> {
        serde_json::value::to_raw_value(&val).unwrap()
    }

    #[test]
    fn refused_connection_points_at_dora_up_whatever_the_os_language() {
        // How a Chinese Windows reports WSAECONNREFUSED (os error 10061).
        let refused = tungstenite::Error::Io(std::io::Error::new(
            std::io::ErrorKind::ConnectionRefused,
            "由于目标计算机积极拒绝，无法连接。 (os error 10061)",
        ));
        let addr: SocketAddr = "127.0.0.1:6013".parse().unwrap();
        let text = connect_failure(addr, refused).to_string();
        assert!(
            text.contains("cannot connect to coordinator at 127.0.0.1:6013"),
            "{text}"
        );
        assert!(text.contains("dora up"), "{text}");
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
        let (data_tx, _data_rx) = topic_channel(TopicBufferLimits::LIVE);
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
        let (data_tx, _data_rx) = topic_channel(TopicBufferLimits::LIVE);
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

    fn topic_frame(sub_id: Uuid, payload: &[u8]) -> Vec<u8> {
        let mut frame = sub_id.as_bytes().to_vec();
        frame.extend_from_slice(payload);
        frame
    }

    fn one_subscriber() -> (Uuid, TopicSubscribers, TopicReceiver) {
        let sub_id = Uuid::new_v4();
        let (subscriber, rx) = topic_channel(TopicBufferLimits::LIVE);
        (sub_id, HashMap::from([(sub_id, subscriber)]), rx)
    }

    fn drain(rx: &TopicReceiver) -> Vec<Vec<u8>> {
        std::iter::from_fn(|| rx.recv_timeout(std::time::Duration::ZERO).ok())
            .map(|frame| frame.unwrap())
            .collect()
    }

    /// A consumer slower than the stream must not make the session buffer
    /// every frame: the session keeps the socket drained, so the
    /// coordinator never drops for it, and an unbounded backlog here grows
    /// until the CLI runs out of memory.
    #[test]
    fn topic_frames_beyond_the_frame_cap_are_dropped() {
        let (sub_id, mut subscribers, rx) = one_subscriber();

        let total = TopicBufferLimits::LIVE.frames as u64 * 3;
        for i in 0..total {
            dispatch_topic_frame(&mut subscribers, &topic_frame(sub_id, &i.to_le_bytes()));
        }

        // The oldest frames are kept, in order; the rest are counted as dropped.
        let received: Vec<u64> = drain(&rx)
            .into_iter()
            .map(|frame| u64::from_le_bytes(frame.try_into().unwrap()))
            .collect();
        assert_eq!(
            received,
            (0..TopicBufferLimits::LIVE.frames as u64).collect::<Vec<_>>()
        );
        assert_eq!(rx.dropped(), total - TopicBufferLimits::LIVE.frames as u64);

        // Once the consumer catches up, frames flow again.
        dispatch_topic_frame(&mut subscribers, &topic_frame(sub_id, b"next"));
        assert_eq!(drain(&rx), vec![b"next".to_vec()]);
    }

    /// Large frames are bounded by bytes, long before the frame cap: 64
    /// camera images would otherwise hold gigabytes.
    #[test]
    fn topic_frames_beyond_the_byte_cap_are_dropped() {
        let (sub_id, mut subscribers, rx) = one_subscriber();
        let big = vec![0u8; TopicBufferLimits::LIVE.bytes / 4 + 1];

        for _ in 0..10 {
            dispatch_topic_frame(&mut subscribers, &topic_frame(sub_id, &big));
        }
        assert_eq!(drain(&rx).len(), 3, "only three fit in the byte budget");
        assert_eq!(rx.dropped(), 7);

        // Receiving frees the budget again, and an empty backlog takes a
        // frame of any size.
        let huge = vec![0u8; TopicBufferLimits::LIVE.bytes + 1];
        dispatch_topic_frame(&mut subscribers, &topic_frame(sub_id, &huge));
        assert_eq!(drain(&rx).len(), 1);
        assert_eq!(rx.dropped(), 7);
    }

    /// A recording takes a much larger backlog than a live view: a short disk
    /// stall must not lose large frames the live limit would drop.
    #[test]
    fn record_limits_absorb_what_live_limits_drop() {
        let frame = vec![0u8; 6 * 1024 * 1024]; // a 1080p RGB image
        for (limits, expected) in [
            (TopicBufferLimits::LIVE, 10),
            (TopicBufferLimits::RECORD, 100),
        ] {
            let sub_id = Uuid::new_v4();
            let (subscriber, rx) = topic_channel(limits);
            let mut subscribers: TopicSubscribers = HashMap::from([(sub_id, subscriber)]);
            for _ in 0..100 {
                dispatch_topic_frame(&mut subscribers, &topic_frame(sub_id, &frame));
            }
            assert_eq!(drain(&rx).len(), expected, "{limits:?}");
        }
    }

    #[test]
    fn topic_frame_for_a_dropped_receiver_unsubscribes_it() {
        let (sub_id, mut subscribers, rx) = one_subscriber();
        drop(rx);

        dispatch_topic_frame(&mut subscribers, &topic_frame(sub_id, b"data"));
        assert!(subscribers.is_empty());
    }

    #[test]
    fn short_or_unknown_topic_frames_are_ignored() {
        let (sub_id, mut subscribers, rx) = one_subscriber();

        dispatch_topic_frame(&mut subscribers, &sub_id.as_bytes()[..15]);
        dispatch_topic_frame(&mut subscribers, &topic_frame(Uuid::new_v4(), b"data"));
        assert!(drain(&rx).is_empty());

        // An empty payload is still a frame.
        dispatch_topic_frame(&mut subscribers, sub_id.as_bytes());
        assert_eq!(drain(&rx), vec![Vec::<u8>::new()]);
    }
}
