use crate::DaemonCoordinatorEvent;
use dora_core::uhlc::HLC;
use dora_message::{
    common::{DaemonId, Timestamped},
    coordinator_to_daemon::RegisterResult,
    daemon_to_coordinator::{
        CoordinatorRequest, DaemonCoordinatorReply, DaemonEvent, DaemonRegisterRequest,
        MAX_DAEMON_TEXT_MESSAGE_BYTES, MAX_TOPIC_DEBUG_FRAME_BYTES, MAX_TOPIC_DEBUG_PAYLOAD_BYTES,
        encode_topic_debug_frame, topic_debug_frame_len,
    },
    ws_protocol::WsResponse,
};
use eyre::eyre;
use futures::{Sink, SinkExt, StreamExt};
use std::{net::SocketAddr, sync::Arc, time::Duration};
use tokio::sync::{OwnedSemaphorePermit, Semaphore, mpsc, oneshot};
use tokio_stream::{Stream, wrappers::ReceiverStream};
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

const DAEMON_COORDINATOR_RETRY_INITIAL: Duration = Duration::from_secs(1);
const DAEMON_COORDINATOR_RETRY_MAX: Duration = Duration::from_secs(30);
/// Maximum number of consecutive failed connection attempts before giving up.
const DAEMON_COORDINATOR_RETRY_LIMIT: u32 = 50;
const REGISTER_TIMEOUT: Duration = Duration::from_secs(30);
/// Timeout for the cross-machine register flow: awaiting the ResolveMachine
/// reply here and the register ack in the tensor-pool extension.
#[cfg(feature = "tensor-pool")]
pub const CROSS_REGISTER_TIMEOUT: Duration = Duration::from_secs(5);

#[derive(Debug)]
pub struct CoordinatorEvent {
    pub event: DaemonCoordinatorEvent,
    pub reply_tx: oneshot::Sender<Option<DaemonCoordinatorReply>>,
}

/// Capacity of the outbound control channel (daemon events and replies).
const CONTROL_CHANNEL_CAPACITY: usize = 64;
/// Capacity of the outbound topic debug channel. Frames beyond it are dropped
/// (see [`CoordinatorSender::try_send_topic_debug_frame`]).
const TOPIC_DEBUG_CHANNEL_CAPACITY: usize = 64;
/// Memory the queued topic debug frames may hold in total.
///
/// Debug frames are variable-sized — a few bytes for a scalar output, up to
/// [`MAX_TOPIC_DEBUG_FRAME_BYTES`] for a camera one — so a message count is
/// not a memory bound on its own: the capacity above would admit a gigabyte of
/// camera frames whenever the coordinator falls behind. Each frame takes one
/// permit per byte before it is queued and releases them once it has been
/// written, so this is the real ceiling and the capacity only caps how many
/// tiny frames can queue.
///
/// Four largest-possible frames, which leaves those slots to the frames a byte
/// bound is beside the point for. Depth is worth having here because this
/// queue's consumer is the socket writer, which does nothing but write:
/// absorbing a burst while one write is in flight is what the queue is for.
/// The coordinator's ingress queue is bounded the other way round — one frame,
/// no byte budget (`ws_daemon::TOPIC_DEBUG_CHANNEL_CAPACITY` there) — because
/// its consumer is the main event loop, where a queued debug frame is work
/// done ahead of control events rather than behind them.
const TOPIC_DEBUG_QUEUE_BYTES: usize = 4 * MAX_TOPIC_DEBUG_FRAME_BYTES;

/// A topic debug frame waiting to be written, holding its share of
/// [`TOPIC_DEBUG_QUEUE_BYTES`] until it has been.
struct QueuedDebugFrame {
    message: Message,
    /// Released on drop, i.e. once the writer is done with `message`.
    _budget: OwnedSemaphorePermit,
}

/// Wraps the WS send channels for fire-and-forget daemon events to the coordinator.
#[derive(Clone)]
pub struct CoordinatorSender {
    sender: mpsc::Sender<String>,
    /// Topic debug frames, kept off `sender` so they can never delay a control
    /// message: the writer only drains this when the control side is empty.
    topic_debug: mpsc::Sender<QueuedDebugFrame>,
    /// One permit per byte of [`TOPIC_DEBUG_QUEUE_BYTES`], held by each queued
    /// frame for its own size.
    topic_debug_budget: Arc<Semaphore>,
    /// Negotiated at registration (`RegisterResult::Ok::binary_debug_frames`):
    /// send topic debug frames as WS binary messages rather than JSON
    /// `TopicDebugData`.
    binary_debug_frames: bool,
}

#[derive(Debug)]
pub enum TrySendEventError {
    InvalidUtf8(std::str::Utf8Error),
    Encode(eyre::Report),
    /// Larger than some hop on the way to the subscriber accepts: sending it
    /// would cost the coordinator connection (an oversized message closes it)
    /// or the subscription (an oversized frame fails the CLI's socket).
    TooLarge {
        bytes: usize,
        limit: usize,
    },
    Full,
    Closed,
}

impl std::fmt::Display for TrySendEventError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidUtf8(err) => write!(f, "event message not UTF-8: {err}"),
            Self::Encode(err) => write!(f, "failed to encode event message: {err}"),
            Self::TooLarge { bytes, limit } => write!(
                f,
                "{bytes} bytes exceeds the {limit}-byte limit a topic debug frame has to fit"
            ),
            Self::Full => write!(f, "WS send channel full"),
            Self::Closed => write!(f, "WS send channel closed"),
        }
    }
}

impl std::error::Error for TrySendEventError {}

impl CoordinatorSender {
    fn format_event_message(message: &[u8]) -> Result<String, TrySendEventError> {
        let params_str = std::str::from_utf8(message).map_err(TrySendEventError::InvalidUtf8)?;
        let id = Uuid::new_v4();
        Ok(format!(
            r#"{{"id":"{id}","method":"daemon_event","params":{params_str}}}"#
        ))
    }

    /// Send a serialized event message to the coordinator (fire-and-forget).
    ///
    /// Embeds the raw JSON bytes directly to preserve u128 fidelity
    /// for uhlc::ID inside timestamps.
    pub async fn send_event(&self, message: &[u8]) -> eyre::Result<()> {
        let json = Self::format_event_message(message).map_err(|err| eyre!("{err}"))?;
        self.sender
            .send(json)
            .await
            .map_err(|_| eyre!("WS send channel closed"))
    }

    /// Send a request with a caller-controlled id so the reply can be
    /// routed back (see COORDINATOR_PENDING in resolve_machine).
    ///
    /// Unlike [`Self::send_event`], which wraps the message in a fresh
    /// envelope with a new id, this builds the single `daemon_event`
    /// envelope itself and expects bare `Timestamped` serialization bytes
    /// as `params`, so the coordinator receives exactly one envelope
    /// layer with the caller's request id.
    pub async fn send_event_with_id(&self, request_id: Uuid, params: &[u8]) -> eyre::Result<()> {
        let json = format!(
            r#"{{"id":"{request_id}","method":"daemon_event","params":{}}}"#,
            std::str::from_utf8(params).map_err(|_| eyre::eyre!("params must be utf-8"))?
        );
        self.sender
            .send(json)
            .await
            .map_err(|_| eyre!("WS send channel closed"))
    }

    /// Queue a topic debug frame for the coordinator, dropping it if the debug
    /// queue is full — either on its message count or, for large frames, on its
    /// [`TOPIC_DEBUG_QUEUE_BYTES`] budget.
    ///
    /// Encoded as a WS binary message when the coordinator negotiated
    /// `binary_debug_frames`, otherwise as the JSON `TopicDebugData` event
    /// every coordinator understands. Either way it goes on the debug channel,
    /// which the writer serves only after the control channels.
    ///
    /// Nothing is copied until the frame has somewhere to go: the queue slot
    /// and, in binary mode, the frame's bytes are taken first, so a frame that
    /// will be dropped never costs a multi-megabyte encode. Both are given
    /// back if a later step fails, and a reserved slot cannot be taken by
    /// anyone else, so the queueing at the end cannot fail.
    ///
    /// A frame larger than the coordinator accepts in that encoding — or than
    /// the CLI can be handed afterwards ([`MAX_TOPIC_DEBUG_PAYLOAD_BYTES`]) —
    /// is dropped here (`TooLarge`): the coordinator closes the connection on
    /// an oversized message, which would take the control plane down with it,
    /// and an oversized *payload* would fail the subscriber's socket on the
    /// next hop.
    pub fn try_send_topic_debug_frame(
        &self,
        daemon_id: &DaemonId,
        clock: &HLC,
        dataflow_id: Uuid,
        subscription_ids: Vec<Uuid>,
        payload: Vec<u8>,
    ) -> Result<(), TrySendEventError> {
        // Queue space first, before anything is rendered or copied.
        let slot = self.topic_debug.try_reserve().map_err(|err| match err {
            mpsc::error::TrySendError::Full(()) => TrySendEventError::Full,
            mpsc::error::TrySendError::Closed(()) => TrySendEventError::Closed,
        })?;
        let (message, budget) = if self.binary_debug_frames {
            // A binary frame's size follows from the payload, so both limits
            // and the byte budget are settled before the payload is copied.
            if payload.len() > MAX_TOPIC_DEBUG_PAYLOAD_BYTES {
                return Err(TrySendEventError::TooLarge {
                    bytes: payload.len(),
                    limit: MAX_TOPIC_DEBUG_PAYLOAD_BYTES,
                });
            }
            let bytes = topic_debug_frame_len(subscription_ids.len(), payload.len());
            if bytes > MAX_TOPIC_DEBUG_FRAME_BYTES {
                return Err(TrySendEventError::TooLarge {
                    bytes,
                    limit: MAX_TOPIC_DEBUG_FRAME_BYTES,
                });
            }
            let budget = self.reserve_debug_bytes(bytes)?;
            let frame = encode_topic_debug_frame(dataflow_id, &subscription_ids, &payload)
                .map_err(TrySendEventError::Encode)?;
            (Message::Binary(frame.into()), budget)
        } else {
            // Every payload byte takes at least two characters in the JSON
            // number array (a digit and a separator), so a payload past half
            // the limit cannot fit: skip rendering megabytes of text only to
            // throw them away.
            if payload.len() > MAX_DAEMON_TEXT_MESSAGE_BYTES / 2 {
                return Err(TrySendEventError::TooLarge {
                    bytes: payload.len().saturating_mul(2),
                    limit: MAX_DAEMON_TEXT_MESSAGE_BYTES,
                });
            }
            let event = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: daemon_id.clone(),
                    event: DaemonEvent::TopicDebugData {
                        dataflow_id,
                        subscription_ids,
                        payload,
                    },
                },
                timestamp: clock.new_timestamp(),
            })
            .map_err(|err| TrySendEventError::Encode(err.into()))?;
            let json = Self::format_event_message(&event)?;
            if json.len() > MAX_DAEMON_TEXT_MESSAGE_BYTES {
                return Err(TrySendEventError::TooLarge {
                    bytes: json.len(),
                    limit: MAX_DAEMON_TEXT_MESSAGE_BYTES,
                });
            }
            // A JSON message's length is only known once it is rendered, but
            // the check above keeps that bounded by the 1 MiB text limit — the
            // copy this path can waste is nothing like a camera frame.
            let budget = self.reserve_debug_bytes(json.len())?;
            (Message::Text(json.into()), budget)
        };
        slot.send(QueuedDebugFrame {
            message,
            _budget: budget,
        });
        Ok(())
    }

    /// Take `bytes` off the queue's [`TOPIC_DEBUG_QUEUE_BYTES`] budget, or
    /// report `Full` if that much is not left. The permit rides with the queued
    /// frame and frees the bytes once it has been written.
    fn reserve_debug_bytes(&self, bytes: usize) -> Result<OwnedSemaphorePermit, TrySendEventError> {
        let bytes = u32::try_from(bytes).map_err(|_| TrySendEventError::Full)?;
        Arc::clone(&self.topic_debug_budget)
            .try_acquire_many_owned(bytes)
            .map_err(|_| TrySendEventError::Full)
    }

    /// Build a detached sender (and its receiver) for tests that only need a
    /// distinct, valid `CoordinatorSender` instance.
    #[cfg(test)]
    pub(crate) fn for_test() -> (Self, mpsc::Receiver<String>) {
        let (sender, rx) = mpsc::channel(8);
        let (topic_debug, _) = mpsc::channel(1);
        (
            Self {
                sender,
                topic_debug,
                topic_debug_budget: Arc::new(Semaphore::new(TOPIC_DEBUG_QUEUE_BYTES)),
                binary_debug_frames: false,
            },
            rx,
        )
    }
}

/// Pending daemon→coordinator request replies: request id -> reply value.
/// The coordinator answers daemon requests in the same `daemon_event`
/// envelope, so the receive loop routes these replies to the pending
/// caller (see `register`) before dispatching them as commands.
static COORDINATOR_PENDING: std::sync::LazyLock<
    std::sync::Mutex<
        std::collections::HashMap<Uuid, tokio::sync::oneshot::Sender<serde_json::Value>>,
    >,
> = std::sync::LazyLock::new(|| std::sync::Mutex::new(std::collections::HashMap::new()));

/// Register with the coordinator.
///
/// Returns the assigned id, the zenoh endpoints of the daemons that were
/// already registered (see `RegisterResult::Ok::peer_zenoh_endpoints`), the
/// event sender and the incoming event stream.
pub async fn register(
    addr: SocketAddr,
    machine_id: Option<String>,
    labels: std::collections::BTreeMap<String, String>,
    zenoh: crate::ZenohRegistration,
    clock: Arc<HLC>,
) -> eyre::Result<(
    DaemonId,
    Option<String>,
    Vec<String>,
    CoordinatorSender,
    impl Stream<Item = Timestamped<CoordinatorEvent>>,
)> {
    let display_url = format!("ws://{addr}/api/daemon");
    let auth_token = dora_message::auth::discover_token();
    let ws_stream = {
        let mut backoff = DAEMON_COORDINATOR_RETRY_INITIAL;
        let mut attempts: u32 = 0;
        loop {
            let request = {
                let mut req = tokio_tungstenite::tungstenite::http::Request::builder()
                    .uri(&display_url)
                    .header("Host", addr.to_string())
                    .header("Connection", "Upgrade")
                    .header("Upgrade", "websocket")
                    .header("Sec-WebSocket-Version", "13")
                    .header(
                        "Sec-WebSocket-Key",
                        tokio_tungstenite::tungstenite::handshake::client::generate_key(),
                    );
                if let Some(ref token) = auth_token {
                    req = req.header("Authorization", format!("Bearer {}", token.as_hex()));
                }
                req.body(()).expect("valid WS request")
            };
            match tokio_tungstenite::connect_async(request).await {
                Ok((stream, _)) => break stream,
                Err(err) => {
                    attempts += 1;
                    if attempts >= DAEMON_COORDINATOR_RETRY_LIMIT {
                        return Err(eyre::eyre!(
                            "failed to connect to coordinator at {display_url} after {attempts} attempts: {err}"
                        ));
                    }
                    // Add +/- 25% jitter to prevent a thundering herd of
                    // daemons reconnecting in lockstep.
                    let sleep_duration = jittered_backoff(backoff, rand_jitter_millis());
                    tracing::warn!(
                        "Could not connect to WS at {display_url}: {err}. Retrying in {sleep_duration:#?} ({attempts}/{DAEMON_COORDINATOR_RETRY_LIMIT}).."
                    );
                    tokio::time::sleep(sleep_duration).await;
                    backoff = (backoff * 2).min(DAEMON_COORDINATOR_RETRY_MAX);
                }
            }
        }
    };

    // Reserve the zenoh listen port now, with the coordinator socket already
    // up, so the window between reserving the port and zenoh binding it is one
    // register round-trip rather than however long the retry loop above took.
    // Only on the first connect: a reconnect reuses the still-open session's
    // port, and re-reserving would hand out one nothing is listening on.
    // Only a derived bind reaches the reservation here; an explicit one was
    // reserved before the connect loop precisely so its failures stay fatal
    // instead of being retried as connectivity problems. A derived reservation
    // never fails fatally — it degrades to `None`.
    let reserved_listen_endpoint = match zenoh.reserved {
        Some(ep) => Some(ep),
        None => crate::reserve_zenoh_listen_endpoint(zenoh.bind)?,
    };
    let advertised_listen_endpoint = match zenoh.advertise {
        crate::AdvertiseListener::Reserved => reserved_listen_endpoint.clone(),
        crate::AdvertiseListener::Bound(bound) => bound,
    };

    let (mut ws_tx, mut ws_rx) = ws_stream.split();

    // Channel for outgoing messages (daemon events + command replies).
    // The coordinator sender writes to this, and the writer task below reads
    // and forwards to WS.
    let (send_tx, send_rx) = mpsc::channel::<String>(CONTROL_CHANNEL_CAPACITY);
    // Topic debug frames get their own channel so they queue behind nothing
    // but each other; see `run_coordinator_ws_writer`.
    let (topic_debug_tx, topic_debug_rx) =
        mpsc::channel::<QueuedDebugFrame>(TOPIC_DEBUG_CHANNEL_CAPACITY);
    let topic_debug_budget = Arc::new(Semaphore::new(TOPIC_DEBUG_QUEUE_BYTES));

    // Send Register request.
    // Serialize params via to_string (not to_value) to preserve u128 fidelity
    // for uhlc::ID(NonZeroU128) inside the timestamp.
    let register_params_json = serde_json::to_string(&Timestamped {
        inner: CoordinatorRequest::Register(DaemonRegisterRequest::with_zenoh_endpoint(
            machine_id,
            labels,
            advertised_listen_endpoint,
        )),
        timestamp: clock.new_timestamp(),
    })?;
    let register_id = Uuid::new_v4();
    let register_json = format!(
        r#"{{"id":"{register_id}","method":"daemon_event","params":{register_params_json}}}"#
    );
    ws_tx
        .send(Message::Text(register_json.into()))
        .await
        .map_err(|e| eyre!("failed to send register request: {e}"))?;

    // Wait for register reply with timeout.
    // The coordinator's register handler sends back Timestamped<RegisterResult>
    // wrapped in a WsRequest with method "daemon_event".
    let register_result = tokio::time::timeout(REGISTER_TIMEOUT, async {
        loop {
            let msg = ws_rx
                .next()
                .await
                .ok_or_else(|| eyre!("WS connection closed before register reply"))?
                .map_err(|e| eyre!("WS error during register: {e}"))?;

            let Message::Text(text) = msg else {
                continue;
            };

            // Parse directly from raw text to preserve u128 fidelity.
            let raw: RegisterReplyRaw = match serde_json::from_str(&text) {
                Ok(r) => r,
                Err(_) => continue,
            };
            let result = raw.params;

            if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
                tracing::warn!("failed to update timestamp after register: {err}");
            }

            break eyre::Ok(result.inner);
        }
    })
    .await
    .map_err(|_| eyre!("timeout waiting for register reply from coordinator"))??;
    let binary_debug_frames = register_result.binary_debug_frames();
    let (daemon_id, peer_zenoh_endpoints) = register_result.into_parts()?;
    let peer_zenoh_endpoints = usable_peer_endpoints(addr, peer_zenoh_endpoints);

    tracing::info!("Connected to dora-coordinator at ws://{addr}/api/daemon");

    let (tx, rx) = mpsc::channel(1);

    // The coordinator connection is serviced by two cooperating tasks that
    // split the WebSocket:
    //
    //  * a **writer** task is the sole owner/writer of `ws_tx`. It drains both
    //    the external fire-and-forget events (`send_rx`) and internal frames
    //    from the reader (command responses + pongs, via `internal_rx`).
    //  * a **reader** task owns `ws_rx`, forwards each command to the daemon's
    //    main event loop, awaits its reply, and hands responses/pongs to the
    //    writer.
    //
    // Keeping the outbound drain in its own task is what prevents the deadlock
    // fixed here (dora-rs/dora#3164): processing a command on the main loop can
    // require pushing outbound messages through `send_tx`, so a single loop
    // that both awaited replies and drained `send_rx` would wedge — a command
    // whose handling emits a burst larger than `send_tx`'s capacity blocks the
    // main loop on a full `send_tx` while the loop is parked awaiting that
    // command's reply, so `send_rx` never drains and the reply never comes.
    // With an independent writer, `send_rx` always drains, so no internal cycle
    // can form; only genuine peer backpressure can slow transmission.
    let (internal_tx, internal_rx) = mpsc::channel::<OutboundFrame>(64);

    tokio::spawn(run_coordinator_ws_writer(
        ws_tx,
        send_rx,
        internal_rx,
        topic_debug_rx,
    ));

    let task_clock = clock.clone();
    tokio::spawn(run_coordinator_ws_reader(
        ws_rx,
        tx,
        internal_tx,
        task_clock,
    ));

    Ok((
        daemon_id,
        reserved_listen_endpoint,
        peer_zenoh_endpoints,
        CoordinatorSender {
            sender: send_tx,
            topic_debug: topic_debug_tx,
            topic_debug_budget,
            binary_debug_frames,
        },
        ReceiverStream::new(rx),
    ))
}

/// Drop loopback endpoints from the peer list unless this daemon reached the
/// coordinator (at `coordinator`) over loopback itself.
///
/// The coordinator already hands loopback endpoints only to same-host daemons
/// (`DaemonConnections::zenoh_endpoints_for`), but a 1.0.x coordinator hands
/// out whatever it was told, and daemons now report loopback listeners too. A
/// daemon on another host would dial its own loopback.
fn usable_peer_endpoints(coordinator: SocketAddr, endpoints: Vec<String>) -> Vec<String> {
    if coordinator.ip().to_canonical().is_loopback() {
        return endpoints;
    }
    endpoints
        .into_iter()
        .filter(|endpoint| !dora_core::topics::zenoh_endpoint_is_loopback(endpoint))
        .collect()
}

/// A frame the reader hands to the writer for transmission. The writer is the
/// sole owner of the WebSocket sink, so responses and pongs route through it
/// rather than being written by the reader directly.
enum OutboundFrame {
    /// A message to write to the coordinator socket.
    Ws(Message),
    /// Fire a `DestroyResult` completion notification and then close the
    /// connection. Routed through the writer (rather than fired directly in the
    /// reader) so it happens only *after* the preceding `DestroyResult`
    /// response has been flushed to the socket — preserving the original
    /// flush-then-notify ordering across the reader/writer split.
    DestroyNotify(oneshot::Sender<()>),
}

/// Outbound half of the coordinator WS connection: the sole writer of `ws_tx`.
///
/// Drains `send_rx` (external fire-and-forget events) and `internal_rx`
/// (reader-produced responses/pongs) until the connection errors, the reader
/// stops (dropping `internal_tx`), or a `DestroyNotify` frame ends it. Because
/// this drain runs independently of command/reply processing, nothing the
/// reader does can stall it — see `register` and dora-rs/dora#3164.
///
/// `topic_debug_rx` (topic debug frames) is served only when neither control
/// channel has anything ready. A debug frame can be megabytes, so sharing a
/// queue with control traffic let a `dora topic` subscription on a large
/// output hold a stop reply or heartbeat behind a backlog of frames
/// (dora-rs/dora#3535). Now a control message waits for at most the one debug
/// frame already being written. The two control channels keep their existing
/// unbiased interleaving with each other. What may pile up behind that is
/// bounded by bytes, not just by frames — see [`TOPIC_DEBUG_QUEUE_BYTES`].
async fn run_coordinator_ws_writer<Tx>(
    mut ws_tx: Tx,
    mut send_rx: mpsc::Receiver<String>,
    mut internal_rx: mpsc::Receiver<OutboundFrame>,
    mut topic_debug_rx: mpsc::Receiver<QueuedDebugFrame>,
) where
    Tx: Sink<Message> + Unpin,
{
    enum Control {
        Internal(Option<OutboundFrame>),
        Outgoing(Option<String>),
    }

    loop {
        tokio::select! {
            biased;
            // Every `recv` here is cancel-safe, so losing the race to the
            // debug arm drops no control message.
            control = async {
                tokio::select! {
                    frame = internal_rx.recv() => Control::Internal(frame),
                    outgoing = send_rx.recv() => Control::Outgoing(outgoing),
                }
            } => match control {
                Control::Internal(Some(OutboundFrame::Ws(msg))) => {
                    if ws_tx.send(msg).await.is_err() {
                        break;
                    }
                }
                Control::Internal(Some(OutboundFrame::DestroyNotify(notify))) => {
                    let _ = notify.send(());
                    break;
                }
                // Reader stopped; all frames it queued (FIFO) are already
                // flushed, so close the write half by dropping `ws_tx`.
                Control::Internal(None) => break,
                Control::Outgoing(Some(text)) => {
                    if ws_tx.send(Message::Text(text.into())).await.is_err() {
                        break;
                    }
                }
                // CoordinatorSender dropped: nothing more to send.
                Control::Outgoing(None) => break,
            },
            debug = topic_debug_rx.recv() => match debug {
                // `_budget` drops with `frame` at the end of this arm, giving
                // the queue back this frame's bytes only once it is written.
                Some(frame) => {
                    if ws_tx.send(frame.message).await.is_err() {
                        break;
                    }
                }
                // Dropped together with `send_rx`'s sender (both live in
                // `CoordinatorSender`), so this is the same shutdown.
                None => break,
            },
        }
    }
}

/// Inbound half of the coordinator WS connection: reads frames from `ws_rx`,
/// forwards commands to the daemon's main event loop over `tx`, awaits each
/// reply, and routes responses/pongs to the writer over `internal_tx`.
///
/// Each command's reply is awaited before the next frame is read, so at most
/// one command is in flight — unchanged from the pre-split loop. The reply
/// await cannot deadlock because the writer drains `send_rx` independently
/// (dora-rs/dora#3164).
async fn run_coordinator_ws_reader<Rx, E>(
    mut ws_rx: Rx,
    tx: mpsc::Sender<Timestamped<CoordinatorEvent>>,
    internal_tx: mpsc::Sender<OutboundFrame>,
    clock: Arc<HLC>,
) where
    Rx: Stream<Item = Result<Message, E>> + Unpin,
    E: std::fmt::Display,
{
    while let Some(msg) = ws_rx.next().await {
        let text = match msg {
            Ok(Message::Text(text)) => text,
            Ok(Message::Close(_)) => break,
            Ok(Message::Ping(data)) => {
                // Route the pong through the writer (the sole ws_tx owner).
                if internal_tx
                    .send(OutboundFrame::Ws(Message::Pong(data)))
                    .await
                    .is_err()
                {
                    break;
                }
                continue;
            }
            Ok(_) => continue,
            Err(e) => {
                tracing::warn!("WS coordinator connection error: {e}");
                break;
            }
        };

        // Replies to our own daemon→coordinator requests (e.g. ResolveMachine)
        // arrive in the same daemon_event envelope as commands, but with a
        // different params type (`Timestamped<ResolveMachineReply>`) that the
        // typed `CoordinatorCommandRaw` parse below would reject. Route them to
        // the pending caller by id before the command parse.
        if let Ok(reply) = serde_json::from_str::<ReplyRouteRaw>(&text) {
            let pending = COORDINATOR_PENDING
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .remove(&reply.id);
            if let Some(tx) = pending {
                let _ = tx.send(reply.params.unwrap_or(serde_json::Value::Null));
                continue;
            }
        }

        // Parse directly from raw text to preserve u128 fidelity for uhlc::ID
        // inside timestamps.
        let raw: CoordinatorCommandRaw = match serde_json::from_str(&text) {
            Ok(r) => r,
            Err(e) => {
                tracing::warn!("failed to parse coordinator WS message: {e}");
                continue;
            }
        };

        let request_id = raw.id;
        let needs_reply = raw.method == "daemon_command";
        let event = raw.params;

        if let Err(err) = clock.update_with_timestamp(&event.timestamp) {
            tracing::warn!("failed to update daemon clock: {err}");
        }

        let (reply_tx, reply_rx) = oneshot::channel();
        if tx
            .send(Timestamped {
                inner: CoordinatorEvent {
                    event: event.inner,
                    reply_tx,
                },
                timestamp: event.timestamp,
            })
            .await
            .is_err()
        {
            break;
        }

        let Ok(reply) = reply_rx.await else {
            tracing::warn!("daemon sent no reply");
            continue;
        };

        if let Some(reply) = reply {
            if needs_reply {
                let response = match serde_json::to_value(&reply) {
                    Ok(val) => WsResponse::ok(request_id, val),
                    Err(e) => {
                        tracing::error!("failed to serialize reply: {e}");
                        WsResponse::err(request_id, format!("{e}"))
                    }
                };
                if let Ok(json) = serde_json::to_string(&response)
                    && internal_tx
                        .send(OutboundFrame::Ws(Message::Text(json.into())))
                        .await
                        .is_err()
                {
                    break;
                }
            }
            if let DaemonCoordinatorReply::DestroyResult { notify, .. } = reply {
                if let Some(notify) = notify {
                    // Hand the notify to the writer so it fires only after the
                    // response above has been flushed, then closes the socket.
                    let _ = internal_tx.send(OutboundFrame::DestroyNotify(notify)).await;
                }
                break;
            }
        }
    }
}

/// Resolve a machine id through the coordinator. Returns false when the
/// machine is unknown or no coordinator is reachable (warn-and-skip).
///
/// The coordinator replies over the same `daemon_event` envelope the
/// request was sent in (params: `Timestamped<ResolveMachineReply>`); the
/// receive loop in `register` routes the reply here by request id.
#[cfg(feature = "tensor-pool")]
pub(crate) async fn resolve_machine(
    coordinator_sender: &CoordinatorSender,
    clock: &Arc<HLC>,
    machine_id: &str,
) -> Option<std::net::SocketAddr> {
    let request_id = Uuid::new_v4();
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    COORDINATOR_PENDING
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .insert(request_id, reply_tx);
    let params = match serde_json::to_string(&Timestamped {
        inner: CoordinatorRequest::ResolveMachine {
            machine_id: machine_id.to_string(),
        },
        timestamp: clock.new_timestamp(),
    }) {
        Ok(p) => p,
        Err(_) => {
            COORDINATOR_PENDING
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .remove(&request_id);
            return None;
        }
    };
    if coordinator_sender
        .send_event_with_id(request_id, params.as_bytes())
        .await
        .is_err()
    {
        COORDINATOR_PENDING
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remove(&request_id);
        return None;
    }
    match tokio::time::timeout(CROSS_REGISTER_TIMEOUT, reply_rx).await {
        Ok(Ok(value)) => {
            let result = value
                .get("inner")
                .and_then(|v| v.get("ResolveMachineResult"));
            let found = result
                .and_then(|v| v.get("found"))
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
            if !found {
                return None;
            }
            // The target daemon's WS peer address (its direct-TCP data
            // listener lives on the same IP).
            result
                .and_then(|v| v.get("address"))
                .and_then(|v| v.as_str())
                .and_then(|s| s.parse::<std::net::SocketAddr>().ok())
        }
        Ok(Err(_)) => {
            // Sender dropped without sending. In the normal flow this
            // cannot happen: the routing block removes the pending entry
            // *before* sending the reply, so the entry is already gone
            // and there is nothing to clean up here (only the timeout
            // branch below can leave a stale entry).
            None
        }
        Err(_) => {
            // Timeout: drop the stale pending entry so it cannot leak.
            COORDINATOR_PENDING
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .remove(&request_id);
            None
        }
    }
}

/// Helper for deserializing register reply directly from raw JSON text,
/// bypassing `serde_json::Value` to preserve u128 fidelity for uhlc::ID.
#[derive(serde::Deserialize)]
struct RegisterReplyRaw {
    params: Timestamped<RegisterResult>,
}

/// Helper for routing replies to pending daemon→coordinator requests by
/// id, parsing only the fields routing needs (`id` + optional `params`).
/// Like [`RegisterReplyRaw`], a bare Deserialize struct (not a full
/// `serde_json::Value` parse of the whole message) is used.
#[derive(serde::Deserialize)]
struct ReplyRouteRaw {
    id: Uuid,
    params: Option<serde_json::Value>,
}

/// Helper for deserializing coordinator commands directly from raw JSON text,
/// bypassing `serde_json::Value` to preserve u128 fidelity for uhlc::ID.
#[derive(serde::Deserialize)]
struct CoordinatorCommandRaw {
    id: Uuid,
    method: String,
    params: Timestamped<DaemonCoordinatorEvent>,
}

/// Jitter for reconnect backoff using a properly seeded random source.
fn rand_jitter_millis() -> u64 {
    use std::hash::{BuildHasher, Hasher};
    std::collections::hash_map::RandomState::new()
        .build_hasher()
        .finish()
}

/// Apply symmetric +/- 25% jitter to `backoff` to spread out daemons that
/// would otherwise reconnect in lockstep.
///
/// `rand` is an arbitrary value (e.g. from [`rand_jitter_millis`]); it is
/// mapped uniformly onto `-range..=+range` (where `range == backoff / 4`) and
/// added to `backoff`, so the result lies in
/// `[backoff - backoff/4, backoff + backoff/4]`.
fn jittered_backoff(backoff: Duration, rand: u64) -> Duration {
    let range = (backoff / 4).as_millis() as u64;
    // `rand % (2*range + 1)` is uniform in `0..=2*range`; subtracting `range`
    // recenters it to `-range..=+range`. (The previous code applied
    // `saturating_sub` to the unsigned value, which clamped the whole lower
    // half to 0 — so the jitter was actually `+0..=+range`, never negative,
    // and collapsed ~half of all draws onto exactly `backoff`.)
    let offset = (rand % (range * 2 + 1)) as i64 - range as i64;
    Duration::from_millis((backoff.as_millis() as u64).saturating_add_signed(offset))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What a 1.0.x coordinator may hand out: a same-host daemon's loopback
    /// listener next to a routable one. Only a daemon on the coordinator's
    /// host keeps the loopback entry.
    #[test]
    fn loopback_peer_endpoints_are_kept_only_next_to_the_coordinator() {
        let endpoints = || {
            vec![
                "tcp/127.0.0.1:5456".to_string(),
                "tcp/10.0.2.7:5456".to_string(),
            ]
        };
        let remote = SocketAddr::from(([10, 0, 2, 1], 6012));
        assert_eq!(
            usable_peer_endpoints(remote, endpoints()),
            ["tcp/10.0.2.7:5456"]
        );
        let local = SocketAddr::from(([127, 0, 0, 1], 6012));
        assert_eq!(usable_peer_endpoints(local, endpoints()), endpoints());
    }

    #[tokio::test]
    async fn send_event_with_id_sends_single_layer_envelope_with_caller_id() {
        let (sender, mut rx) = CoordinatorSender::for_test();
        let request_id = Uuid::new_v4();
        let params = br#"{"inner":{"ResolveMachine":{"machine_id":"host"}}}"#;
        sender
            .send_event_with_id(request_id, params)
            .await
            .expect("send should succeed");
        let json = rx.recv().await.expect("message should be queued");
        // Exactly one envelope layer, echoing the caller's request id
        // (the double-wrap bug produced two layers and a fresh id that
        // the reply routing could never match).
        let expected = format!(
            r#"{{"id":"{request_id}","method":"daemon_event","params":{}}}"#,
            std::str::from_utf8(params).unwrap()
        );
        assert_eq!(json, expected);
        // The coordinator's parse target: `params` must be the bare
        // request body, not another envelope.
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(
            parsed["id"],
            serde_json::Value::String(request_id.to_string())
        );
        assert_eq!(
            parsed["method"],
            serde_json::Value::String("daemon_event".into())
        );
        assert_eq!(
            parsed["params"]["inner"]["ResolveMachine"]["machine_id"],
            serde_json::Value::String("host".into())
        );
    }

    #[test]
    fn jittered_backoff_is_centered_and_symmetric() {
        let backoff = Duration::from_secs(4); // 4000ms, range = 1000ms
        let range_ms = 1000;
        let base_ms = 4000;

        // rand == 0 maps to the minimum (backoff - range).
        assert_eq!(
            jittered_backoff(backoff, 0),
            Duration::from_millis(base_ms - range_ms)
        );
        // rand == 2*range maps to the maximum (backoff + range).
        assert_eq!(
            jittered_backoff(backoff, range_ms * 2),
            Duration::from_millis(base_ms + range_ms)
        );
        // rand == range maps to exactly backoff.
        assert_eq!(
            jittered_backoff(backoff, range_ms),
            Duration::from_millis(base_ms)
        );
    }

    #[test]
    fn jittered_backoff_stays_within_bounds_and_can_decrease() {
        let backoff = Duration::from_secs(8); // range = 2000ms
        let range_ms = 2000u128;
        let lo = backoff.as_millis() - range_ms;
        let hi = backoff.as_millis() + range_ms;
        let mut saw_below = false;
        for rand in 0..(range_ms as u64 * 2 + 1) {
            let ms = jittered_backoff(backoff, rand).as_millis();
            assert!((lo..=hi).contains(&ms), "out of range: {ms}");
            if ms < backoff.as_millis() {
                saw_below = true;
            }
        }
        // The pre-fix implementation could never sleep less than `backoff`.
        assert!(saw_below, "jitter never produced a value below backoff");
    }

    #[test]
    fn jittered_backoff_handles_zero_range() {
        // Sub-4ms backoff yields range == 0; must not divide/modulo by zero.
        let backoff = Duration::from_millis(3);
        assert_eq!(jittered_backoff(backoff, 12345), backoff);
    }

    /// Regression test for dora-rs/dora#3164: the outbound path must keep
    /// draining `send_rx` while a coordinator command's reply is still in
    /// flight. Otherwise a command whose handling emits a burst larger than the
    /// `send_tx` capacity wedges the daemon's main loop on a full `send_tx`, and
    /// the reply never arrives — a permanent deadlock.
    ///
    /// With the reader and writer split into separate tasks the burst drains
    /// independently of the reply await, so this completes; a single combined
    /// loop (the pre-fix shape) deadlocks and hits the timeout.
    #[tokio::test]
    async fn ws_writer_drains_outbound_while_command_reply_is_in_flight() {
        use futures::stream;

        let clock = Arc::new(HLC::default());

        // Outbound WS sink: unbounded, so the only thing that can stall the
        // outbound path is the writer failing to drain `send_rx` — the bug under
        // test — not sink backpressure.
        let (ws_out_tx, _ws_out_rx) = futures::channel::mpsc::unbounded::<Message>();

        // One `daemon_command`, then pending forever; the command's
        // `DestroyResult` reply is what ends the reader loop.
        let df_id = Uuid::new_v4();
        let event = DaemonCoordinatorEvent::AllNodesReady {
            dataflow_id: df_id,
            exited_before_subscribe: Vec::new(),
        };
        let params_json = serde_json::to_string(&Timestamped {
            inner: event,
            timestamp: clock.new_timestamp(),
        })
        .unwrap();
        let cmd_id = Uuid::new_v4();
        let cmd_json =
            format!(r#"{{"id":"{cmd_id}","method":"daemon_command","params":{params_json}}}"#);
        let ws_in = stream::iter(vec![Ok::<Message, std::io::Error>(Message::Text(
            cmd_json.into(),
        ))])
        .chain(stream::pending());

        let (tx, mut rx) = mpsc::channel::<Timestamped<CoordinatorEvent>>(1);
        // Same capacity as production (`register`). The mock main loop sends far
        // more than this before replying, so the reply can only be produced if
        // the writer keeps draining `send_rx` concurrently.
        let (send_tx, send_rx) = mpsc::channel::<String>(64);
        let (internal_tx, internal_rx) = mpsc::channel::<OutboundFrame>(64);
        // Held open for the test's duration: a closed debug channel ends the
        // writer, just as dropping the `CoordinatorSender` does in production.
        let (_topic_debug_tx, topic_debug_rx) = mpsc::channel::<QueuedDebugFrame>(1);

        let writer = tokio::spawn(run_coordinator_ws_writer(
            ws_out_tx,
            send_rx,
            internal_rx,
            topic_debug_rx,
        ));
        let reader = tokio::spawn(run_coordinator_ws_reader(
            ws_in,
            tx,
            internal_tx,
            clock.clone(),
        ));

        // Mock daemon main loop: on the command, emit a burst that exceeds the
        // send-channel capacity, then reply with `DestroyResult` to end the loop.
        let main_loop = tokio::spawn(async move {
            if let Some(ev) = rx.recv().await {
                for i in 0..500u32 {
                    send_tx
                        .send(format!("outbound-{i}"))
                        .await
                        .expect("writer must keep draining send_rx");
                }
                let _ = ev
                    .inner
                    .reply_tx
                    .send(Some(DaemonCoordinatorReply::DestroyResult {
                        result: Ok(()),
                        notify: None,
                    }));
            }
        });

        tokio::time::timeout(Duration::from_secs(10), async {
            reader.await.unwrap();
            writer.await.unwrap();
            main_loop.await.unwrap();
        })
        .await
        .expect("WS router deadlocked under outbound backpressure (#3164)");
    }

    /// Regression test for dora-rs/dora#3535: with the topic debug queue full,
    /// a control message still goes out next — ahead of every queued debug
    /// frame, not behind them.
    ///
    /// Everything is queued before the writer starts, so the order it writes
    /// in is decided by its select priority alone, not by which task ran first.
    #[tokio::test]
    async fn ws_writer_sends_control_before_queued_topic_debug_frames() {
        let (ws_out_tx, mut ws_out_rx) = futures::channel::mpsc::unbounded::<Message>();
        let (send_tx, send_rx) = mpsc::channel::<String>(CONTROL_CHANNEL_CAPACITY);
        let (internal_tx, internal_rx) = mpsc::channel::<OutboundFrame>(1);
        let (topic_debug_tx, topic_debug_rx) =
            mpsc::channel::<QueuedDebugFrame>(TOPIC_DEBUG_CHANNEL_CAPACITY);
        let budget = Arc::new(Semaphore::new(TOPIC_DEBUG_QUEUE_BYTES));

        let (debug_frame_count, big_frame) = (TOPIC_DEBUG_CHANNEL_CAPACITY, vec![0u8; 1 << 10]);
        let queue = |message: Message| {
            let _budget = Arc::clone(&budget)
                .try_acquire_many_owned(message.len() as u32)
                .ok()?;
            topic_debug_tx
                .try_send(QueuedDebugFrame { message, _budget })
                .ok()
        };
        for _ in 0..debug_frame_count {
            queue(Message::Binary(big_frame.clone().into()))
                .expect("debug queue has room up to its capacity");
        }
        assert!(
            queue(Message::Binary(big_frame.into())).is_none(),
            "the debug queue must be full for this test to mean anything"
        );
        send_tx.try_send("stop-reply".to_owned()).unwrap();

        let writer = tokio::spawn(run_coordinator_ws_writer(
            ws_out_tx,
            send_rx,
            internal_rx,
            topic_debug_rx,
        ));

        let first = tokio::time::timeout(Duration::from_secs(5), ws_out_rx.next())
            .await
            .expect("writer must make progress")
            .expect("writer must write a message");
        assert_eq!(
            first,
            Message::Text("stop-reply".into()),
            "a control message queued behind a full debug queue must be written first"
        );

        // The debug frames are still delivered once control is drained.
        for _ in 0..debug_frame_count {
            let msg = tokio::time::timeout(Duration::from_secs(5), ws_out_rx.next())
                .await
                .expect("writer must keep draining debug frames")
                .expect("debug frame");
            assert!(matches!(msg, Message::Binary(_)));
        }

        drop((send_tx, internal_tx, topic_debug_tx));
        tokio::time::timeout(Duration::from_secs(5), writer)
            .await
            .expect("writer must stop once its senders are gone")
            .unwrap();
    }

    fn debug_sender(
        binary_debug_frames: bool,
    ) -> (CoordinatorSender, mpsc::Receiver<QueuedDebugFrame>) {
        debug_sender_with_budget(binary_debug_frames, TOPIC_DEBUG_QUEUE_BYTES)
    }

    fn debug_sender_with_budget(
        binary_debug_frames: bool,
        budget_bytes: usize,
    ) -> (CoordinatorSender, mpsc::Receiver<QueuedDebugFrame>) {
        let (sender, _) = mpsc::channel(1);
        let (topic_debug, topic_debug_rx) = mpsc::channel(4);
        (
            CoordinatorSender {
                sender,
                topic_debug,
                topic_debug_budget: Arc::new(Semaphore::new(budget_bytes)),
                binary_debug_frames,
            },
            topic_debug_rx,
        )
    }

    /// Without the negotiated flag the daemon must keep sending the JSON
    /// `TopicDebugData` event: it is the only shape an older coordinator reads.
    #[test]
    fn topic_debug_frame_is_json_when_binary_frames_were_not_negotiated() {
        let (sender, mut rx) = debug_sender(false);
        let daemon_id = DaemonId::new(Some("A".to_string()));
        let (dataflow_id, subscription_id) = (Uuid::new_v4(), Uuid::new_v4());
        sender
            .try_send_topic_debug_frame(
                &daemon_id,
                &HLC::default(),
                dataflow_id,
                vec![subscription_id],
                vec![1, 2, 3],
            )
            .unwrap();

        let Ok(Message::Text(text)) = rx.try_recv().map(|frame| frame.message) else {
            panic!("expected a JSON text frame");
        };
        let raw: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(raw["method"], "daemon_event");
        let event = &raw["params"]["inner"]["Event"]["event"]["TopicDebugData"];
        assert_eq!(event["dataflow_id"], dataflow_id.to_string());
        assert_eq!(event["subscription_ids"][0], subscription_id.to_string());
        assert_eq!(event["payload"], serde_json::json!([1, 2, 3]));
    }

    #[test]
    fn topic_debug_frame_is_binary_when_binary_frames_were_negotiated() {
        let (sender, mut rx) = debug_sender(true);
        let (dataflow_id, subscription_id) = (Uuid::new_v4(), Uuid::new_v4());
        sender
            .try_send_topic_debug_frame(
                &DaemonId::new(Some("A".to_string())),
                &HLC::default(),
                dataflow_id,
                vec![subscription_id],
                vec![1, 2, 3],
            )
            .unwrap();

        let Ok(Message::Binary(data)) = rx.try_recv().map(|frame| frame.message) else {
            panic!("expected a binary frame");
        };
        let frame = dora_message::daemon_to_coordinator::decode_topic_debug_frame(&data).unwrap();
        assert_eq!(frame.dataflow_id, dataflow_id);
        assert_eq!(frame.subscription_ids, vec![subscription_id]);
        assert_eq!(frame.payload, &[1, 2, 3]);
    }

    /// A full debug queue drops the frame (reported as `Full`) rather than
    /// blocking the caller, which runs on the daemon's main loop.
    #[test]
    fn topic_debug_frame_is_dropped_when_the_debug_queue_is_full() {
        let (sender, _rx) = debug_sender(true);
        let send = || {
            sender.try_send_topic_debug_frame(
                &DaemonId::new(None),
                &HLC::default(),
                Uuid::new_v4(),
                vec![Uuid::new_v4()],
                Vec::new(),
            )
        };
        for _ in 0..4 {
            send().unwrap();
        }
        assert!(matches!(send(), Err(TrySendEventError::Full)));
    }

    /// The queue is bounded in bytes as well as in frames: a handful of camera
    /// frames must not be able to hold hundreds of MiB just because the message
    /// count is still under its capacity (dora-rs/dora#3535 review).
    #[test]
    fn topic_debug_frames_are_dropped_once_the_queue_byte_budget_is_spent() {
        let payload_len = 4 * 1024;
        // Room for two of these frames, not three, while the channel capacity
        // (4 messages) is nowhere near reached.
        let (sender, mut rx) = debug_sender_with_budget(
            true,
            2 * topic_debug_frame_len(1, payload_len) + payload_len / 2,
        );
        let send = || {
            sender.try_send_topic_debug_frame(
                &DaemonId::new(None),
                &HLC::default(),
                Uuid::new_v4(),
                vec![Uuid::new_v4()],
                vec![7; payload_len],
            )
        };

        send().unwrap();
        send().unwrap();
        assert!(
            matches!(send(), Err(TrySendEventError::Full)),
            "a frame past the byte budget must be dropped, not queued"
        );

        // Writing a queued frame returns its bytes, so the queue recovers.
        drop(rx.try_recv().expect("a queued frame"));
        send().expect("the budget frees up once a frame has been written");
    }

    /// Nothing is copied for a frame that will not be queued: the slot and the
    /// bytes are taken before the frame is built and given back if it turns out
    /// not to fit, so a dropped frame leaves the queue exactly as it found it.
    #[test]
    fn a_dropped_frame_leaves_neither_a_queue_slot_nor_budget_behind() {
        let (sender, _rx) = debug_sender(true);
        let free = sender.topic_debug_budget.available_permits();
        let send = |payload_len: usize| {
            sender.try_send_topic_debug_frame(
                &DaemonId::new(None),
                &HLC::default(),
                Uuid::new_v4(),
                vec![Uuid::new_v4()],
                vec![0; payload_len],
            )
        };

        assert!(matches!(
            send(MAX_TOPIC_DEBUG_PAYLOAD_BYTES + 1),
            Err(TrySendEventError::TooLarge { .. })
        ));
        assert_eq!(
            sender.topic_debug_budget.available_permits(),
            free,
            "a frame that was never queued must not hold any of the byte budget"
        );

        // The slot it reserved is back too: the queue still takes its full
        // capacity afterwards.
        for _ in 0..4 {
            send(8).expect("the queue is still empty");
        }
        assert!(matches!(send(8), Err(TrySendEventError::Full)));
    }

    /// A frame the coordinator would reject must be dropped at the daemon, not
    /// sent: the coordinator closes the connection on an oversized message. In
    /// JSON mode that limit is reached by payloads far below a camera frame.
    ///
    /// A payload the *CLI* could not be handed is dropped here too: the
    /// coordinator forwards it as a single frame, so one past
    /// `MAX_TOPIC_DEBUG_PAYLOAD_BYTES` would fail the subscriber's socket
    /// rather than be skipped by it — and there is no point putting those
    /// bytes on the wire for the coordinator to drop.
    #[test]
    fn topic_debug_frame_beyond_the_coordinator_limit_is_dropped() {
        let send = |binary: bool, payload_len: usize| {
            let (sender, mut rx) = debug_sender(binary);
            let result = sender.try_send_topic_debug_frame(
                &DaemonId::new(None),
                &HLC::default(),
                Uuid::new_v4(),
                vec![Uuid::new_v4()],
                vec![255; payload_len],
            );
            (result, rx.try_recv().ok().map(|frame| frame.message))
        };

        // JSON: 300 KB renders to well over 1 MiB of text ...
        let (result, sent) = send(false, 300 * 1024);
        assert!(matches!(result, Err(TrySendEventError::TooLarge { .. })));
        assert!(sent.is_none());
        // ... also when it only fails the exact check after rendering.
        let (result, sent) = send(false, MAX_DAEMON_TEXT_MESSAGE_BYTES / 2);
        assert!(matches!(result, Err(TrySendEventError::TooLarge { .. })));
        assert!(sent.is_none());
        // ... while the same payload goes out fine as a binary frame.
        let (result, sent) = send(true, 300 * 1024);
        assert!(result.is_ok());
        assert!(matches!(sent, Some(Message::Binary(_))));

        // Binary: the frame limit includes the header.
        let (result, sent) = send(true, MAX_TOPIC_DEBUG_FRAME_BYTES);
        assert!(matches!(result, Err(TrySendEventError::TooLarge { .. })));
        assert!(sent.is_none());
        // ... and a payload the last hop could not deliver is dropped on that
        // limit, which is the stricter of the two.
        let (result, sent) = send(true, MAX_TOPIC_DEBUG_PAYLOAD_BYTES + 1);
        assert!(matches!(
            result,
            Err(TrySendEventError::TooLarge { limit, .. }) if limit == MAX_TOPIC_DEBUG_PAYLOAD_BYTES
        ));
        assert!(sent.is_none());
        // The largest payload that does fit still goes out.
        let (result, sent) = send(true, MAX_TOPIC_DEBUG_PAYLOAD_BYTES);
        assert!(result.is_ok());
        assert!(matches!(sent, Some(Message::Binary(_))));
    }

    /// Regression test for the reply routing `resolve_machine` depends on
    /// (dora-rs/dora#3079): the coordinator answers a daemon->coordinator
    /// request inside the same `daemon_event` envelope as an inbound command,
    /// but with a params type the typed `CoordinatorCommandRaw` parse rejects.
    /// The reader must therefore route it to the pending caller by id, *before*
    /// that parse.
    ///
    /// Pinned because the routing block sits mid-loop in the reader and was
    /// dropped once already while restructuring this connection: without it
    /// every cross-machine `resolve_machine` falls through to the command
    /// parse, logs a parse failure, and times out returning `None`.
    ///
    /// The routing block itself is generic and ungated; this test reaches it
    /// through `resolve_machine`, its only caller, so it is gated the same way.
    /// CI runs the daemon's tests in both feature configurations, so the pin
    /// still holds on every PR.
    #[cfg(feature = "tensor-pool")]
    #[tokio::test]
    async fn ws_reader_routes_replies_to_pending_daemon_requests() {
        let clock = Arc::new(HLC::default());

        // Inbound frames are fed by the test, so the reply can echo the request
        // id `resolve_machine` picks at call time.
        let (ws_in_tx, ws_in_rx) = mpsc::channel::<Result<Message, std::io::Error>>(1);
        let (tx, mut rx) = mpsc::channel::<Timestamped<CoordinatorEvent>>(1);
        let (internal_tx, mut internal_rx) = mpsc::channel::<OutboundFrame>(1);
        let reader = tokio::spawn(run_coordinator_ws_reader(
            ReceiverStream::new(ws_in_rx),
            tx,
            internal_tx,
            clock.clone(),
        ));

        let (sender, mut outbound_rx) = CoordinatorSender::for_test();
        let resolve = tokio::spawn({
            let clock = clock.clone();
            async move { resolve_machine(&sender, &clock, "machine-a").await }
        });

        // Answer the request the daemon just sent, echoing its id.
        let request = outbound_rx
            .recv()
            .await
            .expect("resolve_machine must send a request");
        let request_id = serde_json::from_str::<serde_json::Value>(&request).unwrap()["id"]
            .as_str()
            .expect("the envelope carries the request id")
            .to_owned();
        let reply = format!(
            r#"{{"id":"{request_id}","method":"daemon_event","params":{{"inner":{{"ResolveMachineResult":{{"found":true,"address":"127.0.0.1:6021"}}}}}}}}"#
        );
        ws_in_tx
            .send(Ok(Message::Text(reply.into())))
            .await
            .unwrap();

        let resolved = tokio::time::timeout(Duration::from_secs(5), resolve)
            .await
            .expect(
                "the reply must reach the pending caller, not fall through to the command parse",
            )
            .unwrap();
        assert_eq!(resolved, Some("127.0.0.1:6021".parse().unwrap()));

        // A routed reply is consumed by the routing block: it must neither be
        // dispatched to the main loop nor answered on the wire.
        assert!(
            rx.try_recv().is_err(),
            "a routed reply must not reach the daemon main loop"
        );
        assert!(
            internal_rx.try_recv().is_err(),
            "a routed reply must not produce an outbound response frame"
        );

        drop(ws_in_tx);
        reader.await.unwrap();
    }
}
