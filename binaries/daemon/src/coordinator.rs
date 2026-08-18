use crate::DaemonCoordinatorEvent;
use dora_core::uhlc::HLC;
use dora_message::{
    common::{DaemonId, Timestamped},
    coordinator_to_daemon::RegisterResult,
    daemon_to_coordinator::{CoordinatorRequest, DaemonCoordinatorReply, DaemonRegisterRequest},
    ws_protocol::WsResponse,
};
use eyre::eyre;
use futures::{Sink, SinkExt, StreamExt};
use std::{net::SocketAddr, sync::Arc, time::Duration};
use tokio::sync::{mpsc, oneshot};
use tokio_stream::{Stream, wrappers::ReceiverStream};
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

const DAEMON_COORDINATOR_RETRY_INITIAL: Duration = Duration::from_secs(1);
const DAEMON_COORDINATOR_RETRY_MAX: Duration = Duration::from_secs(30);
/// Maximum number of consecutive failed connection attempts before giving up.
const DAEMON_COORDINATOR_RETRY_LIMIT: u32 = 50;
const REGISTER_TIMEOUT: Duration = Duration::from_secs(30);
/// Timeout for the cross-machine register flow: awaiting the ResolveMachine
/// reply here and the RegisterPoolAck in lib.rs.
pub const CROSS_REGISTER_TIMEOUT: Duration = Duration::from_secs(5);

#[derive(Debug)]
pub struct CoordinatorEvent {
    pub event: DaemonCoordinatorEvent,
    pub reply_tx: oneshot::Sender<Option<DaemonCoordinatorReply>>,
}

/// Wraps the WS send channel for fire-and-forget daemon events to the coordinator.
#[derive(Clone)]
pub struct CoordinatorSender {
    sender: mpsc::Sender<String>,
}

#[derive(Debug)]
pub enum TrySendEventError {
    InvalidUtf8(std::str::Utf8Error),
    Full,
    Closed,
}

impl std::fmt::Display for TrySendEventError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidUtf8(err) => write!(f, "event message not UTF-8: {err}"),
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

    pub fn try_send_event(&self, message: &[u8]) -> Result<(), TrySendEventError> {
        let json = Self::format_event_message(message)?;
        self.sender.try_send(json).map_err(|err| match err {
            mpsc::error::TrySendError::Full(_) => TrySendEventError::Full,
            mpsc::error::TrySendError::Closed(_) => TrySendEventError::Closed,
        })
    }

    /// Build a detached sender (and its receiver) for tests that only need a
    /// distinct, valid `CoordinatorSender` instance.
    #[cfg(test)]
    pub(crate) fn for_test() -> (Self, mpsc::Receiver<String>) {
        let (sender, rx) = mpsc::channel(8);
        (Self { sender }, rx)
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

pub async fn register(
    addr: SocketAddr,
    machine_id: Option<String>,
    labels: std::collections::BTreeMap<String, String>,
    clock: Arc<HLC>,
) -> eyre::Result<(
    DaemonId,
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

    let (mut ws_tx, mut ws_rx) = ws_stream.split();

    // Channel for outgoing messages (daemon events + command replies).
    // The coordinator sender writes to this, and the writer task below reads
    // and forwards to WS.
    let (send_tx, send_rx) = mpsc::channel::<String>(64);

    // Send Register request.
    // Serialize params via to_string (not to_value) to preserve u128 fidelity
    // for uhlc::ID(NonZeroU128) inside the timestamp.
    let register_params_json = serde_json::to_string(&Timestamped {
        inner: CoordinatorRequest::Register(DaemonRegisterRequest::new(machine_id, labels)),
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
    let daemon_id = tokio::time::timeout(REGISTER_TIMEOUT, async {
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

            break result.inner.to_result();
        }
    })
    .await
    .map_err(|_| eyre!("timeout waiting for register reply from coordinator"))??;

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

    tokio::spawn(run_coordinator_ws_writer(ws_tx, send_rx, internal_rx));

    let task_clock = clock.clone();
    tokio::spawn(run_coordinator_ws_reader(
        ws_rx,
        tx,
        internal_tx,
        task_clock,
    ));

    Ok((
        daemon_id,
        CoordinatorSender { sender: send_tx },
        ReceiverStream::new(rx),
    ))
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
async fn run_coordinator_ws_writer<Tx>(
    mut ws_tx: Tx,
    mut send_rx: mpsc::Receiver<String>,
    mut internal_rx: mpsc::Receiver<OutboundFrame>,
) where
    Tx: Sink<Message> + Unpin,
{
    loop {
        tokio::select! {
            frame = internal_rx.recv() => match frame {
                Some(OutboundFrame::Ws(msg)) => {
                    if ws_tx.send(msg).await.is_err() {
                        break;
                    }
                }
                Some(OutboundFrame::DestroyNotify(notify)) => {
                    let _ = notify.send(());
                    break;
                }
                // Reader stopped; all frames it queued (FIFO) are already
                // flushed, so close the write half by dropping `ws_tx`.
                None => break,
            },
            outgoing = send_rx.recv() => match outgoing {
                Some(text) => {
                    if ws_tx.send(Message::Text(text.into())).await.is_err() {
                        break;
                    }
                }
                // CoordinatorSender dropped: nothing more to send.
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

        let writer = tokio::spawn(run_coordinator_ws_writer(ws_out_tx, send_rx, internal_rx));
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
