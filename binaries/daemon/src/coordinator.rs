use crate::DaemonCoordinatorEvent;
use dora_core::uhlc::HLC;
use dora_message::{
    common::{DaemonId, Timestamped},
    coordinator_to_daemon::RegisterResult,
    daemon_to_coordinator::{CoordinatorRequest, DaemonCoordinatorReply, DaemonRegisterRequest},
    ws_protocol::WsResponse,
};
use eyre::eyre;
use futures::{SinkExt, StreamExt};
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
    // The coordinator sender writes to this, and the spawned task reads and forwards to WS.
    let (send_tx, mut send_rx) = mpsc::channel::<String>(64);

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

    // Spawned task: bidirectional WS message routing.
    // - Reads coordinator commands from WS, sends to event channel, awaits reply, sends reply back.
    // - Reads outgoing events from send_rx, forwards to WS.
    let task_clock = clock.clone();
    tokio::spawn(async move {
        let clock = task_clock;
        loop {
            tokio::select! {
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
                        Err(e) => {
                            tracing::warn!("WS coordinator connection error: {e}");
                            break;
                        }
                    };

                    // Parse directly from raw text to preserve u128 fidelity
                    // for uhlc::ID inside timestamps.
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
                                && ws_tx.send(Message::Text(json.into())).await.is_err() {
                                    break;
                                }
                        }
                        if let DaemonCoordinatorReply::DestroyResult { notify, .. } = reply {
                            if let Some(notify) = notify {
                                let _ = notify.send(());
                            }
                            break;
                        }
                    }
                }
                Some(outgoing) = send_rx.recv() => {
                    if ws_tx.send(Message::Text(outgoing.into())).await.is_err() {
                        break;
                    }
                }
            }
        }
    });

    Ok((
        daemon_id,
        CoordinatorSender { sender: send_tx },
        ReceiverStream::new(rx),
    ))
}

/// Helper for deserializing register reply directly from raw JSON text,
/// bypassing `serde_json::Value` to preserve u128 fidelity for uhlc::ID.
#[derive(serde::Deserialize)]
struct RegisterReplyRaw {
    params: Timestamped<RegisterResult>,
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
            jittered_backoff(backoff, (range_ms * 2)),
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
}
