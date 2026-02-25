use crate::DaemonCoordinatorEvent;
use adora_core::uhlc::HLC;
use adora_message::{
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

impl CoordinatorSender {
    /// Send a serialized event message to the coordinator (fire-and-forget).
    ///
    /// Embeds the raw JSON bytes directly to preserve u128 fidelity
    /// for uhlc::ID inside timestamps.
    pub async fn send_event(&self, message: &[u8]) -> eyre::Result<()> {
        let params_str =
            std::str::from_utf8(message).map_err(|e| eyre!("event message not UTF-8: {e}"))?;
        let id = Uuid::new_v4();
        let json =
            format!(r#"{{"id":"{id}","method":"daemon_event","params":{params_str}}}"#);
        self.sender
            .send(json)
            .await
            .map_err(|_| eyre!("WS send channel closed"))
    }
}

pub async fn register(
    addr: SocketAddr,
    machine_id: Option<String>,
    clock: Arc<HLC>,
) -> eyre::Result<(
    DaemonId,
    CoordinatorSender,
    impl Stream<Item = Timestamped<CoordinatorEvent>>,
)> {
    let ws_url = format!("ws://{addr}/api/daemon");
    let ws_stream = {
        let mut backoff = DAEMON_COORDINATOR_RETRY_INITIAL;
        loop {
            match tokio_tungstenite::connect_async(&ws_url).await {
                Ok((stream, _)) => break stream,
                Err(err) => {
                    // Add jitter: +/- 25% of backoff to prevent thundering herd
                    let jitter_range = backoff / 4;
                    let jitter = Duration::from_millis(
                        (rand_jitter_millis() % (jitter_range.as_millis() as u64 * 2 + 1))
                            .saturating_sub(jitter_range.as_millis() as u64),
                    );
                    let sleep_duration = backoff.saturating_add(jitter);
                    tracing::warn!(
                        "Could not connect to WS at {ws_url}: {err}. Retrying in {sleep_duration:#?}.."
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
        inner: CoordinatorRequest::Register(DaemonRegisterRequest::new(machine_id)),
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

    tracing::info!("Connected to adora-coordinator at ws://{addr}/api/daemon");

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
                            if let Ok(json) = serde_json::to_string(&response) {
                                if ws_tx.send(Message::Text(json.into())).await.is_err() {
                                    break;
                                }
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

/// Simple jitter: uses system time nanos as a cheap pseudo-random source.
fn rand_jitter_millis() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .subsec_nanos() as u64
}
