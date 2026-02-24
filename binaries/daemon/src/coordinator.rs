use crate::DaemonCoordinatorEvent;
use adora_core::uhlc::HLC;
use adora_message::{
    common::{DaemonId, Timestamped},
    coordinator_to_daemon::RegisterResult,
    daemon_to_coordinator::{CoordinatorRequest, DaemonCoordinatorReply, DaemonRegisterRequest},
    ws_protocol::{WsRequest, WsResponse},
};
use eyre::eyre;
use futures::{SinkExt, StreamExt};
use std::{net::SocketAddr, time::Duration};
use tokio::sync::{mpsc, oneshot};
use tokio_stream::{Stream, wrappers::ReceiverStream};
use tokio_tungstenite::tungstenite::Message;
use uuid::Uuid;

const DAEMON_COORDINATOR_RETRY_INITIAL: Duration = Duration::from_secs(1);
const DAEMON_COORDINATOR_RETRY_MAX: Duration = Duration::from_secs(30);

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
    pub async fn send_event(&self, message: &[u8]) -> eyre::Result<()> {
        let params: serde_json::Value =
            serde_json::from_slice(message).map_err(|e| eyre!("failed to parse event: {e}"))?;
        let request = WsRequest {
            id: Uuid::new_v4(),
            method: "daemon_event".to_string(),
            params,
        };
        self.sender
            .send(serde_json::to_string(&request)?)
            .await
            .map_err(|_| eyre!("WS send channel closed"))
    }
}

pub async fn register(
    addr: SocketAddr,
    machine_id: Option<String>,
    clock: &HLC,
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
                    tracing::warn!(
                        "Could not connect to WS at {ws_url}: {err}. Retrying in {backoff:#?}.."
                    );
                    tokio::time::sleep(backoff).await;
                    backoff = (backoff * 2).min(DAEMON_COORDINATOR_RETRY_MAX);
                }
            }
        }
    };

    let (mut ws_tx, mut ws_rx) = ws_stream.split();

    // Channel for outgoing messages (daemon events + command replies).
    // The coordinator sender writes to this, and the spawned task reads and forwards to WS.
    let (send_tx, mut send_rx) = mpsc::channel::<String>(64);

    // Send Register request
    let register_params = serde_json::to_value(&Timestamped {
        inner: CoordinatorRequest::Register(DaemonRegisterRequest::new(machine_id)),
        timestamp: clock.new_timestamp(),
    })?;
    let register_req = WsRequest {
        id: Uuid::new_v4(),
        method: "daemon_event".to_string(),
        params: register_params,
    };
    ws_tx
        .send(Message::Text(serde_json::to_string(&register_req)?.into()))
        .await
        .map_err(|e| eyre!("failed to send register request: {e}"))?;

    // Wait for register reply.
    // The coordinator's register handler sends back Timestamped<RegisterResult>
    // wrapped in a WsRequest with method "daemon_event".
    let daemon_id = loop {
        let msg = ws_rx
            .next()
            .await
            .ok_or_else(|| eyre!("WS connection closed before register reply"))?
            .map_err(|e| eyre!("WS error during register: {e}"))?;

        let Message::Text(text) = msg else {
            continue;
        };

        let req: WsRequest = match serde_json::from_str(&text) {
            Ok(r) => r,
            Err(_) => continue,
        };

        let result: Timestamped<RegisterResult> = match serde_json::from_value(req.params) {
            Ok(r) => r,
            Err(_) => continue,
        };

        if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
            tracing::warn!("failed to update timestamp after register: {err}");
        }

        break result.inner.to_result()?;
    };

    tracing::info!("Connected to adora-coordinator at ws://{addr}/api/daemon");

    let (tx, rx) = mpsc::channel(1);

    // Spawned task: bidirectional WS message routing.
    // - Reads coordinator commands from WS, sends to event channel, awaits reply, sends reply back.
    // - Reads outgoing events from send_rx, forwards to WS.
    tokio::spawn(async move {
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

                    let req: WsRequest = match serde_json::from_str(&text) {
                        Ok(r) => r,
                        Err(e) => {
                            tracing::warn!("failed to parse coordinator WS message: {e}");
                            continue;
                        }
                    };

                    let request_id = req.id;
                    let needs_reply = req.method == "daemon_command";

                    let event: Timestamped<DaemonCoordinatorEvent> =
                        match serde_json::from_value(req.params) {
                            Ok(e) => e,
                            Err(e) => {
                                tracing::warn!("failed to parse coordinator event: {e}");
                                continue;
                            }
                        };

                    if let Err(err) = clock_update_noop(&event.timestamp) {
                        tracing::warn!("timestamp note: {err}");
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

    Ok((daemon_id, CoordinatorSender { sender: send_tx }, ReceiverStream::new(rx)))
}

/// Placeholder: the spawned task doesn't have access to the HLC, so we skip clock updates.
/// The main daemon loop updates the clock when processing events.
fn clock_update_noop(_timestamp: &adora_core::uhlc::Timestamp) -> Result<(), &'static str> {
    // Clock update happens in the daemon's main event processing loop
    Ok(())
}
