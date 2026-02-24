use crate::{
    events::{DaemonRequest, DataflowEvent, Event},
    state::DaemonConnection,
};
use adora_core::uhlc::HLC;
use adora_message::{
    common::DaemonId,
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent, Timestamped},
    ws_protocol::WsResponse,
};
use axum::extract::ws::{Message, WebSocket};
use futures::{SinkExt, StreamExt};
use std::{collections::HashMap, sync::Arc};
use tokio::sync::{Mutex, mpsc, oneshot};
use uuid::Uuid;

/// Handle a single daemon WebSocket connection on `/api/daemon`.
///
/// Bidirectional: daemon sends events/responses to coordinator,
/// coordinator sends commands to daemon via the cmd channel.
pub(crate) async fn handle_daemon_ws(
    socket: WebSocket,
    event_tx: mpsc::Sender<Event>,
    clock: Arc<HLC>,
) {
    let (mut ws_tx, mut ws_rx) = socket.split();

    // Channel for coordinator -> daemon commands
    let (cmd_tx, mut cmd_rx) = mpsc::channel::<String>(64);
    let pending_replies: Arc<Mutex<HashMap<Uuid, oneshot::Sender<String>>>> =
        Arc::new(Mutex::new(HashMap::new()));

    // Track daemon_id from incoming events for cleanup on disconnect
    let mut tracked_daemon_id: Option<DaemonId> = None;

    loop {
        tokio::select! {
            // Incoming messages from daemon
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
                        tracing::trace!("WS daemon connection error: {e}");
                        break;
                    }
                };

                // Distinguish request vs response by checking for "method" key
                let value: serde_json::Value = match serde_json::from_str(&text) {
                    Ok(v) => v,
                    Err(e) => {
                        tracing::warn!("invalid JSON from daemon WS: {e}");
                        continue;
                    }
                };

                if value.get("method").is_some() {
                    // Daemon event (WsRequest containing CoordinatorRequest)
                    if !handle_daemon_request(
                        value,
                        &event_tx,
                        &clock,
                        &cmd_tx,
                        &pending_replies,
                        &mut tracked_daemon_id,
                    ).await {
                        break;
                    }
                } else {
                    // Response to coordinator command
                    handle_daemon_response(value, &pending_replies).await;
                }
            }
            // Outgoing commands from coordinator to daemon
            Some(cmd_json) = cmd_rx.recv() => {
                if ws_tx.send(Message::Text(cmd_json.into())).await.is_err() {
                    break;
                }
            }
        }
    }

    // Emit DaemonExit on WS close for immediate cleanup
    if let Some(daemon_id) = tracked_daemon_id {
        tracing::info!("daemon WS connection closed for `{daemon_id}`");
        let _ = event_tx.send(Event::DaemonExit { daemon_id }).await;
    }
}

/// Handle a daemon request (event or register). Returns false if the event channel closed.
async fn handle_daemon_request(
    value: serde_json::Value,
    event_tx: &mpsc::Sender<Event>,
    clock: &HLC,
    cmd_tx: &mpsc::Sender<String>,
    pending_replies: &Arc<Mutex<HashMap<Uuid, oneshot::Sender<String>>>>,
    tracked_daemon_id: &mut Option<DaemonId>,
) -> bool {
    let params = match value.get("params") {
        Some(p) => p.clone(),
        None => {
            tracing::warn!("daemon WS request missing params");
            return true;
        }
    };

    let message: Timestamped<CoordinatorRequest> = match serde_json::from_value(params) {
        Ok(m) => m,
        Err(e) => {
            tracing::warn!("failed to parse daemon request params: {e}");
            return true;
        }
    };

    if let Err(err) = clock.update_with_timestamp(&message.timestamp) {
        tracing::warn!("failed to update coordinator clock: {err}");
    }

    match message.inner {
        CoordinatorRequest::Register(register_request) => {
            let connection = DaemonConnection::new(cmd_tx.clone(), pending_replies.clone());
            let event = DaemonRequest::Register {
                connection,
                version_check_result: register_request.check_version(),
                machine_id: register_request.machine_id,
            };
            event_tx.send(Event::Daemon(event)).await.is_ok()
        }
        CoordinatorRequest::Event { daemon_id, event } => {
            // Track daemon_id for cleanup on disconnect
            if tracked_daemon_id.is_none() {
                *tracked_daemon_id = Some(daemon_id.clone());
            }
            if let Some(coordinator_event) = translate_daemon_event(daemon_id, event) {
                event_tx.send(coordinator_event).await.is_ok()
            } else {
                true
            }
        }
    }
}

fn translate_daemon_event(daemon_id: DaemonId, event: DaemonEvent) -> Option<Event> {
    match event {
        DaemonEvent::AllNodesReady {
            dataflow_id,
            exited_before_subscribe,
        } => Some(Event::Dataflow {
            uuid: dataflow_id,
            event: DataflowEvent::ReadyOnDaemon {
                daemon_id,
                exited_before_subscribe,
            },
        }),
        DaemonEvent::AllNodesFinished {
            dataflow_id,
            result,
        } => Some(Event::Dataflow {
            uuid: dataflow_id,
            event: DataflowEvent::DataflowFinishedOnDaemon { daemon_id, result },
        }),
        DaemonEvent::Heartbeat => Some(Event::DaemonHeartbeat { daemon_id }),
        DaemonEvent::Log(message) => Some(Event::Log(message)),
        DaemonEvent::Exit => Some(Event::DaemonExit { daemon_id }),
        DaemonEvent::NodeMetrics {
            dataflow_id,
            metrics,
        } => Some(Event::NodeMetrics {
            dataflow_id,
            metrics,
        }),
        DaemonEvent::BuildResult { build_id, result } => Some(Event::DataflowBuildResult {
            build_id,
            daemon_id,
            result: result.map_err(|err| eyre::eyre!(err)),
        }),
        DaemonEvent::SpawnResult {
            dataflow_id,
            result,
        } => Some(Event::DataflowSpawnResult {
            dataflow_id,
            daemon_id,
            result: result.map_err(|err| eyre::eyre!(err)),
        }),
    }
}

async fn handle_daemon_response(
    value: serde_json::Value,
    pending_replies: &Arc<Mutex<HashMap<Uuid, oneshot::Sender<String>>>>,
) {
    let response: WsResponse = match serde_json::from_value(value) {
        Ok(r) => r,
        Err(e) => {
            tracing::warn!("failed to parse daemon WS response: {e}");
            return;
        }
    };

    if let Some(sender) = pending_replies.lock().await.remove(&response.id) {
        let result_json = if let Some(val) = response.result {
            serde_json::to_string(&val).unwrap_or_default()
        } else if let Some(err) = response.error {
            // Error response - format as JSON that will fail DaemonCoordinatorReply deser
            format!("{{\"ws_error\":\"{}\"}}", err.replace('"', "\\\""))
        } else {
            "null".to_string()
        };
        let _ = sender.send(result_json);
    } else {
        tracing::warn!(
            "no pending reply for daemon WS response id {}",
            response.id
        );
    }
}
