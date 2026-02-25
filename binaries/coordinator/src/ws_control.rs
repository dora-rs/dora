use crate::{Event, control::ControlEvent};
use adora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
    ws_protocol::{WsRequest, WsResponse},
};
use axum::extract::ws::{Message, WebSocket};
use eyre::Context;
use futures::{SinkExt, StreamExt};
use tokio::sync::{mpsc, oneshot};
use uuid::Uuid;

/// Serialize a `WsResponse` and send it over the WS connection.
/// Returns `Err` if the WS send fails (connection closed).
async fn send_ws_response(
    ws_tx: &mut futures::stream::SplitSink<WebSocket, Message>,
    resp: &WsResponse,
) -> Result<(), ()> {
    let json = match serde_json::to_string(resp) {
        Ok(s) => s,
        Err(e) => {
            tracing::error!("failed to serialize WsResponse: {e}");
            return Err(());
        }
    };
    ws_tx.send(Message::Text(json.into())).await.map_err(|_| ())
}

/// Handle a single CLI WebSocket connection on `/api/control`.
///
/// For normal requests: deserialize ControlRequest from WsRequest.params,
/// send ControlEvent to coordinator via `event_tx`, await oneshot reply, send WsResponse.
///
/// For LogSubscribe/BuildLogSubscribe: ack via WsResponse, then push WsEvent{event:"log"}
/// on the same connection.
pub(crate) async fn handle_control_ws(
    socket: WebSocket,
    event_tx: mpsc::Sender<Event>,
) {
    let (mut ws_tx, mut ws_rx) = socket.split();
    // Channel for log events to push back on same WS connection
    let (log_tx, mut log_rx) = mpsc::channel::<String>(64);

    loop {
        tokio::select! {
            // Incoming WS messages from CLI
            msg = ws_rx.next() => {
                let Some(msg) = msg else { break };
                let msg = match msg {
                    Ok(Message::Text(text)) => text,
                    Ok(Message::Close(_)) => break,
                    Ok(Message::Ping(data)) => {
                        let _ = ws_tx.send(Message::Pong(data)).await;
                        continue;
                    }
                    Ok(_) => continue,
                    Err(e) => {
                        tracing::trace!("WS control connection error: {e}");
                        break;
                    }
                };

                let req: WsRequest = match serde_json::from_str(&msg) {
                    Ok(r) => r,
                    Err(e) => {
                        let resp = WsResponse::err(Uuid::nil(), format!("invalid request: {e}"));
                        let _ = send_ws_response(&mut ws_tx, &resp).await;
                        continue;
                    }
                };

                let control_request: ControlRequest = match serde_json::from_value(req.params.clone()) {
                    Ok(r) => r,
                    Err(e) => {
                        let resp = WsResponse::err(req.id, format!("invalid params: {e}"));
                        let _ = send_ws_response(&mut ws_tx, &resp).await;
                        continue;
                    }
                };

                // Handle LogSubscribe / BuildLogSubscribe specially
                match &control_request {
                    ControlRequest::LogSubscribe { dataflow_id, level } => {
                        let (found_tx, found_rx) = oneshot::channel();
                        let _ = event_tx.send(Event::Control(ControlEvent::LogSubscribe {
                            dataflow_id: *dataflow_id,
                            level: *level,
                            sender: log_tx.clone(),
                            found_tx,
                        })).await;

                        let found = found_rx.await.unwrap_or(false);
                        let resp = if found {
                            WsResponse::ok(req.id, serde_json::json!({"subscribed": true}))
                        } else {
                            WsResponse::err(req.id, format!("no running dataflow with id {dataflow_id}"))
                        };
                        let _ = send_ws_response(&mut ws_tx, &resp).await;
                        continue;
                    }
                    ControlRequest::BuildLogSubscribe { build_id, level } => {
                        let (found_tx, found_rx) = oneshot::channel();
                        let _ = event_tx.send(Event::Control(ControlEvent::BuildLogSubscribe {
                            build_id: *build_id,
                            level: *level,
                            sender: log_tx.clone(),
                            found_tx,
                        })).await;

                        let found = found_rx.await.unwrap_or(false);
                        let resp = if found {
                            WsResponse::ok(req.id, serde_json::json!({"subscribed": true}))
                        } else {
                            WsResponse::err(req.id, format!("no running build with id {build_id}"))
                        };
                        let _ = send_ws_response(&mut ws_tx, &resp).await;
                        continue;
                    }
                    _ => {}
                }

                // Normal request-reply
                let (reply_tx, reply_rx) = oneshot::channel();
                let event = ControlEvent::IncomingRequest {
                    request: control_request,
                    reply_sender: reply_tx,
                };

                if event_tx.send(Event::Control(event)).await.is_err() {
                    let resp = WsResponse::err(req.id, "coordinator stopped".to_string());
                    let _ = send_ws_response(&mut ws_tx, &resp).await;
                    break;
                }

                let reply = match reply_rx.await {
                    Ok(Ok(reply)) => reply,
                    Ok(Err(err)) => ControlRequestReply::Error(format!("{err:?}")),
                    Err(_) => ControlRequestReply::Error("coordinator dropped reply".to_string()),
                };

                let stop = matches!(reply, ControlRequestReply::CoordinatorStopped);

                let resp = match serde_json::to_value(&reply)
                    .context("failed to serialize reply")
                {
                    Ok(val) => WsResponse::ok(req.id, val),
                    Err(e) => WsResponse::err(req.id, format!("{e}")),
                };
                if send_ws_response(&mut ws_tx, &resp).await.is_err() || stop {
                    break;
                }
            }
            // Log events to push to CLI
            Some(log_json) = log_rx.recv() => {
                if ws_tx.send(Message::Text(log_json.into())).await.is_err() {
                    break;
                }
            }
        }
    }
}
