use crate::{Event, control::ControlEvent};
use adora_core::topics::zenoh_output_publish_topic;
use adora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::ControlRequestReply,
    ws_protocol::{WsRequest, WsResponse},
};
use axum::extract::ws::{Message, WebSocket};
use futures::{SinkExt, StreamExt};
use std::collections::HashMap;
use tokio::sync::{mpsc, oneshot};
use tokio::task::JoinHandle;
use uuid::Uuid;

/// Maximum topics allowed in a single TopicSubscribe request.
const MAX_TOPICS_PER_SUBSCRIBE: usize = 64;

/// Maximum concurrent topic subscriptions per WebSocket connection.
const MAX_SUBSCRIPTIONS_PER_CONNECTION: usize = 16;

/// Maximum binary payload size forwarded from Zenoh (64 MiB).
const MAX_BINARY_PAYLOAD_BYTES: usize = 64 * 1024 * 1024;

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

/// Format a `WsResponse`-shaped JSON envelope using `serde_json::to_string`.
///
/// This is needed (instead of `send_ws_response`) for replies that contain u128
/// values (e.g. uhlc::ID) which lose fidelity through `serde_json::Value`.
fn format_response_json(id: Uuid, reply: &impl serde::Serialize) -> String {
    match serde_json::to_string(reply) {
        Ok(result_json) => {
            format!(r#"{{"id":"{id}","result":{result_json}}}"#)
        }
        Err(e) => {
            let err_json = serde_json::to_string(&format!("{e}"))
                .unwrap_or_else(|_| "\"serialization error\"".to_string());
            format!(r#"{{"id":"{id}","error":{err_json}}}"#)
        }
    }
}

/// Lazily-initialized Zenoh session shared across topic subscriptions.
async fn get_or_init_zenoh(
    zenoh_session: &mut Option<zenoh::Session>,
) -> Result<zenoh::Session, String> {
    if let Some(session) = zenoh_session.as_ref() {
        return Ok(session.clone());
    }
    match adora_core::topics::open_zenoh_session(None).await {
        Ok(session) => {
            *zenoh_session = Some(session.clone());
            Ok(session)
        }
        Err(e) => Err(format!("failed to open zenoh session: {e}")),
    }
}

/// Handle a single CLI WebSocket connection on `/api/control`.
///
/// For normal requests: deserialize ControlRequest from WsRequest.params,
/// send ControlEvent to coordinator via `event_tx`, await oneshot reply, send WsResponse.
///
/// For LogSubscribe/BuildLogSubscribe: ack via WsResponse, then push WsEvent{event:"log"}
/// on the same connection.
///
/// For TopicSubscribe: validate via coordinator, open Zenoh subscribers, forward as binary
/// WS frames with `subscription_id ++ payload` format.
pub(crate) async fn handle_control_ws(socket: WebSocket, event_tx: mpsc::Sender<Event>) {
    let (mut ws_tx, mut ws_rx) = socket.split();
    // Channel for log events to push back on same WS connection
    let (log_tx, mut log_rx) = mpsc::channel::<String>(64);
    // Channel for binary topic data frames
    let (binary_tx, mut binary_rx) = mpsc::channel::<Vec<u8>>(64);
    // Active topic subscription tasks, keyed by subscription_id
    let mut topic_tasks: HashMap<Uuid, Vec<JoinHandle<()>>> = HashMap::new();
    // Lazily initialized Zenoh session
    let mut zenoh_session: Option<zenoh::Session> = None;

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

                // Handle LogSubscribe / BuildLogSubscribe / TopicSubscribe / TopicUnsubscribe specially
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
                    ControlRequest::TopicSubscribe { dataflow_id, topics } => {
                        // Validate topic count
                        if topics.len() > MAX_TOPICS_PER_SUBSCRIBE {
                            let resp = WsResponse::err(
                                req.id,
                                format!(
                                    "too many topics ({}, max {})",
                                    topics.len(),
                                    MAX_TOPICS_PER_SUBSCRIBE
                                ),
                            );
                            let _ = send_ws_response(&mut ws_tx, &resp).await;
                            continue;
                        }

                        // Validate subscription count
                        if topic_tasks.len() >= MAX_SUBSCRIPTIONS_PER_CONNECTION {
                            let resp = WsResponse::err(
                                req.id,
                                format!(
                                    "too many active subscriptions ({}, max {})",
                                    topic_tasks.len(),
                                    MAX_SUBSCRIPTIONS_PER_CONNECTION
                                ),
                            );
                            let _ = send_ws_response(&mut ws_tx, &resp).await;
                            continue;
                        }

                        let (found_tx, found_rx) = oneshot::channel();
                        let _ = event_tx.send(Event::Control(ControlEvent::TopicSubscribe {
                            dataflow_id: *dataflow_id,
                            topics: topics.clone(),
                            found_tx,
                        })).await;

                        let found = found_rx.await.unwrap_or(false);
                        if !found {
                            let resp = WsResponse::err(
                                req.id,
                                format!(
                                    "dataflow {dataflow_id} not found or publish_all_messages_to_zenoh not enabled"
                                ),
                            );
                            let _ = send_ws_response(&mut ws_tx, &resp).await;
                            continue;
                        }

                        // Open Zenoh session lazily
                        let session = match get_or_init_zenoh(&mut zenoh_session).await {
                            Ok(s) => s,
                            Err(e) => {
                                let resp = WsResponse::err(req.id, e);
                                let _ = send_ws_response(&mut ws_tx, &resp).await;
                                continue;
                            }
                        };

                        let subscription_id = Uuid::new_v4();
                        let mut handles = Vec::with_capacity(topics.len());

                        for (node_id, data_id) in topics {
                            let topic = zenoh_output_publish_topic(
                                *dataflow_id,
                                node_id,
                                data_id,
                            );
                            let session = session.clone();
                            let binary_tx = binary_tx.clone();
                            let sub_id_bytes = subscription_id.into_bytes();

                            handles.push(tokio::spawn(async move {
                                let subscriber = match session.declare_subscriber(&topic).await {
                                    Ok(s) => s,
                                    Err(e) => {
                                        tracing::warn!("failed to subscribe to zenoh topic {topic}: {e}");
                                        return;
                                    }
                                };

                                let mut dropped: u64 = 0;
                                while let Ok(sample) = subscriber.recv_async().await {
                                    let payload = sample.payload().to_bytes();
                                    if payload.len() > MAX_BINARY_PAYLOAD_BYTES {
                                        tracing::warn!(
                                            "dropping oversized payload ({} bytes) on {topic}",
                                            payload.len()
                                        );
                                        continue;
                                    }
                                    let mut frame = Vec::with_capacity(16 + payload.len());
                                    frame.extend_from_slice(&sub_id_bytes);
                                    frame.extend_from_slice(&payload);
                                    if binary_tx.try_send(frame).is_err() {
                                        dropped += 1;
                                        if dropped == 1 || dropped % 100 == 0 {
                                            tracing::warn!(
                                                "backpressure: dropped {dropped} frame(s) on {topic} (channel full)"
                                            );
                                        }
                                    }
                                }
                            }));
                        }

                        topic_tasks.insert(subscription_id, handles);

                        let reply = ControlRequestReply::TopicSubscribed { subscription_id };
                        let resp_json = format_response_json(req.id, &reply);
                        if ws_tx.send(Message::Text(resp_json.into())).await.is_err() {
                            break;
                        }
                        continue;
                    }
                    ControlRequest::TopicUnsubscribe { subscription_id } => {
                        if let Some(handles) = topic_tasks.remove(subscription_id) {
                            for h in handles {
                                h.abort();
                            }
                        }
                        let resp = WsResponse::ok(
                            req.id,
                            serde_json::json!({"unsubscribed": true, "subscription_id": subscription_id}),
                        );
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
                    Ok(Err(err)) => {
                        tracing::error!("control request failed: {err:?}");
                        ControlRequestReply::Error(format!("{err}"))
                    }
                    Err(_) => ControlRequestReply::Error("coordinator dropped reply".to_string()),
                };

                let stop = matches!(reply, ControlRequestReply::CoordinatorStopped);

                // Serialize reply via to_string (not to_value) to preserve u128
                // fidelity for uhlc::ID inside DataflowResult timestamps.
                let resp_json = format_response_json(req.id, &reply);
                if ws_tx.send(Message::Text(resp_json.into())).await.is_err() || stop {
                    break;
                }
            }
            // Log events to push to CLI
            Some(log_json) = log_rx.recv() => {
                if ws_tx.send(Message::Text(log_json.into())).await.is_err() {
                    break;
                }
            }
            // Binary topic data to push to CLI
            Some(data) = binary_rx.recv() => {
                if ws_tx.send(Message::Binary(data.into())).await.is_err() {
                    break;
                }
            }
        }
    }

    // Clean up: abort all topic subscription tasks
    for (_, handles) in topic_tasks {
        for h in handles {
            h.abort();
        }
    }
}
