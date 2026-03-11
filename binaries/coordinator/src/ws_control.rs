use crate::{Event, control::ControlEvent};
use adora_core::topics::zenoh_output_publish_topic;
use adora_message::{
    cli_to_coordinator::ControlRequest,
    common::Timestamped,
    coordinator_to_cli::ControlRequestReply,
    daemon_to_daemon::InterDaemonEvent,
    metadata::{ArrowTypeInfo, BufferOffset, Metadata},
    ws_protocol::{WsRequest, WsResponse},
};
use axum::extract::ws::{Message, WebSocket};
use futures::{SinkExt, StreamExt};
use std::collections::HashMap;
use std::sync::Arc;
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
pub(crate) async fn handle_control_ws(
    socket: WebSocket,
    event_tx: mpsc::Sender<Event>,
    clock: Arc<adora_core::uhlc::HLC>,
) {
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
                    ControlRequest::TopicPublish {
                        dataflow_id,
                        node_id,
                        output_id,
                        data_json,
                    } => {
                        // Validate that the dataflow has debug publishing enabled
                        let (found_tx, found_rx) = oneshot::channel();
                        let _ = event_tx.send(Event::Control(ControlEvent::TopicSubscribe {
                            dataflow_id: *dataflow_id,
                            topics: vec![(node_id.clone(), output_id.clone())],
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

                        let resp = match publish_topic(
                            &mut zenoh_session,
                            *dataflow_id,
                            node_id,
                            output_id,
                            data_json,
                            &clock,
                        )
                        .await
                        {
                            Ok(()) => {
                                let reply = ControlRequestReply::TopicPublished;
                                format_response_json(req.id, &reply)
                            }
                            Err(e) => {
                                let reply = ControlRequestReply::Error(e);
                                format_response_json(req.id, &reply)
                            }
                        };
                        if ws_tx.send(Message::Text(resp.into())).await.is_err() {
                            break;
                        }
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
                        // Send only the root error message to the client, not the
                        // full internal chain (which may leak implementation details).
                        let root = err.root_cause().to_string();
                        ControlRequestReply::Error(root)
                    }
                    Err(_) => ControlRequestReply::Error(
                        "coordinator dropped the request without a reply \
                         (it may have shut down or the dataflow exited unexpectedly)"
                            .to_string(),
                    ),
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

/// Publish JSON data to a Zenoh topic as a serialized `InterDaemonEvent::Output`.
///
/// The JSON string is stored as raw UTF-8 bytes in a UInt8 Arrow array.
async fn publish_topic(
    zenoh_session: &mut Option<zenoh::Session>,
    dataflow_id: Uuid,
    node_id: &adora_message::id::NodeId,
    output_id: &adora_message::id::DataId,
    data_json: &str,
    clock: &adora_core::uhlc::HLC,
) -> Result<(), String> {
    let session = get_or_init_zenoh(zenoh_session).await?;
    let topic = zenoh_output_publish_topic(dataflow_id, node_id, output_id);

    // Store JSON as raw UTF-8 bytes in a UInt8 array
    let data_bytes = data_json.as_bytes();
    let data = adora_message::aligned_vec::AVec::from_slice(128, data_bytes);

    let type_info = ArrowTypeInfo {
        data_type: adora_message::arrow_schema::DataType::UInt8,
        len: data_bytes.len(),
        null_count: 0,
        validity: None,
        offset: 0,
        buffer_offsets: vec![BufferOffset {
            offset: 0,
            len: data_bytes.len(),
        }],
        child_data: vec![],
    };

    let timestamp = clock.new_timestamp();
    let metadata = Metadata::new(timestamp, type_info);

    let event = Timestamped {
        inner: InterDaemonEvent::Output {
            dataflow_id,
            node_id: node_id.clone(),
            output_id: output_id.clone(),
            metadata,
            data: Some(data),
        },
        timestamp,
    };

    let payload = event
        .serialize()
        .map_err(|e| format!("failed to serialize event: {e}"))?;

    session
        .put(&topic, payload)
        .await
        .map_err(|e| format!("failed to publish to zenoh: {e}"))?;

    Ok(())
}
