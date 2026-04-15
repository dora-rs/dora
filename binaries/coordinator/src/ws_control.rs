use crate::{Event, control::ControlEvent};
use axum::extract::ws::{Message, WebSocket};
use dora_message::{
    cli_to_coordinator::{ControlRequest, check_cli_version},
    common::Timestamped,
    coordinator_to_cli::ControlRequestReply,
    current_crate_version,
    daemon_to_daemon::InterDaemonEvent,
    metadata::{ArrowTypeInfo, BufferOffset, Metadata},
    ws_protocol::{WsRequest, WsResponse},
};
use futures::{SinkExt, StreamExt};
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot};
use uuid::Uuid;

/// Maximum topics allowed in a single TopicSubscribe request.
const MAX_TOPICS_PER_SUBSCRIBE: usize = 64;

/// Maximum concurrent topic subscriptions per WebSocket connection.
const MAX_SUBSCRIPTIONS_PER_CONNECTION: usize = 16;

#[derive(Clone)]
struct ActiveTopicSubscription {
    subscription_id: Uuid,
    dataflow_id: Uuid,
    topics: Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
}

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

/// Handle a single CLI WebSocket connection on `/api/control`.
///
/// For normal requests: deserialize ControlRequest from WsRequest.params,
/// send ControlEvent to coordinator via `event_tx`, await oneshot reply, send WsResponse.
///
/// For LogSubscribe/BuildLogSubscribe: ack via WsResponse, then push WsEvent{event:"log"}
/// on the same connection.
///
/// For TopicSubscribe: register a daemon-backed debug stream via coordinator
/// and forward binary frames with `subscription_id ++ payload` format.
pub(crate) async fn handle_control_ws(
    socket: WebSocket,
    event_tx: mpsc::Sender<Event>,
    clock: Arc<dora_core::uhlc::HLC>,
) {
    let (mut ws_tx, mut ws_rx) = socket.split();
    // Channel for log events to push back on same WS connection
    let (log_tx, mut log_rx) = mpsc::channel::<String>(64);
    // Channel for binary topic data frames
    let (binary_tx, mut binary_rx) = mpsc::channel::<crate::topic_subscriber::TopicFrame>(64);
    let mut topic_subscriptions: Vec<ActiveTopicSubscription> = Vec::new();
    let mut publish_session: Option<zenoh::Session> = None;

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

                // Handle Hello / LogSubscribe / BuildLogSubscribe / TopicSubscribe / TopicUnsubscribe specially
                match &control_request {
                    ControlRequest::Hello { dora_version } => {
                        // Protocol version handshake — reply directly from
                        // this loop rather than forwarding to the event
                        // loop, so mismatched CLIs fail fast before any
                        // stateful interaction (dora-rs/adora#151).
                        let resp = match check_cli_version(dora_version) {
                            Ok(()) => {
                                let reply = ControlRequestReply::HelloOk {
                                    dora_version: current_crate_version(),
                                };
                                match serde_json::to_value(&reply) {
                                    Ok(val) => WsResponse::ok(req.id, val),
                                    Err(e) => WsResponse::err(
                                        req.id,
                                        format!("failed to serialize HelloOk: {e}"),
                                    ),
                                }
                            }
                            Err(msg) => {
                                tracing::warn!("rejecting CLI with {msg}");
                                WsResponse::err(req.id, msg)
                            }
                        };
                        let _ = send_ws_response(&mut ws_tx, &resp).await;
                        continue;
                    }
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
                        let mut normalized_topics = topics.clone();
                        normalized_topics.sort();

                        if let Some(existing) = topic_subscriptions.iter().find(|subscription| {
                            subscription.dataflow_id == *dataflow_id
                                && subscription.topics == normalized_topics
                        }) {
                            let reply = ControlRequestReply::TopicSubscribed {
                                subscription_id: existing.subscription_id,
                            };
                            let resp_json = format_response_json(req.id, &reply);
                            if ws_tx.send(Message::Text(resp_json.into())).await.is_err() {
                                break;
                            }
                            continue;
                        }

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
                        if topic_subscriptions.len() >= MAX_SUBSCRIPTIONS_PER_CONNECTION {
                            let resp = WsResponse::err(
                                req.id,
                                format!(
                                    "too many active subscriptions ({}, max {})",
                                    topic_subscriptions.len(),
                                    MAX_SUBSCRIPTIONS_PER_CONNECTION
                                ),
                            );
                            let _ = send_ws_response(&mut ws_tx, &resp).await;
                            continue;
                        }

                        let (done_tx, done_rx) = oneshot::channel();
                        let _ = event_tx.send(Event::Control(ControlEvent::TopicSubscribe {
                            dataflow_id: *dataflow_id,
                            topics: topics.clone(),
                            sender: binary_tx.clone(),
                            done_tx,
                        })).await;

                        let subscription_id = match done_rx.await {
                            Ok(Ok(subscription_id)) => {
                                topic_subscriptions.push(ActiveTopicSubscription {
                                    subscription_id,
                                    dataflow_id: *dataflow_id,
                                    topics: normalized_topics,
                                });
                                subscription_id
                            }
                            Ok(Err(err)) => {
                                let resp = WsResponse::err(req.id, err);
                                let _ = send_ws_response(&mut ws_tx, &resp).await;
                                continue;
                            }
                            Err(_) => {
                                let resp = WsResponse::err(
                                    req.id,
                                    "topic subscribe request dropped before completion".to_string(),
                                );
                                let _ = send_ws_response(&mut ws_tx, &resp).await;
                                continue;
                            }
                        };

                        let reply = ControlRequestReply::TopicSubscribed { subscription_id };
                        let resp_json = format_response_json(req.id, &reply);
                        if ws_tx.send(Message::Text(resp_json.into())).await.is_err() {
                            break;
                        }
                        continue;
                    }
                    ControlRequest::TopicUnsubscribe { subscription_id } => {
                        topic_subscriptions
                            .retain(|active| active.subscription_id != *subscription_id);
                        let (done_tx, done_rx) = oneshot::channel();
                        let _ = event_tx
                            .send(Event::Control(ControlEvent::TopicUnsubscribe {
                                subscription_id: *subscription_id,
                                done_tx,
                            }))
                            .await;
                        let _ = done_rx.await;
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
                        let _ = event_tx.send(Event::Control(ControlEvent::TopicCheck {
                            dataflow_id: *dataflow_id,
                            topics: vec![(node_id.clone(), output_id.clone())],
                            found_tx,
                        })).await;
                        let found = found_rx.await.unwrap_or(false);
                        if !found {
                            let resp = WsResponse::err(
                                req.id,
                                format!(
                                    "dataflow {dataflow_id} not found, output unavailable, or topic publish requires `_unstable_debug.publish_all_messages_to_zenoh: true`"
                                ),
                            );
                            let _ = send_ws_response(&mut ws_tx, &resp).await;
                            continue;
                        }

                        let resp = match publish_topic(
                            *dataflow_id,
                            node_id,
                            output_id,
                            data_json,
                            &clock,
                            &mut publish_session,
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
                    request: Box::new(control_request),
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
            Some(frame) = binary_rx.recv() => {
                let mut data = Vec::with_capacity(16 + frame.payload.len());
                data.extend_from_slice(&frame.subscription_id.into_bytes());
                data.extend_from_slice(&frame.payload);
                if ws_tx.send(Message::Binary(data.into())).await.is_err() {
                    break;
                }
            }
        }
    }

    for subscription_id in topic_subscriptions
        .into_iter()
        .map(|active| active.subscription_id)
    {
        let (done_tx, done_rx) = oneshot::channel();
        let _ = event_tx
            .send(Event::Control(ControlEvent::TopicUnsubscribe {
                subscription_id,
                done_tx,
            }))
            .await;
        let _ = done_rx.await;
    }
}

/// Publish JSON data to a Zenoh topic as a serialized `InterDaemonEvent::Output`.
///
/// The JSON string is stored as raw UTF-8 bytes in a UInt8 Arrow array.
async fn publish_topic(
    dataflow_id: Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
    data_json: &str,
    clock: &dora_core::uhlc::HLC,
    session: &mut Option<zenoh::Session>,
) -> Result<(), String> {
    let topic = dora_core::topics::zenoh_output_publish_topic(dataflow_id, node_id, output_id);

    // Store JSON as raw UTF-8 bytes in a UInt8 array
    let data_bytes = data_json.as_bytes();
    let data = dora_message::aligned_vec::AVec::from_slice(128, data_bytes);

    let type_info = ArrowTypeInfo {
        data_type: dora_message::arrow_schema::DataType::UInt8,
        len: data_bytes.len(),
        null_count: 0,
        validity: None,
        offset: 0,
        buffer_offsets: vec![BufferOffset {
            offset: 0,
            len: data_bytes.len(),
        }],
        child_data: vec![],
        field_names: None,
        schema_hash: None,
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

    if session.is_none() {
        *session = Some(
            dora_core::topics::open_zenoh_session(None)
                .await
                .map_err(|e| format!("failed to open zenoh session: {e}"))?,
        );
    }

    session
        .as_mut()
        .expect("zenoh publish session initialized")
        .put(&topic, payload)
        .await
        .map_err(|e| format!("failed to publish to zenoh: {e}"))?;

    Ok(())
}
