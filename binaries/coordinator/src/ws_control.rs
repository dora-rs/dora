use crate::{Event, control::ControlEvent};
use axum::extract::ws::{Message, WebSocket};
use dora_message::{
    TOPIC_DATA_PROTOCOL_VERSION,
    cli_to_coordinator::{ControlRequest, check_cli_version},
    common::Timestamped,
    coordinator_to_cli::ControlRequestReply,
    current_crate_version,
    daemon_to_daemon::InterDaemonEvent,
    metadata::{FRAMING, FRAMING_ARROW_IPC, Metadata, MetadataParameters, Parameter},
    ws_protocol::{WsRequest, WsResponse},
};
use futures::{SinkExt, StreamExt};
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot};
use uuid::Uuid;

/// Maximum topics allowed in a single TopicSubscribe request.
const MAX_TOPICS_PER_SUBSCRIBE: usize = 64;

/// Whether a client's advertised binary-frame encoding is one we can serve.
///
/// `Err` carries the rejection message. Split out from the request handler so
/// the decision is testable without a live WebSocket — the handler itself only
/// runs inside `handle_control_ws`.
fn check_topic_protocol(client_version: Option<u16>) -> Result<(), String> {
    if client_version == Some(TOPIC_DATA_PROTOCOL_VERSION) {
        Ok(())
    } else {
        Err(dora_message::topic_protocol_mismatch_message(
            "client",
            client_version,
        ))
    }
}

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

/// Send the standard reply for a log-subscribe request: `{"subscribed": true}`
/// when the target existed, or an error carrying `not_found_msg` otherwise.
/// Shared by the `LogSubscribe` and `BuildLogSubscribe` handlers, which differ
/// only in the event they emit and this message.
async fn send_subscribe_reply(
    ws_tx: &mut futures::stream::SplitSink<WebSocket, Message>,
    req_id: Uuid,
    found: bool,
    not_found_msg: impl FnOnce() -> String,
) -> Result<(), ()> {
    let resp = if found {
        WsResponse::ok(req_id, serde_json::json!({"subscribed": true}))
    } else {
        WsResponse::err(req_id, not_found_msg())
    };
    send_ws_response(ws_tx, &resp).await
}

/// Format a `WsResponse`-shaped JSON envelope using `serde_json::to_string`.
///
/// Used instead of `send_ws_response` where the reply is already a concrete
/// type: it serializes straight into the envelope rather than materializing an
/// intermediate `serde_json::Value` for the `WsResponse::result` field.
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

/// Push one log line to the CLI.
async fn send_log_line(
    ws_tx: &mut futures::stream::SplitSink<WebSocket, Message>,
    log_json: String,
) -> Result<(), ()> {
    ws_tx
        .send(Message::Text(log_json.into()))
        .await
        .map_err(|_| ())
}

/// Push one topic data frame to the CLI as `subscription_id ++ payload`.
async fn send_topic_frame(
    ws_tx: &mut futures::stream::SplitSink<WebSocket, Message>,
    frame: crate::topic_subscriber::TopicFrame,
) -> Result<(), ()> {
    let mut data = Vec::with_capacity(16 + frame.payload.len());
    data.extend_from_slice(&frame.subscription_id.into_bytes());
    data.extend_from_slice(&frame.payload);
    ws_tx
        .send(Message::Binary(data.into()))
        .await
        .map_err(|_| ())
}

/// Wait for a control request's reply while still draining the connection's
/// pushed log lines and topic frames to the CLI.
///
/// The coordinator event loop fills `log_rx`/`binary_rx` (64 slots each) and
/// only this connection empties them. Awaiting the reply alone would leave
/// them undrained for as long as the request takes, which for `WaitForBuild`
/// or `Stop` is the whole build or shutdown: once full, every further log line
/// costs the event loop a 100 ms send timeout and the subscriber is dropped
/// after 100 of them, losing the rest of the log.
///
/// `Err` means the WebSocket send failed (the connection is gone).
async fn await_reply_forwarding_pushes(
    mut reply_rx: oneshot::Receiver<eyre::Result<ControlRequestReply>>,
    ws_tx: &mut futures::stream::SplitSink<WebSocket, Message>,
    log_rx: &mut mpsc::Receiver<String>,
    binary_rx: &mut mpsc::Receiver<crate::topic_subscriber::TopicFrame>,
) -> Result<ControlRequestReply, ()> {
    let reply = loop {
        tokio::select! {
            reply = &mut reply_rx => break reply,
            Some(log_json) = log_rx.recv() => send_log_line(ws_tx, log_json).await?,
            Some(frame) = binary_rx.recv() => send_topic_frame(ws_tx, frame).await?,
        }
    };
    // Lines the event loop queued before answering belong ahead of the
    // reply: a CLI that exits on it would lose them.
    while let Ok(log_json) = log_rx.try_recv() {
        send_log_line(ws_tx, log_json).await?;
    }
    while let Ok(frame) = binary_rx.try_recv() {
        send_topic_frame(ws_tx, frame).await?;
    }
    Ok(match reply {
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
    })
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
                        let _ = send_subscribe_reply(
                            &mut ws_tx,
                            req.id,
                            found,
                            || format!("no running dataflow with id {dataflow_id}"),
                        )
                        .await;
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
                        let _ = send_subscribe_reply(
                            &mut ws_tx,
                            req.id,
                            found,
                            || format!("no running build with id {build_id}"),
                        )
                        .await;
                        continue;
                    }
                    ControlRequest::TopicSubscribe {
                        dataflow_id,
                        topics,
                        protocol_version,
                        ..
                    } => {
                        // Reject before doing any work: the frames this
                        // subscription would produce are positionally encoded,
                        // so a client on the other encoding misparses them
                        // instead of erroring (dora-rs/dora#3153).
                        if let Err(msg) = check_topic_protocol(*protocol_version) {
                            let resp = WsResponse::err(req.id, msg);
                            let _ = send_ws_response(&mut ws_tx, &resp).await;
                            continue;
                        }

                        let mut normalized_topics = topics.clone();
                        normalized_topics.sort();

                        if let Some(existing) = topic_subscriptions.iter().find(|subscription| {
                            subscription.dataflow_id == *dataflow_id
                                && subscription.topics == normalized_topics
                        }) {
                            let reply = ControlRequestReply::TopicSubscribed {
                                subscription_id: existing.subscription_id,
                                protocol_version: Some(TOPIC_DATA_PROTOCOL_VERSION),
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

                        let reply = ControlRequestReply::TopicSubscribed {
                            subscription_id,
                            protocol_version: Some(TOPIC_DATA_PROTOCOL_VERSION),
                        };
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
                                    "dataflow {dataflow_id} not found, output unavailable, or topic publish requires `debug.enable_debug_inspection: true`"
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

                // Keep forwarding pushed log lines and topic frames while the
                // reply is pending: `WaitForBuild` and `Stop` only answer once
                // the build or dataflow ends, and the logs they produce in the
                // meantime are pushed through this same connection.
                let Ok(reply) =
                    await_reply_forwarding_pushes(reply_rx, &mut ws_tx, &mut log_rx, &mut binary_rx)
                        .await
                else {
                    break;
                };

                let stop = matches!(reply, ControlRequestReply::CoordinatorStopped);

                let resp_json = format_response_json(req.id, &reply);
                if ws_tx.send(Message::Text(resp_json.into())).await.is_err() || stop {
                    break;
                }
            }
            // Log events to push to CLI
            Some(log_json) = log_rx.recv() => {
                if send_log_line(&mut ws_tx, log_json).await.is_err() {
                    break;
                }
            }
            // Binary topic data to push to CLI
            Some(frame) = binary_rx.recv() => {
                if send_topic_frame(&mut ws_tx, frame).await.is_err() {
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

/// Publish JSON data as a serialized `InterDaemonEvent::Output`.
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
    let topic = dora_core::topics::zenoh_daemon_control_topic(dataflow_id, node_id, output_id);

    // Encode the JSON (as a UInt8 array) into a self-describing Arrow IPC
    // stream — the framing every data-plane payload now uses, so the receiving
    // node's IPC decode reconstructs it.
    let array = arrow::array::UInt8Array::from(data_json.as_bytes().to_vec());
    let ipc_bytes = encode_topic_ipc(&array)
        .map_err(|e| format!("failed to Arrow-IPC-encode topic data: {e}"))?;
    let data = dora_message::aligned_vec::AVec::from_slice(128, &ipc_bytes);

    let mut params = MetadataParameters::new();
    params.insert(
        FRAMING.to_string(),
        Parameter::String(FRAMING_ARROW_IPC.to_string()),
    );

    let timestamp = clock.new_timestamp();
    let metadata = Metadata::from_parameters(timestamp, params);

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
            dora_core::topics::open_zenoh_session()
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

/// Encode `array` into a single-column Arrow IPC stream (column named `data`),
/// matching the framing produced by `dora_node_api::arrow_utils::encode_arrow_ipc`
/// so the node-side IPC decode reconstructs it.
fn encode_topic_ipc(array: &arrow::array::UInt8Array) -> eyre::Result<Vec<u8>> {
    use arrow::datatypes::{DataType, Field, Schema};
    use arrow::ipc::writer::StreamWriter;
    use arrow::record_batch::RecordBatch;
    use eyre::Context;
    use std::sync::Arc;

    let schema = Arc::new(Schema::new(vec![Field::new("data", DataType::UInt8, true)]));
    let batch = RecordBatch::try_new(schema.clone(), vec![Arc::new(array.clone())])
        .context("failed to create RecordBatch for topic IPC encoding")?;

    let mut buf = Vec::new();
    {
        let mut writer = StreamWriter::try_new(&mut buf, &schema)
            .context("failed to create Arrow IPC StreamWriter")?;
        writer
            .write(&batch)
            .context("failed to write RecordBatch to IPC stream")?;
        writer
            .finish()
            .context("failed to finish Arrow IPC stream")?;
    }
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A request whose reply is held back -- `WaitForBuild` is answered only
    /// when the build ends -- must not stop the connection from forwarding
    /// the log lines pushed to it meanwhile. Before the fix the WS loop sat in
    /// `reply_rx.await`, so after 64 lines the coordinator's sends blocked
    /// (and in production timed out, then dropped the subscriber).
    #[tokio::test]
    async fn logs_keep_flowing_while_a_reply_is_pending() {
        use futures::{SinkExt as _, StreamExt as _};
        use std::time::Duration;
        use tokio_tungstenite::tungstenite::Message as ClientMessage;

        const LINES: usize = 500;

        let (event_tx, mut event_rx) = mpsc::channel(16);
        let clock = Arc::new(dora_core::uhlc::HLC::default());
        let app =
            axum::Router::new().route(
                "/",
                axum::routing::any(move |ws: axum::extract::WebSocketUpgrade| {
                    let event_tx = event_tx.clone();
                    let clock = clock.clone();
                    async move {
                        ws.on_upgrade(move |socket| handle_control_ws(socket, event_tx, clock))
                    }
                }),
            );
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move { axum::serve(listener, app).await });

        // Stand-in for the coordinator event loop: accept the log subscription,
        // then push LINES log lines before answering the pending request.
        let coordinator = tokio::spawn(async move {
            let mut log_sender = None;
            while let Some(event) = event_rx.recv().await {
                match event {
                    Event::Control(ControlEvent::BuildLogSubscribe {
                        sender, found_tx, ..
                    }) => {
                        log_sender = Some(sender);
                        let _ = found_tx.send(true);
                    }
                    Event::Control(ControlEvent::IncomingRequest { reply_sender, .. }) => {
                        let sender = log_sender.take().expect("subscribed before the request");
                        for i in 0..LINES {
                            tokio::time::timeout(
                                Duration::from_secs(5),
                                sender.send(format!("{{\"line\":{i}}}")),
                            )
                            .await
                            .expect("log channel not drained while the reply was pending")
                            .expect("log channel closed");
                        }
                        let _ = reply_sender.send(Ok(ControlRequestReply::TopicPublished));
                        return;
                    }
                    _ => {}
                }
            }
        });

        let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/"))
            .await
            .unwrap();
        let mut send = async |request: ControlRequest| {
            let id = Uuid::new_v4();
            let req = WsRequest {
                id,
                method: "control".into(),
                params: serde_json::to_value(&request).unwrap(),
            };
            ws.send(ClientMessage::Text(
                serde_json::to_string(&req).unwrap().into(),
            ))
            .await
            .unwrap();
            id
        };
        let build_id = dora_message::BuildId::generate();
        send(ControlRequest::BuildLogSubscribe {
            build_id,
            level: log::LevelFilter::Info,
        })
        .await;
        let wait_id = send(ControlRequest::WaitForBuild { build_id }).await;

        let mut lines = 0;
        loop {
            let msg = tokio::time::timeout(Duration::from_secs(30), ws.next())
                .await
                .expect("timed out waiting for the connection")
                .expect("connection closed")
                .unwrap();
            let ClientMessage::Text(text) = msg else {
                continue;
            };
            let value: serde_json::Value = serde_json::from_str(&text).unwrap();
            if value.get("line").is_some() {
                assert_eq!(value["line"], lines, "log lines must arrive in order");
                lines += 1;
            } else if value["id"] == serde_json::json!(wait_id) {
                break;
            }
        }
        assert_eq!(lines, LINES, "every log line must reach the CLI");
        coordinator.await.unwrap();
    }

    /// The rejection has to be actionable: an operator seeing it should know
    /// which side is old without reading the source. Both arms name our
    /// version and say why a mismatch cannot simply be tolerated — binary
    /// topic frames are positionally encoded, so the wrong encoding misparses
    /// instead of erroring (dora-rs/dora#3153).
    /// The accept/reject decision itself, not just its wording: inverting or
    /// deleting the guard in the request handler must fail here.
    #[test]
    fn only_our_exact_protocol_version_is_accepted() {
        assert!(
            check_topic_protocol(Some(TOPIC_DATA_PROTOCOL_VERSION)).is_ok(),
            "our own version must be accepted"
        );
        assert!(
            check_topic_protocol(None).is_err(),
            "a pre-handshake client must be rejected, not defaulted to compatible"
        );
        for version in [0, 1, TOPIC_DATA_PROTOCOL_VERSION + 1, u16::MAX] {
            assert!(
                check_topic_protocol(Some(version)).is_err(),
                "version {version} must be rejected: binary frames are positional, \
                 so a mismatch misparses rather than failing"
            );
        }
    }

    /// The rejection must actually carry the shared explanation, not an empty
    /// or generic string — that text is what tells an operator which side is old.
    #[test]
    fn rejection_carries_the_shared_mismatch_message() {
        let err = check_topic_protocol(Some(1)).expect_err("version 1 must be rejected");
        assert_eq!(
            err,
            dora_message::topic_protocol_mismatch_message("client", Some(1)),
            "should reuse the shared message rather than a local variant"
        );
    }
}
