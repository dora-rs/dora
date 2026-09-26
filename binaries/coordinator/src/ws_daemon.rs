use crate::{
    events::{DaemonRequest, DataflowEvent, Event},
    state::DaemonConnection,
};
use axum::extract::ws::{Message, WebSocket};
use dora_coordinator_store::CoordinatorStore;
use dora_core::uhlc::HLC;
use dora_message::{
    common::DaemonId,
    coordinator_to_daemon::ResolveMachineReply,
    daemon_to_coordinator::{
        CoordinatorRequest, DaemonEvent, Timestamped, TopicDebugFrameAssembler,
        decode_topic_debug_frame,
    },
    ws_protocol::WsResponse,
};
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
    topic_debug_tx: mpsc::Sender<Event>,
    clock: Arc<HLC>,
    store: Arc<dyn CoordinatorStore>,
    peer_addr: std::net::SocketAddr,
    daemon_peer_addrs: Arc<
        std::sync::RwLock<std::collections::HashMap<String, std::net::SocketAddr>>,
    >,
) {
    let (mut ws_tx, mut ws_rx) = socket.split();

    // Channel for coordinator -> daemon commands
    let (cmd_tx, mut cmd_rx) = mpsc::channel::<String>(64);
    let pending_replies: Arc<Mutex<HashMap<Uuid, oneshot::Sender<String>>>> =
        Arc::new(Mutex::new(HashMap::new()));

    // Track daemon_id and connection_id from incoming events for cleanup on disconnect
    let mut tracked_daemon_id: Option<DaemonId> = None;
    let mut tracked_connection_id: Option<Uuid> = None;
    // Topic debug frames this connection had to drop; see `DroppedDebugFrames`.
    let mut dropped_debug_frames = DroppedDebugFrames::default();
    // The binary topic debug frame whose chunks are arriving, if any.
    let mut debug_frame_assembler = TopicDebugFrameAssembler::default();

    loop {
        tokio::select! {
            // Incoming messages from daemon
            msg = ws_rx.next() => {
                let Some(msg) = msg else { break };
                let text = match msg {
                    Ok(Message::Text(text)) => text,
                    Ok(Message::Binary(data)) => {
                        if !handle_daemon_binary_frame(
                            &data,
                            &mut debug_frame_assembler,
                            &topic_debug_tx,
                            tracked_daemon_id.as_ref(),
                            &mut dropped_debug_frames,
                        ) {
                            break;
                        }
                        continue;
                    }
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

                // Distinguish request vs response by checking for "method" key.
                // Parse to Value first just for routing; the typed payload is
                // deserialized from the raw text below.
                let value: serde_json::Value = match serde_json::from_str(&text) {
                    Ok(v) => v,
                    Err(e) => {
                        tracing::warn!("invalid JSON from daemon WS: {e}");
                        continue;
                    }
                };

                if value.get("method").is_some() {
                    // Daemon request: deserialize Timestamped<CoordinatorRequest>
                    // directly from the raw text, skipping a second pass over
                    // the `Value` we only built for routing.
                    if !handle_daemon_request(
                        &text,
                        &event_tx,
                        &topic_debug_tx,
                        &mut dropped_debug_frames,
                        &clock,
                        &cmd_tx,
                        &pending_replies,
                        &store,
                        &mut tracked_daemon_id,
                        &mut tracked_connection_id,
                        peer_addr,
                        daemon_peer_addrs.clone(),
                    )
                    .await
                    {
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

    // Emit DaemonExit on WS close for immediate cleanup.
    // Only emit if we also have a connection_id — they are always set together,
    // but this guards against a half-initialized state.
    if let (Some(daemon_id), Some(connection_id)) = (tracked_daemon_id, tracked_connection_id) {
        tracing::info!("daemon WS connection closed for `{daemon_id}`");
        let _ = event_tx
            .send(Event::DaemonExit {
                daemon_id,
                connection_id,
            })
            .await;
    }
}

/// Capacity of the main loop's topic debug channel: one pending frame.
///
/// Debug frames reach the main loop on this channel rather than the shared
/// event one, and `crate::control_before_topic_debug` serves it only when no
/// control event is ready — so a debug frame can never be handled ahead of
/// control work, however many of them a subscription produces. What depth
/// would buy on top of that is only how far a burst may run ahead of the loop,
/// and a single frame can be a whole camera image: one pending frame is
/// enough, and it is what bounds the debug data resident here. A frame that
/// does not fit is dropped, which is what debug data is for.
pub(crate) const TOPIC_DEBUG_CHANNEL_CAPACITY: usize = 1;

/// The channel topic debug frames reach the main loop on, in either encoding.
pub(crate) fn topic_debug_channel() -> (mpsc::Sender<Event>, mpsc::Receiver<Event>) {
    mpsc::channel(TOPIC_DEBUG_CHANNEL_CAPACITY)
}

/// Handle a daemon WS binary message: a chunk of a topic debug frame, sent in
/// place of JSON `DaemonEvent::TopicDebugData` because this coordinator offered
/// `RegisterResult::Ok::binary_debug_frames` (dora-rs/dora#3535).
///
/// Chunks are collected in `assembler` until the frame's last one arrives;
/// only then is the frame handed on. A frame is sent as chunks so that the
/// daemon's control messages — which this loop reads between them — never
/// wait on the whole of a large frame, on either end of the socket.
///
/// Returns false if the connection should close: on the topic debug channel
/// closing, or on a frame from a daemon that has not registered, matching how
/// `handle_daemon_request` treats an unregistered daemon's events. A frame
/// that fails to decode is dropped with a warning; losing one debug frame is
/// not worth a daemon's connection.
///
/// The hand-off is a reservation, never an await: waiting for room would stop
/// this connection from reading the socket at all — including the daemon's
/// next stop reply — whenever the main loop falls behind (it can spend up to
/// 100 ms per subscriber per frame in `send_topic_frames`), which is the same
/// head-of-line block on the ingress side that the daemon's writer avoids on
/// egress. Debug frames are droppable, so a full channel drops the frame.
///
/// The slot is taken before the frame is decoded, so a frame that will be
/// dropped is not copied again to build its event.
fn handle_daemon_binary_frame(
    data: &[u8],
    assembler: &mut TopicDebugFrameAssembler,
    topic_debug_tx: &mpsc::Sender<Event>,
    tracked_daemon_id: Option<&DaemonId>,
    dropped: &mut DroppedDebugFrames,
) -> bool {
    if tracked_daemon_id.is_none() {
        tracing::warn!("daemon sent binary frame before registering — closing connection");
        return false;
    }
    let frame = match assembler.push(data) {
        Ok(Some(frame)) => frame,
        Ok(None) => return true,
        Err(err) => {
            tracing::warn!("dropping malformed topic debug frame from daemon: {err}");
            return true;
        }
    };
    let slot = match topic_debug_tx.try_reserve() {
        Ok(slot) => slot,
        Err(mpsc::error::TrySendError::Full(())) => {
            dropped.record();
            return true;
        }
        Err(mpsc::error::TrySendError::Closed(())) => return false,
    };
    match decode_daemon_binary_frame(&frame) {
        Ok(event) => {
            slot.send(event);
            true
        }
        // Dropping `slot` here returns the capacity it held.
        Err(err) => {
            tracing::warn!("dropping malformed topic debug frame from daemon: {err}");
            true
        }
    }
}

/// Hand an already-decoded topic debug event to the main loop, or drop it.
///
/// The JSON counterpart of the binary path above, for a daemon that did not
/// negotiate binary frames: same channel, same drop-rather-than-await rule, so
/// neither shape can queue ahead of control events.
fn try_send_topic_debug(
    event: Event,
    topic_debug_tx: &mpsc::Sender<Event>,
    dropped: &mut DroppedDebugFrames,
) -> bool {
    match topic_debug_tx.try_send(event) {
        Ok(()) => true,
        Err(mpsc::error::TrySendError::Full(_)) => {
            dropped.record();
            true
        }
        Err(mpsc::error::TrySendError::Closed(_)) => false,
    }
}

/// Shortest gap between warnings about dropped topic debug frames. A backlog
/// produces them at the subscription's rate, so each connection reports at most
/// one line per interval, with the count since the last one.
const TOPIC_DEBUG_DROP_LOG_INTERVAL: std::time::Duration = std::time::Duration::from_secs(5);

/// Rate-limited accounting of topic debug frames the coordinator could not
/// admit, kept per daemon connection.
#[derive(Default)]
struct DroppedDebugFrames {
    count: u64,
    last_log: Option<std::time::Instant>,
}

impl DroppedDebugFrames {
    fn record(&mut self) {
        self.count += 1;
        let now = std::time::Instant::now();
        if self
            .last_log
            .is_none_or(|last| now.duration_since(last) >= TOPIC_DEBUG_DROP_LOG_INTERVAL)
        {
            tracing::warn!(
                "dropped {} topic debug frame(s): the coordinator is still handling the last one",
                self.count,
            );
            self.count = 0;
            self.last_log = Some(now);
        }
    }
}

/// Decode a reassembled binary topic debug frame into the same
/// [`Event::TopicDebugData`] the JSON `DaemonEvent::TopicDebugData` translates
/// to.
fn decode_daemon_binary_frame(data: &[u8]) -> eyre::Result<Event> {
    let frame = decode_topic_debug_frame(data)?;
    Ok(Event::TopicDebugData {
        dataflow_id: frame.dataflow_id,
        subscription_ids: frame.subscription_ids,
        payload: frame.payload.to_vec(),
    })
}

/// A helper struct to deserialize `Timestamped<CoordinatorRequest>` directly
/// from the raw JSON text, so the payload is parsed once into its real type
/// instead of going through the `serde_json::Value` used for routing.
#[derive(serde::Deserialize)]
struct DaemonWsRequestRaw {
    /// Request id from the daemon envelope — echoed back in replies so
    /// the daemon can route the reply to its pending caller.
    id: Uuid,
    params: dora_message::daemon_to_coordinator::Timestamped<
        dora_message::daemon_to_coordinator::CoordinatorRequest,
    >,
}

/// Handle a daemon request (event or register). Returns false if the channel it
/// belongs on closed — the shared event one, or the topic debug one.
#[allow(clippy::too_many_arguments)]
async fn handle_daemon_request(
    raw_text: &str,
    event_tx: &mpsc::Sender<Event>,
    topic_debug_tx: &mpsc::Sender<Event>,
    dropped: &mut DroppedDebugFrames,
    clock: &HLC,
    cmd_tx: &mpsc::Sender<String>,
    pending_replies: &Arc<Mutex<HashMap<Uuid, oneshot::Sender<String>>>>,
    store: &Arc<dyn CoordinatorStore>,
    tracked_daemon_id: &mut Option<DaemonId>,
    tracked_connection_id: &mut Option<Uuid>,
    peer_addr: std::net::SocketAddr,
    daemon_peer_addrs: Arc<
        std::sync::RwLock<std::collections::HashMap<String, std::net::SocketAddr>>,
    >,
) -> bool {
    let parsed: DaemonWsRequestRaw = match serde_json::from_str(raw_text) {
        Ok(m) => m,
        Err(e) => {
            tracing::warn!("failed to parse daemon request: {e}");
            return true;
        }
    };
    let message = parsed.params;
    let request_id = parsed.id;

    if let Err(err) = clock.update_with_timestamp(&message.timestamp) {
        tracing::warn!("failed to update coordinator clock: {err}");
    }

    match message.inner {
        CoordinatorRequest::Register(register_request) => {
            // Reject re-registration on the same connection
            if tracked_daemon_id.is_some() {
                tracing::warn!("daemon attempted re-register on same connection, rejecting");
                return false;
            }
            // Validate machine_id length to prevent abuse
            if let Some(ref mid) = register_request.machine_id
                && mid.len() > 256
            {
                tracing::warn!(
                    "daemon register rejected: machine_id too long ({} bytes)",
                    mid.len()
                );
                return false;
            }
            let version_check_result = register_request.check_version();
            // capture before the partial moves below consume `register_request`
            let supports_hub_sources = register_request.supports_hub_sources();
            let zenoh_listen_endpoint = accept_reported_zenoh_endpoint(
                register_request.zenoh_listen_endpoint,
                "registering",
            );
            let labels = register_request.labels;
            let machine_id = register_request.machine_id;
            let mut connection =
                DaemonConnection::new(cmd_tx.clone(), pending_replies.clone(), labels.clone());
            connection.supports_hub_sources = supports_hub_sources;
            connection.peer_addr = Some(peer_addr);
            // Recorded here, so it is on the connection before it is added to
            // the registry — the next daemon's register reply, built later on
            // the same serial event loop, therefore already contains it.
            connection.zenoh_listen_endpoint = zenoh_listen_endpoint;
            // Capture the connection_id before moving `connection` into the event.
            let connection_id = connection.connection_id;
            let (daemon_id_tx, daemon_id_rx) = oneshot::channel();
            let event = DaemonRequest::Register {
                connection,
                version_check_result,
                machine_id,
                labels,
                daemon_id_tx,
            };
            if event_tx.send(Event::Daemon(event)).await.is_err() {
                return false;
            }
            // Capture the assigned daemon_id and connection_id for cleanup on disconnect
            if let Ok(daemon_id) = daemon_id_rx.await {
                *tracked_daemon_id = Some(daemon_id);
                *tracked_connection_id = Some(connection_id);
            }
            true
        }
        CoordinatorRequest::Event { daemon_id, event } => {
            match tracked_daemon_id {
                None => {
                    tracing::warn!("daemon sent event before registering — closing connection");
                    return false;
                }
                Some(tracked) if *tracked != daemon_id => {
                    tracing::warn!(
                        "daemon sent event with mismatched id: expected `{tracked}`, got `{daemon_id}` — closing connection"
                    );
                    return false;
                }
                Some(_) => {}
            }
            // Use the tracked connection_id (set at registration); fall back to a fresh
            // UUID if somehow called before registration (shouldn't happen in practice).
            let connection_id = tracked_connection_id.unwrap_or_else(Uuid::new_v4);
            match translate_daemon_event(daemon_id, event, connection_id) {
                // Debug data takes the debug channel whichever shape it
                // arrived in: a JSON frame from a daemon that did not
                // negotiate binary frames must not queue ahead of control
                // events either, and is equally droppable.
                Some(event @ Event::TopicDebugData { .. }) => {
                    try_send_topic_debug(event, topic_debug_tx, dropped)
                }
                Some(coordinator_event) => event_tx.send(coordinator_event).await.is_ok(),
                None => true,
            }
        }
        CoordinatorRequest::ResolveMachine { machine_id } => {
            // Resolve the machine id against the registered-daemon store;
            // unknown machines (or store errors) resolve to `found: false`.
            // Also report the target daemon's WS peer address so the
            // requesting daemon can reach its direct-TCP data listener.
            let (found, address) = match store.get_daemon_by_machine(&machine_id) {
                Ok(Some(d)) => (
                    true,
                    daemon_peer_addrs
                        .read()
                        .unwrap_or_else(|e| e.into_inner())
                        .get(&d.to_string())
                        .copied(),
                ),
                Ok(None) => (false, None),
                Err(e) => {
                    tracing::warn!("failed to resolve machine `{machine_id}`: {e}");
                    (false, None)
                }
            };
            // Reply over the same WS envelope the Register flow uses
            // (`{"id", "method": "daemon_event", "params": <Timestamped<...>>}`),
            // mirroring `DaemonConnection::send`.
            let reply = Timestamped {
                inner: ResolveMachineReply::ResolveMachineResult { found, address },
                timestamp: clock.new_timestamp(),
            };
            let params = match serde_json::to_string(&reply) {
                Ok(params) => params,
                Err(err) => {
                    tracing::warn!("failed to serialize ResolveMachine reply: {err}");
                    return true;
                }
            };
            // Echo the request id so the daemon can route this reply to
            // its pending caller (COORDINATOR_PENDING).
            let json =
                format!(r#"{{"id":"{request_id}","method":"daemon_event","params":{params}}}"#);
            if cmd_tx.send(json).await.is_err() {
                return false;
            }
            true
        }
        // `CoordinatorRequest` is `#[non_exhaustive]`: a newer daemon may send a
        // variant this coordinator predates. Keep the connection open and drop
        // the request rather than tearing down a daemon over an unknown message.
        _ => {
            tracing::warn!(
                "ignoring unrecognized request from daemon (daemon is likely newer than this coordinator)"
            );

            true
        }
    }
}

/// Accept a zenoh endpoint a daemon reported, or drop it.
///
/// Both ways a daemon can report one — in its registration, and in the
/// `ZenohListenEndpoint` correction that follows — end at the same place: the
/// coordinator stores it and hands it to every daemon that registers later,
/// which puts it straight into their zenoh `connect/endpoints`. So both are
/// checked here, through one function, rather than at each site: a daemon is
/// not a trusted input just because it registered, and the correction exists
/// precisely to *replace* the registered value, so validating only the
/// registration would have left the invariant to whichever path ran last.
///
/// A rejected value becomes `None`, which withdraws any endpoint already on
/// record rather than leaving a stale one standing. Withdrawal is the safe
/// direction: the daemon simply is not dialed directly and its dataflows fall
/// back to the daemon-forwarded path. Never fatal — the daemon is otherwise
/// healthy, and dropping its connection over a malformed endpoint would cost
/// far more than the direct link is worth.
fn accept_reported_zenoh_endpoint(endpoint: Option<String>, state: &str) -> Option<String> {
    let endpoint = endpoint?;
    match dora_core::topics::validate_zenoh_endpoint(&endpoint) {
        Ok(()) => Some(endpoint),
        Err(err) => {
            tracing::warn!("ignoring zenoh endpoint reported by {state} daemon: {err}");
            None
        }
    }
}

fn translate_daemon_event(
    daemon_id: DaemonId,
    event: DaemonEvent,
    connection_id: Uuid,
) -> Option<Event> {
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
        DaemonEvent::Heartbeat { ft_stats } => Some(Event::DaemonHeartbeat {
            daemon_id,
            ft_stats,
        }),
        DaemonEvent::ZenohListenEndpoint { endpoint } => Some(Event::DaemonZenohEndpoint {
            daemon_id,
            connection_id,
            endpoint: accept_reported_zenoh_endpoint(endpoint, "connected"),
        }),
        DaemonEvent::Log(message) => Some(Event::Log(message)),
        DaemonEvent::Exit => Some(Event::DaemonExit {
            daemon_id,
            connection_id,
        }),
        DaemonEvent::NodeMetrics {
            dataflow_id,
            metrics,
            network,
        } => Some(Event::NodeMetrics {
            dataflow_id,
            metrics,
            network,
        }),
        DaemonEvent::TopicDebugData {
            dataflow_id,
            subscription_ids,
            payload,
        } => Some(Event::TopicDebugData {
            dataflow_id,
            subscription_ids,
            payload,
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
        DaemonEvent::StatusReport { running_dataflows } => Some(Event::DaemonStatusReport {
            daemon_id,
            running_dataflows,
        }),
        DaemonEvent::StateCatchUpAck {
            dataflow_id,
            ack_sequence,
        } => Some(Event::DaemonStateCatchUpAck {
            daemon_id,
            dataflow_id,
            ack_sequence,
        }),
        DaemonEvent::NodeStopped {
            dataflow_id,
            node_id,
            clean_stop,
        } => Some(Event::DaemonNodeStopped {
            daemon_id,
            dataflow_id,
            node_id,
            clean_stop,
        }),
        // `DaemonEvent` is `#[non_exhaustive]`: a newer daemon may report an
        // event this coordinator has no translation for. `None` is already the
        // "nothing to forward" signal, so an unknown event is dropped quietly.
        _ => {
            tracing::debug!("ignoring unrecognized daemon event from `{daemon_id}`");
            None
        }
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
            tracing::warn!("daemon returned error for request {}: {err}", response.id);
            serde_json::json!({"ws_error": err}).to_string()
        } else {
            "null".to_string()
        };
        let _ = sender.send(result_json);
    } else {
        tracing::warn!("no pending reply for daemon WS response id {}", response.id);
    }
}

#[cfg(test)]
mod topic_debug_frame_tests {
    use super::*;
    use dora_message::daemon_to_coordinator::{
        TOPIC_DEBUG_CHUNK_BYTES, encode_topic_debug_chunks, encode_topic_debug_frame,
    };

    /// A small frame as the single binary message a daemon sends it in.
    fn one_chunk(dataflow_id: Uuid, subscription_ids: &[Uuid], payload: &[u8]) -> Vec<u8> {
        let [chunk] = <[_; 1]>::try_from(
            encode_topic_debug_chunks(dataflow_id, subscription_ids, payload).unwrap(),
        )
        .expect("a small frame fits one chunk");
        chunk
    }

    fn topic_debug_data(event: Option<Event>) -> (Uuid, Vec<Uuid>, Vec<u8>) {
        match event {
            Some(Event::TopicDebugData {
                dataflow_id,
                subscription_ids,
                payload,
                ..
            }) => (dataflow_id, subscription_ids, payload),
            other => panic!("expected a TopicDebugData event, got {other:?}"),
        }
    }

    /// The channel the main loop takes debug frames off, at the capacity the
    /// coordinator really runs with.
    fn debug_channel() -> (mpsc::Sender<Event>, mpsc::Receiver<Event>) {
        topic_debug_channel()
    }

    /// Both shapes a daemon may send — JSON from a daemon that did not get (or
    /// does not know) the flag, binary from one that did — must reach the
    /// coordinator as the same event.
    #[test]
    fn json_and_binary_frames_decode_to_the_same_event() {
        let dataflow_id = Uuid::new_v4();
        let subscription_ids = vec![Uuid::new_v4(), Uuid::new_v4()];
        let payload = vec![0, 1, 2, 254, 255];

        let from_json = topic_debug_data(translate_daemon_event(
            DaemonId::new(Some("A".to_string())),
            DaemonEvent::TopicDebugData {
                dataflow_id,
                subscription_ids: subscription_ids.clone(),
                payload: payload.clone(),
            },
            Uuid::new_v4(),
        ));
        let binary = encode_topic_debug_frame(dataflow_id, &subscription_ids, &payload).unwrap();
        let from_binary = topic_debug_data(decode_daemon_binary_frame(&binary).ok());

        assert_eq!(from_json, (dataflow_id, subscription_ids, payload));
        assert_eq!(from_binary, from_json);
    }

    #[test]
    fn a_binary_frame_is_forwarded_once_the_daemon_is_registered() {
        let (topic_debug_tx, mut topic_debug_rx) = debug_channel();
        let daemon_id = DaemonId::new(Some("A".to_string()));
        let dataflow_id = Uuid::new_v4();
        let frame = one_chunk(dataflow_id, &[Uuid::new_v4()], b"data");

        assert!(handle_daemon_binary_frame(
            &frame,
            &mut TopicDebugFrameAssembler::default(),
            &topic_debug_tx,
            Some(&daemon_id),
            &mut DroppedDebugFrames::default()
        ));
        let (forwarded_dataflow, _, payload) = topic_debug_data(topic_debug_rx.try_recv().ok());
        assert_eq!(forwarded_dataflow, dataflow_id);
        assert_eq!(payload, b"data");
    }

    /// A frame larger than a chunk reaches the main loop whole, and only once
    /// its last chunk is in — nothing of it is handed on part-way.
    #[test]
    fn a_chunked_frame_is_forwarded_once_its_last_chunk_arrives() {
        let (topic_debug_tx, mut topic_debug_rx) = debug_channel();
        let daemon_id = DaemonId::new(Some("A".to_string()));
        let mut assembler = TopicDebugFrameAssembler::default();
        let dataflow_id = Uuid::new_v4();
        let payload: Vec<u8> = (0..2 * TOPIC_DEBUG_CHUNK_BYTES + 3)
            .map(|i| i as u8)
            .collect();
        let chunks = encode_topic_debug_chunks(dataflow_id, &[Uuid::new_v4()], &payload).unwrap();
        assert_eq!(chunks.len(), 3);

        for chunk in &chunks {
            assert!(
                topic_debug_rx.try_recv().is_err(),
                "a partial frame must not be forwarded"
            );
            assert!(handle_daemon_binary_frame(
                chunk,
                &mut assembler,
                &topic_debug_tx,
                Some(&daemon_id),
                &mut DroppedDebugFrames::default()
            ));
        }
        let (forwarded_dataflow, _, forwarded) = topic_debug_data(topic_debug_rx.try_recv().ok());
        assert_eq!(forwarded_dataflow, dataflow_id);
        assert_eq!(forwarded, payload);
    }

    #[test]
    fn a_binary_frame_before_registration_closes_the_connection() {
        let (topic_debug_tx, mut topic_debug_rx) = debug_channel();
        let frame = one_chunk(Uuid::new_v4(), &[], b"data");

        assert!(!handle_daemon_binary_frame(
            &frame,
            &mut TopicDebugFrameAssembler::default(),
            &topic_debug_tx,
            None,
            &mut DroppedDebugFrames::default()
        ));
        assert!(topic_debug_rx.try_recv().is_err());
    }

    /// A frame is decoded only once it has a slot, so a malformed one has to
    /// give that slot back — otherwise a daemon sending garbage would keep the
    /// channel occupied against every other daemon.
    #[test]
    fn a_malformed_binary_frame_is_dropped_without_closing_or_holding_a_slot() {
        let (topic_debug_tx, mut topic_debug_rx) = debug_channel();
        let daemon_id = DaemonId::new(Some("A".to_string()));

        assert!(handle_daemon_binary_frame(
            &[0, 1, 2, 3],
            &mut TopicDebugFrameAssembler::default(),
            &topic_debug_tx,
            Some(&daemon_id),
            &mut DroppedDebugFrames::default()
        ));
        assert!(topic_debug_rx.try_recv().is_err());

        let frame = one_chunk(Uuid::new_v4(), &[], b"data");
        assert!(handle_daemon_binary_frame(
            &frame,
            &mut TopicDebugFrameAssembler::default(),
            &topic_debug_tx,
            Some(&daemon_id),
            &mut DroppedDebugFrames::default()
        ));
        assert!(
            topic_debug_rx.try_recv().is_ok(),
            "the malformed frame must not have consumed the channel slot"
        );
    }

    /// With the main loop still on the last frame, the next one is dropped and
    /// the connection stays open: the alternative is waiting for room while the
    /// daemon's control traffic — a stop reply among it — goes unread.
    #[test]
    fn a_binary_frame_is_dropped_rather_than_awaited_when_the_channel_is_full() {
        let (topic_debug_tx, mut topic_debug_rx) = debug_channel();
        let daemon_id = DaemonId::new(Some("A".to_string()));
        let frame = one_chunk(Uuid::new_v4(), &[Uuid::new_v4()], b"data");
        let mut dropped = DroppedDebugFrames::default();

        for _ in 0..TOPIC_DEBUG_CHANNEL_CAPACITY + 2 {
            assert!(handle_daemon_binary_frame(
                &frame,
                &mut TopicDebugFrameAssembler::default(),
                &topic_debug_tx,
                Some(&daemon_id),
                &mut dropped
            ));
        }

        for _ in 0..TOPIC_DEBUG_CHANNEL_CAPACITY {
            assert!(topic_debug_rx.try_recv().is_ok());
        }
        assert!(
            topic_debug_rx.try_recv().is_err(),
            "frames past the channel capacity are dropped, not queued"
        );

        // Room returns as soon as the loop has taken one off.
        assert!(handle_daemon_binary_frame(
            &frame,
            &mut TopicDebugFrameAssembler::default(),
            &topic_debug_tx,
            Some(&daemon_id),
            &mut dropped
        ));
        assert!(topic_debug_rx.try_recv().is_ok());
    }

    /// A closed channel means the coordinator is gone: close the socket
    /// instead of dropping frames into it forever.
    #[test]
    fn a_binary_frame_on_a_closed_channel_closes_the_connection() {
        let (topic_debug_tx, topic_debug_rx) = debug_channel();
        drop(topic_debug_rx);
        let daemon_id = DaemonId::new(Some("A".to_string()));
        let frame = one_chunk(Uuid::new_v4(), &[], b"data");

        assert!(!handle_daemon_binary_frame(
            &frame,
            &mut TopicDebugFrameAssembler::default(),
            &topic_debug_tx,
            Some(&daemon_id),
            &mut DroppedDebugFrames::default()
        ));
    }

    /// The JSON shape reaches the main loop the same way, so a daemon that did
    /// not negotiate binary frames cannot put debug data on the shared event
    /// channel — where it would sit ahead of the control events behind it.
    #[tokio::test]
    async fn a_json_topic_debug_event_takes_the_debug_channel_too() {
        let daemon_id = DaemonId::new(Some("A".to_string()));
        let dataflow_id = Uuid::new_v4();
        let clock = HLC::default();
        let request = Timestamped {
            inner: CoordinatorRequest::Event {
                daemon_id: daemon_id.clone(),
                event: DaemonEvent::TopicDebugData {
                    dataflow_id,
                    subscription_ids: vec![Uuid::new_v4()],
                    payload: vec![1, 2, 3],
                },
            },
            timestamp: clock.new_timestamp(),
        };
        let raw = format!(
            r#"{{"id":"{}","method":"daemon_event","params":{}}}"#,
            Uuid::new_v4(),
            serde_json::to_string(&request).unwrap()
        );

        let (event_tx, mut event_rx) = mpsc::channel(8);
        let (topic_debug_tx, mut topic_debug_rx) = debug_channel();
        let (cmd_tx, _cmd_rx) = mpsc::channel(8);
        let store: Arc<dyn CoordinatorStore> = Arc::new(crate::InMemoryStore::new());

        assert!(
            handle_daemon_request(
                &raw,
                &event_tx,
                &topic_debug_tx,
                &mut DroppedDebugFrames::default(),
                &clock,
                &cmd_tx,
                &Arc::new(Mutex::new(HashMap::new())),
                &store,
                &mut Some(daemon_id),
                &mut Some(Uuid::new_v4()),
                "127.0.0.1:1234".parse().unwrap(),
                Arc::new(std::sync::RwLock::new(HashMap::new())),
            )
            .await
        );

        assert!(
            event_rx.try_recv().is_err(),
            "topic debug data must not take the shared event channel"
        );
        let (forwarded_dataflow, _, payload) = topic_debug_data(topic_debug_rx.try_recv().ok());
        assert_eq!(forwarded_dataflow, dataflow_id);
        assert_eq!(payload, vec![1, 2, 3]);
    }
}

#[cfg(test)]
mod reported_endpoint_tests {
    use super::*;

    fn endpoint_of(event: Option<Event>) -> Option<String> {
        match event {
            Some(Event::DaemonZenohEndpoint { endpoint, .. }) => endpoint,
            other => panic!("expected a DaemonZenohEndpoint event, got {other:?}"),
        }
    }

    fn translate(endpoint: Option<String>) -> Option<String> {
        endpoint_of(translate_daemon_event(
            DaemonId::new(Some("A".to_string())),
            DaemonEvent::ZenohListenEndpoint { endpoint },
            Uuid::new_v4(),
        ))
    }

    #[test]
    fn a_valid_reported_endpoint_is_kept() {
        assert_eq!(
            translate(Some("tcp/10.0.2.100:5456".into())),
            Some("tcp/10.0.2.100:5456".to_string())
        );
    }

    /// The correction path is the *second* way a daemon can report an endpoint,
    /// and it is designed to replace the value validated at registration — so
    /// it has to be checked too, or the invariant holds only until the
    /// correction arrives.
    #[test]
    fn an_invalid_reported_endpoint_is_dropped_on_the_correction_path() {
        assert_eq!(
            translate(Some(r#"tcp/1.2.3.4:1","tcp/evil:7447"#.into())),
            None
        );
        assert_eq!(translate(Some("tcp/1.2.3.4:1 rm -rf".into())), None);
        assert_eq!(translate(Some("a".repeat(300))), None);
    }

    /// An explicit withdrawal must still reach the coordinator: it is how a
    /// daemon whose listener failed to bind stops being advertised.
    #[test]
    fn an_explicit_withdrawal_passes_through() {
        assert_eq!(translate(None), None);
    }
}
