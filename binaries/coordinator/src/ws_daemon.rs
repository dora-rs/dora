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
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent, Timestamped},
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
    let mut dropped_debug_frames = DroppedDebugFrames::default();

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

/// Hand a daemon event to the main loop. Returns false if the connection
/// should close, on its channel closing.
///
/// A topic debug frame goes on its own channel and is dropped if that is full,
/// never waited for: waiting would stop this connection from reading the
/// socket, the daemon's next stop reply included, whenever the main loop falls
/// behind (dora-rs/dora#3535).
async fn forward_daemon_event(
    event: Event,
    event_tx: &mpsc::Sender<Event>,
    topic_debug_tx: &mpsc::Sender<Event>,
    dropped_debug_frames: &mut DroppedDebugFrames,
) -> bool {
    match event {
        Event::TopicDebugData { .. } => match topic_debug_tx.try_send(event) {
            Err(mpsc::error::TrySendError::Full(_)) => {
                dropped_debug_frames.record();
                true
            }
            result => result.is_ok(),
        },
        event => event_tx.send(event).await.is_ok(),
    }
}

/// Topic debug frames a daemon connection had to drop, warned about at most
/// once per interval with the count since the last warning.
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
            .is_none_or(|last| now - last >= std::time::Duration::from_secs(5))
        {
            tracing::warn!(
                "dropped {} topic debug frame(s): the coordinator's topic debug queue is full",
                self.count,
            );
            self.count = 0;
            self.last_log = Some(now);
        }
    }
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

/// Handle a daemon request (event or register). Returns false if the channel
/// it belongs on closed: the shared event one, or the topic debug one.
#[allow(clippy::too_many_arguments)]
async fn handle_daemon_request(
    raw_text: &str,
    event_tx: &mpsc::Sender<Event>,
    topic_debug_tx: &mpsc::Sender<Event>,
    dropped_debug_frames: &mut DroppedDebugFrames,
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
                register_request.zenoh_loopback_listen_endpoint,
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
            if let Some(coordinator_event) = translate_daemon_event(daemon_id, event, connection_id)
            {
                forward_daemon_event(
                    coordinator_event,
                    event_tx,
                    topic_debug_tx,
                    dropped_debug_frames,
                )
                .await
            } else {
                true
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
/// coordinator stores it and hands it to eligible daemons that register later,
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
fn accept_reported_zenoh_endpoint(
    endpoint: Option<String>,
    loopback_endpoint: Option<String>,
    state: &str,
) -> Option<String> {
    let uses_legacy_field = endpoint.is_some();
    let has_both_fields = uses_legacy_field && loopback_endpoint.is_some();
    let endpoint = endpoint.or(loopback_endpoint)?;
    match dora_core::topics::validate_zenoh_endpoint(&endpoint) {
        Ok(()) => {
            if uses_legacy_field && dora_message::zenoh::zenoh_endpoint_is_loopback(&endpoint) {
                // Pre-split daemons used this field for loopback too. Accept
                // them, but make the old format visible when debugging upgrades.
                tracing::debug!(
                    state,
                    %endpoint,
                    "daemon reported a loopback zenoh address in the legacy endpoint field"
                );
            }
            if has_both_fields {
                tracing::debug!(
                    state,
                    %endpoint,
                    "daemon reported both zenoh endpoint fields; preferring the legacy endpoint field"
                );
            }
            Some(endpoint)
        }
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
        DaemonEvent::ZenohListenEndpoint {
            endpoint,
            loopback_endpoint,
            ..
        } => Some(Event::DaemonZenohEndpoint {
            daemon_id,
            connection_id,
            endpoint: accept_reported_zenoh_endpoint(endpoint, loopback_endpoint, "connected"),
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
mod topic_debug_tests {
    use super::*;
    use futures::FutureExt;

    /// Regression test for dora-rs/dora#3535: with the main loop's topic debug
    /// channel full, a daemon connection drops the next frame instead of
    /// waiting for room, and the daemon's control events still get through.
    #[test]
    fn a_full_topic_debug_channel_does_not_hold_up_control_events() {
        let (event_tx, mut event_rx) = mpsc::channel(1);
        let (topic_debug_tx, mut topic_debug_rx) = mpsc::channel(1);
        let mut dropped = DroppedDebugFrames::default();
        let mut forward = |event| {
            forward_daemon_event(event, &event_tx, &topic_debug_tx, &mut dropped)
                .now_or_never()
                .expect("a daemon connection must not wait on the main loop")
        };
        let debug_frame = || Event::TopicDebugData {
            dataflow_id: Uuid::new_v4(),
            subscription_ids: Vec::new(),
            payload: Vec::new(),
        };

        assert!(forward(debug_frame()));
        assert!(
            forward(debug_frame()),
            "dropping a frame keeps the connection"
        );
        assert!(forward(Event::DaemonHeartbeat {
            daemon_id: DaemonId::new(None),
            ft_stats: None,
        }));

        assert!(matches!(
            event_rx.try_recv(),
            Ok(Event::DaemonHeartbeat { .. })
        ));
        assert!(topic_debug_rx.try_recv().is_ok());
        assert!(
            topic_debug_rx.try_recv().is_err(),
            "the second frame is dropped"
        );
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
            DaemonEvent::zenoh_listen_endpoint(endpoint),
            Uuid::new_v4(),
        ))
    }

    #[test]
    fn a_loopback_confirmation_is_kept_for_host_filtering() {
        assert_eq!(
            translate(Some("tcp/127.0.0.1:5456".into())),
            Some("tcp/127.0.0.1:5456".to_string())
        );
    }

    #[test]
    fn legacy_loopback_confirmations_are_kept_for_host_filtering() {
        for endpoint in ["tcp/127.0.0.1:5456", "tcp/localhost:5456#iface=lo"] {
            // Daemons built before the split advertised loopback in this
            // field. Decode their frame directly, without the new constructor.
            let json = serde_json::json!({"ZenohListenEndpoint": {"endpoint": endpoint}});
            let event: DaemonEvent = serde_json::from_value(json).unwrap();
            assert_eq!(
                endpoint_of(translate_daemon_event(
                    DaemonId::new(Some("A".to_string())),
                    event,
                    Uuid::new_v4(),
                )),
                Some(endpoint.to_string())
            );
        }
    }

    #[test]
    fn both_endpoint_fields_preserve_the_legacy_precedence() {
        assert_eq!(
            accept_reported_zenoh_endpoint(
                Some("tcp/10.0.2.100:5456".into()),
                Some("tcp/127.0.0.1:5456".into()),
                "connected",
            ),
            Some("tcp/10.0.2.100:5456".into())
        );
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
