use crate::{
    ArchivedDataflow, BuildFinishedResult, CachedResult, DaemonRequest, Event, dataflow_result,
    state, tcp_utils::tcp_receive,
};
use dora_core::uhlc::HLC;
use dora_message::{
    common::DaemonId,
    coordinator_to_cli::{DataflowResult, StopDataflowReply},
    daemon_to_coordinator::{
        CoordinatorNotify, CoordinatorRequest, DataflowDaemonResult, NodeMetrics,
        StateDeleteRequest, StateGetRequest, StateSetRequest, Timestamped,
    },
    tarpc,
};
use eyre::Context;
use std::{collections::BTreeMap, io::ErrorKind, net::SocketAddr, sync::Arc, time::Instant};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};
use uuid::Uuid;

pub async fn create_listener(bind: SocketAddr) -> eyre::Result<TcpListener> {
    let socket = match TcpListener::bind(bind).await {
        Ok(socket) => socket,
        Err(err) => {
            return Err(eyre::Report::new(err).wrap_err("failed to create local TCP listener"));
        }
    };
    Ok(socket)
}

pub async fn handle_connection(
    mut connection: TcpStream,
    events_tx: mpsc::Sender<Event>,
    clock: Arc<HLC>,
) {
    loop {
        // receive the next message and parse it
        let raw = match tcp_receive(&mut connection).await {
            Ok(data) => data,
            Err(err)
                if matches!(
                    err.kind(),
                    ErrorKind::UnexpectedEof
                        | ErrorKind::ConnectionAborted
                        | ErrorKind::ConnectionReset
                ) =>
            {
                break;
            }
            Err(err) => {
                tracing::error!("{err:?}");
                break;
            }
        };
        let message: Timestamped<CoordinatorRequest> = match serde_json::from_slice(&raw)
            .wrap_err_with(|| {
                format!(
                    "failed to deserialize message: {}",
                    String::from_utf8_lossy(&raw)
                )
            }) {
            Ok(e) => e,
            Err(err) => {
                tracing::warn!("{err:?}");
                continue;
            }
        };

        if let Err(err) = clock.update_with_timestamp(&message.timestamp) {
            tracing::warn!("failed to update coordinator clock: {err}");
        }

        match message.inner {
            CoordinatorRequest::Register(register_request) => {
                let event = DaemonRequest::Register {
                    connection,
                    version_check_result: register_request.check_version(),
                    machine_id: register_request.machine_id,
                    machine_uid: register_request.machine_uid,
                    zenoh_peer_id: register_request.zenoh_peer_id,
                };
                let _ = events_tx.send(Event::Daemon(event)).await;
                break;
            }
            CoordinatorRequest::RegisterNotificationChannel { daemon_id } => {
                let event = DaemonRequest::RegisterNotificationChannel {
                    daemon_id,
                    connection,
                };
                let _ = events_tx.send(Event::Daemon(event)).await;
                break;
            }
        };
    }
}

/// tarpc server that handles daemon→coordinator notification RPC calls.
///
/// Each daemon gets its own server instance, identified by `daemon_id`.
/// All notifications are handled directly using the shared `CoordinatorState`.
#[derive(Clone)]
pub struct CoordinatorNotifyServer {
    pub daemon_id: DaemonId,
    pub coordinator_state: Arc<state::CoordinatorState>,
}

impl CoordinatorNotify for CoordinatorNotifyServer {
    async fn all_nodes_ready(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        exited_before_subscribe: Vec<dora_message::id::NodeId>,
    ) {
        // Collect what we need from the DashMap entry, then drop the lock
        // before doing any async I/O.
        let send_info = match self.coordinator_state.running_dataflows.entry(dataflow_id) {
            dashmap::Entry::Occupied(mut entry) => {
                let dataflow = entry.get_mut();
                dataflow.pending_daemons.remove(&self.daemon_id);
                dataflow
                    .exited_before_subscribe
                    .extend(exited_before_subscribe);
                if dataflow.pending_daemons.is_empty() {
                    let exited = dataflow.exited_before_subscribe.clone();
                    let daemons: Vec<DaemonId> = dataflow.daemons.iter().cloned().collect();
                    Some((exited, daemons))
                } else {
                    None
                }
            }
            dashmap::Entry::Vacant(_) => {
                tracing::warn!("dataflow not running on ReadyOnMachine");
                None
            }
        };
        // DashMap lock is now dropped — safe to do async I/O.
        if let Some((exited_before_subscribe, daemons)) = send_info {
            tracing::debug!("sending all nodes ready message to daemons");
            for daemon_id in &daemons {
                let client = match self.coordinator_state.daemon_connections.get(daemon_id) {
                    Some(connection) => connection.client.clone(),
                    None => {
                        tracing::warn!("no daemon connection found for machine `{daemon_id}`");
                        continue;
                    }
                };
                // DashMap lock is dropped — safe to do async I/O.
                if let Err(err) = client
                    .all_nodes_ready(
                        tarpc::context::current(),
                        dataflow_id,
                        exited_before_subscribe.clone(),
                    )
                    .await
                {
                    tracing::error!(
                        "failed to send AllNodesReady({dataflow_id}) message \
                         to machine {daemon_id}: {err:?}"
                    );
                }
            }
        }
    }

    async fn all_nodes_finished(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        result: DataflowDaemonResult,
    ) {
        let daemon_id = self.daemon_id;
        tracing::debug!(
            "coordinator received DataflowFinishedOnDaemon ({daemon_id:?}, result: {result:?})"
        );
        match self.coordinator_state.running_dataflows.entry(dataflow_id) {
            dashmap::Entry::Occupied(mut entry) => {
                let dataflow = entry.get_mut();
                dataflow.daemons.remove(&daemon_id);
                tracing::info!(
                    "removed machine id: {daemon_id} from dataflow: {:#?}",
                    dataflow.uuid
                );
                self.coordinator_state
                    .dataflow_results
                    .entry(dataflow_id)
                    .or_default()
                    .insert(daemon_id, result);

                if dataflow.daemons.is_empty() {
                    // Archive finished dataflow
                    self.coordinator_state
                        .archived_dataflows
                        .entry(dataflow_id)
                        .or_insert_with(|| ArchivedDataflow::from(entry.get()));
                    let finished_dataflow = entry.remove();
                    let clock = &self.coordinator_state.clock;

                    let reply = StopDataflowReply {
                        uuid: dataflow_id,
                        result: self
                            .coordinator_state
                            .dataflow_results
                            .get(&dataflow_id)
                            .map(|r| dataflow_result(r.value(), dataflow_id, clock))
                            .unwrap_or_else(|| {
                                DataflowResult::ok_empty(dataflow_id, clock.new_timestamp())
                            }),
                    };
                    for sender in finished_dataflow.stop_reply_senders {
                        let _ = sender.send(Ok(reply.clone()));
                    }
                    if !matches!(finished_dataflow.spawn_result, CachedResult::Cached { .. }) {
                        log::error!("pending spawn result on dataflow finish");
                    }
                }
            }
            dashmap::Entry::Vacant(_) => {
                tracing::warn!("dataflow not running on DataflowFinishedOnDaemon");
            }
        }
    }

    async fn heartbeat(self, _ctx: tarpc::context::Context) {
        if let Some(mut connection_ref) = self
            .coordinator_state
            .daemon_connections
            .get_mut(&self.daemon_id)
        {
            connection_ref.last_heartbeat = Instant::now();
        }
    }

    async fn daemon_exit(self, _ctx: tarpc::context::Context) {
        tracing::info!("Daemon `{}` exited", self.daemon_id);
        self.coordinator_state
            .daemon_connections
            .remove(&self.daemon_id);
    }

    async fn node_metrics(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        metrics: BTreeMap<dora_message::id::NodeId, NodeMetrics>,
    ) {
        if let Some(mut dataflow) = self
            .coordinator_state
            .running_dataflows
            .get_mut(&dataflow_id)
        {
            for (node_id, node_metrics) in metrics {
                dataflow.node_metrics.insert(node_id, node_metrics);
            }
        }
    }

    async fn build_result(
        self,
        _ctx: tarpc::context::Context,
        build_id: dora_message::BuildId,
        result: Result<(), String>,
    ) {
        let daemon_id = self.daemon_id;
        let result = result.map_err(|err| eyre::eyre!(err));
        match self.coordinator_state.running_builds.entry(build_id) {
            dashmap::Entry::Occupied(mut entry) => {
                let build = entry.get_mut();
                build.pending_build_results.remove(&daemon_id);
                if let Err(err) = &result {
                    build.errors.push(format!("{err:?}"));
                }
                if build.pending_build_results.is_empty() {
                    tracing::info!("dataflow build finished: `{build_id}`");
                    let (build_id, mut build) = entry.remove_entry();
                    let result = if build.errors.is_empty() {
                        Ok(())
                    } else {
                        Err(format!("build failed: {}", build.errors.join("\n\n")))
                    };

                    build
                        .build_result
                        .set_result(Ok(BuildFinishedResult { build_id, result }));

                    self.coordinator_state
                        .finished_builds
                        .insert(build_id, build.build_result);
                }
            }
            dashmap::Entry::Vacant(_) => {
                tracing::warn!(
                    "received build result from daemon `{daemon_id}`, \
                     but no matching build `{build_id}` in `running_builds` map"
                );
            }
        }
    }

    async fn spawn_result(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        result: Result<(), String>,
    ) {
        let result = result.map_err(|err| eyre::eyre!(err));
        match self
            .coordinator_state
            .running_dataflows
            .get_mut(&dataflow_id)
        {
            Some(mut dataflow) => {
                dataflow.pending_spawn_results.remove(&self.daemon_id);
                match result {
                    Ok(()) => {
                        if dataflow.pending_spawn_results.is_empty() {
                            tracing::info!("successfully spawned dataflow `{dataflow_id}`");
                            dataflow.spawn_result.set_result(Ok(dataflow_id));
                        }
                    }
                    Err(err) => {
                        tracing::warn!("error while spawning dataflow `{dataflow_id}`");
                        dataflow.spawn_result.set_result(Err(err));
                    }
                }
            }
            None => {
                tracing::warn!(
                    "received spawn result, but no matching dataflow \
                     `{dataflow_id}` in `running_dataflows` map"
                );
            }
        }
    }

    async fn state_get(
        self,
        _ctx: tarpc::context::Context,
        request: StateGetRequest,
    ) -> Result<Option<Vec<u8>>, String> {
        Ok(self
            .coordinator_state
            .shared_state
            .get(&request.namespace, &request.key))
    }

    async fn state_set(
        self,
        _ctx: tarpc::context::Context,
        request: StateSetRequest,
    ) -> Result<(), String> {
        self.coordinator_state.shared_state.set(
            request.namespace,
            request.key,
            request.value,
            request.ttl_ms,
        )
    }

    async fn state_delete(
        self,
        _ctx: tarpc::context::Context,
        request: StateDeleteRequest,
    ) -> Result<bool, String> {
        Ok(self
            .coordinator_state
            .shared_state
            .delete(&request.namespace, &request.key))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::CoordinatorState;
    use dora_core::uhlc::HLC;
    use dora_message::daemon_to_coordinator::CoordinatorNotifyClient;
    use futures::{StreamExt, stream::AbortHandle};
    use tarpc::server::Channel;
    use tokio::sync::mpsc;

    fn test_state() -> Arc<CoordinatorState> {
        let (daemon_events_tx, _daemon_events_rx) = mpsc::channel(1);
        let (abort_handle, _abort_registration) = AbortHandle::new_pair();
        Arc::new(CoordinatorState {
            clock: Arc::new(HLC::default()),
            running_builds: Default::default(),
            finished_builds: Default::default(),
            running_dataflows: Default::default(),
            dataflow_results: Default::default(),
            archived_dataflows: Default::default(),
            shared_state: Default::default(),
            daemon_connections: Default::default(),
            daemon_events_tx,
            abort_handle,
        })
    }

    fn test_server(state: Arc<CoordinatorState>) -> CoordinatorNotifyServer {
        CoordinatorNotifyServer {
            daemon_id: DaemonId::new(Some("test-daemon".to_owned())),
            coordinator_state: state,
        }
    }

    fn state_client(server: CoordinatorNotifyServer) -> CoordinatorNotifyClient {
        let (client_transport, server_transport) = tarpc::transport::channel::unbounded();
        let channel = tarpc::server::BaseChannel::with_defaults(server_transport);
        tokio::spawn(channel.execute(server.serve()).for_each(|fut| async {
            tokio::spawn(fut);
        }));
        CoordinatorNotifyClient::new(tarpc::client::Config::default(), client_transport).spawn()
    }

    #[tokio::test]
    async fn state_set_get_delete_and_ttl_over_rpc() {
        let state = test_state();
        let client = state_client(test_server(state));

        client
            .state_set(
                tarpc::context::current(),
                StateSetRequest {
                    namespace: "runtime".to_owned(),
                    key: "node/camera".to_owned(),
                    value: b"v1".to_vec(),
                    ttl_ms: None,
                },
            )
            .await
            .expect("state_set should succeed")
            .expect("state_set result should be Ok");

        let value = client
            .state_get(
                tarpc::context::current(),
                StateGetRequest {
                    namespace: "runtime".to_owned(),
                    key: "node/camera".to_owned(),
                },
            )
            .await
            .expect("state_get should succeed")
            .expect("state_get result should be Ok");
        assert_eq!(value, Some(b"v1".to_vec()));

        let deleted = client
            .state_delete(
                tarpc::context::current(),
                StateDeleteRequest {
                    namespace: "runtime".to_owned(),
                    key: "node/camera".to_owned(),
                },
            )
            .await
            .expect("state_delete should succeed")
            .expect("state_delete result should be Ok");
        assert!(deleted);

        let missing = client
            .state_get(
                tarpc::context::current(),
                StateGetRequest {
                    namespace: "runtime".to_owned(),
                    key: "node/camera".to_owned(),
                },
            )
            .await
            .expect("state_get should succeed")
            .expect("state_get result should be Ok");
        assert!(missing.is_none());

        client
            .state_set(
                tarpc::context::current(),
                StateSetRequest {
                    namespace: "runtime".to_owned(),
                    key: "ephemeral".to_owned(),
                    value: b"temp".to_vec(),
                    ttl_ms: Some(20),
                },
            )
            .await
            .expect("state_set should succeed")
            .expect("state_set result should be Ok");

        tokio::time::sleep(std::time::Duration::from_millis(35)).await;

        let expired = client
            .state_get(
                tarpc::context::current(),
                StateGetRequest {
                    namespace: "runtime".to_owned(),
                    key: "ephemeral".to_owned(),
                },
            )
            .await
            .expect("state_get should succeed")
            .expect("state_get result should be Ok");
        assert!(expired.is_none());
    }
}
