use crate::{
    Event, log, read_last_n_lines, send_with_timestamp,
    socket_stream_utils::{socket_stream_receive, socket_stream_send},
    state::DaemonState,
};
use dora_core::uhlc::HLC;
use dora_message::{
    DataflowId,
    common::{DaemonId, Timestamped},
    coordinator_to_daemon::{
        BuildDataflowNodes, DaemonControl, DaemonControlRequest, DaemonControlResponse,
        RegisterResult, SpawnDataflowNodes,
    },
    daemon_to_coordinator::{
        CoordinatorRequest, DaemonRegisterRequest, DaemonToCoordinatorControlClient,
        DaemonToCoordinatorControlRequest, DaemonToCoordinatorControlResponse,
    },
    daemon_to_node::NodeEvent,
    id::{NodeId, OperatorId},
    tarpc::{
        self, ClientMessage, Response, client,
        server::{BaseChannel, Channel},
        tokio_serde,
    },
};
use eyre::Context;
use futures::StreamExt;
use std::{net::SocketAddr, sync::Arc, time::Duration};
use tokio::{io::AsyncReadExt, net::TcpStream, task::JoinHandle, time::sleep};
use tracing::warn;

const DAEMON_COORDINATOR_RETRY_INTERVAL: std::time::Duration = Duration::from_secs(1);

/// Result of [`register`]: everything the daemon needs after connecting to the
/// coordinator.
pub struct RegisterResult2 {
    pub daemon_id: DaemonId,
    /// Handle for the coordinator→daemon tarpc server task.
    pub rpc_server_handle: JoinHandle<()>,
    /// tarpc client for daemon→coordinator RPC calls.
    pub coordinator_client: DaemonToCoordinatorControlClient,
}

/// Connect to the coordinator, register, set up bidirectional tarpc channels.
///
/// 1. Opens a TCP connection, sends `Register`, receives `DaemonId`.
/// 2. Converts that connection into a tarpc server (coordinator→daemon `DaemonControl`).
/// 3. Opens a **second** TCP connection, sends `RegisterReverseChannel`, and
///    creates a tarpc client (daemon→coordinator `DaemonToCoordinatorControl`).
pub async fn register(
    addr: SocketAddr,
    machine_id: Option<String>,
    clock: &Arc<HLC>,
    state: Arc<DaemonState>,
) -> eyre::Result<RegisterResult2> {
    // --- First connection: registration + coordinator→daemon RPC ---
    let mut stream = loop {
        match TcpStream::connect(addr)
            .await
            .wrap_err("failed to connect to dora-coordinator")
        {
            Err(err) => {
                warn!(
                    "Could not connect to: {addr}, with error: {err}. Retrying in {DAEMON_COORDINATOR_RETRY_INTERVAL:#?}.."
                );
                sleep(DAEMON_COORDINATOR_RETRY_INTERVAL).await;
            }
            Ok(stream) => {
                break stream;
            }
        };
    };
    stream
        .set_nodelay(true)
        .wrap_err("failed to set TCP_NODELAY")?;

    // Registration handshake (raw length-prefixed JSON)
    let register = serde_json::to_vec(&Timestamped {
        inner: CoordinatorRequest::Register(DaemonRegisterRequest::new(machine_id)),
        timestamp: clock.new_timestamp(),
    })?;
    socket_stream_send(&mut stream, &register)
        .await
        .wrap_err("failed to send register request to dora-coordinator")?;
    let reply_raw = socket_stream_receive(&mut stream)
        .await
        .wrap_err("failed to receive register reply from dora-coordinator")?;
    let result: Timestamped<RegisterResult> = serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize dora-coordinator reply")?;
    let daemon_id = result.inner.to_result()?;
    if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
        tracing::warn!("failed to update timestamp after register: {err}");
    }

    tracing::info!("Connected to dora-coordinator at {:?}", addr);

    // Set up tarpc server for coordinator→daemon RPC on the registered stream
    let codec = tokio_serde::formats::Json::<
        ClientMessage<DaemonControlRequest>,
        Response<DaemonControlResponse>,
    >::default();
    let transport = tarpc::serde_transport::Transport::from((stream, codec));

    let server = DaemonRpcServer {
        state: state.clone(),
    };

    let channel = BaseChannel::with_defaults(transport);
    let rpc_server_handle = tokio::spawn(channel.execute(server.serve()).for_each(|fut| async {
        tokio::spawn(fut);
    }));

    // --- Second connection: daemon→coordinator RPC ---
    let mut reverse_stream = TcpStream::connect(addr)
        .await
        .wrap_err("failed to open reverse channel to dora-coordinator")?;
    reverse_stream
        .set_nodelay(true)
        .wrap_err("failed to set TCP_NODELAY on reverse channel")?;

    let reverse_register = serde_json::to_vec(&Timestamped {
        inner: CoordinatorRequest::RegisterNotificationChannel {
            daemon_id: daemon_id.clone(),
        },
        timestamp: clock.new_timestamp(),
    })?;
    socket_stream_send(&mut reverse_stream, &reverse_register)
        .await
        .wrap_err("failed to send RegisterNotificationChannel to dora-coordinator")?;

    // Set up tarpc client for daemon→coordinator RPC on the reverse stream
    let reverse_codec = tokio_serde::formats::Json::<
        Response<DaemonToCoordinatorControlResponse>,
        ClientMessage<DaemonToCoordinatorControlRequest>,
    >::default();
    let reverse_transport =
        tarpc::serde_transport::Transport::from((reverse_stream, reverse_codec));
    let coordinator_client =
        DaemonToCoordinatorControlClient::new(client::Config::default(), reverse_transport).spawn();

    tracing::info!("Reverse-channel RPC client established for daemon→coordinator");

    Ok(RegisterResult2 {
        daemon_id,
        rpc_server_handle,
        coordinator_client,
    })
}

/// tarpc server that handles coordinator→daemon RPC calls directly using
/// shared `DaemonState`.
#[derive(Clone)]
struct DaemonRpcServer {
    state: Arc<DaemonState>,
}

impl DaemonControl for DaemonRpcServer {
    async fn build(
        self,
        _ctx: tarpc::context::Context,
        request: BuildDataflowNodes,
    ) -> Result<(), String> {
        let BuildDataflowNodes {
            build_id,
            session_id,
            local_working_dir,
            git_sources,
            prev_git_sources,
            dataflow_descriptor,
            nodes_on_machine,
            uv,
        } = request;

        match dataflow_descriptor.communication.remote {
            dora_core::config::RemoteCommunicationConfig::Tcp => {}
        }

        let base_working_dir =
            crate::Daemon::base_working_dir_static(local_working_dir, session_id)
                .map_err(|err| format!("{err:?}"))?;

        let result = crate::Daemon::build_dataflow_static(
            &self.state,
            build_id,
            session_id,
            base_working_dir,
            git_sources,
            prev_git_sources,
            dataflow_descriptor,
            nodes_on_machine,
            uv,
        )
        .await;
        let (trigger_result, result_task) = match result {
            Ok(result_task) => (Ok(()), Some(result_task)),
            Err(err) => (Err(format!("{err:?}")), None),
        };

        // Spawn background task to report build completion to coordinator
        if let Some(result_task) = result_task {
            let state = self.state.clone();
            tokio::spawn(async move {
                let result = result_task.await;
                let build_result = result
                    .as_ref()
                    .map(|_| ())
                    .map_err(|err| format!("{err:?}"));

                // Store the build info
                if let Ok(info) = result {
                    state.builds.insert(build_id, info);
                }
                state.sessions.insert(session_id, build_id);

                // Report to coordinator
                if let Some(client) = state.coordinator_client() {
                    let ctx = tarpc::context::current();
                    let _ = client.build_result(ctx, build_id, build_result).await;
                }
            });
        }

        trigger_result
    }

    async fn spawn(
        self,
        _ctx: tarpc::context::Context,
        request: SpawnDataflowNodes,
    ) -> Result<(), String> {
        let SpawnDataflowNodes {
            build_id,
            session_id: _,
            dataflow_id,
            local_working_dir,
            nodes,
            dataflow_descriptor,
            spawn_nodes,
            uv,
            write_events_to,
        } = request;

        match dataflow_descriptor.communication.remote {
            dora_core::config::RemoteCommunicationConfig::Tcp => {}
        }

        // For spawn, we still route through the event loop because spawn_dataflow
        // needs mutable access to the logger and complex event loop integration.
        // TODO: Move spawn logic here once logger is refactored.
        let (result_tx, result_rx) = tokio::sync::oneshot::channel();
        let clock = self.state.clock.clone();
        let event = Timestamped {
            inner: Event::SpawnRequest {
                build_id,
                dataflow_id,
                local_working_dir,
                nodes,
                dataflow_descriptor,
                spawn_nodes,
                uv,
                write_events_to,
                reply_tx: result_tx,
            },
            timestamp: clock.new_timestamp(),
        };
        self.state
            .events_tx
            .send(event)
            .await
            .map_err(|_| "daemon event loop closed".to_string())?;

        result_rx
            .await
            .map_err(|_| "daemon dropped spawn reply channel".to_string())?
    }

    async fn all_nodes_ready(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        exited_before_subscribe: Vec<NodeId>,
    ) {
        tracing::debug!(
            "received AllNodesReady (exited_before_subscribe: {exited_before_subscribe:?})"
        );
        // Handle pending nodes while holding the DashMap guard, then drop
        // the guard before doing any further async work.
        let ready = {
            let Some(mut dataflow) = self.state.running.get_mut(&dataflow_id) else {
                tracing::warn!("received AllNodesReady for unknown dataflow (ID `{dataflow_id}`)");
                return;
            };
            let ready = exited_before_subscribe.is_empty();
            let df = &mut *dataflow;
            if let Err(err) = df
                .pending_nodes
                .handle_external_all_nodes_ready(
                    exited_before_subscribe,
                    &mut df.cascading_error_causes,
                )
                .await
            {
                tracing::error!("failed to handle AllNodesReady: {err:?}");
                return;
            }
            ready
        };
        // DashMap guard is now dropped — safe to re-acquire.
        if ready {
            tracing::info!("coordinator reported that all nodes are ready, starting dataflow");
            if let Some(mut dataflow) = self.state.running.get_mut(&dataflow_id) {
                if let Err(err) = dataflow
                    .start(&self.state.events_tx, &self.state.clock)
                    .await
                {
                    tracing::error!("failed to start dataflow: {err:?}");
                }
            }
        }
    }

    async fn stop_dataflow(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<(), String> {
        // Route through the event loop to avoid holding DashMap guards
        // across `.await` points. `stop_all` can trigger RPC calls
        // back to the coordinator (via `report_nodes_ready`), which may
        // call back into this daemon concurrently — deadlocking the
        // DashMap if we held a RefMut here.
        let (result_tx, result_rx) = tokio::sync::oneshot::channel();
        let event = Timestamped {
            inner: Event::StopDataflowRequest {
                dataflow_id,
                grace_duration,
                force,
                reply_tx: result_tx,
            },
            timestamp: self.state.clock.new_timestamp(),
        };
        self.state
            .events_tx
            .send(event)
            .await
            .map_err(|_| "daemon event loop closed".to_string())?;

        result_rx
            .await
            .map_err(|_| "daemon dropped stop reply channel".to_string())?
    }

    async fn reload_dataflow(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<(), String> {
        let mut dataflow =
            self.state.running.get_mut(&dataflow_id).ok_or_else(|| {
                format!("Reload failed: no running dataflow with ID `{dataflow_id}`")
            })?;
        if let Some(channel) = dataflow.subscribe_channels.get(&node_id) {
            match send_with_timestamp(
                channel,
                NodeEvent::Reload { operator_id },
                &self.state.clock,
            ) {
                Ok(()) => {}
                Err(_) => {
                    dataflow.subscribe_channels.remove(&node_id);
                }
            }
        }
        Ok(())
    }

    async fn logs(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        node_id: NodeId,
        tail: Option<usize>,
    ) -> Result<Vec<u8>, String> {
        let working_dir = self
            .state
            .working_dir
            .get(&dataflow_id)
            .map(|entry| entry.clone())
            .ok_or_else(|| format!("no working dir for dataflow `{dataflow_id}`"))?;

        async {
            let mut file =
                tokio::fs::File::open(log::log_path(&working_dir, &dataflow_id, &node_id))
                    .await
                    .map_err(|err| {
                        eyre::eyre!(
                            "Could not open log file: {:#?}: {err}",
                            log::log_path(&working_dir, &dataflow_id, &node_id)
                        )
                    })?;

            let mut contents = match tail {
                None | Some(0) => {
                    let mut contents = vec![];
                    file.read_to_end(&mut contents).await.map(|_| contents)
                }
                Some(tail) => read_last_n_lines(&mut file, tail).await,
            }
            .map_err(|err| eyre::eyre!("Could not read log file: {err}"))?;
            if !contents.ends_with(b"\n") {
                contents.push(b'\n');
            }
            Result::<Vec<u8>, eyre::Report>::Ok(contents)
        }
        .await
        .map_err(|err| format!("{err:?}"))
    }

    async fn destroy(self, _ctx: tarpc::context::Context) -> Result<(), String> {
        tracing::info!("received destroy command -> exiting");
        // Send a Destroy event to the event loop to trigger shutdown
        let event = Timestamped {
            inner: Event::Destroy,
            timestamp: self.state.clock.new_timestamp(),
        };
        let _ = self.state.events_tx.send(event).await;
        Ok(())
    }

    async fn heartbeat(self, _ctx: tarpc::context::Context) {
        *self.state.last_coordinator_heartbeat.lock().await = std::time::Instant::now();
    }

    async fn get_version(
        self,
        _ctx: tarpc::context::Context,
    ) -> dora_message::coordinator_to_daemon::DaemonVersionInfo {
        dora_message::coordinator_to_daemon::DaemonVersionInfo {
            daemon_version: env!("CARGO_PKG_VERSION").to_string(),
            message_format_version: dora_message::VERSION.to_string(),
        }
    }
}
