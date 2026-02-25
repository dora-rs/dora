use crate::{
    DaemonCoordinatorEvent,
    socket_stream_utils::{socket_stream_receive, socket_stream_send},
};
use dora_core::uhlc::HLC;
use dora_message::{
    DataflowId,
    common::{DaemonId, Timestamped},
    coordinator_to_daemon::{
        BuildDataflowNodes, DaemonControl, DaemonControlRequest, DaemonControlResponse,
        RegisterResult, SpawnDataflowNodes,
    },
    daemon_to_coordinator::{CoordinatorRequest, DaemonCoordinatorReply, DaemonRegisterRequest},
    id::{NodeId, OperatorId},
    tarpc::{
        self, ClientMessage, Response,
        server::{BaseChannel, Channel},
        tokio_serde,
    },
};
use eyre::Context;
use futures::StreamExt;
use std::{net::SocketAddr, sync::Arc, time::Duration};
use tokio::{
    net::TcpStream,
    sync::{mpsc, oneshot},
    task::JoinHandle,
    time::sleep,
};
use tracing::warn;

const DAEMON_COORDINATOR_RETRY_INTERVAL: std::time::Duration = Duration::from_secs(1);

#[derive(Debug)]
pub struct CoordinatorEvent {
    pub event: DaemonCoordinatorEvent,
    pub reply_tx: oneshot::Sender<Option<DaemonCoordinatorReply>>,
}

/// Connect to the coordinator, send a `Register` message, and start a tarpc
/// server on the registered TCP stream. The coordinator will call RPC methods
/// on this server.
///
/// Returns the assigned `DaemonId` and a `JoinHandle` for the tarpc server
/// task. Coordinator RPC calls are bridged to the daemon's event loop via
/// `daemon_request_tx`.
pub async fn register(
    addr: SocketAddr,
    machine_id: Option<String>,
    clock: &Arc<HLC>,
    daemon_request_tx: mpsc::Sender<Timestamped<CoordinatorEvent>>,
) -> eyre::Result<(DaemonId, JoinHandle<()>)> {
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

    // --- Registration handshake (raw length-prefixed JSON) ---
    let register = serde_json::to_vec(&Timestamped {
        inner: CoordinatorRequest::Register(DaemonRegisterRequest::new(machine_id)),
        timestamp: clock.new_timestamp(),
    })?;
    socket_stream_send(&mut stream, &register)
        .await
        .wrap_err("failed to send register request to dora-coordinator")?;
    let reply_raw = socket_stream_receive(&mut stream)
        .await
        .wrap_err("failed to register reply from dora-coordinator")?;
    let result: Timestamped<RegisterResult> = serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize dora-coordinator reply")?;
    let daemon_id = result.inner.to_result()?;
    if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
        tracing::warn!("failed to update timestamp after register: {err}");
    }

    tracing::info!("Connected to dora-coordinator at {:?}", addr);

    // --- Set up tarpc server on the registered stream ---
    let codec = tokio_serde::formats::Json::<
        ClientMessage<DaemonControlRequest>,
        Response<DaemonControlResponse>,
    >::default();
    let transport = tarpc::serde_transport::Transport::from((stream, codec));

    let server = DaemonRpcServer {
        request_tx: daemon_request_tx,
        clock: clock.clone(),
    };

    let channel = BaseChannel::with_defaults(transport);
    let handle = tokio::spawn(channel.execute(server.serve()).for_each(|fut| async {
        tokio::spawn(fut);
    }));

    Ok((daemon_id, handle))
}

/// tarpc server implementation that bridges RPC calls to the daemon's event
/// loop via an mpsc channel.
#[derive(Clone)]
struct DaemonRpcServer {
    request_tx: mpsc::Sender<Timestamped<CoordinatorEvent>>,
    clock: Arc<HLC>,
}

impl DaemonRpcServer {
    /// Send a `DaemonCoordinatorEvent` to the daemon event loop and wait for the reply.
    async fn send_event(
        &self,
        event: DaemonCoordinatorEvent,
    ) -> Result<Option<DaemonCoordinatorReply>, String> {
        let (reply_tx, reply_rx) = oneshot::channel();
        let timestamped = Timestamped {
            inner: CoordinatorEvent { event, reply_tx },
            timestamp: self.clock.new_timestamp(),
        };
        self.request_tx
            .send(timestamped)
            .await
            .map_err(|_| "daemon event loop closed".to_string())?;
        reply_rx
            .await
            .map_err(|_| "daemon dropped reply channel".to_string())
    }
}

impl DaemonControl for DaemonRpcServer {
    async fn build(
        self,
        _ctx: tarpc::context::Context,
        request: BuildDataflowNodes,
    ) -> Result<(), String> {
        match self
            .send_event(DaemonCoordinatorEvent::Build(request))
            .await?
        {
            Some(DaemonCoordinatorReply::TriggerBuildResult(r)) => r,
            other => Err(format!("unexpected reply: {other:?}")),
        }
    }

    async fn spawn(
        self,
        _ctx: tarpc::context::Context,
        request: SpawnDataflowNodes,
    ) -> Result<(), String> {
        match self
            .send_event(DaemonCoordinatorEvent::Spawn(request))
            .await?
        {
            Some(DaemonCoordinatorReply::TriggerSpawnResult(r)) => r,
            other => Err(format!("unexpected reply: {other:?}")),
        }
    }

    async fn all_nodes_ready(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        exited_before_subscribe: Vec<NodeId>,
    ) {
        let _ = self
            .send_event(DaemonCoordinatorEvent::AllNodesReady {
                dataflow_id,
                exited_before_subscribe,
            })
            .await;
    }

    async fn stop_dataflow(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<(), String> {
        match self
            .send_event(DaemonCoordinatorEvent::StopDataflow {
                dataflow_id,
                grace_duration,
                force,
            })
            .await?
        {
            Some(DaemonCoordinatorReply::StopResult(r)) => r,
            other => Err(format!("unexpected reply: {other:?}")),
        }
    }

    async fn reload_dataflow(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<(), String> {
        match self
            .send_event(DaemonCoordinatorEvent::ReloadDataflow {
                dataflow_id,
                node_id,
                operator_id,
            })
            .await?
        {
            Some(DaemonCoordinatorReply::ReloadResult(r)) => r,
            other => Err(format!("unexpected reply: {other:?}")),
        }
    }

    async fn logs(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: DataflowId,
        node_id: NodeId,
        tail: Option<usize>,
    ) -> Result<Vec<u8>, String> {
        match self
            .send_event(DaemonCoordinatorEvent::Logs {
                dataflow_id,
                node_id,
                tail,
            })
            .await?
        {
            Some(DaemonCoordinatorReply::Logs(r)) => r,
            other => Err(format!("unexpected reply: {other:?}")),
        }
    }

    async fn destroy(self, _ctx: tarpc::context::Context) -> Result<(), String> {
        match self.send_event(DaemonCoordinatorEvent::Destroy).await? {
            Some(DaemonCoordinatorReply::DestroyResult { result }) => result,
            other => Err(format!("unexpected reply: {other:?}")),
        }
    }

    async fn heartbeat(self, _ctx: tarpc::context::Context) {
        let _ = self.send_event(DaemonCoordinatorEvent::Heartbeat).await;
    }
}
