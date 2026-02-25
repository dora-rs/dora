use crate::{DaemonRequest, DataflowEvent, Event, tcp_utils::tcp_receive};
use dora_core::uhlc::HLC;
use dora_message::{
    common::DaemonId,
    daemon_to_coordinator::{
        CoordinatorRequest, DaemonToCoordinatorControl, DataflowDaemonResult, LogMessage,
        NodeMetrics, Timestamped,
    },
    tarpc,
};
use eyre::Context;
use std::{collections::BTreeMap, io::ErrorKind, net::SocketAddr, sync::Arc};
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
                    machine_id: register_request.machine_id,
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
            CoordinatorRequest::Log {
                daemon_id: _,
                message,
            } => {
                let event = Event::Log(message);
                if events_tx.send(event).await.is_err() {
                    break;
                }
            }
        };
    }
}

/// tarpc server that handles daemon→coordinator RPC calls.
///
/// Each daemon gets its own server instance, identified by `daemon_id`.
/// The server translates incoming RPC calls into coordinator `Event`s.
#[derive(Clone)]
pub struct DaemonEventServer {
    pub daemon_id: DaemonId,
    pub events_tx: mpsc::Sender<Event>,
}

impl DaemonToCoordinatorControl for DaemonEventServer {
    async fn all_nodes_ready(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        exited_before_subscribe: Vec<dora_message::id::NodeId>,
    ) {
        let event = Event::Dataflow {
            uuid: dataflow_id,
            event: DataflowEvent::ReadyOnDaemon {
                daemon_id: self.daemon_id,
                exited_before_subscribe,
            },
        };
        let _ = self.events_tx.send(event).await;
    }

    async fn all_nodes_finished(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        result: DataflowDaemonResult,
    ) {
        let event = Event::Dataflow {
            uuid: dataflow_id,
            event: DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: self.daemon_id,
                result,
            },
        };
        let _ = self.events_tx.send(event).await;
    }

    async fn heartbeat(self, _ctx: tarpc::context::Context) {
        let event = Event::DaemonHeartbeat {
            daemon_id: self.daemon_id,
        };
        let _ = self.events_tx.send(event).await;
    }

    async fn log(self, _ctx: tarpc::context::Context, message: LogMessage) {
        let event = Event::Log(message);
        let _ = self.events_tx.send(event).await;
    }

    async fn daemon_exit(self, _ctx: tarpc::context::Context) {
        let event = Event::DaemonExit {
            daemon_id: self.daemon_id,
        };
        let _ = self.events_tx.send(event).await;
    }

    async fn node_metrics(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        metrics: BTreeMap<dora_message::id::NodeId, NodeMetrics>,
    ) {
        let event = Event::NodeMetrics {
            dataflow_id,
            metrics,
        };
        let _ = self.events_tx.send(event).await;
    }

    async fn build_result(
        self,
        _ctx: tarpc::context::Context,
        build_id: dora_message::BuildId,
        result: Result<(), String>,
    ) {
        let event = Event::DataflowBuildResult {
            build_id,
            daemon_id: self.daemon_id,
            result: result.map_err(|err| eyre::eyre!(err)),
        };
        let _ = self.events_tx.send(event).await;
    }

    async fn spawn_result(
        self,
        _ctx: tarpc::context::Context,
        dataflow_id: Uuid,
        result: Result<(), String>,
    ) {
        let event = Event::DataflowSpawnResult {
            dataflow_id,
            daemon_id: self.daemon_id,
            result: result.map_err(|err| eyre::eyre!(err)),
        };
        let _ = self.events_tx.send(event).await;
    }
}
