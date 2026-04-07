use dora_core::{
    config::{DataId, NodeId},
    uhlc::HLC,
};
use dora_message::{
    common::Timestamped,
    daemon_to_node::{NodeConfig, NodeDropEvent, NodeEvent},
    metadata::Metadata,
    node_to_daemon::{
        DataMessage, DropToken, DynamicNodeEvent, NodeControl, NodeControlRequest,
        NodeControlResponse, NodeRegisterRequest,
    },
    tarpc::{
        self,
        server::{BaseChannel, Channel},
        tokio_serde,
    },
};
use eyre::Context;
use futures::StreamExt;
use std::{io::ErrorKind, net::SocketAddr, sync::Arc};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::oneshot,
};

#[derive(Debug)]
pub struct DynamicNodeEventWrapper {
    pub event: DynamicNodeEvent,
    pub reply_tx: oneshot::Sender<Result<NodeConfig, String>>,
}

pub async fn spawn_listener_loop(
    bind: SocketAddr,
    events_tx: flume::Sender<Timestamped<DynamicNodeEventWrapper>>,
    clock: Arc<HLC>,
) -> eyre::Result<Option<u16>> {
    let socket = match TcpListener::bind(bind).await {
        Ok(socket) => socket,
        Err(err) if err.kind() == ErrorKind::AddrInUse => {
            tracing::warn!(
                "Daemon listen port already in use. There might be another daemon running already."
            );
            return Ok(None);
        }
        Err(err) => {
            let kind = err.kind();
            return Err(eyre::Report::new(err)
                .wrap_err(format!("failed to create local TCP listener ({kind:?})")));
        }
    };
    let listen_port = socket
        .local_addr()
        .wrap_err("failed to get local addr of socket")?
        .port();

    tokio::spawn(async move {
        listener_loop(socket, events_tx, clock).await;
    });

    Ok(Some(listen_port))
}

async fn listener_loop(
    listener: TcpListener,
    events_tx: flume::Sender<Timestamped<DynamicNodeEventWrapper>>,
    clock: Arc<HLC>,
) {
    loop {
        match listener
            .accept()
            .await
            .wrap_err("failed to accept new connection")
        {
            Err(err) => {
                tracing::info!("{err}");
            }
            Ok((connection, _)) => {
                tokio::spawn(handle_connection(
                    connection,
                    events_tx.clone(),
                    clock.clone(),
                ));
            }
        }
    }
}

async fn handle_connection(
    connection: TcpStream,
    events_tx: flume::Sender<Timestamped<DynamicNodeEventWrapper>>,
    clock: Arc<HLC>,
) {
    if let Err(err) = connection.set_nodelay(true) {
        tracing::warn!("failed to set nodelay for connection: {err}");
    }

    let codec = tokio_serde::formats::Json::<
        tarpc::ClientMessage<NodeControlRequest>,
        tarpc::Response<NodeControlResponse>,
    >::default();
    let transport = tarpc::serde_transport::Transport::from((connection, codec));

    let server = MainListenerServer { events_tx, clock };
    let channel = BaseChannel::with_defaults(transport);

    channel
        .execute(server.serve())
        .for_each(|response_handler| async {
            tokio::spawn(response_handler);
        })
        .await;
}

/// tarpc server for the daemon's main listener port.
///
/// Only handles `node_config` requests (for dynamic node initialization).
/// All other RPC methods return errors since they belong on per-node channels.
#[derive(Clone)]
struct MainListenerServer {
    events_tx: flume::Sender<Timestamped<DynamicNodeEventWrapper>>,
    clock: Arc<HLC>,
}

impl NodeControl for MainListenerServer {
    async fn register(
        self,
        _context: tarpc::context::Context,
        _request: NodeRegisterRequest,
    ) -> Result<(), String> {
        Err("register is not supported on the main daemon listener".to_string())
    }

    async fn subscribe(self, _context: tarpc::context::Context) -> Result<(), String> {
        Err("subscribe is not supported on the main daemon listener".to_string())
    }

    async fn subscribe_drop(self, _context: tarpc::context::Context) -> Result<(), String> {
        Err("subscribe_drop is not supported on the main daemon listener".to_string())
    }

    async fn next_event(
        self,
        _context: tarpc::context::Context,
        _drop_tokens: Vec<DropToken>,
    ) -> Vec<Timestamped<NodeEvent>> {
        tracing::warn!("next_event is not supported on the main daemon listener");
        vec![]
    }

    async fn next_finished_drop_tokens(
        self,
        _context: tarpc::context::Context,
    ) -> Vec<Timestamped<NodeDropEvent>> {
        tracing::warn!("next_finished_drop_tokens is not supported on the main daemon listener");
        vec![]
    }

    async fn send_message(
        self,
        _context: tarpc::context::Context,
        _output_id: DataId,
        _metadata: Metadata,
        _data: Option<DataMessage>,
    ) {
        tracing::warn!("send_message is not supported on the main daemon listener");
    }

    async fn close_outputs(
        self,
        _context: tarpc::context::Context,
        _outputs: Vec<DataId>,
    ) -> Result<(), String> {
        Err("close_outputs is not supported on the main daemon listener".to_string())
    }

    async fn outputs_done(self, _context: tarpc::context::Context) -> Result<(), String> {
        Err("outputs_done is not supported on the main daemon listener".to_string())
    }

    async fn report_drop_tokens(
        self,
        _context: tarpc::context::Context,
        _drop_tokens: Vec<DropToken>,
    ) {
        tracing::warn!("report_drop_tokens is not supported on the main daemon listener");
    }

    async fn event_stream_dropped(self, _context: tarpc::context::Context) -> Result<(), String> {
        Err("event_stream_dropped is not supported on the main daemon listener".to_string())
    }

    async fn node_config(
        self,
        _context: tarpc::context::Context,
        node_id: NodeId,
    ) -> Result<NodeConfig, String> {
        let (reply_tx, reply_rx) = oneshot::channel();
        let timestamp = self.clock.new_timestamp();

        self.events_tx
            .send_async(Timestamped {
                inner: DynamicNodeEventWrapper {
                    event: DynamicNodeEvent::NodeConfig {
                        node_id: node_id.clone(),
                    },
                    reply_tx,
                },
                timestamp,
            })
            .await
            .map_err(|_| format!("failed to send node_config request for `{node_id}` to daemon"))?;

        reply_rx
            .await
            .map_err(|_| "daemon did not reply to node_config request".to_string())?
    }
}
