use std::sync::Arc;

use super::NodeControlServer;
use crate::Event;
use dora_core::uhlc::HLC;
use dora_message::{
    common::Timestamped,
    node_to_daemon::{NodeControl, NodeControlRequest, NodeControlResponse},
    tarpc::{
        self,
        server::{BaseChannel, Channel},
        tokio_serde,
    },
};
use futures::StreamExt;
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};

#[tracing::instrument(skip(listener, daemon_tx, clock), level = "trace")]
pub async fn listener_loop(
    listener: TcpListener,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
) {
    loop {
        match listener.accept().await {
            Err(err) => {
                tracing::info!("failed to accept connection: {err}");
            }
            Ok((connection, _)) => {
                tokio::spawn(handle_connection(
                    connection,
                    daemon_tx.clone(),
                    clock.clone(),
                ));
            }
        }
    }
}

#[tracing::instrument(skip(connection, daemon_tx, clock), level = "trace")]
async fn handle_connection(
    connection: TcpStream,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
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

    let server = NodeControlServer::new(daemon_tx, clock);
    let channel = BaseChannel::with_defaults(transport);

    // Spawn each response handler concurrently so that long-blocking RPCs
    // (next_event, next_finished_drop_tokens) don't prevent other requests.
    channel
        .execute(server.serve())
        .for_each(|response_handler| async {
            tokio::spawn(response_handler);
        })
        .await;
}
