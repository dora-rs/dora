use super::Listener;
use crate::{
    shared_mem_handler,
    tcp_utils::{tcp_receive, tcp_send},
    Event,
};
use dora_core::daemon_messages::{DaemonReply, DaemonRequest};
use eyre::Context;
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};

#[tracing::instrument(skip(listener, daemon_tx, shmem_handler_tx))]
pub async fn listener_loop(
    listener: TcpListener,
    daemon_tx: mpsc::Sender<Event>,
    shmem_handler_tx: flume::Sender<shared_mem_handler::NodeEvent>,
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
                tokio::spawn(handle_connection_loop(
                    connection,
                    daemon_tx.clone(),
                    shmem_handler_tx.clone(),
                ));
            }
        }
    }
}

#[tracing::instrument(skip(connection, daemon_tx, shmem_handler_tx))]
async fn handle_connection_loop(
    connection: TcpStream,
    daemon_tx: mpsc::Sender<Event>,
    shmem_handler_tx: flume::Sender<shared_mem_handler::NodeEvent>,
) {
    if let Err(err) = connection.set_nodelay(true) {
        tracing::warn!("failed to set nodelay for connection: {err}");
    }

    Listener::run(TcpConnection(connection), daemon_tx, shmem_handler_tx).await
}

struct TcpConnection(TcpStream);

#[async_trait::async_trait]
impl super::Connection for TcpConnection {
    async fn receive_message(&mut self) -> eyre::Result<Option<DaemonRequest>> {
        let raw = match tcp_receive(&mut self.0).await {
            Ok(raw) => raw,
            Err(err) => match err.kind() {
                std::io::ErrorKind::UnexpectedEof | std::io::ErrorKind::ConnectionAborted => {
                    return Ok(None)
                }
                _other => {
                    return Err(err)
                        .context("unexpected I/O error while trying to receive DaemonRequest")
                }
            },
        };
        bincode::deserialize(&raw)
            .wrap_err("failed to deserialize DaemonRequest")
            .map(Some)
    }

    async fn send_reply(&mut self, message: DaemonReply) -> eyre::Result<()> {
        let serialized =
            bincode::serialize(&message).wrap_err("failed to serialize DaemonReply")?;
        tcp_send(&mut self.0, &serialized)
            .await
            .wrap_err("failed to send DaemonReply")?;
        Ok(())
    }
}
