use std::io::ErrorKind;

use super::{Connection, Listener};
use crate::{
    Event,
    socket_stream_utils::{socket_stream_receive, socket_stream_send},
};
use dora_message::{
    common::Timestamped, daemon_to_node::DaemonReply, node_to_daemon::DaemonRequest,
};
use eyre::Context;
use std::sync::Arc;
use tokio::{
    net::{UnixListener, UnixStream},
    sync::mpsc,
};

#[tracing::instrument(skip(listener, daemon_tx, clock), level = "trace")]
pub async fn listener_loop(
    listener: UnixListener,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<dora_core::uhlc::HLC>,
) {
    loop {
        match listener
            .accept()
            .await
            .wrap_err("failed to accept new Unix connection")
        {
            Err(err) => {
                tracing::info!("{err}");
            }
            Ok((connection, _)) => {
                tokio::spawn(handle_connection_loop(
                    connection,
                    daemon_tx.clone(),
                    clock.clone(),
                ));
            }
        }
    }
}

#[tracing::instrument(skip(connection, daemon_tx, clock), level = "trace")]
async fn handle_connection_loop(
    connection: UnixStream,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<dora_core::uhlc::HLC>,
) {
    Listener::run(UnixConnection(connection), daemon_tx, clock).await
}

struct UnixConnection(UnixStream);

#[async_trait::async_trait]
impl Connection for UnixConnection {
    async fn receive_message(&mut self) -> eyre::Result<Option<Timestamped<DaemonRequest>>> {
        let raw = match socket_stream_receive(&mut self.0).await {
            Ok(raw) => raw,
            Err(err) => match err.kind() {
                ErrorKind::UnexpectedEof
                | ErrorKind::ConnectionAborted
                | ErrorKind::ConnectionReset => return Ok(None),
                _other => {
                    return Err(err)
                        .context("unexpected I/O error while trying to receive DaemonRequest");
                }
            },
        };
        bincode::deserialize(&raw)
            .wrap_err("failed to deserialize DaemonRequest")
            .map(Some)
    }

    async fn send_reply(&mut self, message: DaemonReply) -> eyre::Result<()> {
        if matches!(message, DaemonReply::Empty) {
            return Ok(());
        }
        let serialized =
            bincode::serialize(&message).wrap_err("failed to serialize DaemonReply")?;
        socket_stream_send(&mut self.0, &serialized)
            .await
            .wrap_err("failed to send DaemonReply")?;
        Ok(())
    }
}
