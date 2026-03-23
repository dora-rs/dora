use std::{
    io::ErrorKind,
    sync::{Arc, atomic::AtomicU64},
};

use super::{Connection, Listener};
use crate::{
    Event,
    socket_stream_utils::{socket_stream_receive, socket_stream_send},
};
use adora_core::uhlc::HLC;
use adora_message::{
    common::Timestamped, daemon_to_node::DaemonReply, node_to_daemon::DaemonRequest,
};
use eyre::Context;
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};

#[tracing::instrument(skip(listener, daemon_tx, clock, last_activity), level = "trace")]
pub async fn listener_loop(
    listener: TcpListener,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
    last_activity: Arc<AtomicU64>,
    mut shutdown: tokio::sync::watch::Receiver<bool>,
) {
    loop {
        tokio::select! {
            result = listener.accept() => {
                match result.wrap_err("failed to accept new connection") {
                    Err(err) => tracing::warn!("{err}"),
                    Ok((connection, _)) => {
                        tokio::spawn(handle_connection_loop(
                            connection,
                            daemon_tx.clone(),
                            clock.clone(),
                            last_activity.clone(),
                        ));
                    }
                }
            }
            _ = shutdown.changed() => {
                tracing::trace!("TCP listener shutting down");
                break;
            }
        }
    }
}

#[tracing::instrument(skip(connection, daemon_tx, clock, last_activity), level = "trace")]
async fn handle_connection_loop(
    connection: TcpStream,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
    last_activity: Arc<AtomicU64>,
) {
    if let Err(err) = connection.set_nodelay(true) {
        tracing::warn!("failed to set nodelay for connection: {err}");
    }

    Listener::run(TcpConnection(connection), daemon_tx, clock, last_activity).await
}

struct TcpConnection(TcpStream);

impl Connection for TcpConnection {
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
            // don't send empty replies
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
