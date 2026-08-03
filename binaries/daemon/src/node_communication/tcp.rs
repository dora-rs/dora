use std::{
    io::ErrorKind,
    sync::{Arc, atomic::AtomicU64},
};

use super::{Connection, Listener};
use crate::{
    Event,
    socket_stream_utils::{socket_stream_receive_with_header_timeout, socket_stream_send},
};
use dora_core::uhlc::HLC;
use dora_message::{
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
    generation: Arc<AtomicU64>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
    last_activity: Arc<AtomicU64>,
    mut shutdown: tokio::sync::watch::Receiver<bool>,
    mut node_shutdown: tokio::sync::watch::Receiver<bool>,
) {
    loop {
        tokio::select! {
            result = listener.accept() => {
                match result.wrap_err("failed to accept new connection") {
                    Err(err) => tracing::warn!("{err}"),
                    Ok((connection, _)) => {
                        tokio::spawn(handle_connection_loop(
                            connection,
                            generation.clone(),
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
            // Per-node lifetime (dora-rs/dora#2988 review, finding 3): the
            // senders live in the node's RunningNode entry and its restart
            // loop, so removing the node (replace/remove/teardown) closes
            // this listener instead of leaking it until the whole dataflow
            // finishes. `changed()` errors when the last sender drops —
            // either way, stop accepting. Existing connections are
            // unaffected (their tasks run independently and are
            // generation-gated).
            result = node_shutdown.changed() => {
                let _ = result;
                tracing::trace!("TCP listener shutting down (node retired)");
                break;
            }
        }
    }
}

#[tracing::instrument(skip(connection, daemon_tx, clock, last_activity), level = "trace")]
async fn handle_connection_loop(
    connection: TcpStream,
    generation: Arc<AtomicU64>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
    last_activity: Arc<AtomicU64>,
) {
    if let Err(err) = connection.set_nodelay(true) {
        tracing::warn!("failed to set nodelay for connection: {err}");
    }

    Listener::run(
        TcpConnection(connection),
        generation,
        daemon_tx,
        clock,
        last_activity,
    )
    .await
}

struct TcpConnection(TcpStream);

impl Connection for TcpConnection {
    async fn receive_message(&mut self) -> eyre::Result<Option<Timestamped<DaemonRequest>>> {
        // No header timeout: a node connection may legitimately stay idle
        // between requests, so only mid-frame (body) stalls are faults.
        let raw = match socket_stream_receive_with_header_timeout(&mut self.0, None).await {
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
