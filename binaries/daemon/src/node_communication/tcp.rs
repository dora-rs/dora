use std::{
    io::ErrorKind,
    sync::{Arc, atomic::AtomicU64},
};

use super::{BackpressureConfig, Connection, Listener};
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

#[tracing::instrument(
    skip(listener, daemon_tx, clock, last_activity, backpressure),
    level = "trace"
)]
#[allow(clippy::too_many_arguments)]
pub async fn listener_loop(
    listener: TcpListener,
    generation: Arc<AtomicU64>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
    last_activity: Arc<AtomicU64>,
    backpressure: Arc<BackpressureConfig>,
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
                            backpressure.clone(),
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

#[tracing::instrument(
    skip(connection, daemon_tx, clock, last_activity, backpressure),
    level = "trace"
)]
async fn handle_connection_loop(
    connection: TcpStream,
    generation: Arc<AtomicU64>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
    last_activity: Arc<AtomicU64>,
    backpressure: Arc<BackpressureConfig>,
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
        backpressure,
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
            Err(err) => {
                // Any error leaves the stream at an unknown offset inside a
                // frame (e.g. an oversized length header, or a body stalled
                // past `TCP_READ_TIMEOUT`), so the next "header" would be old
                // body bytes. Treat it as a disconnect: the node sees EOF
                // instead of waiting forever on a reply.
                if !matches!(
                    err.kind(),
                    ErrorKind::UnexpectedEof
                        | ErrorKind::ConnectionAborted
                        | ErrorKind::ConnectionReset
                ) {
                    tracing::warn!(
                        "closing node connection after I/O error while receiving \
                         DaemonRequest: {err}"
                    );
                }
                return Ok(None);
            }
        };
        // A decode error is different: the whole frame was consumed, so the
        // stream is still aligned and the listener can keep going.
        dora_message::decode(&raw)
            .wrap_err("failed to deserialize DaemonRequest")
            .map(Some)
    }

    async fn send_reply(&mut self, message: DaemonReply) -> eyre::Result<()> {
        if matches!(message, DaemonReply::Empty) {
            // don't send empty replies
            return Ok(());
        }
        let serialized = dora_message::encode_presized(&message, message.encode_size_hint())
            .wrap_err("failed to serialize DaemonReply")?;
        socket_stream_send(&mut self.0, &serialized)
            .await
            .wrap_err("failed to send DaemonReply")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::AsyncWriteExt;

    async fn connected_pair() -> (TcpConnection, TcpStream) {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let (client, server) = tokio::join!(TcpStream::connect(addr), listener.accept());
        (TcpConnection(server.unwrap().0), client.unwrap())
    }

    #[tokio::test]
    async fn oversized_header_closes_the_connection() {
        let (mut conn, mut client) = connected_pair().await;
        let len = dora_message::MAX_MESSAGE_BYTES as u64 + 1;
        client.write_all(&len.to_le_bytes()).await.unwrap();
        // Bytes that would otherwise be misread as the next frame header.
        client.write_all(&[0xAB; 32]).await.unwrap();

        let received = conn.receive_message().await;
        assert!(
            matches!(received, Ok(None)),
            "a frame the listener cannot consume must end the connection, got {received:?}"
        );
    }

    #[tokio::test]
    async fn undecodable_frame_keeps_the_connection() {
        let (mut conn, mut client) = connected_pair().await;
        let body = [0xFF; 16];
        client
            .write_all(&(body.len() as u64).to_le_bytes())
            .await
            .unwrap();
        client.write_all(&body).await.unwrap();

        // The frame was consumed in full, so the stream is still aligned:
        // report the error but don't treat it as a disconnect.
        assert!(conn.receive_message().await.is_err());
    }
}
