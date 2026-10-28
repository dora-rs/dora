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
        loop {
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
            match dora_message::decode(&raw) {
                Ok(message) => return Ok(Some(message)),
                Err(err) => {
                    // The whole frame was consumed, so the stream is still
                    // aligned. A node built against the same `dora-message`
                    // never sends an undecodable frame: this is a newer node
                    // sending a `DaemonRequest` variant this daemon predates.
                    // Every such variant expects a binary reply (the rule on
                    // `DaemonRequest`), and the node blocks on it with no
                    // timeout, so answer with an error and keep serving the
                    // connection. Skipping the frame would hang the node.
                    tracing::warn!(
                        "failed to deserialize DaemonRequest (is the node newer than \
                         this daemon?): {err}"
                    );
                    let reply = DaemonReply::Result(Err(format!(
                        "unsupported request: this daemon (dora {}) cannot decode it; \
                         the node is likely newer than the daemon",
                        dora_message::current_crate_version()
                    )));
                    if let Err(err) = self.send_reply(reply).await {
                        tracing::warn!("closing node connection: {err:?}");
                        return Ok(None);
                    }
                }
            }
        }
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
    async fn undecodable_frame_gets_an_error_reply_and_keeps_the_connection() {
        use tokio::io::AsyncReadExt;

        let (mut conn, mut client) = connected_pair().await;
        // Typically a request variant from a newer node.
        let body = [0xFF; 16];
        client
            .write_all(&(body.len() as u64).to_le_bytes())
            .await
            .unwrap();
        client.write_all(&body).await.unwrap();
        // A valid request right behind it, which must still be served.
        let valid = dora_message::encode(&Timestamped {
            inner: DaemonRequest::Subscribe,
            timestamp: HLC::default().new_timestamp(),
        })
        .unwrap();
        client
            .write_all(&(valid.len() as u64).to_le_bytes())
            .await
            .unwrap();
        client.write_all(&valid).await.unwrap();

        let received = conn.receive_message().await.unwrap();
        assert!(
            matches!(
                received.as_ref().map(|m| &m.inner),
                Some(DaemonRequest::Subscribe)
            ),
            "the connection must stay usable after an undecodable request, got {received:?}"
        );

        // The node, blocked on its reply, gets an error instead of a hang.
        let mut len = [0; 8];
        client.read_exact(&mut len).await.unwrap();
        let mut reply = vec![0; u64::from_le_bytes(len) as usize];
        client.read_exact(&mut reply).await.unwrap();
        let reply: DaemonReply = dora_message::decode(&reply).unwrap();
        assert!(
            matches!(&reply, DaemonReply::Result(Err(msg)) if msg.contains("unsupported request")),
            "expected an error reply, got {reply:?}"
        );
    }
}
