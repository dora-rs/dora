use crate::socket_stream_utils::{socket_stream_receive_with_header_timeout, socket_stream_send};
use dora_message::{
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, DynamicNodeEvent, Timestamped},
};
use eyre::Context;
use std::{io::ErrorKind, net::SocketAddr, time::Duration};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::oneshot,
};

#[derive(Debug)]
pub struct DynamicNodeEventWrapper {
    pub event: DynamicNodeEvent,
    pub reply_tx: oneshot::Sender<Option<DaemonReply>>,
}

pub async fn spawn_listener_loop(
    bind: SocketAddr,
    events_tx: flume::Sender<Timestamped<DynamicNodeEventWrapper>>,
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
        listener_loop(socket, events_tx).await;
    });

    Ok(Some(listen_port))
}

async fn listener_loop(
    listener: TcpListener,
    events_tx: flume::Sender<Timestamped<DynamicNodeEventWrapper>>,
) {
    // Exponential backoff for repeated resource-exhaustion `accept()` failures.
    // A persistent error (e.g. `EMFILE` — too many open files) would otherwise
    // busy-spin the loop at 100% CPU and flood the logs (dora-rs/dora#2027).
    // Per-connection errors (a peer that reset before `accept` completed) are
    // transient and retried immediately, matching the standard accept-loop
    // idiom — only resource exhaustion warrants the pause.
    const ACCEPT_BACKOFF_MAX: Duration = Duration::from_secs(1);
    let mut backoff: Option<Duration> = None;
    loop {
        match listener.accept().await {
            Ok((connection, _)) => {
                backoff = None;
                tokio::spawn(handle_connection_loop(connection, events_tx.clone()));
            }
            Err(err)
                if matches!(
                    err.kind(),
                    ErrorKind::ConnectionAborted
                        | ErrorKind::ConnectionReset
                        | ErrorKind::Interrupted
                ) =>
            {
                // Transient: the offending connection is already gone, so the
                // next `accept` makes progress. Retry without a penalty.
                tracing::warn!("failed to accept new connection: {err}");
                backoff = None;
            }
            Err(err) => {
                let delay = backoff
                    .map_or(Duration::from_millis(10), |d| d * 2)
                    .min(ACCEPT_BACKOFF_MAX);
                backoff = Some(delay);
                tracing::warn!("failed to accept new connection: {err} (retrying in {delay:?})");
                tokio::time::sleep(delay).await;
            }
        }
    }
}

async fn handle_connection_loop(
    mut connection: TcpStream,
    events_tx: flume::Sender<Timestamped<DynamicNodeEventWrapper>>,
) {
    if let Err(err) = connection.set_nodelay(true) {
        tracing::warn!("failed to set nodelay for connection: {err}");
    }

    loop {
        match receive_message(&mut connection).await {
            Ok(Some(Timestamped {
                inner: DaemonRequest::NodeConfig { node_id },
                timestamp,
            })) => {
                let (reply_tx, reply_rx) = oneshot::channel();
                if events_tx
                    .send_async(Timestamped {
                        inner: DynamicNodeEventWrapper {
                            event: DynamicNodeEvent::NodeConfig { node_id },
                            reply_tx,
                        },
                        timestamp,
                    })
                    .await
                    .is_err()
                {
                    break;
                }
                let Ok(reply) = reply_rx.await else {
                    tracing::warn!("daemon sent no reply");
                    continue;
                };
                if let Some(reply) = reply {
                    let serialized = match serde_json::to_vec(&reply)
                        .wrap_err("failed to serialize DaemonReply")
                    {
                        Ok(r) => r,
                        Err(err) => {
                            tracing::error!("{err:?}");
                            continue;
                        }
                    };
                    if let Err(err) = socket_stream_send(&mut connection, &serialized).await {
                        tracing::warn!("failed to send reply: {err}");
                        continue;
                    };
                }
            }
            Ok(None) => break,
            Err(err) => {
                tracing::warn!("{err:?}");
                break;
            }
            // `DaemonRequest` is `#[non_exhaustive]` and this listener only
            // implements `NodeConfig`. A version-skewed or misbehaving client
            // may send some other request and then block on the request/reply
            // it expects. Mirror the node TCP listener
            // (`node_communication::tcp::send_reply`): answer with an explicit
            // error so the client fails loudly instead of hanging forever, and
            // — like the `NodeConfig` arm above and `node_communication` — keep
            // serving the connection afterwards rather than tearing it down.
            //
            // The reply codec is dictated by the *request*, not by this
            // listener: `apis/rust/node/src/daemon_connection/tcp.rs` decodes
            // with Postcard when `expects_tcp_binary_reply()`, with serde_json
            // only for `NodeConfig` (handled above), and reads nothing at all
            // otherwise. So encode with Postcard here — the serde_json used by
            // the `NodeConfig` arm would land as a decode error on the client
            // rather than as the message we built — and stay silent for the
            // no-reply requests (`SendMessage`, `OutputSent`), where an
            // unsolicited frame would be left buffered and desync the next
            // request/reply on this connection.
            Ok(Some(Timestamped { inner: other, .. })) => {
                tracing::warn!("unsupported request on local listener: {other:?}");
                if !other.expects_tcp_binary_reply() {
                    continue;
                }
                let reply = DaemonReply::Result(Err(format!(
                    "unsupported request on local listener (client is likely \
                     newer than this daemon): {other:?}"
                )));
                match dora_message::encode_presized(&reply, reply.encode_size_hint())
                    .wrap_err("failed to serialize DaemonReply")
                {
                    Ok(serialized) => {
                        if let Err(err) = socket_stream_send(&mut connection, &serialized).await {
                            tracing::warn!("failed to send unsupported-request reply: {err}");
                        }
                    }
                    Err(err) => tracing::error!("{err:?}"),
                }
            }
        }
    }
}

async fn receive_message(
    connection: &mut TcpStream,
) -> eyre::Result<Option<Timestamped<DaemonRequest>>> {
    let raw = match socket_stream_receive_with_header_timeout(connection, None).await {
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
    dora_message::decode(&raw)
        .wrap_err("failed to deserialize DaemonRequest")
        .map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_core::uhlc::HLC;
    use dora_message::metadata::Metadata;

    async fn connect_to_listener() -> (
        TcpStream,
        flume::Receiver<Timestamped<DynamicNodeEventWrapper>>,
    ) {
        let (events_tx, events_rx) = flume::unbounded();
        let port = spawn_listener_loop("127.0.0.1:0".parse().unwrap(), events_tx)
            .await
            .expect("failed to spawn listener")
            .expect("listener should bind to an ephemeral port");
        let client = TcpStream::connect(("127.0.0.1", port))
            .await
            .expect("failed to connect to listener");
        (client, events_rx)
    }

    async fn send_request(client: &mut TcpStream, request: DaemonRequest) {
        let clock = HLC::default();
        let request = Timestamped {
            inner: request,
            timestamp: clock.new_timestamp(),
        };
        let encoded = dora_message::encode(&request).expect("failed to encode request");
        socket_stream_send(client, &encoded)
            .await
            .expect("failed to send request");
    }

    /// A well-formed binary-reply request this listener does not implement must
    /// be answered with an explicit error, so a request/reply client fails
    /// loudly instead of blocking forever on a reply that never comes (the
    /// listener only implements `NodeConfig`). The connection then keeps
    /// serving, so a second unsupported request is answered too.
    ///
    /// The reply is decoded with Postcard — the codec the real client picks for
    /// a `Subscribe` request (`expects_tcp_binary_reply()`), per
    /// `apis/rust/node/src/daemon_connection/tcp.rs`. Decoding it as JSON here
    /// would let a serde_json-encoded reply pass the test while the actual
    /// client hit a decode error.
    #[tokio::test]
    async fn unsupported_request_gets_error_reply_and_keeps_serving() {
        let (mut client, _events_rx) = connect_to_listener().await;

        // `Subscribe` is a valid, well-formed request the listener does not
        // handle, so it exercises the unsupported-request arm. Send it twice on
        // the same connection: the first proves the error reply, the second
        // proves the connection is still being served afterwards.
        for attempt in 0..2 {
            assert!(
                DaemonRequest::Subscribe.expects_tcp_binary_reply(),
                "test premise: the real client decodes a Subscribe reply with Postcard"
            );
            send_request(&mut client, DaemonRequest::Subscribe).await;

            // Expect an explicit error reply rather than a hang.
            let raw = socket_stream_receive_with_header_timeout(
                &mut client,
                Some(Duration::from_secs(5)),
            )
            .await
            .unwrap_or_else(|e| {
                panic!("attempt {attempt}: expected an error reply, not a hang: {e}")
            });
            let reply: DaemonReply = dora_message::decode(&raw).unwrap_or_else(|e| {
                panic!("attempt {attempt}: reply must decode with the client's codec: {e:?}")
            });
            match reply {
                DaemonReply::Result(Err(msg)) => {
                    assert!(
                        msg.contains("unsupported request"),
                        "attempt {attempt}: unexpected error message: {msg}"
                    );
                }
                other => {
                    panic!("attempt {attempt}: expected DaemonReply::Result(Err(_)), got {other:?}")
                }
            }
        }
    }

    /// `SendMessage` and `OutputSent` are fire-and-forget: the client reads no
    /// reply at all (neither `expects_tcp_binary_reply` nor
    /// `expects_tcp_json_reply`). Answering them would leave an unsolicited
    /// frame buffered on the connection and desync the *next* request/reply, so
    /// the listener must stay silent — and must still serve the connection.
    #[tokio::test]
    async fn no_reply_requests_are_not_answered() {
        let (mut client, _events_rx) = connect_to_listener().await;

        let no_reply = DaemonRequest::SendMessage {
            output_id: "out".to_string().into(),
            metadata: Metadata::new(HLC::default().new_timestamp()),
            data: None,
        };
        assert!(
            !no_reply.expects_tcp_binary_reply() && !no_reply.expects_tcp_json_reply(),
            "test premise: the real client reads no reply for this request"
        );
        send_request(&mut client, no_reply).await;

        // Nothing may come back. A short timeout is the assertion: the listener
        // sending anything here is the bug.
        let unexpected = socket_stream_receive_with_header_timeout(
            &mut client,
            Some(Duration::from_millis(500)),
        )
        .await;
        assert!(
            unexpected.is_err(),
            "listener must not answer a no-reply request; got {} bytes",
            unexpected.map(|r| r.len()).unwrap_or(0)
        );

        // The connection must still be alive and serving afterwards.
        send_request(&mut client, DaemonRequest::Subscribe).await;
        let raw =
            socket_stream_receive_with_header_timeout(&mut client, Some(Duration::from_secs(5)))
                .await
                .expect("connection must still serve after a silently-ignored request");
        let reply: DaemonReply = dora_message::decode(&raw).expect("failed to decode reply");
        assert!(
            matches!(reply, DaemonReply::Result(Err(ref msg)) if msg.contains("unsupported request")),
            "unexpected reply: {reply:?}"
        );
    }
}
