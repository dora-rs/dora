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
            // (`node_communication::mod`): answer with an explicit error so the
            // client fails loudly instead of hanging forever, and — like the
            // `NodeConfig` arm above and `node_communication` — keep serving the
            // connection afterwards rather than tearing it down.
            Ok(Some(Timestamped { inner: other, .. })) => {
                tracing::warn!("unsupported request on local listener: {other:?}");
                let reply = DaemonReply::Result(Err(format!(
                    "unsupported request on local listener (client is likely \
                     newer than this daemon): {other:?}"
                )));
                match serde_json::to_vec(&reply).wrap_err("failed to serialize DaemonReply") {
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

    /// A well-formed request this listener does not implement must be answered
    /// with an explicit error, so a request/reply client fails loudly instead
    /// of blocking forever on a reply that never comes (the listener only
    /// implements `NodeConfig`). The connection then keeps serving, so a second
    /// unsupported request is answered too.
    #[tokio::test]
    async fn unsupported_request_gets_error_reply_and_keeps_serving() {
        let (events_tx, _events_rx) = flume::unbounded();
        let port = spawn_listener_loop("127.0.0.1:0".parse().unwrap(), events_tx)
            .await
            .expect("failed to spawn listener")
            .expect("listener should bind to an ephemeral port");

        let mut client = TcpStream::connect(("127.0.0.1", port))
            .await
            .expect("failed to connect to listener");

        let clock = HLC::default();
        // `Subscribe` is a valid, well-formed request the listener does not
        // handle, so it exercises the unsupported-request arm. Send it twice on
        // the same connection: the first proves the error reply, the second
        // proves the connection is still being served afterwards.
        for attempt in 0..2 {
            let request = Timestamped {
                inner: DaemonRequest::Subscribe,
                timestamp: clock.new_timestamp(),
            };
            let encoded = dora_message::encode(&request).expect("failed to encode request");
            socket_stream_send(&mut client, &encoded)
                .await
                .expect("failed to send request");

            // Expect an explicit error reply rather than a hang.
            let raw = socket_stream_receive_with_header_timeout(
                &mut client,
                Some(Duration::from_secs(5)),
            )
            .await
            .unwrap_or_else(|e| {
                panic!("attempt {attempt}: expected an error reply, not a hang: {e}")
            });
            let reply: DaemonReply = serde_json::from_slice(&raw).expect("failed to decode reply");
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
}
