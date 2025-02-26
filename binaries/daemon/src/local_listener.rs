use crate::socket_stream_utils::{socket_stream_receive, socket_stream_send};
use dora_message::{
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, DynamicNodeEvent, Timestamped},
};
use eyre::Context;
use std::{io::ErrorKind, net::SocketAddr};
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
                tokio::spawn(handle_connection_loop(connection, events_tx.clone()));
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
            _ => tracing::warn!(
                "Unexpected Daemon Request that is not yet by Additional local listener controls"
            ),
        }
    }
}

async fn receive_message(
    connection: &mut TcpStream,
) -> eyre::Result<Option<Timestamped<DaemonRequest>>> {
    let raw = match socket_stream_receive(connection).await {
        Ok(raw) => raw,
        Err(err) => match err.kind() {
            ErrorKind::UnexpectedEof
            | ErrorKind::ConnectionAborted
            | ErrorKind::ConnectionReset => return Ok(None),
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
