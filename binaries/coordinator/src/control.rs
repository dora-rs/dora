use crate::{
    tcp_utils::{tcp_receive, tcp_send},
    Event,
};
use dora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, BuildId,
};
use eyre::{eyre, Context};
use futures::{
    future::{self, Either},
    stream::FuturesUnordered,
    FutureExt, Stream, StreamExt,
};
use futures_concurrency::future::Race;
use std::{io::ErrorKind, net::SocketAddr};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::{mpsc, oneshot},
    task::JoinHandle,
};
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;

pub(crate) async fn control_events(
    control_listen_addr: SocketAddr,
    tasks: &FuturesUnordered<JoinHandle<()>>,
) -> eyre::Result<impl Stream<Item = Event> + use<>> {
    let (tx, rx) = mpsc::channel(10);

    let (finish_tx, mut finish_rx) = mpsc::channel(1);
    tasks.push(tokio::spawn(listen(control_listen_addr, tx, finish_tx)));
    tasks.push(tokio::spawn(async move {
        while let Some(()) = finish_rx.recv().await {}
    }));

    Ok(ReceiverStream::new(rx).map(Event::Control))
}

async fn listen(
    control_listen_addr: SocketAddr,
    tx: mpsc::Sender<ControlEvent>,
    _finish_tx: mpsc::Sender<()>,
) {
    let result = TcpListener::bind(control_listen_addr)
        .await
        .wrap_err("failed to listen for control messages");
    let incoming = match result {
        Ok(incoming) => incoming,
        Err(err) => {
            let _ = tx.send(err.into()).await;
            return;
        }
    };

    loop {
        let new_connection = incoming.accept().map(Either::Left);
        let coordinator_stop = tx.closed().map(Either::Right);
        let connection = match (new_connection, coordinator_stop).race().await {
            future::Either::Left(connection) => connection,
            future::Either::Right(()) => {
                // coordinator was stopped
                break;
            }
        };
        match connection.wrap_err("failed to connect") {
            Ok((connection, _)) => {
                let tx = tx.clone();
                tokio::spawn(handle_requests(connection, tx, _finish_tx.clone()));
            }
            Err(err) => {
                if tx.blocking_send(err.into()).is_err() {
                    break;
                }
            }
        }
    }
}

async fn handle_requests(
    mut connection: TcpStream,
    tx: mpsc::Sender<ControlEvent>,
    _finish_tx: mpsc::Sender<()>,
) {
    let peer_addr = connection.peer_addr().ok();
    loop {
        let next_request = tcp_receive(&mut connection).map(Either::Left);
        let coordinator_stopped = tx.closed().map(Either::Right);
        let raw = match (next_request, coordinator_stopped).race().await {
            Either::Right(()) => break,
            Either::Left(request) => match request {
                Ok(message) => message,
                Err(err) => match err.kind() {
                    ErrorKind::UnexpectedEof => {
                        tracing::trace!("Control connection closed");
                        break;
                    }
                    err => {
                        let err = eyre!(err).wrap_err("failed to receive incoming message");
                        tracing::error!("{err}");
                        break;
                    }
                },
            },
        };

        let request =
            serde_json::from_slice(&raw).wrap_err("failed to deserialize incoming message");

        if let Ok(ControlRequest::LogSubscribe { dataflow_id, level }) = request {
            let _ = tx
                .send(ControlEvent::LogSubscribe {
                    dataflow_id,
                    level,
                    connection,
                })
                .await;
            break;
        }

        if let Ok(ControlRequest::BuildLogSubscribe { build_id, level }) = request {
            let _ = tx
                .send(ControlEvent::BuildLogSubscribe {
                    build_id,
                    level,
                    connection,
                })
                .await;
            break;
        }

        let mut result = match request {
            Ok(request) => handle_request(request, &tx).await,
            Err(err) => Err(err),
        };

        if let Ok(ControlRequestReply::CliAndDefaultDaemonIps { cli, .. }) = &mut result {
            if cli.is_none() {
                // fill cli IP address in reply
                *cli = peer_addr.map(|s| s.ip());
            }
        }

        let reply = result.unwrap_or_else(|err| ControlRequestReply::Error(format!("{err:?}")));
        let serialized: Vec<u8> =
            match serde_json::to_vec(&reply).wrap_err("failed to serialize ControlRequestReply") {
                Ok(s) => s,
                Err(err) => {
                    tracing::error!("{err:?}");
                    break;
                }
            };
        match tcp_send(&mut connection, &serialized).await {
            Ok(()) => {}
            Err(err) => match err.kind() {
                ErrorKind::UnexpectedEof => {
                    tracing::debug!("Control connection closed while trying to send reply");
                    break;
                }
                err => {
                    let err = eyre!(err).wrap_err("failed to send reply");
                    tracing::error!("{err}");
                    break;
                }
            },
        }

        if matches!(reply, ControlRequestReply::CoordinatorStopped) {
            break;
        }
    }
}

async fn handle_request(
    request: ControlRequest,
    tx: &mpsc::Sender<ControlEvent>,
) -> eyre::Result<ControlRequestReply> {
    let (reply_tx, reply_rx) = oneshot::channel();
    let event = ControlEvent::IncomingRequest {
        request: request.clone(),
        reply_sender: reply_tx,
    };

    if tx.send(event).await.is_err() {
        return Ok(ControlRequestReply::CoordinatorStopped);
    }

    reply_rx
        .await
        .wrap_err_with(|| format!("no coordinator reply to {request:?}"))?
}

#[derive(Debug)]
pub enum ControlEvent {
    IncomingRequest {
        request: ControlRequest,
        reply_sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
    },
    LogSubscribe {
        dataflow_id: Uuid,
        level: log::LevelFilter,
        connection: TcpStream,
    },
    BuildLogSubscribe {
        build_id: BuildId,
        level: log::LevelFilter,
        connection: TcpStream,
    },
    Error(eyre::Report),
}

impl From<eyre::Report> for ControlEvent {
    fn from(err: eyre::Report) -> Self {
        ControlEvent::Error(err)
    }
}
