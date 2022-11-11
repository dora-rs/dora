use crate::Event;
use communication_layer_request_reply::{ListenConnection, RequestReplyLayer, TcpLayer};
use dora_core::topics::ControlRequest;
use eyre::Context;
use futures::{Stream, StreamExt};
use std::{
    io::{self, ErrorKind},
    net::SocketAddr,
};
use tokio::sync::{mpsc, oneshot};
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;

pub(crate) async fn control_events(
    control_listen_addr: SocketAddr,
) -> eyre::Result<impl Stream<Item = Event>> {
    let (tx, rx) = mpsc::channel(10);

    tokio::task::spawn_blocking(move || listen(control_listen_addr, tx));

    Ok(ReceiverStream::new(rx).map(Event::Control))
}

fn listen(control_listen_addr: SocketAddr, tx: mpsc::Sender<ControlEvent>) {
    let mut com_layer = TcpLayer::new();
    let result = com_layer
        .listen(control_listen_addr)
        .wrap_err("failed to listen for control messages");
    let incoming = match result {
        Ok(incoming) => incoming,
        Err(err) => {
            let _ = tx.blocking_send(err.into());
            return;
        }
    };

    for connection in incoming {
        match connection.wrap_err("failed to connect") {
            Ok(connection) => {
                let tx = tx.clone();
                tokio::task::spawn_blocking(|| handle_requests(connection, tx));
            }
            Err(err) => {
                if tx.blocking_send(err.into()).is_err() {
                    break;
                }
            }
        }
    }
}

fn handle_requests(
    mut connection: Box<
        dyn ListenConnection<RequestData = Vec<u8>, ReplyData = Vec<u8>, Error = std::io::Error>,
    >,
    tx: mpsc::Sender<ControlEvent>,
) {
    loop {
        let tx = tx.clone();
        let result = connection.handle_next(Box::new(move |raw| {
            let (reply, reply_rx) = oneshot::channel();
            let request = match serde_json::from_slice(&raw) {
                Ok(request) => ControlEvent::IncomingRequest {
                    request,
                    reply_sender: reply,
                },
                Err(err) => return Err(io::Error::new(ErrorKind::Other, HandlerError::from(err))),
            };
            if tx.blocking_send(request).is_err() {
                return Err(io::Error::new(
                    io::ErrorKind::Other,
                    HandlerError::ServerStopped,
                ));
            }

            let Ok(reply) = reply_rx.blocking_recv() else {
                return Err(io::Error::new(
                    io::ErrorKind::Other,
                    HandlerError::ServerStopped,
                ));
            };
            Ok(reply)
        }));
        if let Err(err) = result {
            if err.kind() == ErrorKind::Other {
                let inner = err.into_inner().unwrap();
                let downcasted = inner.downcast_ref().unwrap();
                match downcasted {
                    HandlerError::ParseError(err) => {
                        tracing::warn!("failed to parse request: {err}");
                    }
                    HandlerError::ServerStopped => break,
                }
            }
        }
    }
}

#[derive(Debug, thiserror::Error)]
enum HandlerError {
    #[error("failed to parse request")]
    ParseError(#[from] serde_json::Error),
    #[error("server was stopped already")]
    ServerStopped,
}

pub enum ControlEvent {
    IncomingRequest {
        request: ControlRequest,
        reply_sender: oneshot::Sender<Vec<u8>>,
    },
    Error(eyre::Report),
}

impl From<eyre::Report> for ControlEvent {
    fn from(err: eyre::Report) -> Self {
        ControlEvent::Error(err)
    }
}
