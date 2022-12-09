use crate::{
    tcp_utils::{tcp_receive, tcp_send},
    DaemonNodeEvent, Event,
};
use dora_core::daemon_messages;
use eyre::{eyre, Context};
use std::{io::ErrorKind, net::Ipv4Addr};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::{mpsc, oneshot},
};
use tokio_stream::StreamExt;

pub async fn create_listener() -> eyre::Result<TcpListener> {
    let localhost = Ipv4Addr::new(127, 0, 0, 1);
    let socket = match TcpListener::bind((localhost, 0)).await {
        Ok(socket) => socket,
        Err(err) => {
            return Err(eyre::Report::new(err).wrap_err("failed to create local TCP listener"))
        }
    };
    Ok(socket)
}

pub async fn handle_connection(mut connection: TcpStream, events_tx: mpsc::Sender<Event>) {
    let mut id = None;
    let mut enter_subscribe_loop = None;
    loop {
        // receive the next message and parse it
        let raw = match tcp_receive(&mut connection).await {
            Ok(data) => data,
            Err(err) if err.kind() == ErrorKind::UnexpectedEof => {
                break;
            }
            Err(err) => {
                tracing::error!("{err:?}");
                continue;
            }
        };
        let message: daemon_messages::ControlRequest =
            match serde_json::from_slice(&raw).wrap_err("failed to deserialize node message") {
                Ok(e) => e,
                Err(err) => {
                    tracing::warn!("{err:?}");
                    continue;
                }
            };

        // handle the message and translate it to a NodeEvent
        let node_event = match message {
            daemon_messages::ControlRequest::Register { node_id } => {
                id = Some(node_id);

                let reply = daemon_messages::ControlReply::Result(Ok(()));
                let serialized = serde_json::to_vec(&reply)
                    .wrap_err("failed to serialize register result")
                    .unwrap();

                match tcp_send(&mut connection, &serialized).await {
                    Ok(()) => continue, // don't trigger an event for register calls
                    Err(err) => {
                        tracing::warn!("{err:?}");
                        break; // close connection
                    }
                }
            }
            daemon_messages::ControlRequest::Stopped => DaemonNodeEvent::Stopped,
            daemon_messages::ControlRequest::PrepareOutputMessage { output_id, len } => {
                DaemonNodeEvent::PrepareOutputMessage { output_id, len }
            }
            daemon_messages::ControlRequest::SendOutMessage { id } => {
                DaemonNodeEvent::SendOutMessage { id }
            }
            daemon_messages::ControlRequest::Subscribe { node_id } => {
                let (tx, rx) = flume::bounded(10);

                id = Some(node_id);
                enter_subscribe_loop = Some(rx);

                DaemonNodeEvent::Subscribe { event_sender: tx }
            }
        };

        // send NodeEvent to daemon main loop
        let (reply_tx, reply) = oneshot::channel();
        let event = Event::Node {
            id: match &id {
                Some(id) => id.clone(),
                None => {
                    tracing::warn!(
                        "Ignoring node event because no register \
                        message was sent yet: {node_event:?}"
                    );
                    continue;
                }
            },
            event: node_event,
            reply_sender: reply_tx,
        };
        let Ok(()) = events_tx.send(event).await else {
            break;
        };

        // wait for reply and send it out
        let Ok(reply) = reply.await else {
            break; // main loop exited
        };
        let Ok(serialized) = serde_json::to_vec(&reply) else {
            tracing::error!("failed to serialize reply");
            continue;
        };
        match tcp_send(&mut connection, &serialized).await {
            Ok(()) => {}
            Err(err) if err.kind() == ErrorKind::UnexpectedEof => {
                break;
            }
            Err(err) => {
                tracing::error!("{err:?}");
            }
        }

        // enter subscribe loop after receiving a subscribe message
        if let Some(events) = enter_subscribe_loop {
            subscribe_loop(connection, events).await;
            break; // the subscribe loop only exits when the connection was closed
        }
    }
}

async fn subscribe_loop(
    mut connection: TcpStream,
    events: flume::Receiver<daemon_messages::NodeEvent>,
) {
    while let Some(event) = events.stream().next().await {
        let message = match serde_json::to_vec(&event) {
            Ok(m) => m,
            Err(err) => {
                let err = eyre!(err).wrap_err("failed to serialize node event");
                tracing::warn!("{err:?}");
                continue;
            }
        };
        match tcp_send(&mut connection, &message).await {
            Ok(()) => {}
            Err(err) if err.kind() == ErrorKind::UnexpectedEof => {
                break;
            }
            Err(err) => {
                tracing::error!("{err:?}");
            }
        }
    }
}
