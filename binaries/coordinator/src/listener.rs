use crate::{tcp_utils::tcp_receive, DaemonEvent, DataflowEvent, Event};
use dora_core::coordinator_messages;
use eyre::{eyre, Context};
use std::{io::ErrorKind, net::Ipv4Addr};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};

pub async fn create_listener(port: u16) -> eyre::Result<TcpListener> {
    let localhost = Ipv4Addr::new(127, 0, 0, 1);
    let socket = match TcpListener::bind((localhost, port)).await {
        Ok(socket) => socket,
        Err(err) => {
            return Err(eyre::Report::new(err).wrap_err("failed to create local TCP listener"))
        }
    };
    Ok(socket)
}

pub async fn handle_connection(mut connection: TcpStream, events_tx: mpsc::Sender<Event>) {
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
        let message: coordinator_messages::CoordinatorRequest =
            match serde_json::from_slice(&raw).wrap_err("failed to deserialize node message") {
                Ok(e) => e,
                Err(err) => {
                    tracing::warn!("{err:?}");
                    continue;
                }
            };

        // handle the message and translate it to a DaemonEvent
        match message {
            coordinator_messages::CoordinatorRequest::Register { machine_id } => {
                let event = DaemonEvent::Register {
                    machine_id,
                    connection,
                };
                let _ = events_tx.send(Event::Daemon(event)).await;
                break;
            }
            coordinator_messages::CoordinatorRequest::Event { machine_id, event } => match event {
                coordinator_messages::DaemonEvent::AllNodesFinished {
                    dataflow_id,
                    result,
                } => {
                    let event = Event::Dataflow {
                        uuid: dataflow_id,
                        event: DataflowEvent::DataflowFinishedOnMachine {
                            machine_id,
                            result: result.map_err(|e| eyre!(e)),
                        },
                    };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
            },
        };
    }
}
