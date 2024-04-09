use crate::{tcp_utils::tcp_receive, DaemonEvent, DataflowEvent, Event};
use dora_core::{coordinator_messages, daemon_messages::Timestamped, message::uhlc::HLC};
use eyre::{eyre, Context};
use std::env;
use std::{io::ErrorKind, net::Ipv4Addr, sync::Arc};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};

pub async fn create_listener(port: u16) -> eyre::Result<TcpListener> {
    let localhost = env::var("DORA_DAEMON_HOST").unwrap_or_else(|_| "127.0.0.1".to_string());
    let localhost = Ipv4Addr::new(localhost.parse().unwrap());
    let socket = match TcpListener::bind((localhost, port)).await {
        Ok(socket) => socket,
        Err(err) => {
            return Err(eyre::Report::new(err).wrap_err("failed to create local TCP listener"))
        }
    };
    Ok(socket)
}

pub async fn handle_connection(
    mut connection: TcpStream,
    events_tx: mpsc::Sender<Event>,
    clock: Arc<HLC>,
) {
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
        let message: Timestamped<coordinator_messages::CoordinatorRequest> =
            match serde_json::from_slice(&raw).wrap_err("failed to deserialize node message") {
                Ok(e) => e,
                Err(err) => {
                    tracing::warn!("{err:?}");
                    continue;
                }
            };

        if let Err(err) = clock.update_with_timestamp(&message.timestamp) {
            tracing::warn!("failed to update coordinator clock: {err}");
        }

        // handle the message and translate it to a DaemonEvent
        match message.inner {
            coordinator_messages::CoordinatorRequest::Register {
                machine_id,
                dora_version,
                listen_socket,
            } => {
                let event = DaemonEvent::Register {
                    dora_version,
                    machine_id,
                    connection,
                    listen_socket,
                };
                let _ = events_tx.send(Event::Daemon(event)).await;
                break;
            }
            coordinator_messages::CoordinatorRequest::Event { machine_id, event } => match event {
                coordinator_messages::DaemonEvent::AllNodesReady {
                    dataflow_id,
                    success,
                } => {
                    let event = Event::Dataflow {
                        uuid: dataflow_id,
                        event: DataflowEvent::ReadyOnMachine {
                            machine_id,
                            success,
                        },
                    };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
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
                coordinator_messages::DaemonEvent::Heartbeat => {
                    let event = Event::DaemonHeartbeat { machine_id };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
            },
        };
    }
}
