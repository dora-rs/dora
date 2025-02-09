use crate::{tcp_utils::tcp_receive, DaemonRequest, DataflowEvent, Event};
use dora_core::uhlc::HLC;
use dora_message::daemon_to_coordinator::{CoordinatorRequest, DaemonEvent, Timestamped};
use eyre::Context;
use std::{io::ErrorKind, net::SocketAddr, sync::Arc};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};

pub async fn create_listener(bind: SocketAddr) -> eyre::Result<TcpListener> {
    let socket = match TcpListener::bind(bind).await {
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
        let message: Timestamped<CoordinatorRequest> =
            match serde_json::from_slice(&raw).wrap_err("failed to deserialize message") {
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
            CoordinatorRequest::Register(register_request) => {
                let event = DaemonRequest::Register {
                    connection,
                    version_check_result: register_request.check_version(),
                    machine_id: register_request.machine_id,
                };
                let _ = events_tx.send(Event::Daemon(event)).await;
                break;
            }
            CoordinatorRequest::Event { daemon_id, event } => match event {
                DaemonEvent::AllNodesReady {
                    dataflow_id,
                    exited_before_subscribe,
                } => {
                    let event = Event::Dataflow {
                        uuid: dataflow_id,
                        event: DataflowEvent::ReadyOnDeamon {
                            daemon_id,
                            exited_before_subscribe,
                        },
                    };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
                DaemonEvent::AllNodesFinished {
                    dataflow_id,
                    result,
                } => {
                    let event = Event::Dataflow {
                        uuid: dataflow_id,
                        event: DataflowEvent::DataflowFinishedOnDaemon { daemon_id, result },
                    };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
                DaemonEvent::Heartbeat => {
                    let event = Event::DaemonHeartbeat { daemon_id };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
                DaemonEvent::Log(message) => {
                    let event = Event::Log(message);
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
            },
        };
    }
}
