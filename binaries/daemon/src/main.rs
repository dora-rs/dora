use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{self, ControlReply},
    topics::DORA_DAEMON_PORT_DEFAULT,
};
use dora_message::{uhlc, Metadata};
use eyre::{eyre, Context};
use futures_concurrency::stream::Merge;
use shared_memory::ShmemConf;
use std::{collections::HashMap, io::ErrorKind, net::Ipv4Addr};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    sync::{mpsc, oneshot},
};
use tokio_stream::{
    wrappers::{ReceiverStream, TcpListenerStream},
    StreamExt,
};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    main_inner().await
}

async fn main_inner() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    let localhost = Ipv4Addr::new(127, 0, 0, 1);
    let socket = match TcpListener::bind((localhost, DORA_DAEMON_PORT_DEFAULT)).await {
        Ok(socket) => socket,
        Err(err) if err.kind() == ErrorKind::AddrInUse => {
            eyre::bail!(
                "port {DORA_DAEMON_PORT_DEFAULT} is already in use. \
                Is `dora-daemon` already running?"
            );
        }
        Err(err) => {
            return Err(eyre::Report::new(err).wrap_err(format!(
                "failed to listen on port {DORA_DAEMON_PORT_DEFAULT}"
            )))
        }
    };

    // TODO: set up connection to coordinator

    let new_connections = TcpListenerStream::new(socket).map(|c| {
        c.map(Event::NewConnection)
            .wrap_err("failed to open connection")
            .unwrap_or_else(Event::ConnectError)
    });
    let (node_events_tx, node_events_rx) = mpsc::channel(10);
    let node_events = ReceiverStream::new(node_events_rx);

    let mut events = (new_connections, node_events).merge();
    let hlc = uhlc::HLC::default();

    let mut uninit_shared_memory = HashMap::new();
    let mut sent_out_shared_memory = HashMap::new();

    let mut subscribe_channels = HashMap::new();

    while let Some(event) = events.next().await {
        match event {
            Event::NewConnection(connection) => {
                let events_tx = node_events_tx.clone();
                tokio::spawn(handle_connection(connection, events_tx));
            }
            Event::ConnectError(err) => {
                tracing::warn!("{:?}", err.wrap_err("failed to connect"));
            }
            Event::Node {
                id,
                event,
                reply_sender,
            } => match event {
                NodeEvent::Subscribe { event_sender } => {
                    subscribe_channels.insert(id, event_sender);
                    let _ = reply_sender.send(ControlReply::Result(Ok(())));
                }
                NodeEvent::PrepareOutputMessage { output_id, len } => {
                    let memory = ShmemConf::new()
                        .size(len)
                        .create()
                        .wrap_err("failed to allocate shared memory")?;
                    let id = memory.get_os_id().to_owned();
                    uninit_shared_memory.insert(id.clone(), memory);

                    let reply = ControlReply::PreparedMessage {
                        shared_memory_id: id.clone(),
                    };
                    if reply_sender.send(reply).is_err() {
                        // free shared memory slice again
                        uninit_shared_memory.remove(&id);
                    }
                }
                NodeEvent::SendOutMessage { id } => {
                    let memory = uninit_shared_memory
                        .remove(&id)
                        .ok_or_else(|| eyre!("invalid shared memory id"))?;

                    // TODO figure out receivers from dataflow graph
                    let local_receivers = &[];

                    // send shared memory ID to all local receivers
                    let mut closed = Vec::new();
                    for receiver_id in local_receivers {
                        if let Some(channel) = subscribe_channels.get(receiver_id) {
                            let input_id = DataId::from("<unknown>".to_owned());
                            if channel
                                .send_async(daemon_messages::NodeEvent::Input {
                                    id: input_id,
                                    metadata: Metadata::new(hlc.new_timestamp()), // TODO
                                    data: unsafe { daemon_messages::InputData::new(id.clone()) },
                                })
                                .await
                                .is_err()
                            {
                                closed.push(receiver_id);
                            }
                        }
                    }
                    for id in closed {
                        subscribe_channels.remove(id);
                    }

                    // TODO send `data` via network to all remove receivers
                    let data = std::ptr::slice_from_raw_parts(memory.as_ptr(), memory.len());

                    // keep shared memory ptr in order to free it once all subscribers are done
                    sent_out_shared_memory.insert(id, memory);
                }
                NodeEvent::Stopped => {
                    // TODO send stop message to downstream nodes

                    let _ = reply_sender.send(ControlReply::Result(Ok(())));
                }
            },
        }
    }

    Ok(())
}

async fn handle_connection(mut connection: TcpStream, events_tx: mpsc::Sender<Event>) {
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
            daemon_messages::ControlRequest::Stopped => NodeEvent::Stopped,
            daemon_messages::ControlRequest::PrepareOutputMessage { output_id, len } => {
                NodeEvent::PrepareOutputMessage { output_id, len }
            }
            daemon_messages::ControlRequest::SendOutMessage { id } => {
                NodeEvent::SendOutMessage { id }
            }
            daemon_messages::ControlRequest::Subscribe { node_id } => {
                let (tx, rx) = flume::bounded(10);

                id = Some(node_id);
                enter_subscribe_loop = Some(rx);

                NodeEvent::Subscribe { event_sender: tx }
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

enum Event {
    NewConnection(TcpStream),
    ConnectError(eyre::Report),
    Node {
        id: NodeId,
        event: NodeEvent,
        reply_sender: oneshot::Sender<ControlReply>,
    },
}

#[derive(Debug)]
pub enum NodeEvent {
    PrepareOutputMessage {
        output_id: DataId,
        len: usize,
    },
    SendOutMessage {
        id: MessageId,
    },
    Stopped,
    Subscribe {
        event_sender: flume::Sender<daemon_messages::NodeEvent>,
    },
}

type MessageId = String;

async fn tcp_send(connection: &mut TcpStream, message: &[u8]) -> std::io::Result<()> {
    let len_raw = (message.len() as u64).to_le_bytes();
    connection.write_all(&len_raw).await?;
    connection.write_all(message).await?;
    Ok(())
}

async fn tcp_receive(connection: &mut TcpStream) -> std::io::Result<Vec<u8>> {
    let reply_len = {
        let mut raw = [0; 8];
        connection.read_exact(&mut raw).await?;
        u64::from_le_bytes(raw) as usize
    };
    let mut reply = vec![0; reply_len];
    connection.read_exact(&mut reply).await?;
    Ok(reply)
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
