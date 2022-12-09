use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{self, ControlReply, DaemonCoordinatorEvent, DataflowId, SpawnDataflowNodes},
    topics::DORA_COORDINATOR_PORT_DEFAULT,
};
use dora_message::{uhlc, Metadata};
use eyre::{bail, eyre, Context};
use futures_concurrency::stream::Merge;
use shared_memory::{Shmem, ShmemConf};
use std::{
    collections::HashMap,
    net::{Ipv4Addr, SocketAddr},
};
use tokio::{
    net::TcpStream,
    sync::{mpsc, oneshot},
};
use tokio_stream::{
    wrappers::{ReceiverStream, TcpListenerStream},
    Stream, StreamExt,
};

mod coordinator;
mod listener;
mod spawn;
mod tcp_utils;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // the tokio::main proc macro confuses some tools such as rust-analyzer, so
    // directly invoke a "normal" async function
    run().await
}

async fn run() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    tracing::info!("Starting in local mode");
    let localhost = Ipv4Addr::new(127, 0, 0, 1);
    let coordinator_socket = (localhost, DORA_COORDINATOR_PORT_DEFAULT);

    Daemon::run(coordinator_socket.into()).await
}

struct Daemon {
    port: u16,
    hlc: uhlc::HLC,
    uninit_shared_memory: HashMap<String, Shmem>,
    sent_out_shared_memory: HashMap<String, Shmem>,
    subscribe_channels: HashMap<NodeId, flume::Sender<daemon_messages::NodeEvent>>,

    node_tasks: HashMap<DataflowId, HashMap<NodeId, tokio::task::JoinHandle<eyre::Result<()>>>>,
}

impl Daemon {
    pub async fn run(coordinator_addr: SocketAddr) -> eyre::Result<()> {
        // connect to the coordinator
        let coordinator_events = coordinator::connect(coordinator_addr)
            .await
            .wrap_err("failed to connect to dora-coordinator")?
            .map(Event::Coordinator);

        // create listener for node connection
        let listener = listener::create_listener().await?;
        let port = listener
            .local_addr()
            .wrap_err("failed to get local addr of listener")?
            .port();
        let new_connections = TcpListenerStream::new(listener).map(|c| {
            c.map(Event::NewConnection)
                .wrap_err("failed to open connection")
                .unwrap_or_else(Event::ConnectError)
        });
        tracing::info!("Listening for node connections on 127.0.0.1:{port}");

        let daemon = Self {
            port,
            hlc: uhlc::HLC::default(),
            uninit_shared_memory: Default::default(),
            sent_out_shared_memory: Default::default(),
            subscribe_channels: Default::default(),
            node_tasks: HashMap::new(),
        };
        let events = (coordinator_events, new_connections).merge();
        daemon.run_inner(events).await
    }

    async fn run_inner(
        mut self,
        incoming_events: impl Stream<Item = Event> + Unpin,
    ) -> eyre::Result<()> {
        let (node_events_tx, node_events_rx) = mpsc::channel(10);
        let node_events = ReceiverStream::new(node_events_rx);

        let mut events = (incoming_events, node_events).merge();

        while let Some(event) = events.next().await {
            match event {
                Event::NewConnection(connection) => {
                    let events_tx = node_events_tx.clone();
                    tokio::spawn(listener::handle_connection(connection, events_tx));
                }
                Event::ConnectError(err) => {
                    tracing::warn!("{:?}", err.wrap_err("failed to connect"));
                }
                Event::Coordinator(event) => self.handle_coordinator_event(event).await?,
                Event::Node {
                    id,
                    event,
                    reply_sender,
                } => self.handle_node_event(event, id, reply_sender).await?,
            }
        }

        Ok(())
    }

    async fn handle_coordinator_event(
        &mut self,
        event: DaemonCoordinatorEvent,
    ) -> eyre::Result<()> {
        match event {
            DaemonCoordinatorEvent::Spawn(SpawnDataflowNodes { dataflow_id, nodes }) => {
                let node_tasks = match self.node_tasks.entry(dataflow_id) {
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        entry.insert(Default::default())
                    }
                    std::collections::hash_map::Entry::Occupied(_) => {
                        bail!("there is already a running dataflow with ID `{dataflow_id}`")
                    }
                };
                for (node_id, params) in nodes {
                    let node_id = node_id.clone();
                    let task = spawn::spawn_node(params, self.port)
                        .await
                        .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))?;
                    node_tasks.insert(node_id, task);
                }

                // TODO: spawn timers
                Ok(())
            }
        }
    }

    async fn handle_node_event(
        &mut self,
        event: DaemonNodeEvent,
        id: NodeId,
        reply_sender: oneshot::Sender<ControlReply>,
    ) -> Result<(), eyre::ErrReport> {
        match event {
            DaemonNodeEvent::Subscribe { event_sender } => {
                self.subscribe_channels.insert(id, event_sender);
                let _ = reply_sender.send(ControlReply::Result(Ok(())));
            }
            DaemonNodeEvent::PrepareOutputMessage { output_id, len } => {
                let memory = ShmemConf::new()
                    .size(len)
                    .create()
                    .wrap_err("failed to allocate shared memory")?;
                let id = memory.get_os_id().to_owned();
                self.uninit_shared_memory.insert(id.clone(), memory);

                let reply = ControlReply::PreparedMessage {
                    shared_memory_id: id.clone(),
                };
                if reply_sender.send(reply).is_err() {
                    // free shared memory slice again
                    self.uninit_shared_memory.remove(&id);
                }
            }
            DaemonNodeEvent::SendOutMessage { id } => {
                let memory = self
                    .uninit_shared_memory
                    .remove(&id)
                    .ok_or_else(|| eyre!("invalid shared memory id"))?;

                // TODO figure out receivers from dataflow graph
                let local_receivers = &[];

                // send shared memory ID to all local receivers
                let mut closed = Vec::new();
                for receiver_id in local_receivers {
                    if let Some(channel) = self.subscribe_channels.get(receiver_id) {
                        let input_id = DataId::from("<unknown>".to_owned());
                        if channel
                            .send_async(daemon_messages::NodeEvent::Input {
                                id: input_id,
                                metadata: Metadata::new(self.hlc.new_timestamp()), // TODO
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
                    self.subscribe_channels.remove(id);
                }

                // TODO send `data` via network to all remove receivers
                let data = std::ptr::slice_from_raw_parts(memory.as_ptr(), memory.len());

                // keep shared memory ptr in order to free it once all subscribers are done
                self.sent_out_shared_memory.insert(id, memory);
            }
            DaemonNodeEvent::Stopped => {
                // TODO send stop message to downstream nodes

                let _ = reply_sender.send(ControlReply::Result(Ok(())));
            }
        }
        Ok(())
    }
}

pub enum Event {
    NewConnection(TcpStream),
    ConnectError(eyre::Report),
    Node {
        id: NodeId,
        event: DaemonNodeEvent,
        reply_sender: oneshot::Sender<ControlReply>,
    },
    Coordinator(DaemonCoordinatorEvent),
}

#[derive(Debug)]
pub enum DaemonNodeEvent {
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

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
