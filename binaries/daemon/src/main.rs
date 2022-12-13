use dora_core::{
    config::{DataId, InputMapping, NodeId},
    daemon_messages::{self, ControlReply, DaemonCoordinatorEvent, DataflowId, SpawnDataflowNodes},
    topics::DORA_COORDINATOR_PORT_DEFAULT,
};
use dora_message::uhlc::HLC;
use eyre::{bail, eyre, Context, ContextCompat};
use futures::FutureExt;
use futures_concurrency::stream::Merge;
use shared_memory::{Shmem, ShmemConf};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::{Ipv4Addr, SocketAddr},
    time::Duration,
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
    uninit_shared_memory: HashMap<String, (DataId, dora_message::Metadata<'static>, Shmem)>,
    sent_out_shared_memory: HashMap<String, Shmem>,

    running: HashMap<DataflowId, RunningDataflow>,

    dora_events_tx: mpsc::Sender<DoraEvent>,
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

        let (dora_events_tx, dora_events_rx) = mpsc::channel(5);
        let daemon = Self {
            port,
            uninit_shared_memory: Default::default(),
            sent_out_shared_memory: Default::default(),
            running: HashMap::new(),
            dora_events_tx,
        };
        let dora_events = ReceiverStream::new(dora_events_rx).map(Event::Dora);
        let events = (coordinator_events, new_connections, dora_events).merge();
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
                    dataflow_id: dataflow,
                    node_id,
                    event,
                    reply_sender,
                } => {
                    self.handle_node_event(event, dataflow, node_id, reply_sender)
                        .await?
                }
                Event::Dora(event) => self.handle_dora_event(event).await?,
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
                let dataflow = match self.running.entry(dataflow_id) {
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        entry.insert(Default::default())
                    }
                    std::collections::hash_map::Entry::Occupied(_) => {
                        bail!("there is already a running dataflow with ID `{dataflow_id}`")
                    }
                };

                for (node_id, params) in nodes {
                    for (input_id, mapping) in params.node.run_config.inputs.clone() {
                        match mapping {
                            InputMapping::User(mapping) => {
                                if mapping.operator.is_some() {
                                    bail!("operators are not supported");
                                }
                                dataflow
                                    .mappings
                                    .entry((mapping.source, mapping.output))
                                    .or_default()
                                    .insert((node_id.clone(), input_id));
                            }
                            InputMapping::Timer { interval } => {
                                dataflow
                                    .timers
                                    .entry(interval)
                                    .or_default()
                                    .insert((node_id.clone(), input_id));
                            }
                        }
                    }

                    spawn::spawn_node(dataflow_id, params, self.port, self.dora_events_tx.clone())
                        .await
                        .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))?;
                }

                // spawn timer tasks
                for interval in dataflow.timers.keys().copied() {
                    let events_tx = self.dora_events_tx.clone();
                    let task = async move {
                        let mut interval_stream = tokio::time::interval(interval);
                        let hlc = HLC::default();
                        loop {
                            interval_stream.tick().await;

                            let event = DoraEvent::Timer {
                                dataflow_id,
                                interval,
                                metadata: dora_message::Metadata::from_parameters(
                                    hlc.new_timestamp(),
                                    Default::default(),
                                ),
                            };
                            if events_tx.send(event).await.is_err() {
                                break;
                            }
                        }
                    };
                    let (task, handle) = task.remote_handle();
                    tokio::spawn(task);
                    dataflow._timer_handles.push(handle);
                }

                Ok(())
            }
        }
    }

    async fn handle_node_event(
        &mut self,
        event: DaemonNodeEvent,
        dataflow_id: DataflowId,
        node_id: NodeId,
        reply_sender: oneshot::Sender<ControlReply>,
    ) -> Result<(), eyre::ErrReport> {
        match event {
            DaemonNodeEvent::Subscribe { event_sender } => {
                let result = match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        dataflow.subscribe_channels.insert(node_id, event_sender);
                        Ok(())
                    }
                    None => Err(format!("no running dataflow with ID `{dataflow_id}`")),
                };
                let _ = reply_sender.send(ControlReply::Result(result));
            }
            DaemonNodeEvent::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            } => {
                let memory = ShmemConf::new()
                    .size(data_len)
                    .create()
                    .wrap_err("failed to allocate shared memory")?;
                let id = memory.get_os_id().to_owned();
                self.uninit_shared_memory
                    .insert(id.clone(), (output_id, metadata, memory));

                let reply = ControlReply::PreparedMessage {
                    shared_memory_id: id.clone(),
                };
                if reply_sender.send(reply).is_err() {
                    // free shared memory slice again
                    self.uninit_shared_memory.remove(&id);
                }
            }
            DaemonNodeEvent::SendOutMessage { id } => {
                let (output_id, metadata, memory) = self
                    .uninit_shared_memory
                    .remove(&id)
                    .ok_or_else(|| eyre!("invalid shared memory id"))?;

                let dataflow = self
                    .running
                    .get_mut(&dataflow_id)
                    .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;

                // figure out receivers from dataflow graph
                let empty_set = BTreeSet::new();
                let local_receivers = dataflow
                    .mappings
                    .get(&(node_id, output_id))
                    .unwrap_or(&empty_set);

                // send shared memory ID to all local receivers
                let mut closed = Vec::new();
                for (receiver_id, input_id) in local_receivers {
                    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
                        if channel
                            .send_async(daemon_messages::NodeEvent::Input {
                                id: input_id.clone(),
                                metadata: metadata.clone(),
                                data: Some(unsafe { daemon_messages::InputData::new(id.clone()) }),
                            })
                            .await
                            .is_err()
                        {
                            closed.push(receiver_id);
                        }
                    }
                }
                for id in closed {
                    dataflow.subscribe_channels.remove(id);
                }

                // TODO send `data` via network to all remove receivers
                let data = std::ptr::slice_from_raw_parts(memory.as_ptr(), memory.len());

                // keep shared memory ptr in order to free it once all subscribers are done
                self.sent_out_shared_memory.insert(id, memory);

                let _ = reply_sender.send(ControlReply::Result(Ok(())));
            }
            DaemonNodeEvent::Stopped => {
                tracing::info!("Stopped: {dataflow_id}/{node_id}");

                let _ = reply_sender.send(ControlReply::Result(Ok(())));

                // notify downstream nodes
                let dataflow = self
                    .running
                    .get_mut(&dataflow_id)
                    .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
                let downstream_nodes: BTreeSet<_> = dataflow
                    .mappings
                    .iter()
                    .filter(|((source_id, _), _)| source_id == &node_id)
                    .flat_map(|(_, v)| v)
                    .collect();
                for (receiver_id, input_id) in downstream_nodes {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        continue;
                    };

                    let _ = channel
                        .send_async(daemon_messages::NodeEvent::InputClosed {
                            id: input_id.clone(),
                        })
                        .await;
                }

                // TODO: notify remote nodes

                dataflow.subscribe_channels.remove(&node_id);
                if dataflow.subscribe_channels.is_empty() {
                    tracing::info!("Dataflow `{dataflow_id}` finished");
                    self.running.remove(&dataflow_id);
                }
            }
        }
        Ok(())
    }

    async fn handle_dora_event(&mut self, event: DoraEvent) -> eyre::Result<()> {
        match event {
            DoraEvent::Timer {
                dataflow_id,
                interval,
                metadata,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Timer event for unknown dataflow `{dataflow_id}`");
                    return Ok(())
                };

                let Some(subscribers) = dataflow.timers.get(&interval) else {
                    return Ok(());
                };

                let mut closed = Vec::new();
                for (receiver_id, input_id) in subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        continue;
                    };

                    if channel
                        .send_async(daemon_messages::NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: metadata.clone(),
                            data: None,
                        })
                        .await
                        .is_err()
                    {
                        closed.push(receiver_id);
                    }
                }
                for id in closed {
                    dataflow.subscribe_channels.remove(id);
                }
            }
            DoraEvent::SpawnedNodeResult {
                dataflow_id,
                node_id,
                result,
            } => {
                if self
                    .running
                    .get(&dataflow_id)
                    .and_then(|d| d.subscribe_channels.get(&node_id))
                    .is_some()
                {
                    tracing::warn!(
                        "node `{dataflow_id}/{node_id}` finished without sending `Stopped` message"
                    );
                }
                match result {
                    Ok(()) => {
                        tracing::info!("node {dataflow_id}/{node_id} finished");
                    }
                    Err(err) => {
                        tracing::error!(
                            "{:?}",
                            err.wrap_err(format!("error in node `{dataflow_id}/{node_id}`"))
                        );
                    }
                }
            }
        }
        Ok(())
    }
}

#[derive(Default)]
pub struct RunningDataflow {
    subscribe_channels: HashMap<NodeId, flume::Sender<daemon_messages::NodeEvent>>,
    mappings: HashMap<OutputId, BTreeSet<InputId>>,
    timers: BTreeMap<Duration, BTreeSet<InputId>>,
    /// Keep handles to all timer tasks of this dataflow to cancel them on drop.
    _timer_handles: Vec<futures::future::RemoteHandle<()>>,
}

type OutputId = (NodeId, DataId);
type InputId = (NodeId, DataId);

pub enum Event {
    NewConnection(TcpStream),
    ConnectError(eyre::Report),
    Node {
        dataflow_id: DataflowId,
        node_id: NodeId,
        event: DaemonNodeEvent,
        reply_sender: oneshot::Sender<ControlReply>,
    },
    Coordinator(DaemonCoordinatorEvent),
    Dora(DoraEvent),
}

#[derive(Debug)]
pub enum DaemonNodeEvent {
    PrepareOutputMessage {
        output_id: DataId,
        metadata: dora_message::Metadata<'static>,
        data_len: usize,
    },
    SendOutMessage {
        id: MessageId,
    },
    Stopped,
    Subscribe {
        event_sender: flume::Sender<daemon_messages::NodeEvent>,
    },
}

pub enum DoraEvent {
    Timer {
        dataflow_id: DataflowId,
        interval: Duration,
        metadata: dora_message::Metadata<'static>,
    },
    SpawnedNodeResult {
        dataflow_id: DataflowId,
        node_id: NodeId,
        result: eyre::Result<()>,
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
