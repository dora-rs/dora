use coordinator::CoordinatorEvent;
use dora_core::{
    config::{DataId, InputMapping, NodeId},
    coordinator_messages::DaemonEvent,
    daemon_messages::{
        self, ControlReply, DaemonCoordinatorEvent, DaemonCoordinatorReply, DataflowId, DropEvent,
        DropToken, SpawnDataflowNodes,
    },
};
use dora_message::uhlc::HLC;
use eyre::{bail, eyre, Context, ContextCompat};
use futures::{future::Either, stream, FutureExt};
use futures_concurrency::stream::Merge;
use shared_memory::{Shmem, ShmemConf};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::SocketAddr,
    rc::Rc,
    time::Duration,
};
use tokio::{
    net::TcpStream,
    sync::{mpsc, oneshot},
    time::timeout,
};
use tokio_stream::{
    wrappers::{ReceiverStream, TcpListenerStream},
    Stream, StreamExt,
};

mod coordinator;
mod listener;
mod spawn;
mod tcp_utils;

pub struct Daemon {
    port: u16,
    uninit_shared_memory: HashMap<String, (DataId, dora_message::Metadata<'static>, Shmem)>,
    sent_out_shared_memory: HashMap<DropToken, Rc<Shmem>>,

    running: HashMap<DataflowId, RunningDataflow>,

    dora_events_tx: mpsc::Sender<DoraEvent>,

    coordinator_addr: Option<SocketAddr>,
    machine_id: String,
}

impl Daemon {
    pub async fn run(coordinator_addr: Option<SocketAddr>, machine_id: String) -> eyre::Result<()> {
        // connect to the coordinator
        let coordinator_events = match coordinator_addr {
            Some(addr) => Either::Left(
                coordinator::register(addr, machine_id.clone())
                    .await
                    .wrap_err("failed to connect to dora-coordinator")?
                    .map(Event::Coordinator),
            ),
            None => Either::Right(stream::empty()),
        };

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
            coordinator_addr,
            machine_id,
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
                Event::Coordinator(CoordinatorEvent { event, reply_tx }) => {
                    let result = self.handle_coordinator_event(event).await;
                    let _ = reply_tx.send(DaemonCoordinatorReply::SpawnResult(
                        result.map_err(|err| format!("{err:?}")),
                    ));
                }
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
                Event::Drop(DropEvent { token }) => {
                    match self.sent_out_shared_memory.remove(&token) {
                        Some(rc) => {
                            if let Ok(_shmem) = Rc::try_unwrap(rc) {
                                tracing::trace!(
                                    "freeing shared memory after receiving last drop token"
                                )
                            }
                        }
                        None => tracing::warn!("received unknown drop token {token:?}"),
                    }
                }
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
                self.spawn_dataflow(dataflow_id, nodes).await
            }
            DaemonCoordinatorEvent::StopDataflow { dataflow_id } => {
                let dataflow = self
                    .running
                    .get_mut(&dataflow_id)
                    .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;

                for channel in dataflow.subscribe_channels.values_mut() {
                    let _ = channel.send_async(daemon_messages::NodeEvent::Stop).await;
                }

                Ok(())
            }
        }
    }

    async fn spawn_dataflow(
        &mut self,
        dataflow_id: uuid::Uuid,
        nodes: BTreeMap<NodeId, daemon_messages::SpawnNodeParams>,
    ) -> eyre::Result<()> {
        let dataflow = match self.running.entry(dataflow_id) {
            std::collections::hash_map::Entry::Vacant(entry) => entry.insert(Default::default()),
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

                let memory = Rc::new(memory);

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
                        let drop_token = DropToken::generate();
                        let send_result = channel.send_async(daemon_messages::NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: metadata.clone(),
                            data: Some(daemon_messages::InputData {
                                shared_memory_id: id.clone(),
                                drop_token: drop_token.clone(),
                            }),
                        });

                        match timeout(Duration::from_millis(10), send_result).await {
                            Ok(Ok(())) => {
                                // keep shared memory ptr in order to free it once all subscribers are done
                                self.sent_out_shared_memory
                                    .insert(drop_token, memory.clone());
                            }
                            Ok(Err(_)) => {
                                closed.push(receiver_id);
                            }
                            Err(_) => {
                                tracing::warn!(
                                    "dropping input event `{receiver_id}/{input_id}` (send timeout)"
                                );
                            }
                        }
                    }
                }
                for id in closed {
                    dataflow.subscribe_channels.remove(id);
                }

                // TODO send `data` via network to all remove receivers
                let data = std::ptr::slice_from_raw_parts(memory.as_ptr(), memory.len());

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
                    tracing::info!(
                        "Dataflow `{dataflow_id}` finished on machine `{}`",
                        self.machine_id
                    );
                    if let Some(addr) = self.coordinator_addr {
                        if coordinator::send_event(
                            addr,
                            self.machine_id.clone(),
                            DaemonEvent::AllNodesFinished {
                                dataflow_id,
                                result: Ok(()),
                            },
                        )
                        .await
                        .is_err()
                        {
                            tracing::warn!("failed to report dataflow finish to coordinator");
                        }
                    }
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

                    let send_result = channel.send_async(daemon_messages::NodeEvent::Input {
                        id: input_id.clone(),
                        metadata: metadata.clone(),
                        data: None,
                    });
                    match timeout(Duration::from_millis(1), send_result).await {
                        Ok(Ok(())) => {}
                        Ok(Err(_)) => {
                            closed.push(receiver_id);
                        }
                        Err(_) => {
                            tracing::info!(
                                "dropping timer tick event for `{receiver_id}` (send timeout)"
                            );
                        }
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

#[derive(Debug)]
pub enum Event {
    NewConnection(TcpStream),
    ConnectError(eyre::Report),
    Node {
        dataflow_id: DataflowId,
        node_id: NodeId,
        event: DaemonNodeEvent,
        reply_sender: oneshot::Sender<ControlReply>,
    },
    Coordinator(CoordinatorEvent),
    Dora(DoraEvent),
    Drop(DropEvent),
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

#[derive(Debug)]
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
