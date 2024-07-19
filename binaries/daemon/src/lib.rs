use aligned_vec::{AVec, ConstAlign};
use coordinator::CoordinatorEvent;
use crossbeam::queue::ArrayQueue;
use dora_core::config::{Input, OperatorId};
use dora_core::coordinator_messages::{CoordinatorRequest, Level, LogMessage};
use dora_core::daemon_messages::{
    DataMessage, DynamicNodeEvent, InterDaemonEvent, NodeConfig, Timestamped,
};
use dora_core::descriptor::runtime_node_inputs;
use dora_core::message::uhlc::{self, HLC};
use dora_core::message::{ArrowTypeInfo, Metadata, MetadataParameters};
use dora_core::topics::LOCALHOST;
use dora_core::topics::{
    DataflowDaemonResult, DataflowResult, NodeError, NodeErrorCause, NodeExitStatus,
};
use dora_core::{
    config::{DataId, InputMapping, NodeId},
    coordinator_messages::DaemonEvent,
    daemon_messages::{
        self, DaemonCoordinatorEvent, DaemonCoordinatorReply, DaemonReply, DataflowId, DropToken,
        SpawnDataflowNodes,
    },
    descriptor::{CoreNodeKind, Descriptor, ResolvedNode},
};

use eyre::{bail, eyre, Context, ContextCompat, Result};
use futures::{future, stream, FutureExt, TryFutureExt};
use futures_concurrency::stream::Merge;
use inter_daemon::InterDaemonConnection;
use local_listener::DynamicNodeEventWrapper;
use pending::PendingNodes;
use shared_memory_server::ShmemConf;
use std::sync::Arc;
use std::time::Instant;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::SocketAddr,
    path::{Path, PathBuf},
    time::Duration,
};
use sysinfo::Pid;
use socket_stream_utils::socket_stream_send;
use tokio::fs::File;
use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::sync::mpsc::UnboundedSender;
use tokio::sync::oneshot::Sender;
use tokio::sync::{mpsc, oneshot};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use tracing::{error, warn};
use uuid::{NoContext, Timestamp, Uuid};

mod coordinator;
mod inter_daemon;
mod local_listener;
mod log;
mod node_communication;
mod pending;
mod spawn;
mod socket_stream_utils;

#[cfg(feature = "telemetry")]
use dora_tracing::telemetry::serialize_context;
#[cfg(feature = "telemetry")]
use tracing_opentelemetry::OpenTelemetrySpanExt;

use crate::pending::DataflowStatus;

const STDERR_LOG_LINES: usize = 10;

pub struct Daemon {
    running: HashMap<DataflowId, RunningDataflow>,
    working_dir: HashMap<DataflowId, PathBuf>,

    events_tx: mpsc::Sender<Timestamped<Event>>,

    coordinator_connection: Option<TcpStream>,
    last_coordinator_heartbeat: Instant,
    inter_daemon_connections: BTreeMap<String, InterDaemonConnection>,
    machine_id: String,

    /// used for testing and examples
    exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
    /// used to record dataflow results when `exit_when_done` is used
    dataflow_node_results: BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>,

    clock: Arc<uhlc::HLC>,
}

type DaemonRunResult = BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>;

impl Daemon {
    pub async fn run(
        coordinator_addr: SocketAddr,
        machine_id: String,
        inter_daemon_addr: SocketAddr,
        local_listen_port: u16,
    ) -> eyre::Result<()> {
        let clock = Arc::new(HLC::default());

        let ctrlc_events = set_up_ctrlc_handler(clock.clone())?;

        // spawn inter daemon listen loop
        let (events_tx, events_rx) = flume::bounded(10);
        let listen_port =
            inter_daemon::spawn_listener_loop(inter_daemon_addr, machine_id.clone(), events_tx)
                .await?;
        let daemon_events = events_rx.into_stream().map(|e| Timestamped {
            inner: Event::Daemon(e.inner),
            timestamp: e.timestamp,
        });

        // connect to the coordinator
        let coordinator_events =
            coordinator::register(coordinator_addr, machine_id.clone(), listen_port, &clock)
                .await
                .wrap_err("failed to connect to dora-coordinator")?
                .map(
                    |Timestamped {
                         inner: event,
                         timestamp,
                     }| Timestamped {
                        inner: Event::Coordinator(event),
                        timestamp,
                    },
                );

        // Spawn local listener loop
        let (events_tx, events_rx) = flume::bounded(10);
        let _listen_port = local_listener::spawn_listener_loop(
            (LOCALHOST, local_listen_port).into(),
            machine_id.clone(),
            events_tx,
        )
        .await?;
        let dynamic_node_events = events_rx.into_stream().map(|e| Timestamped {
            inner: Event::DynamicNode(e.inner),
            timestamp: e.timestamp,
        });
        Self::run_general(
            (
                coordinator_events,
                ctrlc_events,
                daemon_events,
                dynamic_node_events,
            )
                .merge(),
            Some(coordinator_addr),
            machine_id,
            None,
            clock,
        )
        .await
        .map(|_| ())
    }

    pub async fn run_dataflow(dataflow_path: &Path) -> eyre::Result<DataflowResult> {
        let working_dir = dataflow_path
            .canonicalize()
            .context("failed to canoncialize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
            .to_owned();

        let descriptor = Descriptor::read(dataflow_path).await?;
        descriptor.check(&working_dir)?;
        let nodes = descriptor.resolve_aliases_and_set_defaults()?;

        let dataflow_id = Uuid::new_v7(Timestamp::now(NoContext));
        let spawn_command = SpawnDataflowNodes {
            dataflow_id,
            working_dir,
            nodes,
            machine_listen_ports: BTreeMap::new(),
            dataflow_descriptor: descriptor,
        };

        let clock = Arc::new(HLC::default());

        let exit_when_done = spawn_command
            .nodes
            .iter()
            .map(|n| (spawn_command.dataflow_id, n.id.clone()))
            .collect();
        let (reply_tx, reply_rx) = oneshot::channel();
        let timestamp = clock.new_timestamp();
        let coordinator_events = stream::once(async move {
            Timestamped {
                inner: Event::Coordinator(CoordinatorEvent {
                    event: DaemonCoordinatorEvent::Spawn(spawn_command),
                    reply_tx,
                }),
                timestamp,
            }
        });
        let run_result = Self::run_general(
            Box::pin(coordinator_events),
            None,
            "".to_string(),
            Some(exit_when_done),
            clock.clone(),
        );

        let spawn_result = reply_rx
            .map_err(|err| eyre!("failed to receive spawn result: {err}"))
            .and_then(|r| async {
                match r {
                    Some(DaemonCoordinatorReply::SpawnResult(result)) => {
                        result.map_err(|err| eyre!(err))
                    }
                    _ => Err(eyre!("unexpected spawn reply")),
                }
            });

        let (mut dataflow_results, ()) = future::try_join(run_result, spawn_result).await?;

        Ok(DataflowResult {
            uuid: dataflow_id,
            timestamp: clock.new_timestamp(),
            node_results: dataflow_results
                .remove(&dataflow_id)
                .context("no node results for dataflow_id")?,
        })
    }

    async fn run_general(
        external_events: impl Stream<Item = Timestamped<Event>> + Unpin,
        coordinator_addr: Option<SocketAddr>,
        machine_id: String,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        clock: Arc<HLC>,
    ) -> eyre::Result<DaemonRunResult> {
        let coordinator_connection = match coordinator_addr {
            Some(addr) => {
                let stream = TcpStream::connect(addr)
                    .await
                    .wrap_err("failed to connect to dora-coordinator")?;
                stream
                    .set_nodelay(true)
                    .wrap_err("failed to set TCP_NODELAY")?;
                Some(stream)
            }
            None => None,
        };

        let (dora_events_tx, dora_events_rx) = mpsc::channel(5);
        let daemon = Self {
            running: HashMap::new(),
            working_dir: HashMap::new(),
            events_tx: dora_events_tx,
            coordinator_connection,
            last_coordinator_heartbeat: Instant::now(),
            inter_daemon_connections: BTreeMap::new(),
            machine_id,
            exit_when_done,
            dataflow_node_results: BTreeMap::new(),
            clock,
        };

        let dora_events = ReceiverStream::new(dora_events_rx);
        let watchdog_clock = daemon.clock.clone();
        let watchdog_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(5),
        ))
        .map(|_| Timestamped {
            inner: Event::HeartbeatInterval,
            timestamp: watchdog_clock.new_timestamp(),
        });
        let events = (external_events, dora_events, watchdog_interval).merge();
        daemon.run_inner(events).await
    }

    #[tracing::instrument(skip(incoming_events, self), fields(%self.machine_id))]
    async fn run_inner(
        mut self,
        incoming_events: impl Stream<Item = Timestamped<Event>> + Unpin,
    ) -> eyre::Result<DaemonRunResult> {
        let mut events = incoming_events;

        while let Some(event) = events.next().await {
            let Timestamped { inner, timestamp } = event;
            if let Err(err) = self.clock.update_with_timestamp(&timestamp) {
                tracing::warn!("failed to update HLC with incoming event timestamp: {err}");
            }

            match inner {
                Event::Coordinator(CoordinatorEvent { event, reply_tx }) => {
                    let status = self.handle_coordinator_event(event, reply_tx).await?;

                    match status {
                        RunStatus::Continue => {}
                        RunStatus::Exit => break,
                    }
                }
                Event::Daemon(event) => {
                    self.handle_inter_daemon_event(event).await?;
                }
                Event::Node {
                    dataflow_id: dataflow,
                    node_id,
                    event,
                } => self.handle_node_event(event, dataflow, node_id).await?,
                Event::Dora(event) => match self.handle_dora_event(event).await? {
                    RunStatus::Continue => {}
                    RunStatus::Exit => break,
                },
                Event::DynamicNode(event) => self.handle_dynamic_node_event(event).await?,
                Event::HeartbeatInterval => {
                    if let Some(connection) = &mut self.coordinator_connection {
                        let msg = serde_json::to_vec(&Timestamped {
                            inner: CoordinatorRequest::Event {
                                machine_id: self.machine_id.clone(),
                                event: DaemonEvent::Heartbeat,
                            },
                            timestamp: self.clock.new_timestamp(),
                        })?;
                        socket_stream_send(connection, &msg)
                            .await
                            .wrap_err("failed to send watchdog message to dora-coordinator")?;

                        if self.last_coordinator_heartbeat.elapsed() > Duration::from_secs(20) {
                            bail!("lost connection to coordinator")
                        }
                    }
                }
                Event::CtrlC => {
                    for dataflow in self.running.values_mut() {
                        dataflow
                            .stop_all(&mut self.coordinator_connection, &self.clock, None)
                            .await?;
                    }
                }
            }
        }

        Ok(self.dataflow_node_results)
    }

    async fn send_log_message(&mut self, message: LogMessage) -> eyre::Result<()> {
        if let Some(connection) = &mut self.coordinator_connection {
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    machine_id: self.machine_id.clone(),
                    event: DaemonEvent::Log(message),
                },
                timestamp: self.clock.new_timestamp(),
            })?;
            socket_stream_send(connection, &msg)
                .await
                .wrap_err("failed to send watchdog message to dora-coordinator")?;

            if self.last_coordinator_heartbeat.elapsed() > Duration::from_secs(20) {
                bail!("lost connection to coordinator")
            }
        }
        Ok(())
    }

    async fn handle_coordinator_event(
        &mut self,
        event: DaemonCoordinatorEvent,
        reply_tx: Sender<Option<DaemonCoordinatorReply>>,
    ) -> eyre::Result<RunStatus> {
        let status = match event {
            DaemonCoordinatorEvent::Spawn(SpawnDataflowNodes {
                dataflow_id,
                working_dir,
                nodes,
                machine_listen_ports,
                dataflow_descriptor,
            }) => {
                match dataflow_descriptor.communication.remote {
                    dora_core::config::RemoteCommunicationConfig::Tcp => {}
                }
                for (machine_id, socket) in machine_listen_ports {
                    match self.inter_daemon_connections.entry(machine_id) {
                        std::collections::btree_map::Entry::Vacant(entry) => {
                            entry.insert(InterDaemonConnection::new(socket));
                        }
                        std::collections::btree_map::Entry::Occupied(mut entry) => {
                            if entry.get().socket() != socket {
                                entry.insert(InterDaemonConnection::new(socket));
                            }
                        }
                    }
                }

                let result = self
                    .spawn_dataflow(dataflow_id, working_dir, nodes, dataflow_descriptor)
                    .await;
                if let Err(err) = &result {
                    tracing::error!("{err:?}");
                }
                let reply =
                    DaemonCoordinatorReply::SpawnResult(result.map_err(|err| format!("{err:?}")));
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send `SpawnResult` reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::AllNodesReady {
                dataflow_id,
                exited_before_subscribe,
            } => {
                match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        let ready = exited_before_subscribe.is_empty();
                        dataflow
                            .pending_nodes
                            .handle_external_all_nodes_ready(
                                exited_before_subscribe,
                                &mut dataflow.cascading_error_causes,
                            )
                            .await?;
                        if ready {
                            tracing::info!("coordinator reported that all nodes are ready, starting dataflow `{dataflow_id}`");
                            dataflow.start(&self.events_tx, &self.clock).await?;
                        }
                    }
                    None => {
                        tracing::warn!(
                            "received AllNodesReady for unknown dataflow (ID `{dataflow_id}`)"
                        );
                    }
                }
                let _ = reply_tx.send(None).map_err(|_| {
                    error!("could not send `AllNodesReady` reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Logs {
                dataflow_id,
                node_id,
            } => {
                match self.working_dir.get(&dataflow_id) {
                    Some(working_dir) => {
                        let working_dir = working_dir.clone();
                        tokio::spawn(async move {
                            let logs = async {
                                let mut file =
                                    File::open(log::log_path(&working_dir, &dataflow_id, &node_id))
                                        .await
                                        .wrap_err(format!(
                                            "Could not open log file: {:#?}",
                                            log::log_path(&working_dir, &dataflow_id, &node_id)
                                        ))?;

                                let mut contents = vec![];
                                file.read_to_end(&mut contents)
                                    .await
                                    .wrap_err("Could not read content of log file")?;
                                Result::<Vec<u8>, eyre::Report>::Ok(contents)
                            }
                            .await
                            .map_err(|err| format!("{err:?}"));
                            let _ = reply_tx
                                .send(Some(DaemonCoordinatorReply::Logs(logs)))
                                .map_err(|_| {
                                    error!("could not send logs reply from daemon to coordinator")
                                });
                        });
                    }
                    None => {
                        tracing::warn!("received Logs for unknown dataflow (ID `{dataflow_id}`)");
                        let _ = reply_tx.send(None).map_err(|_| {
                            error!(
                                "could not send `AllNodesReady` reply from daemon to coordinator"
                            )
                        });
                    }
                }
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::ReloadDataflow {
                dataflow_id,
                node_id,
                operator_id,
            } => {
                let result = self.send_reload(dataflow_id, node_id, operator_id).await;
                let reply =
                    DaemonCoordinatorReply::ReloadResult(result.map_err(|err| format!("{err:?}")));
                let _ = reply_tx
                    .send(Some(reply))
                    .map_err(|_| error!("could not send reload reply from daemon to coordinator"));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StopDataflow {
                dataflow_id,
                grace_duration,
            } => {
                let dataflow = self
                    .running
                    .get_mut(&dataflow_id)
                    .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;

                let reply = DaemonCoordinatorReply::StopResult(Ok(()));
                let _ = reply_tx
                    .send(Some(reply))
                    .map_err(|_| error!("could not send stop reply from daemon to coordinator"));

                dataflow
                    .stop_all(
                        &mut self.coordinator_connection,
                        &self.clock,
                        grace_duration,
                    )
                    .await?;
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Destroy => {
                tracing::info!("received destroy command -> exiting");
                let (notify_tx, notify_rx) = oneshot::channel();
                let reply = DaemonCoordinatorReply::DestroyResult {
                    result: Ok(()),
                    notify: Some(notify_tx),
                };
                let _ = reply_tx
                    .send(Some(reply))
                    .map_err(|_| error!("could not send destroy reply from daemon to coordinator"));
                // wait until the reply is sent out
                if notify_rx.await.is_err() {
                    tracing::warn!("no confirmation received for DestroyReply");
                }
                RunStatus::Exit
            }
            DaemonCoordinatorEvent::Heartbeat => {
                self.last_coordinator_heartbeat = Instant::now();
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
        };
        Ok(status)
    }

    async fn handle_inter_daemon_event(&mut self, event: InterDaemonEvent) -> eyre::Result<()> {
        match event {
            InterDaemonEvent::Output {
                dataflow_id,
                node_id,
                output_id,
                metadata,
                data,
            } => {
                let inner = async {
                    let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                        format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                    })?;
                    send_output_to_local_receivers(
                        node_id.clone(),
                        output_id.clone(),
                        dataflow,
                        &metadata,
                        data.map(DataMessage::Vec),
                        &self.clock,
                    )
                    .await?;
                    Result::<_, eyre::Report>::Ok(())
                };
                if let Err(err) = inner
                    .await
                    .wrap_err("failed to forward remote output to local receivers")
                {
                    tracing::warn!("{err:?}")
                }
                Ok(())
            }
            InterDaemonEvent::InputsClosed {
                dataflow_id,
                inputs,
            } => {
                tracing::debug!(?dataflow_id, ?inputs, "received InputsClosed event");
                let inner = async {
                    let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                        format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                    })?;
                    for (receiver_id, input_id) in &inputs {
                        close_input(dataflow, receiver_id, input_id, &self.clock);
                    }
                    Result::<(), eyre::Report>::Ok(())
                };
                if let Err(err) = inner
                    .await
                    .wrap_err("failed to handle InputsClosed event sent by coordinator")
                {
                    tracing::warn!("{err:?}")
                }
                Ok(())
            }
        }
    }

    async fn spawn_dataflow(
        &mut self,
        dataflow_id: uuid::Uuid,
        working_dir: PathBuf,
        nodes: Vec<ResolvedNode>,
        dataflow_descriptor: Descriptor,
    ) -> eyre::Result<()> {
        let dataflow = RunningDataflow::new(dataflow_id, self.machine_id.clone());
        let dataflow = match self.running.entry(dataflow_id) {
            std::collections::hash_map::Entry::Vacant(entry) => {
                self.working_dir.insert(dataflow_id, working_dir.clone());
                entry.insert(dataflow)
            }
            std::collections::hash_map::Entry::Occupied(_) => {
                bail!("there is already a running dataflow with ID `{dataflow_id}`")
            }
        };

        let mut log_messages = Vec::new();
        for node in nodes {
            let local = node.deploy.machine == self.machine_id;

            let inputs = node_inputs(&node);
            for (input_id, input) in inputs {
                if local {
                    dataflow
                        .open_inputs
                        .entry(node.id.clone())
                        .or_default()
                        .insert(input_id.clone());
                    match input.mapping {
                        InputMapping::User(mapping) => {
                            dataflow
                                .mappings
                                .entry(OutputId(mapping.source, mapping.output))
                                .or_default()
                                .insert((node.id.clone(), input_id));
                        }
                        InputMapping::Timer { interval } => {
                            dataflow
                                .timers
                                .entry(interval)
                                .or_default()
                                .insert((node.id.clone(), input_id));
                        }
                    }
                } else if let InputMapping::User(mapping) = input.mapping {
                    dataflow
                        .open_external_mappings
                        .entry(OutputId(mapping.source, mapping.output))
                        .or_default()
                        .entry(node.deploy.machine.clone())
                        .or_default()
                        .insert((node.id.clone(), input_id));
                }
            }
            if local {
                dataflow.pending_nodes.insert(node.id.clone());

                if node.kind.dynamic() {
                    dataflow.dynamic_nodes.insert(node.id.clone());
                }

                let node_id = node.id.clone();
                let node_stderr_most_recent = dataflow
                    .node_stderr_most_recent
                    .entry(node.id.clone())
                    .or_insert_with(|| Arc::new(ArrayQueue::new(STDERR_LOG_LINES)))
                    .clone();
                match spawn::spawn_node(
                    dataflow_id,
                    &working_dir,
                    node,
                    self.events_tx.clone(),
                    dataflow_descriptor.clone(),
                    self.clock.clone(),
                    node_stderr_most_recent,
                )
                .await
                .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))
                {
                    Ok(running_node) => {
                        dataflow.running_nodes.insert(node_id, running_node);
                    }
                    Err(err) => {
                        log_messages.push(LogMessage {
                            dataflow_id,
                            node_id: Some(node_id.clone()),
                            level: Level::Error,
                            target: None,
                            module_path: None,
                            file: None,
                            line: None,
                            message: format!("{err:?}"),
                        });
                        let messages = dataflow
                            .pending_nodes
                            .handle_node_stop(
                                &node_id,
                                &mut self.coordinator_connection,
                                &self.clock,
                                &mut dataflow.cascading_error_causes,
                            )
                            .await?;
                        log_messages.extend(messages);
                    }
                }
            } else {
                dataflow.pending_nodes.set_external_nodes(true);
            }
        }

        for log_message in log_messages {
            self.send_log_message(log_message).await?;
        }

        Ok(())
    }

    async fn handle_dynamic_node_event(
        &mut self,
        event: DynamicNodeEventWrapper,
    ) -> eyre::Result<()> {
        match event {
            DynamicNodeEventWrapper {
                event: DynamicNodeEvent::NodeConfig { node_id },
                reply_tx,
            } => {
                let number_node_id = self
                    .running
                    .iter()
                    .filter(|(_id, dataflow)| dataflow.running_nodes.contains_key(&node_id))
                    .count();

                let node_config = match number_node_id {
                    2.. => {
                        let _ = reply_tx.send(Some(DaemonReply::NodeConfig {
                            result: Err(format!(
                                "multiple dataflows contains dynamic node id {}. Please only have one running dataflow with the specified node id if you want to use dynamic node",
                                node_id
                            )
                            .to_string()),
                        }));
                        return Ok(());
                    }
                    1 => self
                        .running
                        .iter()
                        .filter(|(_id, dataflow)| dataflow.running_nodes.contains_key(&node_id))
                        .map(|(id, dataflow)| -> Result<NodeConfig> {
                            let node_config = dataflow
                                .running_nodes
                                .get(&node_id)
                                .context("no node with ID `{node_id}` within the given dataflow")?
                                .node_config
                                .clone();
                            if !node_config.dynamic {
                                bail!("node with ID `{node_id}` in {id} is not dynamic");
                            }
                            Ok(node_config)
                        })
                        .next()
                        .context("no node with ID `{node_id}`")?
                        .context("failed to get dynamic node config within given dataflow")?,
                    0 => {
                        let _ = reply_tx.send(Some(DaemonReply::NodeConfig {
                            result: Err("no node with ID `{node_id}`".to_string()),
                        }));
                        return Ok(());
                    }
                };

                let reply = DaemonReply::NodeConfig {
                    result: Ok(node_config),
                };
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send node info reply from daemon to coordinator")
                });
                Ok(())
            }
        }
    }

    async fn handle_node_event(
        &mut self,
        event: DaemonNodeEvent,
        dataflow_id: DataflowId,
        node_id: NodeId,
    ) -> eyre::Result<()> {
        match event {
            DaemonNodeEvent::Subscribe {
                event_sender,
                reply_sender,
            } => {
                let dataflow = self.running.get_mut(&dataflow_id).ok_or_else(|| {
                    format!("subscribe failed: no running dataflow with ID `{dataflow_id}`")
                });

                match dataflow {
                    Err(err) => {
                        let _ = reply_sender.send(DaemonReply::Result(Err(err)));
                    }
                    Ok(dataflow) => {
                        tracing::debug!("node `{node_id}` is ready");
                        Self::subscribe(dataflow, node_id.clone(), event_sender, &self.clock).await;

                        let status = dataflow
                            .pending_nodes
                            .handle_node_subscription(
                                node_id.clone(),
                                reply_sender,
                                &mut self.coordinator_connection,
                                &self.clock,
                                &mut dataflow.cascading_error_causes,
                            )
                            .await?;
                        match status {
                            DataflowStatus::AllNodesReady => {
                                tracing::info!(
                                    "all nodes are ready, starting dataflow `{dataflow_id}`"
                                );
                                dataflow.start(&self.events_tx, &self.clock).await?;
                            }
                            DataflowStatus::Pending => {}
                        }
                    }
                }
            }
            DaemonNodeEvent::SubscribeDrop {
                event_sender,
                reply_sender,
            } => {
                let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!("failed to subscribe: no running dataflow with ID `{dataflow_id}`")
                })?;
                dataflow.drop_channels.insert(node_id, event_sender);
                let _ = reply_sender.send(DaemonReply::Result(Ok(())));
            }
            DaemonNodeEvent::CloseOutputs {
                outputs,
                reply_sender,
            } => {
                // notify downstream nodes
                let inner = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`"))?;
                    send_input_closed_events(
                        dataflow,
                        &mut self.inter_daemon_connections,
                        |OutputId(source_id, output_id)| {
                            source_id == &node_id && outputs.contains(output_id)
                        },
                        &self.clock,
                    )
                    .await
                };

                let reply = inner.await.map_err(|err| format!("{err:?}"));
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
            DaemonNodeEvent::OutputsDone { reply_sender } => {
                let result = match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        Self::handle_outputs_done(dataflow, &mut self.inter_daemon_connections, &node_id, &self.clock)
                    .await
                    },
                    None => Err(eyre!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`")),
                };

                let _ = reply_sender.send(DaemonReply::Result(
                    result.map_err(|err| format!("{err:?}")),
                ));
            }
            DaemonNodeEvent::SendOut {
                output_id,
                metadata,
                data,
            } => {
                self.send_out(dataflow_id, node_id, output_id, metadata, data)
                    .await?
            }
            DaemonNodeEvent::ReportDrop { tokens } => {
                let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!(
                        "failed to get handle drop tokens: \
                        no running dataflow with ID `{dataflow_id}`"
                    )
                })?;

                for token in tokens {
                    match dataflow.pending_drop_tokens.get_mut(&token) {
                        Some(info) => {
                            if info.pending_nodes.remove(&node_id) {
                                dataflow.check_drop_token(token, &self.clock).await?;
                            } else {
                                tracing::warn!(
                                    "node `{node_id}` is not pending for drop token `{token:?}`"
                                );
                            }
                        }
                        None => tracing::warn!("unknown drop token `{token:?}`"),
                    }
                }
            }
            DaemonNodeEvent::EventStreamDropped { reply_sender } => {
                let inner = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
                    dataflow.subscribe_channels.remove(&node_id);
                    Result::<_, eyre::Error>::Ok(())
                };

                let reply = inner.await.map_err(|err| format!("{err:?}"));
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
        }
        Ok(())
    }

    async fn send_reload(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<(), eyre::ErrReport> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("Reload failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        if let Some(channel) = dataflow.subscribe_channels.get(&node_id) {
            match send_with_timestamp(
                channel,
                daemon_messages::NodeEvent::Reload { operator_id },
                &self.clock,
            ) {
                Ok(()) => {}
                Err(_) => {
                    dataflow.subscribe_channels.remove(&node_id);
                }
            }
        }
        Ok(())
    }

    async fn send_out(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        metadata: dora_core::message::Metadata,
        data: Option<DataMessage>,
    ) -> Result<(), eyre::ErrReport> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        let data_bytes = send_output_to_local_receivers(
            node_id.clone(),
            output_id.clone(),
            dataflow,
            &metadata,
            data,
            &self.clock,
        )
        .await?;

        let output_id = OutputId(node_id, output_id);
        let remote_receivers: Vec<_> = dataflow
            .open_external_mappings
            .get(&output_id)
            .map(|m| m.keys().cloned().collect())
            .unwrap_or_default();
        if !remote_receivers.is_empty() {
            let event = Timestamped {
                inner: InterDaemonEvent::Output {
                    dataflow_id,
                    node_id: output_id.0,
                    output_id: output_id.1,
                    metadata,
                    data: data_bytes,
                },
                timestamp: self.clock.new_timestamp(),
            };
            inter_daemon::send_inter_daemon_event(
                &remote_receivers,
                &mut self.inter_daemon_connections,
                &event,
            )
            .await
            .wrap_err("failed to forward output to remote receivers")?;
        }

        Ok(())
    }

    async fn subscribe(
        dataflow: &mut RunningDataflow,
        node_id: NodeId,
        event_sender: UnboundedSender<Timestamped<daemon_messages::NodeEvent>>,
        clock: &HLC,
    ) {
        // some inputs might have been closed already -> report those events
        let closed_inputs = dataflow
            .mappings
            .values()
            .flatten()
            .filter(|(node, _)| node == &node_id)
            .map(|(_, input)| input)
            .filter(|input| {
                dataflow
                    .open_inputs
                    .get(&node_id)
                    .map(|open_inputs| !open_inputs.contains(*input))
                    .unwrap_or(true)
            });
        for input_id in closed_inputs {
            let _ = send_with_timestamp(
                &event_sender,
                daemon_messages::NodeEvent::InputClosed {
                    id: input_id.clone(),
                },
                clock,
            );
        }
        if dataflow.open_inputs(&node_id).is_empty() {
            let _ = send_with_timestamp(
                &event_sender,
                daemon_messages::NodeEvent::AllInputsClosed,
                clock,
            );
        }

        // if a stop event was already sent for the dataflow, send it to
        // the newly connected node too
        if dataflow.stop_sent {
            let _ = send_with_timestamp(&event_sender, daemon_messages::NodeEvent::Stop, clock);
        }

        dataflow.subscribe_channels.insert(node_id, event_sender);
    }

    #[tracing::instrument(skip(dataflow, inter_daemon_connections, clock), fields(uuid = %dataflow.id), level = "trace")]
    async fn handle_outputs_done(
        dataflow: &mut RunningDataflow,
        inter_daemon_connections: &mut BTreeMap<String, InterDaemonConnection>,
        node_id: &NodeId,
        clock: &HLC,
    ) -> eyre::Result<()> {
        send_input_closed_events(
            dataflow,
            inter_daemon_connections,
            |OutputId(source_id, _)| source_id == node_id,
            clock,
        )
        .await?;
        dataflow.drop_channels.remove(node_id);
        Ok(())
    }

    async fn handle_node_stop(&mut self, dataflow_id: Uuid, node_id: &NodeId) -> eyre::Result<()> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`")
        })?;

        let log_messages = dataflow
            .pending_nodes
            .handle_node_stop(
                node_id,
                &mut self.coordinator_connection,
                &self.clock,
                &mut dataflow.cascading_error_causes,
            )
            .await?;

        Self::handle_outputs_done(
            dataflow,
            &mut self.inter_daemon_connections,
            node_id,
            &self.clock,
        )
        .await?;

        dataflow.running_nodes.remove(node_id);
        if dataflow
            .running_nodes
            .iter()
            .all(|(_id, n)| n.node_config.dynamic)
        {
            let result = DataflowDaemonResult {
                timestamp: self.clock.new_timestamp(),
                node_results: self
                    .dataflow_node_results
                    .get(&dataflow.id)
                    .context("failed to get dataflow node results")?
                    .clone(),
            };

            tracing::info!(
                "Dataflow `{dataflow_id}` finished on machine `{}`",
                self.machine_id
            );
            if let Some(connection) = &mut self.coordinator_connection {
                let msg = serde_json::to_vec(&Timestamped {
                    inner: CoordinatorRequest::Event {
                        machine_id: self.machine_id.clone(),
                        event: DaemonEvent::AllNodesFinished {
                            dataflow_id,
                            result,
                        },
                    },
                    timestamp: self.clock.new_timestamp(),
                })?;
                socket_stream_send(connection, &msg)
                    .await
                    .wrap_err("failed to report dataflow finish to dora-coordinator")?;
            }
            self.running.remove(&dataflow_id);
        }

        for log_message in log_messages {
            self.send_log_message(log_message).await?;
        }

        Ok(())
    }

    async fn handle_dora_event(&mut self, event: DoraEvent) -> eyre::Result<RunStatus> {
        match event {
            DoraEvent::Timer {
                dataflow_id,
                interval,
                metadata,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Timer event for unknown dataflow `{dataflow_id}`");
                    return Ok(RunStatus::Continue);
                };

                let Some(subscribers) = dataflow.timers.get(&interval) else {
                    return Ok(RunStatus::Continue);
                };

                let mut closed = Vec::new();
                for (receiver_id, input_id) in subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        continue;
                    };

                    let send_result = send_with_timestamp(
                        channel,
                        daemon_messages::NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: metadata.clone(),
                            data: None,
                        },
                        &self.clock,
                    );
                    match send_result {
                        Ok(()) => {}
                        Err(_) => {
                            closed.push(receiver_id);
                        }
                    }
                }
                for id in closed {
                    dataflow.subscribe_channels.remove(id);
                }
            }
            DoraEvent::Logs {
                dataflow_id,
                output_id,
                message,
                metadata,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Logs event for unknown dataflow `{dataflow_id}`");
                    return Ok(RunStatus::Continue);
                };

                let Some(subscribers) = dataflow.mappings.get(&output_id) else {
                    tracing::warn!(
                        "No subscribers found for {:?} in {:?}",
                        output_id,
                        dataflow.mappings
                    );
                    return Ok(RunStatus::Continue);
                };

                let mut closed = Vec::new();
                for (receiver_id, input_id) in subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        tracing::warn!("No subscriber channel found for {:?}", output_id);
                        continue;
                    };

                    let send_result = send_with_timestamp(
                        channel,
                        daemon_messages::NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: metadata.clone(),
                            data: Some(message.clone()),
                        },
                        &self.clock,
                    );
                    match send_result {
                        Ok(()) => {}
                        Err(_) => {
                            closed.push(receiver_id);
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
                exit_status,
            } => {
                let node_result = match exit_status {
                    NodeExitStatus::Success => {
                        tracing::info!("node {dataflow_id}/{node_id} finished successfully");
                        Ok(())
                    }
                    exit_status => {
                        let dataflow = self.running.get(&dataflow_id);
                        let caused_by_node = dataflow
                            .and_then(|dataflow| {
                                dataflow.cascading_error_causes.error_caused_by(&node_id)
                            })
                            .cloned();
                        let grace_duration_kill = dataflow
                            .map(|d| d.grace_duration_kills.contains(&node_id))
                            .unwrap_or_default();

                        let cause = match caused_by_node {
                            Some(caused_by_node) => {
                                tracing::info!("marking `{node_id}` as cascading error caused by `{caused_by_node}`");
                                NodeErrorCause::Cascading { caused_by_node }
                            }
                            None if grace_duration_kill => NodeErrorCause::GraceDuration,
                            None => NodeErrorCause::Other {
                                stderr: dataflow
                                    .and_then(|d| d.node_stderr_most_recent.get(&node_id))
                                    .map(|queue| {
                                        let mut s = if queue.is_full() {
                                            "[...]".into()
                                        } else {
                                            String::new()
                                        };
                                        while let Some(line) = queue.pop() {
                                            s += &line;
                                        }
                                        s
                                    })
                                    .unwrap_or_default(),
                            },
                        };
                        Err(NodeError {
                            timestamp: self.clock.new_timestamp(),
                            cause,
                            exit_status,
                        })
                    }
                };

                self.send_log_message(LogMessage {
                    dataflow_id,
                    node_id: Some(node_id.clone()),
                    level: if node_result.is_ok() {
                        Level::Info
                    } else {
                        Level::Error
                    },
                    target: None,
                    module_path: None,
                    file: None,
                    line: None,
                    message: match &node_result {
                        Ok(()) => "node finished successfully".to_string(),
                        Err(err) => format!("{err}"),
                    },
                })
                .await?;

                self.dataflow_node_results
                    .entry(dataflow_id)
                    .or_default()
                    .insert(node_id.clone(), node_result);

                self.handle_node_stop(dataflow_id, &node_id).await?;

                if let Some(exit_when_done) = &mut self.exit_when_done {
                    exit_when_done.remove(&(dataflow_id, node_id));
                    if exit_when_done.is_empty() {
                        tracing::info!(
                            "exiting daemon because all required dataflows are finished"
                        );
                        return Ok(RunStatus::Exit);
                    }
                }
            }
        }
        Ok(RunStatus::Continue)
    }
}

async fn send_output_to_local_receivers(
    node_id: NodeId,
    output_id: DataId,
    dataflow: &mut RunningDataflow,
    metadata: &dora_core::message::Metadata,
    data: Option<DataMessage>,
    clock: &HLC,
) -> Result<Option<AVec<u8, ConstAlign<128>>>, eyre::ErrReport> {
    let timestamp = metadata.timestamp();
    let empty_set = BTreeSet::new();
    let output_id = OutputId(node_id, output_id);
    let local_receivers = dataflow.mappings.get(&output_id).unwrap_or(&empty_set);
    let OutputId(node_id, _) = output_id;
    let mut closed = Vec::new();
    for (receiver_id, input_id) in local_receivers {
        if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
            let item = daemon_messages::NodeEvent::Input {
                id: input_id.clone(),
                metadata: metadata.clone(),
                data: data.clone(),
            };
            match channel.send(Timestamped {
                inner: item,
                timestamp,
            }) {
                Ok(()) => {
                    if let Some(token) = data.as_ref().and_then(|d| d.drop_token()) {
                        dataflow
                            .pending_drop_tokens
                            .entry(token)
                            .or_insert_with(|| DropTokenInformation {
                                owner: node_id.clone(),
                                pending_nodes: Default::default(),
                            })
                            .pending_nodes
                            .insert(receiver_id.clone());
                    }
                }
                Err(_) => {
                    closed.push(receiver_id);
                }
            }
        }
    }
    for id in closed {
        dataflow.subscribe_channels.remove(id);
    }
    let (data_bytes, drop_token) = match data {
        None => (None, None),
        Some(DataMessage::SharedMemory {
            shared_memory_id,
            len,
            drop_token,
        }) => {
            let memory = ShmemConf::new()
                .os_id(shared_memory_id)
                .open()
                .wrap_err("failed to map shared memory output")?;
            let data = Some(AVec::from_slice(1, &unsafe { memory.as_slice() }[..len]));
            (data, Some(drop_token))
        }
        Some(DataMessage::Vec(v)) => (Some(v), None),
    };
    if let Some(token) = drop_token {
        // insert token into `pending_drop_tokens` even if there are no local subscribers
        dataflow
            .pending_drop_tokens
            .entry(token)
            .or_insert_with(|| DropTokenInformation {
                owner: node_id.clone(),
                pending_nodes: Default::default(),
            });
        // check if all local subscribers are finished with the token
        dataflow.check_drop_token(token, clock).await?;
    }
    Ok(data_bytes)
}

fn node_inputs(node: &ResolvedNode) -> BTreeMap<DataId, Input> {
    match &node.kind {
        CoreNodeKind::Custom(n) => n.run_config.inputs.clone(),
        CoreNodeKind::Runtime(n) => runtime_node_inputs(n),
    }
}

async fn send_input_closed_events<F>(
    dataflow: &mut RunningDataflow,
    inter_daemon_connections: &mut BTreeMap<String, InterDaemonConnection>,
    mut filter: F,
    clock: &HLC,
) -> eyre::Result<()>
where
    F: FnMut(&OutputId) -> bool,
{
    let local_node_inputs: BTreeSet<_> = dataflow
        .mappings
        .iter()
        .filter(|(k, _)| filter(k))
        .flat_map(|(_, v)| v)
        .cloned()
        .collect();
    for (receiver_id, input_id) in &local_node_inputs {
        close_input(dataflow, receiver_id, input_id, clock);
    }

    let mut external_node_inputs = BTreeMap::new();
    for (output_id, mapping) in &mut dataflow.open_external_mappings {
        if filter(output_id) {
            external_node_inputs.append(mapping);
        }
    }
    if !external_node_inputs.is_empty() {
        for (target_machine, inputs) in external_node_inputs {
            let event = Timestamped {
                inner: InterDaemonEvent::InputsClosed {
                    dataflow_id: dataflow.id,
                    inputs,
                },
                timestamp: clock.new_timestamp(),
            };
            inter_daemon::send_inter_daemon_event(
                &[target_machine],
                inter_daemon_connections,
                &event,
            )
            .await
            .wrap_err("failed to sent InputClosed event to remote receiver")?;
        }
    }
    Ok(())
}

fn close_input(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    input_id: &DataId,
    clock: &HLC,
) {
    if let Some(open_inputs) = dataflow.open_inputs.get_mut(receiver_id) {
        if !open_inputs.remove(input_id) {
            return;
        }
    }
    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
        let _ = send_with_timestamp(
            channel,
            daemon_messages::NodeEvent::InputClosed {
                id: input_id.clone(),
            },
            clock,
        );

        if dataflow.open_inputs(receiver_id).is_empty() {
            let _ =
                send_with_timestamp(channel, daemon_messages::NodeEvent::AllInputsClosed, clock);
        }
    }
}

#[derive(Debug, Clone)]
struct RunningNode {
    pid: Option<u32>,
    node_config: NodeConfig,
}

pub struct RunningDataflow {
    id: Uuid,
    /// Local nodes that are not started yet
    pending_nodes: PendingNodes,

    subscribe_channels: HashMap<NodeId, UnboundedSender<Timestamped<daemon_messages::NodeEvent>>>,
    drop_channels: HashMap<NodeId, UnboundedSender<Timestamped<daemon_messages::NodeDropEvent>>>,
    mappings: HashMap<OutputId, BTreeSet<InputId>>,
    timers: BTreeMap<Duration, BTreeSet<InputId>>,
    open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    running_nodes: BTreeMap<NodeId, RunningNode>,

    /// List of all dynamic node IDs.
    ///
    /// We want to treat dynamic nodes differently in some cases, so we need
    /// to know which nodes are dynamic.
    dynamic_nodes: BTreeSet<NodeId>,

    open_external_mappings: HashMap<OutputId, BTreeMap<String, BTreeSet<InputId>>>,

    pending_drop_tokens: HashMap<DropToken, DropTokenInformation>,

    /// Keep handles to all timer tasks of this dataflow to cancel them on drop.
    _timer_handles: Vec<futures::future::RemoteHandle<()>>,
    stop_sent: bool,

    /// Used in `open_inputs`.
    ///
    /// TODO: replace this with a constant once `BTreeSet::new` is `const` on stable.
    empty_set: BTreeSet<DataId>,

    /// Contains the node that caused the error for nodes that experienced a cascading error.
    cascading_error_causes: CascadingErrorCauses,
    grace_duration_kills: Arc<crossbeam_skiplist::SkipSet<NodeId>>,

    node_stderr_most_recent: BTreeMap<NodeId, Arc<ArrayQueue<String>>>,
}

impl RunningDataflow {
    fn new(dataflow_id: Uuid, machine_id: String) -> RunningDataflow {
        Self {
            id: dataflow_id,
            pending_nodes: PendingNodes::new(dataflow_id, machine_id),
            subscribe_channels: HashMap::new(),
            drop_channels: HashMap::new(),
            mappings: HashMap::new(),
            timers: BTreeMap::new(),
            open_inputs: BTreeMap::new(),
            running_nodes: BTreeMap::new(),
            dynamic_nodes: BTreeSet::new(),
            open_external_mappings: HashMap::new(),
            pending_drop_tokens: HashMap::new(),
            _timer_handles: Vec::new(),
            stop_sent: false,
            empty_set: BTreeSet::new(),
            cascading_error_causes: Default::default(),
            grace_duration_kills: Default::default(),
            node_stderr_most_recent: BTreeMap::new(),
        }
    }

    async fn start(
        &mut self,
        events_tx: &mpsc::Sender<Timestamped<Event>>,
        clock: &Arc<HLC>,
    ) -> eyre::Result<()> {
        for interval in self.timers.keys().copied() {
            let events_tx = events_tx.clone();
            let dataflow_id = self.id;
            let clock = clock.clone();
            let task = async move {
                let mut interval_stream = tokio::time::interval(interval);
                let hlc = HLC::default();
                loop {
                    interval_stream.tick().await;

                    let span = tracing::span!(tracing::Level::TRACE, "tick");
                    let _ = span.enter();

                    let metadata = dora_core::message::Metadata::from_parameters(
                        hlc.new_timestamp(),
                        ArrowTypeInfo::empty(),
                        MetadataParameters {
                            watermark: 0,
                            deadline: 0,
                            #[cfg(feature = "telemetry")]
                            open_telemetry_context: serialize_context(&span.context()),
                            #[cfg(not(feature = "telemetry"))]
                            open_telemetry_context: "".into(),
                        },
                    );

                    let event = Timestamped {
                        inner: DoraEvent::Timer {
                            dataflow_id,
                            interval,
                            metadata,
                        }
                        .into(),
                        timestamp: clock.new_timestamp(),
                    };
                    if events_tx.send(event).await.is_err() {
                        break;
                    }
                }
            };
            let (task, handle) = task.remote_handle();
            tokio::spawn(task);
            self._timer_handles.push(handle);
        }

        Ok(())
    }

    async fn stop_all(
        &mut self,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
        grace_duration: Option<Duration>,
    ) -> eyre::Result<()> {
        self.pending_nodes
            .handle_dataflow_stop(
                coordinator_connection,
                clock,
                &mut self.cascading_error_causes,
                &self.dynamic_nodes,
            )
            .await?;

        for (_node_id, channel) in self.subscribe_channels.drain() {
            let _ = send_with_timestamp(&channel, daemon_messages::NodeEvent::Stop, clock);
        }

        let running_nodes = self.running_nodes.clone();
        let grace_duration_kills = self.grace_duration_kills.clone();
        tokio::spawn(async move {
            let duration = grace_duration.unwrap_or(Duration::from_millis(2000));
            tokio::time::sleep(duration).await;
            let mut system = sysinfo::System::new();
            system.refresh_processes();

            for (node, node_details) in running_nodes.iter() {
                if let Some(pid) = node_details.pid {
                    if let Some(process) = system.process(Pid::from(pid as usize)) {
                        grace_duration_kills.insert(node.clone());
                        process.kill();
                        warn!(
                            "{node} was killed due to not stopping within the {:#?} grace period",
                            duration
                        )
                    }
                }
            }
        });
        self.stop_sent = true;
        Ok(())
    }

    fn open_inputs(&self, node_id: &NodeId) -> &BTreeSet<DataId> {
        self.open_inputs.get(node_id).unwrap_or(&self.empty_set)
    }

    async fn check_drop_token(&mut self, token: DropToken, clock: &HLC) -> eyre::Result<()> {
        match self.pending_drop_tokens.entry(token) {
            std::collections::hash_map::Entry::Occupied(entry) => {
                if entry.get().pending_nodes.is_empty() {
                    let (drop_token, info) = entry.remove_entry();
                    let result = match self.drop_channels.get_mut(&info.owner) {
                        Some(channel) => send_with_timestamp(
                            channel,
                            daemon_messages::NodeDropEvent::OutputDropped { drop_token },
                            clock,
                        )
                        .wrap_err("send failed"),
                        None => Err(eyre!("no subscribe channel for node `{}`", &info.owner)),
                    };
                    if let Err(err) = result.wrap_err_with(|| {
                        format!(
                            "failed to report drop token `{drop_token:?}` to owner `{}`",
                            &info.owner
                        )
                    }) {
                        tracing::warn!("{err:?}");
                    }
                }
            }
            std::collections::hash_map::Entry::Vacant(_) => {
                tracing::warn!("check_drop_token called with already closed token")
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct OutputId(NodeId, DataId);
type InputId = (NodeId, DataId);

struct DropTokenInformation {
    /// The node that created the associated drop token.
    owner: NodeId,
    /// Contains the set of pending nodes that still have access to the input
    /// associated with a drop token.
    pending_nodes: BTreeSet<NodeId>,
}

#[derive(Debug)]
pub enum Event {
    Node {
        dataflow_id: DataflowId,
        node_id: NodeId,
        event: DaemonNodeEvent,
    },
    Coordinator(CoordinatorEvent),
    Daemon(InterDaemonEvent),
    Dora(DoraEvent),
    DynamicNode(DynamicNodeEventWrapper),
    HeartbeatInterval,
    CtrlC,
}

impl From<DoraEvent> for Event {
    fn from(event: DoraEvent) -> Self {
        Event::Dora(event)
    }
}

#[derive(Debug)]
pub enum DaemonNodeEvent {
    OutputsDone {
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    Subscribe {
        event_sender: UnboundedSender<Timestamped<daemon_messages::NodeEvent>>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SubscribeDrop {
        event_sender: UnboundedSender<Timestamped<daemon_messages::NodeDropEvent>>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    CloseOutputs {
        outputs: Vec<dora_core::config::DataId>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SendOut {
        output_id: DataId,
        metadata: dora_core::message::Metadata,
        data: Option<DataMessage>,
    },
    ReportDrop {
        tokens: Vec<DropToken>,
    },
    EventStreamDropped {
        reply_sender: oneshot::Sender<DaemonReply>,
    },
}

#[derive(Debug)]
pub enum DoraEvent {
    Timer {
        dataflow_id: DataflowId,
        interval: Duration,
        metadata: dora_core::message::Metadata,
    },
    Logs {
        dataflow_id: DataflowId,
        output_id: OutputId,
        message: DataMessage,
        metadata: Metadata,
    },
    SpawnedNodeResult {
        dataflow_id: DataflowId,
        node_id: NodeId,
        exit_status: NodeExitStatus,
    },
}

#[must_use]
enum RunStatus {
    Continue,
    Exit,
}

fn send_with_timestamp<T>(
    sender: &UnboundedSender<Timestamped<T>>,
    event: T,
    clock: &HLC,
) -> Result<(), mpsc::error::SendError<Timestamped<T>>> {
    sender.send(Timestamped {
        inner: event,
        timestamp: clock.new_timestamp(),
    })
}

fn set_up_ctrlc_handler(
    clock: Arc<HLC>,
) -> Result<impl Stream<Item = Timestamped<Event>>, eyre::ErrReport> {
    let (ctrlc_tx, ctrlc_rx) = mpsc::channel(1);

    let mut ctrlc_sent = false;
    ctrlc::set_handler(move || {
        if ctrlc_sent {
            tracing::warn!("received second ctrlc signal -> aborting immediately");
            std::process::abort();
        } else {
            tracing::info!("received ctrlc signal");
            if ctrlc_tx
                .blocking_send(Timestamped {
                    inner: Event::CtrlC,
                    timestamp: clock.new_timestamp(),
                })
                .is_err()
            {
                tracing::error!("failed to report ctrl-c event to dora-coordinator");
            }

            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    Ok(ReceiverStream::new(ctrlc_rx))
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CascadingErrorCauses {
    caused_by: BTreeMap<NodeId, NodeId>,
}

impl CascadingErrorCauses {
    pub fn experienced_cascading_error(&self, node: &NodeId) -> bool {
        self.caused_by.contains_key(node)
    }

    /// Return the ID of the node that caused a cascading error for the given node, if any.
    pub fn error_caused_by(&self, node: &NodeId) -> Option<&NodeId> {
        self.caused_by.get(node)
    }

    pub fn report_cascading_error(&mut self, causing_node: NodeId, affected_node: NodeId) {
        self.caused_by.entry(affected_node).or_insert(causing_node);
    }
}
