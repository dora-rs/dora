use aligned_vec::{AVec, ConstAlign};
use coordinator::CoordinatorEvent;
use dora_core::config::{Input, OperatorId};
use dora_core::coordinator_messages::CoordinatorRequest;
use dora_core::daemon_messages::{DataMessage, InterDaemonEvent, Timestamped};
use dora_core::message::uhlc::{self, HLC};
use dora_core::message::{ArrowTypeInfo, MetadataParameters};
use dora_core::{
    config::{DataId, InputMapping, NodeId},
    coordinator_messages::DaemonEvent,
    daemon_messages::{
        self, DaemonCoordinatorEvent, DaemonCoordinatorReply, DaemonReply, DataflowId, DropToken,
        SpawnDataflowNodes,
    },
    descriptor::{CoreNodeKind, Descriptor, ResolvedNode},
};
use eyre::{bail, eyre, Context, ContextCompat};
use futures::{future, stream, FutureExt, TryFutureExt};
use futures_concurrency::stream::Merge;
use inter_daemon::InterDaemonConnection;
use pending::PendingNodes;
use shared_memory_server::ShmemConf;
use std::env::temp_dir;
use std::sync::Arc;
use std::time::Instant;
use std::{
    borrow::Cow,
    collections::{BTreeMap, BTreeSet, HashMap},
    io,
    net::SocketAddr,
    path::{Path, PathBuf},
    time::Duration,
};
use tcp_utils::tcp_send;
use tokio::fs::File;
use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::sync::mpsc::UnboundedSender;
use tokio::sync::oneshot::Sender;
use tokio::sync::{mpsc, oneshot};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use tracing::error;
use uuid::Uuid;

mod coordinator;
mod inter_daemon;
mod log;
mod node_communication;
mod pending;
mod spawn;
mod tcp_utils;

#[cfg(feature = "telemetry")]
use dora_tracing::telemetry::serialize_context;
#[cfg(feature = "telemetry")]
use tracing_opentelemetry::OpenTelemetrySpanExt;

use crate::pending::DataflowStatus;

pub struct Daemon {
    running: HashMap<DataflowId, RunningDataflow>,

    events_tx: mpsc::Sender<Timestamped<Event>>,

    coordinator_connection: Option<TcpStream>,
    last_coordinator_heartbeat: Instant,
    inter_daemon_connections: BTreeMap<String, InterDaemonConnection>,
    machine_id: String,

    /// used for testing and examples
    exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
    /// used to record dataflow results when `exit_when_done` is used
    dataflow_errors: BTreeMap<Uuid, BTreeMap<NodeId, eyre::Report>>,

    clock: Arc<uhlc::HLC>,
}

impl Daemon {
    pub async fn run(
        coordinator_addr: SocketAddr,
        machine_id: String,
        external_events: impl Stream<Item = Timestamped<Event>> + Unpin,
    ) -> eyre::Result<()> {
        let clock = Arc::new(HLC::default());

        // spawn listen loop
        let (events_tx, events_rx) = flume::bounded(10);
        let listen_socket =
            inter_daemon::spawn_listener_loop(machine_id.clone(), events_tx).await?;
        let daemon_events = events_rx.into_stream().map(|e| Timestamped {
            inner: Event::Daemon(e.inner),
            timestamp: e.timestamp,
        });

        // connect to the coordinator
        let coordinator_events =
            coordinator::register(coordinator_addr, machine_id.clone(), listen_socket, &clock)
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

        Self::run_general(
            (coordinator_events, external_events, daemon_events).merge(),
            Some(coordinator_addr),
            machine_id,
            None,
            clock,
        )
        .await
        .map(|_| ())
    }

    pub async fn run_dataflow(dataflow_path: &Path) -> eyre::Result<()> {
        let working_dir = dataflow_path
            .canonicalize()
            .context("failed to canoncialize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
            .to_owned();

        let descriptor = Descriptor::read(dataflow_path).await?;
        descriptor.check(&working_dir)?;
        let nodes = descriptor.resolve_aliases_and_set_defaults();

        let spawn_command = SpawnDataflowNodes {
            dataflow_id: Uuid::new_v4(),
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
            "".into(),
            Some(exit_when_done),
            clock,
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

        let (dataflow_errors, ()) = future::try_join(run_result, spawn_result).await?;

        if dataflow_errors.is_empty() {
            Ok(())
        } else {
            let mut output = "some nodes failed:".to_owned();
            for (dataflow, node_errors) in dataflow_errors {
                for (node, error) in node_errors {
                    use std::fmt::Write;
                    write!(&mut output, "\n  - {dataflow}/{node}: {error}").unwrap();
                }
            }
            bail!("{output}");
        }
    }

    async fn run_general(
        external_events: impl Stream<Item = Timestamped<Event>> + Unpin,
        coordinator_addr: Option<SocketAddr>,
        machine_id: String,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        clock: Arc<HLC>,
    ) -> eyre::Result<BTreeMap<Uuid, BTreeMap<NodeId, eyre::Report>>> {
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
            events_tx: dora_events_tx,
            coordinator_connection,
            last_coordinator_heartbeat: Instant::now(),
            inter_daemon_connections: BTreeMap::new(),
            machine_id,
            exit_when_done,
            dataflow_errors: BTreeMap::new(),
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
    ) -> eyre::Result<BTreeMap<Uuid, BTreeMap<NodeId, eyre::Report>>> {
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
                Event::HeartbeatInterval => {
                    if let Some(connection) = &mut self.coordinator_connection {
                        let msg = serde_json::to_vec(&Timestamped {
                            inner: CoordinatorRequest::Event {
                                machine_id: self.machine_id.clone(),
                                event: DaemonEvent::Heartbeat,
                            },
                            timestamp: self.clock.new_timestamp(),
                        })?;
                        tcp_send(connection, &msg)
                            .await
                            .wrap_err("failed to send watchdog message to dora-coordinator")?;

                        if self.last_coordinator_heartbeat.elapsed() > Duration::from_secs(20) {
                            bail!("lost connection to coordinator")
                        }
                    }
                }
                Event::CtrlC => {
                    for dataflow in self.running.values_mut() {
                        dataflow.stop_all(&self.clock).await;
                    }
                }
            }
        }

        Ok(self.dataflow_errors)
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
                success,
            } => {
                match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        dataflow
                            .pending_nodes
                            .handle_external_all_nodes_ready(success)
                            .await?;
                        if success {
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
                tokio::spawn(async move {
                    let logs = async {
                        let log_dir = temp_dir();

                        let mut file =
                            File::open(log_dir.join(log::log_path(&dataflow_id, &node_id)))
                                .await
                                .wrap_err("Could not open log file")?;

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
            DaemonCoordinatorEvent::StopDataflow { dataflow_id } => {
                let stop = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
                    dataflow.stop_all(&self.clock).await;
                    Result::<(), eyre::Report>::Ok(())
                };
                let reply = DaemonCoordinatorReply::StopResult(
                    stop.await.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx
                    .send(Some(reply))
                    .map_err(|_| error!("could not send stop reply from daemon to coordinator"));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Destroy => {
                tracing::info!("received destroy command -> exiting");
                let reply = DaemonCoordinatorReply::DestroyResult(Ok(()));
                let _ = reply_tx
                    .send(Some(reply))
                    .map_err(|_| error!("could not send destroy reply from daemon to coordinator"));
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
            std::collections::hash_map::Entry::Vacant(entry) => entry.insert(dataflow),
            std::collections::hash_map::Entry::Occupied(_) => {
                bail!("there is already a running dataflow with ID `{dataflow_id}`")
            }
        };

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

                let node_id = node.id.clone();
                match spawn::spawn_node(
                    dataflow_id,
                    &working_dir,
                    node,
                    self.events_tx.clone(),
                    dataflow_descriptor.clone(),
                    self.clock.clone(),
                )
                .await
                .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))
                {
                    Ok(()) => {
                        dataflow.running_nodes.insert(node_id);
                    }
                    Err(err) => {
                        tracing::error!("{err:?}");
                        dataflow
                            .pending_nodes
                            .handle_node_stop(
                                &node_id,
                                &mut self.coordinator_connection,
                                &self.clock,
                            )
                            .await?;
                    }
                }
            } else {
                dataflow.pending_nodes.set_external_nodes(true);
            }
        }

        Ok(())
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

        dataflow
            .pending_nodes
            .handle_node_stop(node_id, &mut self.coordinator_connection, &self.clock)
            .await?;

        Self::handle_outputs_done(
            dataflow,
            &mut self.inter_daemon_connections,
            node_id,
            &self.clock,
        )
        .await?;

        dataflow.running_nodes.remove(node_id);
        if dataflow.running_nodes.is_empty() {
            let result = match self.dataflow_errors.get(&dataflow.id) {
                None => Ok(()),
                Some(errors) => {
                    let mut output = "some nodes failed:".to_owned();
                    for (node, error) in errors {
                        use std::fmt::Write;
                        write!(&mut output, "\n  - {node}: {error}").unwrap();
                    }
                    Err(output)
                }
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
                tcp_send(connection, &msg)
                    .await
                    .wrap_err("failed to report dataflow finish to dora-coordinator")?;
            }
            self.running.remove(&dataflow_id);
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
            DoraEvent::SpawnedNodeResult {
                dataflow_id,
                node_id,
                exit_status,
            } => {
                let node_error = match exit_status {
                    NodeExitStatus::Success => {
                        tracing::info!("node {dataflow_id}/{node_id} finished successfully");
                        None
                    }
                    NodeExitStatus::IoError(err) => {
                        let err = eyre!(err).wrap_err(format!(
                            "
    I/O error while waiting for node `{dataflow_id}/{node_id}. 

    Check logs using: dora logs {dataflow_id} {node_id}
                            "
                        ));
                        tracing::error!("{err:?}");
                        Some(err)
                    }
                    NodeExitStatus::ExitCode(code) => {
                        let err = eyre!(
                            "
    {dataflow_id}/{node_id} failed with exit code {code}.

    Check logs using: dora logs {dataflow_id} {node_id}
                            "
                        );
                        tracing::error!("{err}");
                        Some(err)
                    }
                    NodeExitStatus::Signal(signal) => {
                        let signal: Cow<_> = match signal {
                            1 => "SIGHUP".into(),
                            2 => "SIGINT".into(),
                            3 => "SIGQUIT".into(),
                            4 => "SIGILL".into(),
                            6 => "SIGABRT".into(),
                            8 => "SIGFPE".into(),
                            9 => "SIGKILL".into(),
                            11 => "SIGSEGV".into(),
                            13 => "SIGPIPE".into(),
                            14 => "SIGALRM".into(),
                            15 => "SIGTERM".into(),
                            22 => "SIGABRT".into(),
                            23 => "NSIG".into(),

                            other => other.to_string().into(),
                        };
                        let err = eyre!(
                            "
    {dataflow_id}/{node_id} failed with signal `{signal}`

    Check logs using: dora logs {dataflow_id} {node_id}
                            "
                        );
                        tracing::error!("{err}");
                        Some(err)
                    }
                    NodeExitStatus::Unknown => {
                        let err = eyre!(
                            "
    {dataflow_id}/{node_id} failed with unknown exit code
    
    Check logs using: dora logs {dataflow_id} {node_id}
                            "
                        );
                        tracing::error!("{err}");
                        Some(err)
                    }
                };

                if let Some(err) = node_error {
                    self.dataflow_errors
                        .entry(dataflow_id)
                        .or_default()
                        .insert(node_id.clone(), err);
                }

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

pub fn run_dora_runtime() -> eyre::Result<()> {
    dora_runtime::main()
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

fn runtime_node_inputs(n: &dora_core::descriptor::RuntimeNode) -> BTreeMap<DataId, Input> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator.config.inputs.iter().map(|(input_id, mapping)| {
                (
                    DataId::from(format!("{}/{input_id}", operator.id)),
                    mapping.clone(),
                )
            })
        })
        .collect()
}

fn runtime_node_outputs(n: &dora_core::descriptor::RuntimeNode) -> BTreeSet<DataId> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator
                .config
                .outputs
                .iter()
                .map(|output_id| DataId::from(format!("{}/{output_id}", operator.id)))
        })
        .collect()
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

pub struct RunningDataflow {
    id: Uuid,
    /// Local nodes that are not started yet
    pending_nodes: PendingNodes,

    subscribe_channels: HashMap<NodeId, UnboundedSender<Timestamped<daemon_messages::NodeEvent>>>,
    drop_channels: HashMap<NodeId, UnboundedSender<Timestamped<daemon_messages::NodeDropEvent>>>,
    mappings: HashMap<OutputId, BTreeSet<InputId>>,
    timers: BTreeMap<Duration, BTreeSet<InputId>>,
    open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    running_nodes: BTreeSet<NodeId>,

    open_external_mappings: HashMap<OutputId, BTreeMap<String, BTreeSet<InputId>>>,

    pending_drop_tokens: HashMap<DropToken, DropTokenInformation>,

    /// Keep handles to all timer tasks of this dataflow to cancel them on drop.
    _timer_handles: Vec<futures::future::RemoteHandle<()>>,
    stop_sent: bool,

    /// Used in `open_inputs`.
    ///
    /// TODO: replace this with a constant once `BTreeSet::new` is `const` on stable.
    empty_set: BTreeSet<DataId>,
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
            running_nodes: BTreeSet::new(),
            open_external_mappings: HashMap::new(),
            pending_drop_tokens: HashMap::new(),
            _timer_handles: Vec::new(),
            stop_sent: false,
            empty_set: BTreeSet::new(),
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

    async fn stop_all(&mut self, clock: &HLC) {
        for (_node_id, channel) in self.subscribe_channels.drain() {
            let _ = send_with_timestamp(&channel, daemon_messages::NodeEvent::Stop, clock);
        }
        self.stop_sent = true;
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
struct OutputId(NodeId, DataId);
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
    SpawnedNodeResult {
        dataflow_id: DataflowId,
        node_id: NodeId,
        exit_status: NodeExitStatus,
    },
}

#[derive(Debug)]
pub enum NodeExitStatus {
    Success,
    IoError(io::Error),
    ExitCode(i32),
    Signal(i32),
    Unknown,
}

impl From<Result<std::process::ExitStatus, io::Error>> for NodeExitStatus {
    fn from(result: Result<std::process::ExitStatus, io::Error>) -> Self {
        match result {
            Ok(status) => {
                if status.success() {
                    NodeExitStatus::Success
                } else if let Some(code) = status.code() {
                    Self::ExitCode(code)
                } else {
                    #[cfg(unix)]
                    {
                        use std::os::unix::process::ExitStatusExt;
                        if let Some(signal) = status.signal() {
                            return Self::Signal(signal);
                        }
                    }
                    Self::Unknown
                }
            }
            Err(err) => Self::IoError(err),
        }
    }
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
