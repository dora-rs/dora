use coordinator::CoordinatorEvent;
use dora_core::config::Input;
use dora_core::daemon_messages::Data;
use dora_core::message::uhlc::HLC;
use dora_core::{
    config::{DataId, InputMapping, NodeId},
    coordinator_messages::DaemonEvent,
    daemon_messages::{
        self, DaemonCommunicationConfig, DaemonCoordinatorEvent, DaemonCoordinatorReply,
        DaemonReply, DataflowId, DropToken, SpawnDataflowNodes,
    },
    descriptor::{CoreNodeKind, Descriptor, ResolvedNode},
};
use eyre::{bail, eyre, Context, ContextCompat};
use futures::{future, stream, FutureExt, TryFutureExt};
use futures_concurrency::stream::Merge;
use shared_memory_server::ShmemConf;
use std::collections::HashSet;
use std::{
    borrow::Cow,
    collections::{BTreeMap, BTreeSet, HashMap},
    io,
    net::SocketAddr,
    path::{Path, PathBuf},
    time::Duration,
};
use tcp_utils::tcp_receive;
use tokio::sync::mpsc::UnboundedSender;
use tokio::sync::{mpsc, oneshot};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use uuid::Uuid;

mod coordinator;
mod listener;
mod spawn;
mod tcp_utils;

pub struct Daemon {
    running: HashMap<DataflowId, RunningDataflow>,

    events_tx: mpsc::Sender<Event>,

    coordinator_addr: Option<SocketAddr>,
    machine_id: String,

    /// used for testing and examples
    exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
    /// used to record dataflow results when `exit_when_done` is used
    dataflow_errors: Vec<(Uuid, NodeId, eyre::Report)>,

    dora_runtime_path: Option<PathBuf>,
}

impl Daemon {
    pub async fn run(
        coordinator_addr: SocketAddr,
        machine_id: String,
        dora_runtime_path: Option<PathBuf>,
    ) -> eyre::Result<()> {
        // connect to the coordinator
        let coordinator_events = coordinator::register(coordinator_addr, machine_id.clone())
            .await
            .wrap_err("failed to connect to dora-coordinator")?
            .map(Event::Coordinator);
        Self::run_general(
            coordinator_events,
            Some(coordinator_addr),
            machine_id,
            None,
            dora_runtime_path,
        )
        .await
        .map(|_| ())
    }

    pub async fn run_dataflow(
        dataflow_path: &Path,
        dora_runtime_path: Option<PathBuf>,
    ) -> eyre::Result<()> {
        let working_dir = dataflow_path
            .canonicalize()
            .context("failed to canoncialize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
            .to_owned();

        let descriptor = Descriptor::read(dataflow_path).await?;
        descriptor.check(dataflow_path, dora_runtime_path.clone())?;
        let nodes = descriptor.resolve_aliases();

        let spawn_command = SpawnDataflowNodes {
            dataflow_id: Uuid::new_v4(),
            working_dir,
            nodes,
            daemon_communication: descriptor.daemon_config,
        };

        let exit_when_done = spawn_command
            .nodes
            .iter()
            .map(|n| (spawn_command.dataflow_id, n.id.clone()))
            .collect();
        let (reply_tx, reply_rx) = oneshot::channel();
        let coordinator_events = stream::once(async move {
            Event::Coordinator(CoordinatorEvent {
                event: DaemonCoordinatorEvent::Spawn(spawn_command),
                reply_tx,
            })
        });
        let run_result = Self::run_general(
            Box::pin(coordinator_events),
            None,
            "".into(),
            Some(exit_when_done),
            dora_runtime_path,
        );

        let spawn_result = reply_rx
            .map_err(|err| eyre!("failed to receive spawn result: {err}"))
            .and_then(|r| async {
                match r {
                    DaemonCoordinatorReply::SpawnResult(result) => result.map_err(|err| eyre!(err)),
                    _ => Err(eyre!("unexpected spawn reply")),
                }
            });

        let (dataflow_errors, ()) = future::try_join(run_result, spawn_result).await?;

        if dataflow_errors.is_empty() {
            Ok(())
        } else {
            let mut output = "some nodes failed:".to_owned();
            for (dataflow, node, error) in dataflow_errors {
                use std::fmt::Write;
                write!(&mut output, "\n  - {dataflow}/{node}: {error}").unwrap();
            }
            bail!("{output}");
        }
    }

    async fn run_general(
        external_events: impl Stream<Item = Event> + Unpin,
        coordinator_addr: Option<SocketAddr>,
        machine_id: String,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        dora_runtime_path: Option<PathBuf>,
    ) -> eyre::Result<Vec<(Uuid, NodeId, eyre::Report)>> {
        let (dora_events_tx, dora_events_rx) = mpsc::channel(5);
        let ctrlc_tx = dora_events_tx.clone();
        let mut ctrlc_sent = false;
        ctrlc::set_handler(move || {
            if ctrlc_sent {
                tracing::warn!("received second ctrlc signal -> aborting immediately");
                std::process::abort();
            } else {
                tracing::info!("received ctrlc signal");
                if ctrlc_tx.blocking_send(Event::CtrlC).is_err() {
                    tracing::error!("failed to report ctrl-c event to dora-daemon");
                }
                ctrlc_sent = true;
            }
        })
        .wrap_err("failed to set ctrl-c handler")?;

        let daemon = Self {
            running: HashMap::new(),
            events_tx: dora_events_tx,
            coordinator_addr,
            machine_id,
            exit_when_done,
            dora_runtime_path,
            dataflow_errors: Vec::new(),
        };

        let dora_events = ReceiverStream::new(dora_events_rx);
        let watchdog_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(5),
        ))
        .map(|_| Event::WatchdogInterval);
        let events = (external_events, dora_events, watchdog_interval).merge();
        daemon.run_inner(events).await
    }

    async fn run_inner(
        mut self,
        incoming_events: impl Stream<Item = Event> + Unpin,
    ) -> eyre::Result<Vec<(Uuid, NodeId, eyre::Report)>> {
        let mut events = incoming_events;

        while let Some(event) = events.next().await {
            match event {
                Event::Coordinator(CoordinatorEvent { event, reply_tx }) => {
                    let (reply, status) = self.handle_coordinator_event(event).await;
                    let _ = reply_tx.send(reply);
                    match status {
                        RunStatus::Continue => {}
                        RunStatus::Exit => break,
                    }
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
                Event::WatchdogInterval => {
                    if let Some(addr) = self.coordinator_addr {
                        let mut connection = coordinator::send_event(
                            addr,
                            self.machine_id.clone(),
                            DaemonEvent::Watchdog,
                        )
                        .await
                        .wrap_err("lost connection to coordinator")?;
                        let reply_raw = tcp_receive(&mut connection)
                            .await
                            .wrap_err("lost connection to coordinator")?;
                        let _: dora_core::coordinator_messages::WatchdogAck =
                            serde_json::from_slice(&reply_raw)
                                .wrap_err("received unexpected watchdog reply from coordinator")?;
                    }
                }
                Event::CtrlC => {
                    for dataflow in self.running.values_mut() {
                        dataflow.stop_all().await;
                    }
                }
            }
        }

        Ok(self.dataflow_errors)
    }

    async fn handle_coordinator_event(
        &mut self,
        event: DaemonCoordinatorEvent,
    ) -> (DaemonCoordinatorReply, RunStatus) {
        match event {
            DaemonCoordinatorEvent::Spawn(SpawnDataflowNodes {
                dataflow_id,
                working_dir,
                nodes,
                daemon_communication,
            }) => {
                let result = self
                    .spawn_dataflow(dataflow_id, working_dir, nodes, daemon_communication)
                    .await;
                if let Err(err) = &result {
                    tracing::error!("{err:?}");
                }
                let reply =
                    DaemonCoordinatorReply::SpawnResult(result.map_err(|err| format!("{err:?}")));
                (reply, RunStatus::Continue)
            }
            DaemonCoordinatorEvent::StopDataflow { dataflow_id } => {
                let stop = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
                    dataflow.stop_all().await;
                    Result::<(), eyre::Report>::Ok(())
                };
                let reply = DaemonCoordinatorReply::StopResult(
                    stop.await.map_err(|err| format!("{err:?}")),
                );
                (reply, RunStatus::Continue)
            }
            DaemonCoordinatorEvent::Destroy => {
                tracing::info!("received destroy command -> exiting");
                let reply = DaemonCoordinatorReply::DestroyResult(Ok(()));
                (reply, RunStatus::Exit)
            }
            DaemonCoordinatorEvent::Watchdog => {
                (DaemonCoordinatorReply::WatchdogAck, RunStatus::Continue)
            }
        }
    }

    async fn spawn_dataflow(
        &mut self,
        dataflow_id: uuid::Uuid,
        working_dir: PathBuf,
        nodes: Vec<ResolvedNode>,
        daemon_communication_config: DaemonCommunicationConfig,
    ) -> eyre::Result<()> {
        let dataflow = RunningDataflow::new(dataflow_id, &nodes);
        let dataflow = match self.running.entry(dataflow_id) {
            std::collections::hash_map::Entry::Vacant(entry) => entry.insert(dataflow),
            std::collections::hash_map::Entry::Occupied(_) => {
                bail!("there is already a running dataflow with ID `{dataflow_id}`")
            }
        };
        for node in nodes {
            dataflow.running_nodes.insert(node.id.clone());
            let inputs = node_inputs(&node);

            for (input_id, input) in inputs {
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
            }

            let node_id = node.id.clone();
            spawn::spawn_node(
                dataflow_id,
                &working_dir,
                node,
                self.events_tx.clone(),
                daemon_communication_config,
                self.dora_runtime_path.as_deref(),
            )
            .await
            .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))?;
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
                let result = self
                    .subscribe(dataflow_id, node_id.clone(), event_sender)
                    .await;
                let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!("failed to subscribe: no running dataflow with ID `{dataflow_id}`")
                })?;
                tracing::debug!("node `{node_id}` is ready");

                dataflow
                    .subscribe_replies
                    .insert(node_id.clone(), (reply_sender, result));
                dataflow.pending_nodes.remove(&node_id);
                if dataflow.pending_nodes.is_empty() {
                    // TODO synchronize with dora-coordinator if dataflow is
                    // split across multiple daemons
                    tracing::info!("all nodes are ready, starting dataflow `{dataflow_id}`");
                    dataflow.start(&self.events_tx).await?;
                }
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
                    send_input_closed_events(dataflow, |OutputId(source_id, output_id)| {
                        source_id == &node_id && outputs.contains(output_id)
                    })
                    .await;
                    Result::<_, eyre::Error>::Ok(())
                };

                let reply = inner.await.map_err(|err| format!("{err:?}"));
                let _ = reply_sender.send(DaemonReply::Result(reply));
                // TODO: notify remote nodes
            }
            DaemonNodeEvent::Stopped { reply_sender } => {
                tracing::info!("Stopped: {dataflow_id}/{node_id}");

                let _ = reply_sender.send(DaemonReply::Result(Ok(())));

                self.handle_node_stop(dataflow_id, &node_id).await?;
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
                                dataflow.check_drop_token(token).await?;
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
        }
        Ok(())
    }

    async fn send_out(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        metadata: dora_core::message::Metadata<'static>,
        data: Option<Data>,
    ) -> Result<(), eyre::ErrReport> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        let empty_set = BTreeSet::new();
        let output_id = OutputId(node_id, output_id);
        let local_receivers = { dataflow.mappings.get(&output_id).unwrap_or(&empty_set) };
        let OutputId(node_id, _) = output_id;
        let mut closed = Vec::new();
        for (receiver_id, input_id) in local_receivers {
            if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
                let item = daemon_messages::NodeEvent::Input {
                    id: input_id.clone(),
                    metadata: metadata.clone(),
                    data: data.clone(),
                };
                match channel.send(item) {
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
            Some(Data::SharedMemory {
                shared_memory_id,
                len,
                drop_token,
            }) => {
                let memory = ShmemConf::new()
                    .os_id(shared_memory_id)
                    .open()
                    .wrap_err("failed to map shared memory output")?;
                let data = Some(unsafe { memory.as_slice() }[..len].to_owned());
                (data, Some(drop_token))
            }
            Some(Data::Vec(v)) => (Some(v), None),
        };
        if let Some(token) = drop_token {
            dataflow.check_drop_token(token).await?;
        }
        // TODO: Send the data to remote daemon instances if the dataflow
        // is split across multiple machines
        let _data_bytes = data_bytes;
        Ok(())
    }

    async fn subscribe(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        event_sender: UnboundedSender<daemon_messages::NodeEvent>,
    ) -> Result<(), String> {
        let dataflow = self.running.get_mut(&dataflow_id).ok_or_else(|| {
            format!("subscribe failed: no running dataflow with ID `{dataflow_id}`")
        })?;

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
            let _ = event_sender.send(daemon_messages::NodeEvent::InputClosed {
                id: input_id.clone(),
            });
        }
        if dataflow.open_inputs(&node_id).is_empty() {
            let _ = event_sender.send(daemon_messages::NodeEvent::AllInputsClosed);
        }

        // if a stop event was already sent for the dataflow, send it to
        // the newly connected node too
        if dataflow.stop_sent {
            let _ = event_sender.send(daemon_messages::NodeEvent::Stop);
        }

        dataflow.subscribe_channels.insert(node_id, event_sender);

        Ok(())
    }

    #[tracing::instrument(skip(self), level = "trace")]
    async fn handle_node_stop(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
    ) -> Result<(), eyre::ErrReport> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`")
        })?;
        send_input_closed_events(dataflow, |OutputId(source_id, _)| source_id == node_id).await;
        dataflow.running_nodes.remove(node_id);
        if dataflow.running_nodes.is_empty() {
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

                    let send_result = channel.send(daemon_messages::NodeEvent::Input {
                        id: input_id.clone(),
                        metadata: metadata.clone(),
                        data: None,
                    });
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
                let mut signal_exit = false;
                let node_error = match exit_status {
                    NodeExitStatus::Success => {
                        tracing::info!("node {dataflow_id}/{node_id} finished successfully");
                        None
                    }
                    NodeExitStatus::IoError(err) => {
                        let err = eyre!(err).wrap_err(format!(
                            "I/O error while waiting for node `{dataflow_id}/{node_id}`"
                        ));
                        tracing::error!("{err:?}");
                        Some(err)
                    }
                    NodeExitStatus::ExitCode(code) => {
                        let err =
                            eyre!("node {dataflow_id}/{node_id} finished with exit code {code}");
                        tracing::warn!("{err}");
                        Some(err)
                    }
                    NodeExitStatus::Signal(signal) => {
                        signal_exit = true;
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
                            "node {dataflow_id}/{node_id} finished because of signal `{signal}`"
                        );
                        tracing::warn!("{err}");
                        Some(err)
                    }
                    NodeExitStatus::Unknown => {
                        let err =
                            eyre!("node {dataflow_id}/{node_id} finished with unknown exit code");
                        tracing::warn!("{err}");
                        Some(err)
                    }
                };

                if self
                    .running
                    .get(&dataflow_id)
                    .and_then(|d| d.running_nodes.get(&node_id))
                    .is_some()
                {
                    if !signal_exit {
                        tracing::warn!(
                            "node `{dataflow_id}/{node_id}` finished without sending `Stopped` message"
                        );
                    }
                    self.handle_node_stop(dataflow_id, &node_id).await?;
                }

                if let Some(exit_when_done) = &mut self.exit_when_done {
                    if let Some(err) = node_error {
                        self.dataflow_errors
                            .push((dataflow_id, node_id.clone(), err));
                    }
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

async fn send_input_closed_events<F>(dataflow: &mut RunningDataflow, mut filter: F)
where
    F: FnMut(&OutputId) -> bool,
{
    let downstream_nodes: BTreeSet<_> = dataflow
        .mappings
        .iter()
        .filter(|(k, _)| filter(k))
        .flat_map(|(_, v)| v)
        .collect();
    for (receiver_id, input_id) in downstream_nodes {
        if let Some(open_inputs) = dataflow.open_inputs.get_mut(receiver_id) {
            open_inputs.remove(input_id);
        }
        if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
            let _ = channel.send(daemon_messages::NodeEvent::InputClosed {
                id: input_id.clone(),
            });

            if dataflow.open_inputs(receiver_id).is_empty() {
                let _ = channel.send(daemon_messages::NodeEvent::AllInputsClosed);
            }
        }
    }
}

pub struct RunningDataflow {
    id: Uuid,
    /// Nodes that are not started yet
    pending_nodes: HashSet<NodeId>,
    /// Used to synchronize node starts.
    ///
    /// Subscribe requests block the node until all other nodes are ready too.
    subscribe_replies: HashMap<NodeId, (oneshot::Sender<DaemonReply>, Result<(), String>)>,

    subscribe_channels: HashMap<NodeId, UnboundedSender<daemon_messages::NodeEvent>>,
    mappings: HashMap<OutputId, BTreeSet<InputId>>,
    timers: BTreeMap<Duration, BTreeSet<InputId>>,
    open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    running_nodes: BTreeSet<NodeId>,

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
    fn new(id: Uuid, nodes: &[ResolvedNode]) -> RunningDataflow {
        Self {
            id,
            pending_nodes: nodes.iter().map(|n| n.id.clone()).collect(),
            subscribe_replies: HashMap::new(),
            subscribe_channels: HashMap::new(),
            mappings: HashMap::new(),
            timers: BTreeMap::new(),
            open_inputs: BTreeMap::new(),
            running_nodes: BTreeSet::new(),
            pending_drop_tokens: HashMap::new(),
            _timer_handles: Vec::new(),
            stop_sent: false,
            empty_set: BTreeSet::new(),
        }
    }

    async fn start(&mut self, events_tx: &mpsc::Sender<Event>) -> eyre::Result<()> {
        // answer all subscribe requests
        let subscribe_replies = std::mem::take(&mut self.subscribe_replies);
        for (reply_sender, subscribe_result) in subscribe_replies.into_values() {
            let _ = reply_sender.send(DaemonReply::Result(subscribe_result));
        }

        for interval in self.timers.keys().copied() {
            let events_tx = events_tx.clone();
            let dataflow_id = self.id;
            let task = async move {
                let mut interval_stream = tokio::time::interval(interval);
                let hlc = HLC::default();
                loop {
                    interval_stream.tick().await;

                    let event = DoraEvent::Timer {
                        dataflow_id,
                        interval,
                        metadata: dora_core::message::Metadata::from_parameters(
                            hlc.new_timestamp(),
                            Default::default(),
                        ),
                    };
                    if events_tx.send(event.into()).await.is_err() {
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

    async fn stop_all(&mut self) {
        for (_node_id, channel) in self.subscribe_channels.drain() {
            let _ = channel.send(daemon_messages::NodeEvent::Stop);
        }
        self.stop_sent = true;
    }

    fn open_inputs(&self, node_id: &NodeId) -> &BTreeSet<DataId> {
        self.open_inputs.get(node_id).unwrap_or(&self.empty_set)
    }

    async fn check_drop_token(&mut self, token: DropToken) -> eyre::Result<()> {
        match self.pending_drop_tokens.entry(token) {
            std::collections::hash_map::Entry::Occupied(entry) => {
                if entry.get().pending_nodes.is_empty() {
                    let (drop_token, info) = entry.remove_entry();
                    let result = match self.subscribe_channels.get_mut(&info.owner) {
                        Some(channel) => channel
                            .send(daemon_messages::NodeEvent::OutputDropped { drop_token })
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
    Dora(DoraEvent),
    WatchdogInterval,
    CtrlC,
}

impl From<DoraEvent> for Event {
    fn from(event: DoraEvent) -> Self {
        Event::Dora(event)
    }
}

#[derive(Debug)]
pub enum DaemonNodeEvent {
    Stopped {
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    Subscribe {
        event_sender: UnboundedSender<daemon_messages::NodeEvent>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    CloseOutputs {
        outputs: Vec<dora_core::config::DataId>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SendOut {
        output_id: DataId,
        metadata: dora_core::message::Metadata<'static>,
        data: Option<Data>,
    },
    ReportDrop {
        tokens: Vec<DropToken>,
    },
}

#[derive(Debug)]
pub enum DoraEvent {
    Timer {
        dataflow_id: DataflowId,
        interval: Duration,
        metadata: dora_core::message::Metadata<'static>,
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
