use coordinator::CoordinatorEvent;
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
use shared_mem_handler::SharedMemSample;
use std::{
    borrow::Cow,
    collections::{BTreeMap, BTreeSet, HashMap},
    fmt, io,
    net::SocketAddr,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
use tcp_utils::tcp_receive;
use tokio::{
    fs,
    sync::{mpsc, oneshot},
    time::timeout,
};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use uuid::Uuid;

mod coordinator;
mod listener;
mod shared_mem_handler;
mod spawn;
mod tcp_utils;

pub struct Daemon {
    running: HashMap<DataflowId, RunningDataflow>,

    events_tx: mpsc::Sender<Event>,

    shared_memory_handler: flume::Sender<shared_mem_handler::DaemonEvent>,
    shared_memory_handler_node: flume::Sender<shared_mem_handler::NodeEvent>,

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

        let descriptor = read_descriptor(dataflow_path).await?;
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

        let (shared_memory_handler, shared_memory_daemon_rx) = flume::unbounded();
        let (shared_memory_handler_node, shared_memory_node_rx) = flume::bounded(10);
        let daemon = Self {
            running: HashMap::new(),
            events_tx: dora_events_tx,
            shared_memory_handler,
            shared_memory_handler_node,
            coordinator_addr,
            machine_id,
            exit_when_done,
            dora_runtime_path,
            dataflow_errors: Vec::new(),
        };
        let (shmem_events_tx, shmem_events_rx) = flume::bounded(5);
        tokio::spawn(async {
            let mut handler = shared_mem_handler::SharedMemHandler::new(shmem_events_tx);
            handler
                .run(shared_memory_node_rx, shared_memory_daemon_rx)
                .await;
        });
        let dora_events = ReceiverStream::new(dora_events_rx);
        let shmem_events = shmem_events_rx.into_stream().map(Event::ShmemHandler);
        let watchdog_interval = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(5),
        ))
        .map(|_| Event::WatchdogInterval);
        let events = (
            external_events,
            dora_events,
            shmem_events,
            watchdog_interval,
        )
            .merge();
        daemon.run_inner(events).await
    }

    async fn run_inner(
        mut self,
        incoming_events: impl Stream<Item = Event> + Unpin,
    ) -> eyre::Result<Vec<(Uuid, NodeId, eyre::Report)>> {
        let mut events = incoming_events;

        while let Some(event) = events.next().await {
            let start = Instant::now();

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
                    reply_sender,
                } => {
                    self.handle_node_event(event, dataflow, node_id, reply_sender)
                        .await?
                }
                Event::Dora(event) => match self.handle_dora_event(event).await? {
                    RunStatus::Continue => {}
                    RunStatus::Exit => break,
                },
                Event::ShmemHandler(event) => self.handle_shmem_handler_event(event).await?,
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
                        for (_node_id, channel) in dataflow.subscribe_channels.drain() {
                            let _ = channel.send_async(daemon_messages::NodeEvent::Stop).await;
                        }
                    }
                }
            }

            let elapsed = start.elapsed();
            // if elapsed.as_micros() > 10 {
            //     tracing::debug!("handled event in {elapsed:?}: {event_debug}");
            // }
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

                    for (_node_id, channel) in dataflow.subscribe_channels.drain() {
                        let _ = channel.send_async(daemon_messages::NodeEvent::Stop).await;
                    }
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
        let dataflow = match self.running.entry(dataflow_id) {
            std::collections::hash_map::Entry::Vacant(entry) => entry.insert(Default::default()),
            std::collections::hash_map::Entry::Occupied(_) => {
                bail!("there is already a running dataflow with ID `{dataflow_id}`")
            }
        };
        for node in nodes {
            dataflow.running_nodes.insert(node.id.clone());
            let inputs = node_inputs(&node);

            for (input_id, mapping) in inputs {
                dataflow
                    .open_inputs
                    .entry(node.id.clone())
                    .or_default()
                    .insert(input_id.clone());
                match mapping {
                    InputMapping::User(mapping) => {
                        dataflow
                            .mappings
                            .entry((mapping.source, mapping.output))
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
                self.shared_memory_handler_node.clone(),
                daemon_communication_config,
                self.dora_runtime_path.as_deref(),
            )
            .await
            .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))?;
        }
        for interval in dataflow.timers.keys().copied() {
            let events_tx = self.events_tx.clone();
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
            dataflow._timer_handles.push(handle);
        }
        Ok(())
    }

    async fn handle_node_event(
        &mut self,
        event: DaemonNodeEvent,
        dataflow_id: DataflowId,
        node_id: NodeId,
        reply_sender: oneshot::Sender<DaemonReply>,
    ) -> eyre::Result<()> {
        match event {
            DaemonNodeEvent::Subscribe { event_sender } => {
                let result = match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        dataflow.subscribe_channels.insert(node_id, event_sender);
                        Ok(())
                    }
                    None => Err(format!(
                        "subscribe failed: no running dataflow with ID `{dataflow_id}`"
                    )),
                };
                let _ = reply_sender.send(DaemonReply::Result(result));
            }
            DaemonNodeEvent::CloseOutputs(outputs) => {
                // notify downstream nodes
                let inner = async {
                    let dataflow = self
                    .running
                    .get_mut(&dataflow_id)
                    .wrap_err_with(|| format!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`"))?;
                    send_input_closed_events(dataflow, |(source_id, output_id)| {
                        source_id == &node_id && outputs.contains(output_id)
                    })
                    .await;
                    Result::<_, eyre::Error>::Ok(())
                };

                let reply = inner.await.map_err(|err| format!("{err:?}"));
                let _ = reply_sender.send(DaemonReply::Result(reply));
                // TODO: notify remote nodes
            }
            DaemonNodeEvent::Stopped => {
                tracing::info!("Stopped: {dataflow_id}/{node_id}");

                let _ = reply_sender.send(DaemonReply::Result(Ok(())));

                self.handle_node_stop(dataflow_id, &node_id).await?;
            }
        }
        Ok(())
    }

    #[tracing::instrument(skip(self))]
    async fn handle_node_stop(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
    ) -> Result<(), eyre::ErrReport> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`")
        })?;
        send_input_closed_events(dataflow, |(source_id, _)| source_id == node_id).await;
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

    async fn handle_shmem_handler_event(&mut self, event: ShmemHandlerEvent) -> eyre::Result<()> {
        match event {
            ShmemHandlerEvent::SendOut {
                dataflow_id,
                node_id,
                output_id,
                metadata,
                data,
            } => {
                let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                })?;

                tracing::trace!(
                    "Time between prepare and send out: {:?}",
                    metadata
                        .timestamp()
                        .get_time()
                        .to_system_time()
                        .elapsed()
                        .unwrap()
                );

                // figure out receivers from dataflow graph
                let empty_set = BTreeSet::new();
                let local_receivers = dataflow
                    .mappings
                    .get(&(node_id, output_id))
                    .unwrap_or(&empty_set);

                // send shared memory ID to all local receivers
                let mut closed = Vec::new();
                let mut drop_tokens = Vec::new();
                for (receiver_id, input_id) in local_receivers {
                    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
                        let drop_token = DropToken::generate();
                        let send_result = channel.send_async(daemon_messages::NodeEvent::Input {
                            id: input_id.clone(),
                            metadata: metadata.clone(),
                            data: data.as_ref().map(|data| daemon_messages::InputData {
                                shared_memory_id: data.get_os_id().to_owned(),
                                len: data.len(),
                                drop_token: drop_token.clone(),
                            }),
                        });

                        match timeout(Duration::from_millis(10), send_result).await {
                            Ok(Ok(())) => {
                                drop_tokens.push(drop_token);
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
                let data_bytes = data.as_ref().map(|d| unsafe { d.as_slice() }.to_owned());

                // report drop tokens to shared memory handler
                if let Some(data) = data {
                    if let Err(err) = self
                        .shared_memory_handler
                        .send_async(shared_mem_handler::DaemonEvent::SentOut {
                            data: *data,
                            drop_tokens,
                        })
                        .await
                        .wrap_err("shared mem handler crashed after send out")
                    {
                        tracing::error!("{err:?}");
                    }
                }

                // TODO send `data` via network to all remove receivers
                if let Some(data) = data_bytes {}
            }
            ShmemHandlerEvent::HandlerError(err) => {
                bail!(err.wrap_err("shared memory handler failed"))
            }
        }

        Ok(())
    }
}

fn node_inputs(node: &ResolvedNode) -> BTreeMap<DataId, InputMapping> {
    match &node.kind {
        CoreNodeKind::Custom(n) => n.run_config.inputs.clone(),
        CoreNodeKind::Runtime(n) => runtime_node_inputs(n),
    }
}

fn runtime_node_inputs(n: &dora_core::descriptor::RuntimeNode) -> BTreeMap<DataId, InputMapping> {
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
    F: FnMut(&(NodeId, DataId)) -> bool,
{
    let downstream_nodes: BTreeSet<_> = dataflow
        .mappings
        .iter()
        .filter(|(k, _)| filter(k))
        .flat_map(|(_, v)| v)
        .collect();
    for (receiver_id, input_id) in downstream_nodes {
        if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
            let _ = channel
                .send_async(daemon_messages::NodeEvent::InputClosed {
                    id: input_id.clone(),
                })
                .await;
        };

        if let Some(open_inputs) = dataflow.open_inputs.get_mut(receiver_id) {
            open_inputs.remove(input_id);
            if open_inputs.is_empty() {
                // close the subscriber channel
                dataflow.subscribe_channels.remove(receiver_id);
            }
        }
    }
}

#[derive(Default)]
pub struct RunningDataflow {
    subscribe_channels: HashMap<NodeId, flume::Sender<daemon_messages::NodeEvent>>,
    mappings: HashMap<OutputId, BTreeSet<InputId>>,
    timers: BTreeMap<Duration, BTreeSet<InputId>>,
    open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    running_nodes: BTreeSet<NodeId>,
    /// Keep handles to all timer tasks of this dataflow to cancel them on drop.
    _timer_handles: Vec<futures::future::RemoteHandle<()>>,
}

type OutputId = (NodeId, DataId);
type InputId = (NodeId, DataId);

#[derive(Debug)]
pub enum Event {
    Node {
        dataflow_id: DataflowId,
        node_id: NodeId,
        event: DaemonNodeEvent,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    Coordinator(CoordinatorEvent),
    Dora(DoraEvent),
    ShmemHandler(ShmemHandlerEvent),
    WatchdogInterval,
    CtrlC,
}

impl From<DoraEvent> for Event {
    fn from(event: DoraEvent) -> Self {
        Event::Dora(event)
    }
}
impl From<ShmemHandlerEvent> for Event {
    fn from(event: ShmemHandlerEvent) -> Self {
        Event::ShmemHandler(event)
    }
}

#[derive(Debug)]
pub enum DaemonNodeEvent {
    Stopped,
    Subscribe {
        event_sender: flume::Sender<daemon_messages::NodeEvent>,
    },
    CloseOutputs(Vec<dora_core::config::DataId>),
}

pub enum ShmemHandlerEvent {
    SendOut {
        dataflow_id: DataflowId,
        node_id: NodeId,
        output_id: DataId,
        metadata: dora_core::message::Metadata<'static>,
        data: Option<Box<SharedMemSample>>,
    },
    HandlerError(eyre::ErrReport),
}

impl fmt::Debug for ShmemHandlerEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SendOut {
                dataflow_id,
                node_id,
                output_id,
                metadata,
                data,
            } => f
                .debug_struct("SendOut")
                .field("dataflow_id", dataflow_id)
                .field("node_id", node_id)
                .field("output_id", output_id)
                .field("metadata", metadata)
                .field("data", &data.as_ref().map(|_| "Some(..)").unwrap_or("None"))
                .finish(),
            ShmemHandlerEvent::HandlerError(err) => {
                f.debug_tuple("HandlerError").field(err).finish()
            }
        }
    }
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

type MessageId = String;

#[must_use]
enum RunStatus {
    Continue,
    Exit,
}

pub async fn read_descriptor(file: &Path) -> eyre::Result<Descriptor> {
    let descriptor_file = fs::read(file).await.wrap_err("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}
