use aligned_vec::{AVec, ConstAlign};
use coordinator::CoordinatorEvent;
use crossbeam::queue::ArrayQueue;
use dora_core::{
    config::{DataId, Input, InputMapping, NodeId, NodeRunConfig, OperatorId},
    descriptor::{
        read_as_descriptor, CoreNodeKind, Descriptor, DescriptorExt, ResolvedNode, RuntimeNode,
        DYNAMIC_SOURCE,
    },
    topics::LOCALHOST,
    uhlc::{self, HLC},
};
use dora_message::{
    common::{
        DaemonId, DataMessage, DropToken, LogLevel, NodeError, NodeErrorCause, NodeExitStatus,
    },
    coordinator_to_cli::DataflowResult,
    coordinator_to_daemon::{DaemonCoordinatorEvent, SpawnDataflowNodes},
    daemon_to_coordinator::{
        CoordinatorRequest, DaemonCoordinatorReply, DaemonEvent, DataflowDaemonResult,
    },
    daemon_to_daemon::InterDaemonEvent,
    daemon_to_node::{DaemonReply, NodeConfig, NodeDropEvent, NodeEvent},
    metadata::{self, ArrowTypeInfo},
    node_to_daemon::{DynamicNodeEvent, Timestamped},
    DataflowId,
};
use dora_node_api::{arrow::datatypes::DataType, Parameter};
use eyre::{bail, eyre, Context, ContextCompat, Result};
use futures::{future, stream, FutureExt, TryFutureExt};
use futures_concurrency::stream::Merge;
use local_listener::DynamicNodeEventWrapper;
use log::{DaemonLogger, DataflowLogger, Logger};
use pending::PendingNodes;
use shared_memory_server::ShmemConf;
use socket_stream_utils::socket_stream_send;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    net::SocketAddr,
    path::{Path, PathBuf},
    pin::pin,
    sync::Arc,
    time::{Duration, Instant},
};
use sysinfo::Pid;
use tokio::{
    fs::File,
    io::AsyncReadExt,
    net::TcpStream,
    sync::{
        broadcast,
        mpsc::{self, UnboundedSender},
        oneshot::{self, Sender},
    },
};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use tracing::{error, warn};
use uuid::{NoContext, Timestamp, Uuid};

mod coordinator;
mod local_listener;
mod log;
mod node_communication;
mod pending;
mod socket_stream_utils;
mod spawn;

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
    daemon_id: DaemonId,

    /// used for testing and examples
    exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
    /// set on ctrl-c
    exit_when_all_finished: bool,
    /// used to record results of local nodes
    dataflow_node_results: BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>,

    clock: Arc<uhlc::HLC>,

    zenoh_session: zenoh::Session,
    remote_daemon_events_tx: Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,

    logger: DaemonLogger,
}

type DaemonRunResult = BTreeMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>;

impl Daemon {
    pub async fn run(
        coordinator_addr: SocketAddr,
        machine_id: Option<String>,
        local_listen_port: u16,
    ) -> eyre::Result<()> {
        let clock = Arc::new(HLC::default());

        let mut ctrlc_events = set_up_ctrlc_handler(clock.clone())?;
        let (remote_daemon_events_tx, remote_daemon_events_rx) = flume::bounded(10);
        let (daemon_id, incoming_events) = {
            let incoming_events = set_up_event_stream(
                coordinator_addr,
                &machine_id,
                &clock,
                remote_daemon_events_rx,
                local_listen_port,
            );

            // finish early if ctrl-c is is pressed during event stream setup
            let ctrl_c = pin!(ctrlc_events.recv());
            match futures::future::select(ctrl_c, pin!(incoming_events)).await {
                future::Either::Left((_ctrl_c, _)) => {
                    tracing::info!("received ctrl-c signal -> stopping daemon");
                    return Ok(());
                }
                future::Either::Right((events, _)) => events?,
            }
        };
        Self::run_general(
            (ReceiverStream::new(ctrlc_events), incoming_events).merge(),
            Some(coordinator_addr),
            daemon_id,
            None,
            clock,
            Some(remote_daemon_events_tx),
        )
        .await
        .map(|_| ())
    }

    pub async fn run_dataflow(dataflow_path: &Path, uv: bool) -> eyre::Result<DataflowResult> {
        let working_dir = dataflow_path
            .canonicalize()
            .context("failed to canonicalize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
            .to_owned();

        let descriptor = read_as_descriptor(dataflow_path).await?;
        descriptor.check(&working_dir)?;
        let nodes = descriptor.resolve_aliases_and_set_defaults()?;

        let dataflow_id = Uuid::new_v7(Timestamp::now(NoContext));
        let spawn_command = SpawnDataflowNodes {
            dataflow_id,
            working_dir,
            spawn_nodes: nodes.keys().cloned().collect(),
            nodes,
            dataflow_descriptor: descriptor,
            uv,
        };

        let clock = Arc::new(HLC::default());

        let ctrlc_events = ReceiverStream::new(set_up_ctrlc_handler(clock.clone())?);

        let exit_when_done = spawn_command
            .nodes
            .values()
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
        let events = (coordinator_events, ctrlc_events).merge();
        let run_result = Self::run_general(
            Box::pin(events),
            None,
            DaemonId::new(None),
            Some(exit_when_done),
            clock.clone(),
            None,
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
        daemon_id: DaemonId,
        exit_when_done: Option<BTreeSet<(Uuid, NodeId)>>,
        clock: Arc<HLC>,
        remote_daemon_events_tx: Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
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

        // additional connection for logging
        let logger_coordinator_connection = match coordinator_addr {
            Some(addr) => {
                let stream = TcpStream::connect(addr)
                    .await
                    .wrap_err("failed to connect log to dora-coordinator")?;
                stream
                    .set_nodelay(true)
                    .wrap_err("failed to set TCP_NODELAY")?;
                Some(stream)
            }
            None => None,
        };

        let zenoh_session = match std::env::var(zenoh::Config::DEFAULT_CONFIG_PATH_ENV) {
            Ok(path) => {
                let zenoh_config = zenoh::Config::from_file(&path)
                    .map_err(|e| eyre!(e))
                    .wrap_err_with(|| format!("failed to read zenoh config from {path}"))?;
                zenoh::open(zenoh_config)
                    .await
                    .map_err(|e| eyre!(e))
                    .context("failed to open zenoh session")?
            }
            Err(std::env::VarError::NotPresent) => {
                let mut zenoh_config = zenoh::Config::default();

                if let Some(addr) = coordinator_addr {
                    // Linkstate make it possible to connect two daemons on different network through a public daemon
                    // TODO: There is currently a CI/CD Error in windows linkstate.
                    if cfg!(not(target_os = "windows")) {
                        zenoh_config
                            .insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
                            .unwrap();
                    }

                    zenoh_config
                        .insert_json5(
                            "connect/endpoints",
                            &format!(
                                r#"{{ router: ["tcp/[::]:7447"], peer: ["tcp/{}:5456"] }}"#,
                                addr.ip()
                            ),
                        )
                        .unwrap();
                    zenoh_config
                        .insert_json5(
                            "listen/endpoints",
                            r#"{ router: ["tcp/[::]:7447"], peer: ["tcp/[::]:5456"] }"#,
                        )
                        .unwrap();
                    if cfg!(target_os = "macos") {
                        warn!("disabling multicast on macos systems. Enable it with the ZENOH_CONFIG env variable or file");
                        zenoh_config
                            .insert_json5("scouting/multicast", r#"{ enabled: false }"#)
                            .unwrap();
                    }
                };
                if let Ok(zenoh_session) = zenoh::open(zenoh_config).await {
                    zenoh_session
                } else {
                    warn!(
                        "failed to open zenoh session, retrying with default config + coordinator"
                    );
                    let mut zenoh_config = zenoh::Config::default();
                    // Linkstate make it possible to connect two daemons on different network through a public daemon
                    // TODO: There is currently a CI/CD Error in windows linkstate.
                    if cfg!(not(target_os = "windows")) {
                        zenoh_config
                            .insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
                            .unwrap();
                    }

                    if let Some(addr) = coordinator_addr {
                        zenoh_config
                            .insert_json5(
                                "connect/endpoints",
                                &format!(
                                    r#"{{ router: ["tcp/[::]:7447"], peer: ["tcp/{}:5456"] }}"#,
                                    addr.ip()
                                ),
                            )
                            .unwrap();
                        if cfg!(target_os = "macos") {
                            warn!("disabling multicast on macos systems. Enable it with the ZENOH_CONFIG env variable or file");
                            zenoh_config
                                .insert_json5("scouting/multicast", r#"{ enabled: false }"#)
                                .unwrap();
                        }
                    }
                    if let Ok(zenoh_session) = zenoh::open(zenoh_config).await {
                        zenoh_session
                    } else {
                        warn!("failed to open zenoh session, retrying with default config");
                        let zenoh_config = zenoh::Config::default();
                        zenoh::open(zenoh_config)
                            .await
                            .map_err(|e| eyre!(e))
                            .context("failed to open zenoh session")?
                    }
                }
            }
            Err(std::env::VarError::NotUnicode(_)) => eyre::bail!(
                "{} env variable is not valid unicode",
                zenoh::Config::DEFAULT_CONFIG_PATH_ENV
            ),
        };
        let (dora_events_tx, dora_events_rx) = mpsc::channel(5);
        let daemon = Self {
            logger: Logger {
                coordinator_connection: logger_coordinator_connection,
                daemon_id: daemon_id.clone(),
                clock: clock.clone(),
            }
            .for_daemon(daemon_id.clone()),
            running: HashMap::new(),
            working_dir: HashMap::new(),
            events_tx: dora_events_tx,
            coordinator_connection,
            last_coordinator_heartbeat: Instant::now(),
            daemon_id,
            exit_when_done,
            exit_when_all_finished: false,
            dataflow_node_results: BTreeMap::new(),
            clock,
            zenoh_session,
            remote_daemon_events_tx,
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

    #[tracing::instrument(skip(incoming_events, self), fields(?self.daemon_id))]
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
                                daemon_id: self.daemon_id.clone(),
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
                    tracing::info!("received ctrlc signal -> stopping all dataflows");
                    for dataflow in self.running.values_mut() {
                        let mut logger = self.logger.for_dataflow(dataflow.id);
                        dataflow
                            .stop_all(
                                &mut self.coordinator_connection,
                                &self.clock,
                                None,
                                &mut logger,
                            )
                            .await?;
                    }
                    self.exit_when_all_finished = true;
                    if self.running.is_empty() {
                        break;
                    }
                }
                Event::SecondCtrlC => {
                    tracing::warn!("received second ctrlc signal -> exit immediately");
                    bail!("received second ctrl-c signal");
                }
                Event::DaemonError(err) => {
                    tracing::error!("Daemon error: {err:?}");
                }
            }
        }

        if let Some(mut connection) = self.coordinator_connection.take() {
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: self.daemon_id.clone(),
                    event: DaemonEvent::Exit,
                },
                timestamp: self.clock.new_timestamp(),
            })?;
            socket_stream_send(&mut connection, &msg)
                .await
                .wrap_err("failed to send Exit message to dora-coordinator")?;
        }

        Ok(self.dataflow_node_results)
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
                dataflow_descriptor,
                spawn_nodes,
                uv,
            }) => {
                match dataflow_descriptor.communication.remote {
                    dora_core::config::RemoteCommunicationConfig::Tcp => {}
                }

                // Use the working directory if it exists, otherwise use the working directory where the daemon is spawned
                let working_dir = if working_dir.exists() {
                    working_dir
                } else {
                    std::env::current_dir().wrap_err("failed to get current working dir")?
                };

                let result = self
                    .spawn_dataflow(
                        dataflow_id,
                        working_dir,
                        nodes,
                        dataflow_descriptor,
                        spawn_nodes,
                        uv,
                    )
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
                let mut logger = self.logger.for_dataflow(dataflow_id);
                logger.log(LogLevel::Debug, None,
                    Some("daemon".into()),
                    format!("received AllNodesReady (exited_before_subscribe: {exited_before_subscribe:?})"
                )).await;
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
                            logger.log(LogLevel::Info, None,
                                Some("daemon".into()),
                                "coordinator reported that all nodes are ready, starting dataflow",
                            ).await;
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
                let mut logger = self.logger.for_dataflow(dataflow_id);
                let dataflow = self
                    .running
                    .get_mut(&dataflow_id)
                    .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"));
                let (reply, future) = match dataflow {
                    Ok(dataflow) => {
                        let future = dataflow.stop_all(
                            &mut self.coordinator_connection,
                            &self.clock,
                            grace_duration,
                            &mut logger,
                        );
                        (Ok(()), Some(future))
                    }
                    Err(err) => (Err(err.to_string()), None),
                };

                let _ = reply_tx
                    .send(Some(DaemonCoordinatorReply::StopResult(reply)))
                    .map_err(|_| error!("could not send stop reply from daemon to coordinator"));

                if let Some(future) = future {
                    future.await?;
                }

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
                    let mut logger = self.logger.for_dataflow(dataflow_id).for_node(node_id);
                    logger
                        .log(LogLevel::Warn, Some("daemon".into()), format!("{err:?}"))
                        .await;
                }
                Ok(())
            }
            InterDaemonEvent::OutputClosed {
                dataflow_id,
                node_id,
                output_id,
            } => {
                let output_id = OutputId(node_id.clone(), output_id);
                let mut logger = self
                    .logger
                    .for_dataflow(dataflow_id)
                    .for_node(node_id.clone());
                logger
                    .log(
                        LogLevel::Debug,
                        Some("daemon".into()),
                        format!("received OutputClosed event for output {output_id:?}"),
                    )
                    .await;

                let inner = async {
                    let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                        format!("send out failed: no running dataflow with ID `{dataflow_id}`")
                    })?;

                    if let Some(inputs) = dataflow.mappings.get(&output_id).cloned() {
                        for (receiver_id, input_id) in &inputs {
                            close_input(dataflow, receiver_id, input_id, &self.clock);
                        }
                    }
                    Result::<(), eyre::Report>::Ok(())
                };
                if let Err(err) = inner
                    .await
                    .wrap_err("failed to handle InputsClosed event sent by coordinator")
                {
                    logger
                        .log(LogLevel::Warn, Some("daemon".into()), format!("{err:?}"))
                        .await;
                }
                Ok(())
            }
        }
    }

    async fn spawn_dataflow(
        &mut self,
        dataflow_id: uuid::Uuid,
        working_dir: PathBuf,
        nodes: BTreeMap<NodeId, ResolvedNode>,
        dataflow_descriptor: Descriptor,
        spawn_nodes: BTreeSet<NodeId>,
        uv: bool,
    ) -> eyre::Result<()> {
        let mut logger = self.logger.for_dataflow(dataflow_id);
        let dataflow =
            RunningDataflow::new(dataflow_id, self.daemon_id.clone(), &dataflow_descriptor);
        let dataflow = match self.running.entry(dataflow_id) {
            std::collections::hash_map::Entry::Vacant(entry) => {
                self.working_dir.insert(dataflow_id, working_dir.clone());
                entry.insert(dataflow)
            }
            std::collections::hash_map::Entry::Occupied(_) => {
                bail!("there is already a running dataflow with ID `{dataflow_id}`")
            }
        };

        let mut stopped = Vec::new();

        // calculate info about mappings
        for node in nodes.values() {
            let local = spawn_nodes.contains(&node.id);

            let inputs = node_inputs(node);
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
                        .insert(OutputId(mapping.source, mapping.output));
                }
            }
        }

        // spawn nodes and set up subscriptions
        for node in nodes.into_values() {
            let mut logger = logger.reborrow().for_node(node.id.clone());
            let local = spawn_nodes.contains(&node.id);
            if local {
                if node.kind.dynamic() {
                    dataflow.dynamic_nodes.insert(node.id.clone());
                } else {
                    dataflow.pending_nodes.insert(node.id.clone());
                }

                let node_id = node.id.clone();
                let node_stderr_most_recent = dataflow
                    .node_stderr_most_recent
                    .entry(node.id.clone())
                    .or_insert_with(|| Arc::new(ArrayQueue::new(STDERR_LOG_LINES)))
                    .clone();
                logger
                    .log(LogLevel::Info, Some("daemon".into()), "spawning")
                    .await;
                match spawn::spawn_node(
                    dataflow_id,
                    &working_dir,
                    node,
                    self.events_tx.clone(),
                    dataflow_descriptor.clone(),
                    self.clock.clone(),
                    node_stderr_most_recent,
                    uv,
                    &mut logger,
                )
                .await
                .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))
                {
                    Ok(running_node) => {
                        dataflow.running_nodes.insert(node_id, running_node);
                    }
                    Err(err) => {
                        logger
                            .log(LogLevel::Error, Some("daemon".into()), format!("{err:?}"))
                            .await;
                        self.dataflow_node_results
                            .entry(dataflow_id)
                            .or_default()
                            .insert(
                                node_id.clone(),
                                Err(NodeError {
                                    timestamp: self.clock.new_timestamp(),
                                    cause: NodeErrorCause::Other {
                                        stderr: format!("spawn failed: {err:?}"),
                                    },
                                    exit_status: NodeExitStatus::Unknown,
                                }),
                            );
                        stopped.push(node_id.clone());
                    }
                }
            } else {
                // wait until node is ready before starting
                dataflow.pending_nodes.set_external_nodes(true);

                // subscribe to all node outputs that are mapped to some local inputs
                for output_id in dataflow.mappings.keys().filter(|o| o.0 == node.id) {
                    let tx = self
                        .remote_daemon_events_tx
                        .clone()
                        .wrap_err("no remote_daemon_events_tx channel")?;
                    let mut finished_rx = dataflow.finished_tx.subscribe();
                    let subscribe_topic = dataflow.output_publish_topic(output_id);
                    tracing::debug!("declaring subscriber on {subscribe_topic}");
                    let subscriber = self
                        .zenoh_session
                        .declare_subscriber(subscribe_topic)
                        .await
                        .map_err(|e| eyre!(e))
                        .wrap_err_with(|| format!("failed to subscribe to {output_id:?}"))?;
                    tokio::spawn(async move {
                        let mut finished = pin!(finished_rx.recv());
                        loop {
                            let finished_or_next =
                                futures::future::select(finished, subscriber.recv_async());
                            match finished_or_next.await {
                                future::Either::Left((finished, _)) => {
                                    match finished {
                                        Err(broadcast::error::RecvError::Closed) => {
                                            tracing::debug!("dataflow finished, breaking from zenoh subscribe task");
                                            break;
                                        }
                                        other => {
                                            tracing::warn!("unexpected return value of dataflow finished_rx channel: {other:?}");
                                            break;
                                        }
                                    }
                                }
                                future::Either::Right((sample, f)) => {
                                    finished = f;
                                    let event = sample.map_err(|e| eyre!(e)).and_then(|s| {
                                        Timestamped::deserialize_inter_daemon_event(
                                            &s.payload().to_bytes(),
                                        )
                                    });
                                    if tx.send_async(event).await.is_err() {
                                        // daemon finished
                                        break;
                                    }
                                }
                            }
                        }
                    });
                }
            }
        }
        for node_id in stopped {
            self.handle_node_stop(dataflow_id, &node_id).await?;
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
                    2.. => Err(format!(
                        "multiple dataflows contains dynamic node id {node_id}. \
                        Please only have one running dataflow with the specified \
                        node id if you want to use dynamic node",
                    )),
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
                        .ok_or_else(|| eyre!("no node with ID `{node_id}`"))
                        .and_then(|r| r)
                        .map_err(|err| {
                            format!(
                                "failed to get dynamic node config within given dataflow: {err}"
                            )
                        }),
                    0 => Err("no node with ID `{node_id}`".to_string()),
                };

                let reply = DaemonReply::NodeConfig {
                    result: node_config,
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
                let mut logger = self.logger.for_dataflow(dataflow_id);
                logger
                    .log(
                        LogLevel::Info,
                        Some(node_id.clone()),
                        Some("daemon".into()),
                        "node is ready",
                    )
                    .await;

                let dataflow = self.running.get_mut(&dataflow_id).ok_or_else(|| {
                    format!("subscribe failed: no running dataflow with ID `{dataflow_id}`")
                });

                match dataflow {
                    Err(err) => {
                        let _ = reply_sender.send(DaemonReply::Result(Err(err)));
                    }
                    Ok(dataflow) => {
                        Self::subscribe(dataflow, node_id.clone(), event_sender, &self.clock).await;

                        let status = dataflow
                            .pending_nodes
                            .handle_node_subscription(
                                node_id.clone(),
                                reply_sender,
                                &mut self.coordinator_connection,
                                &self.clock,
                                &mut dataflow.cascading_error_causes,
                                &mut logger,
                            )
                            .await?;
                        match status {
                            DataflowStatus::AllNodesReady => {
                                logger
                                    .log(
                                        LogLevel::Info,
                                        None,
                                        Some("daemon".into()),
                                        "all nodes are ready, starting dataflow",
                                    )
                                    .await;
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
                });
                let result = match dataflow {
                    Ok(dataflow) => {
                        dataflow.drop_channels.insert(node_id, event_sender);
                        Ok(())
                    }
                    Err(err) => Err(err.to_string()),
                };
                let _ = reply_sender.send(DaemonReply::Result(result));
            }
            DaemonNodeEvent::CloseOutputs {
                outputs,
                reply_sender,
            } => {
                // notify downstream nodes
                let inner = async {
                    self.send_output_closed_events(dataflow_id, node_id, outputs)
                        .await
                };

                let reply = inner.await.map_err(|err| format!("{err:?}"));
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
            DaemonNodeEvent::OutputsDone { reply_sender } => {
                let result = self.handle_outputs_done(dataflow_id, &node_id).await;

                let _ = reply_sender.send(DaemonReply::Result(
                    result.map_err(|err| format!("{err:?}")),
                ));
            }
            DaemonNodeEvent::SendOut {
                output_id,
                metadata,
                data,
            } => self
                .send_out(dataflow_id, node_id, output_id, metadata, data)
                .await
                .context("failed to send out")?,
            DaemonNodeEvent::ReportDrop { tokens } => {
                let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                    format!(
                        "failed to get handle drop tokens: \
                        no running dataflow with ID `{dataflow_id}`"
                    )
                });

                match dataflow {
                    Ok(dataflow) => {
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
                    Err(err) => tracing::warn!("{err:?}"),
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
            match send_with_timestamp(channel, NodeEvent::Reload { operator_id }, &self.clock) {
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
        metadata: dora_message::metadata::Metadata,
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
        let remote_receivers = dataflow.open_external_mappings.contains(&output_id)
            || dataflow.publish_all_messages_to_zenoh;
        if remote_receivers {
            let event = InterDaemonEvent::Output {
                dataflow_id,
                node_id: output_id.0.clone(),
                output_id: output_id.1.clone(),
                metadata,
                data: data_bytes,
            };
            self.send_to_remote_receivers(dataflow_id, &output_id, event)
                .await?;
        }

        Ok(())
    }

    async fn send_to_remote_receivers(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        event: InterDaemonEvent,
    ) -> Result<(), eyre::Error> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;

        // publish via zenoh
        let publisher = match dataflow.publishers.get(output_id) {
            Some(publisher) => publisher,
            None => {
                let publish_topic = dataflow.output_publish_topic(output_id);
                tracing::debug!("declaring publisher on {publish_topic}");
                let publisher = self
                    .zenoh_session
                    .declare_publisher(publish_topic)
                    .await
                    .map_err(|e| eyre!(e))
                    .context("failed to create zenoh publisher")?;
                dataflow.publishers.insert(output_id.clone(), publisher);
                dataflow.publishers.get(output_id).unwrap()
            }
        };

        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        }
        .serialize();
        publisher
            .put(serialized_event)
            .await
            .map_err(|e| eyre!(e))
            .context("zenoh put failed")?;
        Ok(())
    }

    async fn send_output_closed_events(
        &mut self,
        dataflow_id: DataflowId,
        node_id: NodeId,
        outputs: Vec<DataId>,
    ) -> eyre::Result<()> {
        let dataflow = self
            .running
            .get_mut(&dataflow_id)
            .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
        let local_node_inputs: BTreeSet<_> = dataflow
            .mappings
            .iter()
            .filter(|(k, _)| k.0 == node_id && outputs.contains(&k.1))
            .flat_map(|(_, v)| v)
            .cloned()
            .collect();
        for (receiver_id, input_id) in &local_node_inputs {
            close_input(dataflow, receiver_id, input_id, &self.clock);
        }

        let mut closed = Vec::new();
        for output_id in &dataflow.open_external_mappings {
            if output_id.0 == node_id && outputs.contains(&output_id.1) {
                closed.push(output_id.clone());
            }
        }

        for output_id in closed {
            let event = InterDaemonEvent::OutputClosed {
                dataflow_id,
                node_id: output_id.0.clone(),
                output_id: output_id.1.clone(),
            };
            self.send_to_remote_receivers(dataflow_id, &output_id, event)
                .await?;
        }

        Ok(())
    }

    async fn subscribe(
        dataflow: &mut RunningDataflow,
        node_id: NodeId,
        event_sender: UnboundedSender<Timestamped<NodeEvent>>,
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
                NodeEvent::InputClosed {
                    id: input_id.clone(),
                },
                clock,
            );
        }
        if dataflow.open_inputs(&node_id).is_empty() {
            let _ = send_with_timestamp(&event_sender, NodeEvent::AllInputsClosed, clock);
        }

        // if a stop event was already sent for the dataflow, send it to
        // the newly connected node too
        if dataflow.stop_sent {
            let _ = send_with_timestamp(&event_sender, NodeEvent::Stop, clock);
        }

        dataflow.subscribe_channels.insert(node_id, event_sender);
    }

    #[tracing::instrument(skip(self), level = "trace")]
    async fn handle_outputs_done(
        &mut self,
        dataflow_id: DataflowId,
        node_id: &NodeId,
    ) -> eyre::Result<()> {
        let dataflow = self
            .running
            .get_mut(&dataflow_id)
            .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;

        let outputs = dataflow
            .mappings
            .keys()
            .filter(|m| &m.0 == node_id)
            .map(|m| &m.1)
            .cloned()
            .collect();
        self.send_output_closed_events(dataflow_id, node_id.clone(), outputs)
            .await?;

        let dataflow = self
            .running
            .get_mut(&dataflow_id)
            .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
        dataflow.drop_channels.remove(node_id);
        Ok(())
    }

    async fn handle_node_stop(&mut self, dataflow_id: Uuid, node_id: &NodeId) -> eyre::Result<()> {
        let mut logger = self.logger.for_dataflow(dataflow_id);
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`")
        })?;

        dataflow
            .pending_nodes
            .handle_node_stop(
                node_id,
                &mut self.coordinator_connection,
                &self.clock,
                &mut dataflow.cascading_error_causes,
                &mut logger,
            )
            .await?;

        self.handle_outputs_done(dataflow_id, node_id).await?;

        let mut logger = self.logger.for_dataflow(dataflow_id);
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`")
        })?;
        if let Some(mut pid) = dataflow.running_nodes.remove(node_id).and_then(|n| n.pid) {
            pid.mark_as_stopped()
        }
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

            logger
                .log(
                    LogLevel::Info,
                    None,
                    Some("daemon".into()),
                    format!("dataflow finished on machine `{}`", self.daemon_id),
                )
                .await;
            if let Some(connection) = &mut self.coordinator_connection {
                let msg = serde_json::to_vec(&Timestamped {
                    inner: CoordinatorRequest::Event {
                        daemon_id: self.daemon_id.clone(),
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
                        NodeEvent::Input {
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
                        NodeEvent::Input {
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
                let mut logger = self
                    .logger
                    .for_dataflow(dataflow_id)
                    .for_node(node_id.clone());
                logger
                    .log(
                        LogLevel::Debug,
                        Some("daemon".into()),
                        format!("handling node stop with exit status {exit_status:?}"),
                    )
                    .await;

                let node_result = match exit_status {
                    NodeExitStatus::Success => Ok(()),
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
                                logger
                                    .log(
                                        LogLevel::Info,
                                        Some("daemon".into()),
                                        format!("marking `{node_id}` as cascading error caused by `{caused_by_node}`")
                                    )
                                    .await;

                                NodeErrorCause::Cascading { caused_by_node }
                            }
                            None if grace_duration_kill => NodeErrorCause::GraceDuration,
                            None => {
                                let cause = dataflow
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
                                    .unwrap_or_default();

                                NodeErrorCause::Other { stderr: cause }
                            }
                        };
                        Err(NodeError {
                            timestamp: self.clock.new_timestamp(),
                            cause,
                            exit_status,
                        })
                    }
                };

                logger
                    .log(
                        if node_result.is_ok() {
                            LogLevel::Info
                        } else {
                            LogLevel::Error
                        },
                        Some("daemon".into()),
                        match &node_result {
                            Ok(()) => format!("{node_id} finished successfully"),
                            Err(err) => format!("{err}"),
                        },
                    )
                    .await;

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
                if self.exit_when_all_finished && self.running.is_empty() {
                    return Ok(RunStatus::Exit);
                }
            }
        }
        Ok(RunStatus::Continue)
    }
}

async fn set_up_event_stream(
    coordinator_addr: SocketAddr,
    machine_id: &Option<String>,
    clock: &Arc<HLC>,
    remote_daemon_events_rx: flume::Receiver<eyre::Result<Timestamped<InterDaemonEvent>>>,
    // used for dynamic nodes
    local_listen_port: u16,
) -> eyre::Result<(DaemonId, impl Stream<Item = Timestamped<Event>> + Unpin)> {
    let clock_cloned = clock.clone();
    let remote_daemon_events = remote_daemon_events_rx.into_stream().map(move |e| match e {
        Ok(e) => Timestamped {
            inner: Event::Daemon(e.inner),
            timestamp: e.timestamp,
        },
        Err(err) => Timestamped {
            inner: Event::DaemonError(err),
            timestamp: clock_cloned.new_timestamp(),
        },
    });
    let (daemon_id, coordinator_events) =
        coordinator::register(coordinator_addr, machine_id.clone(), clock)
            .await
            .wrap_err("failed to connect to dora-coordinator")?;
    let coordinator_events = coordinator_events.map(
        |Timestamped {
             inner: event,
             timestamp,
         }| Timestamped {
            inner: Event::Coordinator(event),
            timestamp,
        },
    );
    let (events_tx, events_rx) = flume::bounded(10);
    let _listen_port =
        local_listener::spawn_listener_loop((LOCALHOST, local_listen_port).into(), events_tx)
            .await?;
    let dynamic_node_events = events_rx.into_stream().map(|e| Timestamped {
        inner: Event::DynamicNode(e.inner),
        timestamp: e.timestamp,
    });
    let incoming = (
        coordinator_events,
        remote_daemon_events,
        dynamic_node_events,
    )
        .merge();
    Ok((daemon_id, incoming))
}

async fn send_output_to_local_receivers(
    node_id: NodeId,
    output_id: DataId,
    dataflow: &mut RunningDataflow,
    metadata: &metadata::Metadata,
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
            let item = NodeEvent::Input {
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
            NodeEvent::InputClosed {
                id: input_id.clone(),
            },
            clock,
        );

        if dataflow.open_inputs(receiver_id).is_empty() {
            let _ = send_with_timestamp(channel, NodeEvent::AllInputsClosed, clock);
        }
    }
}

#[derive(Debug)]
struct RunningNode {
    pid: Option<ProcessId>,
    node_config: NodeConfig,
}

#[derive(Debug)]
struct ProcessId(Option<u32>);

impl ProcessId {
    pub fn new(process_id: u32) -> Self {
        Self(Some(process_id))
    }

    pub fn mark_as_stopped(&mut self) {
        self.0 = None;
    }

    pub fn kill(&mut self) -> bool {
        if let Some(pid) = self.0 {
            let mut system = sysinfo::System::new();
            system.refresh_processes();

            if let Some(process) = system.process(Pid::from(pid as usize)) {
                process.kill();
                self.mark_as_stopped();
                return true;
            }
        }

        false
    }
}

impl Drop for ProcessId {
    fn drop(&mut self) {
        // kill the process if it's still running
        if let Some(pid) = self.0 {
            if self.kill() {
                warn!("process {pid} was killed on drop because it was still running")
            }
        }
    }
}

pub struct RunningDataflow {
    id: Uuid,
    /// Local nodes that are not started yet
    pending_nodes: PendingNodes,

    subscribe_channels: HashMap<NodeId, UnboundedSender<Timestamped<NodeEvent>>>,
    drop_channels: HashMap<NodeId, UnboundedSender<Timestamped<NodeDropEvent>>>,
    mappings: HashMap<OutputId, BTreeSet<InputId>>,
    timers: BTreeMap<Duration, BTreeSet<InputId>>,
    open_inputs: BTreeMap<NodeId, BTreeSet<DataId>>,
    running_nodes: BTreeMap<NodeId, RunningNode>,

    /// List of all dynamic node IDs.
    ///
    /// We want to treat dynamic nodes differently in some cases, so we need
    /// to know which nodes are dynamic.
    dynamic_nodes: BTreeSet<NodeId>,

    open_external_mappings: BTreeSet<OutputId>,

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

    publishers: BTreeMap<OutputId, zenoh::pubsub::Publisher<'static>>,

    finished_tx: broadcast::Sender<()>,

    publish_all_messages_to_zenoh: bool,
}

impl RunningDataflow {
    fn new(
        dataflow_id: Uuid,
        daemon_id: DaemonId,
        dataflow_descriptor: &Descriptor,
    ) -> RunningDataflow {
        let (finished_tx, _) = broadcast::channel(1);
        Self {
            id: dataflow_id,
            pending_nodes: PendingNodes::new(dataflow_id, daemon_id),
            subscribe_channels: HashMap::new(),
            drop_channels: HashMap::new(),
            mappings: HashMap::new(),
            timers: BTreeMap::new(),
            open_inputs: BTreeMap::new(),
            running_nodes: BTreeMap::new(),
            dynamic_nodes: BTreeSet::new(),
            open_external_mappings: Default::default(),
            pending_drop_tokens: HashMap::new(),
            _timer_handles: Vec::new(),
            stop_sent: false,
            empty_set: BTreeSet::new(),
            cascading_error_causes: Default::default(),
            grace_duration_kills: Default::default(),
            node_stderr_most_recent: BTreeMap::new(),
            publishers: Default::default(),
            finished_tx,
            publish_all_messages_to_zenoh: dataflow_descriptor.debug.publish_all_messages_to_zenoh,
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

                    let mut parameters = BTreeMap::new();
                    parameters.insert(
                        "open_telemetry_context".to_string(),
                        #[cfg(feature = "telemetry")]
                        Parameter::String(serialize_context(&span.context())),
                        #[cfg(not(feature = "telemetry"))]
                        Parameter::String("".into()),
                    );

                    let metadata = metadata::Metadata::from_parameters(
                        hlc.new_timestamp(),
                        empty_type_info(),
                        parameters,
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
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<()> {
        self.pending_nodes
            .handle_dataflow_stop(
                coordinator_connection,
                clock,
                &mut self.cascading_error_causes,
                &self.dynamic_nodes,
                logger,
            )
            .await?;

        for (_node_id, channel) in self.subscribe_channels.drain() {
            let _ = send_with_timestamp(&channel, NodeEvent::Stop, clock);
        }

        let running_processes: Vec<_> = self
            .running_nodes
            .iter_mut()
            .map(|(id, n)| (id.clone(), n.pid.take()))
            .collect();
        let grace_duration_kills = self.grace_duration_kills.clone();
        tokio::spawn(async move {
            let duration = grace_duration.unwrap_or(Duration::from_millis(15000));
            tokio::time::sleep(duration).await;

            for (node, pid) in running_processes {
                if let Some(mut pid) = pid {
                    if pid.kill() {
                        grace_duration_kills.insert(node.clone());
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
                            NodeDropEvent::OutputDropped { drop_token },
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

    fn output_publish_topic(&self, output_id: &OutputId) -> String {
        let network_id = "default";
        let dataflow_id = self.id;
        let OutputId(node_id, output_id) = output_id;
        format!("dora/{network_id}/{dataflow_id}/output/{node_id}/{output_id}")
    }
}

fn empty_type_info() -> ArrowTypeInfo {
    ArrowTypeInfo {
        data_type: DataType::Null,
        len: 0,
        null_count: 0,
        validity: None,
        offset: 0,
        buffer_offsets: Vec::new(),
        child_data: Vec::new(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
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
    SecondCtrlC,
    DaemonError(eyre::Report),
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
        event_sender: UnboundedSender<Timestamped<NodeEvent>>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SubscribeDrop {
        event_sender: UnboundedSender<Timestamped<NodeDropEvent>>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    CloseOutputs {
        outputs: Vec<dora_core::config::DataId>,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SendOut {
        output_id: DataId,
        metadata: metadata::Metadata,
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
        metadata: metadata::Metadata,
    },
    Logs {
        dataflow_id: DataflowId,
        output_id: OutputId,
        message: DataMessage,
        metadata: metadata::Metadata,
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
) -> eyre::Result<tokio::sync::mpsc::Receiver<Timestamped<Event>>> {
    let (ctrlc_tx, ctrlc_rx) = mpsc::channel(1);

    let mut ctrlc_sent = 0;
    ctrlc::set_handler(move || {
        let event = match ctrlc_sent {
            0 => Event::CtrlC,
            1 => Event::SecondCtrlC,
            _ => {
                tracing::warn!("received 3rd ctrlc signal -> aborting immediately");
                std::process::abort();
            }
        };
        if ctrlc_tx
            .blocking_send(Timestamped {
                inner: event,
                timestamp: clock.new_timestamp(),
            })
            .is_err()
        {
            tracing::error!("failed to report ctrl-c event to dora-coordinator");
        }

        ctrlc_sent += 1;
    })
    .wrap_err("failed to set ctrl-c handler")?;

    Ok(ctrlc_rx)
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

fn runtime_node_inputs(n: &RuntimeNode) -> BTreeMap<DataId, Input> {
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

fn runtime_node_outputs(n: &RuntimeNode) -> BTreeSet<DataId> {
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

trait CoreNodeKindExt {
    fn run_config(&self) -> NodeRunConfig;
    fn dynamic(&self) -> bool;
}

impl CoreNodeKindExt for CoreNodeKind {
    fn run_config(&self) -> NodeRunConfig {
        match self {
            CoreNodeKind::Runtime(n) => NodeRunConfig {
                inputs: runtime_node_inputs(n),
                outputs: runtime_node_outputs(n),
            },
            CoreNodeKind::Custom(n) => n.run_config.clone(),
        }
    }

    fn dynamic(&self) -> bool {
        match self {
            CoreNodeKind::Runtime(_n) => false,
            CoreNodeKind::Custom(n) => n.source == DYNAMIC_SOURCE,
        }
    }
}
