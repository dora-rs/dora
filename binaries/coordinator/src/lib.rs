use crate::{
    server::ControlServer,
    tcp_utils::{tcp_receive, tcp_send},
};
pub use control::ControlEvent;
use dashmap::{DashMap, mapref::one::RefMut};
use dora_core::{
    config::{NodeId, OperatorId},
    descriptor::DescriptorExt,
    uhlc::{self, HLC},
};
use dora_message::{
    BuildId, SessionId,
    cli_to_coordinator::{
        BuildRequest, CliControl, CliControlClient, CliControlRequest, CliControlResponse,
    },
    common::DaemonId,
    coordinator_to_cli::{DataflowResult, LogLevel, LogMessage, StopDataflowReply},
    coordinator_to_daemon::{
        BuildDataflowNodes, DaemonCoordinatorEvent, RegisterResult, Timestamped,
    },
    daemon_to_coordinator::{DaemonCoordinatorReply, DataflowDaemonResult},
    descriptor::{Descriptor, ResolvedNode},
    tarpc::{
        self, ClientMessage, Response, Transport, client,
        server::{BaseChannel, Channel},
        tokio_serde,
    },
};
use eyre::{ContextCompat, Result, WrapErr, bail, eyre};
use futures::{Future, Stream, StreamExt, future, stream::FuturesUnordered};
use futures_concurrency::stream::Merge;
use itertools::Itertools;
use log_subscriber::LogSubscriber;

use std::{
    collections::{BTreeMap, BTreeSet},
    net::SocketAddr,
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::{
    net::TcpStream,
    sync::{mpsc, oneshot},
    task::JoinHandle,
};
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;

mod control;
mod listener;
mod log_subscriber;
mod run;
mod server;
mod state;
mod tcp_utils;

/// Start the coordinator with a TCP listener for control messages. Returns the daemon port and
/// a future that resolves when the coordinator finishes.
pub async fn start(
    bind: SocketAddr,
    bind_control: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(u16, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let tasks = FuturesUnordered::new();
    let control_events = control::control_events(bind_control, &tasks)
        .await
        .wrap_err("failed to create control events")?;

    let (daemon_port, coordinator_state, future) =
        init_coordinator(bind, external_events, control_events, tasks).await?;

    // Bind the tarpc RPC server on the same interface
    let rpc_bind = SocketAddr::new(
        bind_control.ip(),
        dora_core::topics::dora_coordinator_port_rpc(bind_control.port()),
    );
    let listener =
        tarpc::serde_transport::tcp::listen(rpc_bind, tokio_serde::formats::Json::default)
            .await
            .wrap_err("failed to start tarpc server for control messages")?;

    let stream = listener
        // ignore connect errors
        .filter_map(|c| future::ready(c.ok()))
        .map(move |transport| {
            let client_ip = transport.peer_addr().ok().map(|addr| addr.ip());
            serve_control_requests(transport, coordinator_state.clone(), client_ip)
        });
    tokio::spawn(stream.for_each(|handle_connection| async {
        tokio::spawn(handle_connection);
    }));

    Ok((daemon_port, future))
}

/// Start the coordinator with an in-process RPC server instead of a TCP listener.
///
/// Returns the `CliControlClient` to communicate with the RPC server.
///
/// This function is mainly useful for testing.
pub async fn start_with_channel_rpc(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
) -> Result<(CliControlClient, impl Future<Output = eyre::Result<()>>), eyre::ErrReport> {
    let tasks = FuturesUnordered::new();

    let (_daemon_port, coordinator_state, future) =
        init_coordinator(bind, external_events, futures::stream::empty(), tasks).await?;

    // Create an in-process channel-based client (no TCP overhead)
    let (client_transport, server_transport) = tarpc::transport::channel::unbounded();
    tokio::spawn(serve_control_requests(
        server_transport,
        coordinator_state,
        None,
    ));
    let control_client = CliControlClient::new(client::Config::default(), client_transport).spawn();

    Ok((control_client, future))
}

/// Shared coordinator setup used by both [`start`] and [`start_with_channel_rpc`].
async fn init_coordinator(
    bind: SocketAddr,
    external_events: impl Stream<Item = Event> + Unpin,
    control_events: impl Stream<Item = Event> + Unpin,
    mut tasks: FuturesUnordered<JoinHandle<()>>,
) -> Result<(
    u16,
    Arc<state::CoordinatorState>,
    impl Future<Output = eyre::Result<()>>,
)> {
    use tokio_stream::wrappers::TcpListenerStream;

    let daemon_listener = listener::create_listener(bind).await?;
    let daemon_port = daemon_listener
        .local_addr()
        .wrap_err("failed to get local addr of daemon listener")?
        .port();
    let new_daemon_connections = TcpListenerStream::new(daemon_listener).map(|c| {
        c.map(Event::NewDaemonConnection)
            .wrap_err("failed to open connection")
            .unwrap_or_else(Event::DaemonConnectError)
    });

    // Setup ctrl-c handler
    let ctrlc_events = set_up_ctrlc_handler()?;

    let events = (
        external_events,
        new_daemon_connections,
        control_events,
        ctrlc_events,
    )
        .merge();

    let daemon_heartbeat_interval =
        tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(Duration::from_secs(3)))
            .map(|_| Event::DaemonHeartbeatInterval);

    // events that should be aborted on `dora destroy`
    let (abortable_events, abort_handle) =
        futures::stream::abortable((events, daemon_heartbeat_interval).merge());

    let (daemon_events_tx, daemon_events) = tokio::sync::mpsc::channel(100);
    let coordinator_state = Arc::new(state::CoordinatorState {
        clock: Arc::new(HLC::default()),
        running_builds: Default::default(),
        finished_builds: Default::default(),
        running_dataflows: Default::default(),
        dataflow_results: Default::default(),
        archived_dataflows: Default::default(),
        daemon_connections: Default::default(),
        daemon_events_tx,
        abort_handle,
    });

    let state_for_caller = coordinator_state.clone();

    let future = async move {
        start_inner(abortable_events, &tasks, daemon_events, coordinator_state).await?;

        tracing::debug!("coordinator main loop finished, waiting on spawned tasks");
        while let Some(join_result) = tasks.next().await {
            if let Err(err) = join_result {
                tracing::error!("task panicked: {err}");
            }
        }
        tracing::debug!("all spawned tasks finished, exiting..");
        Ok(())
    };
    Ok((daemon_port, state_for_caller, future))
}

/// Serve [`CliControl`] RPC requests from the given transport.
fn serve_control_requests<T>(
    transport: T,
    state: Arc<state::CoordinatorState>,
    client_ip: Option<std::net::IpAddr>,
) -> impl Future<Output = ()>
where
    T: Transport<Response<CliControlResponse>, ClientMessage<CliControlRequest>> + Send + 'static,
    T::Error: std::error::Error + Send + Sync + 'static,
{
    let channel = BaseChannel::with_defaults(transport);
    let server = ControlServer { state, client_ip };
    channel.execute(server.serve()).for_each(|fut| async {
        tokio::spawn(fut);
    })
}

// Resolve the dataflow name.
fn resolve_name(
    name: String,
    running_dataflows: &DashMap<Uuid, RunningDataflow>,
    archived_dataflows: &DashMap<Uuid, ArchivedDataflow>,
) -> eyre::Result<Uuid> {
    let uuids: Vec<_> = running_dataflows
        .iter()
        .filter(|r| r.value().name.as_deref() == Some(name.as_str()))
        .map(|r| *r.key())
        .collect();
    let archived_uuids: Vec<_> = archived_dataflows
        .iter()
        .filter(|r| r.value().name.as_deref() == Some(name.as_str()))
        .map(|r| *r.key())
        .collect();

    if uuids.is_empty() {
        if archived_uuids.is_empty() {
            bail!("no dataflow with name `{name}`");
        } else if let [uuid] = archived_uuids.as_slice() {
            Ok(*uuid)
        } else {
            // TODO: Index the archived dataflows in order to return logs based on the index.
            bail!(
                "multiple archived dataflows found with name `{name}`, Please provide the UUID instead."
            );
        }
    } else if let [uuid] = uuids.as_slice() {
        Ok(*uuid)
    } else {
        bail!("multiple dataflows found with name `{name}`");
    }
}

#[derive(Default)]
struct DaemonConnections {
    daemons: DashMap<DaemonId, DaemonConnection>,
}

impl DaemonConnections {
    fn add(&self, daemon_id: DaemonId, connection: DaemonConnection) {
        let previous = self.daemons.insert(daemon_id.clone(), connection);
        if previous.is_some() {
            tracing::info!("closing previous connection `{daemon_id}` on new register");
        }
    }

    fn get_mut(&self, id: &DaemonId) -> Option<RefMut<'_, DaemonId, DaemonConnection>> {
        self.daemons.get_mut(id)
    }

    /// Get a cloned `Arc<Mutex<TcpStream>>` for the given daemon.
    ///
    /// The DashMap lock is only held long enough to clone the Arc, so this is
    /// safe to use before `.await` points.
    fn get_stream(&self, id: &DaemonId) -> Option<Arc<tokio::sync::Mutex<TcpStream>>> {
        self.daemons.get(id).map(|r| r.stream.clone())
    }

    fn get_matching_daemon_id(&self, machine_id: &str) -> Option<DaemonId> {
        self.daemons
            .iter()
            .find(|r| r.key().matches_machine_id(machine_id))
            .map(|r| r.key().clone())
    }

    fn clear(&self) {
        self.daemons.clear();
    }

    fn is_empty(&self) -> bool {
        self.daemons.is_empty()
    }

    fn keys(&self) -> impl Iterator<Item = DaemonId> {
        self.daemons.iter().map(|r| r.key().clone())
    }

    fn iter(
        &self,
    ) -> impl Iterator<Item = dashmap::mapref::multiple::RefMulti<'_, DaemonId, DaemonConnection>>
    {
        self.daemons.iter()
    }

    fn remove(&self, daemon_id: &DaemonId) -> Option<DaemonConnection> {
        self.daemons
            .remove(daemon_id)
            .map(|(_, connection)| connection)
    }

    fn unnamed(&self) -> impl Iterator<Item = DaemonId> {
        self.daemons
            .iter()
            .filter(|r| r.key().machine_id().is_none())
            .map(|r| r.key().clone())
    }
}

async fn start_inner(
    events: impl Stream<Item = Event> + Unpin,
    tasks: &FuturesUnordered<JoinHandle<()>>,
    daemon_events: tokio::sync::mpsc::Receiver<Event>,
    coordinator_state: Arc<state::CoordinatorState>,
) -> eyre::Result<()> {
    let clock = coordinator_state.clock.clone();

    let daemon_events = ReceiverStream::new(daemon_events);

    let mut events = (events, daemon_events).merge();

    while let Some(event) = events.next().await {
        // used below for measuring the event handling duration
        let start = Instant::now();
        let event_kind = event.kind();

        if event.log() {
            tracing::trace!("Handling event {event:?}");
        }
        match event {
            Event::Close => {
                tracing::info!("Received Close event, shutting down coordinator");
                break;
            }
            Event::NewDaemonConnection(connection) => {
                connection.set_nodelay(true)?;
                let events_tx = coordinator_state.daemon_events_tx.clone();
                if !events_tx.is_closed() {
                    let task = tokio::spawn(listener::handle_connection(
                        connection,
                        events_tx,
                        clock.clone(),
                    ));
                    tasks.push(task);
                } else {
                    tracing::warn!(
                        "ignoring new daemon connection because events_tx was closed already"
                    );
                }
            }
            Event::DaemonConnectError(err) => {
                tracing::warn!("{:?}", err.wrap_err("failed to connect to dora-daemon"));
            }
            Event::Daemon(event) => match event {
                DaemonRequest::Register {
                    machine_id,
                    mut connection,
                    version_check_result,
                } => {
                    let existing = match &machine_id {
                        Some(id) => coordinator_state
                            .daemon_connections
                            .get_matching_daemon_id(id),
                        None => coordinator_state.daemon_connections.unnamed().next(),
                    };
                    let existing_result = if existing.is_some() {
                        Err(format!(
                            "There is already a connected daemon with machine ID `{machine_id:?}`"
                        ))
                    } else {
                        Ok(())
                    };

                    // assign a unique ID to the daemon
                    let daemon_id = DaemonId::new(machine_id);

                    let reply: Timestamped<RegisterResult> = Timestamped {
                        inner: match version_check_result.as_ref().and(existing_result.as_ref()) {
                            Ok(_) => RegisterResult::Ok {
                                daemon_id: daemon_id.clone(),
                            },
                            Err(err) => RegisterResult::Err(err.clone()),
                        },
                        timestamp: clock.new_timestamp(),
                    };

                    let send_result = tcp_send(&mut connection, &serde_json::to_vec(&reply)?)
                        .await
                        .context("tcp send failed");
                    match version_check_result.map_err(|e| eyre!(e)).and(send_result) {
                        Ok(()) => {
                            coordinator_state.daemon_connections.add(
                                daemon_id.clone(),
                                DaemonConnection {
                                    stream: Arc::new(tokio::sync::Mutex::new(connection)),
                                    last_heartbeat: Instant::now(),
                                },
                            );
                        }
                        Err(err) => {
                            tracing::warn!(
                                "failed to register daemon connection for daemon `{daemon_id}`: {err}"
                            );
                        }
                    }
                }
            },
            Event::Dataflow { uuid, event } => match event {
                DataflowEvent::ReadyOnDaemon {
                    daemon_id,
                    exited_before_subscribe,
                } => {
                    // Collect what we need from the DashMap entry, then drop
                    // the lock before doing any async I/O.
                    let send_info = match coordinator_state.running_dataflows.entry(uuid) {
                        dashmap::Entry::Occupied(mut entry) => {
                            let dataflow = entry.get_mut();
                            dataflow.pending_daemons.remove(&daemon_id);
                            dataflow
                                .exited_before_subscribe
                                .extend(exited_before_subscribe);
                            if dataflow.pending_daemons.is_empty() {
                                let exited = dataflow.exited_before_subscribe.clone();
                                let daemons: Vec<DaemonId> =
                                    dataflow.daemons.iter().cloned().collect();
                                Some((exited, daemons))
                            } else {
                                None
                            }
                        }
                        dashmap::Entry::Vacant(_) => {
                            tracing::warn!("dataflow not running on ReadyOnMachine");
                            None
                        }
                    };
                    // DashMap lock is now dropped — safe to do async I/O.
                    if let Some((exited_before_subscribe, daemons)) = send_info {
                        tracing::debug!("sending all nodes ready message to daemons");
                        let message = serde_json::to_vec(&Timestamped {
                            inner: DaemonCoordinatorEvent::AllNodesReady {
                                dataflow_id: uuid,
                                exited_before_subscribe,
                            },
                            timestamp: clock.new_timestamp(),
                        })
                        .wrap_err("failed to serialize AllNodesReady message")?;

                        for daemon_id in &daemons {
                            let stream = coordinator_state.daemon_connections.get_stream(daemon_id);
                            let Some(stream) = stream else {
                                tracing::warn!(
                                    "no daemon connection found for machine `{daemon_id}`"
                                );
                                continue;
                            };
                            let mut stream = stream.lock().await;
                            tcp_send(&mut stream, &message).await.wrap_err_with(|| {
                                format!(
                                    "failed to send AllNodesReady({uuid}) message \
                                    to machine {daemon_id}"
                                )
                            })?;
                        }
                    }
                }
                DataflowEvent::DataflowFinishedOnDaemon { daemon_id, result } => {
                    tracing::debug!(
                        "coordinator received DataflowFinishedOnDaemon ({daemon_id:?}, result: {result:?})"
                    );
                    match coordinator_state.running_dataflows.entry(uuid) {
                        dashmap::Entry::Occupied(mut entry) => {
                            let dataflow = entry.get_mut();
                            dataflow.daemons.remove(&daemon_id);
                            tracing::info!(
                                "removed machine id: {daemon_id} from dataflow: {:#?}",
                                dataflow.uuid
                            );
                            coordinator_state
                                .dataflow_results
                                .entry(uuid)
                                .or_default()
                                .insert(daemon_id, result);

                            if dataflow.daemons.is_empty() {
                                // Archive finished dataflow
                                coordinator_state
                                    .archived_dataflows
                                    .entry(uuid)
                                    .or_insert_with(|| ArchivedDataflow::from(entry.get()));
                                let mut finished_dataflow = entry.remove();
                                let dataflow_id = finished_dataflow.uuid;
                                send_log_message(
                                    &mut finished_dataflow.log_subscribers,
                                    &LogMessage {
                                        build_id: None,
                                        dataflow_id: Some(dataflow_id),
                                        node_id: None,
                                        daemon_id: None,
                                        level: LogLevel::Info.into(),
                                        target: Some("coordinator".into()),
                                        module_path: None,
                                        file: None,
                                        line: None,
                                        message: "dataflow finished".into(),
                                        timestamp: clock
                                            .new_timestamp()
                                            .get_time()
                                            .to_system_time()
                                            .into(),
                                        fields: None,
                                    },
                                )
                                .await;

                                let reply = StopDataflowReply {
                                    uuid,
                                    result: coordinator_state
                                        .dataflow_results
                                        .get(&uuid)
                                        .map(|r| dataflow_result(r.value(), uuid, &clock))
                                        .unwrap_or_else(|| {
                                            DataflowResult::ok_empty(uuid, clock.new_timestamp())
                                        }),
                                };
                                for sender in finished_dataflow.stop_reply_senders {
                                    let _ = sender.send(Ok(reply.clone()));
                                }
                                if !matches!(
                                    finished_dataflow.spawn_result,
                                    CachedResult::Cached { .. }
                                ) {
                                    log::error!("pending spawn result on dataflow finish");
                                }
                            }
                        }
                        dashmap::Entry::Vacant(_) => {
                            tracing::warn!("dataflow not running on DataflowFinishedOnDaemon");
                        }
                    }
                }
            },

            Event::Control(event) => match event {
                ControlEvent::Error(err) => tracing::error!("{err:?}"),
                ControlEvent::LogSubscribe {
                    dataflow_id,
                    level,
                    connection,
                } => {
                    if let Some(mut dataflow) =
                        coordinator_state.running_dataflows.get_mut(&dataflow_id)
                    {
                        dataflow
                            .log_subscribers
                            .push(LogSubscriber::new(level, connection));
                        let buffered = std::mem::take(&mut dataflow.buffered_log_messages);
                        for message in buffered {
                            send_log_message(&mut dataflow.log_subscribers, &message).await;
                        }
                    }
                }
                ControlEvent::BuildLogSubscribe {
                    build_id,
                    level,
                    connection,
                } => {
                    if let Some(mut build) = coordinator_state.running_builds.get_mut(&build_id) {
                        build
                            .log_subscribers
                            .push(LogSubscriber::new(level, connection));
                        let buffered = std::mem::take(&mut build.buffered_log_messages);
                        for message in buffered {
                            send_log_message(&mut build.log_subscribers, &message).await;
                        }
                    }
                }
            },
            Event::DaemonHeartbeatInterval => {
                // Collect daemon info and stream handles while briefly holding
                // the DashMap lock.  Drop the lock before doing any async I/O.
                let daemons_to_check: Vec<(
                    DaemonId,
                    Duration,
                    Arc<tokio::sync::Mutex<TcpStream>>,
                )> = coordinator_state
                    .daemon_connections
                    .iter()
                    .map(|r| {
                        (
                            r.key().clone(),
                            r.value().last_heartbeat.elapsed(),
                            r.value().stream.clone(),
                        )
                    })
                    .collect();
                // DashMap lock is now dropped.

                let mut disconnected = BTreeSet::new();
                for (machine_id, elapsed, stream) in daemons_to_check {
                    if elapsed > Duration::from_secs(15) {
                        tracing::warn!(
                            "no heartbeat message from machine `{machine_id}` since {elapsed:?}",
                        )
                    }
                    if elapsed > Duration::from_secs(30) {
                        disconnected.insert(machine_id);
                        continue;
                    }
                    let result: eyre::Result<()> =
                        tokio::time::timeout(Duration::from_millis(500), async {
                            let mut stream = stream.lock().await;
                            send_heartbeat_message(&mut stream, clock.new_timestamp()).await
                        })
                        .await
                        .wrap_err("timeout")
                        .and_then(|r| r)
                        .wrap_err_with(|| {
                            format!("failed to send heartbeat message to daemon at `{machine_id}`")
                        });
                    if let Err(err) = result {
                        tracing::warn!("{err:?}");
                        disconnected.insert(machine_id);
                    }
                }
                if !disconnected.is_empty() {
                    tracing::error!("Disconnecting daemons that failed watchdog: {disconnected:?}");
                    for machine_id in disconnected {
                        coordinator_state.daemon_connections.remove(&machine_id);
                    }
                }
            }
            Event::CtrlC => {
                tracing::info!("Destroying coordinator after receiving Ctrl-C signal");
                handle_destroy(&coordinator_state).await?;
            }
            Event::DaemonHeartbeat {
                daemon_id: machine_id,
            } => {
                if let Some(mut connection_ref) =
                    coordinator_state.daemon_connections.get_mut(&machine_id)
                {
                    let connection = connection_ref.value_mut();
                    connection.last_heartbeat = Instant::now();
                }
            }
            Event::Log(message) => {
                if let Some(dataflow_id) = &message.dataflow_id {
                    if let Some(mut dataflow) =
                        coordinator_state.running_dataflows.get_mut(dataflow_id)
                    {
                        if dataflow.log_subscribers.is_empty() {
                            // buffer log message until there are subscribers
                            dataflow.buffered_log_messages.push(message);
                        } else {
                            send_log_message(&mut dataflow.log_subscribers, &message).await;
                        }
                    }
                } else if let Some(build_id) = &message.build_id {
                    if let Some(mut build) = coordinator_state.running_builds.get_mut(build_id) {
                        if build.log_subscribers.is_empty() {
                            // buffer log message until there are subscribers
                            build.buffered_log_messages.push(message);
                        } else {
                            send_log_message(&mut build.log_subscribers, &message).await;
                        }
                    }
                }
            }
            Event::DaemonExit { daemon_id } => {
                tracing::info!("Daemon `{daemon_id}` exited");
                coordinator_state.daemon_connections.remove(&daemon_id);
            }
            Event::NodeMetrics {
                dataflow_id,
                metrics,
            } => {
                // Store metrics for this dataflow
                if let Some(mut dataflow) =
                    coordinator_state.running_dataflows.get_mut(&dataflow_id)
                {
                    for (node_id, node_metrics) in metrics {
                        dataflow.node_metrics.insert(node_id, node_metrics);
                    }
                }
            }
            Event::DataflowBuildResult {
                build_id,
                daemon_id,
                result,
            } => match coordinator_state.running_builds.entry(build_id) {
                dashmap::Entry::Occupied(mut entry) => {
                    let build = entry.get_mut();
                    build.pending_build_results.remove(&daemon_id);
                    match result {
                        Ok(()) => {}
                        Err(err) => {
                            build.errors.push(format!("{err:?}"));
                        }
                    };
                    if build.pending_build_results.is_empty() {
                        tracing::info!("dataflow build finished: `{build_id}`");
                        let (build_id, mut build) = entry.remove_entry();
                        let result = if build.errors.is_empty() {
                            Ok(())
                        } else {
                            Err(format!("build failed: {}", build.errors.join("\n\n")))
                        };

                        build
                            .build_result
                            .set_result(Ok(BuildFinishedResult { build_id, result }));

                        coordinator_state
                            .finished_builds
                            .insert(build_id, build.build_result);
                    }
                }
                dashmap::Entry::Vacant(_) => {
                    tracing::warn!(
                        "received DataflowSpawnResult, but no matching dataflow in `running_dataflows` map"
                    );
                }
            },
            Event::DataflowSpawnResult {
                dataflow_id,
                daemon_id,
                result,
            } => match coordinator_state.running_dataflows.get_mut(&dataflow_id) {
                Some(mut dataflow) => {
                    dataflow.pending_spawn_results.remove(&daemon_id);
                    match result {
                        Ok(()) => {
                            if dataflow.pending_spawn_results.is_empty() {
                                tracing::info!("successfully spawned dataflow `{dataflow_id}`",);
                                dataflow.spawn_result.set_result(Ok(dataflow_id));
                            }
                        }
                        Err(err) => {
                            tracing::warn!("error while spawning dataflow `{dataflow_id}`");
                            dataflow.spawn_result.set_result(Err(err));
                        }
                    };
                }
                None => {
                    tracing::warn!(
                        "received DataflowSpawnResult, but no matching dataflow in `running_dataflows` map"
                    );
                }
            },
        }

        // warn if event handling took too long -> the main loop should never be blocked for too long
        let elapsed = start.elapsed();
        if elapsed > Duration::from_millis(100) {
            tracing::warn!(
                "Coordinator took {}ms for handling event: {event_kind}",
                elapsed.as_millis()
            );
        }
    }

    tracing::info!("stopped");

    Ok(())
}

async fn send_log_message(log_subscribers: &mut Vec<LogSubscriber>, message: &LogMessage) {
    for subscriber in log_subscribers.iter_mut() {
        let send_result =
            tokio::time::timeout(Duration::from_millis(100), subscriber.send_message(message));

        if send_result.await.is_err() {
            subscriber.close();
        }
    }
    log_subscribers.retain(|s| !s.is_closed());
}

fn dataflow_result(
    results: &BTreeMap<DaemonId, DataflowDaemonResult>,
    dataflow_uuid: Uuid,
    clock: &uhlc::HLC,
) -> DataflowResult {
    let mut node_results = BTreeMap::new();
    for result in results.values() {
        node_results.extend(result.node_results.clone());
        if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
            tracing::warn!("failed to update HLC: {err}");
        }
    }

    DataflowResult {
        uuid: dataflow_uuid,
        timestamp: clock.new_timestamp(),
        node_results,
    }
}

struct DaemonConnection {
    stream: Arc<tokio::sync::Mutex<TcpStream>>,
    last_heartbeat: Instant,
}

async fn handle_destroy(
    coordinator_state: &state::CoordinatorState,
) -> Result<(), eyre::ErrReport> {
    coordinator_state.abort_handle.abort();
    for dataflow_uuid in coordinator_state
        .running_dataflows
        .iter()
        .map(|entry| *entry.key())
        .collect::<Vec<_>>()
    {
        let _ = stop_dataflow(
            &coordinator_state.running_dataflows,
            dataflow_uuid,
            &coordinator_state.daemon_connections,
            coordinator_state.clock.new_timestamp(),
            None,
            false,
        )
        .await?;
    }

    let result = destroy_daemons(
        &coordinator_state.daemon_connections,
        coordinator_state.clock.new_timestamp(),
    )
    .await;

    let _ = coordinator_state.daemon_events_tx.send(Event::Close).await;
    result
}

async fn send_heartbeat_message(
    connection: &mut TcpStream,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Heartbeat,
        timestamp,
    })
    .context("Could not serialize heartbeat message")?;

    tcp_send(connection, &message)
        .await
        .wrap_err("failed to send heartbeat message to daemon")
}

/// Result of a completed dataflow build.
#[derive(Debug, Clone)]
pub struct BuildFinishedResult {
    pub build_id: BuildId,
    pub result: Result<(), String>,
}

struct RunningBuild {
    errors: Vec<String>,
    build_result: CachedResult<BuildFinishedResult>,

    /// Buffer for log messages that were sent before there were any subscribers.
    buffered_log_messages: Vec<LogMessage>,
    log_subscribers: Vec<LogSubscriber>,

    pending_build_results: BTreeSet<DaemonId>,
}

struct RunningDataflow {
    name: Option<String>,
    uuid: Uuid,
    descriptor: Descriptor,
    /// The IDs of the daemons that the dataflow is running on.
    daemons: BTreeSet<DaemonId>,
    /// IDs of daemons that are waiting until all nodes are started.
    pending_daemons: BTreeSet<DaemonId>,
    exited_before_subscribe: Vec<NodeId>,
    nodes: BTreeMap<NodeId, ResolvedNode>,
    /// Maps each node to the daemon it's running on
    node_to_daemon: BTreeMap<NodeId, DaemonId>,
    /// Latest metrics for each node (from daemons)
    node_metrics: BTreeMap<NodeId, dora_message::daemon_to_coordinator::NodeMetrics>,

    spawn_result: CachedResult<Uuid>,
    stop_reply_senders: Vec<tokio::sync::oneshot::Sender<eyre::Result<StopDataflowReply>>>,

    /// Buffer for log messages that were sent before there were any subscribers.
    buffered_log_messages: Vec<LogMessage>,
    log_subscribers: Vec<LogSubscriber>,

    pending_spawn_results: BTreeSet<DaemonId>,
}

pub enum CachedResult<T> {
    Pending {
        result_senders: Vec<tokio::sync::oneshot::Sender<eyre::Result<T>>>,
    },
    Cached {
        result: eyre::Result<T>,
    },
}

impl<T> Default for CachedResult<T> {
    fn default() -> Self {
        Self::Pending {
            result_senders: Vec::new(),
        }
    }
}

impl<T: Clone> CachedResult<T> {
    fn register(&mut self, reply_sender: tokio::sync::oneshot::Sender<eyre::Result<T>>) {
        match self {
            CachedResult::Pending { result_senders } => result_senders.push(reply_sender),
            CachedResult::Cached { result } => {
                Self::send_result_to(result, reply_sender);
            }
        }
    }

    fn set_result(&mut self, result: eyre::Result<T>) {
        match self {
            CachedResult::Pending { result_senders } => {
                for sender in result_senders.drain(..) {
                    Self::send_result_to(&result, sender);
                }
                *self = CachedResult::Cached { result };
            }
            CachedResult::Cached { .. } => {}
        }
    }

    fn send_result_to(result: &eyre::Result<T>, sender: oneshot::Sender<eyre::Result<T>>) {
        let result = match result {
            Ok(r) => Ok(r.clone()),
            Err(err) => Err(eyre!("{err:?}")),
        };
        let _ = sender.send(result);
    }
}

struct ArchivedDataflow {
    name: Option<String>,
    nodes: BTreeMap<NodeId, ResolvedNode>,
}

impl From<&RunningDataflow> for ArchivedDataflow {
    fn from(dataflow: &RunningDataflow) -> ArchivedDataflow {
        ArchivedDataflow {
            name: dataflow.name.clone(),
            nodes: dataflow.nodes.clone(),
        }
    }
}

impl PartialEq for RunningDataflow {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.uuid == other.uuid && self.daemons == other.daemons
    }
}

impl Eq for RunningDataflow {}

async fn stop_dataflow<'a>(
    running_dataflows: &'a DashMap<Uuid, RunningDataflow>,
    dataflow_uuid: Uuid,
    daemon_connections: &DaemonConnections,
    timestamp: uhlc::Timestamp,
    grace_duration: Option<Duration>,
    force: bool,
) -> eyre::Result<RefMut<'a, Uuid, RunningDataflow>> {
    // Collect daemon list and stream handles while briefly holding the lock.
    let daemon_streams: Vec<(DaemonId, Arc<tokio::sync::Mutex<TcpStream>>)> = {
        let Some(dataflow) = running_dataflows.get(&dataflow_uuid) else {
            bail!("no known running dataflow found with UUID `{dataflow_uuid}`")
        };
        dataflow
            .daemons
            .iter()
            .filter_map(|daemon_id| {
                daemon_connections
                    .get_stream(daemon_id)
                    .map(|s| (daemon_id.clone(), s))
            })
            .collect()
    };

    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id: dataflow_uuid,
            grace_duration,
            force,
        },
        timestamp,
    })?;

    // Send stop commands without holding any DashMap locks.
    for (_daemon_id, stream) in &daemon_streams {
        let mut stream = stream.lock().await;
        tcp_send(&mut stream, &message)
            .await
            .wrap_err("failed to send stop message to daemon")?;

        let reply_raw = tcp_receive(&mut stream)
            .await
            .wrap_err("failed to receive stop reply from daemon")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize stop reply from daemon")?
        {
            DaemonCoordinatorReply::StopResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to stop dataflow")?,
            other => bail!("unexpected reply after sending stop: {other:?}"),
        }
    }

    tracing::info!("successfully send stop dataflow `{dataflow_uuid}` to all daemons");

    // Re-acquire the lock for the caller.
    running_dataflows
        .get_mut(&dataflow_uuid)
        .wrap_err("dataflow was removed while sending stop commands")
}

async fn reload_dataflow(
    running_dataflows: &DashMap<Uuid, RunningDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    operator_id: Option<OperatorId>,
    daemon_connections: &DaemonConnections,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    // Collect daemon streams while briefly holding the lock.
    let daemon_streams: Vec<(DaemonId, Arc<tokio::sync::Mutex<TcpStream>>)> = {
        let Some(dataflow) = running_dataflows.get(&dataflow_id) else {
            bail!("No running dataflow found with UUID `{dataflow_id}`")
        };
        dataflow
            .daemons
            .iter()
            .filter_map(|daemon_id| {
                daemon_connections
                    .get_stream(daemon_id)
                    .map(|s| (daemon_id.clone(), s))
            })
            .collect()
    };

    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::ReloadDataflow {
            dataflow_id,
            node_id,
            operator_id,
        },
        timestamp,
    })?;

    for (_machine_id, stream) in &daemon_streams {
        let mut stream = stream.lock().await;
        tcp_send(&mut stream, &message)
            .await
            .wrap_err("failed to send reload message to daemon")?;

        let reply_raw = tcp_receive(&mut stream)
            .await
            .wrap_err("failed to receive reload reply from daemon")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize reload reply from daemon")?
        {
            DaemonCoordinatorReply::ReloadResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to reload dataflow")?,
            other => bail!("unexpected reply after sending reload: {other:?}"),
        }
    }
    tracing::info!("successfully reloaded dataflow `{dataflow_id}`");

    Ok(())
}

async fn retrieve_logs(
    running_dataflows: &DashMap<Uuid, RunningDataflow>,
    archived_dataflows: &DashMap<Uuid, ArchivedDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    daemon_connections: &DaemonConnections,
    timestamp: uhlc::Timestamp,
    tail: Option<usize>,
) -> eyre::Result<Vec<u8>> {
    let nodes = if let Some(dataflow) = archived_dataflows.get(&dataflow_id) {
        dataflow.nodes.clone()
    } else if let Some(dataflow) = running_dataflows.get(&dataflow_id) {
        dataflow.nodes.clone()
    } else {
        bail!("No dataflow found with UUID `{dataflow_id}`")
    };

    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Logs {
            dataflow_id,
            node_id: node_id.clone(),
            tail,
        },
        timestamp,
    })?;

    let machine_ids: Vec<Option<String>> = nodes
        .values()
        .filter(|node| node.id == node_id)
        .map(|node| node.deploy.as_ref().and_then(|d| d.machine.clone()))
        .collect();

    let machine_id = if let [machine_id] = &machine_ids[..] {
        machine_id
    } else if machine_ids.is_empty() {
        bail!("No machine contains {}/{}", dataflow_id, node_id)
    } else {
        bail!(
            "More than one machine contains {}/{}. However, it should only be present on one.",
            dataflow_id,
            node_id
        )
    };

    let daemon_ids: Vec<_> = match machine_id {
        None => daemon_connections.unnamed().collect(),
        Some(machine_id) => daemon_connections
            .get_matching_daemon_id(machine_id)
            .into_iter()
            .collect(),
    };
    let daemon_id = match &daemon_ids[..] {
        [id] => (*id).clone(),
        [] => eyre::bail!("no matching daemon connections for machine ID `{machine_id:?}`"),
        _ => eyre::bail!("multiple matching daemon connections for machine ID `{machine_id:?}`"),
    };
    let stream = daemon_connections
        .get_stream(&daemon_id)
        .wrap_err_with(|| format!("no daemon connection to `{daemon_id}`"))?;
    let mut stream = stream.lock().await;
    tcp_send(&mut stream, &message)
        .await
        .wrap_err("failed to send logs message to daemon")?;

    // wait for reply
    let reply_raw = tcp_receive(&mut stream)
        .await
        .wrap_err("failed to retrieve logs reply from daemon")?;
    let reply_logs = match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize logs reply from daemon")?
    {
        DaemonCoordinatorReply::Logs(logs) => logs,
        other => bail!("unexpected reply after sending logs: {other:?}"),
    };
    tracing::info!("successfully retrieved logs for `{dataflow_id}/{node_id}`");

    reply_logs.map_err(|err| eyre!(err))
}

#[tracing::instrument(skip(daemon_connections, clock))]
async fn build_dataflow(
    build_request: BuildRequest,
    build_id: BuildId,
    clock: &HLC,
    daemon_connections: &DaemonConnections,
) -> eyre::Result<RunningBuild> {
    let BuildRequest {
        session_id,
        dataflow,
        git_sources,
        prev_git_sources,
        local_working_dir,
        uv,
    } = build_request;

    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

    let mut git_sources_by_daemon = git_sources
        .into_iter()
        .into_grouping_map_by(|(id, _)| {
            nodes
                .get(id)
                .and_then(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()))
        })
        .collect();
    let mut prev_git_sources_by_daemon = prev_git_sources
        .into_iter()
        .into_grouping_map_by(|(id, _)| {
            nodes
                .get(id)
                .and_then(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()))
        })
        .collect();

    let nodes_by_daemon = nodes
        .values()
        .into_group_map_by(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()));

    let mut daemons = BTreeSet::new();
    for (machine, nodes_on_machine) in &nodes_by_daemon {
        let nodes_on_machine = nodes_on_machine.iter().map(|n| n.id.clone()).collect();
        tracing::debug!(
            "Running dataflow build `{build_id}` on machine `{machine:?}` (nodes: {nodes_on_machine:?})"
        );

        let build_command = BuildDataflowNodes {
            build_id,
            session_id,
            local_working_dir: local_working_dir.clone(),
            git_sources: git_sources_by_daemon.remove(machine).unwrap_or_default(),
            prev_git_sources: prev_git_sources_by_daemon
                .remove(machine)
                .unwrap_or_default(),
            dataflow_descriptor: dataflow.clone(),
            nodes_on_machine,
            uv,
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::Build(build_command),
            timestamp: clock.new_timestamp(),
        })?;

        let daemon_id =
            build_dataflow_on_machine(daemon_connections, machine.map(|s| s.as_str()), &message)
                .await
                .wrap_err_with(|| format!("failed to build dataflow on machine `{machine:?}`"))?;
        daemons.insert(daemon_id);
    }

    tracing::info!("successfully triggered dataflow build `{build_id}`",);

    Ok(RunningBuild {
        errors: Vec::new(),
        build_result: CachedResult::default(),
        buffered_log_messages: Vec::new(),
        log_subscribers: Vec::new(),
        pending_build_results: daemons,
    })
}

async fn build_dataflow_on_machine(
    daemon_connections: &DaemonConnections,
    machine: Option<&str>,
    message: &[u8],
) -> Result<DaemonId, eyre::ErrReport> {
    let daemon_id = match machine {
        Some(machine) => daemon_connections
            .get_matching_daemon_id(machine)
            .wrap_err_with(|| format!("no matching daemon for machine id {machine:?}"))?
            .clone(),
        None => daemon_connections
            .unnamed()
            .next()
            .wrap_err("no unnamed daemon connections")?
            .clone(),
    };

    let stream = daemon_connections
        .get_stream(&daemon_id)
        .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?;
    let mut stream = stream.lock().await;
    tcp_send(&mut stream, message)
        .await
        .wrap_err("failed to send build message to daemon")?;

    let reply_raw = tcp_receive(&mut stream)
        .await
        .wrap_err("failed to receive build reply from daemon")?;
    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize build reply from daemon")?
    {
        DaemonCoordinatorReply::TriggerBuildResult(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("daemon returned an error")?,
        _ => bail!("unexpected reply"),
    }
    Ok(daemon_id)
}

#[allow(clippy::too_many_arguments)]
async fn start_dataflow(
    build_id: Option<BuildId>,
    session_id: SessionId,
    dataflow: Descriptor,
    local_working_dir: Option<PathBuf>,
    name: Option<String>,
    daemon_connections: &DaemonConnections,
    clock: &HLC,
    running_dataflows: &DashMap<Uuid, RunningDataflow>,
    uv: bool,
    write_events_to: Option<PathBuf>,
) -> eyre::Result<Uuid> {
    let plan = run::plan_dataflow(
        build_id,
        session_id,
        &dataflow,
        local_working_dir,
        daemon_connections,
        clock,
        uv,
        write_events_to,
    )?;

    let uuid = plan.uuid;
    let daemons = plan.daemons.clone();

    let run::DataflowPlan {
        uuid: _,
        daemons: _,
        nodes,
        node_to_daemon,
        daemon_messages,
    } = plan;

    // Insert the RunningDataflow into the map BEFORE sending spawn commands to
    // the daemons.  This avoids a race where the daemon finishes spawning
    // and sends the SpawnResult back before start_dataflow returns — if the
    // entry isn't in the map yet, the coordinator event loop would discard
    // the result and `wait_for_spawn` would time out.
    running_dataflows.insert(
        uuid,
        RunningDataflow {
            uuid,
            name,
            descriptor: dataflow,
            pending_daemons: if daemons.len() > 1 {
                daemons.clone()
            } else {
                BTreeSet::new()
            },
            exited_before_subscribe: Default::default(),
            daemons: daemons.clone(),
            nodes,
            node_to_daemon,
            node_metrics: BTreeMap::new(),
            spawn_result: CachedResult::default(),
            stop_reply_senders: Vec::new(),
            buffered_log_messages: Vec::new(),
            log_subscribers: Vec::new(),
            pending_spawn_results: daemons,
        },
    );

    // Now send the spawn commands.  If a result arrives quickly, the entry is
    // already in the map so the event loop won't discard it.
    if let Err(err) = run::execute_dataflow_plan(uuid, &daemon_messages, daemon_connections).await {
        running_dataflows.remove(&uuid);
        return Err(err);
    }

    Ok(uuid)
}

async fn destroy_daemon(
    daemon_id: DaemonId,
    stream: &Arc<tokio::sync::Mutex<TcpStream>>,
    timestamp: uhlc::Timestamp,
) -> Result<()> {
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Destroy,
        timestamp,
    })?;

    let mut stream = stream.lock().await;
    tcp_send(&mut stream, &message)
        .await
        .wrap_err_with(|| format!("failed to send destroy message to daemon `{daemon_id}`"))?;

    // wait for reply
    let reply_raw = tcp_receive(&mut stream)
        .await
        .wrap_err("failed to receive destroy reply from daemon")?;
    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize destroy reply from daemon")?
    {
        DaemonCoordinatorReply::DestroyResult { result, .. } => result
            .map_err(|e| eyre!(e))
            .wrap_err("failed to destroy dataflow")?,
        other => bail!("unexpected reply after sending `destroy`: {other:?}"),
    }

    tracing::info!("successfully destroyed daemon `{daemon_id}`");
    Ok(())
}

async fn destroy_daemons(
    daemon_connections: &DaemonConnections,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    // Collect daemon IDs and stream handles, then drop the DashMap lock.
    let daemons: Vec<(DaemonId, Arc<tokio::sync::Mutex<TcpStream>>)> = daemon_connections
        .iter()
        .map(|r| (r.key().clone(), r.value().stream.clone()))
        .collect();

    let results = futures::future::join_all(daemons.iter().map(|(daemon_id, stream)| {
        tracing::info!("Destroying daemon connection for `{daemon_id}`");
        destroy_daemon(daemon_id.clone(), stream, timestamp)
    }))
    .await;
    daemon_connections.clear();

    for result in results {
        result?;
    }
    Ok(())
}

#[derive(Debug)]
pub enum Event {
    NewDaemonConnection(TcpStream),
    DaemonConnectError(eyre::Report),
    DaemonHeartbeat {
        daemon_id: DaemonId,
    },
    Dataflow {
        uuid: Uuid,
        event: DataflowEvent,
    },
    Control(ControlEvent),
    Daemon(DaemonRequest),
    DaemonHeartbeatInterval,
    CtrlC,
    Log(LogMessage),
    DaemonExit {
        daemon_id: dora_message::common::DaemonId,
    },
    DataflowBuildResult {
        build_id: BuildId,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
    DataflowSpawnResult {
        dataflow_id: uuid::Uuid,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    },
    NodeMetrics {
        dataflow_id: uuid::Uuid,
        metrics: BTreeMap<NodeId, dora_message::daemon_to_coordinator::NodeMetrics>,
    },
    Close,
}

impl Event {
    /// Whether this event should be logged.
    #[allow(clippy::match_like_matches_macro)]
    pub fn log(&self) -> bool {
        match self {
            Event::DaemonHeartbeatInterval => false,
            _ => true,
        }
    }

    fn kind(&self) -> &'static str {
        match self {
            Event::NewDaemonConnection(_) => "NewDaemonConnection",
            Event::DaemonConnectError(_) => "DaemonConnectError",
            Event::DaemonHeartbeat { .. } => "DaemonHeartbeat",
            Event::Dataflow { .. } => "Dataflow",
            Event::Control(_) => "Control",
            Event::Daemon(_) => "Daemon",
            Event::DaemonHeartbeatInterval => "DaemonHeartbeatInterval",
            Event::CtrlC => "CtrlC",
            Event::Log(_) => "Log",
            Event::DaemonExit { .. } => "DaemonExit",
            Event::DataflowBuildResult { .. } => "DataflowBuildResult",
            Event::DataflowSpawnResult { .. } => "DataflowSpawnResult",
            Event::NodeMetrics { .. } => "NodeMetrics",
            Event::Close => "Close",
        }
    }
}

#[derive(Debug)]
pub enum DataflowEvent {
    DataflowFinishedOnDaemon {
        daemon_id: DaemonId,
        result: DataflowDaemonResult,
    },
    ReadyOnDaemon {
        daemon_id: DaemonId,
        exited_before_subscribe: Vec<NodeId>,
    },
}

#[derive(Debug)]
pub enum DaemonRequest {
    Register {
        version_check_result: Result<(), String>,
        machine_id: Option<String>,
        connection: TcpStream,
    },
}

fn set_up_ctrlc_handler() -> Result<impl Stream<Item = Event>, eyre::ErrReport> {
    let (ctrlc_tx, ctrlc_rx) = mpsc::channel(1);

    let mut ctrlc_sent = false;
    ctrlc::set_handler(move || {
        if ctrlc_sent {
            tracing::warn!("received second ctrlc signal -> aborting immediately");
            std::process::abort();
        } else {
            tracing::info!("received ctrlc signal");
            if ctrlc_tx.blocking_send(Event::CtrlC).is_err() {
                tracing::error!("failed to report ctrl-c event to dora-coordinator");
            }

            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    Ok(ReceiverStream::new(ctrlc_rx))
}
