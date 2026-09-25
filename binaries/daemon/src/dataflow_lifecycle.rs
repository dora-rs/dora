//! Building and spawning a dataflow's nodes on this daemon.

use crate::log::DataflowLogger;
use crate::spawn::Spawner;
use crate::{
    CoreNodeKindExt, Daemon, DebugSchemaCache, Event, InputDeadline, OutputId, RunningDataflow,
    STDERR_LOG_LINES_MAX, node_inputs, output_routing, rebuild_debug_topic_stream,
    retain_debug_schema, spawn,
};
use crossbeam::queue::ArrayQueue;
use dora_core::{
    build::{self, BuildInfo, PrevGitSource},
    config::{InputMapping, NodeId},
    descriptor::{Descriptor, DescriptorExt, ResolvedNode},
    topics::{zenoh_daemon_control_topic, zenoh_output_publish_topic},
    uhlc::HLC,
};
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{GitSource, LogLevel, NodeError, NodeErrorCause, NodeExitStatus},
    node_to_daemon::Timestamped,
};
use eyre::{Context, ContextCompat, bail, eyre};
use futures::future;
use std::{
    collections::{BTreeMap, BTreeSet},
    env::current_dir,
    future::Future,
    path::PathBuf,
    pin::pin,
    sync::Arc,
    time::Duration,
};
use tokio::sync::{broadcast, mpsc};
use uuid::Uuid;

pub(crate) struct NodeBuildTask<F> {
    node_id: NodeId,
    dynamic_node: bool,
    task: F,
}

impl Daemon {
    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn build_dataflow(
        &mut self,
        build_id: BuildId,
        session_id: SessionId,
        base_working_dir: PathBuf,
        git_sources: BTreeMap<NodeId, GitSource>,
        prev_git_sources: BTreeMap<NodeId, GitSource>,
        dataflow_descriptor: Descriptor,
        local_nodes: BTreeSet<NodeId>,
        uv: bool,
    ) -> eyre::Result<impl Future<Output = eyre::Result<BuildInfo>> + use<>> {
        let builder = build::Builder {
            session_id,
            base_working_dir,
            uv,
        };
        self.git_manager.clear_planned_builds(session_id);

        let nodes = dataflow_descriptor.resolve_aliases_and_set_defaults()?;

        let mut tasks = Vec::new();

        // build nodes
        for node in nodes.into_values().filter(|n| local_nodes.contains(&n.id)) {
            let dynamic_node = node.kind.dynamic();

            let node_id = node.id.clone();
            let mut logger = self.logger.for_node_build(build_id, node_id.clone());
            logger.log(LogLevel::Debug, "building").await;
            let git_source = git_sources.get(&node_id).cloned();
            let prev_git_source = prev_git_sources.get(&node_id).cloned();
            let prev_git = prev_git_source.map(|prev_source| PrevGitSource {
                // compare clone identity (repo + commit) only: hub provenance
                // and subdir don't change which directory the clone occupies
                still_needed_for_this_build: git_sources.values().any(|s| {
                    s.repo == prev_source.repo && s.commit_hash == prev_source.commit_hash
                }),
                git_source: prev_source,
            });

            let logger_cloned = logger
                .try_clone_impl()
                .await
                .wrap_err("failed to clone logger")?;

            let mut builder = builder.clone();
            if let Some(node_working_dir) =
                node.deploy.as_ref().and_then(|d| d.working_dir.as_deref())
            {
                builder.base_working_dir = builder.base_working_dir.join(node_working_dir);
            }

            match builder
                .build_node(
                    node,
                    git_source,
                    prev_git,
                    logger_cloned,
                    &mut self.git_manager,
                )
                .await
                .wrap_err_with(|| format!("failed to build node `{node_id}`"))
            {
                Ok(result) => {
                    tasks.push(NodeBuildTask {
                        node_id,
                        task: result,
                        dynamic_node,
                    });
                }
                Err(err) => {
                    logger.log(LogLevel::Error, format!("{err:?}")).await;
                    return Err(err);
                }
            }
        }

        // hub-sourced nodes (recognized by the provenance marker on their
        // git source) are spawned with confined path resolution (spec §11)
        let confined_nodes: BTreeSet<NodeId> = git_sources
            .iter()
            .filter(|(_, source)| source.hub.is_some())
            .map(|(node_id, _)| node_id.clone())
            .collect();
        let task = async move {
            let mut info = BuildInfo {
                node_working_dirs: Default::default(),
                python_env_dirs: Default::default(),
                confined_nodes,
            };
            for task in tasks {
                let NodeBuildTask {
                    node_id,
                    dynamic_node: _,
                    task,
                } = task;
                let node = task
                    .await
                    .with_context(|| format!("failed to build node `{node_id}`"))?;
                info.node_working_dirs
                    .insert(node_id.clone(), node.node_working_dir);
                if let Some(python_env_dir) = node.python_env_dir {
                    info.python_env_dirs.insert(node_id, python_env_dir);
                }
            }
            Ok(info)
        };

        Ok(task)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn spawn_dataflow(
        &mut self,
        build_id: Option<BuildId>,
        dataflow_id: DataflowId,
        base_working_dir: PathBuf,
        nodes: BTreeMap<NodeId, ResolvedNode>,
        dataflow_descriptor: Descriptor,
        spawn_nodes: BTreeSet<NodeId>,
        uv: bool,
        write_events_to: Option<PathBuf>,
    ) -> eyre::Result<impl Future<Output = eyre::Result<()>> + use<>> {
        // Before anything below touches state keyed by the dataflow id (the
        // pool subscriber, the endpoint queryable), all of which belongs to the
        // live dataflow.
        if self.running.contains_key(&dataflow_id) {
            bail!("there is already a running dataflow with ID `{dataflow_id}`")
        }
        // Reclaim `/dev/shm` segments a previous crash of this dataflow's
        // nodes left behind. Scoped to the nodes this daemon spawns, since
        // a co-located daemon may be starting the other half of the same
        // dataflow right now.
        #[cfg(feature = "tensor-pool")]
        dora_tensor_pool::daemon::PoolState::sweep_orphans_for_dataflow(dataflow_id, |node| {
            spawn_nodes.iter().any(|id| id.as_ref() == node)
        });

        #[cfg(feature = "tensor-pool")]
        self.pool_subscribe_dataflow(dataflow_id);

        let mut logger = self
            .logger
            .for_dataflow(dataflow_id)
            .try_clone()
            .await
            .context("failed to clone logger")?;
        let mut dataflow = RunningDataflow::new(
            dataflow_id,
            self.daemon_id.clone(),
            dataflow_descriptor.clone(),
        );
        // Read from the descriptor, which is the one copy that survives
        // everything a dataflow outlives: auto-recovery re-spawn,
        // coordinator restart with state reconstruction, and `dora
        // restart`. A daemon serving a coordinator hosts many dataflows
        // and only some are batch-style, so this is necessarily
        // per-dataflow rather than daemon-wide (#2920).
        dataflow.timers_gate_drain = !dataflow.descriptor.exit_when_nodes_finish.unwrap_or(false);
        // Decide who dials whom before anything spawns: zenoh 1.9 peers do not
        // relay, so a producer/consumer pair that never forms a direct link can
        // never exchange data. Assign the links explicitly instead of leaving
        // them to gossip's best-effort autoconnect.
        //
        // Three steps, in this order because each needs the previous one's
        // result: reserve this daemon's own listeners, trade endpoints with the
        // daemons running the rest of the dataflow, then build the dial lists.
        // The trade has to finish before any node spawns — a node's zenoh
        // session reads its dial list once at startup and never again.
        let listeners =
            crate::spawn::reserve_node_listeners(&nodes, &spawn_nodes, self.zenoh_routable_addr);
        // Started here but awaited below, so its bounded wait overlaps the
        // build metadata and working-directory work in between rather than
        // stalling the event loop on its own. Same reason the memory-pool
        // subscriber above runs off the loop: a degraded inter-daemon link must
        // not wedge this spawn handler and every event queued behind it.
        let wanted = crate::spawn::wanted_remote_sources(&nodes, &spawn_nodes);
        let local_endpoints: BTreeMap<NodeId, String> = listeners
            .iter()
            .filter_map(|(id, l)| Some((id.clone(), l.routable()?.to_string())))
            .collect();
        // Every daemon of a dataflow that spans more than one takes part, even
        // with nothing to announce or ask: its reply is what tells the others
        // they can reach it at all (see `endpoint_exchange`). A daemon with no
        // upstream elsewhere declares and returns at once.
        let peers = crate::spawn::remote_placements(&nodes, &spawn_nodes);
        let mut endpoint_exchange = None;
        if crate::spawn::spans_daemons(&nodes, &spawn_nodes) {
            let session = self.zenoh_session.clone();
            let daemon_id = self.daemon_id.clone();
            let exchange_logger = logger.try_clone().await.ok();
            endpoint_exchange = Some(tokio::spawn(crate::spawn::endpoint_exchange::exchange(
                session,
                dataflow_id,
                daemon_id,
                local_endpoints,
                wanted,
                peers,
                exchange_logger,
            )));
        }
        self.working_dir
            .insert(dataflow_id, base_working_dir.clone());
        let dataflow = self.running.entry(dataflow_id).or_insert(dataflow);

        let mut stopped = Vec::new();

        // A present `build_id` means the session has a build that wasn't
        // invalidated (the CLI clears it when build inputs change). We use this
        // below to decide whether re-deriving an on-disk managed env is safe.
        let have_build_id = build_id.is_some();
        let build_info = build_id.and_then(|build_id| self.builds.get(&build_id));
        let node_with_git_source = nodes.values().find(|n| n.has_git_source());
        if let Some(git_node) = node_with_git_source
            && build_info.is_none()
        {
            eyre::bail!(
                "node {} has git source, but no `dora build` was run yet\n\n\
                    nodes with a `git` field must be built using `dora build` before starting the \
                    dataflow",
                git_node.id
            )
        }
        // Reuse build-time metadata so runtime spawn can follow the same
        // working-directory and managed-env decisions.
        let (node_working_dirs, python_env_dirs, confined_nodes) = build_info
            .map(|info| {
                (
                    info.node_working_dirs.clone(),
                    info.python_env_dirs.clone(),
                    info.confined_nodes.clone(),
                )
            })
            .unwrap_or_default();

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
                    // Record which inputs can ever close. Timer and Logs
                    // inputs are fed by the daemon and have no upstream
                    // node, so they never close; only a `User` mapping is
                    // a real data dependency (#2920).
                    if matches!(input.mapping, InputMapping::User(_)) {
                        dataflow
                            .data_inputs
                            .entry(node.id.clone())
                            .or_default()
                            .insert(input_id.clone());
                    }
                    match input.mapping {
                        InputMapping::User(mapping) => {
                            if let Some(timeout) = input.input_timeout {
                                dataflow.input_deadlines.insert(
                                    (node.id.clone(), input_id.clone()),
                                    InputDeadline {
                                        timeout: Duration::from_secs_f64(timeout),
                                        // Unarmed until the first message
                                        // arrives — see issue #149.
                                        last_received: None,
                                    },
                                );
                            }
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
                        InputMapping::Logs(filter) => {
                            dataflow
                                .log_subscribers
                                .push(crate::running_dataflow::LogSubscriber {
                                    node_id: node.id.clone(),
                                    input_id,
                                    filter,
                                });
                        }
                    }
                } else if let InputMapping::User(mapping) = input.mapping {
                    dataflow
                        .open_external_mappings
                        .insert(OutputId(mapping.source, mapping.output));
                }
            }
        }

        // When debug inspection is enabled, the daemon subscribes to its own
        // local nodes' Zenoh output topics so `dora topic info/echo/hz` keep
        // working. Since #1787, node→node data flows directly over Zenoh and
        // never reaches the daemon's `send_out`; without this the inspection
        // commands observe zero messages. Each sample is forwarded as an
        // `Event::DebugTopicData` and only relayed to coordinator debug
        // watchers — never re-delivered to local receivers (the publishing
        // node already delivered the payload via Zenoh).
        if dataflow.enable_debug_inspection {
            use zenoh_ext::{AdvancedSubscriberBuilderExt, HistoryConfig};
            for node in nodes.values().filter(|n| spawn_nodes.contains(&n.id)) {
                for output_id in node.kind.run_config().outputs {
                    let topic = zenoh_output_publish_topic(dataflow_id, &node.id, &output_id);
                    tracing::debug!("declaring debug subscriber on {topic}");
                    let subscriber = self
                        .zenoh_session
                        .declare_subscriber(topic.clone())
                        .await
                        .map_err(|e| eyre!(e))
                        .wrap_err_with(|| {
                            format!("failed to declare debug subscriber on {topic}")
                        })?;

                    // Small node→node outputs use the schema-once path: the
                    // schema is published once on the `@schema` subtopic and each
                    // data sample carries only the schema-less record batch (see
                    // `DoraNode::publish_schema_once`). `dora topic` expects a
                    // full self-describing stream, so cache the schema here and
                    // rebuild it before forwarding. An `AdvancedSubscriber`
                    // (history + late-publisher detection) mirrors the node
                    // receive path so the (single, stable) schema is fetched even
                    // when the producer starts after this subscriber.
                    let schema_cache: DebugSchemaCache =
                        Arc::new(std::sync::Mutex::new(Vec::new()));
                    let schema_topic = dora_core::topics::zenoh_output_schema_topic(
                        dataflow_id,
                        &node.id,
                        &output_id,
                    );
                    let schema_subscriber = {
                        let cache = schema_cache.clone();
                        match self
                            .zenoh_session
                            .declare_subscriber(schema_topic.clone())
                            .history(HistoryConfig::default().detect_late_publishers())
                            .callback(move |sample| {
                                let bytes = sample.payload().to_bytes();
                                let hash = dora_message::metadata::fnv1a(&bytes);
                                retain_debug_schema(
                                    &mut cache.lock().unwrap_or_else(|e| e.into_inner()),
                                    hash,
                                    &bytes,
                                );
                            })
                            .await
                        {
                            Ok(s) => Some(s),
                            Err(e) => {
                                tracing::warn!(
                                    "failed to declare @schema debug subscriber on \
                                     {schema_topic} ({e}); schema-once outputs may render as \
                                     undecodable in `dora topic`"
                                );
                                None
                            }
                        }
                    };

                    let events_tx = self.events_tx.clone();
                    let clock = self.clock.clone();
                    let node_id = node.id.clone();
                    let mut finished_rx = dataflow.finished_tx.subscribe();
                    tokio::spawn(async move {
                        // Keep the `@schema` subscriber alive for the task's life.
                        let _schema_subscriber = schema_subscriber;
                        let mut finished = pin!(finished_rx.recv());
                        loop {
                            match future::select(finished, subscriber.recv_async()).await {
                                future::Either::Left((_, _)) => break,
                                future::Either::Right((sample, f)) => {
                                    finished = f;
                                    let Ok(sample) = sample else { break };
                                    // Node publishes raw payload + encoded metadata
                                    // attachment (see `DoraNode::zenoh_publish`).
                                    use dora_message::metadata::Metadata;
                                    let Some(mut metadata) = sample.attachment().and_then(|a| {
                                        dora_message::decode::<Metadata>(&a.to_bytes()).ok()
                                    }) else {
                                        continue;
                                    };
                                    // Startup-handshake markers ride the real data
                                    // topic (empty payload); consumers filter them
                                    // before decode and so must this inspection
                                    // path, or `dora topic echo`/`hz` would show
                                    // spurious empty frames and inflated rates
                                    // during every startup handshake.
                                    if metadata.is_startup_marker() {
                                        continue;
                                    }
                                    let payload = sample.payload().to_bytes();
                                    let data = {
                                        let mut cached =
                                            schema_cache.lock().unwrap_or_else(|e| e.into_inner());
                                        rebuild_debug_topic_stream(
                                            &mut cached,
                                            &metadata.parameters,
                                            &payload[..],
                                        )
                                    };
                                    let Some(data) = data else {
                                        // schema-once batch whose schema hasn't been
                                        // cached yet — drop this inspection frame
                                        // (transient, recovers once `@schema` or the
                                        // producer's next full-stream refresh arrives).
                                        continue;
                                    };
                                    // `data` is now always a full self-describing
                                    // stream, so drop the internal wire keys — a
                                    // `_schema_hash` here would contradict the
                                    // rebuilt payload for any consumer applying the
                                    // wire rule "hash present => schema-less batch".
                                    dora_message::metadata::strip_internal_parameters(
                                        &mut metadata.parameters,
                                    );
                                    // Record the true on-wire size so `dora topic
                                    // info` measures the schema-less batch that
                                    // actually travelled, not the rebuilt stream
                                    // (which prepends the schema to every frame even
                                    // though it ships once). For a full
                                    // self-describing frame this equals `data.len()`;
                                    // for a schema-once batch it excludes the
                                    // prepended schema block (dora-rs/dora#2584).
                                    metadata.parameters.insert(
                                        dora_message::metadata::WIRE_SIZE.to_string(),
                                        dora_message::metadata::Parameter::Integer(
                                            payload.len() as i64
                                        ),
                                    );
                                    let event = Event::DebugTopicData {
                                        dataflow_id,
                                        output_id: OutputId(node_id.clone(), output_id.clone()),
                                        metadata,
                                        data: Some(data),
                                    };
                                    if events_tx
                                        .send(Timestamped {
                                            inner: event,
                                            timestamp: clock.new_timestamp(),
                                        })
                                        .await
                                        .is_err()
                                    {
                                        break;
                                    }
                                }
                            }
                        }
                    });
                }
            }
        }

        // Collect the endpoint exchange started before the build metadata work
        // above, and only now build the dial lists — this is the last moment
        // before nodes spawn, and a node reads its dial list once at startup.
        // A task that panicked or was cancelled resolves to "no remote
        // endpoints", which is the same degradation as an expired deadline.
        let (remote_endpoints, exchange_handle) = match endpoint_exchange {
            Some(task) => task.await.unwrap_or_else(|err| {
                tracing::warn!("zenoh node-endpoint exchange failed: {err}");
                Default::default()
            }),
            None => Default::default(),
        };
        dataflow.zenoh_peering = Arc::new(crate::spawn::build_peering_plan(
            &nodes,
            &listeners,
            self.zenoh_listen_endpoint.as_deref(),
            &remote_endpoints,
        ));
        dataflow.endpoint_exchange = exchange_handle;

        let spawner = Spawner {
            dataflow_id,
            daemon_tx: self.events_tx.clone(),
            dataflow_descriptor,
            clock: self.clock.clone(),
            uv,
            ft_stats: self.ft_stats.clone(),
            shutdown: dataflow.listener_shutdown_rx.clone(),
            zenoh_connect_endpoint: self.zenoh_listen_endpoint.clone(),
            zenoh_peering: dataflow.zenoh_peering.clone(),
            disable_multicast: self.disable_multicast,
            machine_id: self.machine_id.clone(),
            bind_nodes_to_parent: self.bind_nodes_to_parent,
            shell_guard_host: self.shell_guard_host.clone(),
        };

        // Startup-handshake routing, from actual placement (`spawn_nodes`):
        // which outputs are pinned to the daemon path and which static
        // consumers must ack before an output may switch to the direct zenoh
        // path. A remote consumer can only ack a producer it can dial, so the
        // set of producers that got a routable endpoint decides which
        // cross-machine edges are even candidates.
        let routable_producers: BTreeSet<NodeId> = dataflow
            .zenoh_peering
            .iter()
            .filter(|(_, peering)| peering.routable)
            .map(|(id, _)| id.clone())
            .collect();
        let mut output_routing =
            output_routing::compute_output_routing(&nodes, &spawn_nodes, &routable_producers);
        let mut backpressured_outputs = output_routing::backpressured_outputs(&nodes, |_| true);
        dataflow.remote_backpressured_outputs =
            output_routing::backpressured_outputs(&nodes, |consumer| {
                !spawn_nodes.contains(consumer)
            })
            .into_iter()
            .flat_map(|(producer, outputs)| {
                outputs
                    .into_iter()
                    .map(move |output| OutputId(producer.clone(), output))
            })
            .collect();

        let mut tasks = Vec::new();

        // spawn nodes and set up subscriptions
        for node in nodes.into_values() {
            let mut logger = logger.reborrow().for_node(node.id.clone());
            let local = spawn_nodes.contains(&node.id);
            if local {
                let dynamic_node = node.kind.dynamic();
                if dynamic_node {
                    dataflow.dynamic_nodes.insert(node.id.clone());
                } else {
                    dataflow.pending_nodes.insert(node.id.clone());
                }

                let node_id = node.id.clone();
                let node_stderr_most_recent = dataflow
                    .node_stderr_most_recent
                    .entry(node.id.clone())
                    .or_insert_with(|| Arc::new(ArrayQueue::new(STDERR_LOG_LINES_MAX)))
                    .clone();

                let configured_node_working_dir = node_working_dirs.get(&node_id).cloned();
                if configured_node_working_dir.is_none() && node.has_git_source() {
                    eyre::bail!(
                        "node {} has git source, but no git clone directory was found for it\n\n\
                        try running `dora build` again",
                        node.id
                    )
                }
                let node_working_dir = configured_node_working_dir
                    .or_else(|| {
                        node.deploy
                            .as_ref()
                            .and_then(|d| d.working_dir.as_ref().map(|d| base_working_dir.join(d)))
                    })
                    .unwrap_or(base_working_dir.clone())
                    .clone();
                let node_write_events_to = write_events_to
                    .as_ref()
                    .map(|p| p.join(format!("inputs-{}.json", node.id)));
                let mut configured_python_env_dir = python_env_dirs.get(&node_id).cloned();
                // Under `--uv`, a node may have no recorded managed env even when
                // `dora build` prepared one: a networked build runs in a separate
                // process from the daemon that later serves `dora start`, so the
                // daemon's in-memory build record can be empty (dora-rs/dora#2004).
                // The env dir is deterministic, so re-derive it (the restart path
                // does the same) and reuse the on-disk env -- but ONLY when a
                // build id is present, i.e. a build occurred and this daemon just
                // lacks the in-memory record. When the build id was cleared (the
                // CLI invalidates it on build-input changes, a prior non-`--uv`
                // build, or `start --uv` with no build), do NOT reuse a possibly
                // stale env; require a rebuild. Either way, never silently fall
                // back to the ambient Python.
                if uv
                    && configured_python_env_dir.is_none()
                    && let Some(expected) =
                        dora_core::build::managed_python_env_dir(&node, &node_working_dir)
                {
                    if have_build_id
                        && dora_core::build::managed_python_interpreter(&expected).is_file()
                    {
                        configured_python_env_dir = Some(expected);
                    } else {
                        eyre::bail!(
                            "node `{node_id}` is a Python node that needs a managed env under `--uv`, \
                             but no current build provides one (the build cache is absent or was \
                             invalidated by changed build inputs). \
                             Run `dora build --uv <dataflow>` before `dora start --uv`, \
                             or omit `--uv` to run against the ambient Python."
                        );
                    }
                }
                match spawner
                    .clone()
                    .spawn_node(
                        node,
                        node_working_dir,
                        configured_python_env_dir,
                        confined_nodes.contains(&node_id),
                        node_stderr_most_recent,
                        node_write_events_to,
                        output_routing.remove(&node_id).unwrap_or_default(),
                        backpressured_outputs.remove(&node_id).unwrap_or_default(),
                        &mut logger,
                    )
                    .await
                    .wrap_err_with(|| format!("failed to spawn node `{node_id}`"))
                {
                    Ok(result) => {
                        tasks.push(NodeBuildTask {
                            node_id,
                            task: result,
                            dynamic_node,
                        });
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
                                    cause: NodeErrorCause::FailedToSpawn(format!("{err:?}")),
                                    exit_status: NodeExitStatus::Unknown,
                                }),
                            );
                        stopped.push((node_id.clone(), dynamic_node));
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
                    let subscribe_topic =
                        zenoh_daemon_control_topic(dataflow.id, &output_id.0, &output_id.1);
                    tracing::debug!("declaring control subscriber on {subscribe_topic}");
                    let subscriber = self
                        .zenoh_session
                        .declare_subscriber(subscribe_topic)
                        .await
                        .map_err(|e| eyre!(e))
                        .wrap_err_with(|| format!("failed to subscribe to {output_id:?}"))?;
                    let net_bytes_rx = dataflow.net_bytes_received.clone();
                    let net_msgs_rx = dataflow.net_messages_received.clone();
                    tokio::spawn(async move {
                        let mut finished = pin!(finished_rx.recv());
                        loop {
                            let finished_or_next =
                                futures::future::select(finished, subscriber.recv_async());
                            match finished_or_next.await {
                                future::Either::Left((finished, _)) => match finished {
                                    Err(broadcast::error::RecvError::Closed) => {
                                        tracing::debug!(
                                            "dataflow finished, breaking from zenoh subscribe task"
                                        );
                                        break;
                                    }
                                    other => {
                                        tracing::warn!(
                                            "unexpected return value of dataflow finished_rx channel: {other:?}"
                                        );
                                        break;
                                    }
                                },
                                future::Either::Right((sample, f)) => {
                                    finished = f;
                                    match sample {
                                        Ok(s) => {
                                            // Count telemetry for every received control sample.
                                            net_bytes_rx.fetch_add(
                                                s.payload().len() as u64,
                                                std::sync::atomic::Ordering::Relaxed,
                                            );
                                            net_msgs_rx
                                                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                            let bytes = s.payload().to_bytes();
                                            let event =
                                                Timestamped::deserialize_inter_daemon_event(&bytes)
                                                    .map_err(|e| eyre!(e));
                                            if tx.send_async(event).await.is_err() {
                                                // daemon finished
                                                break;
                                            }
                                        }
                                        Err(e) => {
                                            if tx.send_async(Err(eyre!(e))).await.is_err() {
                                                // daemon finished
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    });
                }
            }
        }
        for (node_id, dynamic) in stopped {
            // Pre-spawn failures (resolve/validate errors). Surface
            // as Failed so the dataflow doesn't silently hide them.
            self.handle_node_stop(dataflow_id, &node_id, dynamic, false)
                .await?;
        }

        let spawn_result = Self::spawn_prepared_nodes(
            dataflow_id,
            logger,
            tasks,
            self.events_tx.clone(),
            self.clock.clone(),
        );

        Ok(spawn_result)
    }

    pub(crate) async fn spawn_prepared_nodes(
        dataflow_id: Uuid,
        mut logger: DataflowLogger<'_>,
        tasks: Vec<NodeBuildTask<impl Future<Output = eyre::Result<spawn::PreparedNode>>>>,
        events_tx: mpsc::Sender<Timestamped<Event>>,
        clock: Arc<HLC>,
    ) -> eyre::Result<()> {
        let node_result = |node_id, dynamic_node, result| Timestamped {
            inner: Event::SpawnNodeResult {
                dataflow_id,
                node_id,
                dynamic_node,
                result,
            },
            timestamp: clock.new_timestamp(),
        };
        let mut failed_to_prepare = None;
        let mut prepared_nodes = Vec::new();
        for task in tasks {
            let NodeBuildTask {
                node_id,
                dynamic_node,
                task,
            } = task;
            match task.await {
                Ok(node) => prepared_nodes.push(node),
                Err(err) => {
                    if failed_to_prepare.is_none() {
                        failed_to_prepare = Some(node_id.clone());
                    }
                    let node_err: NodeError = NodeError {
                        timestamp: clock.new_timestamp(),
                        cause: NodeErrorCause::FailedToSpawn(format!(
                            "preparing for spawn failed: {err:?}"
                        )),
                        exit_status: NodeExitStatus::Unknown,
                    };
                    let send_result = events_tx
                        .send(node_result(node_id, dynamic_node, Err(node_err)))
                        .await;
                    if send_result.is_err() {
                        tracing::error!("failed to send SpawnNodeResult to main daemon task")
                    }
                }
            }
        }

        // once all nodes are prepared, do the actual spawning
        if let Some(failed_node) = failed_to_prepare {
            // don't spawn any nodes when an error occurred before
            for node in prepared_nodes {
                let err = NodeError {
                    timestamp: clock.new_timestamp(),
                    cause: NodeErrorCause::Cascading {
                        caused_by_node: failed_node.clone(),
                    },
                    exit_status: NodeExitStatus::Unknown,
                };
                let send_result = events_tx
                    .send(node_result(
                        node.node_id().clone(),
                        node.dynamic(),
                        Err(err),
                    ))
                    .await;
                if send_result.is_err() {
                    tracing::error!("failed to send SpawnNodeResult to main daemon task")
                }
            }
            Err(eyre!("failed to prepare node {failed_node}"))
        } else {
            let mut spawn_result = Ok(());

            logger
                .log(
                    LogLevel::Info,
                    None,
                    Some("dora daemon".into()),
                    "finished building nodes, spawning...",
                )
                .await;

            // spawn the nodes
            for node in prepared_nodes {
                let node_id = node.node_id().clone();
                let dynamic_node = node.dynamic();
                let logger = logger
                    .reborrow()
                    .for_node(node_id.clone())
                    .try_clone()
                    .await
                    .context("failed to clone NodeLogger")?;
                let result = node.spawn(logger).await;
                let node_spawn_result = match result {
                    Ok(node) => Ok(node),
                    Err(err) => {
                        let node_err = NodeError {
                            timestamp: clock.new_timestamp(),
                            cause: NodeErrorCause::FailedToSpawn(format!("spawn failed: {err:?}")),
                            exit_status: NodeExitStatus::Unknown,
                        };
                        if spawn_result.is_ok() {
                            spawn_result = Err(err.wrap_err(format!("failed to spawn {node_id}")));
                        }
                        Err(node_err)
                    }
                };
                let send_result = events_tx
                    .send(node_result(node_id, dynamic_node, node_spawn_result))
                    .await;
                if send_result.is_err() {
                    tracing::error!("failed to send SpawnNodeResult to main daemon task")
                }
            }
            spawn_result
        }
    }

    pub(crate) fn base_working_dir(
        &self,
        local_working_dir: Option<PathBuf>,
        session_id: SessionId,
    ) -> eyre::Result<PathBuf> {
        match local_working_dir {
            Some(working_dir) => {
                // check that working directory exists
                if working_dir.exists() {
                    Ok(working_dir)
                } else {
                    bail!(
                        "working directory does not exist: {}",
                        working_dir.display(),
                    )
                }
            }
            None => {
                // use subfolder of daemon working dir
                let daemon_working_dir =
                    current_dir().context("failed to get daemon working dir")?;
                Ok(daemon_working_dir
                    .join("_work")
                    .join(session_id.uuid().to_string()))
            }
        }
    }
}
