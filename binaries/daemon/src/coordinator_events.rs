//! Handling of `DaemonCoordinatorEvent`s: the commands the coordinator sends a
//! daemon (build/spawn/stop/reload, parameter updates, topology edits, state
//! catch-up after a reconnect).

use crate::spawn::Spawner;
use crate::{
    CoreNodeKindExt, Daemon, Event, FinishDataflowWhen, InputDeadline, OutputId, RunStatus,
    RunningDataflow, STDERR_LOG_LINES_MAX, clear_node_result, close_input, log, node_inputs,
    output_routing, read_last_n_lines, reject_unpinnable_backpressure_inputs, send_with_timestamp,
};
use crossbeam::queue::ArrayQueue;
use dora_core::{
    config::{DataId, InputMapping, NodeId},
    uhlc::HLC,
};
use dora_message::{
    common::LogLevel,
    coordinator_to_daemon::{
        BuildDataflowNodes, DaemonCoordinatorEvent, SpawnDataflowNodes, StateCatchUpEntry,
        StateCatchUpOperation,
    },
    daemon_to_coordinator::{CoordinatorRequest, DaemonCoordinatorReply, DaemonEvent},
    daemon_to_node::NodeEvent,
    node_to_daemon::Timestamped,
};
use eyre::{Context, ContextCompat, Result, eyre};
use futures::TryFutureExt;
use std::{
    collections::BTreeMap,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::{fs::File, io::AsyncReadExt, sync::oneshot::Sender};
use tracing::error;

pub(crate) fn deliver_param_update_strict(
    dataflow: &RunningDataflow,
    node_id: &NodeId,
    key: String,
    value: serde_json::Value,
    clock: &HLC,
) -> eyre::Result<()> {
    let channel = dataflow
        .subscribe_channels
        .get(node_id)
        .ok_or_else(|| eyre!("node `{node_id}` not connected"))?;
    let value_json = serde_json::to_vec(&value)
        .map_err(|e| eyre!("failed to serialize param value for node `{node_id}`: {e}"))?;
    match send_with_timestamp(channel, NodeEvent::ParamUpdate { key, value_json }, clock) {
        Ok(true) => {
            dataflow.inc_pending(node_id);
            Ok(())
        }
        Ok(false) => Err(eyre!("node `{node_id}` channel full")),
        Err(_) => Err(eyre!("node `{node_id}` channel closed")),
    }
}

pub(crate) fn deliver_param_delete_strict(
    dataflow: &RunningDataflow,
    node_id: &NodeId,
    key: String,
    clock: &HLC,
) -> eyre::Result<()> {
    let channel = dataflow
        .subscribe_channels
        .get(node_id)
        .ok_or_else(|| eyre!("node `{node_id}` not connected"))?;
    match send_with_timestamp(channel, NodeEvent::ParamDeleted { key }, clock) {
        Ok(true) => {
            dataflow.inc_pending(node_id);
            Ok(())
        }
        Ok(false) => Err(eyre!("node `{node_id}` channel full")),
        Err(_) => Err(eyre!("node `{node_id}` channel closed")),
    }
}

pub(crate) fn apply_state_catch_up_entries(
    dataflow: &RunningDataflow,
    entries: &[StateCatchUpEntry],
    clock: &HLC,
) -> u64 {
    let mut applied_through = 0;

    for entry in entries {
        let delivered = match &entry.operation {
            StateCatchUpOperation::SetParam {
                node_id,
                key,
                value,
            } => {
                let Some(channel) = dataflow.subscribe_channels.get(node_id) else {
                    tracing::warn!(
                        "catch-up: node `{node_id}` not connected; stopping replay at seq {}",
                        entry.sequence
                    );
                    break;
                };
                let value_json = match serde_json::to_vec(value) {
                    Ok(bytes) => bytes,
                    Err(err) => {
                        tracing::warn!(
                            "catch-up: failed to serialize param value for node `{node_id}` at seq {}: {err}",
                            entry.sequence
                        );
                        break;
                    }
                };
                match send_with_timestamp(
                    channel,
                    NodeEvent::ParamUpdate {
                        key: key.clone(),
                        value_json,
                    },
                    clock,
                ) {
                    Ok(true) => {
                        dataflow.inc_pending(node_id);
                        true
                    }
                    Ok(false) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel full; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                    Err(_) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel closed; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                }
            }
            StateCatchUpOperation::DeleteParam { node_id, key } => {
                let Some(channel) = dataflow.subscribe_channels.get(node_id) else {
                    tracing::warn!(
                        "catch-up: node `{node_id}` not connected; stopping replay at seq {}",
                        entry.sequence
                    );
                    break;
                };
                match send_with_timestamp(
                    channel,
                    NodeEvent::ParamDeleted { key: key.clone() },
                    clock,
                ) {
                    Ok(true) => {
                        dataflow.inc_pending(node_id);
                        true
                    }
                    Ok(false) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel full; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                    Err(_) => {
                        tracing::warn!(
                            "catch-up: node `{node_id}` channel closed; stopping replay at seq {}",
                            entry.sequence
                        );
                        false
                    }
                }
            }
        };

        if !delivered {
            break;
        }
        applied_through = entry.sequence;
    }

    applied_through
}

impl Daemon {
    pub(crate) async fn handle_coordinator_event(
        &mut self,
        event: DaemonCoordinatorEvent,
        reply_tx: Sender<Option<DaemonCoordinatorReply>>,
    ) -> eyre::Result<RunStatus> {
        let status = match event {
            DaemonCoordinatorEvent::Build(BuildDataflowNodes {
                build_id,
                session_id,
                local_working_dir,
                git_sources,
                prev_git_sources,
                dataflow_descriptor,
                nodes_on_machine,
                uv,
            }) => {
                let base_working_dir = self.base_working_dir(local_working_dir, session_id)?;

                let result = self
                    .build_dataflow(
                        build_id,
                        session_id,
                        base_working_dir,
                        git_sources,
                        prev_git_sources,
                        dataflow_descriptor,
                        nodes_on_machine,
                        uv,
                    )
                    .await;
                let (trigger_result, result_task) = match result {
                    Ok(result_task) => (Ok(()), Some(result_task)),
                    Err(err) => (Err(format!("{err:?}")), None),
                };
                let reply = DaemonCoordinatorReply::TriggerBuildResult(trigger_result);
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send `TriggerBuildResult` reply from daemon to coordinator")
                });

                let result_tx = self.events_tx.clone();
                let clock = self.clock.clone();
                if let Some(result_task) = result_task {
                    tokio::spawn(async move {
                        let message = Timestamped {
                            inner: Event::BuildDataflowResult {
                                build_id,
                                session_id,
                                result: result_task.await,
                            },
                            timestamp: clock.new_timestamp(),
                        };
                        let _ = result_tx
                            .send(message)
                            .map_err(|_| {
                                error!(
                                    "could not send `BuildResult` reply from daemon to coordinator"
                                )
                            })
                            .await;
                    });
                }

                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Spawn(SpawnDataflowNodes {
                build_id,
                session_id,
                dataflow_id,
                local_working_dir,
                nodes,
                dataflow_descriptor,
                spawn_nodes,
                uv,
                write_events_to,
                artifact_base_url: _,
            }) => {
                let base_working_dir = self.base_working_dir(local_working_dir, session_id)?;

                let result = self
                    .spawn_dataflow(
                        build_id,
                        dataflow_id,
                        base_working_dir,
                        nodes,
                        dataflow_descriptor,
                        spawn_nodes,
                        uv,
                        write_events_to,
                    )
                    .await;
                let (trigger_result, result_task) = match result {
                    Ok(result_task) => (Ok(()), Some(result_task)),
                    Err(err) => {
                        // The spawn failed after the memory-pool subscriber
                        // task was started (it is spawned before the node
                        // build): the dataflow never reaches `self.running`,
                        // so `finish_dataflow` will not run — terminate the
                        // subscriber here or it leaks for the daemon's
                        // lifetime.
                        #[cfg(feature = "tensor-pool")]
                        self.pool.abort_subscriber(&dataflow_id);
                        (Err(format!("{err:?}")), None)
                    }
                };
                let reply = DaemonCoordinatorReply::TriggerSpawnResult(trigger_result);
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send `TriggerSpawnResult` reply from daemon to coordinator")
                });

                let result_tx = self.events_tx.clone();
                let clock = self.clock.clone();
                if let Some(result_task) = result_task {
                    tokio::spawn(async move {
                        let message = Timestamped {
                            inner: Event::SpawnDataflowResult {
                                dataflow_id,
                                result: result_task.await,
                            },
                            timestamp: clock.new_timestamp(),
                        };
                        let _ = result_tx
                            .send(message)
                            .map_err(|_| {
                                error!(
                                    "could not send `SpawnResult` reply from daemon to coordinator"
                                )
                            })
                            .await;
                    });
                }

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
                        // The verdict must fold in this daemon's local
                        // `exited_before_subscribe`, not just the coordinator's
                        // external list: a cohort member that died before
                        // subscribing while this daemon was disconnected is
                        // invisible to the replayed external list, but
                        // `answer_subscribe_requests` still fails the parked
                        // survivors on it. Deciding `start()` from the external
                        // list alone would start the dataflow while failing a
                        // healthy cohort node's init (dora-rs/dora#3243).
                        let status = dataflow
                            .pending_nodes
                            .handle_external_all_nodes_ready(
                                exited_before_subscribe,
                                &mut dataflow.cascading_error_causes,
                            )
                            .await?;
                        let ready = matches!(status, crate::pending::DataflowStatus::AllNodesReady);
                        // Not gated on `dataflow_started` — `start()` is
                        // idempotent — but it must not resurrect a dataflow that
                        // is already tearing down (dora-rs/dora#3053). The
                        // release can legitimately arrive after a local stop: the
                        // daemon can stop on its own (`trigger_manual_stop`), and
                        // the coordinator replays `AllNodesReady` to a daemon
                        // that reconnects.
                        if ready && !dataflow.stop_sent {
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
                tail,
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

                                let mut contents = match tail {
                                    None | Some(0) => {
                                        let mut contents = vec![];
                                        file.read_to_end(&mut contents).await.map(|_| contents)
                                    }
                                    Some(tail) => read_last_n_lines(&mut file, tail).await,
                                }
                                .wrap_err("Could not read last n lines of log file")?;
                                if !contents.ends_with(b"\n") {
                                    // Append newline for better readability
                                    contents.push(b'\n');
                                }
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
            DaemonCoordinatorEvent::RestartNode {
                dataflow_id,
                node_id,
                grace_duration,
            } => {
                let result = match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        dataflow.restart_single_node(&node_id, &self.clock, grace_duration)
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::RestartNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send restart node reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StopNode {
                dataflow_id,
                node_id,
                grace_duration,
            } => {
                let result = match self.running.get_mut(&dataflow_id) {
                    Some(dataflow) => {
                        dataflow.stop_single_node(&node_id, &self.clock, grace_duration)
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::StopNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send stop node reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::SetParam {
                dataflow_id,
                node_id,
                key,
                value,
            } => {
                let result = match self.running.get(&dataflow_id) {
                    Some(dataflow) => {
                        deliver_param_update_strict(dataflow, &node_id, key, value, &self.clock)
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::SetParamResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send set param reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::DeleteParam {
                dataflow_id,
                node_id,
                key,
            } => {
                let result = match self.running.get(&dataflow_id) {
                    Some(dataflow) => {
                        deliver_param_delete_strict(dataflow, &node_id, key, &self.clock)
                    }
                    None => Err(eyre::eyre!("no running dataflow with ID `{dataflow_id}`")),
                };
                let reply = DaemonCoordinatorReply::DeleteParamResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send delete param reply from daemon to coordinator")
                });
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StopDataflow {
                dataflow_id,
                grace_duration,
                force,
            } => {
                let finish_when = {
                    let mut logger = self.logger.for_dataflow(dataflow_id);
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"));
                    let (reply, future) = match dataflow {
                        Ok(dataflow) => {
                            let future = dataflow.stop_all(
                                &mut self.coordinator_sender,
                                &self.clock,
                                grace_duration,
                                force,
                                &mut logger,
                            );
                            (Ok(()), Some(future))
                        }
                        Err(err) => (Err(err.to_string()), None),
                    };

                    let _ = reply_tx
                        .send(Some(DaemonCoordinatorReply::StopResult(reply)))
                        .map_err(|_| {
                            error!("could not send stop reply from daemon to coordinator")
                        });

                    if let Some(future) = future {
                        Some(future.await?)
                    } else {
                        None
                    }
                };

                // If stop_all returns Now, finish the dataflow immediately
                if matches!(finish_when, Some(FinishDataflowWhen::Now)) {
                    self.finish_dataflow(dataflow_id).await?;
                }

                RunStatus::Continue
            }
            DaemonCoordinatorEvent::Destroy => {
                tracing::info!("received destroy command -> exiting");
                // Anything still running when this process ends is orphaned
                // to `ppid 1`: the per-node supervision tasks that would
                // deliver a kill are never polled again. So hold the reply
                // until the nodes are gone (#2980) — the coordinator's
                // destroy completes when it lands, which is what makes
                // `dora down` mean what it documents.
                //
                // Deferred rather than awaited here: a node shutting down
                // cooperatively ends by asking this very event loop to close
                // its outputs, so blocking the loop would deadlock every
                // well-behaved node against the wait and turn its clean exit
                // into a signal kill.
                match self.begin_pending_destroy(reply_tx) {
                    Some(reply_tx) => {
                        Self::finish_destroy(reply_tx).await;
                        RunStatus::Exit
                    }
                    None => RunStatus::Continue,
                }
            }
            DaemonCoordinatorEvent::Heartbeat => {
                self.last_coordinator_heartbeat = Instant::now();
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::PeerDaemonDisconnected { daemon_id } => {
                tracing::warn!(%daemon_id, "peer daemon disconnected");
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
            // --- Dynamic Topology ---
            DaemonCoordinatorEvent::AddNode {
                dataflow_id,
                node,
                uv,
            } => {
                let node_id = node.id.clone();
                tracing::info!(%dataflow_id, %node_id, "adding node to running dataflow");

                let result: eyre::Result<()> = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
                    let base_working_dir = self
                        .working_dir
                        .get(&dataflow_id)
                        .cloned()
                        .unwrap_or_else(|| std::path::PathBuf::from("."));

                    // Collect input metadata for post-spawn registration.
                    // State mutations are DEFERRED until after the spawn
                    // succeeds so a spawn failure doesn't leave stale
                    // routing/deadline/pending state behind.
                    let inputs = node_inputs(&node);
                    let is_dynamic = node.kind.dynamic();
                    reject_unpinnable_backpressure_inputs(dataflow, &node_id, &inputs)?;

                    // Startup-handshake routing for the added node, from the
                    // *live* dataflow state rather than the (stale) descriptor
                    // — see `added_node_output_routing` for the policy
                    // (existing receivers on a re-added id handshake as usual;
                    // receiver-less outputs are pinned to the daemon path so
                    // `dora node connect` edges can deliver).
                    let mut output_routing = output_routing::added_node_output_routing(
                        &node_id,
                        node.kind.run_config().outputs,
                        &dataflow.mappings,
                        &dataflow.open_external_mappings,
                        &dataflow.dynamic_nodes,
                        |receiver, input_id| {
                            dataflow.input_requires_backpressure(receiver, input_id)
                        },
                    );
                    output_routing::pin_backpressure_self_loops(
                        &node_id,
                        &inputs,
                        &mut output_routing,
                    );

                    // Prepare stderr buffer (harmless — just an empty
                    // ArrayQueue, no routing implications).
                    let node_stderr = dataflow
                        .node_stderr_most_recent
                        .entry(node_id.clone())
                        .or_insert_with(|| Arc::new(ArrayQueue::new(STDERR_LOG_LINES_MAX)))
                        .clone();

                    // --- Spawn the node (before any routing state is touched) ---
                    let descriptor = dataflow.descriptor.clone();
                    let spawner = Spawner {
                        dataflow_id,
                        daemon_tx: self.events_tx.clone(),
                        dataflow_descriptor: descriptor,
                        clock: self.clock.clone(),
                        uv,
                        ft_stats: self.ft_stats.clone(),
                        shutdown: dataflow.listener_shutdown_rx.clone(),
                        zenoh_connect_endpoint: self.zenoh_listen_endpoint.clone(),
                        zenoh_peering: dataflow.zenoh_peering.clone(),
                        disable_multicast: self.disable_multicast,
                        bind_nodes_to_parent: self.bind_nodes_to_parent,
                        machine_id: self.machine_id.clone(),
                    };
                    let mut logger = self
                        .logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .try_clone()
                        .await
                        .context("failed to clone logger")?;
                    // Re-derive the managed Python env dir deterministically so
                    // node restarts reuse the same venv that `dora build` prepared.
                    let python_env_dir =
                        dora_core::build::managed_python_env_dir(&node, &base_working_dir);
                    let task = spawner
                        .spawn_node(
                            node.clone(),
                            base_working_dir,
                            python_env_dir,
                            false,
                            node_stderr,
                            None,
                            output_routing,
                            &mut logger,
                        )
                        .await
                        .wrap_err("failed to prepare node")?;
                    let prepared = task.await.wrap_err("failed to build node")?;
                    let mut running_node = prepared
                        .spawn(logger)
                        .await
                        .wrap_err("failed to spawn node")?;

                    // --- Spawn succeeded — now apply state mutations ---
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("dataflow disappeared during spawn"))?;

                    // Register inputs (open_inputs, mappings, timers, deadlines)
                    for (input_id, input) in &inputs {
                        dataflow
                            .open_inputs
                            .entry(node_id.clone())
                            .or_default()
                            .insert(input_id.clone());
                        // See the spawn path: only `User` inputs can ever
                        // close, so only they gate the drain (#2920).
                        if matches!(input.mapping, InputMapping::User(_)) {
                            dataflow
                                .data_inputs
                                .entry(node_id.clone())
                                .or_default()
                                .insert(input_id.clone());
                        }
                        match &input.mapping {
                            InputMapping::User(mapping) => {
                                if let Some(timeout) = input.input_timeout {
                                    dataflow.input_deadlines.insert(
                                        (node_id.clone(), input_id.clone()),
                                        InputDeadline {
                                            timeout: Duration::from_secs_f64(timeout),
                                            last_received: None,
                                        },
                                    );
                                }
                                dataflow
                                    .mappings
                                    .entry(OutputId(mapping.source.clone(), mapping.output.clone()))
                                    .or_default()
                                    .insert((node_id.clone(), input_id.clone()));
                            }
                            InputMapping::Timer { interval } => {
                                dataflow
                                    .timers
                                    .entry(*interval)
                                    .or_default()
                                    .insert((node_id.clone(), input_id.clone()));
                            }
                            InputMapping::Logs(filter) => {
                                dataflow.log_subscribers.push(
                                    crate::running_dataflow::LogSubscriber {
                                        node_id: node_id.clone(),
                                        input_id: input_id.clone(),
                                        filter: filter.clone(),
                                    },
                                );
                            }
                        }
                    }

                    if is_dynamic {
                        dataflow.dynamic_nodes.insert(node_id.clone());
                    }
                    // Deliberately NOT enrolled in `pending_nodes`: this
                    // node is not part of the descriptor the dataflow
                    // started from, so it must neither gate that
                    // cohort's barrier nor inherit its failures.
                    // Enrolling it meant a crash here was broadcast as
                    // the subscribe result of unrelated nodes, and a
                    // node that never subscribed could stall startup
                    // outright (dora-rs/dora#2917).

                    // Open the restart-loop gate and insert the running
                    // node. Marking before the insert is safe: the loop's
                    // first events queue behind this handler on the same
                    // event loop, so they cannot be processed before the
                    // entry is registered.
                    running_node.mark_registered();
                    dataflow.running_nodes.insert(node_id.clone(), running_node);

                    // Update the daemon's stored descriptor so
                    // descriptor-based lookups (e.g. AllInputsClosed
                    // check at handle_outputs_done) find the new node.
                    // Construct a minimal Node from the inputs we
                    // already collected — only `id` and `inputs` are
                    // consulted by the daemon.
                    let mut added = dora_message::descriptor::Node::new(node_id.clone());
                    added.inputs = inputs.into_iter().collect();
                    dataflow.descriptor.nodes.push(added);

                    // Spawn timer tasks for any interval this node just
                    // registered. Timer tasks are only ever created in
                    // `start()`, so a node added to an already-running dataflow
                    // whose timer input uses an interval no existing node uses
                    // would otherwise register into `dataflow.timers` but never
                    // get a tick-emitting task — silently starving that input.
                    // `start()` is idempotent (it skips intervals that already
                    // have a handle), so this only spawns tasks for genuinely
                    // new intervals. Guard on `dataflow_started` so that during
                    // initial bring-up the readiness path stays the sole
                    // trigger. Done last, after all state mutations, so a
                    // spawn/registration failure above never leaves a half-added
                    // node with live timer tasks.
                    if dataflow.dataflow_started {
                        dataflow.start(&self.events_tx, &self.clock).await?;
                    }

                    tracing::info!(
                        %dataflow_id,
                        %node_id,
                        dynamic = is_dynamic,
                        "node added successfully"
                    );
                    Ok(())
                }
                .await;

                if let Err(err) = &result {
                    tracing::error!(%dataflow_id, %node_id, "AddNode failed: {err:?}");
                }
                if result.is_ok() {
                    clear_node_result(&mut self.dataflow_node_results, dataflow_id, &node_id);
                }
                // Return a specific `AddNodeResult` variant so the
                // coordinator can validate the reply against its
                // expected request, instead of treating any non-error
                // reply as success (#1682, rescue of #1757).
                let reply =
                    DaemonCoordinatorReply::AddNodeResult(result.map_err(|err| format!("{err:?}")));
                let _ = reply_tx.send(Some(reply));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::RemoveNode {
                dataflow_id,
                node_id,
                grace_duration,
            } => {
                tracing::info!(%dataflow_id, %node_id, "removing node from running dataflow");
                let result: eyre::Result<()> = (|| {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
                    dataflow.stop_single_node(&node_id, &self.clock, grace_duration)?;

                    // Clean up routing tables: remove all mappings where this
                    // node is a source, and close inputs on downstream nodes.
                    let outputs_to_remove: Vec<OutputId> = dataflow
                        .mappings
                        .keys()
                        .filter(|oid| oid.0 == node_id)
                        .cloned()
                        .collect();
                    for output_id in outputs_to_remove {
                        if let Some(receivers) = dataflow.mappings.remove(&output_id) {
                            for (receiver_id, input_id) in receivers {
                                let _ = close_input(dataflow, &receiver_id, &input_id, &self.clock);
                            }
                        }
                    }

                    // Remove all mappings where this node is a receiver.
                    for receivers in dataflow.mappings.values_mut() {
                        receivers.retain(|(nid, _)| nid != &node_id);
                    }
                    // Drop this node's timer/log virtual-input subscriptions —
                    // both to stop delivering to a removed node and so a re-added
                    // ID is classified by its own inputs, not stale timer/log
                    // state (which would mark it never-finishing forever, #2270).
                    // Cancels the timer task of any interval left with no
                    // subscribers (#2585); see the method for details.
                    dataflow.unsubscribe_node_from_timers(&node_id);
                    dataflow
                        .log_subscribers
                        .retain(|sub| sub.node_id != node_id);

                    // Clean up remaining state for this node.
                    dataflow.running_nodes.remove(&node_id);
                    dataflow.open_inputs.remove(&node_id);
                    dataflow.data_inputs.remove(&node_id);
                    dataflow.subscribe_channels.remove(&node_id);
                    dataflow.pending_messages.remove(&node_id);
                    dataflow.all_inputs_closed_at.remove(&node_id);
                    // clear the connected marker too, else a re-added node ID
                    // would look already-connected before its new incarnation
                    // subscribes and could be selected mid-startup (dora#2270).
                    dataflow.connected_nodes.remove(&node_id);
                    dataflow.finish_escalated.remove(&node_id);
                    // Purge per-node bookkeeping keyed by node id that the
                    // routing cleanup above doesn't touch. Otherwise stale
                    // input_deadlines/broken_inputs entries are re-scanned
                    // every tick forever and the stderr queue leaks across
                    // repeated dynamic add/remove cycles.
                    dataflow.forget_node_bookkeeping(&node_id);

                    // Remove from stored descriptor (inverse of AddNode
                    // push) so descriptor-based lookups stay consistent.
                    dataflow.descriptor.nodes.retain(|n| n.id != node_id);
                    Ok(())
                })();

                // Reclaim the removed node's extension-table entries. Its
                // later `SpawnedNodeResult` exit event is swallowed by the
                // generation guard (the node is already gone from
                // `running_nodes`), so the reclaim at that call site is
                // unreachable for a removed node. Without a reclaim here,
                // repeated dynamic add/remove cycles of an entry-owning node
                // leak entries toward the per-dataflow cap and readers never
                // receive the drop notification (dora-rs/dora#3177,
                // originally #2881/#3014).
                //
                // Reclaiming here rather than only on the node's death also
                // notifies readers promptly, and is the ONLY release for a
                // node that never reports an exit at all (a dynamic node has
                // no process handle to wait on). What it does not cover —
                // stores made during the stop grace window — is picked up by
                // the second reclaim on the dropped exit event.
                if result.is_ok() {
                    self.reclaim_extensions(dataflow_id, &node_id);
                }

                // Outside the closure because it is async. Why removal
                // has to drive the barrier at all: see
                // `PendingNodes::handle_node_removal`.
                let result = match result {
                    Err(err) => Err(err),
                    Ok(()) => {
                        let mut logger = self.logger.for_dataflow(dataflow_id);
                        // The closure above already resolved this id, and
                        // nothing awaits in between, so a miss here is a
                        // bug rather than a race — report it instead of
                        // returning success like the closure's own
                        // `no running dataflow` arm would.
                        match self.running.get_mut(&dataflow_id) {
                            Some(dataflow) => {
                                let status = dataflow
                                    .pending_nodes
                                    .handle_node_removal(
                                        &node_id,
                                        &mut self.coordinator_sender,
                                        &self.clock,
                                        &mut dataflow.cascading_error_causes,
                                        &mut logger,
                                    )
                                    .await;
                                match status {
                                    // Removing the last pending cohort member
                                    // completes the barrier, and a `RemoveNode`
                                    // can land mid-teardown, so this shares the
                                    // `!stop_sent` gate with the subscribe and
                                    // node-death triggers (dora-rs/dora#3053).
                                    Ok(status)
                                        if dataflow.should_start_on_barrier_completion(&status) =>
                                    {
                                        logger
                                            .log(
                                                LogLevel::Info,
                                                None,
                                                Some("daemon".into()),
                                                "all nodes are ready after node removal, \
                                                 starting dataflow",
                                            )
                                            .await;
                                        dataflow.start(&self.events_tx, &self.clock).await
                                    }
                                    Ok(_) => Ok(()),
                                    Err(err) => Err(err),
                                }
                            }
                            None => Err(eyre!(
                                "dataflow `{dataflow_id}` disappeared while removing `{node_id}`"
                            )),
                        }
                    }
                };

                if let Err(err) = &result {
                    tracing::error!(%dataflow_id, %node_id, "RemoveNode failed: {err:?}");
                }
                let reply = DaemonCoordinatorReply::RemoveNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::ReplaceNode {
                dataflow_id,
                node,
                unresolved_node,
                uv,
                grace_duration,
            } => {
                let node_id = node.id.clone();
                tracing::info!(%dataflow_id, %node_id, "replacing node in running dataflow");

                let result: eyre::Result<()> = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;
                    // Replace must not swap a node the startup barrier is
                    // still waiting on: the barrier tracks the original
                    // incarnation's subscription, and a mid-barrier swap
                    // would let the replacement inherit or corrupt that
                    // cohort's state (cf. AddNode's deliberate
                    // non-enrollment, dora-rs/dora#2917). Deliberately NOT
                    // gated on `dataflow_started`: that flag is set by the
                    // all-daemons-ready roundtrip, which can lag seconds
                    // behind this node being visibly Running — the precise
                    // hazard is this id gating the barrier, nothing more.
                    eyre::ensure!(
                        !dataflow.pending_nodes.is_pending(&node_id),
                        "node `{node_id}` is still starting (startup barrier); \
                         retry once the dataflow is ready"
                    );
                    eyre::ensure!(
                        dataflow.running_nodes.contains_key(&node_id),
                        "no running node `{node_id}` to replace; use `dora node add`"
                    );
                    // v1 scope (dora-rs/dora#2927): spawned custom nodes
                    // only. A dynamic replacement spawns no process (the
                    // command would kill the old node and leave nothing
                    // running behind the id), an outgoing dynamic node has
                    // no handle for the grace-kill escalation, and
                    // runtime/operator nodes keep their inputs in a
                    // different descriptor location than the node-level
                    // comparison below.
                    eyre::ensure!(
                        !node.kind.dynamic()
                            && matches!(node.kind, dora_core::descriptor::CoreNodeKind::Custom(_)),
                        "`dora node replace` currently supports spawned custom nodes only; \
                         the replacement definition for `{node_id}` is a {} node — use \
                         `dora node remove` + `dora node add` instead",
                        if node.kind.dynamic() {
                            "dynamic"
                        } else {
                            "runtime/operator"
                        }
                    );
                    eyre::ensure!(
                        !dataflow.dynamic_nodes.contains(&node_id),
                        "node `{node_id}` is a dynamic node; `dora node replace` currently \
                         supports spawned custom nodes only — use `dora node remove` + \
                         `dora node add` instead"
                    );

                    // --- same-edges validation -------------------------------
                    // A replace is a swap, not a topology edit
                    // (dora-rs/dora#2927): the replacement must keep the
                    // node's LIVE input edges exactly and still produce
                    // every output a consumer (local or on a remote
                    // daemon) is mapped to. The live edge state
                    // (mappings/timers/log subscriptions) is the source of
                    // truth rather than the stored descriptor entry: entry
                    // input locations vary by node kind and by how the
                    // node entered the dataflow (spawn vs AddNode), while
                    // the live maps are uniformly keyed and already
                    // reflect `dora node connect`/`disconnect` edits.
                    let new_inputs = node_inputs(&node);
                    reject_unpinnable_backpressure_inputs(dataflow, &node_id, &new_inputs)?;
                    let mut current_edges: BTreeMap<DataId, InputMapping> = BTreeMap::new();
                    for (output_id, receivers) in &dataflow.mappings {
                        for (receiver, input_id) in receivers {
                            if receiver == &node_id {
                                current_edges.insert(
                                    input_id.clone(),
                                    InputMapping::User(dora_message::config::UserInputMapping {
                                        source: output_id.0.clone(),
                                        output: output_id.1.clone(),
                                    }),
                                );
                            }
                        }
                    }
                    for (interval, receivers) in &dataflow.timers {
                        for (receiver, input_id) in receivers {
                            if receiver == &node_id {
                                current_edges.insert(
                                    input_id.clone(),
                                    InputMapping::Timer {
                                        interval: *interval,
                                    },
                                );
                            }
                        }
                    }
                    for subscriber in &dataflow.log_subscribers {
                        if subscriber.node_id == node_id {
                            current_edges.insert(
                                subscriber.input_id.clone(),
                                InputMapping::Logs(subscriber.filter.clone()),
                            );
                        }
                    }
                    const EDGE_HINT: &str = "`dora node replace` keeps the node's edges — use \
                         `dora node remove`/`add` or `dora node connect`/`disconnect` for \
                         topology changes";
                    for (input_id, input) in &new_inputs {
                        match current_edges.remove(input_id) {
                            Some(mapping) if mapping == input.mapping => {}
                            Some(_) => {
                                eyre::bail!("replacement remaps input `{input_id}`; {EDGE_HINT}")
                            }
                            None => {
                                eyre::bail!("replacement adds input `{input_id}`; {EDGE_HINT}")
                            }
                        }
                    }
                    if let Some((input_id, _)) = current_edges.into_iter().next() {
                        eyre::bail!("replacement drops input `{input_id}`; {EDGE_HINT}");
                    }
                    // `node_output_ids` chains local mappings AND
                    // `open_external_mappings`, so outputs consumed only by
                    // nodes on other daemons are covered too.
                    let new_outputs = node.kind.run_config().outputs;
                    for output_id in dataflow.node_output_ids(&node_id) {
                        eyre::ensure!(
                            new_outputs.contains(&output_id),
                            "replacement does not declare output `{output_id}`, which \
                             downstream nodes consume"
                        );
                    }

                    // --- spawn the replacement (old incarnation untouched) ---
                    // Any failure up to and including the spawn leaves the
                    // current incarnation running (dora-rs/dora#2927).
                    // NOTE: this block deliberately mirrors the AddNode
                    // arm's spawn sequence above — keep the two in sync
                    // when adding Spawner fields or spawn parameters.
                    let base_working_dir = self
                        .working_dir
                        .get(&dataflow_id)
                        .cloned()
                        .unwrap_or_else(|| std::path::PathBuf::from("."));
                    let is_dynamic = node.kind.dynamic();
                    let mut output_routing = output_routing::added_node_output_routing(
                        &node_id,
                        node.kind.run_config().outputs,
                        &dataflow.mappings,
                        &dataflow.open_external_mappings,
                        &dataflow.dynamic_nodes,
                        |receiver, input_id| {
                            dataflow.input_requires_backpressure(receiver, input_id)
                        },
                    );
                    output_routing::pin_backpressure_self_loops(
                        &node_id,
                        &new_inputs,
                        &mut output_routing,
                    );
                    // Fresh stderr buffer for the new incarnation; installed
                    // into the map only after the spawn succeeds.
                    let node_stderr = Arc::new(ArrayQueue::new(STDERR_LOG_LINES_MAX));
                    // Replace the descriptor CLONE handed to the spawner with
                    // the replacement's definition before it is serialized
                    // into the child's DORA_NODE_CONFIG — the stored
                    // descriptor still describes the outgoing incarnation
                    // at this point (state mutations are deferred until the
                    // spawn succeeds). Assigning the coordinator-supplied
                    // original YAML-shape node wholesale keeps every field
                    // (path/checksum/build/type maps/cpu_affinity/deploy/
                    // git-source/log-rotation/…) paired with the replacement,
                    // with no field list to drift as new fields are added to
                    // `Node` (dora-rs/dora#2988 review, finding 2).
                    let mut descriptor = dataflow.descriptor.clone();
                    if let Some(entry) = descriptor.nodes.iter_mut().find(|n| n.id == node_id) {
                        *entry = unresolved_node.clone();
                    }
                    let spawner = Spawner {
                        dataflow_id,
                        daemon_tx: self.events_tx.clone(),
                        dataflow_descriptor: descriptor,
                        clock: self.clock.clone(),
                        uv,
                        ft_stats: self.ft_stats.clone(),
                        shutdown: dataflow.listener_shutdown_rx.clone(),
                        zenoh_connect_endpoint: self.zenoh_listen_endpoint.clone(),
                        zenoh_peering: dataflow.zenoh_peering.clone(),
                        disable_multicast: self.disable_multicast,
                        bind_nodes_to_parent: self.bind_nodes_to_parent,
                        machine_id: self.machine_id.clone(),
                    };
                    let mut logger = self
                        .logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .try_clone()
                        .await
                        .context("failed to clone logger")?;
                    let python_env_dir =
                        dora_core::build::managed_python_env_dir(&node, &base_working_dir);
                    let task = spawner
                        .spawn_node(
                            node.clone(),
                            base_working_dir,
                            python_env_dir,
                            false,
                            node_stderr.clone(),
                            None,
                            output_routing,
                            &mut logger,
                        )
                        .await
                        .wrap_err("failed to prepare replacement node")?;
                    let prepared = task.await.wrap_err("failed to build replacement node")?;
                    let mut running_node = prepared
                        .spawn(logger)
                        .await
                        .wrap_err("failed to spawn replacement node")?;

                    // --- commit: swap entries and stop the outgoing one ------
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .ok_or_else(|| eyre!("dataflow disappeared during replacement spawn"))?;
                    // If the entry vanished mid-spawn (concurrent remove),
                    // dropping `running_node` kills the fresh process via
                    // its ProcessHandle Drop and cancels its (ungated)
                    // restart loop — no orphan.
                    let mut outgoing =
                        dataflow.running_nodes.remove(&node_id).ok_or_else(|| {
                            eyre!("node `{node_id}` was removed during the replacement spawn")
                        })?;
                    outgoing.disable_restart();
                    let outgoing_generation = outgoing.generation;
                    let outgoing_process = outgoing.process.take();
                    // Stop the outgoing incarnation while its subscribe
                    // channel is still installed so the graceful `Stop`
                    // reaches it; its later exit event carries
                    // `outgoing_generation` and is dropped by the
                    // generation guard instead of being attributed to the
                    // replacement (dora-rs/dora#2926).
                    dataflow.stop_replaced_incarnation(
                        &node_id,
                        outgoing_generation,
                        outgoing_process,
                        &self.clock,
                        grace_duration,
                    );

                    // Per-incarnation state reset. Three deliberate
                    // NON-resets:
                    // - mappings/timers/log subscriptions are keyed by
                    //   (node id, input id) and the edges are validated
                    //   identical, so they carry over unchanged;
                    // - `open_inputs`/`data_inputs` record DATAFLOW-level
                    //   input closure (an upstream that already finished),
                    //   which is delivered exactly once and afterwards
                    //   reconstructed from these maps by the subscribe-time
                    //   replay — resetting them would make a replacement
                    //   installed after an upstream finished wait forever
                    //   on a dead input and hang dataflow completion;
                    // - the descriptor entry keeps the outgoing
                    //   definition's other fields (path/env/...) — stale
                    //   metadata the daemon itself never consults (only
                    //   `id` and `inputs` are read; the coordinator's
                    //   descriptor holds the authoritative new definition).
                    dataflow.subscribe_channels.remove(&node_id);
                    dataflow.pending_messages.remove(&node_id);
                    dataflow.all_inputs_closed_at.remove(&node_id);
                    dataflow.connected_nodes.remove(&node_id);
                    dataflow.finish_escalated.remove(&node_id);
                    // Also drops the cascading-error cause and the id's
                    // `OutputId`-keyed remote publishers (re-declared on the
                    // replacement's next remote send); `debug_topic_watchers`
                    // is left intact on every path. See the method's doc
                    // comment for the full rationale.
                    dataflow.forget_node_bookkeeping(&node_id);
                    dataflow
                        .node_stderr_most_recent
                        .insert(node_id.clone(), node_stderr);

                    // Re-register input deadlines, but only for inputs that
                    // are still open — a deadline on an already-closed
                    // input would be re-scanned forever without ever
                    // arming.
                    let still_open = dataflow.open_inputs.get(&node_id).cloned();
                    for (input_id, input) in &new_inputs {
                        if matches!(input.mapping, InputMapping::User(_))
                            && let Some(timeout) = input.input_timeout
                            && still_open
                                .as_ref()
                                .is_some_and(|open| open.contains(input_id))
                        {
                            dataflow.input_deadlines.insert(
                                (node_id.clone(), input_id.clone()),
                                InputDeadline {
                                    timeout: Duration::from_secs_f64(timeout),
                                    last_received: None,
                                },
                            );
                        }
                    }

                    // Commit the same wholesale assignment on the stored
                    // descriptor so later spawns (and restart respawns of
                    // OTHER nodes) serialize the replacement, not the
                    // outgoing incarnation. Mirrors the coordinator's own
                    // `*existing = original_node` commit.
                    if let Some(entry) = dataflow
                        .descriptor
                        .nodes
                        .iter_mut()
                        .find(|n| n.id == node_id)
                    {
                        *entry = unresolved_node.clone();
                    }

                    running_node.mark_registered();
                    dataflow.running_nodes.insert(node_id.clone(), running_node);

                    tracing::info!(
                        %dataflow_id,
                        %node_id,
                        outgoing_generation,
                        dynamic = is_dynamic,
                        "node replaced successfully"
                    );
                    Ok(())
                }
                .await;

                if let Err(err) = &result {
                    tracing::error!(%dataflow_id, %node_id, "ReplaceNode failed: {err:?}");
                }
                if result.is_ok() {
                    // The outgoing (or an even earlier) incarnation's stale
                    // result must not be attributed to the replacement.
                    clear_node_result(&mut self.dataflow_node_results, dataflow_id, &node_id);
                    // Reclaim the outgoing incarnation's extension-table
                    // entries. Its exit event is dropped by the generation
                    // guard, so this is the only reachable point to release
                    // them (dora-rs/dora#3177, originally #2881/#3014).
                    //
                    // Unlike `RemoveNode` this needs no second reclaim on the
                    // exit event, and it drops only the outgoing incarnation's
                    // entries: the replacement's higher generation is already
                    // installed above, and the daemon's event loop is serial,
                    // so it has not stored anything yet and the outgoing
                    // incarnation's later stores are dropped as superseded
                    // (#2926, #2927) instead of landing under the shared id.
                    self.reclaim_extensions(dataflow_id, &node_id);
                }
                let reply = DaemonCoordinatorReply::ReplaceNodeResult(
                    result.map_err(|err| format!("{err:?}")),
                );
                let _ = reply_tx.send(Some(reply));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::AddMapping {
                dataflow_id,
                source_node,
                source_output,
                target_node,
                target_input,
            } => {
                tracing::info!(%dataflow_id, "{source_node}/{source_output} -> {target_node}/{target_input}");
                // Previously this handler ignored unknown dataflow_id and
                // always replied `None`, which the WS layer dropped on the
                // floor → coordinator timed out after 30s without ever
                // knowing whether the mapping applied. Reply with an
                // explicit `AddMappingResult` so the coordinator can pattern-
                // match the outcome (same class as #1682's AddNodeResult).
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    dataflow.add_mapping(source_node, source_output, target_node, target_input);
                    Ok(())
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::AddMappingResult(result)));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::RemoveMapping {
                dataflow_id,
                source_node,
                source_output,
                target_node,
                target_input,
            } => {
                tracing::info!(%dataflow_id, "{source_node}/{source_output} -x- {target_node}/{target_input}");
                // Same silent-reply fix as AddMapping above.
                // Errors on missing-mapping (rather than silently succeeding)
                // to match the `RemoveNode` semantics in stop_single_node,
                // which surface "node not found" as a daemon Err. A double-
                // disconnect or typo'd edge then produces a clear CLI error
                // instead of a misleading "Mapping removed" message.
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    let output_id = OutputId(source_node.clone(), source_output.clone());
                    let removed = dataflow
                        .mappings
                        .get_mut(&output_id)
                        .map(|r| r.remove(&(target_node.clone(), target_input.clone())))
                        .unwrap_or(false);
                    if removed {
                        let _ = close_input(dataflow, &target_node, &target_input, &self.clock);
                        Ok(())
                    } else {
                        Err(format!(
                            "mapping `{source_node}/{source_output}` -> \
                             `{target_node}/{target_input}` not found"
                        ))
                    }
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::RemoveMappingResult(result)));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StartTopicDebugStream {
                dataflow_id,
                outputs,
                subscription_id,
                ..
            } => {
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    for (node_id, data_id) in outputs {
                        dataflow
                            .debug_topic_watchers
                            .entry(OutputId(node_id, data_id))
                            .or_default()
                            .insert(subscription_id);
                    }
                    Ok(())
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::StartTopicDebugStreamResult(
                    result,
                )));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StopTopicDebugStream {
                dataflow_id,
                subscription_id,
            } => {
                let result = if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    // Scan watchers rather than maintain an inverse map. Unsubscribe
                    // is rare; scan is bounded by the count of outputs with active
                    // subscribers. retain() drops empty entries in one pass.
                    dataflow
                        .debug_topic_watchers
                        .retain(|_output_id, watchers| {
                            watchers.remove(&subscription_id);
                            !watchers.is_empty()
                        });
                    Ok(())
                } else {
                    Err(format!("no running dataflow with ID `{dataflow_id}`"))
                };
                let _ = reply_tx.send(Some(DaemonCoordinatorReply::StopTopicDebugStreamResult(
                    result,
                )));
                RunStatus::Continue
            }
            DaemonCoordinatorEvent::StateCatchUp {
                dataflow_id,
                entries,
            } => {
                let max_seq = entries.last().map(|e| e.sequence).unwrap_or(0);
                tracing::info!(
                    %dataflow_id,
                    "state catch-up: applying {} entry(ies) (up to seq {max_seq})",
                    entries.len(),
                );
                let applied_through = match self.running.get(&dataflow_id) {
                    Some(dataflow) => apply_state_catch_up_entries(dataflow, &entries, &self.clock),
                    None => {
                        tracing::warn!(
                            "state catch-up: dataflow `{dataflow_id}` no longer running on daemon"
                        );
                        0
                    }
                };
                // Ack only the prefix that was actually accepted for delivery.
                if applied_through > 0
                    && let Some(sender) = &self.coordinator_sender
                {
                    let ack = DaemonEvent::StateCatchUpAck {
                        dataflow_id,
                        ack_sequence: applied_through,
                    };
                    let stamped = Timestamped {
                        inner: CoordinatorRequest::Event {
                            daemon_id: self.daemon_id.clone(),
                            event: ack,
                        },
                        timestamp: self.clock.new_timestamp(),
                    };
                    if let Ok(bytes) = serde_json::to_vec(&stamped)
                        && let Err(err) = sender.send_event(&bytes).await
                    {
                        tracing::warn!("failed to send state catch-up ack to coordinator: {err}");
                    }
                }
                let _ = reply_tx.send(None);
                RunStatus::Continue
            }
        };
        Ok(status)
    }
}
