//! Requests from the CLI (and other control clients) over the WebSocket
//! control plane.

use crate::{
    ControlEvent, Coordinator, cancel_pending_restart, ensure_add_mapping_applied,
    ensure_add_node_applied, ensure_delete_param_forward_applied, ensure_remove_mapping_applied,
    ensure_remove_node_applied, ensure_replace_node_applied, ensure_set_param_forward_applied,
    handle_get_trace_spans, handle_get_traces,
    handlers::{
        build_dataflow, dataflow_result, handle_destroy, parse_logs_node_id, reload_dataflow,
        resolve_name, restart_node, retrieve_logs, send_log_message, start_dataflow, stop_dataflow,
        stop_node,
    },
    initiate_restart,
    log_subscriber::LogSubscriber,
    resolve_param_target, resolve_single_node, start_topic_debug_stream,
    state::{ParamTarget, RunningDataflow},
    stop_topic_debug_stream, topic_debug_enabled, topic_outputs_by_daemon,
};
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
use dora_message::{
    BuildId, SessionId,
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{
        CleanFailure, ControlRequestReply, DataflowIdAndName, DataflowList, DataflowListEntry,
        DataflowResult, DataflowStatus,
    },
    coordinator_to_daemon::{DaemonCoordinatorEvent, StateCatchUpOperation, Timestamped},
    descriptor::{Descriptor, Node},
    id::{DataId, NodeId},
};
use eyre::{Result, bail, eyre};
use petname::petname;
use std::{path::PathBuf, time::Duration};
use uuid::Uuid;

type ReplySender = tokio::sync::oneshot::Sender<eyre::Result<ControlRequestReply>>;

impl Coordinator {
    pub(crate) async fn handle_control_event(&mut self, event: ControlEvent) -> eyre::Result<()> {
        match event {
            ControlEvent::IncomingRequest {
                request,
                reply_sender,
            } => {
                match *request {
                    ControlRequest::Build {
                        session_id,
                        dataflow,
                        git_sources,
                        prev_git_sources,
                        local_working_dir,
                        uv,
                    } => {
                        // assign a random build id
                        let build_id = BuildId::generate();

                        let result = build_dataflow(
                            build_id,
                            session_id,
                            dataflow,
                            git_sources,
                            prev_git_sources,
                            local_working_dir,
                            &self.clock,
                            uv,
                            &mut self.daemon_connections,
                        )
                        .await;
                        match result {
                            Ok(build) => {
                                self.running_builds.insert(build_id, build);
                                let _ = reply_sender.send(Ok(
                                    ControlRequestReply::DataflowBuildTriggered { build_id },
                                ));
                            }
                            Err(err) => {
                                let _ = reply_sender.send(Err(err));
                            }
                        }
                    }
                    ControlRequest::WaitForBuild { build_id } => {
                        if let Some(build) = self.running_builds.get_mut(&build_id) {
                            build.build_result.register(reply_sender);
                        } else if let Some(result) = self.finished_builds.get_mut(&build_id) {
                            result.register(reply_sender);
                        } else {
                            let _ = reply_sender.send(Err(eyre!("unknown build id {build_id}")));
                        }
                    }
                    ControlRequest::Start {
                        build_id,
                        session_id,
                        dataflow,
                        name,
                        local_working_dir,
                        uv,
                        write_events_to,
                    } => {
                        self.handle_start(
                            build_id,
                            session_id,
                            dataflow,
                            name,
                            local_working_dir,
                            uv,
                            write_events_to,
                            reply_sender,
                        )
                        .await?
                    }
                    ControlRequest::WaitForSpawn { dataflow_id } => {
                        if let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) {
                            dataflow.spawn_result.register(reply_sender);
                        } else {
                            let _ = reply_sender.send(Err(eyre!("unknown dataflow {dataflow_id}")));
                        }
                    }
                    ControlRequest::Check { dataflow_uuid } => {
                        let status = match &self.running_dataflows.get(&dataflow_uuid) {
                            Some(_) => ControlRequestReply::DataflowSpawned {
                                uuid: dataflow_uuid,
                            },
                            None => ControlRequestReply::DataflowStopped {
                                uuid: dataflow_uuid,
                                result: self
                                    .dataflow_results
                                    .get(&dataflow_uuid)
                                    .map(|r| dataflow_result(r, dataflow_uuid, &self.clock))
                                    .unwrap_or_else(|| {
                                        DataflowResult::ok_empty(
                                            dataflow_uuid,
                                            self.clock.new_timestamp(),
                                        )
                                    }),
                            },
                        };
                        let _ = reply_sender.send(Ok(status));
                    }
                    ControlRequest::Reload {
                        dataflow_id,
                        node_id,
                        operator_id,
                    } => {
                        let reload = async {
                            reload_dataflow(
                                &self.running_dataflows,
                                dataflow_id,
                                node_id,
                                operator_id,
                                &mut self.daemon_connections,
                                self.clock.new_timestamp(),
                            )
                            .await?;
                            Result::<_, eyre::Report>::Ok(())
                        };
                        let reply = reload
                            .await
                            .map(|()| ControlRequestReply::DataflowReloaded { uuid: dataflow_id });
                        let _ = reply_sender.send(reply);
                    }
                    ControlRequest::RestartNode {
                        dataflow_id,
                        node_id,
                        grace_duration,
                    } => {
                        let result = restart_node(
                            &self.running_dataflows,
                            dataflow_id,
                            node_id.clone(),
                            grace_duration,
                            &mut self.daemon_connections,
                            self.clock.new_timestamp(),
                        )
                        .await;
                        let reply = result.map(|()| ControlRequestReply::NodeRestarted {
                            dataflow_id,
                            node_id,
                        });
                        let _ = reply_sender.send(reply);
                    }
                    ControlRequest::StopNode {
                        dataflow_id,
                        node_id,
                        grace_duration,
                    } => {
                        let result = stop_node(
                            &self.running_dataflows,
                            dataflow_id,
                            node_id.clone(),
                            grace_duration,
                            &mut self.daemon_connections,
                            self.clock.new_timestamp(),
                        )
                        .await;
                        let reply = result.map(|()| ControlRequestReply::NodeStopped {
                            dataflow_id,
                            node_id,
                        });
                        let _ = reply_sender.send(reply);
                    }
                    ControlRequest::Stop {
                        dataflow_uuid,
                        grace_duration,
                        force,
                    } => {
                        self.handle_stop(dataflow_uuid, grace_duration, force, reply_sender)
                            .await?
                    }
                    ControlRequest::StopByName {
                        name,
                        grace_duration,
                        force,
                    } => {
                        self.handle_stop_by_name(name, grace_duration, force, reply_sender)
                            .await?
                    }
                    ControlRequest::Restart {
                        dataflow_uuid,
                        grace_duration,
                        force,
                    } => {
                        initiate_restart(
                            dataflow_uuid,
                            grace_duration,
                            force,
                            &mut self.running_dataflows,
                            &mut self.pending_restarts,
                            &mut self.daemon_connections,
                            &self.clock,
                            self.store.as_ref(),
                            reply_sender,
                        )
                        .await;
                    }
                    ControlRequest::RestartByName {
                        name,
                        grace_duration,
                        force,
                    } => {
                        match resolve_name(name, &self.running_dataflows, &self.archived_dataflows)
                        {
                            Ok(dataflow_uuid) => {
                                initiate_restart(
                                    dataflow_uuid,
                                    grace_duration,
                                    force,
                                    &mut self.running_dataflows,
                                    &mut self.pending_restarts,
                                    &mut self.daemon_connections,
                                    &self.clock,
                                    self.store.as_ref(),
                                    reply_sender,
                                )
                                .await;
                            }
                            Err(err) => {
                                let _ = reply_sender.send(Err(err));
                            }
                        }
                    }
                    ControlRequest::Logs {
                        uuid,
                        name,
                        node,
                        tail,
                    } => {
                        self.handle_logs(uuid, name, node, tail, reply_sender)
                            .await?
                    }
                    ControlRequest::Info { dataflow_uuid } => {
                        if let Some(dataflow) = self.running_dataflows.get(&dataflow_uuid) {
                            let _ = reply_sender.send(Ok(ControlRequestReply::DataflowInfo {
                                uuid: dataflow.uuid,
                                name: dataflow.name.clone(),
                                descriptor: dataflow.descriptor.clone(),
                            }));
                        } else {
                            let _ = reply_sender.send(Err(eyre!(
                                "No running dataflow with uuid `{dataflow_uuid}`"
                            )));
                        }
                    }
                    ControlRequest::Destroy => {
                        tracing::info!("Received destroy command");

                        let reply = handle_destroy(
                            &mut self.running_dataflows,
                            &mut self.daemon_connections,
                            &self.abort_handle,
                            &self.clock,
                            self.store.as_ref(),
                        )
                        .await
                        .map(|()| ControlRequestReply::DestroyOk);
                        let _ = reply_sender.send(reply);
                    }
                    ControlRequest::List => self.handle_list(reply_sender).await?,
                    ControlRequest::Clean => self.handle_clean(reply_sender).await?,
                    ControlRequest::DaemonConnected => {
                        let running = !self.daemon_connections.is_empty();
                        let _ =
                            reply_sender.send(Ok(ControlRequestReply::DaemonConnected(running)));
                    }
                    ControlRequest::ConnectedMachines => {
                        let daemon_infos: Vec<_> = self
                            .daemon_connections
                            .iter_mut()
                            .map(|(id, conn)| dora_message::coordinator_to_cli::DaemonInfo {
                                daemon_id: id.clone(),
                                last_heartbeat_ago_ms: conn.last_heartbeat.elapsed().as_millis()
                                    as u64,
                                ft_stats: conn.ft_stats.clone(),
                            })
                            .collect();
                        let reply = Ok(ControlRequestReply::ConnectedDaemons(daemon_infos));
                        let _ = reply_sender.send(reply);
                    }
                    ControlRequest::LogSubscribe { .. } => {
                        let _ = reply_sender.send(Err(eyre::eyre!(
                            "LogSubscribe request should be handled separately"
                        )));
                    }
                    ControlRequest::BuildLogSubscribe { .. } => {
                        let _ = reply_sender.send(Err(eyre::eyre!(
                            "BuildLogSubscribe request should be handled separately"
                        )));
                    }
                    ControlRequest::TopicSubscribe { .. } => {
                        let _ = reply_sender.send(Err(eyre::eyre!(
                            "TopicSubscribe request should be handled separately"
                        )));
                    }
                    ControlRequest::TopicUnsubscribe { .. } => {
                        let _ = reply_sender.send(Err(eyre::eyre!(
                            "TopicUnsubscribe request should be handled separately"
                        )));
                    }
                    ControlRequest::TopicPublish { .. } => {
                        let _ = reply_sender.send(Err(eyre::eyre!(
                            "TopicPublish request should be handled separately"
                        )));
                    }
                    ControlRequest::CliAndDefaultDaemonOnSameMachine => {
                        // With WS we can't inspect peer addresses.
                        // If an unnamed (local) daemon is connected, assume same
                        // machine by returning localhost for both.
                        let has_unnamed = self.daemon_connections.unnamed().next().is_some();
                        let ip = if has_unnamed {
                            Some(std::net::IpAddr::V4(std::net::Ipv4Addr::LOCALHOST))
                        } else {
                            None
                        };
                        let _ =
                            reply_sender.send(Ok(ControlRequestReply::CliAndDefaultDaemonIps {
                                default_daemon: ip,
                                cli: ip,
                            }));
                    }
                    ControlRequest::GetNodeInfo => self.handle_get_node_info(reply_sender).await?,
                    ControlRequest::GetTraces => {
                        let reply = handle_get_traces(&self.span_store);
                        let _ = reply_sender.send(Ok(reply));
                    }
                    ControlRequest::GetTraceSpans { trace_id } => {
                        let reply = if trace_id.len() <= 36 && trace_id.is_ascii() {
                            handle_get_trace_spans(&self.span_store, &trace_id)
                        } else {
                            ControlRequestReply::Error("invalid trace_id format".to_string())
                        };
                        let _ = reply_sender.send(Ok(reply));
                    }
                    ControlRequest::GetParams {
                        dataflow_id,
                        node_id,
                    } => {
                        let reply = match resolve_param_target(
                            &self.running_dataflows,
                            self.store.as_ref(),
                            &dataflow_id,
                            &node_id,
                        ) {
                            Err(e) => Err(e),
                            Ok(_) => match self.store.list_node_params(&dataflow_id, &node_id) {
                                Ok(params) => {
                                    let params: Vec<_> = params
                                        .into_iter()
                                        .filter_map(|(k, v)| {
                                            serde_json::from_slice(&v).ok().map(|val| (k, val))
                                        })
                                        .collect();
                                    Ok(ControlRequestReply::ParamList { params })
                                }
                                Err(e) => Err(e),
                            },
                        };
                        let _ = reply_sender.send(reply);
                    }
                    ControlRequest::GetParam {
                        dataflow_id,
                        node_id,
                        key,
                    } => {
                        let reply = match resolve_param_target(
                            &self.running_dataflows,
                            self.store.as_ref(),
                            &dataflow_id,
                            &node_id,
                        ) {
                            Err(e) => Err(e),
                            Ok(_) => {
                                match self.store.get_node_param(&dataflow_id, &node_id, &key) {
                                    Ok(Some(bytes)) => match serde_json::from_slice(&bytes) {
                                        Ok(value) => {
                                            Ok(ControlRequestReply::ParamValue { key, value })
                                        }
                                        Err(e) => Err(eyre::eyre!("corrupt param value: {e}")),
                                    },
                                    Ok(None) => Err(eyre::eyre!("param not found: {key}")),
                                    Err(e) => Err(e),
                                }
                            }
                        };
                        let _ = reply_sender.send(reply);
                    }
                    ControlRequest::SetParam {
                        dataflow_id,
                        node_id,
                        key,
                        value,
                    } => {
                        self.handle_set_param(dataflow_id, node_id, key, value, reply_sender)
                            .await?
                    }
                    ControlRequest::DeleteParam {
                        dataflow_id,
                        node_id,
                        key,
                    } => {
                        self.handle_delete_param(dataflow_id, node_id, key, reply_sender)
                            .await?
                    }
                    // --- Dynamic Topology ---
                    ControlRequest::AddNode { dataflow_id, node } => {
                        self.handle_add_node(dataflow_id, node, reply_sender)
                            .await?
                    }
                    ControlRequest::RemoveNode {
                        dataflow_id,
                        node_id,
                        grace_duration,
                    } => {
                        self.handle_remove_node(dataflow_id, node_id, grace_duration, reply_sender)
                            .await?
                    }
                    ControlRequest::ReplaceNode {
                        dataflow_id,
                        node,
                        grace_duration,
                    } => {
                        self.handle_replace_node(dataflow_id, node, grace_duration, reply_sender)
                            .await?
                    }
                    ControlRequest::AddMapping {
                        dataflow_id,
                        source_node,
                        source_output,
                        target_node,
                        target_input,
                    } => {
                        self.handle_add_mapping(
                            dataflow_id,
                            source_node,
                            source_output,
                            target_node,
                            target_input,
                            reply_sender,
                        )
                        .await?
                    }
                    ControlRequest::RemoveMapping {
                        dataflow_id,
                        source_node,
                        source_output,
                        target_node,
                        target_input,
                    } => {
                        self.handle_remove_mapping(
                            dataflow_id,
                            source_node,
                            source_output,
                            target_node,
                            target_input,
                            reply_sender,
                        )
                        .await?
                    }
                    ControlRequest::Hello { .. } => {
                        // Handled directly in ws_control.rs; never
                        // forwarded to the event loop. This arm exists
                        // only to keep the match exhaustive
                        // (dora-rs/adora#151).
                        let _ = reply_sender.send(Err(eyre!(
                            "Hello must be handled at the WS layer; \
                                 reaching the event loop is a bug"
                        )));
                    }
                }
            }
            ControlEvent::LogSubscribe {
                dataflow_id,
                level,
                sender,
                found_tx,
            } => {
                if let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) {
                    dataflow
                        .log_subscribers
                        .push(LogSubscriber::new(level, sender));
                    let buffered = std::mem::take(&mut dataflow.buffered_log_messages);
                    for message in buffered {
                        send_log_message(&mut dataflow.log_subscribers, &message).await;
                    }
                    let _ = found_tx.send(true);
                } else if self.archived_dataflows.contains_key(&dataflow_id) {
                    // Dataflow already finished before the CLI could subscribe.
                    // Acknowledge the subscription so the CLI doesn't error, then
                    // drop `sender` immediately — the closed channel signals EOF.
                    let _ = found_tx.send(true);
                } else {
                    let _ = found_tx.send(false);
                }
            }
            ControlEvent::BuildLogSubscribe {
                build_id,
                level,
                sender,
                found_tx,
            } => {
                if let Some(build) = self.running_builds.get_mut(&build_id) {
                    build
                        .log_subscribers
                        .push(LogSubscriber::new(level, sender));
                    let buffered = std::mem::take(&mut build.buffered_log_messages);
                    for message in buffered {
                        send_log_message(&mut build.log_subscribers, &message).await;
                    }
                    let _ = found_tx.send(true);
                } else {
                    let _ = found_tx.send(false);
                }
            }
            ControlEvent::TopicSubscribe {
                dataflow_id,
                topics,
                sender,
                done_tx,
            } => {
                let result = start_topic_debug_stream(
                    &mut self.running_dataflows,
                    &mut self.daemon_connections,
                    dataflow_id,
                    topics,
                    sender,
                    &self.clock,
                )
                .await
                .map_err(|err| format!("{err:?}"));
                let _ = done_tx.send(result);
            }
            ControlEvent::TopicCheck {
                dataflow_id,
                topics,
                found_tx,
            } => {
                let found = topic_outputs_by_daemon(&self.running_dataflows, dataflow_id, &topics)
                    .is_ok()
                    && topic_debug_enabled(&self.running_dataflows, dataflow_id).unwrap_or(false);
                let _ = found_tx.send(found);
            }
            ControlEvent::TopicUnsubscribe {
                subscription_id,
                done_tx,
            } => {
                if let Err(err) = stop_topic_debug_stream(
                    &mut self.running_dataflows,
                    &mut self.daemon_connections,
                    subscription_id,
                    &self.clock,
                )
                .await
                {
                    tracing::warn!("failed to unsubscribe topic debug stream: {err:?}");
                }
                let _ = done_tx.send(());
            }
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn handle_start(
        &mut self,
        build_id: Option<BuildId>,
        session_id: SessionId,
        dataflow: Descriptor,
        name: Option<String>,
        local_working_dir: Option<PathBuf>,
        uv: bool,
        write_events_to: Option<PathBuf>,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let name = name.or_else(|| petname(2, "-"));

        let inner = async {
            if let Some(name) = name.as_deref() {
                // check that name is unique
                if self
                    .running_dataflows
                    .values()
                    .any(|d: &RunningDataflow| d.name.as_deref() == Some(name))
                {
                    bail!("there is already a running dataflow with name `{name}`");
                }
            }
            let dataflow = start_dataflow(
                build_id,
                session_id,
                dataflow,
                local_working_dir,
                name,
                &mut self.daemon_connections,
                &self.clock,
                uv,
                write_events_to,
            )
            .await?;
            Ok(dataflow)
        };
        match inner.await {
            Ok(mut dataflow) => {
                let uuid = dataflow.uuid;
                // Persist: dataflow started
                if let Err(e) = dataflow
                    .make_record(StoreDataflowStatus::Pending)
                    .and_then(|r| self.store.put_dataflow(&r))
                {
                    tracing::warn!("failed to persist dataflow start: {e}");
                }
                self.running_dataflows.insert(uuid, dataflow);
                let _ = reply_sender.send(Ok(ControlRequestReply::DataflowStartTriggered { uuid }));
            }
            Err(err) => {
                let _ = reply_sender.send(Err(err));
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_stop(
        &mut self,
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        force: bool,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        // A pending restart already sent `StopDataflow` to
        // the daemon(s) and is waiting for
        // `DataflowFinishedOnDaemon` to spawn the new
        // incarnation under a fresh UUID; `running_dataflows`
        // still contains the old UUID in the meantime. An
        // explicit `Stop` for that UUID means the caller
        // wants the dataflow gone, not restarted — cancel
        // the pending restart (erroring its caller) rather
        // than letting it silently spawn a new incarnation
        // after this stop reports success. Last-writer-wins,
        // and unlike an outright rejection this doesn't
        // block `--force` from ever landing while a
        // long-`--grace-duration` restart is in flight.
        cancel_pending_restart(
            &mut self.pending_restarts,
            dataflow_uuid,
            format!("dataflow `{dataflow_uuid}` was stopped before the restart could complete"),
        );

        // `dataflow_results` is filled incrementally, one
        // entry per daemon, while a multi-daemon dataflow is
        // still running on the others (see
        // `DataflowFinishedOnDaemon`). Only take the
        // already-stopped fast path when the dataflow is
        // truly gone from `running_dataflows`; otherwise fall
        // through to `stop_dataflow` so the daemons that are
        // still running actually get told to stop. Mirrors
        // the `Clean` handler's guard.
        if !self.running_dataflows.contains_key(&dataflow_uuid)
            && let Some(result) = self.dataflow_results.get(&dataflow_uuid)
        {
            let reply = ControlRequestReply::DataflowStopped {
                uuid: dataflow_uuid,
                result: dataflow_result(result, dataflow_uuid, &self.clock),
            };
            let _ = reply_sender.send(Ok(reply));

            return Ok(());
        }

        let dataflow = stop_dataflow(
            &mut self.running_dataflows,
            dataflow_uuid,
            &mut self.daemon_connections,
            self.clock.new_timestamp(),
            grace_duration,
            force,
        )
        .await;

        match dataflow {
            Ok(dataflow) => {
                // Persist: dataflow stopping
                if let Err(e) = dataflow
                    .make_record(StoreDataflowStatus::Stopping)
                    .and_then(|r| self.store.put_dataflow(&r))
                {
                    tracing::warn!("failed to persist dataflow stopping: {e}");
                }
                dataflow.stop_reply_senders.push(reply_sender);
            }
            Err(err) => {
                let _ = reply_sender.send(Err(err));
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_stop_by_name(
        &mut self,
        name: String,
        grace_duration: Option<Duration>,
        force: bool,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        match resolve_name(name, &self.running_dataflows, &self.archived_dataflows) {
            Ok(dataflow_uuid) => {
                // Same pending-restart cancellation as `Stop`
                // — see the comment there for why.
                cancel_pending_restart(
                    &mut self.pending_restarts,
                    dataflow_uuid,
                    format!(
                        "dataflow `{dataflow_uuid}` was stopped before the restart could complete"
                    ),
                );

                // Same partial-completion guard as `Stop`: a
                // still-running multi-daemon dataflow has a
                // partial `dataflow_results` entry, but must
                // still be stopped rather than reported done.
                if !self.running_dataflows.contains_key(&dataflow_uuid)
                    && let Some(result) = self.dataflow_results.get(&dataflow_uuid)
                {
                    let reply = ControlRequestReply::DataflowStopped {
                        uuid: dataflow_uuid,
                        result: dataflow_result(result, dataflow_uuid, &self.clock),
                    };
                    let _ = reply_sender.send(Ok(reply));

                    return Ok(());
                }

                let dataflow = stop_dataflow(
                    &mut self.running_dataflows,
                    dataflow_uuid,
                    &mut self.daemon_connections,
                    self.clock.new_timestamp(),
                    grace_duration,
                    force,
                )
                .await;

                match dataflow {
                    Ok(dataflow) => {
                        // Persist: dataflow stopping
                        if let Err(e) = dataflow
                            .make_record(StoreDataflowStatus::Stopping)
                            .and_then(|r| self.store.put_dataflow(&r))
                        {
                            tracing::warn!("failed to persist dataflow stopping: {e}");
                        }
                        dataflow.stop_reply_senders.push(reply_sender);
                    }
                    Err(err) => {
                        let _ = reply_sender.send(Err(err));
                    }
                }
            }
            Err(err) => {
                let _ = reply_sender.send(Err(err));
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_logs(
        &mut self,
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let dataflow_uuid = if let Some(uuid) = uuid {
            Ok(uuid)
        } else if let Some(name) = name {
            resolve_name(name, &self.running_dataflows, &self.archived_dataflows)
        } else {
            Err(eyre!("No uuid"))
        };

        match dataflow_uuid {
            Ok(uuid) => {
                // `node` arrives as a raw wire `String`, so it may be an
                // invalid node id. Validate it instead of using the panicking
                // `String -> NodeId` conversion, which would unwind the
                // coordinator's single event loop and take down every
                // daemon/CLI connection (control-plane DoS). See #3450 — the
                // node-id sub-case that #650's fix for #648 missed.
                let reply = match parse_logs_node_id(&node) {
                    Ok(node_id) => retrieve_logs(
                        &self.running_dataflows,
                        &self.archived_dataflows,
                        uuid,
                        node_id,
                        &mut self.daemon_connections,
                        self.clock.new_timestamp(),
                        tail,
                    )
                    .await
                    .map(ControlRequestReply::Logs),
                    Err(err) => Err(err),
                };
                let _ = reply_sender.send(reply);
            }
            Err(err) => {
                let _ = reply_sender.send(Err(err));
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_list(&mut self, reply_sender: ReplySender) -> eyre::Result<()> {
        let mut dataflows: Vec<_> = self.running_dataflows.values().collect();
        dataflows.sort_by_key(|d| (&d.name, d.uuid));

        let running = dataflows.into_iter().map(|d| DataflowListEntry {
            id: DataflowIdAndName {
                uuid: d.uuid,
                name: d.name.clone(),
            },
            status: DataflowStatus::Running,
        });
        // Skip uuids still in `running_dataflows`: a
        // partially-finished multi-daemon dataflow has a
        // partial `dataflow_results` entry while it keeps
        // running, and would otherwise be listed twice (once
        // Running, once Finished/Failed) with contradictory
        // statuses. It is already yielded above as Running.
        let finished_failed = self
            .dataflow_results
            .iter()
            .filter(|(uuid, _)| !self.running_dataflows.contains_key(uuid))
            .map(|(&uuid, results)| {
                let name = self
                    .archived_dataflows
                    .get(&uuid)
                    .and_then(|d| d.name.clone());
                let id = DataflowIdAndName { uuid, name };
                let status = if results.values().all(|r| r.is_ok()) {
                    DataflowStatus::Finished
                } else {
                    DataflowStatus::Failed
                };
                DataflowListEntry { id, status }
            });

        let reply = Ok(ControlRequestReply::DataflowList(DataflowList(
            running.chain(finished_failed).collect(),
        )));
        let _ = reply_sender.send(reply);
        Ok(())
    }

    pub(crate) async fn handle_clean(&mut self, reply_sender: ReplySender) -> eyre::Result<()> {
        // `dora clean` semantics (see #1835):
        //
        // * Only FULLY completed dataflows are eligible. For
        //   multi-daemon dataflows `dataflow_results` is
        //   populated incrementally as each daemon finishes,
        //   while the dataflow stays in `running_dataflows`
        //   until ALL daemons are gone. Cleaning a partial
        //   entry would corrupt the final status: when the
        //   last daemon finishes the reply is computed from
        //   the (now-missing) entry and can default to
        //   Succeeded even if an earlier daemon reported a
        //   node failure.
        //
        // * Each cleaned entry is removed from the persisted
        //   store so the on-disk state file doesn't grow
        //   unboundedly. The persisted-store delete cascades
        //   to associated `dora param` rows.
        //
        // * `finished_builds` is intentionally NOT touched —
        //   clearing it would break concurrent `dora build`
        //   calls with "unknown build id" errors.
        //
        // Phase A: enumerate completed candidates from BOTH
        // `dataflow_results` AND `store.list_dataflows()` so
        // a restarted coordinator can still reap historical
        // Succeeded/Failed rows that only exist on disk
        // (startup recovery intentionally does NOT reload
        // them into memory — see the empty match arm at
        // `StoreDataflowStatus::Succeeded | Failed` in the
        // startup loop). Per-candidate tuple:
        // (uuid, name, cli-facing status, in_memory).
        let mut candidates: Vec<(Uuid, Option<String>, DataflowStatus, bool)> = Vec::new();

        for (uuid, results) in self.dataflow_results.iter() {
            if self.running_dataflows.contains_key(uuid) {
                // Multi-daemon dataflow still completing —
                // keep partial results so the final status
                // is computed correctly when the last daemon
                // completes.
                continue;
            }
            let name = self
                .archived_dataflows
                .get(uuid)
                .and_then(|d| d.name.clone());
            let status = if results.values().all(|r| r.is_ok()) {
                DataflowStatus::Finished
            } else {
                DataflowStatus::Failed
            };
            candidates.push((*uuid, name, status, true));
        }

        // Hard-fail if we can't enumerate the persisted
        // store. With a partial view we cannot honor the
        // "trim disk state" contract, and silently
        // processing only the in-memory subset would let
        // the CLI claim "nothing to clean" while
        // historical rows still sit on disk untouched.
        // The in-memory entries we would have processed
        // stay in `dataflow_results`, so a subsequent
        // `dora clean` (after the operator fixes the
        // underlying store issue) reaps them on the next
        // call. No state is mutated on this path.
        let records = match self.store.list_dataflows() {
            Ok(records) => records,
            Err(e) => {
                let _ = reply_sender.send(Err(eyre!(
                    "dora clean: failed to enumerate persisted \
                                         dataflows: {e}. No state was modified; the \
                                         next `dora clean` will retry once the \
                                         coordinator's store is healthy again."
                )));
                return Ok(());
            }
        };
        for record in records {
            if self.running_dataflows.contains_key(&record.uuid) {
                continue;
            }
            if self.dataflow_results.contains_key(&record.uuid) {
                // Already covered by the in-memory pass;
                // skip to avoid double-counting.
                continue;
            }
            let status = match record.status {
                StoreDataflowStatus::Succeeded => DataflowStatus::Finished,
                StoreDataflowStatus::Failed { .. } => DataflowStatus::Failed,
                _ => continue,
            };
            candidates.push((record.uuid, record.name, status, false));
        }

        // Phase B: per-candidate, persist-first, then mutate
        // in-memory state on success. Collect-then-mutate
        // avoids borrow friction with two sources and makes
        // the success/failure split obvious.
        let mut cleaned: Vec<DataflowListEntry> = Vec::new();
        let mut failed: Vec<CleanFailure> = Vec::new();
        for (uuid, name, status, in_memory) in candidates {
            let id = DataflowIdAndName { uuid, name };
            if let Err(e) = self.store.delete_dataflow(&uuid) {
                tracing::warn!(
                    "skipping clean for dataflow {uuid}: \
                                         persisted-store delete failed: {e}. \
                                         {state} preserved so a later `dora clean` \
                                         can retry.",
                    state = if in_memory {
                        "In-memory entry"
                    } else {
                        "Persisted record"
                    }
                );
                failed.push(CleanFailure {
                    id,
                    error: e.to_string(),
                });
                continue;
            }
            if in_memory {
                self.dataflow_results.shift_remove(&uuid);
            }
            self.archived_dataflows.shift_remove(&uuid);
            cleaned.push(DataflowListEntry { id, status });
        }

        cleaned.sort_by(|a, b| {
            (a.id.name.as_deref(), a.id.uuid).cmp(&(b.id.name.as_deref(), b.id.uuid))
        });
        failed.sort_by(|a, b| {
            (a.id.name.as_deref(), a.id.uuid).cmp(&(b.id.name.as_deref(), b.id.uuid))
        });

        let reply = Ok(ControlRequestReply::CleanResult {
            cleaned: DataflowList(cleaned),
            failed,
        });
        let _ = reply_sender.send(reply);
        Ok(())
    }

    pub(crate) async fn handle_get_node_info(
        &mut self,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        use dora_message::coordinator_to_cli::{NodeInfo, NodeMetricsInfo};

        let mut node_infos = Vec::new();
        for dataflow in self.running_dataflows.values() {
            for node_id in dataflow.nodes.keys() {
                // Get the specific daemon this node is running on
                if let Some(daemon_id) = dataflow.node_to_daemon.get(node_id) {
                    // Get metrics if available
                    let metrics = dataflow.node_metrics.get(node_id).map(|m| {
                        NodeMetricsInfo {
                            pid: m.pid,
                            cpu_usage: m.cpu_usage,
                            // Use 1000 for MB (megabytes) instead of 1024 (mebibytes)
                            memory_mb: m.memory_bytes as f64 / 1000.0 / 1000.0,
                            disk_read_mb_s: m.disk_read_bytes.map(|b| b as f64 / 1000.0 / 1000.0),
                            disk_write_mb_s: m.disk_write_bytes.map(|b| b as f64 / 1000.0 / 1000.0),
                            restart_count: m.restart_count,
                            broken_inputs: m.broken_inputs.clone(),
                            status: m.status.clone(),
                            pending_messages: m.pending_messages,
                        }
                    });

                    node_infos.push(NodeInfo {
                        dataflow_id: dataflow.uuid,
                        dataflow_name: dataflow.name.clone(),
                        node_id: node_id.clone(),
                        daemon_id: daemon_id.clone(),
                        metrics,
                        network: dataflow.network_metrics.clone(),
                    });
                }
            }
        }
        let _ = reply_sender.send(Ok(ControlRequestReply::NodeInfoList(node_infos)));
        Ok(())
    }

    pub(crate) async fn handle_set_param(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        key: String,
        value: serde_json::Value,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let reply: eyre::Result<ControlRequestReply> = async {
                                let target = resolve_param_target(
                                    &self.running_dataflows,
                                    self.store.as_ref(),
                                    &dataflow_id,
                                    &node_id,
                                )?;
                                let bytes = serde_json::to_vec(&value)
                                    .map_err(|e| eyre!("failed to serialize param value: {e}"))?;
                                // Persist first (source of truth), then attempt synchronous
                                // runtime forwarding. If forwarding fails, caller gets Error(...)
                                // but persisted value will be replayed on catch-up/reconnect.
                                self.store.put_node_param(&dataflow_id, &node_id, &key, &bytes)?;

                                if let ParamTarget::Running { daemon_id } = target {
                                    let df = self.running_dataflows.get_mut(&dataflow_id).ok_or_else(
                                        || {
                                            eyre!(
                                                "param persisted in store but running dataflow `{dataflow_id}` disappeared before runtime forwarding for node `{node_id}`"
                                            )
                                        },
                                    )?;
                                    df.append_state_log(StateCatchUpOperation::SetParam {
                                        node_id: node_id.clone(),
                                        key: key.clone(),
                                        value: value.clone(),
                                    });

                                    let msg = serde_json::to_vec(&Timestamped {
                                        inner: DaemonCoordinatorEvent::SetParam {
                                            dataflow_id,
                                            node_id: node_id.clone(),
                                            key: key.clone(),
                                            value: value.clone(),
                                        },
                                        timestamp: self.clock.new_timestamp(),
                                    })
                                    .map_err(|e| {
                                        eyre!("failed to serialize SetParam event for node `{node_id}`: {e}")
                                    })?;

                                    let conn =
                                        self.daemon_connections.get_mut(&daemon_id).ok_or_else(|| {
                                            eyre!(
                                                "param persisted in store but daemon `{daemon_id}` is not connected"
                                            )
                                        })?;
                                    let reply_raw = conn.send_and_receive(&msg).await.map_err(|e| {
                                        eyre!(
                                            "failed to forward SetParam to daemon `{daemon_id}` for node `{node_id}`: {e}"
                                        )
                                    })?;
                                    ensure_set_param_forward_applied(&reply_raw, &node_id)?;
                                }
                                Ok(ControlRequestReply::ParamSet)
                            }
                            .await;
        let _ = reply_sender.send(reply);
        Ok(())
    }

    pub(crate) async fn handle_delete_param(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        key: String,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let reply: eyre::Result<ControlRequestReply> = async {
                                let target = resolve_param_target(
                                    &self.running_dataflows,
                                    self.store.as_ref(),
                                    &dataflow_id,
                                    &node_id,
                                )?;
                                // Persist first (source of truth), then attempt synchronous
                                // runtime forwarding. If forwarding fails, caller gets Error(...)
                                // but delete is still reflected in persisted state/catch-up log.
                                self.store.delete_node_param(&dataflow_id, &node_id, &key)?;

                                if let ParamTarget::Running { daemon_id } = target {
                                    let df = self.running_dataflows.get_mut(&dataflow_id).ok_or_else(
                                        || {
                                            eyre!(
                                                "param deleted in store but running dataflow `{dataflow_id}` disappeared before runtime forwarding for node `{node_id}`"
                                            )
                                        },
                                    )?;
                                    df.append_state_log(StateCatchUpOperation::DeleteParam {
                                        node_id: node_id.clone(),
                                        key: key.clone(),
                                    });

                                    let msg = serde_json::to_vec(&Timestamped {
                                        inner: DaemonCoordinatorEvent::DeleteParam {
                                            dataflow_id,
                                            node_id: node_id.clone(),
                                            key: key.clone(),
                                        },
                                        timestamp: self.clock.new_timestamp(),
                                    })
                                    .map_err(|e| {
                                        eyre!(
                                            "failed to serialize DeleteParam event for node `{node_id}`: {e}"
                                        )
                                    })?;

                                    let conn =
                                        self.daemon_connections.get_mut(&daemon_id).ok_or_else(|| {
                                            eyre!(
                                                "param deleted in store but daemon `{daemon_id}` is not connected"
                                            )
                                        })?;
                                    let reply_raw = conn.send_and_receive(&msg).await.map_err(|e| {
                                        eyre!(
                                            "failed to forward DeleteParam to daemon `{daemon_id}` for node `{node_id}`: {e}"
                                        )
                                    })?;
                                    ensure_delete_param_forward_applied(&reply_raw, &node_id)?;
                                }
                                Ok(ControlRequestReply::ParamDeleted)
                            }
                            .await;
        let _ = reply_sender.send(reply);
        Ok(())
    }

    pub(crate) async fn handle_add_node(
        &mut self,
        dataflow_id: Uuid,
        node: Node,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let result = match self.running_dataflows.get_mut(&dataflow_id) {
            Some(dataflow) => {
                if dataflow.node_to_daemon.contains_key(&node.id) {
                    Err(eyre!(
                        "node '{}' already exists in dataflow {dataflow_id}",
                        node.id
                    ))
                } else {
                    // Keep a clone of the original Node so
                    // we can push it into the stored
                    // descriptor after a successful spawn
                    // (so `dora info` reflects the new
                    // node).
                    let original_node = node.clone();

                    // See `resolve_single_node` for what
                    // the running descriptor contributes
                    // (env carry-through #2919,
                    // single-operator output prefixing
                    // #2877).
                    match resolve_single_node(node, &dataflow.descriptor) {
                        Ok((node_id, resolved_node)) => {
                            // Pick the first daemon (single-daemon case)
                            // TODO: use machine label or load balancing for multi-daemon
                            let daemon_id = dataflow.daemons.iter().next().cloned();
                            match daemon_id {
                                Some(did) => {
                                    let msg = serde_json::to_vec(&Timestamped {
                                        inner: DaemonCoordinatorEvent::AddNode {
                                            dataflow_id,
                                            node: resolved_node.clone(),
                                            uv: dataflow.uv,
                                        },
                                        timestamp: self.clock.new_timestamp(),
                                    })?;
                                    match self.daemon_connections.get_mut(&did) {
                                        Some(conn) => {
                                            match conn.send_and_receive(&msg).await {
                                                Ok(reply_raw) => {
                                                    // Validate the daemon reply is
                                                    // specifically an `AddNodeResult`
                                                    // (not just any non-error reply)
                                                    // before committing state. Without
                                                    // this, a `SetParamResult` or an
                                                    // explicit `AddNodeResult(Err)`
                                                    // would still be reported as
                                                    // applied and corrupt the dataflow
                                                    // state (#1682, rescue of #1757).
                                                    // The validator's error is folded
                                                    // into the `Err` arm of `result`
                                                    // (via explicit `Err(e) => Err(e)`
                                                    // below — no `?`), which the
                                                    // coordinator's main loop sends
                                                    // back to the CLI as
                                                    // `ControlRequestReply::Error`.
                                                    // Addresses phil-opp's review of
                                                    // #1757 (do not tear down the
                                                    // event loop on a recoverable
                                                    // per-request failure).
                                                    match ensure_add_node_applied(
                                                        &reply_raw, &node_id,
                                                    ) {
                                                        Ok(()) => {
                                                            dataflow
                                                                .node_to_daemon
                                                                .insert(node_id.clone(), did);
                                                            // Update the stored descriptor
                                                            // and resolved nodes so
                                                            // `dora info` reflects the
                                                            // new node.
                                                            dataflow
                                                                .descriptor
                                                                .nodes
                                                                .push(original_node);
                                                            dataflow.nodes.insert(
                                                                node_id.clone(),
                                                                resolved_node,
                                                            );
                                                            // Clear any stale Stopped/
                                                            // Finalized state for this
                                                            // node id so the new
                                                            // incarnation's metrics push
                                                            // isn't blocked by the prior
                                                            // stop's `node_stopped_at` /
                                                            // `node_finalized` entries.
                                                            dataflow
                                                                .node_stopped_at
                                                                .remove(&node_id);
                                                            dataflow
                                                                .node_finalized
                                                                .remove(&node_id);
                                                            dataflow.node_metrics.remove(&node_id);
                                                            Ok(ControlRequestReply::NodeAdded {
                                                                dataflow_id,
                                                                node_id,
                                                            })
                                                        }
                                                        Err(e) => Err(e),
                                                    }
                                                }
                                                Err(e) => Err(eyre!("daemon dispatch failed: {e}")),
                                            }
                                        }
                                        None => Err(eyre!("no connection for daemon {did}")),
                                    }
                                }
                                None => {
                                    Err(eyre!("no daemons registered for dataflow {dataflow_id}"))
                                }
                            }
                        }
                        // `resolve_single_node` already
                        // prefixes the resolve context.
                        Err(e) => Err(e),
                    }
                }
            }
            None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
        };
        let _ = reply_sender.send(result);
        Ok(())
    }

    pub(crate) async fn handle_remove_node(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        grace_duration: Option<Duration>,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let result = match self.running_dataflows.get(&dataflow_id) {
            Some(dataflow) => {
                match dataflow.node_to_daemon.get(&node_id) {
                    Some(daemon_id) => {
                        let msg = serde_json::to_vec(&Timestamped {
                            inner: DaemonCoordinatorEvent::RemoveNode {
                                dataflow_id,
                                node_id: node_id.clone(),
                                grace_duration,
                            },
                            timestamp: self.clock.new_timestamp(),
                        })?;
                        match self.daemon_connections.get_mut(daemon_id) {
                            Some(conn) => {
                                match conn.send_and_receive(&msg).await {
                                    Ok(reply_raw) => {
                                        match ensure_remove_node_applied(&reply_raw, &node_id) {
                                            Ok(()) => {
                                                // Clean up coordinator state
                                                // (inverse of AddNode inserts)
                                                if let Some(dataflow) =
                                                    self.running_dataflows.get_mut(&dataflow_id)
                                                {
                                                    dataflow.node_to_daemon.remove(&node_id);
                                                    dataflow
                                                        .descriptor
                                                        .nodes
                                                        .retain(|n| n.id != node_id);
                                                    dataflow.nodes.remove(&node_id);
                                                }
                                                Ok(ControlRequestReply::NodeRemoved {
                                                    dataflow_id,
                                                    node_id,
                                                })
                                            }
                                            Err(e) => Err(e),
                                        }
                                    }
                                    Err(e) => Err(eyre!("daemon dispatch failed: {e}")),
                                }
                            }
                            None => Err(eyre!("no connection for daemon {daemon_id}")),
                        }
                    }
                    None => Err(eyre!(
                        "node '{node_id}' not found in dataflow {dataflow_id}"
                    )),
                }
            }
            None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
        };
        let _ = reply_sender.send(result);
        Ok(())
    }

    pub(crate) async fn handle_replace_node(
        &mut self,
        dataflow_id: Uuid,
        node: Node,
        grace_duration: Option<Duration>,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let result = async {
            // Route to the daemon that OWNS the id — a
            // replace targets an existing node, unlike
            // AddNode's first-daemon placement.
            let original_node = node.clone();
            // Resolve inside the borrow so the running
            // descriptor can be passed by reference; the
            // borrow ends before `daemon_connections` is
            // taken mutably below.
            let (daemon_id, uv, node_id, resolved_node) = {
                let dataflow = self
                    .running_dataflows
                    .get(&dataflow_id)
                    .ok_or_else(|| eyre!("no running dataflow with ID {dataflow_id}"))?;
                let daemon_id =
                    dataflow
                        .node_to_daemon
                        .get(&node.id)
                        .cloned()
                        .ok_or_else(|| {
                            eyre!(
                                "node '{}' not found in dataflow {dataflow_id}; \
                                                 use `dora node add` to add a new node",
                                node.id
                            )
                        })?;
                let (node_id, resolved_node) = resolve_single_node(node, &dataflow.descriptor)?;
                (daemon_id, dataflow.uv, node_id, resolved_node)
            };
            let msg = serde_json::to_vec(&Timestamped {
                inner: DaemonCoordinatorEvent::ReplaceNode {
                    dataflow_id,
                    node: resolved_node.clone(),
                    // Ship the original YAML-shape node so
                    // the daemon can assign the descriptor
                    // entry wholesale, mirroring the
                    // `*existing = original_node` commit
                    // this arm does below.
                    unresolved_node: original_node.clone(),
                    uv,
                    grace_duration,
                },
                timestamp: self.clock.new_timestamp(),
            })?;
            let conn = self
                .daemon_connections
                .get_mut(&daemon_id)
                .ok_or_else(|| eyre!("no connection for daemon {daemon_id}"))?;
            let reply_raw = conn
                .send_and_receive(&msg)
                .await
                .map_err(|e| eyre!("daemon dispatch failed: {e}"))?;
            // Commit coordinator state only after the
            // daemon confirms with the specific reply
            // variant (#1682 contract).
            ensure_replace_node_applied(&reply_raw, &node_id)?;
            if let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) {
                if let Some(existing) = dataflow
                    .descriptor
                    .nodes
                    .iter_mut()
                    .find(|n| n.id == node_id)
                {
                    *existing = original_node;
                } else {
                    dataflow.descriptor.nodes.push(original_node);
                }
                dataflow.nodes.insert(node_id.clone(), resolved_node);
                // Clear stale lifecycle markers so the new
                // incarnation's metrics are not suppressed
                // (same set AddNode clears).
                dataflow.node_stopped_at.remove(&node_id);
                dataflow.node_finalized.remove(&node_id);
                dataflow.node_metrics.remove(&node_id);
            }
            Ok(ControlRequestReply::NodeReplaced {
                dataflow_id,
                node_id,
            })
        }
        .await;
        let _ = reply_sender.send(result);
        Ok(())
    }

    pub(crate) async fn handle_add_mapping(
        &mut self,
        dataflow_id: Uuid,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let result = match self.running_dataflows.get(&dataflow_id) {
            Some(dataflow) => match dataflow.node_to_daemon.get(&target_node) {
                Some(daemon_id) => {
                    let msg = serde_json::to_vec(&Timestamped {
                        inner: DaemonCoordinatorEvent::AddMapping {
                            dataflow_id,
                            source_node: source_node.clone(),
                            source_output: source_output.clone(),
                            target_node: target_node.clone(),
                            target_input: target_input.clone(),
                        },
                        timestamp: self.clock.new_timestamp(),
                    })?;
                    match self.daemon_connections.get_mut(daemon_id) {
                        Some(conn) => match conn.send_and_receive(&msg).await {
                            Ok(reply_raw) => {
                                // Validate the daemon reply is specifically an
                                // `AddMappingResult` before reporting success,
                                // mirroring the #1682 / #1873 rescue for AddNode.
                                let src = format!("{source_node}/{source_output}");
                                let tgt = format!("{target_node}/{target_input}");
                                match ensure_add_mapping_applied(&reply_raw, &src, &tgt) {
                                    Ok(()) => Ok(ControlRequestReply::MappingAdded {
                                        dataflow_id,
                                        source_node,
                                        source_output,
                                        target_node,
                                        target_input,
                                    }),
                                    Err(e) => Err(e),
                                }
                            }
                            Err(e) => Err(eyre!("daemon dispatch failed: {e}")),
                        },
                        None => Err(eyre!("no connection for daemon {daemon_id}")),
                    }
                }
                None => Err(eyre!(
                    "target node '{target_node}' not found in dataflow {dataflow_id}"
                )),
            },
            None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
        };
        let _ = reply_sender.send(result);
        Ok(())
    }

    pub(crate) async fn handle_remove_mapping(
        &mut self,
        dataflow_id: Uuid,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
        reply_sender: ReplySender,
    ) -> eyre::Result<()> {
        let result = match self.running_dataflows.get(&dataflow_id) {
            Some(dataflow) => match dataflow.node_to_daemon.get(&target_node) {
                Some(daemon_id) => {
                    let msg = serde_json::to_vec(&Timestamped {
                        inner: DaemonCoordinatorEvent::RemoveMapping {
                            dataflow_id,
                            source_node: source_node.clone(),
                            source_output: source_output.clone(),
                            target_node: target_node.clone(),
                            target_input: target_input.clone(),
                        },
                        timestamp: self.clock.new_timestamp(),
                    })?;
                    match self.daemon_connections.get_mut(daemon_id) {
                        Some(conn) => match conn.send_and_receive(&msg).await {
                            Ok(reply_raw) => {
                                let src = format!("{source_node}/{source_output}");
                                let tgt = format!("{target_node}/{target_input}");
                                match ensure_remove_mapping_applied(&reply_raw, &src, &tgt) {
                                    Ok(()) => Ok(ControlRequestReply::MappingRemoved {
                                        dataflow_id,
                                        source_node,
                                        source_output,
                                        target_node,
                                        target_input,
                                    }),
                                    Err(e) => Err(e),
                                }
                            }
                            Err(e) => Err(eyre!("daemon dispatch failed: {e}")),
                        },
                        None => Err(eyre!("no connection for daemon {daemon_id}")),
                    }
                }
                None => Err(eyre!(
                    "target node '{target_node}' not found in dataflow {dataflow_id}"
                )),
            },
            None => Err(eyre!("no running dataflow with ID {dataflow_id}")),
        };
        let _ = reply_sender.send(result);
        Ok(())
    }
}
