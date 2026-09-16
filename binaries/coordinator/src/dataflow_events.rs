//! Events about a dataflow's progress: readiness and completion reports,
//! build and spawn results, log and topic-debug frames.

use crate::{
    Coordinator, DataflowEvent, MAX_ARCHIVED_DATAFLOWS, MAX_BUFFERED_LOG_MESSAGES,
    broadcast_all_nodes_ready, buffer_log_message, cap_dataflow_results,
    close_topic_subscribers_on_finish, finalize_build, handle_dataflow_spawn_result,
    handlers::{dataflow_result, send_log_message, send_topic_frames, start_dataflow},
    state::{ArchivedDataflow, CachedResult},
};
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
use dora_message::{
    BuildId,
    common::DaemonId,
    coordinator_to_cli::{ControlRequestReply, DataflowResult, LogLevel, LogMessage},
    daemon_to_coordinator::DataflowDaemonResult,
};
use eyre::eyre;
use std::collections::BTreeMap;
use uuid::Uuid;

impl Coordinator {
    pub(crate) async fn handle_dataflow_event(
        &mut self,
        uuid: Uuid,
        event: DataflowEvent,
    ) -> eyre::Result<()> {
        match event {
            DataflowEvent::ReadyOnDaemon {
                daemon_id,
                exited_before_subscribe,
            } => match self.running_dataflows.entry(uuid) {
                std::collections::hash_map::Entry::Occupied(mut entry) => {
                    let dataflow = entry.get_mut();
                    dataflow.pending_daemons.remove(&daemon_id);
                    dataflow
                        .exited_before_subscribe
                        .extend(exited_before_subscribe);
                    if dataflow.pending_daemons.is_empty() {
                        broadcast_all_nodes_ready(
                            uuid,
                            dataflow,
                            &mut self.daemon_connections,
                            &self.store,
                            &self.clock,
                        )
                        .await?;
                    }
                }
                std::collections::hash_map::Entry::Vacant(_) => {
                    tracing::warn!("dataflow not running on ReadyOnMachine");
                }
            },
            DataflowEvent::DataflowFinishedOnDaemon { daemon_id, result } => {
                tracing::debug!(
                    "coordinator received DataflowFinishedOnDaemon ({daemon_id:?}, result: {result:?})"
                );
                match self.running_dataflows.entry(uuid) {
                    std::collections::hash_map::Entry::Occupied(mut entry) => {
                        let dataflow = entry.get_mut();
                        dataflow.daemons.remove(&daemon_id);
                        tracing::info!(
                            "removed machine id: {daemon_id} from dataflow: {:#?}",
                            dataflow.uuid
                        );
                        self.dataflow_results
                            .entry(uuid)
                            .or_default()
                            .insert(daemon_id, result);

                        if dataflow.daemons.is_empty() {
                            // Archive finished dataflow (cap at 200 to prevent unbounded growth)
                            self.archived_dataflows
                                .entry(uuid)
                                .or_insert_with(|| ArchivedDataflow::from(entry.get()));
                            while self.archived_dataflows.len() > MAX_ARCHIVED_DATAFLOWS {
                                self.archived_dataflows.shift_remove_index(0);
                            }
                            let mut finished_dataflow = entry.remove();

                            // Complete any pending restart on this dataflow.
                            // The restart was deferred in `initiate_restart` so that
                            // old nodes' Zenoh subscribers/declarations are fully
                            // torn down before new nodes spawn, avoiding the race where
                            // new nodes start before old nodes exit (dora-rs/dora#2082).
                            //
                            // This runs *before* close_topic_subscribers_on_finish
                            // intentionally: the restart block only touches the
                            // coordinator's `self.running_dataflows` map and `self.pending_restarts`
                            // map, it does not interact with `finished_dataflow` or its
                            // Zenoh-side state (which has already been cleaned up by the
                            // daemons — that's why we're in this handler).
                            if let Some(restart) = self.pending_restarts.remove(&uuid) {
                                let name = restart.name.clone();
                                match start_dataflow(
                                    None,
                                    dora_message::SessionId::generate(),
                                    restart.descriptor,
                                    None,
                                    restart.name,
                                    &mut self.daemon_connections,
                                    &self.clock,
                                    restart.uv,
                                    None,
                                )
                                .await
                                {
                                    Ok(new_dataflow) => {
                                        let new_uuid = new_dataflow.uuid;
                                        // Persist new dataflow as Pending
                                        let mut new_df = new_dataflow;
                                        if let Err(e) = new_df
                                            .make_record(StoreDataflowStatus::Pending)
                                            .and_then(|r| self.store.put_dataflow(&r))
                                        {
                                            tracing::warn!(
                                                "failed to persist restarted dataflow: {e}"
                                            );
                                        }
                                        self.running_dataflows.insert(new_uuid, new_df);
                                        let _ = restart.reply_sender.send(Ok(
                                            ControlRequestReply::DataflowRestarted {
                                                old_uuid: uuid,
                                                new_uuid,
                                            },
                                        ));
                                    }
                                    Err(err) => {
                                        tracing::error!(
                                            "failed to start dataflow during deferred restart of `{name:?}` ({uuid}): {err:#}"
                                        );
                                        let _ = restart.reply_sender.send(Err(eyre!(
                                            "failed to start restarted dataflow: {err:#}"
                                        )));
                                    }
                                }
                            }

                            close_topic_subscribers_on_finish(&mut finished_dataflow);
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
                                    timestamp: self
                                        .clock
                                        .new_timestamp()
                                        .get_time()
                                        .to_system_time()
                                        .into(),
                                    fields: None,
                                },
                            )
                            .await;

                            let reply = ControlRequestReply::DataflowStopped {
                                uuid,
                                result: self
                                    .dataflow_results
                                    .get(&uuid)
                                    .map(|r| dataflow_result(r, uuid, &self.clock))
                                    .unwrap_or_else(|| {
                                        DataflowResult::ok_empty(uuid, self.clock.new_timestamp())
                                    }),
                            };
                            // Persist: dataflow finished
                            let final_status =
                                if let Some(results) = self.dataflow_results.get(&uuid) {
                                    let errors: Vec<String> = results
                                        .values()
                                        .flat_map(|dr| dr.node_results.iter())
                                        .filter_map(|(node_id, r)| {
                                            r.as_ref().err().map(|e| format!("{node_id}: {e}"))
                                        })
                                        .collect();
                                    if errors.is_empty() {
                                        StoreDataflowStatus::Succeeded
                                    } else {
                                        StoreDataflowStatus::Failed {
                                            error: errors.join("; "),
                                            // Normal end-of-life failure: not flagged terminal
                                            // because there is no concurrent path that could
                                            // resurrect a properly-finished dataflow.
                                            terminal: false,
                                        }
                                    }
                                } else {
                                    StoreDataflowStatus::Succeeded
                                };
                            if let Err(e) = finished_dataflow
                                .make_record(final_status)
                                .and_then(|r| self.store.put_dataflow(&r))
                            {
                                tracing::warn!("failed to persist dataflow finish: {e}");
                            }

                            for sender in finished_dataflow.stop_reply_senders {
                                let _ = sender.send(Ok(reply.clone()));
                            }
                            // If WaitForSpawn waiters are still pending, notify them
                            // that the dataflow finished before spawn completed (e.g.,
                            // a node crashed at startup or the build failed).
                            if !matches!(
                                finished_dataflow.spawn_result,
                                CachedResult::Cached { .. }
                            ) {
                                let node_errors: Vec<String> = self
                                    .dataflow_results
                                    .get(&uuid)
                                    .into_iter()
                                    .flat_map(|r| r.values())
                                    .flat_map(|dr| dr.node_results.iter())
                                    .filter_map(|(node_id, r)| {
                                        r.as_ref().err().map(|e| format!("{node_id}: {e}"))
                                    })
                                    .collect();
                                let msg = if node_errors.is_empty() {
                                    "dataflow exited before spawn completed".to_string()
                                } else {
                                    format!(
                                        "dataflow failed to start:\n  {}",
                                        node_errors.join("\n  ")
                                    )
                                };
                                finished_dataflow.spawn_result.set_result(Err(eyre!(msg)));
                            }
                        }
                    }
                    std::collections::hash_map::Entry::Vacant(_) => {
                        // If the dataflow was previously archived by the
                        // spawn-timeout watchdog (round-6 Finding 2), merge
                        // the daemon's late-arriving completion result into
                        // the synthetic `self.dataflow_results` entry so the
                        // per-node details are surfaced via `dora list` /
                        // `dora check` instead of being silently dropped.
                        // Per-node `Err(FailedToSpawn(..))` entries
                        // (synthesized at watchdog time) are overwritten
                        // by the daemon's real per-node results where they
                        // overlap, giving the user the best available
                        // post-mortem info.
                        if self.archived_dataflows.contains_key(&uuid) {
                            let entry = self.dataflow_results.entry(uuid).or_default();
                            let existing = entry.entry(daemon_id.clone()).or_insert_with(|| {
                                DataflowDaemonResult {
                                    timestamp: result.timestamp,
                                    node_results: BTreeMap::new(),
                                }
                            });
                            existing.timestamp = result.timestamp;
                            existing.node_results.extend(result.node_results);
                        } else {
                            tracing::warn!("dataflow not running on DataflowFinishedOnDaemon",);
                        }
                    }
                }
                // Bound finished-history growth (active multi-daemon entries
                // are preserved). Done after the match so `self.running_dataflows`
                // is no longer borrowed by the `entry(uuid)` scrutinee.
                cap_dataflow_results(&mut self.dataflow_results, &self.running_dataflows);
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_log(&mut self, message: LogMessage) -> eyre::Result<()> {
        // `dataflow_id`/`build_id` are `Copy`, so match them by value to
        // leave `message` free to move into `buffer_log_message`.
        if let Some(dataflow_id) = message.dataflow_id {
            if let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) {
                if dataflow.log_subscribers.is_empty() {
                    if buffer_log_message(&mut dataflow.buffered_log_messages, message) {
                        tracing::warn!(
                            "log buffer full for dataflow {dataflow_id} \
                                     ({MAX_BUFFERED_LOG_MESSAGES} messages); dropping further \
                                     messages until a log subscriber attaches"
                        );
                    }
                } else {
                    send_log_message(&mut dataflow.log_subscribers, &message).await;
                }
            }
        } else if let Some(build_id) = message.build_id
            && let Some(build) = self.running_builds.get_mut(&build_id)
        {
            if build.log_subscribers.is_empty() {
                if buffer_log_message(&mut build.buffered_log_messages, message) {
                    tracing::warn!(
                        "log buffer full for build {build_id} \
                                 ({MAX_BUFFERED_LOG_MESSAGES} messages); dropping further \
                                 messages until a log subscriber attaches"
                    );
                }
            } else {
                send_log_message(&mut build.log_subscribers, &message).await;
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_topic_debug_data(
        &mut self,
        dataflow_id: Uuid,
        subscription_ids: Vec<Uuid>,
        payload: Vec<u8>,
    ) -> eyre::Result<()> {
        tracing::trace!(
            %dataflow_id,
            subscriptions = subscription_ids.len(),
            bytes = payload.len(),
            "received topic debug frame from daemon"
        );
        if let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) {
            send_topic_frames(&mut dataflow.topic_subscribers, subscription_ids, payload).await;
        }
        Ok(())
    }

    pub(crate) async fn handle_build_result(
        &mut self,
        build_id: BuildId,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    ) -> eyre::Result<()> {
        match self.running_builds.get_mut(&build_id) {
            Some(build) => {
                build.pending_build_results.remove(&daemon_id);
                match result {
                    Ok(()) => {}
                    Err(err) => {
                        tracing::error!("build error for {build_id}: {err:?}");
                        build.errors.push(format!("{err}"));
                    }
                };
                if build.pending_build_results.is_empty() {
                    tracing::info!("dataflow build finished: `{build_id}`");
                    let Some(build) = self.running_builds.remove(&build_id) else {
                        tracing::error!("build {build_id} disappeared from self.running_builds");
                        return Ok(());
                    };
                    finalize_build(build_id, build, &mut self.finished_builds);
                }
            }
            None => {
                // Build no longer in `self.running_builds` — usually means
                // the watchdog (`check_build_timeouts`) already marked
                // it as terminally failed and moved it to
                // `self.finished_builds`. Late replies are expected in that
                // case; warn but do not resurrect (the cached failure
                // result is the authoritative one). #1465.
                tracing::warn!(
                    build_id = %build_id,
                    daemon_id = %daemon_id,
                    "received DataflowBuildResult for a build no longer in `self.running_builds` (already finalized or timed out — ignoring)"
                );
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_spawn_result(
        &mut self,
        dataflow_id: Uuid,
        daemon_id: DaemonId,
        result: eyre::Result<()>,
    ) -> eyre::Result<()> {
        handle_dataflow_spawn_result(
            dataflow_id,
            daemon_id,
            result,
            &mut self.running_dataflows,
            &mut self.archived_dataflows,
            &mut self.dataflow_results,
            &mut self.daemon_connections,
            &mut self.pending_restarts,
            &self.clock,
            self.store.as_ref(),
        )
        .await;
        Ok(())
    }
}
