//! Events about a dataflow's progress: readiness and completion reports,
//! build and spawn results, log and topic-debug frames.

use crate::{
    Coordinator, DataflowEvent, MAX_ARCHIVED_DATAFLOWS, MAX_BUFFERED_LOG_MESSAGES,
    broadcast_all_nodes_ready, buffer_log_message, cap_dataflow_results,
    close_topic_subscribers_on_finish, finalize_build, handle_dataflow_spawn_result,
    handlers::{dataflow_result, send_log_message, start_dataflow},
    state::{ArchivedDataflow, CachedResult, now_millis},
};
use dora_coordinator_store::{DataflowRecord, DataflowStatus as StoreDataflowStatus};
use dora_core::descriptor::DescriptorExt;
use dora_message::{
    BuildId,
    common::DaemonId,
    coordinator_to_cli::{ControlRequestReply, DataflowResult, LogLevel, LogMessage},
    daemon_to_coordinator::DataflowDaemonResult,
    descriptor::{CoreNodeKind, DYNAMIC_SOURCE, Descriptor, NodeSource, ResolvedNode},
    id::NodeId,
};
use eyre::eyre;
use std::collections::{BTreeMap, BTreeSet};
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
                            // coordinator's `running_dataflows` map and `pending_restarts`
                            // map, it does not interact with `finished_dataflow` or its
                            // Zenoh-side state (which has already been cleaned up by the
                            // daemons — that's why we're in this handler).
                            if let Some(restart) = self.pending_restarts.remove(&uuid) {
                                let name = restart.name.clone();
                                match start_dataflow(
                                    restart.descriptor,
                                    restart.launch,
                                    restart.name,
                                    &mut self.daemon_connections,
                                    &self.clock,
                                    restart.uv,
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
                            let final_status = dataflow_store_status(
                                self.dataflow_results
                                    .get(&uuid)
                                    .into_iter()
                                    .flat_map(|results| results.values()),
                            );
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
                        // the synthetic `dataflow_results` entry so the
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
                            match self.store.get_dataflow(&uuid) {
                                // A finish report the daemon queued on a connection
                                // that died is resent when it reconnects (#3612).
                                // When the coordinator processed the disconnect
                                // first, `cleanup_disconnected_daemons_from_running_dataflows`
                                // took the dataflow out of `running_dataflows` and
                                // `begin_orphaned_dataflow_reclaim` left it
                                // `Recovering`; the resend lands here.
                                Ok(Some(record))
                                    if record.status == StoreDataflowStatus::Recovering =>
                                {
                                    self.record_resent_finish_report(record, daemon_id, result);
                                }
                                Ok(_) => {
                                    tracing::warn!(
                                        "dataflow not running on DataflowFinishedOnDaemon",
                                    );
                                }
                                Err(e) => {
                                    tracing::warn!(
                                        "failed to look up dataflow {uuid} on \
                                         DataflowFinishedOnDaemon: {e}"
                                    );
                                }
                            }
                        }
                    }
                }
                // Bound finished-history growth (active multi-daemon entries
                // are preserved). Done after the match so `running_dataflows`
                // is no longer borrowed by the `entry(uuid)` scrutinee.
                cap_dataflow_results(&mut self.dataflow_results, &self.running_dataflows);
            }
        }
        Ok(())
    }

    /// Records a finish report a daemon resent after reconnecting for a
    /// dataflow the orphan reclaim left `Recovering` in the store
    /// (dora-rs/dora#3631), and finalizes it once the reports cover the whole
    /// dataflow.
    ///
    /// The report only covers the daemon that sent it. Until the reported
    /// node results cover every node the persisted record assigns — counting a
    /// `path: dynamic` node only once its own daemon has reported, since no
    /// daemon ever reports a result for one — the dataflow may still be running
    /// on a daemon that hasn't reported yet, so the record is left `Recovering`
    /// for that daemon's `DaemonStatusReport` to re-establish. The reports are
    /// parked in [`Self::resent_finish_reports`] until then: `dataflow_results`
    /// is what the rest of the coordinator reads as a finished dataflow, and a
    /// partial report there would make `dora list` / `stop` / `clean` act on a
    /// dataflow that is still recovering. Once the reports cover the record,
    /// nothing is left running and the status is settled from the results, as
    /// the normal finish path does. A duplicate report cannot settle it twice:
    /// the first one moves the record out of `Recovering`, and the lookup in
    /// `handle_dataflow_event` then drops the rest.
    fn record_resent_finish_report(
        &mut self,
        mut record: DataflowRecord,
        daemon_id: DaemonId,
        result: DataflowDaemonResult,
    ) {
        let uuid = record.uuid;
        self.resent_finish_reports
            .entry(uuid)
            .or_default()
            .insert(daemon_id, result);

        // The persisted descriptor is the record's authoritative node set.
        // `node_to_daemon` cannot decide coverage on its own: after an earlier
        // recovery it holds only the reconnecting daemon's share
        // (`RunningDataflow::recovered`), so a partial report would look like
        // it covered the whole dataflow and settle one that another daemon is
        // still running. It is still the only per-node daemon assignment the
        // record has, so it is what tells a `path: dynamic` node's owner apart.
        let Some(nodes) = resolve_record_nodes(&record) else {
            tracing::warn!(
                "cannot check coverage of a resent finish report for recovered dataflow \
                 {uuid}: the persisted descriptor did not resolve; leaving it Recovering"
            );
            return;
        };
        // Which daemons have already reported. The record's `node_to_daemon`
        // holds the ids from before the reclaim, and a reconnecting daemon
        // registers under a fresh `DaemonId`, so a node's daemon is matched by
        // machine id, the part that survives the reconnect.
        let reported_daemons: BTreeSet<&DaemonId> = self
            .dataflow_results
            .get(&uuid)
            .into_iter()
            .chain(self.resent_finish_reports.get(&uuid))
            .flat_map(|results| results.keys())
            .collect();
        // A daemon never reports a result for a `path: dynamic` node — they
        // send no `SpawnedNodeResult` — so requiring one would keep the
        // dataflow `Recovering` until the recovery timeout. A dynamic node is
        // therefore skipped, but only once the daemon it belongs to has
        // reported: a daemon whose whole share is dynamic nodes never finishes
        // on its own (`should_finish` in `binaries/daemon/src/node_events.rs`
        // is only evaluated when a node stops, and it then still has its
        // dynamic node running), so dropping its node early would settle a
        // dataflow that daemon is still running. An unknown owner (missing from
        // `node_to_daemon`) counts too, erring towards leaving it `Recovering`.
        let expected: BTreeSet<String> = nodes
            .values()
            .filter(|node| {
                if !is_dynamic_node(node) {
                    return true;
                }
                !record
                    .node_to_daemon
                    .get(&node.id.to_string())
                    .and_then(|daemon| DaemonId::from_display_str(daemon))
                    .is_some_and(|assigned| {
                        reported_daemons
                            .iter()
                            .any(|reported| reported.machine_id() == assigned.machine_id())
                    })
            })
            .map(|node| node.id.to_string())
            .collect();
        let reported: BTreeSet<String> = self
            .dataflow_results
            .get(&uuid)
            .into_iter()
            .chain(self.resent_finish_reports.get(&uuid))
            .flat_map(|results| results.values())
            .flat_map(|result| result.node_results.keys())
            .map(|node_id| node_id.to_string())
            .collect();
        if !expected.is_subset(&reported) {
            tracing::debug!(
                "resent finish report for recovered dataflow {uuid} covers only part of \
                 it; leaving it Recovering for the daemons that haven't reported"
            );
            return;
        }

        let status = dataflow_store_status(
            self.dataflow_results
                .get(&uuid)
                .into_iter()
                .chain(self.resent_finish_reports.get(&uuid))
                .flat_map(|results| results.values()),
        );
        record.status = status;
        record.generation += 1;
        record.updated_at = now_millis();
        if let Err(e) = self.store.put_dataflow(&record) {
            tracing::warn!("failed to persist finish of recovered dataflow {uuid}: {e}");
            return;
        }

        // Coverage is complete, so these are the dataflow's final results.
        // Move them where the rest of the coordinator reads finished
        // dataflows, and archive it like the normal finish path so `dora list`
        // keeps its name and `logs` / `stop` can still resolve it by name.
        self.merge_resent_finish_reports(uuid);
        self.archived_dataflows
            .entry(uuid)
            .or_insert_with(|| ArchivedDataflow {
                name: record.name.clone(),
                nodes,
            });
        while self.archived_dataflows.len() > MAX_ARCHIVED_DATAFLOWS {
            self.archived_dataflows.shift_remove_index(0);
        }
        tracing::info!("finalized recovered dataflow {uuid} from a resent finish report");
    }

    /// Move the finish reports parked for `uuid` into `dataflow_results`,
    /// merging per-node results for a daemon that already has an entry (it
    /// finished normally before the reclaim).
    ///
    /// Called when the parked reports cover the record
    /// ([`Self::record_resent_finish_report`]) and when another daemon
    /// re-establishes the dataflow as `Running`: the parked reports are the
    /// results of daemons that finished before the reclaim, so the eventual
    /// finish must settle with them rather than lose them.
    pub(crate) fn merge_resent_finish_reports(&mut self, uuid: Uuid) {
        let Some(reports) = self.resent_finish_reports.remove(&uuid) else {
            return;
        };
        let results = self.dataflow_results.entry(uuid).or_default();
        for (daemon_id, result) in reports {
            merge_daemon_result(results, daemon_id, result);
        }
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
        crate::topic_debug::forward_topic_frames(
            &mut self.running_dataflows,
            &mut self.daemon_connections,
            dataflow_id,
            subscription_ids,
            payload,
            &self.clock,
        )
        .await;
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
                        tracing::error!("build {build_id} disappeared from running_builds");
                        return Ok(());
                    };
                    finalize_build(build_id, build, &mut self.finished_builds);
                }
            }
            None => {
                // Build no longer in `running_builds` — usually means
                // the watchdog (`check_build_timeouts`) already marked
                // it as terminally failed and moved it to
                // `finished_builds`. Late replies are expected in that
                // case; warn but do not resurrect (the cached failure
                // result is the authoritative one). #1465.
                tracing::warn!(
                    build_id = %build_id,
                    daemon_id = %daemon_id,
                    "received DataflowBuildResult for a build no longer in `running_builds` (already finalized or timed out — ignoring)"
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

/// The persisted status for a dataflow whose daemons have reported: `Succeeded`
/// when every node result is `Ok`, otherwise a non-terminal `Failed` naming the
/// failing nodes. No results at all is treated as success — a dataflow with
/// only dynamic nodes reports none.
fn dataflow_store_status<'a>(
    results: impl Iterator<Item = &'a DataflowDaemonResult>,
) -> StoreDataflowStatus {
    let errors: Vec<String> = results
        .flat_map(|result| result.node_results.iter())
        .filter_map(|(node_id, r)| r.as_ref().err().map(|e| format!("{node_id}: {e}")))
        .collect();
    if errors.is_empty() {
        StoreDataflowStatus::Succeeded
    } else {
        // Normal end-of-life failure: not flagged terminal because there is no
        // concurrent path that could resurrect a properly-finished dataflow.
        StoreDataflowStatus::Failed {
            error: errors.join("; "),
            terminal: false,
        }
    }
}

/// Resolve the node set a persisted record expects, from its
/// `descriptor_json` — the same resolution `reestablish_running_dataflow`
/// does. `None` when the descriptor cannot be parsed or resolved.
fn resolve_record_nodes(record: &DataflowRecord) -> Option<BTreeMap<NodeId, ResolvedNode>> {
    let descriptor: Descriptor = match serde_json::from_str(&record.descriptor_json) {
        Ok(descriptor) => descriptor,
        Err(e) => {
            tracing::warn!(
                "failed to parse persisted descriptor for dataflow {}: {e}",
                record.uuid
            );
            return None;
        }
    };
    match descriptor.resolve_aliases_and_set_defaults() {
        Ok(nodes) => Some(nodes),
        Err(e) => {
            tracing::warn!(
                "failed to resolve persisted descriptor for dataflow {}: {e}",
                record.uuid
            );
            None
        }
    }
}

/// Whether a node is `path: dynamic`, mirroring the daemon's own predicate
/// (`binaries/daemon/src/lib.rs`): a local custom node whose path is
/// [`DYNAMIC_SOURCE`]. A daemon never reports a node result for one.
fn is_dynamic_node(node: &ResolvedNode) -> bool {
    matches!(
        &node.kind,
        CoreNodeKind::Custom(custom)
            if matches!(&custom.source, NodeSource::Local) && custom.path == DYNAMIC_SOURCE
    )
}

/// Merge `result` for `daemon_id` into `results`, extending the per-node
/// results of an entry that is already there rather than replacing it.
fn merge_daemon_result(
    results: &mut BTreeMap<DaemonId, DataflowDaemonResult>,
    daemon_id: DaemonId,
    result: DataflowDaemonResult,
) {
    match results.entry(daemon_id) {
        std::collections::btree_map::Entry::Occupied(mut entry) => {
            let existing = entry.get_mut();
            existing.timestamp = result.timestamp;
            existing.node_results.extend(result.node_results);
        }
        std::collections::btree_map::Entry::Vacant(entry) => {
            entry.insert(result);
        }
    }
}
