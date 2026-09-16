//! Events that originate from a daemon connection: registration, heartbeats,
//! exits, metrics and status reports, and the periodic heartbeat tick.

use crate::{
    Coordinator, DaemonRequest, apply_disconnect_actions, build_result_timeout,
    check_build_timeouts, check_spawn_timeouts, cleanup_disconnected_daemons_from_running_builds,
    cleanup_disconnected_daemons_from_running_dataflows, expire_stopped_nodes,
    handle_pruned_state_catchup_fallback, notify_daemons_about_disconnected_peers,
    reestablish_running_dataflow, replay_all_nodes_ready, restore_topic_debug_streams_for_daemon,
    send_heartbeat_with_timeout, state, status_report_should_stop_orphan,
    stop_orphaned_dataflow_on_daemon,
};
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
use dora_message::{
    DataflowId,
    common::DaemonId,
    coordinator_to_daemon::{DaemonCoordinatorEvent, RegisterResult, Timestamped},
    daemon_to_coordinator::{
        DataflowStatusEntry, FaultToleranceSnapshot, NetworkMetrics, NodeMetrics,
    },
    id::NodeId,
};
use eyre::{WrapErr, eyre};
use futures::future::join_all;
use std::{
    collections::{BTreeMap, BTreeSet},
    time::{Duration, Instant},
};
use uuid::Uuid;

impl Coordinator {
    pub(crate) async fn handle_daemon_request(&mut self, event: DaemonRequest) -> eyre::Result<()> {
        match event {
            DaemonRequest::Register {
                machine_id,
                labels,
                connection,
                version_check_result,
                daemon_id_tx,
            } => {
                let existing = match &machine_id {
                    Some(id) => self.daemon_connections.get_matching_daemon_id(id),
                    None => self.daemon_connections.unnamed().next(),
                };
                // Allow re-registration: if a daemon with the same machine_id
                // reconnects (e.g., after coordinator restart), replace the old
                // connection. DaemonConnections::add() handles this.
                if existing.is_some() {
                    tracing::info!(
                        ?machine_id,
                        "daemon re-registering (replacing stale connection)"
                    );
                }
                // Reuse existing DaemonId if daemon is re-registering (#5 fix)
                let daemon_id = match existing {
                    Some(existing_id) => existing_id.clone(),
                    None => DaemonId::new(machine_id),
                };

                let reply: Timestamped<RegisterResult> = Timestamped {
                    inner: match version_check_result.as_ref() {
                        // Gathered here rather than before the match, so a
                        // version-mismatched daemon retrying in a loop does
                        // not make the coordinator walk its whole daemon map
                        // and clone every endpoint per attempt — on the
                        // serial event loop that is time no one gets back.
                        //
                        // Gathered before `add` below, so the joining daemon
                        // is not in the map yet and receives exactly the
                        // peers that preceded it; see
                        // `RegisterResult::Ok::peer_zenoh_endpoints`.
                        Ok(_) => RegisterResult::ok(
                            daemon_id.clone(),
                            self.daemon_connections
                                .zenoh_endpoints_for(&daemon_id, connection.peer_addr),
                        ),
                        Err(err) => RegisterResult::Err(err.clone()),
                    },
                    timestamp: self.clock.new_timestamp(),
                };

                let send_result = connection
                    .send(&serde_json::to_vec(&reply)?)
                    .await
                    .context("failed to send register reply");
                match version_check_result.map_err(|e| eyre!(e)).and(send_result) {
                    Ok(()) => {
                        let _ = daemon_id_tx.send(daemon_id.clone());
                        if let Some(peer_addr) = connection.peer_addr {
                            self.daemon_peer_addrs
                                .write()
                                .unwrap_or_else(|e| e.into_inner())
                                .insert(daemon_id.to_string(), peer_addr);
                        }
                        self.daemon_connections.add(daemon_id.clone(), connection);
                        if let Err(e) =
                            self.store
                                .register_daemon(dora_coordinator_store::DaemonInfo {
                                    daemon_id: daemon_id.clone(),
                                    machine_id: daemon_id.machine_id().map(|s| s.to_owned()),
                                    labels,
                                })
                        {
                            tracing::warn!("failed to persist daemon registration: {e}");
                        }
                    }
                    Err(err) => {
                        tracing::warn!(
                            "failed to register daemon connection for daemon `{daemon_id}`: {err}"
                        );
                    }
                }
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_heartbeat_interval(&mut self) -> eyre::Result<()> {
        // Also drives expired-stopped-node cleanup so dataflows
        // with no live nodes (and therefore no NodeMetrics push)
        // still eventually shed their zombie rows.
        for dataflow in self.running_dataflows.values_mut() {
            expire_stopped_nodes(dataflow);
        }
        let mut disconnected = BTreeSet::new();
        // Send the per-daemon heartbeats concurrently rather than
        // awaiting each 500 ms-bounded send in turn. Each send awaits a
        // bounded WS mpsc, so a single backpressured (slow/half-dead but
        // not yet 30 s-stale) daemon would otherwise stall this control
        // loop for up to 500 ms before the next daemon is even tried —
        // and N such daemons stall it for N × 500 ms, delaying every
        // spawn/stop/logs request. `join_all` bounds the whole tick at
        // ~500 ms regardless of daemon count. Mirrors `destroy_daemons`.
        let mut heartbeats = Vec::new();
        for (machine_id, connection) in self.daemon_connections.iter_mut() {
            let elapsed = connection.last_heartbeat.elapsed();
            if elapsed > Duration::from_secs(15) {
                tracing::warn!("no heartbeat message from machine `{machine_id}` since {elapsed:?}")
            }
            if elapsed > Duration::from_secs(30) {
                disconnected.insert(machine_id.clone());
                continue;
            }
            heartbeats.push(send_heartbeat_with_timeout(
                machine_id.clone(),
                connection,
                self.clock.new_timestamp(),
            ));
        }
        for (machine_id, disconnect) in join_all(heartbeats).await {
            if disconnect {
                disconnected.insert(machine_id);
            }
        }
        if !disconnected.is_empty() {
            tracing::error!("Disconnecting daemons that failed watchdog: {disconnected:?}");
            for machine_id in &disconnected {
                self.daemon_connections.remove(machine_id);
                if let Err(e) = self.store.unregister_daemon(machine_id) {
                    tracing::warn!("failed to persist daemon unregistration: {e}");
                }
            }
            let disconnect_actions = cleanup_disconnected_daemons_from_running_dataflows(
                &mut self.running_dataflows,
                &disconnected,
                &mut self.pending_restarts,
            );
            apply_disconnect_actions(
                disconnect_actions,
                &mut self.running_dataflows,
                &mut self.daemon_connections,
                &self.store,
                &self.clock,
            )
            .await?;
            cleanup_disconnected_daemons_from_running_builds(
                &mut self.running_builds,
                &mut self.finished_builds,
                &disconnected,
            );
            notify_daemons_about_disconnected_peers(
                &disconnected,
                &mut self.daemon_connections,
                &self.clock,
            )
            .await?;
        }
        // Spawn timeout watchdog: detect distributed starts that are
        // stuck waiting for one or more `spawn_result` reports and
        // release `wait_for_spawn` waiters with a clear error rather
        // than letting them hang on the client-side RPC deadline.
        // Triggers on either failure mode:
        //   1. A daemon accepted the spawn RPC but its internal flow
        //      is hung (still heartbeating, never reports back).
        //   2. Pending daemons all disconnected; the disconnect path
        //      above cleared their entries from `pending_spawn_results`
        //      but does not itself fire `spawn_result`.
        // Rescue of #1593 (issue #1592).
        //
        // SAFETY net only: 60 s is well above realistic per-daemon
        // spawn time even with `--uv` Python env preparation.
        check_spawn_timeouts(
            &mut self.running_dataflows,
            &mut self.archived_dataflows,
            &mut self.dataflow_results,
            &mut self.daemon_connections,
            &mut self.pending_restarts,
            &self.clock,
            self.store.as_ref(),
        )
        .await;

        // Build timeout watchdog — mirror of `check_spawn_timeouts`
        // for `running_builds`. Releases `wait_for_build` waiters
        // that would otherwise hang on the client-side RPC deadline
        // when a daemon participating in `dora build` disconnects
        // or otherwise never reports its `build_result`. #1465.
        check_build_timeouts(
            &mut self.running_builds,
            &mut self.finished_builds,
            &self.clock,
            build_result_timeout(),
        )
        .await;

        // Recovery timeout: transition stale Recovering dataflows to Failed.
        // Dataflows are marked Recovering on coordinator startup and should
        // be reclaimed by reconnecting daemons within 60 seconds.
        const RECOVERY_TIMEOUT_SECS: u64 = 60;
        let now_ms = state::now_millis();
        match self.store.list_dataflows() {
            Ok(records) => {
                for mut record in records {
                    if record.status == StoreDataflowStatus::Recovering {
                        let age_ms = now_ms.saturating_sub(record.updated_at);
                        if age_ms > RECOVERY_TIMEOUT_SECS * 1000 {
                            tracing::warn!(
                                uuid = %record.uuid,
                                age_secs = age_ms / 1000,
                                "recovery timeout: Recovering -> Failed"
                            );
                            record.status = StoreDataflowStatus::Failed {
                                error: format!(
                                    "recovery timeout ({RECOVERY_TIMEOUT_SECS}s): \
                                             no daemon reconnected"
                                ),
                                // Coordinator gave up waiting for daemon reconnect;
                                // mark terminal so a daemon that eventually returns
                                // doesn't promote this back to Running via reconcile.
                                terminal: true,
                            };
                            record.generation += 1;
                            record.updated_at = now_ms;
                            if let Err(e) = self.store.put_dataflow(&record) {
                                tracing::warn!("failed to mark dataflow as Failed: {e}");
                            }
                        }
                    }
                }
            }
            Err(e) => tracing::warn!("failed to list dataflows for recovery check: {e}"),
        }
        Ok(())
    }

    pub(crate) async fn handle_daemon_heartbeat(
        &mut self,
        machine_id: DaemonId,
        ft_stats: Option<FaultToleranceSnapshot>,
    ) -> eyre::Result<()> {
        if let Some(connection) = self.daemon_connections.get_mut(&machine_id) {
            connection.last_heartbeat = Instant::now();
            if let Some(stats) = ft_stats {
                connection.ft_stats = Some(stats);
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_daemon_zenoh_endpoint(
        &mut self,
        daemon_id: DaemonId,
        connection_id: Uuid,
        endpoint: Option<String>,
    ) -> eyre::Result<()> {
        // Same guard as `DaemonExit` below (#2392): a report still in
        // flight from a connection that has since been replaced would
        // otherwise overwrite the live endpoint with a dead one, and
        // every daemon registering afterwards would be handed it.
        if self.daemon_connections.connection_id_of(&daemon_id) == Some(connection_id) {
            match &endpoint {
                Some(ep) => {
                    tracing::debug!("daemon `{daemon_id}` confirmed zenoh endpoint `{ep}`")
                }
                // The daemon advertised an endpoint when it registered
                // and then failed to bind it. Withdrawing stops the
                // coordinator handing a dead endpoint to every daemon
                // that registers from here on.
                None => tracing::warn!(
                    "daemon `{daemon_id}` withdrew its zenoh endpoint: its listener \
                             did not bind, so other daemons cannot reach it directly"
                ),
            }
            self.daemon_connections
                .set_zenoh_endpoint(&daemon_id, endpoint);
        } else {
            tracing::debug!(
                "ignoring zenoh endpoint {endpoint:?} from a superseded \
                         connection of daemon `{daemon_id}`"
            );
        }
        Ok(())
    }

    pub(crate) async fn handle_daemon_exit(
        &mut self,
        daemon_id: DaemonId,
        connection_id: Uuid,
    ) -> eyre::Result<()> {
        // Only act on the exit if the connection_id matches the
        // currently-registered connection. A named daemon that
        // reconnects before the old connection's handler task
        // delivers its DaemonExit must NOT evict the new connection
        // or trigger spurious dataflow cleanup (#2392).
        if self.daemon_connections.connection_id_of(&daemon_id) == Some(connection_id) {
            tracing::info!("Daemon `{daemon_id}` exited");
            self.daemon_connections.remove(&daemon_id);
            if let Err(e) = self.store.unregister_daemon(&daemon_id) {
                tracing::warn!("failed to persist daemon unregistration: {e}");
            }
            let disconnected = BTreeSet::from([daemon_id]);
            let disconnect_actions = cleanup_disconnected_daemons_from_running_dataflows(
                &mut self.running_dataflows,
                &disconnected,
                &mut self.pending_restarts,
            );
            apply_disconnect_actions(
                disconnect_actions,
                &mut self.running_dataflows,
                &mut self.daemon_connections,
                &self.store,
                &self.clock,
            )
            .await?;
            // Mirror the watchdog disconnect path: fail any in-flight
            // build the exited daemon was still part of. Without this, a
            // multi-daemon `dora build` where one daemon exits cleanly
            // mid-build never sees its pending set resolve (the exited
            // daemon's entry lingers), so the build is not finalized by
            // the `DataflowBuildResult` handler and instead hangs until
            // `check_build_timeouts` fires the 20-minute deadline. #1465.
            cleanup_disconnected_daemons_from_running_builds(
                &mut self.running_builds,
                &mut self.finished_builds,
                &disconnected,
            );
            notify_daemons_about_disconnected_peers(
                &disconnected,
                &mut self.daemon_connections,
                &self.clock,
            )
            .await?;
        } else {
            tracing::debug!(
                "ignoring stale DaemonExit for `{daemon_id}` \
                         (connection replaced by reconnect)"
            );
        }
        Ok(())
    }

    pub(crate) async fn handle_node_metrics(
        &mut self,
        dataflow_id: Uuid,
        metrics: BTreeMap<NodeId, NodeMetrics>,
        network: Option<NetworkMetrics>,
    ) -> eyre::Result<()> {
        // Store metrics for this dataflow
        if let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) {
            // Sweep expired stopped-node entries before applying
            // fresh metrics, so `dora node list` eventually stops
            // showing rows for nodes the daemon last reported as
            // stopped > NODE_STOPPED_GRACE ago.
            expire_stopped_nodes(dataflow);
            for (node_id, node_metrics) in &metrics {
                // A NodeStopped event is authoritative: skip any
                // in-flight metrics row for the same node that was
                // captured by the daemon's pre-stop snapshot. The
                // check uses `node_finalized` (covers Stopped AND
                // Failed) rather than `node_stopped_at` (Stopped
                // only) so a delayed metrics push cannot revive a
                // crashed-Failed row back to a stale Running.
                if dataflow.node_finalized.contains(node_id) {
                    continue;
                }
                dataflow
                    .node_metrics
                    .insert(node_id.clone(), node_metrics.clone());
            }
            if let Some(net) = network {
                dataflow.network_metrics = Some(net);
            }

            #[cfg(feature = "metrics")]
            {
                use crate::otel_metrics::node_attrs;
                use opentelemetry::KeyValue;
                let df_id = dataflow_id.to_string();
                for (node_id, node_metrics) in &metrics {
                    // Same authority as the in-memory table above: a
                    // finalized node's delayed metrics push is a stale
                    // pre-exit snapshot. Skip it here too, so OTEL does
                    // not record a fresh data point that resurrects the
                    // node's CPU/memory time series one push past its
                    // death.
                    if dataflow.node_finalized.contains(node_id) {
                        continue;
                    }
                    let daemon = dataflow
                        .node_to_daemon
                        .get(node_id)
                        .map(|d| d.to_string())
                        .unwrap_or_default();
                    let attrs = node_attrs(df_id.clone(), node_id.to_string(), daemon);
                    self.otel_metrics
                        .node_cpu
                        .record(node_metrics.cpu_usage as f64, &attrs);
                    self.otel_metrics
                        .node_memory
                        .record(node_metrics.memory_bytes as i64, &attrs);
                    self.otel_metrics
                        .node_pending
                        .record(node_metrics.pending_messages as i64, &attrs);
                    self.otel_metrics
                        .node_restarts
                        .record(node_metrics.restart_count as i64, &attrs);
                }
                self.otel_metrics.dataflow_nodes.record(
                    dataflow.nodes.len() as i64,
                    &[
                        KeyValue::new("dataflow", df_id),
                        KeyValue::new("name", dataflow.name.clone().unwrap_or_default()),
                    ],
                );
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_daemon_status_report(
        &mut self,
        daemon_id: DaemonId,
        reported_dataflows: Vec<DataflowStatusEntry>,
    ) -> eyre::Result<()> {
        tracing::info!(
            "daemon {daemon_id} reports {} running dataflow(s)",
            reported_dataflows.len()
        );
        // Reconcile: if daemon reports a dataflow as running and it exists in
        // the store as Pending/Failed/Recovering, update it to Running.
        //
        // Exception: dataflows that are present in `archived_dataflows`
        // have been declared terminally failed by the spawn-timeout
        // watchdog (or any other archive-on-failure path). Promoting
        // their store status back to Running would contradict the
        // terminal verdict the user already received via
        // `wait_for_spawn`. Round-7 Finding 1.
        for entry in &reported_dataflows {
            let df_id = &entry.dataflow_id;
            if self.archived_dataflows.contains_key(df_id) {
                tracing::warn!(
                    "daemon {daemon_id} reports archived (terminally-failed) \
                             dataflow {df_id} as running; skipping reconcile so the \
                             watchdog's Failed verdict is preserved"
                );
                // The daemon kept these nodes alive but the coordinator
                // has terminally failed the dataflow; stop the orphans
                // rather than leave them unmanageable (#2029 P3).
                stop_orphaned_dataflow_on_daemon(
                    *df_id,
                    &daemon_id,
                    &mut self.daemon_connections,
                    &self.clock,
                )
                .await;
                continue;
            }
            match self.store.get_dataflow(df_id) {
                Ok(Some(mut record)) => match record.status {
                    // Failed records: only promote to Running if NOT
                    // terminal. The `terminal: true` marker (set by
                    // the spawn-timeout watchdog and the recovery
                    // timeout) survives coordinator restarts in the
                    // store, so a wedged daemon that reconnects
                    // post-restart cannot resurrect a terminally-
                    // failed dataflow (round-8 Finding 1).
                    // Non-terminal Failed records preserve the
                    // pre-#1854 behaviour where a daemon's report
                    // could override a coordinator-side Failed
                    // (e.g. the multi-daemon partial-failure case
                    // where another daemon is still running).
                    StoreDataflowStatus::Failed {
                        terminal: false, ..
                    }
                    | StoreDataflowStatus::Pending
                    | StoreDataflowStatus::Recovering => {
                        tracing::info!(
                            "reconciling dataflow {df_id}: {:?} -> Running \
                                     (daemon reports {} active node(s))",
                            record.status,
                            entry.running_nodes.len(),
                        );
                        record.status = StoreDataflowStatus::Running;
                        record.generation += 1;
                        record.updated_at = state::now_millis();
                        if let Err(e) = self.store.put_dataflow(&record) {
                            tracing::warn!("failed to reconcile dataflow {df_id}: {e}");
                        }
                        // Rebuild the live in-memory entry so the
                        // surviving nodes are visible + manageable again
                        // (#2029 P1) — store status alone doesn't drive
                        // `dora list` / `stop` / `logs`.
                        if reestablish_running_dataflow(
                            &mut self.running_dataflows,
                            &record,
                            &daemon_id,
                            &entry.running_nodes,
                        ) && let Some(df) = self.running_dataflows.get(&record.uuid)
                        {
                            // Barrier released while this daemon was
                            // gone; nothing else will tell it (#2998).
                            // Best-effort, like every other step in
                            // this reconciliation: a failed send here
                            // means one daemon's nodes stay parked,
                            // which must not take down the coordinator
                            // and every other dataflow with it.
                            if let Err(e) = replay_all_nodes_ready(
                                record.uuid,
                                df,
                                &daemon_id,
                                &mut self.daemon_connections,
                                &self.store,
                                &self.clock,
                            )
                            .await
                            {
                                tracing::warn!(
                                    "failed to replay ready barrier to daemon \
                                             `{daemon_id}` for dataflow {}: {e:#}",
                                    record.uuid
                                );
                            }
                        }
                    }
                    StoreDataflowStatus::Running => {
                        // Already `Running` in the store but possibly
                        // missing from the live map (e.g. a later report,
                        // or a coordinator restart that loaded the record
                        // but not the in-memory entry). Idempotent.
                        if reestablish_running_dataflow(
                            &mut self.running_dataflows,
                            &record,
                            &daemon_id,
                            &entry.running_nodes,
                        ) && let Some(df) = self.running_dataflows.get(&record.uuid)
                        {
                            // Barrier released while this daemon was
                            // gone; nothing else will tell it (#2998).
                            // Best-effort, like every other step in
                            // this reconciliation: a failed send here
                            // means one daemon's nodes stay parked,
                            // which must not take down the coordinator
                            // and every other dataflow with it.
                            if let Err(e) = replay_all_nodes_ready(
                                record.uuid,
                                df,
                                &daemon_id,
                                &mut self.daemon_connections,
                                &self.store,
                                &self.clock,
                            )
                            .await
                            {
                                tracing::warn!(
                                    "failed to replay ready barrier to daemon \
                                             `{daemon_id}` for dataflow {}: {e:#}",
                                    record.uuid
                                );
                            }
                        }
                    }
                    status if status_report_should_stop_orphan(&status) => {
                        // Terminal state (success, watchdog failure, or
                        // equivalent coordinator-side verdict). Daemon's
                        // report is ignored to preserve the verdict the
                        // user already received.
                        tracing::warn!(
                            "daemon {daemon_id} reports terminal \
                                     dataflow {df_id} as running; skipping reconcile",
                        );
                        // Stop the orphaned nodes the daemon kept alive
                        // past the coordinator's terminal verdict, so
                        // they don't run unmanageable forever (#2029 P3).
                        stop_orphaned_dataflow_on_daemon(
                            *df_id,
                            &daemon_id,
                            &mut self.daemon_connections,
                            &self.clock,
                        )
                        .await;
                    }
                    _ => {}
                },
                Ok(None) => {
                    tracing::warn!(
                        "daemon reports dataflow {df_id} running, but not found in store"
                    );
                }
                Err(e) => {
                    tracing::warn!("failed to look up dataflow {df_id} for reconciliation: {e}");
                }
            }
        }

        // Auto-recovery: find dataflows that should be running on this daemon
        // but aren't reported. Uses 30s backoff per daemon per dataflow to
        // avoid infinite re-spawn loops for crash-looping nodes.
        const RECOVERY_BACKOFF: Duration = Duration::from_secs(30);
        let reported_set: BTreeSet<DataflowId> =
            reported_dataflows.iter().map(|e| e.dataflow_id).collect();
        restore_topic_debug_streams_for_daemon(
            &self.running_dataflows,
            &mut self.daemon_connections,
            &daemon_id,
            &reported_set,
            &self.clock,
        )
        .await;
        let now = Instant::now();
        for (uuid, df) in &mut self.running_dataflows {
            if !df.daemons.contains(&daemon_id) {
                continue;
            }
            if reported_set.contains(uuid) {
                // Dataflow is running — clear any previous recovery timestamp
                df.last_recovery_attempt.remove(&daemon_id);
                continue;
            }
            // Backoff: skip if we attempted recovery recently
            if let Some(last) = df.last_recovery_attempt.get(&daemon_id)
                && now.duration_since(*last) < RECOVERY_BACKOFF
            {
                continue;
            }
            // Skip if a spawn is already in-flight for this daemon
            if df.pending_spawn_results.contains(&daemon_id) {
                continue;
            }
            // Skip dataflows that the spawn-timeout watchdog (or
            // any other terminal-failure path) has already marked
            // as failed. Without this, a daemon whose
            // `DaemonStatusReport` lacks a watchdog-failed
            // dataflow would have the dataflow re-spawned here,
            // resurrecting a terminally-failed dataflow in memory
            // even though `spawn_result` is `Cached(Err)` and the
            // store says Failed. See PR #1854 round-4 Finding 1.
            if df.spawn_result.is_terminal_error() {
                continue;
            }
            // Collect nodes assigned to this daemon
            let spawn_nodes: BTreeSet<_> = df
                .node_to_daemon
                .iter()
                .filter(|(_, did)| **did == daemon_id)
                .map(|(nid, _)| nid.clone())
                .collect();
            if spawn_nodes.is_empty() {
                continue;
            }
            df.last_recovery_attempt.insert(daemon_id.clone(), now);
            tracing::info!(
                "auto-recovery: re-spawning {} node(s) for dataflow {uuid} on daemon {daemon_id}",
                spawn_nodes.len()
            );
            let spawn_command = dora_message::coordinator_to_daemon::SpawnDataflowNodes {
                build_id: None,
                session_id: dora_message::SessionId::generate(),
                dataflow_id: *uuid,
                local_working_dir: None,
                nodes: df.nodes.clone(),
                dataflow_descriptor: df.descriptor.clone(),
                spawn_nodes,
                uv: df.uv,
                write_events_to: None,
                artifact_base_url: None,
            };
            let message = match serde_json::to_vec(&Timestamped {
                inner: DaemonCoordinatorEvent::Spawn(spawn_command),
                timestamp: self.clock.new_timestamp(),
            }) {
                Ok(m) => m,
                Err(e) => {
                    tracing::warn!("failed to serialize re-spawn command: {e}");
                    continue;
                }
            };
            if let Some(conn) = self.daemon_connections.get_mut(&daemon_id)
                && let Err(e) = conn.send(&message).await
            {
                tracing::warn!("failed to send re-spawn to daemon {daemon_id}: {e}");
            }
        }

        // State catch-up: for dataflows the daemon reports as running,
        // send any state mutations it missed while disconnected.
        for (uuid, df) in &mut self.running_dataflows {
            if !df.daemons.contains(&daemon_id) || !reported_set.contains(uuid) {
                continue;
            }
            let last_ack = df.daemon_ack_sequence.get(&daemon_id).copied().unwrap_or(0);
            if last_ack >= df.state_log_sequence {
                continue; // already up to date
            }
            match df.state_log_delta(last_ack) {
                Some(entries) if entries.is_empty() => {}
                Some(entries) => {
                    tracing::info!(
                        "state catch-up: sending {} entry(ies) for dataflow {uuid} \
                                 to daemon {daemon_id} (seq {last_ack}..{})",
                        entries.len(),
                        df.state_log_sequence,
                    );
                    let event = DaemonCoordinatorEvent::StateCatchUp {
                        dataflow_id: *uuid,
                        entries,
                    };
                    if let Ok(msg) = serde_json::to_vec(&Timestamped {
                        inner: event,
                        timestamp: self.clock.new_timestamp(),
                    }) && let Some(conn) = self.daemon_connections.get_mut(&daemon_id)
                        && let Err(e) = conn.send(&msg).await
                    {
                        tracing::warn!("failed to send state catch-up to daemon {daemon_id}: {e}");
                    }
                }
                None => {
                    // Log was pruned past this daemon's ack — fall back to full
                    // param replay from the store.
                    tracing::info!(
                        "state catch-up: log pruned for dataflow {uuid}, \
                                 falling back to full param replay for daemon {daemon_id}"
                    );
                    handle_pruned_state_catchup_fallback(
                        *uuid,
                        df,
                        &daemon_id,
                        self.store.clone(),
                        &mut self.daemon_connections,
                        self.clock.clone(),
                        now,
                    )
                    .await;
                }
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_state_catch_up_ack(
        &mut self,
        daemon_id: DaemonId,
        dataflow_id: DataflowId,
        ack_sequence: u64,
    ) -> eyre::Result<()> {
        if let Some(df) = self.running_dataflows.get_mut(&dataflow_id) {
            if !df.daemons.contains(&daemon_id) {
                tracing::warn!(
                    "ignoring StateCatchUpAck from daemon {daemon_id} \
                             not in dataflow {dataflow_id}"
                );
            } else {
                // Clamp: monotonically increasing, bounded by log sequence.
                let current = df.daemon_ack_sequence.get(&daemon_id).copied().unwrap_or(0);
                let clamped = ack_sequence.min(df.state_log_sequence).max(current);
                df.daemon_ack_sequence.insert(daemon_id, clamped);
                df.prune_state_log();
            }
        }
        Ok(())
    }

    pub(crate) async fn handle_daemon_node_stopped(
        &mut self,
        daemon_id: DaemonId,
        dataflow_id: Uuid,
        node_id: NodeId,
        clean_stop: bool,
    ) -> eyre::Result<()> {
        // Daemon reports a node has stopped and will not be
        // restarted. Mark the cached metrics so `dora node list`
        // shows `Stopped` (clean exit) or `Failed` (final-failure
        // exit) instead of the frozen-Running snapshot, and arm
        // the expiry side-band so the row eventually disappears
        // (see `expire_stopped_nodes`).
        //
        // Ownership check: the daemon that owns this node per
        // `node_to_daemon` is the only one allowed to declare it
        // stopped. Drops stale events from a previous incarnation
        // (e.g. stop → remove → re-add on a different daemon) and
        // foreign-daemon spoofing.
        if let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) {
            match dataflow.node_to_daemon.get(&node_id) {
                Some(owner) if owner == &daemon_id => {}
                Some(other) => {
                    tracing::warn!(
                        %dataflow_id, %node_id,
                        "ignoring NodeStopped from daemon `{daemon_id}`: node \
                         is owned by `{other}`"
                    );
                    return Ok(());
                }
                None => {
                    tracing::debug!(
                        %dataflow_id, %node_id,
                        "ignoring NodeStopped: node no longer in dataflow"
                    );
                    return Ok(());
                }
            }
            let status = if clean_stop {
                dora_message::daemon_to_coordinator::NodeStatus::Stopped
            } else {
                // Crash / restart-policy exhaustion: surface as
                // `Failed` so `dora doctor` still counts it. A
                // clean `Stopped` would be invisible to the
                // doctor's healthy/degraded/failed bucketing.
                dora_message::daemon_to_coordinator::NodeStatus::Failed
            };
            let entry = dataflow
                .node_metrics
                .entry(node_id.clone())
                .or_insert_with(|| dora_message::daemon_to_coordinator::NodeMetrics {
                    pid: 0,
                    cpu_usage: 0.0,
                    memory_bytes: 0,
                    disk_read_bytes: None,
                    disk_write_bytes: None,
                    restart_count: 0,
                    broken_inputs: Vec::new(),
                    status: status.clone(),
                    pending_messages: 0,
                });
            entry.status = status;
            entry.pid = 0;
            entry.cpu_usage = 0.0;
            entry.memory_bytes = 0;
            entry.disk_read_bytes = None;
            entry.disk_write_bytes = None;
            // Authoritative finalize marker: protects against late
            // in-flight metrics pushes overwriting the row, for
            // BOTH Stopped and Failed.
            dataflow.node_finalized.insert(node_id.clone());
            // Only arm the auto-expire side-band for clean Stopped
            // rows. Failed rows must stay visible until the dataflow
            // is stopped/destroyed — otherwise a crashed node would
            // disappear from `dora node list` / `dora doctor` after
            // the 60s grace, hiding the failure this PR is meant
            // to surface.
            if clean_stop {
                dataflow.node_stopped_at.insert(node_id, Instant::now());
            } else {
                // Defensive: if the same node id was previously
                // marked Stopped (then re-spawned and now Failed),
                // clear the stale stopped_at so the Failed row
                // isn't swept on the next tick.
                dataflow.node_stopped_at.remove(&node_id);
            }
        }
        Ok(())
    }
}
