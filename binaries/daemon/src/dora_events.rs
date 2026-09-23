//! Handling of daemon-internal `DoraEvent`s (timers, spawned-node results,
//! log fan-out) and the periodic watchdogs that decide when a node is stuck.

use crate::{
    Daemon, DoraEvent, ProcessOperation, break_input,
    extract_err_from_stderr::extract_err_from_stderr, extract_node_results, finish_drain_grace,
    health_check_should_kill, is_sigkill_like_exit, is_sigterm_like_exit, node_communication,
    reclaim_extensions_of_exited_node, running_dataflow, send_with_timestamp,
    spawn_stack_sample_capture, startup_timeout_should_kill,
};
use dora_core::config::NodeId;
use dora_message::{
    common::{DataMessage, LogLevel, NodeError, NodeErrorCause, NodeExitStatus},
    daemon_to_coordinator::DataflowDaemonResult,
    daemon_to_node::NodeEvent,
    metadata::{self, MetadataParameters},
    node_to_daemon::Timestamped,
};
use std::{
    collections::BTreeSet,
    sync::{Arc, atomic},
};
use uuid::Uuid;

impl Daemon {
    /// Watchdog for nodes that block an otherwise-finished dataflow
    /// (dora-rs/dora#2152).
    ///
    /// The natural-finish path sends `AllInputsClosed` and waits for nodes
    /// to exit voluntarily; unlike explicit stops it had no deadline, so a
    /// stuck node hung `dora run` (and CI) until an external timeout. Once
    /// every running node of a dataflow has been draining for longer than
    /// the grace period, escalate through the same Stop → SIGTERM → SIGKILL
    /// ladder used by explicit stops — after capturing a stack sample of
    /// the stuck process so the hang itself stays diagnosable.
    pub(crate) fn check_finish_stragglers(&mut self) {
        // On by default; only DORA_FINISH_DRAIN_GRACE_SECS=off/disabled turns
        // escalation off entirely (see `finish_drain_grace`).
        let Some(grace) = finish_drain_grace() else {
            return;
        };
        let now_millis = node_communication::current_millis();
        for (dataflow_id, dataflow) in self.running.iter_mut() {
            for node_id in dataflow.finish_stragglers(grace, now_millis) {
                let drained_for_secs = dataflow
                    .all_inputs_closed_at
                    .get(&node_id)
                    .map(|since| since.elapsed().as_secs())
                    .unwrap_or_default();
                let pid = dataflow
                    .running_nodes
                    .get(&node_id)
                    .and_then(|node| node.pid.as_ref())
                    .map(|pid| pid.load(atomic::Ordering::Relaxed));
                // Escalate first: if the node exited naturally between
                // selection and now, this fails benignly and nothing
                // should be logged or sampled.
                if dataflow
                    .stop_single_node(&node_id, &self.clock, None)
                    .is_err()
                {
                    tracing::debug!(
                        "finish straggler `{node_id}` exited before escalation; skipping"
                    );
                    continue;
                }
                dataflow.finish_escalated.insert(node_id.clone());
                tracing::warn!(
                    "dataflow {dataflow_id} is finished except node `{node_id}` \
                     (AllInputsClosed sent {drained_for_secs}s ago) — escalating stop \
                     (set DORA_FINISH_DRAIN_GRACE_SECS to adjust the grace period)"
                );
                spawn_stack_sample_capture(node_id, pid);
            }
        }
    }

    pub(crate) fn check_node_health(&self) {
        let now_millis = node_communication::current_millis();
        for dataflow in self.running.values() {
            for (node_id, node) in &dataflow.running_nodes {
                let Some(process) = &node.process else {
                    continue;
                };
                let connected = dataflow.connected_nodes.contains(node_id);
                if !connected {
                    if let Some(startup_timeout) = node.startup_timeout {
                        let spawned_at = node.spawned_at.load(atomic::Ordering::Acquire);
                        if startup_timeout_should_kill(
                            connected,
                            spawned_at,
                            now_millis,
                            startup_timeout,
                        ) && !node.startup_kill_sent.swap(true, atomic::Ordering::AcqRel)
                        {
                            let elapsed_ms = now_millis.saturating_sub(spawned_at);
                            tracing::warn!(
                                "node `{node_id}` failed to connect within {startup_timeout:?} (elapsed: {}ms), killing",
                                elapsed_ms,
                            );
                            self.ft_stats
                                .startup_timeout_kills
                                .fetch_add(1, atomic::Ordering::Relaxed);
                            dataflow
                                .startup_timeout_kills
                                .insert((node_id.clone(), node.generation));
                            process.submit(ProcessOperation::Kill);
                        }
                    }
                    continue;
                }
                // The health-check watchdog only monitors *post-connection*
                // liveness (#2937). Nodes that have not yet connected are
                // bounded by `startup_timeout` above.
                let Some(timeout) = node.health_check_timeout else {
                    continue;
                };
                let last = node.last_activity.load(atomic::Ordering::Acquire);
                if !health_check_should_kill(connected, last, now_millis, timeout) {
                    continue;
                }
                let elapsed_ms = now_millis.saturating_sub(last);
                tracing::warn!(
                    "node `{node_id}` unresponsive for {}ms (timeout: {timeout:?}), killing",
                    elapsed_ms,
                );
                self.ft_stats
                    .health_check_kills
                    .fetch_add(1, atomic::Ordering::Relaxed);
                process.submit(ProcessOperation::Kill);
            }
        }
    }

    pub(crate) fn check_input_timeouts(&mut self) {
        let clock = self.clock.clone();
        for dataflow in self.running.values_mut() {
            let mut timed_out = Vec::new();
            for ((node_id, input_id), deadline) in &dataflow.input_deadlines {
                // Skip inputs already tracked as broken (avoids duplicate warnings)
                if dataflow
                    .broken_inputs
                    .contains_key(&(node_id.clone(), input_id.clone()))
                {
                    continue;
                }
                // Only count elapsed time once the input has actually
                // received a message. Inputs that never saw traffic are
                // considered "not yet armed" — see InputDeadline::is_timed_out
                // (dora-rs/adora#149).
                if deadline.is_timed_out() {
                    timed_out.push((node_id.clone(), input_id.clone(), deadline.timeout));
                }
            }
            for (node_id, input_id, timeout) in &timed_out {
                tracing::warn!(
                    "input `{node_id}/{input_id}` timed out after {timeout:?}, \
                     closing (circuit breaker armed)",
                );
                self.ft_stats
                    .input_timeouts
                    .fetch_add(1, atomic::Ordering::Relaxed);
                dataflow
                    .broken_inputs
                    .insert((node_id.clone(), input_id.clone()), *timeout);
                break_input(dataflow, node_id, input_id, &clock);
            }
            for (node_id, input_id, _) in timed_out {
                dataflow.input_deadlines.remove(&(node_id, input_id));
            }
        }
    }

    /// Mark a dataflow as finished and perform cleanup.
    /// This should be called when:
    /// - `stop_all()` returns `FinishDataflowWhen::Now`, or
    /// - All non-dynamic nodes have sent `SpawnedNodeResult` events
    pub(crate) async fn finish_dataflow(&mut self, dataflow_id: Uuid) -> eyre::Result<()> {
        let mut logger = self.logger.for_dataflow(dataflow_id);

        // Dynamic nodes don't send SpawnedNodeResult events, so there may be no entry
        // in dataflow_node_results. An empty map means all dynamic nodes handled stop successfully.
        let result = DataflowDaemonResult {
            timestamp: self.clock.new_timestamp(),
            // On a persistent, coordinator-managed daemon (`exit_when_done` is
            // `None`) nothing reads this dataflow's entry again after we report
            // it, so remove it to avoid `dataflow_node_results` growing without
            // bound across the daemon's lifetime. In the run-once (`dora run`)
            // path `run_inner` drains the whole map via `std::mem::take` *after*
            // `finish_dataflow` returns, so the entry must survive until then.
            node_results: extract_node_results(
                &mut self.dataflow_node_results,
                dataflow_id,
                self.exit_when_done.is_some(),
            ),
        };

        self.git_manager
            .clones_in_use
            .values_mut()
            .for_each(|dataflows| {
                dataflows.remove(&dataflow_id);
            });

        // Whatever survived node-exit reclamation goes now: the dataflow's
        // channels and listeners are gone, so nothing can reach these entries
        // and no later event would reclaim them.
        let dropped = self.extensions.reclaim_dataflow(&dataflow_id.to_string());
        if dropped > 0 {
            tracing::debug!(%dataflow_id, dropped, "released extension entries of finished dataflow");
        }

        logger
            .log(
                LogLevel::Info,
                None,
                Some("daemon".into()),
                format!("dataflow finished on machine `{}`", self.daemon_id),
            )
            .await;

        // Signal all listener loops for this dataflow to shut down
        if let Some(df) = self.running.get(&dataflow_id) {
            let _ = df.listener_shutdown_tx.send(true);
        }
        if let Some(exchange) = self
            .running
            .remove(&dataflow_id)
            .and_then(|df| df.endpoint_exchange)
        {
            exchange.linger();
        }

        // The memory-pool subscriber task has no shutdown branch of its
        // own — terminate it, releasing its session clone and event
        // sender. Without this, repeated or failed spawns accumulate
        // tasks and can create duplicate consumers.
        #[cfg(feature = "tensor-pool")]
        self.pool_cleanup_dataflow(dataflow_id).await;

        if let Some(sender) = &self.coordinator_sender
            && let Err(err) = self
                .send_all_nodes_finished(sender, dataflow_id, &result)
                .await
        {
            self.pending_finished_dataflows.insert(dataflow_id, result);
            return Err(err);
        }

        Ok(())
    }

    pub(crate) async fn handle_dora_event(&mut self, event: DoraEvent) -> eyre::Result<()> {
        match event {
            DoraEvent::Timer {
                dataflow_id,
                interval,
                metadata,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    tracing::warn!("Timer event for unknown dataflow `{dataflow_id}`");
                    return Ok(());
                };

                let Some(subscribers) = dataflow.timers.get(&interval) else {
                    return Ok(());
                };

                let metadata = Arc::new(metadata);
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
                        Ok(true) => {
                            dataflow.inc_pending(receiver_id);
                        }
                        Ok(false) => { /* event dropped (channel full) */ }
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
                    return Ok(());
                };

                let Some(subscribers) = dataflow.mappings.get(&output_id) else {
                    // A node with `send_stdout_as`/`send_stderr_as` emits a
                    // `Logs` event for every log line regardless of whether
                    // anyone subscribes to the resulting output, so an output
                    // with no consumer is a normal configuration — not a
                    // warning. Return silently (previously this logged a WARN
                    // per log line and `Debug`-formatted the entire edge map
                    // on that hot path).
                    return Ok(());
                };

                let metadata = Arc::new(metadata);
                let message = Arc::new(message);
                let mut closed = Vec::new();
                for (receiver_id, input_id) in subscribers {
                    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
                        // A subscriber is mapped to this output but has no live
                        // channel (e.g. not yet connected, or already gone).
                        // Diagnosable at debug level without the per-log-line
                        // WARN spam the previous `warn!` here produced.
                        tracing::debug!(
                            "no subscriber channel for `{receiver_id}` on {output_id:?}; \
                             dropping log line"
                        );
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
                        Ok(true) => {
                            dataflow.inc_pending(receiver_id);
                        }
                        Ok(false) => { /* event dropped (channel full) */ }
                        Err(_) => {
                            closed.push(receiver_id);
                        }
                    }
                }
                for id in closed {
                    dataflow.subscribe_channels.remove(id);
                }
            }
            DoraEvent::LogBroadcast {
                dataflow_id,
                log_message,
            } => {
                let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
                    return Ok(());
                };

                if dataflow.log_subscribers.is_empty() {
                    return Ok(());
                }

                // Serialize to JSON once (shared across all subscribers)
                let json = match serde_json::to_string(&log_message) {
                    Ok(j) => j,
                    Err(e) => {
                        tracing::warn!("failed to serialize LogMessage: {e}");
                        return Ok(());
                    }
                };

                // Convert to a self-describing Arrow IPC stream once, share the
                // sample across subscribers. The node-side receive path decodes
                // the IPC stream, so set the `arrow-ipc` framing parameter.
                use dora_arrow_convert::IntoArrow;
                let array = json.as_str().into_arrow();
                let ipc_bytes = match dora_node_api::arrow_utils::encode_arrow_ipc(&array) {
                    Ok(bytes) => bytes,
                    Err(e) => {
                        tracing::warn!("failed to Arrow-IPC-encode LogMessage: {e}");
                        return Ok(());
                    }
                };
                let sample: aligned_vec::AVec<u8, aligned_vec::ConstAlign<128>> =
                    aligned_vec::AVec::from_slice(128, &ipc_bytes);
                let data = Arc::new(DataMessage::Vec(sample));

                // Build the metadata once and share the `Arc` across subscribers.
                // It is identical for every delivery (only the `arrow-ipc`
                // framing parameter), so rebuilding it (and re-allocating the
                // `Arc`) per subscriber was wasted work — mirror the Timer/Logs
                // broadcast loops, which build the `Arc<Metadata>` once.
                let mut params = MetadataParameters::new();
                params.insert(
                    dora_message::metadata::FRAMING.to_string(),
                    dora_message::metadata::Parameter::String(
                        dora_message::metadata::FRAMING_ARROW_IPC.to_string(),
                    ),
                );
                let metadata = Arc::new(metadata::Metadata::from_parameters(
                    self.clock.new_timestamp(),
                    params,
                ));

                let mut closed = Vec::new();
                for sub in &dataflow.log_subscribers {
                    // Apply level filter
                    if let Some(min_level) = &sub.filter.min_level
                        && !log_message.level.passes(min_level)
                    {
                        continue;
                    }
                    // Apply node filter
                    if let Some(node_filter) = &sub.filter.node_filter
                        && log_message.node_id.as_ref() != Some(node_filter)
                    {
                        continue;
                    }
                    // Don't deliver logs to the subscriber itself (avoid loops)
                    if log_message.node_id.as_ref() == Some(&sub.node_id) {
                        continue;
                    }

                    let Some(channel) = dataflow.subscribe_channels.get(&sub.node_id) else {
                        continue;
                    };

                    let send_result = send_with_timestamp(
                        channel,
                        NodeEvent::Input {
                            id: sub.input_id.clone(),
                            metadata: metadata.clone(),
                            data: Some(data.clone()),
                        },
                        &self.clock,
                    );
                    match send_result {
                        Ok(true) => {
                            dataflow.inc_pending(&sub.node_id);
                        }
                        Ok(false) => { /* event dropped (channel full) */ }
                        Err(_) => {
                            closed.push(sub.node_id.clone());
                        }
                    }
                }
                for id in &closed {
                    dataflow.subscribe_channels.remove(id);
                }
                // Prune stale log subscribers whose channels were just removed
                dataflow
                    .log_subscribers
                    .retain(|sub| !closed.contains(&sub.node_id));
            }
            DoraEvent::SpawnedNodeResult {
                dataflow_id,
                node_id,
                generation,
                dynamic_node,
                exit_status,
                restart,
                restart_count,
                pid,
            } => {
                let mut logger = self
                    .logger
                    .for_dataflow(dataflow_id)
                    .for_node(node_id.clone());
                logger
                    .log(
                        LogLevel::Debug,
                        Some("daemon".into()),
                        format!("handling node stop with exit status {exit_status:?} (restart: {restart}, restart_count: {restart_count})"),
                    )
                    .await;

                let current_generation = self
                    .running
                    .get(&dataflow_id)
                    .and_then(|dataflow| dataflow.running_nodes.get(&node_id));
                // Nothing owns the id: `RemoveNode` took the entry out, or the
                // dataflow is gone. A mismatching generation with an entry
                // still present means a replacement or a re-add owns it now.
                let node_id_unowned = current_generation.is_none();
                // Deliberate semantics of the entry-absent case: an exit
                // arriving after `RemoveNode` took the entry out is dropped
                // here WITHOUT running the finish accounting below. That
                // means removing the last non-dynamic node leaves the
                // dataflow alive (zero running nodes) until an explicit stop
                // or a re-add — live-editing keep-alive, chosen over the
                // pre-generation behavior where such a stale exit could both
                // record a bogus result (dora-rs/dora#2926) and finish a
                // dataflow out from under a concurrent re-add of the same id.
                if !current_generation.is_some_and(|node| node.matches_generation(generation)) {
                    logger
                        .log(
                            LogLevel::Debug,
                            Some("daemon".into()),
                            format!(
                                "ignoring stale exit from pid {pid} (generation {generation}); \
                                 `{node_id}` has been removed or replaced"
                            ),
                        )
                        .await;
                    // A node removed while alive keeps its daemon connection
                    // for the whole stop grace window — the node-event guard
                    // deliberately lets its events through once the entry is
                    // gone — so it can still store extension entries after
                    // `RemoveNode`'s eager reclaim. This event is that process
                    // actually dying: the last chance to release them, and
                    // without it they would live until dataflow finish, which
                    // is the accumulation dora-rs/dora#3177 set out to stop.
                    // Only when nothing owns the id: ownership is keyed by node
                    // id alone (see `ExtensionTable`), so reclaiming for a
                    // replacement or a re-add would take the live incarnation's
                    // entries with it. Idempotent — an already-reclaimed exit
                    // finds nothing.
                    if node_id_unowned {
                        self.reclaim_extensions(dataflow_id, &node_id);
                    }
                    if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                        dataflow
                            .grace_duration_kills
                            .remove(&(node_id.clone(), generation));
                        dataflow
                            .startup_timeout_kills
                            .remove(&(node_id.clone(), generation));
                    }
                    return Ok(());
                }

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
                            .map(|d| {
                                d.grace_duration_kills
                                    .contains(&(node_id.clone(), generation))
                            })
                            .unwrap_or_default();
                        // Killed by the finish-straggler watchdog
                        // (dora-rs/dora#2152): the node blocked an
                        // otherwise-finished dataflow past the drain grace
                        // period. Unlike an operator-initiated stop this
                        // must NOT classify as clean — a node that needed
                        // force-killing during natural finish is a shutdown
                        // bug, and reporting success would hide every
                        // recurrence behind a green run. (A straggler that
                        // exits 0 on the escalation's Stop event takes the
                        // `Success` arm above and stays clean: it finished
                        // its work and responded to stop, just late.)
                        let finish_escalated = dataflow
                            .map(|d| d.finish_escalated.contains(&node_id))
                            .unwrap_or_default();
                        let startup_timed_out = dataflow
                            .map(|d| {
                                d.startup_timeout_kills
                                    .contains(&(node_id.clone(), generation))
                            })
                            .unwrap_or_default();
                        // The daemon explicitly sent SoftKill (SIGTERM)
                        // to this node as part of an operator-initiated
                        // stop (`dora stop`, `dora destroy`, `dora run
                        // --stop-after`, `dora node stop`, `dora node
                        // restart`) and the node responded by exiting.
                        // On Unix the exit reports as `Signal(15)`.
                        // Wrappers like `uv run python` catch SIGTERM
                        // and exit with code 143 (= 128 + 15) instead
                        // of propagating the signal, so `child.wait()`
                        // returns `ExitCode(143)` not `Signal(15)`.
                        // Same shape for SIGINT (2 / 130). On Windows the
                        // daemon's SoftKill sends `CTRL_BREAK_EVENT`, so a
                        // node without its own console handler reports
                        // `STATUS_CONTROL_C_EXIT` (`ExitCode(-1073741510)`)
                        // — the Windows analog (dora-rs/dora#2425). Treat any
                        // of those as a clean planned stop so `dora run
                        // --stop-after` doesn't report a fake "Node
                        // failed: exited with code 143" when the
                        // dataflow shut down exactly as requested
                        // (dora-rs/dora#1882). See `is_sigterm_like_exit`.
                        //
                        // `grace_duration_kill` is the right
                        // discriminant — not `restarts_disabled` —
                        // because `disable_restart()` fires at subscribe
                        // time for source nodes (see lib.rs:3203 where
                        // `open_inputs().is_empty()` triggers it).
                        // Using `restarts_disabled` would silently
                        // swallow externally-sent SIGTERMs on source
                        // nodes (e.g. `kill -TERM <pid>`) as clean.
                        // `grace_duration_kills` is only populated by
                        // the daemon's own SoftKill/Kill submission
                        // path (`running_dataflow.rs::stop_all` and
                        // `::send_stop_and_schedule_kill`), so it
                        // accurately encodes "daemon asked this node to
                        // stop."
                        //
                        // SIGKILL exits (Signal(9), happens when the
                        // node didn't respond to SoftKill within the
                        // secondary grace) fall through to the existing
                        // `GraceDuration` branch — that's the original
                        // semantic of GraceDuration and we want to
                        // preserve it.
                        //
                        // Cascading failures still win — if some other
                        // node failed first and this one was killed as
                        // collateral, we want to surface the original
                        // failure rather than hide it behind the
                        // shutdown that followed.
                        let is_sigterm_like = is_sigterm_like_exit(&exit_status);
                        if caused_by_node.is_none()
                            && grace_duration_kill
                            && is_sigterm_like
                            && !finish_escalated
                        {
                            logger
                                .log(
                                    LogLevel::Info,
                                    Some("daemon".into()),
                                    format!(
                                        "`{node_id}` exited with {exit_status:?} during planned stop; treating as clean"
                                    ),
                                )
                                .await;
                            Ok(())
                        } else {
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
                                None if grace_duration_kill || finish_escalated => {
                                    NodeErrorCause::GraceDuration
                                }
                                None if startup_timed_out && is_sigkill_like_exit(&exit_status) => {
                                    let cause = dataflow
                                        .and_then(|d| d.node_stderr_most_recent.get(&node_id))
                                        .map(|queue| {
                                            let mut lines = Vec::new();
                                            if queue.is_full() {
                                                lines.push("[...]\n".into());
                                            }
                                            while let Some(line) = queue.pop() {
                                                lines.push(line);
                                            }
                                            lines
                                        })
                                        .map(extract_err_from_stderr)
                                        .unwrap_or_default();

                                    let timeout_msg = "process killed: startup_timeout exceeded before node connected";
                                    let stderr = if cause.is_empty() {
                                        timeout_msg.to_string()
                                    } else {
                                        format!("{timeout_msg}\n{cause}")
                                    };
                                    NodeErrorCause::Other { stderr }
                                }
                                None => {
                                    let cause = dataflow
                                        .and_then(|d| d.node_stderr_most_recent.get(&node_id))
                                        .map(|queue| {
                                            let mut lines = Vec::new();
                                            if queue.is_full() {
                                                lines.push("[...]\n".into());
                                            }
                                            while let Some(line) = queue.pop() {
                                                lines.push(line);
                                            }
                                            lines
                                        })
                                        .map(extract_err_from_stderr)
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
                    }
                };

                // Drop the consumed kill marker. `grace_duration_kills` is
                // keyed by `(node_id, generation)`, so a successor can no
                // longer inherit its predecessor's marker structurally —
                // removal here is hygiene for this incarnation's own entry
                // (it was consumed classifying this exit), not the
                // cross-incarnation leak protection it used to be. Same for
                // the drain clock: a respawned node under the same id must
                // start fresh.
                // (`finish_escalated` is NOT cleared here — it is read
                // and consumed by `handle_node_stop_inner` below to keep
                // the coordinator-facing `clean_stop` flag honest; an
                // escalated node never restarts, so it cannot leak into
                // a next incarnation.)
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    dataflow
                        .grace_duration_kills
                        .remove(&(node_id.clone(), generation));
                    dataflow
                        .startup_timeout_kills
                        .remove(&(node_id.clone(), generation));
                    dataflow.all_inputs_closed_at.remove(&node_id);
                    // a respawned node must re-subscribe before it counts as
                    // connected, else a slow restart could be silence-escalated
                    // mid-startup (dora-rs/dora#2270).
                    dataflow.connected_nodes.remove(&node_id);
                }

                // A node that crashed cannot withdraw its own descriptors.
                // Reclaiming here is the reason the extension table lives in
                // the daemon at all (dora-rs/dora#2881).
                // The free function, not the `&mut self` wrapper: `logger`
                // holds a mutable borrow of `self.logger` across this point.
                reclaim_extensions_of_exited_node(
                    &mut self.extensions,
                    self.running.get(&dataflow_id),
                    dataflow_id,
                    &node_id,
                    &self.clock,
                );

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

                if restart {
                    logger
                        .log(
                            LogLevel::Info,
                            Some("daemon".into()),
                            format!("node will be restarted (attempt {})", restart_count + 1),
                        )
                        .await;

                    // Notify downstream nodes about the restart
                    if let Some(dataflow) = self.running.get(&dataflow_id) {
                        let downstream: BTreeSet<NodeId> = dataflow
                            .mappings
                            .iter()
                            .filter(|(k, _)| k.0 == node_id)
                            .flat_map(|(_, v)| v)
                            .map(|(receiver_id, _)| receiver_id.clone())
                            .collect();
                        for receiver_id in &downstream {
                            if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
                                // First try the non-blocking path. NodeRestarted is
                                // control-classed inside send_with_timestamp, so it
                                // benefits from the control headroom reservation.
                                match send_with_timestamp(
                                    channel,
                                    NodeEvent::NodeRestarted {
                                        id: node_id.clone(),
                                    },
                                    &self.clock,
                                ) {
                                    Ok(true) => {
                                        dataflow.inc_pending(receiver_id);
                                    }
                                    Ok(false) => {
                                        // Channel full even with control headroom.
                                        // NodeRestarted is a critical lifecycle event:
                                        // dropping it leaves service/action clients
                                        // blocked on pre-crash correlations forever
                                        // (dora-rs/adora#148). Guarantee delivery with a
                                        // backpressure-aware send — but do NOT await it
                                        // on the daemon main loop.
                                        //
                                        // Awaiting here suspends the single serial event
                                        // loop until the receiver drains a slot. If that
                                        // receiver's Listener is itself parked on
                                        // `daemon_tx.send().await` (the daemon event
                                        // channel at capacity), it never returns to drain
                                        // its subscribe channel, so the two block each
                                        // other forever and the whole daemon hangs
                                        // (dora-rs/dora#3066). Offload the awaiting send
                                        // to a detached task holding cloned handles so the
                                        // main loop keeps draining `dora_events_rx`.
                                        tracing::warn!(
                                            %dataflow_id,
                                            restarted_node = %node_id,
                                            %receiver_id,
                                            "NodeRestarted try_send failed (channel full); \
                                             offloading backpressure delivery to a task"
                                        );
                                        let channel = channel.clone();
                                        // Clone the receiver's pending counter so the task
                                        // can pair the enqueue with `inc_pending` off the
                                        // main loop. The Listener decrements once per
                                        // drained event, so the increment must land iff
                                        // the message is actually enqueued (Ok), exactly
                                        // as the inline delivery sites do (dora-rs/dora#2827).
                                        let pending =
                                            dataflow.pending_messages.get(receiver_id).cloned();
                                        let msg = Timestamped {
                                            inner: NodeEvent::NodeRestarted {
                                                id: node_id.clone(),
                                            },
                                            timestamp: self.clock.new_timestamp(),
                                        };
                                        let restarted_node = node_id.clone();
                                        let receiver = receiver_id.clone();
                                        tokio::spawn(async move {
                                            match channel.send(msg).await {
                                                Ok(()) => {
                                                    if let Some(counter) = pending {
                                                        counter.fetch_add(
                                                            1,
                                                            std::sync::atomic::Ordering::Relaxed,
                                                        );
                                                    }
                                                }
                                                Err(_closed) => {
                                                    tracing::warn!(
                                                        %dataflow_id,
                                                        %restarted_node,
                                                        %receiver,
                                                        "NodeRestarted delivery failed: \
                                                         receiver channel closed"
                                                    );
                                                }
                                            }
                                        });
                                    }
                                    Err(_) => {
                                        tracing::warn!(
                                            %dataflow_id,
                                            restarted_node = %node_id,
                                            %receiver_id,
                                            "failed to send NodeRestarted: receiver channel closed"
                                        );
                                    }
                                }
                            }
                        }
                    }
                } else {
                    // Send NodeFailed events to downstream nodes when a
                    // node exits with a non-zero exit code (matches
                    // upstream dora behavior for error propagation).
                    if let Err(e) = &node_result
                        && let Some(dataflow) = self.running.get(&dataflow_id)
                    {
                        dataflow.propagate_node_failed(&node_id, &e.to_string(), &self.clock);
                    }

                    let exit_clean = node_result.is_ok();
                    self.dataflow_node_results
                        .entry(dataflow_id)
                        .or_default()
                        .insert(node_id.clone(), node_result);

                    self.handle_node_stop(dataflow_id, &node_id, dynamic_node, exit_clean)
                        .await?;
                }
            }
            DoraEvent::ProcessHandleReplaced {
                dataflow_id,
                node_id,
                previous_generation,
                new_generation,
                new_handle,
            } => {
                // The per-node restart_loop just spawned a replacement
                // process with a fresh op_tx/op_rx pair. Swap it into
                // running_nodes so subsequent stop/kill operations
                // target the new incarnation rather than a dead
                // predecessor's channel (dora-rs/adora#152).
                if let Some(dataflow) = self.running.get_mut(&dataflow_id) {
                    if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                        match node.replace_process_handle(
                            previous_generation,
                            new_generation,
                            new_handle,
                        ) {
                            running_dataflow::HandleReplacement::Replaced => {}
                            running_dataflow::HandleReplacement::RejectedTeardown(new_handle) => {
                                dataflow.stop_rejected_replacement(
                                    &node_id,
                                    new_generation,
                                    new_handle,
                                );
                                tracing::info!(
                                    %dataflow_id,
                                    %node_id,
                                    new_generation,
                                    "teardown won the respawn race: stopping the replacement \
                                     process with the active dataflow stop policy"
                                );
                            }
                            running_dataflow::HandleReplacement::RejectedStale => {
                                tracing::warn!(
                                    %dataflow_id,
                                    %node_id,
                                    previous_generation,
                                    current_generation = node.generation,
                                    "ignoring stale ProcessHandleReplaced event"
                                );
                            }
                        }
                    } else {
                        tracing::warn!(
                            %dataflow_id,
                            %node_id,
                            "ProcessHandleReplaced for unknown node"
                        );
                    }
                } else {
                    tracing::warn!(
                        %dataflow_id,
                        %node_id,
                        "ProcessHandleReplaced for unknown dataflow"
                    );
                }
            }
        }
        Ok(())
    }
}
