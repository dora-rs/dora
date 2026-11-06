//! Handling of requests from local nodes: subscribe, send-out, outputs-done,
//! stop, and the dynamic-node handshake.

use crate::local_listener::DynamicNodeEventWrapper;
use crate::{
    Daemon, DaemonNodeEvent, Event, InterDaemonEvent, OutputId, RunningDataflow,
    ZENOH_OUTPUT_QUEUE_CAPACITY, ZENOH_PUBLISH_TIMEOUT, ZENOH_REQUIRED_ACK_TIMEOUT, ZenohOutbound,
    close_inputs_best_effort, drop_extension_and_notify, extension_table::ExtensionKey,
    local_delivery::DeferredDelivery, note_output_sent_to_local_receivers,
    running_dataflow::ControlPublisher, send_output_to_local_receivers, send_with_timestamp,
};
use dora_core::{
    config::{DataId, NodeId, OperatorId},
    topics::zenoh_daemon_control_topic,
    uhlc::HLC,
};
use dora_message::{
    DataflowId,
    common::{DataMessage, LogLevel},
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent},
    daemon_to_node::{DaemonReply, NodeEvent},
    descriptor::RestartPolicy,
    dynamic_node::DynamicNodeConfigReply,
    node_to_daemon::{DynamicNodeEvent, Timestamped},
};
use eyre::{Context, ContextCompat, Result, eyre};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    sync::{
        Arc,
        atomic::{self, AtomicBool, Ordering},
    },
    time::Instant,
};
use tokio::sync::{mpsc, oneshot};
use tracing::error;
use uuid::Uuid;
use zenoh::Wait;
use zenoh::qos::{CongestionControl, Priority};

/// Cap on `Daemon::warned_late_outputs`, so a daemon that serves many
/// dataflows cannot accumulate an entry per (dataflow, node) forever.
/// Reaching it clears the set rather than freezing it: a daemon is meant
/// to run indefinitely, and simply refusing new entries would silence
/// the warning permanently after ~1024 dataflows — including for the
/// stale-node case that is the reason it is a warning at all.
pub(crate) const MAX_WARNED_LATE_OUTPUT_NODES: usize = 1024;

/// Picks the running dataflow a connecting dynamic node `node_id` belongs to.
///
/// Only a live dataflow that declares `node_id` as dynamic counts: a static
/// node with the same id in another dataflow, or a dataflow that is already
/// stopping, must not make the join ambiguous. If no dataflow qualifies, any
/// dataflow with that id is returned so that `dynamic_node_config` reports
/// why (not dynamic, or stopping).
pub(crate) fn dataflow_for_dynamic_node<'a>(
    running: &'a mut HashMap<DataflowId, RunningDataflow>,
    node_id: &NodeId,
) -> Result<&'a mut RunningDataflow, String> {
    let no_node = || format!("no node with ID `{node_id}`");
    let mut dynamic = running
        .iter()
        .filter(|(_, df)| {
            !df.stop_sent
                && df
                    .running_nodes
                    .get(node_id)
                    .is_some_and(|node| node.node_config.dynamic)
        })
        .map(|(id, _)| *id);
    let id = match (dynamic.next(), dynamic.next()) {
        (Some(_), Some(_)) => {
            return Err(format!(
                "multiple dataflows contain dynamic node id {node_id}. \
                Please only have one running dataflow with the specified \
                node id if you want to use dynamic node",
            ));
        }
        (Some(id), None) => id,
        (None, _) => running
            .iter()
            .find(|(_, df)| df.running_nodes.contains_key(node_id))
            .map(|(id, _)| *id)
            .ok_or_else(no_node)?,
    };
    running.get_mut(&id).ok_or_else(no_node)
}

/// The control publisher for `output_id`, declared on first use and cached
/// per output. The cache is hit on every later message, so it is probed by
/// reference and the `OutputId` key (two heap strings) is cloned only on the
/// one-time miss.
async fn control_publisher<'a>(
    session: &zenoh::Session,
    publishers: &'a mut BTreeMap<OutputId, ControlPublisher>,
    dataflow_id: Uuid,
    output_id: &OutputId,
) -> eyre::Result<&'a ControlPublisher> {
    if !publishers.contains_key(output_id) {
        let publish_topic = zenoh_daemon_control_topic(dataflow_id, &output_id.0, &output_id.1);
        tracing::debug!("declaring control publisher on {publish_topic}");
        let publisher = session
            .declare_publisher(publish_topic)
            .congestion_control(CongestionControl::Drop)
            .express(true)
            .priority(Priority::RealTime)
            .await
            .map_err(|err| eyre!(err))
            .context("failed to create zenoh publisher")?;
        let has_subscribers = Arc::new(AtomicBool::new(false));
        let flag = has_subscribers.clone();
        // Lives as long as the publisher; registered before the initial read
        // so no change between the two is missed.
        publisher
            .matching_listener()
            .callback(move |status| flag.store(status.matching(), Ordering::Relaxed))
            .background()
            .await
            .map_err(|err| eyre!(err))
            .context("failed to watch the zenoh publisher's subscribers")?;
        if let Ok(status) = publisher.matching_status().await
            && status.matching()
        {
            has_subscribers.store(true, Ordering::Relaxed);
        }
        publishers.insert(
            output_id.clone(),
            ControlPublisher {
                publisher: Arc::new(publisher),
                has_subscribers,
            },
        );
    }
    Ok(&publishers[output_id])
}

impl Daemon {
    pub(crate) async fn handle_dynamic_node_event(
        &mut self,
        event: DynamicNodeEventWrapper,
    ) -> eyre::Result<()> {
        match event {
            DynamicNodeEventWrapper {
                event: DynamicNodeEvent::NodeConfig { node_id },
                reply_tx,
            } => {
                let node_config =
                    dataflow_for_dynamic_node(&mut self.running, &node_id).and_then(|dataflow| {
                        dataflow
                            .dynamic_node_config(&node_id, self.zenoh_listen_endpoint.as_deref())
                            .map_err(|err| {
                                format!(
                                    "failed to get dynamic node config within given dataflow: {err}"
                                )
                            })
                    });

                let (result, zenoh) = match node_config {
                    Ok((config, peering)) => (Ok(config), Some(peering)),
                    Err(err) => (Err(err), None),
                };
                let reply = DynamicNodeConfigReply::NodeConfig { result, zenoh };
                let _ = reply_tx.send(Some(reply)).map_err(|_| {
                    error!("could not send node info reply from daemon to coordinator")
                });
                Ok(())
            }
        }
    }

    pub(crate) async fn handle_node_event(
        &mut self,
        event: DaemonNodeEvent,
        dataflow_id: DataflowId,
        node_id: NodeId,
    ) -> eyre::Result<()> {
        let might_restart = || {
            let dataflow = self.running.get(&dataflow_id)?;
            let node = dataflow.running_nodes.get(&node_id)?;
            Some(match node.restart_policy {
                RestartPolicy::Never => false,
                _ if node.restarts_disabled() => false,
                RestartPolicy::OnFailure | RestartPolicy::Always => true,
            })
        };
        match event {
            DaemonNodeEvent::Subscribe {
                event_sender,
                pending_counter,
                drained,
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
                        dataflow
                            .pending_messages
                            .insert(node_id.clone(), pending_counter);
                        dataflow.drain_signals.insert(node_id.clone(), drained);
                        Self::subscribe(dataflow, node_id.clone(), event_sender, &self.clock).await;

                        let status = dataflow
                            .pending_nodes
                            .handle_node_subscription(
                                node_id.clone(),
                                reply_sender,
                                &mut self.coordinator_sender,
                                &self.clock,
                                &mut dataflow.cascading_error_causes,
                                &mut logger,
                            )
                            .await?;
                        // `should_start_on_barrier_completion` keeps this to a
                        // single spawn (`start()` sets `dataflow_started`) and
                        // suppresses it entirely once `stop_sent` is set. A late
                        // subscribe during teardown is an expected path, not a
                        // hypothetical one: `subscribe()` above has a dedicated
                        // `stop_sent` branch that answers such a node with
                        // `Stop`. If that node is the last pending cohort member,
                        // its subscription completes the barrier and would
                        // otherwise start the dataflow that is already stopping
                        // (dora-rs/dora#3053).
                        if dataflow.should_start_on_barrier_completion(&status) {
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
                    }
                }
            }
            DaemonNodeEvent::CloseOutputs {
                outputs,
                reply_sender,
            } => {
                let reply = if might_restart().unwrap_or(false) {
                    self.logger
                        .for_dataflow(dataflow_id)
                        .for_node(node_id.clone())
                        .log(
                            LogLevel::Debug,
                            Some("daemon".into()),
                            "skipping CloseOutputs because node might restart",
                        )
                        .await;
                    Ok(())
                } else {
                    // notify downstream nodes
                    let inner = async {
                        self.send_output_closed_events(dataflow_id, node_id, outputs)
                            .await
                    };

                    inner.await.map_err(|err| format!("{err:?}"))
                };
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
            DaemonNodeEvent::OutputsDone { reply_sender } => {
                let result = self
                    .handle_outputs_done(dataflow_id, &node_id, might_restart().unwrap_or(false))
                    .await;

                let _ = reply_sender.send(DaemonReply::Result(
                    result.map_err(|err| format!("{err:?}")),
                ));
            }
            DaemonNodeEvent::SendOut {
                output_id,
                metadata,
                data,
                deferred_reply,
            } => self
                .send_out(
                    dataflow_id,
                    node_id,
                    output_id,
                    metadata,
                    data,
                    deferred_reply,
                )
                .await
                .context("failed to send out")?,
            DaemonNodeEvent::OutputSent {
                output_id,
                metadata,
            } => self
                .output_sent(dataflow_id, node_id, output_id, metadata)
                .context("failed to mark output sent")?,
            DaemonNodeEvent::ExtensionStore {
                namespace,
                key,
                value,
                reply_sender,
            } => {
                let ext_key = ExtensionKey {
                    dataflow_id: dataflow_id.to_string(),
                    namespace,
                    key,
                };
                let result = self.extensions.store(ext_key, value, &node_id);
                let _ = reply_sender.send(DaemonReply::Result(result));
            }
            DaemonNodeEvent::ExtensionLoad {
                namespace,
                key,
                remove,
                reply_sender,
            } => {
                let ext_key = ExtensionKey {
                    dataflow_id: dataflow_id.to_string(),
                    namespace,
                    key,
                };
                let value = self.extensions.load(&ext_key, &node_id);
                // Remove only after a hit: a miss must not broadcast a drop
                // for a key that was never there.
                if remove && value.is_some() {
                    drop_extension_and_notify(
                        &mut self.extensions,
                        self.running.get(&dataflow_id),
                        &ext_key,
                        &self.clock,
                    );
                }
                let _ = reply_sender.send(DaemonReply::ExtensionValue { value });
            }
            DaemonNodeEvent::ExtensionDrop {
                namespace,
                key,
                reply_sender,
            } => {
                let ext_key = ExtensionKey {
                    dataflow_id: dataflow_id.to_string(),
                    namespace,
                    key,
                };
                drop_extension_and_notify(
                    &mut self.extensions,
                    self.running.get(&dataflow_id),
                    &ext_key,
                    &self.clock,
                );
                // Idempotent: dropping an absent key is success, so a retry
                // after a lost reply does not surface as an error.
                let _ = reply_sender.send(DaemonReply::Result(Ok(())));
            }
            DaemonNodeEvent::EventStreamDropped { reply_sender } => {
                let inner = async {
                    let dataflow = self
                        .running
                        .get_mut(&dataflow_id)
                        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
                    // Remove the send channel and mark this as a deliberate drop
                    // on normal shutdown, so an upstream still producing to this
                    // consumer in the window before its process exit is observed
                    // does not trigger the "failed to re-subscribe" warning
                    // (dora-rs/dora#3556).
                    dataflow.mark_event_stream_dropped(&node_id);
                    Result::<_, eyre::Error>::Ok(())
                };

                let reply = inner.await.map_err(|err| format!("{err:?}"));
                let _ = reply_sender.send(DaemonReply::Result(reply));
            }
            DaemonNodeEvent::ExtensionRequest {
                namespace,
                payload,
                reply_sender,
            } => {
                #[cfg(feature = "tensor-pool")]
                if let Err(err) = self
                    .handle_extension_request(
                        dataflow_id,
                        node_id,
                        namespace,
                        payload,
                        reply_sender,
                    )
                    .await
                {
                    // Defensive: the handler currently always returns `Ok(())`
                    // at function scope, so this arm is unreachable today, but
                    // its signature is fallible. If it ever did return `Err`, a
                    // `?` here would unwind the daemon's main loop and drop the
                    // coordinator connection — losing every other dataflow's
                    // connection too — and it owns the reply channel (it may
                    // already have answered the node). Log and carry on,
                    // matching the `ExtensionStore`/`ExtensionLoad`/
                    // `ExtensionDrop` arms rather than diverging from them.
                    tracing::error!("failed to handle extension request: {err:?}");
                }
                // The node asked for an extension this daemon was not built
                // with. Answer explicitly so it fails loudly instead of
                // waiting on a reply that never comes.
                #[cfg(not(feature = "tensor-pool"))]
                {
                    let _ = payload;
                    let _ = reply_sender.send(DaemonReply::Result(Err(format!(
                        "no extension registered under {namespace:?} on this daemon"
                    ))));
                }
            }
        }
        Ok(())
    }

    pub(crate) async fn send_reload(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<(), eyre::ErrReport> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("Reload failed: no running dataflow with ID `{dataflow_id}`")
        })?;
        let Some(channel) = reload_target(dataflow, &node_id)? else {
            return Ok(());
        };
        match send_with_timestamp(channel, NodeEvent::Reload { operator_id }, &self.clock) {
            Ok(true) => {
                dataflow.inc_pending(&node_id);
            }
            Ok(false) => return Err(eyre!("node `{node_id}` channel full")),
            Err(_) => {
                dataflow.subscribe_channels.remove(&node_id);
                return Err(eyre!("node `{node_id}` channel closed"));
            }
        }
        Ok(())
    }

    /// Record an output event that named a dataflow the daemon no longer
    /// runs, and drop it.
    ///
    /// Usually this means the dataflow finished while the node still had
    /// traffic in flight. Two ways in, both normal:
    ///
    /// * a node's exit and its already-transmitted outputs reach the
    ///   daemon's event loop on independent paths, so a burst sent just
    ///   before exit can be queued behind the `SpawnedNodeResult` that
    ///   finished the dataflow (the intermittent nightly `smoke-suite`
    ///   failure in dora-rs/dora#2742);
    /// * `should_finish` ignores still-running *dynamic* nodes, so a
    ///   dynamic node is expected to outlive `finish_dataflow` and may
    ///   keep sending for as long as it likes afterwards.
    ///
    /// It can also mean a stale or misconfigured node: registration takes
    /// `dataflow_id` from the node's own `Register` request and only
    /// version-checks it, so an id the daemon never ran reaches here too.
    /// That is why this warns rather than logging at `debug`.
    ///
    /// Fatal it must not be: neither `SendOut` nor `OutputSent` carries a
    /// reply channel, so `handle_node_event` cannot report the error back
    /// to the node the way every sibling arm does — it can only return
    /// `Err`, which unwinds the daemon's main loop and drops its
    /// coordinator connection — which loses this message *and* every
    /// other dataflow's connection with it.
    ///
    /// Dropping it is not always free, though: `should_finish` only looks
    /// at this daemon's non-dynamic nodes, so a still-running local
    /// dynamic consumer, or a consumer on another daemon reached through
    /// `open_external_mappings`, can still have been waiting for it. That
    /// is a pre-existing finish-ordering gap, strictly better than taking
    /// the daemon down for it, and why this is a warning rather than a
    /// silent drop.
    ///
    /// Warns once per node, then falls to `debug`: the dynamic-node case
    /// above is unbounded, and a node sending at 1 kHz would otherwise
    /// emit 1000 warn lines a second for as long as it stayed alive.
    pub(crate) fn log_late_node_output(
        &mut self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        output_id: &DataId,
        what: &'static str,
    ) {
        if self.warned_late_outputs.len() >= MAX_WARNED_LATE_OUTPUT_NODES {
            self.warned_late_outputs.clear();
        }
        if self
            .warned_late_outputs
            .insert((*dataflow_id, node_id.clone()))
        {
            tracing::warn!(
                %dataflow_id, %node_id, %output_id,
                "ignoring `{what}` for a dataflow that already finished \
                 (node outlived its dataflow); further such messages from \
                 this node are logged at debug level"
            );
        } else {
            tracing::debug!(
                %dataflow_id, %node_id, %output_id,
                "ignoring `{what}` for a dataflow that already finished"
            );
        }
    }

    /// `deferred_reply`, when given, receives the deliveries local routing
    /// could not complete (see `DeferredDelivery`). Dropping it unanswered —
    /// the dataflow is gone, delivery failed — tells the waiting listener
    /// there is nothing to wait for.
    pub(crate) async fn send_out(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        metadata: dora_message::metadata::Metadata,
        data: Option<DataMessage>,
        deferred_reply: Option<oneshot::Sender<Vec<DeferredDelivery>>>,
    ) -> Result<(), eyre::ErrReport> {
        let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
            self.log_late_node_output(&dataflow_id, &node_id, &output_id, "send out");
            return Ok(());
        };
        // Build the `(NodeId, DataId)` routing key once by moving `node_id`/
        // `output_id` in (neither is read past this point). It is used for the
        // two `contains` lookups below, passed by reference into local delivery,
        // and reused for the inter-daemon path — avoiding the previous
        // per-message clone of the `NodeId`/`DataId` on this output-dispatch hot
        // path.
        let output_id_key = OutputId(node_id, output_id);
        // An output takes the remote path when a remote daemon has an input
        // mapped to it, or, in a `--debug` dataflow, while anyone subscribes
        // to its control topic (a user's own zenoh client, for example; dora's
        // tools go through `has_debug_watchers` below). Publishing a `--debug`
        // dataflow's outputs unconditionally would copy and serialize every
        // payload for nobody, so ask zenoh whether a subscriber matches. A
        // subscriber misses what is sent before its declaration propagates,
        // which is fine for a debug tap.
        let remote_receivers = dataflow.open_external_mappings.contains(&output_id_key)
            || (dataflow.enable_debug_inspection
                && match control_publisher(
                    &self.zenoh_session,
                    &mut dataflow.publishers,
                    dataflow.id,
                    &output_id_key,
                )
                .await
                {
                    Ok(publisher) => publisher.has_subscribers.load(Ordering::Relaxed),
                    // Local delivery below must not depend on zenoh: treat a
                    // failed lookup as "nobody subscribed" and move on.
                    Err(err) => {
                        tracing::warn!(
                            "debug publishing of `{}/{}` disabled for this message: {err:?}",
                            output_id_key.0,
                            output_id_key.1
                        );
                        false
                    }
                });
        let has_debug_watchers = dataflow.debug_topic_watchers.contains_key(&output_id_key);
        let mut deferred = Vec::new();
        let data_bytes = send_output_to_local_receivers(
            &output_id_key,
            dataflow,
            &metadata,
            data,
            &self.clock,
            Some(&self.ft_stats),
            remote_receivers || has_debug_watchers,
            deferred_reply.is_some().then_some(&mut deferred),
        )
        .await?;
        if let Some(reply) = deferred_reply
            && let Err(deferred) = reply.send(deferred)
        {
            // The producer's listener went away with its node before it could
            // take these over: they are lost, on edges that were promised not
            // to lose anything.
            for delivery in &deferred {
                tracing::warn!(
                    node = %delivery.receiver,
                    "producer `{}` exited while `{}` was waiting for room: dropping message",
                    output_id_key.0,
                    output_id_key.1,
                );
            }
            self.ft_stats.record_drop(deferred.len() as u64, true);
        }

        if !remote_receivers && !has_debug_watchers {
            return Ok(());
        }

        let output_id = output_id_key;
        let event = InterDaemonEvent::Output {
            dataflow_id,
            node_id: output_id.0.clone(),
            output_id: output_id.1.clone(),
            metadata,
            data: data_bytes,
        };
        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        }
        .serialize()
        .wrap_err("failed to serialize inter-daemon event")?;

        if has_debug_watchers {
            if remote_receivers {
                self.send_topic_debug_frames(dataflow_id, &output_id, serialized_event.clone())
                    .await?;
            } else {
                self.send_topic_debug_frames(dataflow_id, &output_id, serialized_event)
                    .await?;
                return Ok(());
            }
        }

        if remote_receivers {
            // Regular outputs are data-plane messages. Backpressure should drop
            // them instead of surfacing an error that stops the daemon event loop.
            self.try_send_to_remote_receivers(dataflow_id, &output_id, serialized_event)
                .await?;
        }

        Ok(())
    }

    pub(crate) fn output_sent(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        _metadata: dora_message::metadata::Metadata,
    ) -> Result<(), eyre::ErrReport> {
        let Some(dataflow) = self.running.get_mut(&dataflow_id) else {
            self.log_late_node_output(&dataflow_id, &node_id, &output_id, "output sent");
            return Ok(());
        };
        note_output_sent_to_local_receivers(
            node_id,
            output_id,
            dataflow,
            &self.clock,
            Some(&self.ft_stats),
        );
        Ok(())
    }

    #[cfg(test)]
    pub(crate) async fn send_to_remote_receivers(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        serialized_event: Vec<u8>,
    ) -> Result<(), eyre::Error> {
        let (queue, outbound, receipt) = self
            .prepare_remote_close(dataflow_id, output_id, serialized_event)
            .await?;
        await_remote_close(queue, outbound, receipt).await
    }

    async fn prepare_remote_close(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        serialized_event: Vec<u8>,
    ) -> eyre::Result<(
        mpsc::Sender<ZenohOutbound>,
        ZenohOutbound,
        oneshot::Receiver<eyre::Result<()>>,
    )> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;

        let publisher =
            remote_receiver_control_publisher(&self.zenoh_session, dataflow, output_id).await?;
        let queue = remote_output_publish_queue(dataflow, output_id).await?;
        let (receipt_tx, receipt_rx) = oneshot::channel();
        let mut outbound = zenoh_outbound(
            dataflow,
            publisher,
            serialized_event,
            Some(ZENOH_PUBLISH_TIMEOUT),
        );
        outbound.receipt = Some(receipt_tx);
        Ok((queue, outbound, receipt_rx))
    }

    pub(crate) async fn try_send_to_remote_receivers(
        &mut self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        serialized_event: Vec<u8>,
    ) -> Result<(), eyre::Error> {
        let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
            format!("send out failed: no running dataflow with ID `{dataflow_id}`")
        })?;

        let publisher = control_publisher(
            &self.zenoh_session,
            &mut dataflow.publishers,
            dataflow.id,
            output_id,
        )
        .await?
        .publisher
        .clone();
        let queue = remote_output_publish_queue(dataflow, output_id).await?;
        let outbound = zenoh_outbound(dataflow, publisher, serialized_event, None);
        let remote_backpressured = dataflow.remote_backpressured_outputs.contains(output_id);
        let enqueue_result = try_enqueue_zenoh_outbound(&queue, outbound);
        if enqueue_result != ZenohEnqueueResult::Queued {
            self.ft_stats.record_drop(1, remote_backpressured);
        }
        handle_publish_enqueue_result(dataflow, enqueue_result, "regular inter-daemon output");

        Ok(())
    }

    pub(crate) async fn send_topic_debug_frames(
        &self,
        dataflow_id: Uuid,
        output_id: &OutputId,
        serialized_event: Vec<u8>,
    ) -> Result<(), eyre::Error> {
        let Some(sender) = &self.coordinator_sender else {
            return Ok(());
        };
        let Some(dataflow) = self.running.get(&dataflow_id) else {
            return Ok(());
        };
        let Some(subscription_ids) = dataflow.debug_topic_watchers.get(output_id) else {
            return Ok(());
        };
        let subscription_ids: Vec<_> = subscription_ids.iter().copied().collect();
        let subscription_count = subscription_ids.len();

        match sender.try_send_topic_debug_frame(
            &self.daemon_id,
            &self.clock,
            dataflow_id,
            subscription_ids,
            serialized_event,
        ) {
            Ok(()) => {}
            Err(crate::coordinator::TrySendEventError::Full) => {
                tracing::warn!(
                    %dataflow_id,
                    output = %format!("{}/{}", output_id.0, output_id.1),
                    subscriptions = subscription_count,
                    "dropping topic debug frame because the topic debug queue is full"
                );
            }
            Err(err @ crate::coordinator::TrySendEventError::TooLarge { .. }) => {
                tracing::warn!(
                    %dataflow_id,
                    output = %format!("{}/{}", output_id.0, output_id.1),
                    subscriptions = subscription_count,
                    "dropping topic debug frame: {err}"
                );
            }
            Err(crate::coordinator::TrySendEventError::Closed) => {
                tracing::warn!(
                    %dataflow_id,
                    output = %format!("{}/{}", output_id.0, output_id.1),
                    subscriptions = subscription_count,
                    "dropping topic debug frame because coordinator WS send channel is closed"
                );
            }
            Err(
                err @ (crate::coordinator::TrySendEventError::InvalidUtf8(_)
                | crate::coordinator::TrySendEventError::Encode(_)),
            ) => {
                return Err(eyre!(
                    "failed to encode topic debug frame for coordinator: {err}"
                ));
            }
        }

        Ok(())
    }

    pub(crate) async fn send_output_closed_events(
        &mut self,
        dataflow_id: DataflowId,
        node_id: NodeId,
        outputs: Vec<DataId>,
    ) -> eyre::Result<()> {
        let dataflow = self
            .running
            .get_mut(&dataflow_id)
            .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
        // Look each closed output up by key rather than scanning the whole
        // `mappings` map: `outputs` is small (the outputs of one node), while
        // `mappings` holds an entry per edge in the entire dataflow. This
        // matches the keyed access every other delivery site uses (e.g.
        // `send_output_to_local_receivers`) and is behavior-equivalent — the
        // results are merged into the same deduplicating `BTreeSet`.
        let mut local_node_inputs: BTreeSet<(NodeId, DataId)> = BTreeSet::new();
        for output in &outputs {
            if let Some(receivers) = dataflow
                .mappings
                .get(&OutputId(node_id.clone(), output.clone()))
            {
                local_node_inputs.extend(receivers.iter().cloned());
            }
        }
        let local_close_result = close_inputs_best_effort(
            dataflow,
            local_node_inputs,
            &self.clock,
            "failed to deliver InputClosed while closing outputs",
        );

        let mut closed = Vec::new();
        for output_id in &dataflow.open_external_mappings {
            if output_id.0 == node_id && outputs.contains(&output_id.1) {
                closed.push(output_id.clone());
            }
        }

        tokio::time::timeout(ZENOH_REQUIRED_ACK_TIMEOUT * 3, async {
            let mut close_result = local_close_result;
            let mut remote_closes = Vec::with_capacity(closed.len());
            for output_id in closed {
                let serialized_event = Timestamped {
                    inner: InterDaemonEvent::OutputClosed {
                        dataflow_id,
                        node_id: output_id.0.clone(),
                        output_id: output_id.1.clone(),
                    },
                    timestamp: self.clock.new_timestamp(),
                }
                .serialize()
                .wrap_err("failed to serialize inter-daemon output-closed event")?;
                match self
                    .prepare_remote_close(dataflow_id, &output_id, serialized_event)
                    .await
                {
                    Ok((queue, outbound, receipt)) => remote_closes.push(async move {
                        let result = await_remote_close(queue, outbound, receipt).await;
                        (output_id, result)
                    }),
                    Err(err) => {
                        tracing::error!(?output_id, "failed to prepare OutputClosed: {err:?}");
                        if close_result.is_ok() {
                            close_result = Err(err);
                        }
                    }
                }
            }
            for (output_id, result) in futures::future::join_all(remote_closes).await {
                if let Err(err) = result {
                    tracing::error!(?output_id, "failed to publish OutputClosed: {err:?}");
                    if close_result.is_ok() {
                        close_result = Err(err);
                    }
                }
            }
            close_result
        })
        .await
        .map_err(|_| eyre!("timed out closing inter-daemon outputs as a batch"))?
    }

    pub(crate) async fn subscribe(
        dataflow: &mut RunningDataflow,
        node_id: NodeId,
        event_sender: mpsc::Sender<Timestamped<NodeEvent>>,
        clock: &HLC,
    ) {
        // record that this node has connected — it stays a finish-straggler
        // candidate even if it later drops its event stream (dora#2270).
        dataflow.connected_nodes.insert(node_id.clone());
        dataflow.pending_remote_all_inputs_closed.remove(&node_id);

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
            if send_with_timestamp(
                &event_sender,
                NodeEvent::InputClosed {
                    id: input_id.clone(),
                },
                clock,
            )
            .ok()
                == Some(true)
            {
                dataflow.inc_pending(&node_id);
            }
        }
        // Restart bookkeeping needs BOTH conditions, and they diverge under
        // the drain opt-in:
        //   - "nothing left open" keeps the pre-existing behavior. Other
        //     code depends on it firing at subscribe time for source nodes
        //     (see the `grace_duration_kill` comment in `handle_node_stop`),
        //     and `is_drained` deliberately reports false for a source.
        //     Dropping it would let a source with `restart_policy: always`
        //     respawn forever, so its outputs never close and the graph
        //     hangs — the failure #2920 exists to prevent.
        //   - "is drained" covers the node we are about to tell to finish.
        //     Without it, such a node exits, gets restarted, is told to
        //     finish again, and loops until `max_restarts`.
        // In the default mode the two are the same predicate. A node with a
        // circuit-broken input is excluded either way: the input is
        // recoverable, so the node is not done and must keep its restart
        // policy — `disable_restart` is one-way, with no re-enable on
        // recovery. This matches `signal_all_inputs_closed_if_drained`.
        if (dataflow.open_inputs(&node_id).is_empty() || dataflow.is_drained(&node_id))
            && !dataflow.has_broken_input(&node_id)
            && let Some(node) = dataflow.running_nodes.get_mut(&node_id)
        {
            node.disable_restart();
        }
        // Sources are not told to finish. The check reads the recorded data
        // inputs rather than the descriptor: `descriptor.nodes[].inputs` is
        // empty for a runtime (`operators:`) node, whose inputs live under
        // `operators[].config.inputs`, so the descriptor calls every operator
        // node a source and skips the event — the hang this issue is about
        // (#2920). Under the opt-in `is_drained` already excludes sources;
        // this still matters in the default mode, where a source has no open
        // inputs and so looks drained.
        if dataflow.is_finished_non_source(&node_id)
            && send_with_timestamp(&event_sender, NodeEvent::AllInputsClosed, clock).ok()
                == Some(true)
        {
            dataflow.inc_pending(&node_id);
            dataflow
                .all_inputs_closed_at
                .insert(node_id.clone(), Instant::now());
        }

        // if a stop event was already sent for the dataflow, send it to
        // the newly connected node too
        if dataflow.stop_sent {
            if let Some(node) = dataflow.running_nodes.get_mut(&node_id) {
                node.disable_restart();
            }
            if send_with_timestamp(&event_sender, NodeEvent::Stop, clock).ok() == Some(true) {
                dataflow.inc_pending(&node_id);
            }
        }

        // The receiver is back: forget its stale missing-stream markers so a
        // later drop of the newly-installed channel warns again (dora-rs/
        // dora#3201), and clear any deliberate-drop marker so a genuine later
        // starvation of this fresh channel is diagnosed, not silenced
        // (dora-rs/dora#3556).
        dataflow
            .missing_channel_warned
            .retain(|(node, _)| node != &node_id);
        dataflow.dropped_event_streams.remove(&node_id);
        dataflow.subscribe_channels.insert(node_id, event_sender);
    }

    #[tracing::instrument(skip(self), level = "trace")]
    pub(crate) async fn handle_outputs_done(
        &mut self,
        dataflow_id: DataflowId,
        node_id: &NodeId,
        might_restart: bool,
    ) -> eyre::Result<()> {
        let dataflow = self
            .running
            .get_mut(&dataflow_id)
            .ok_or_else(|| eyre!("no running dataflow with ID `{dataflow_id}`"))?;

        // Include outputs consumed only by nodes on other daemons
        // (`open_external_mappings`), not just those with a local consumer
        // (`mappings`). Otherwise a remote-only output never triggers an
        // `OutputClosed` event when the producing node finishes, leaving the
        // remote consumer's input open until it is force-killed.
        let outputs = dataflow.node_output_ids(node_id).into_iter().collect();

        if might_restart {
            self.logger
                .for_dataflow(dataflow_id)
                .for_node(node_id.clone())
                .log(
                    LogLevel::Debug,
                    Some("daemon".into()),
                    "keeping outputs open because node might restart",
                )
                .await;
        } else {
            self.send_output_closed_events(dataflow_id, node_id.clone(), outputs)
                .await?;
        }

        Ok(())
    }

    pub(crate) async fn handle_node_stop(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
        dynamic_node: bool,
        exit_clean: bool,
    ) -> eyre::Result<()> {
        let result = self
            .handle_node_stop_inner(dataflow_id, node_id, dynamic_node, exit_clean)
            .await;
        let event = Timestamped {
            inner: Event::NodeStopped {
                dataflow_id,
                node_id: node_id.clone(),
            },
            timestamp: self.clock.new_timestamp(),
        };
        // This runs on the main loop, the only reader of `events_tx`, so
        // awaiting a free slot in a full channel would wait forever. Hand
        // the send to a task instead when the channel is full.
        if let Err(mpsc::error::TrySendError::Full(event)) = self.events_tx.try_send(event) {
            let events_tx = self.events_tx.clone();
            tokio::spawn(async move {
                let _ = events_tx.send(event).await;
            });
        }
        result
    }

    /// `exit_clean` indicates the process exited successfully (Success
    /// exit status). Combined with `restarts_disabled` (operator
    /// requested stop via `dora node stop` → `disable_restart()` set,
    /// even when the SIGTERM-induced exit code is non-zero) it produces
    /// the `clean_stop` flag sent to the coordinator: clean_stop = true
    /// → `NodeStatus::Stopped` (auto-expires from `dora node list` after
    /// the 60s grace), clean_stop = false → `NodeStatus::Failed` (stays
    /// visible so `dora doctor` keeps reporting it). A finish-straggler
    /// escalation (dora-rs/dora#2152) forces clean_stop = false unless
    /// the node still exited 0.
    pub(crate) async fn handle_node_stop_inner(
        &mut self,
        dataflow_id: Uuid,
        node_id: &NodeId,
        dynamic_node: bool,
        exit_clean: bool,
    ) -> eyre::Result<()> {
        let mut logger = self.logger.for_dataflow(dataflow_id);
        let dataflow = match self.running.get_mut(&dataflow_id) {
            Some(dataflow) => dataflow,
            None if dynamic_node => {
                // The dataflow might be done already as we don't wait for dynamic nodes. In this
                // case, we don't need to do anything to handle the node stop.
                tracing::debug!(
                    "dynamic node {dataflow_id}/{node_id} stopped after dataflow was done"
                );
                return Ok(());
            }
            None => {
                // The dataflow finished before this non-dynamic node's stop
                // event was processed. A node's exit and its `SpawnNodeResult`
                // arrive on independent paths, so `should_finish` can conclude
                // the dataflow is done (every remaining `running_nodes` entry is
                // dynamic) and `finish_dataflow` can run before this stop lands
                // — the same "two independent paths" ordering that produced the
                // late-output race fixed in #2742, just narrower. Bailing here
                // returns `Err` up through the daemon's main loop, unwinding
                // `run()` and tearing down the coordinator connection (or
                // failing the whole `dora run`). Treat the missing dataflow as
                // benign — matching the `dynamic_node` arm above and the
                // late-output handling in `send_out`/`output_sent`. Kept at
                // `warn` rather than `debug` because a non-dynamic node arriving
                // late is less expected than a dynamic one (#2986).
                tracing::warn!("node {dataflow_id}/{node_id} stopped after dataflow was done");
                return Ok(());
            }
        };

        let status = dataflow
            .pending_nodes
            .handle_node_stop(
                node_id,
                &mut self.coordinator_sender,
                &self.clock,
                &mut dataflow.cascading_error_causes,
                &mut logger,
            )
            .await?;

        // If this death completed the startup barrier (the dying node was the
        // last cohort member yet to subscribe), the barrier resolves to
        // `AllNodesReady` just as a final subscribe or a `RemoveNode` would —
        // and both of those callers start the dataflow here. Do the same, so a
        // non-cohort survivor that was answered `Ok(())` (since #2933) isn't
        // left parked on a dataflow that never starts and never tears down
        // (#2967). `should_start_on_barrier_completion` keeps that idempotent
        // (it gates on `dataflow_started`) and additionally suppresses the start
        // once the dataflow is stopping (dora-rs/dora#3053) — the stop ladder
        // itself kills a still-pending non-dynamic cohort member, so this
        // trigger is reachable during teardown.
        if dataflow.should_start_on_barrier_completion(&status) {
            logger
                .log(
                    LogLevel::Info,
                    None,
                    Some("daemon".into()),
                    "startup barrier completed by a node exit, starting dataflow",
                )
                .await;
            dataflow.start(&self.events_tx, &self.clock).await?;
        }

        // node only reaches here if it will not be restarted
        let might_restart = false;

        // The node has already exited. A failed lifecycle publish must be
        // visible, but it must not keep its running-node entry or coordinator
        // status alive forever. CloseOutputs still reports the error to its
        // live caller through its reply path.
        if let Err(err) = self
            .handle_outputs_done(dataflow_id, node_id, might_restart)
            .await
        {
            tracing::error!(%dataflow_id, %node_id, "failed to close outputs after node exit: {err:?}");
        }

        // Capture `restarts_disabled` BEFORE the remove. Combined with
        // `exit_clean` (passed from the caller, set when the exit_status
        // was Success), produces the `clean_stop` flag:
        //   - operator-requested stop: stop_single_node() set
        //     disable_restart, SIGTERM-induced exit is non-zero
        //     (exit_clean=false). restarts_disabled covers it.
        //   - node finished its own work and exited 0: exit_clean=true.
        //   - crash / panic / restart_policy=Never with non-zero exit:
        //     neither bit is set → clean_stop=false → `Failed`, so the
        //     row sticks around and `dora doctor` keeps reporting it.
        //   - finish-straggler escalation (dora-rs/dora#2152): the
        //     watchdog also goes through stop_single_node, so
        //     restarts_disabled is set — but a force-killed straggler is
        //     a node FAILURE and must not auto-expire from `dora node
        //     list` as a clean stop. `finish_escalated` overrides
        //     restarts_disabled (an escalated node that still exited 0
        //     keeps exit_clean=true and stays clean).
        let (should_finish, clean_stop) = {
            let dataflow = self.running.get_mut(&dataflow_id).wrap_err_with(|| {
                format!(
                    "failed to get downstream nodes: no running dataflow with ID `{dataflow_id}`"
                )
            })?;
            let restarts_disabled = dataflow
                .running_nodes
                .get(node_id)
                .map(|n| n.restarts_disabled())
                .unwrap_or(false);
            let finish_escalated = dataflow.finish_escalated.remove(node_id);
            dataflow.all_inputs_closed_at.remove(node_id);
            dataflow.pending_remote_all_inputs_closed.remove(node_id);
            let clean_stop = exit_clean || (restarts_disabled && !finish_escalated);
            dataflow.running_nodes.remove(node_id);
            // Check if all remaining nodes are dynamic (won't send SpawnedNodeResult)
            let should_finish = !dataflow.pending_nodes.local_nodes_pending()
                && dataflow
                    .running_nodes
                    .iter()
                    .all(|(_id, n)| n.node_config.dynamic);
            (should_finish, clean_stop)
        };

        // Tell the coordinator the node is gone so its cached
        // `node_metrics[node_id]` row stops claiming `Running` with the
        // pre-exit PID/CPU/memory snapshot. Without this signal the
        // daemon's metrics-snapshot loop simply omits the dead node and
        // the coordinator's cache stays frozen at the last pre-exit
        // values forever.
        if let Some(sender) = self.coordinator_sender.as_mut() {
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: self.daemon_id.clone(),
                    event: DaemonEvent::NodeStopped {
                        dataflow_id,
                        node_id: node_id.clone(),
                        clean_stop,
                    },
                },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to serialize NodeStopped")?;
            if let Err(err) = sender.send_event(&msg).await {
                tracing::warn!(
                    %dataflow_id, %node_id,
                    "failed to send NodeStopped to coordinator: {err}"
                );
            }
        }

        if should_finish {
            self.finish_dataflow(dataflow_id).await?;
        }

        Ok(())
    }
}

async fn await_remote_close(
    queue: mpsc::Sender<ZenohOutbound>,
    outbound: ZenohOutbound,
    receipt: oneshot::Receiver<eyre::Result<()>>,
) -> eyre::Result<()> {
    enqueue_required_zenoh_outbound(&queue, outbound).await?;
    tokio::time::timeout(ZENOH_REQUIRED_ACK_TIMEOUT, receipt)
        .await
        .map_err(|_| eyre!("timed out waiting for inter-daemon OutputClosed receipt"))?
        .map_err(|_| eyre!("inter-daemon OutputClosed drain task exited without receipt"))??;
    Ok(())
}

pub(crate) fn reload_target<'a>(
    dataflow: &'a RunningDataflow,
    node_id: &NodeId,
) -> eyre::Result<Option<&'a mpsc::Sender<Timestamped<NodeEvent>>>> {
    match dataflow.subscribe_channels.get(node_id) {
        Some(channel) => Ok(Some(channel)),
        None if dataflow.running_nodes.contains_key(node_id) => {
            Err(eyre!("node `{node_id}` has not subscribed for reload"))
        }
        None => Ok(None),
    }
}

async fn remote_receiver_control_publisher(
    zenoh_session: &zenoh::Session,
    dataflow: &mut RunningDataflow,
    output_id: &OutputId,
) -> eyre::Result<Arc<zenoh::pubsub::Publisher<'static>>> {
    let dataflow_id = dataflow.id;
    remote_receiver_publisher(
        zenoh_session,
        &mut dataflow.control_publishers,
        dataflow_id,
        output_id,
        CongestionControl::Block,
    )
    .await
}

async fn remote_receiver_publisher(
    zenoh_session: &zenoh::Session,
    publishers: &mut BTreeMap<OutputId, Arc<zenoh::pubsub::Publisher<'static>>>,
    dataflow_id: DataflowId,
    output_id: &OutputId,
    congestion_control: CongestionControl,
) -> eyre::Result<Arc<zenoh::pubsub::Publisher<'static>>> {
    match publishers.entry(output_id.clone()) {
        std::collections::btree_map::Entry::Occupied(e) => Ok(e.get().clone()),
        std::collections::btree_map::Entry::Vacant(e) => {
            let publish_topic = zenoh_daemon_control_topic(dataflow_id, &output_id.0, &output_id.1);
            tracing::debug!("declaring control publisher on {publish_topic}");
            let publisher = zenoh_session
                .declare_publisher(publish_topic)
                .congestion_control(congestion_control)
                .express(true)
                .priority(Priority::RealTime)
                .await
                .map_err(|err| eyre!(err))
                .context("failed to create zenoh publisher")?;
            let publisher = Arc::new(publisher);
            e.insert(publisher.clone());
            Ok(publisher)
        }
    }
}

async fn remote_output_publish_queue(
    dataflow: &mut RunningDataflow,
    output_id: &OutputId,
) -> eyre::Result<mpsc::Sender<ZenohOutbound>> {
    if let Some(old_drain) = dataflow.retired_remote_output_queues.get_mut(output_id) {
        tokio::time::timeout(ZENOH_REQUIRED_ACK_TIMEOUT, old_drain)
            .await
            .map_err(|_| eyre!("previous zenoh drain for {output_id:?} did not finish"))?
            .map_err(|err| eyre!("previous zenoh drain for {output_id:?} failed: {err}"))?;
        dataflow.retired_remote_output_queues.remove(output_id);
    }
    match dataflow.remote_output_queues.entry(output_id.clone()) {
        std::collections::btree_map::Entry::Occupied(e) => Ok(e.get().clone()),
        std::collections::btree_map::Entry::Vacant(e) => {
            let (tx, mut rx) = mpsc::channel::<ZenohOutbound>(ZENOH_OUTPUT_QUEUE_CAPACITY);
            let logged_output_id = output_id.clone();
            let handle = tokio::spawn(async move {
                while let Some(msg) = rx.recv().await {
                    // Regular Output and its OutputClosed use distinct Drop/Block
                    // publishers on the same Zenoh key, but this per-output FIFO
                    // is the ordering boundary: each publish is awaited before the
                    // next one is submitted, so a close cannot pass earlier data.
                    publish_zenoh_outbound(msg).await;
                }
                tracing::debug!(
                    ?logged_output_id,
                    "per-output zenoh publish drain task exiting"
                );
            });
            dataflow
                .remote_output_queue_handles
                .insert(output_id.clone(), handle);
            e.insert(tx.clone());
            Ok(tx)
        }
    }
}

fn zenoh_outbound(
    dataflow: &RunningDataflow,
    publisher: Arc<zenoh::pubsub::Publisher<'static>>,
    serialized: Vec<u8>,
    timeout: Option<std::time::Duration>,
) -> ZenohOutbound {
    ZenohOutbound {
        publisher,
        payload_len: serialized.len() as u64,
        serialized,
        timeout,
        net_bytes_sent: dataflow.net_bytes_sent.clone(),
        net_messages_sent: dataflow.net_messages_sent.clone(),
        net_publish_failures: dataflow.net_publish_failures.clone(),
        receipt: None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ZenohEnqueueResult {
    Queued,
    Full,
    Closed,
}

fn try_enqueue_zenoh_outbound<T>(tx: &mpsc::Sender<T>, outbound: T) -> ZenohEnqueueResult {
    match tx.try_send(outbound) {
        Ok(()) => ZenohEnqueueResult::Queued,
        Err(mpsc::error::TrySendError::Full(_)) => ZenohEnqueueResult::Full,
        Err(mpsc::error::TrySendError::Closed(_)) => ZenohEnqueueResult::Closed,
    }
}

#[derive(Debug)]
enum ZenohPublishOutcome<E> {
    Published,
    Failed(E),
    TimedOut,
}

async fn await_zenoh_put_with_timeout<F>(
    publish: F,
    timeout: std::time::Duration,
) -> (
    ZenohPublishOutcome<eyre::Report>,
    Option<tokio::task::JoinHandle<eyre::Result<()>>>,
)
where
    F: FnOnce() -> eyre::Result<()> + Send + 'static,
{
    let mut handle = tokio::task::spawn_blocking(publish);
    match tokio::time::timeout(timeout, &mut handle).await {
        Ok(Ok(Ok(()))) => (ZenohPublishOutcome::Published, None),
        Ok(Ok(Err(err))) => (ZenohPublishOutcome::Failed(err), None),
        Ok(Err(err)) => (ZenohPublishOutcome::Failed(eyre!(err)), None),
        Err(_) => (ZenohPublishOutcome::TimedOut, Some(handle)),
    }
}

async fn publish_zenoh_outbound(mut msg: ZenohOutbound) {
    let (result, in_flight) = if let Some(timeout) = msg.timeout {
        let publisher = msg.publisher.clone();
        let serialized = std::mem::take(&mut msg.serialized);
        await_zenoh_put_with_timeout(
            move || publisher.put(serialized).wait().map_err(|err| eyre!(err)),
            timeout,
        )
        .await
    } else {
        let serialized = std::mem::take(&mut msg.serialized);
        match msg.publisher.put(serialized).await {
            Ok(()) => (ZenohPublishOutcome::Published, None),
            Err(err) => (ZenohPublishOutcome::Failed(eyre!(err)), None),
        }
    };
    finish_zenoh_publish(&mut msg, result);
    if let Some(in_flight) = in_flight {
        // The caller has already received a timeout. Zenoh's synchronous
        // wait cannot be cancelled, so keep this output's FIFO behind it.
        let _ = in_flight.await;
    }
}

fn finish_zenoh_publish(msg: &mut ZenohOutbound, result: ZenohPublishOutcome<eyre::Report>) {
    match result {
        ZenohPublishOutcome::Published => {
            msg.net_bytes_sent
                .fetch_add(msg.payload_len, atomic::Ordering::Relaxed);
            msg.net_messages_sent
                .fetch_add(1, atomic::Ordering::Relaxed);
            if let Some(receipt) = msg.receipt.take() {
                let _ = receipt.send(Ok(()));
            }
        }
        ZenohPublishOutcome::Failed(e) => {
            tracing::error!("zenoh publish failed: {e}");
            msg.net_publish_failures
                .fetch_add(1, atomic::Ordering::Relaxed);
            if let Some(receipt) = msg.receipt.take() {
                let _ = receipt.send(Err(e));
            }
        }
        ZenohPublishOutcome::TimedOut => {
            tracing::error!("zenoh publish timed out after {:?}", msg.timeout.unwrap());
            msg.net_publish_failures
                .fetch_add(1, atomic::Ordering::Relaxed);
            if let Some(receipt) = msg.receipt.take() {
                let _ = receipt.send(Err(eyre!("zenoh publish timed out")));
            }
        }
    }
}

fn handle_publish_enqueue_result(
    dataflow: &RunningDataflow,
    result: ZenohEnqueueResult,
    event_kind: &'static str,
) {
    match result {
        ZenohEnqueueResult::Queued => {}
        ZenohEnqueueResult::Full => {
            dataflow
                .net_publish_failures
                .fetch_add(1, atomic::Ordering::Relaxed);
            tracing::warn!(
                "zenoh publish channel full ({ZENOH_OUTPUT_QUEUE_CAPACITY}); dropping {event_kind}"
            );
        }
        ZenohEnqueueResult::Closed => {
            dataflow
                .net_publish_failures
                .fetch_add(1, atomic::Ordering::Relaxed);
            tracing::error!("zenoh drain task is gone; dropping {event_kind}");
        }
    }
}

async fn enqueue_required_zenoh_outbound(
    tx: &mpsc::Sender<ZenohOutbound>,
    outbound: ZenohOutbound,
) -> eyre::Result<()> {
    let failures = outbound.net_publish_failures.clone();
    let queued = tokio::time::timeout(
        ZENOH_REQUIRED_ACK_TIMEOUT,
        send_required_to_queue(tx, outbound),
    )
    .await
    .map_err(|_| {
        failures.fetch_add(1, atomic::Ordering::Relaxed);
        eyre!("timed out waiting to queue inter-daemon OutputClosed")
    })?;
    match queued {
        Ok(()) => Ok(()),
        Err(mpsc::error::SendError(outbound)) => {
            outbound
                .net_publish_failures
                .fetch_add(1, atomic::Ordering::Relaxed);
            Err(eyre!(
                "zenoh drain task is gone; inter-daemon OutputClosed not queued"
            ))
        }
    }
}

async fn send_required_to_queue<T>(
    tx: &mpsc::Sender<T>,
    outbound: T,
) -> Result<(), mpsc::error::SendError<T>> {
    tx.send(outbound).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{LogDestination, ZenohBind};
    use dora_core::topics::LOCALHOST;
    use dora_message::{common::DaemonId, descriptor::Descriptor};

    fn test_dataflow() -> RunningDataflow {
        RunningDataflow::new(Uuid::nil(), DaemonId::new(None), Descriptor::new(vec![]))
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn output_closed_uses_block_publisher_but_shares_output_fifo_with_regular_output() {
        let clock = Arc::new(HLC::default());
        let (mut daemon, _events_rx) = Daemon::build_daemon(
            Some("daemon".to_string()),
            None,
            DaemonId::new(Some("daemon".to_string())),
            None,
            clock,
            None,
            BTreeMap::new(),
            LogDestination::Tracing,
            None,
            Vec::new(),
            None,
            Vec::new(),
            ZenohBind::Derived(LOCALHOST),
            true,
            false,
        )
        .await
        .unwrap();

        let dataflow_id = Uuid::new_v4();
        let node_id = NodeId::from("source".to_string());
        let output_id = DataId::from("value".to_string());
        let output = OutputId(node_id.clone(), output_id.clone());
        let (publish_tx, mut publish_rx) = mpsc::channel(2);
        let mut dataflow = test_dataflow();
        dataflow.open_external_mappings.insert(output.clone());
        dataflow
            .remote_output_queues
            .insert(output.clone(), publish_tx);
        daemon.running.insert(dataflow_id, dataflow);
        let received = tokio::spawn(async move {
            let data = publish_rx.recv().await.unwrap();
            let mut close = publish_rx.recv().await.unwrap();
            close.receipt.take().unwrap().send(Ok(())).unwrap();
            (data, close)
        });

        daemon
            .try_send_to_remote_receivers(dataflow_id, &output, b"data".to_vec())
            .await
            .unwrap();
        daemon
            .send_output_closed_events(dataflow_id, node_id, vec![output_id])
            .await
            .unwrap();

        let (data, close) = received.await.unwrap();
        assert_eq!(data.publisher.congestion_control(), CongestionControl::Drop);
        assert_eq!(
            close.publisher.congestion_control(),
            CongestionControl::Block
        );
        assert_eq!(data.timeout, None);
        assert_eq!(close.timeout, Some(ZENOH_PUBLISH_TIMEOUT));
        assert_eq!(data.serialized, b"data");
        assert_ne!(close.serialized, b"data");

        let dataflow = daemon.running.get(&dataflow_id).unwrap();
        let close_publisher = dataflow.control_publishers.get(&output).unwrap();
        assert_eq!(
            close_publisher.congestion_control(),
            CongestionControl::Block
        );
        assert!(
            !Arc::ptr_eq(&data.publisher, close_publisher),
            "regular output and OutputClosed must not share one blocking publisher"
        );
        let (receipt_tx, receipt_rx) = oneshot::channel();
        let mut failed = zenoh_outbound(
            dataflow,
            close_publisher.clone(),
            b"failed close".to_vec(),
            Some(ZENOH_PUBLISH_TIMEOUT),
        );
        failed.receipt = Some(receipt_tx);
        finish_zenoh_publish(
            &mut failed,
            ZenohPublishOutcome::Failed(eyre!("fake put failure")),
        );
        assert!(
            receipt_rx
                .await
                .unwrap()
                .unwrap_err()
                .to_string()
                .contains("fake put failure")
        );

        let (closed_tx, closed_rx) = mpsc::channel(1);
        drop(closed_rx);
        daemon
            .running
            .get_mut(&dataflow_id)
            .unwrap()
            .remote_output_queues
            .insert(output.clone(), closed_tx);
        let err = daemon
            .send_to_remote_receivers(dataflow_id, &output, b"closed".to_vec())
            .await
            .unwrap_err();
        assert!(err.to_string().contains("not queued"));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn output_closed_waits_for_multiple_receipts_concurrently() {
        let (mut daemon, _events_rx) = Daemon::build_daemon(
            Some("daemon".to_string()),
            None,
            DaemonId::new(Some("daemon".to_string())),
            None,
            Arc::new(HLC::default()),
            None,
            BTreeMap::new(),
            LogDestination::Tracing,
            None,
            Vec::new(),
            None,
            Vec::new(),
            ZenohBind::Derived(LOCALHOST),
            true,
            false,
        )
        .await
        .unwrap();
        let dataflow_id = Uuid::new_v4();
        let node_id = NodeId::from("source".to_string());
        let outputs = [DataId::from("a".to_string()), DataId::from("b".to_string())];
        let mut dataflow = test_dataflow();
        let mut receivers = Vec::new();
        for output in &outputs {
            let key = OutputId(node_id.clone(), output.clone());
            let (tx, rx) = mpsc::channel(1);
            dataflow.open_external_mappings.insert(key.clone());
            dataflow.remote_output_queues.insert(key, tx);
            receivers.push(rx);
        }
        daemon.running.insert(dataflow_id, dataflow);
        let close = tokio::spawn(async move {
            daemon
                .send_output_closed_events(dataflow_id, node_id, outputs.to_vec())
                .await
        });
        let mut receipts = Vec::new();
        for mut rx in receivers {
            let mut message = tokio::time::timeout(std::time::Duration::from_secs(1), rx.recv())
                .await
                .expect("all closes must be queued before any receipt is released")
                .expect("close message");
            receipts.push(message.receipt.take().unwrap());
        }
        assert!(!close.is_finished());
        for receipt in receipts {
            receipt.send(Ok(())).unwrap();
        }
        close.await.unwrap().unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn later_data_cannot_overtake_a_close_waiting_for_queue_room() {
        let (mut daemon, _events_rx) = Daemon::build_daemon(
            Some("daemon".to_string()),
            None,
            DaemonId::new(Some("daemon".to_string())),
            None,
            Arc::new(HLC::default()),
            None,
            BTreeMap::new(),
            LogDestination::Tracing,
            None,
            Vec::new(),
            None,
            Vec::new(),
            ZenohBind::Derived(LOCALHOST),
            true,
            false,
        )
        .await
        .unwrap();
        let dataflow_id = Uuid::new_v4();
        let node_id = NodeId::from("source".to_string());
        let output_id = DataId::from("value".to_string());
        let output = OutputId(node_id.clone(), output_id.clone());
        let (tx, mut rx) = mpsc::channel(1);
        let mut dataflow = test_dataflow();
        dataflow.open_external_mappings.insert(output.clone());
        dataflow.remote_output_queues.insert(output.clone(), tx);
        daemon.running.insert(dataflow_id, dataflow);
        daemon
            .try_send_to_remote_receivers(dataflow_id, &output, b"before".to_vec())
            .await
            .unwrap();

        let daemon = Arc::new(tokio::sync::Mutex::new(daemon));
        let (started_tx, started_rx) = oneshot::channel();
        let close_daemon = daemon.clone();
        let close = tokio::spawn(async move {
            let mut daemon = close_daemon.lock().await;
            let _ = started_tx.send(());
            daemon
                .send_output_closed_events(dataflow_id, node_id, vec![output_id])
                .await
        });
        started_rx.await.unwrap();
        let data_daemon = daemon.clone();
        let data_output = output.clone();
        let later_data = tokio::spawn(async move {
            data_daemon
                .lock()
                .await
                .try_send_to_remote_receivers(dataflow_id, &data_output, b"after".to_vec())
                .await
        });
        assert_eq!(rx.recv().await.unwrap().serialized, b"before");
        let mut close_msg = tokio::time::timeout(std::time::Duration::from_secs(2), rx.recv())
            .await
            .unwrap()
            .unwrap();
        assert!(!close.is_finished());
        assert!(!later_data.is_finished());
        close_msg.receipt.take().unwrap().send(Ok(())).unwrap();
        close.await.unwrap().unwrap();
        later_data.await.unwrap().unwrap();
        assert_eq!(rx.recv().await.unwrap().serialized, b"after");
    }

    #[tokio::test(start_paused = true)]
    async fn zenoh_publish_wait_is_bounded() {
        let publish = || {
            std::thread::sleep(std::time::Duration::from_millis(100));
            Ok(())
        };
        let handle = tokio::spawn(await_zenoh_put_with_timeout(publish, ZENOH_PUBLISH_TIMEOUT));

        tokio::task::yield_now().await;
        assert!(
            !handle.is_finished(),
            "the helper must not finish before the timeout elapses"
        );

        tokio::time::advance(ZENOH_PUBLISH_TIMEOUT).await;
        assert!(matches!(
            handle.await.unwrap().0,
            ZenohPublishOutcome::TimedOut
        ));
    }

    #[tokio::test]
    async fn blocking_publisher_timeout_does_not_block_runtime() {
        let result = await_zenoh_put_with_timeout(
            || {
                std::thread::sleep(std::time::Duration::from_millis(100));
                Ok(())
            },
            std::time::Duration::from_millis(10),
        )
        .await;
        assert!(matches!(result.0, ZenohPublishOutcome::TimedOut));
    }

    #[test]
    fn required_publish_waits_fit_within_heartbeat_watchdog() {
        assert!(
            ZENOH_REQUIRED_ACK_TIMEOUT * 3 < crate::COORDINATOR_HEARTBEAT_TIMEOUT,
            "old-drain, queue, and receipt waits must not individually exhaust the watchdog"
        );
    }

    #[tokio::test]
    async fn full_required_queue_preserves_close_before_later_data() {
        let (tx, mut rx) = mpsc::channel(1);
        tx.send(1).await.unwrap();
        let close_tx = tx.clone();
        let close =
            tokio::spawn(async move { send_required_to_queue(&close_tx, 2).await.unwrap() });
        tokio::task::yield_now().await;
        assert!(!close.is_finished());
        assert_eq!(rx.recv().await, Some(1));
        close.await.unwrap();
        assert!(tx.try_send(3).is_err());
        assert!(
            tokio::time::timeout(
                std::time::Duration::from_millis(10),
                send_required_to_queue(&tx, 4),
            )
            .await
            .is_err()
        );
        assert_eq!(rx.recv().await, Some(2));
        tx.try_send(3).unwrap();
        assert_eq!(rx.recv().await, Some(3));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn retired_output_drain_finishes_before_new_queue_is_created() {
        let mut dataflow = test_dataflow();
        let output = OutputId(
            NodeId::from("source".to_string()),
            DataId::from("value".to_string()),
        );
        let (daemon, _events_rx) = Daemon::build_daemon(
            Some("daemon".to_string()),
            None,
            DaemonId::new(Some("daemon".to_string())),
            None,
            Arc::new(HLC::default()),
            None,
            BTreeMap::new(),
            LogDestination::Tracing,
            None,
            Vec::new(),
            None,
            Vec::new(),
            ZenohBind::Derived(LOCALHOST),
            true,
            false,
        )
        .await
        .unwrap();
        let publisher = daemon
            .zenoh_session
            .declare_publisher("test/old-output")
            .await
            .unwrap();
        let (old_tx, mut old_rx) = mpsc::channel(1);
        old_tx
            .try_send(zenoh_outbound(
                &dataflow,
                Arc::new(publisher),
                b"old buffered data".to_vec(),
                None,
            ))
            .unwrap();
        let (release_tx, release_rx) = oneshot::channel();
        let (consumed_tx, consumed_rx) = oneshot::channel();
        let (published_tx, published_rx) = oneshot::channel();
        let old_handle = tokio::spawn(async move {
            let _ = release_rx.await;
            let old = old_rx
                .recv()
                .await
                .expect("old queue must have buffered data");
            let _ = consumed_tx.send(old.serialized);
            let _ = published_rx.await;
            assert!(old_rx.recv().await.is_none());
        });
        dataflow.remote_output_queues.insert(output.clone(), old_tx);
        dataflow
            .remote_output_queue_handles
            .insert(output.clone(), old_handle);
        dataflow.forget_node_bookkeeping(&output.0);
        assert!(!dataflow.remote_output_queues.contains_key(&output));
        let mut create = Box::pin(remote_output_publish_queue(&mut dataflow, &output));
        assert!(
            tokio::time::timeout(std::time::Duration::from_millis(10), &mut create)
                .await
                .is_err()
        );
        release_tx.send(()).unwrap();
        assert_eq!(consumed_rx.await.unwrap(), b"old buffered data");
        assert!(
            tokio::time::timeout(std::time::Duration::from_millis(10), &mut create)
                .await
                .is_err(),
            "taking the old message is not enough; its publish must finish"
        );
        published_tx.send(()).unwrap();
        let _new_queue = create.await.unwrap();
    }
}
