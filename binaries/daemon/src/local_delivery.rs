//! Delivering a node's output to the local subscribers of that output, and
//! closing or breaking inputs when a producer goes away.

use crate::{
    CONTROL_EVENT_HEADROOM, FaultToleranceStats, InputDeadline, NODE_EVENT_CHANNEL_CAPACITY,
    OutputId, RunningDataflow, runtime_node_inputs, send_with_timestamp,
};
use aligned_vec::{AVec, ConstAlign};
use dora_core::{
    config::{DataId, Input, NodeId},
    descriptor::{CoreNodeKind, ResolvedNode},
    uhlc::HLC,
};
use dora_message::{
    common::DataMessage, daemon_to_node::NodeEvent, metadata, node_to_daemon::Timestamped,
};
use eyre::Result;
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, atomic},
    time::Instant,
};
use tokio::sync::mpsc;

pub(crate) fn note_output_sent_to_local_receivers(
    node_id: NodeId,
    output_id: DataId,
    dataflow: &mut RunningDataflow,
    clock: &HLC,
    ft_stats: Option<&FaultToleranceStats>,
) {
    // Both side effects below are gated on a non-empty `input_deadlines`
    // (deadline refresh) or `broken_inputs` (circuit-breaker recovery). When
    // neither feature is configured — the common case — the whole loop is a
    // no-op, so skip it entirely rather than paying a clock read plus a
    // `subscribe_channels` lookup per receiver on every `OutputSent`.
    if dataflow.input_deadlines.is_empty() && dataflow.broken_inputs.is_empty() {
        return;
    }

    let empty_set = BTreeSet::new();
    let output_id = OutputId(node_id, output_id);
    let local_receivers = dataflow.mappings.get(&output_id).unwrap_or(&empty_set);
    let now = Instant::now();

    for (receiver_id, input_id) in local_receivers {
        // Refresh the input deadline only when the receiver is actually
        // keeping up. Since #1787 the data payload is published directly
        // over zenoh (not routed through the daemon), so the bare
        // `OutputSent` notification is *not* a delivery confirmation — the
        // node-side zenoh callback drops the input with `try_send` when its
        // event channel is full. Treating `OutputSent` as delivery made
        // `input_timeout` deadlines never fire for a slow consumer (#2021).
        //
        // We use the receiver's daemon-side channel headroom as a
        // backpressure proxy (the same signal `send_output_to_local_receivers`
        // gates on): a node that has fallen far enough behind to saturate its
        // channel is also dropping the zenoh payloads, so its deadline must be
        // allowed to expire. A node that is keeping up still gets refreshed.
        //
        // A *missing* channel means the receiver has no daemon-side event
        // stream at all — e.g. it dropped its stream (`EventStreamDropped`)
        // or has not subscribed yet. Such a receiver is not draining inputs
        // either, so it must NOT count as keeping up; mirror
        // `send_output_to_local_receivers`, which does nothing without a
        // channel.
        let receiver_keeping_up = dataflow
            .subscribe_channels
            .get(receiver_id)
            .is_some_and(|channel| channel.capacity() >= CONTROL_EVENT_HEADROOM);
        // Looking up these maps requires cloning the `(NodeId, DataId)` key (the
        // tuple key type can't borrow). Both maps are empty unless input
        // deadlines or circuit breakers are configured, so skip the per-message
        // key clone + hash in the common case — mirroring
        // `send_output_to_local_receivers`.
        if receiver_keeping_up
            && !dataflow.input_deadlines.is_empty()
            && let Some(deadline) = dataflow
                .input_deadlines
                .get_mut(&(receiver_id.clone(), input_id.clone()))
        {
            deadline.last_received = Some(now);
        }

        // Circuit-breaker recovery must be gated on the same backpressure
        // signal as the deadline refresh above. A bare `OutputSent` is not a
        // delivery confirmation (see the comment above), so re-opening a broken
        // input while the receiver is still saturated only makes it flap
        // `broken ↔ recovered` every `input_timeout` without any data actually
        // getting through (#2627). Leave the `broken_inputs` entry in place
        // until the receiver drains enough to keep up — matching the recovery
        // in `send_output_to_local_receivers`, which only fires inside the
        // successful `try_send` arm.
        if !receiver_keeping_up {
            continue;
        }

        // `broken_inputs` is empty unless circuit breakers are configured, so
        // skip the per-message `(NodeId, DataId)` key clone + hash + remove in
        // the common case — mirroring `send_output_to_local_receivers`.
        if dataflow.broken_inputs.is_empty() {
            continue;
        }
        let Some(timeout) = dataflow
            .broken_inputs
            .remove(&(receiver_id.clone(), input_id.clone()))
        else {
            continue;
        };

        tracing::info!(
            "input `{receiver_id}/{input_id}` recovered, \
             re-opening (circuit breaker reset)",
        );
        if let Some(stats) = ft_stats {
            stats
                .circuit_breaker_recoveries
                .fetch_add(1, atomic::Ordering::Relaxed);
        }
        dataflow
            .open_inputs
            .entry(receiver_id.clone())
            .or_default()
            .insert(input_id.clone());
        dataflow.input_deadlines.insert(
            (receiver_id.clone(), input_id.clone()),
            InputDeadline {
                timeout,
                last_received: Some(now),
            },
        );

        let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
            continue;
        };
        match send_with_timestamp(
            channel,
            NodeEvent::InputRecovered {
                id: input_id.clone(),
            },
            clock,
        ) {
            Ok(true) => {
                dataflow.inc_pending(receiver_id);
            }
            Ok(false) => { /* event dropped (channel full) */ }
            Err(_) => {
                tracing::warn!("failed to send InputRecovered for `{receiver_id}/{input_id}`");
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn send_output_to_local_receivers(
    output_id: &OutputId,
    dataflow: &mut RunningDataflow,
    metadata: &metadata::Metadata,
    data: Option<DataMessage>,
    clock: &HLC,
    ft_stats: Option<&FaultToleranceStats>,
    need_data_bytes: bool,
) -> Result<Option<AVec<u8, ConstAlign<128>>>, eyre::ErrReport> {
    let timestamp = metadata.timestamp();
    let empty_set = BTreeSet::new();
    let local_receivers = dataflow.mappings.get(output_id).unwrap_or(&empty_set);
    let data = data.map(Arc::new);
    let mut closed = Vec::new();
    // Clone the metadata into an `Arc` lazily, on the first actual delivery.
    // Fan-out clones are then O(1) atomic ref bumps instead of O(payload_size)
    // memcpy. For a pure-remote output topology `local_receivers` is empty (all
    // subscribers live on other daemons), so this deep clone (a `BTreeMap` of
    // owned `Parameter`s) is skipped entirely rather than built and dropped on
    // every such message.
    let mut metadata_arc = None;
    for (receiver_id, input_id) in local_receivers {
        if let Some(channel) = dataflow.subscribe_channels.get(receiver_id) {
            // Reserve headroom for control events (Stop, InputClosed, etc.)
            if channel.capacity() < CONTROL_EVENT_HEADROOM {
                tracing::warn!(
                    node = %receiver_id,
                    "event channel low on capacity ({}/{}), dropping data to preserve control headroom",
                    channel.capacity(),
                    NODE_EVENT_CHANNEL_CAPACITY,
                );
                continue;
            }
            let item = NodeEvent::Input {
                id: input_id.clone(),
                metadata: metadata_arc
                    .get_or_insert_with(|| Arc::new(metadata.clone()))
                    .clone(),
                data: data.clone(),
            };
            match channel.try_send(Timestamped {
                inner: item,
                timestamp,
            }) {
                Ok(()) => {
                    dataflow.inc_pending(receiver_id);
                    // Looking up these maps requires cloning the `(NodeId, DataId)`
                    // key (the tuple key type can't borrow). Both maps are empty
                    // unless input deadlines or circuit breakers are configured, so
                    // skip the per-message key clone + hash in the common case.
                    if !dataflow.input_deadlines.is_empty()
                        && let Some(deadline) = dataflow
                            .input_deadlines
                            .get_mut(&(receiver_id.clone(), input_id.clone()))
                    {
                        deadline.last_received = Some(Instant::now());
                    }
                    // Circuit breaker recovery: re-open broken input
                    if !dataflow.broken_inputs.is_empty()
                        && let Some(timeout) = dataflow
                            .broken_inputs
                            .remove(&(receiver_id.clone(), input_id.clone()))
                    {
                        tracing::info!(
                            "input `{receiver_id}/{input_id}` recovered, \
                             re-opening (circuit breaker reset)",
                        );
                        if let Some(stats) = ft_stats {
                            stats
                                .circuit_breaker_recoveries
                                .fetch_add(1, atomic::Ordering::Relaxed);
                        }
                        dataflow
                            .open_inputs
                            .entry(receiver_id.clone())
                            .or_default()
                            .insert(input_id.clone());
                        dataflow.input_deadlines.insert(
                            (receiver_id.clone(), input_id.clone()),
                            InputDeadline {
                                timeout,
                                // A message just arrived — arm immediately.
                                last_received: Some(Instant::now()),
                            },
                        );
                        match send_with_timestamp(
                            channel,
                            NodeEvent::InputRecovered {
                                id: input_id.clone(),
                            },
                            clock,
                        ) {
                            Ok(true) => {
                                dataflow.inc_pending(receiver_id);
                            }
                            Ok(false) => { /* event dropped (channel full) */ }
                            Err(_) => {
                                tracing::warn!(
                                    "failed to send InputRecovered for `{receiver_id}/{input_id}`"
                                );
                            }
                        }
                    }
                }
                Err(mpsc::error::TrySendError::Closed(_)) => {
                    closed.push(receiver_id);
                }
                Err(mpsc::error::TrySendError::Full(_)) => {
                    tracing::warn!(
                        node = %receiver_id,
                        "event channel full (capacity {}), dropping message (node is too slow)",
                        NODE_EVENT_CHANNEL_CAPACITY,
                    );
                }
            }
        } else if dataflow.running_nodes.contains_key(receiver_id) {
            // The receiver is registered in `mappings` AND still a live node,
            // but has no daemon event stream: its channel was dropped (crash
            // that never re-subscribed, `EventStreamDropped`, or a closed
            // listener). This is the silent-routing-loss mode of
            // dora-rs/dora#3201 — the producer's send still "succeeds" but the
            // consumer receives nothing, indefinitely. Make it visible, once
            // per edge; the marker is cleared on (re)subscribe so an edge that
            // drops its stream again after reconnecting gets a fresh warning.
            if dataflow
                .missing_channel_warned
                .insert((receiver_id.clone(), input_id.clone()))
            {
                tracing::warn!(
                    receiver = %receiver_id,
                    input = %input_id,
                    output = %output_id.1,
                    "dropping `{}/{}` to `{receiver_id}`: node has no daemon \
                     event stream (it may still be starting up, restarting, or \
                     failed to re-subscribe) — the edge stays registered in \
                     the routing table while its channel is gone",
                    output_id.0,
                    output_id.1,
                );
            } else {
                tracing::debug!(
                    receiver = %receiver_id,
                    input = %input_id,
                    output = %output_id.1,
                    "dropping `{}/{}` to `{receiver_id}`: no daemon event \
                     stream (warning already emitted for this edge)",
                    output_id.0,
                    output_id.1,
                );
            }
        } else {
            // The receiver's node has already exited or been removed
            // (`running_nodes` has no entry): a consumer that finished or was
            // stopped before the dataflow tore down. Its receiver-edge mapping
            // outlives the node until the dataflow finishes (lib.rs:
            // handle_node_stop_inner), so an upstream that keeps sending still
            // reaches here — expected, not a restart failure, so debug only.
            tracing::debug!(
                receiver = %receiver_id,
                input = %input_id,
                output = %output_id.1,
                "dropping `{}/{}` to `{receiver_id}`: node has exited (no \
                 daemon event stream, no running node)",
                output_id.0,
                output_id.1,
            );
        }
    }
    for id in closed {
        dataflow.subscribe_channels.remove(id);
    }
    let data_bytes = if need_data_bytes {
        // If no local receiver kept a reference (pure remote topology), the
        // Arc is uniquely owned and the payload can be moved out instead of
        // copied.
        data.map(|arc| match Arc::try_unwrap(arc) {
            Ok(DataMessage::Vec(v)) => v,
            Err(arc) => {
                let DataMessage::Vec(v) = arc.as_ref();
                v.clone()
            }
        })
    } else {
        None
    };
    Ok(data_bytes)
}

/// Refuses a node entering a running dataflow (`dora node add`/`replace`)
/// whose `queue_policy: backpressure` input cannot be honored because its
/// producer already runs here with the output on the direct zenoh path — see
/// `RunningDataflow::unpinnable_backpressure_input` for why nothing can re-pin
/// it (dora-rs/dora#3428).
pub(crate) fn reject_unpinnable_backpressure_inputs(
    dataflow: &RunningDataflow,
    node_id: &NodeId,
    inputs: &BTreeMap<DataId, Input>,
) -> eyre::Result<()> {
    if let Some((input_id, OutputId(source, output))) =
        dataflow.unpinnable_backpressure_input(node_id, inputs)
    {
        eyre::bail!(
            "input `{input_id}` declares `queue_policy: backpressure`, but its producer \
             `{source}` is already running with output `{output}` on the direct zenoh path, \
             which cannot honor that policy (a producer learns its routing only when it \
             starts); start or `dora node replace` the producer after this consumer is in the \
             dataflow, or use `queue_policy: drop_oldest`"
        );
    }
    Ok(())
}

pub(crate) fn node_inputs(node: &ResolvedNode) -> BTreeMap<DataId, Input> {
    match &node.kind {
        CoreNodeKind::Custom(n) => n.run_config.inputs.clone(),
        CoreNodeKind::Runtime(n) => runtime_node_inputs(n),
    }
}

pub(crate) fn close_input(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    input_id: &DataId,
    clock: &HLC,
) -> eyre::Result<()> {
    // Clean up broken state if this input was circuit-broken
    let was_broken = dataflow
        .broken_inputs
        .remove(&(receiver_id.clone(), input_id.clone()))
        .is_some();

    // Drop any armed deadline for this input. A closed input can never
    // meaningfully time out, and leaving the entry behind lets
    // `check_input_timeouts` fire on it later, insert a `broken_inputs`
    // record, and then no-op in `break_input` (the input is already gone from
    // `open_inputs`) — orphaning that record so `has_broken_input` stays true
    // forever and the drained node is never sent `AllInputsClosed` (#2968).
    dataflow
        .input_deadlines
        .remove(&(receiver_id.clone(), input_id.clone()));

    let was_open = dataflow
        .open_inputs
        .get_mut(receiver_id)
        .map(|inputs| inputs.remove(input_id))
        .unwrap_or(false);

    if !was_open && !was_broken {
        return Ok(());
    }

    let mut result = Ok(());
    if was_open && let Err(err) = send_input_closed_strict(dataflow, receiver_id, input_id, clock) {
        result = Err(err);
    }
    if let Err(err) = signal_all_inputs_closed_if_drained(dataflow, receiver_id, clock) {
        if result.is_ok() {
            result = Err(err);
        } else {
            tracing::warn!("failed to signal drained input `{receiver_id}/{input_id}`: {err:?}");
        }
    }

    result
}

pub(crate) fn close_inputs_best_effort<I>(
    dataflow: &mut RunningDataflow,
    inputs: I,
    clock: &HLC,
    context: &'static str,
) where
    I: IntoIterator<Item = (NodeId, DataId)>,
{
    for (receiver_id, input_id) in inputs {
        if let Err(err) = close_input(dataflow, &receiver_id, &input_id, clock) {
            tracing::warn!(%receiver_id, %input_id, "{context}: {err:?}");
        }
    }
}

fn send_input_closed_strict(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    input_id: &DataId,
    clock: &HLC,
) -> eyre::Result<()> {
    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
        return Ok(());
    };
    match send_with_timestamp(
        channel,
        NodeEvent::InputClosed {
            id: input_id.clone(),
        },
        clock,
    ) {
        Ok(true) => {
            dataflow.inc_pending(receiver_id);
            Ok(())
        }
        Ok(false) => Err(eyre::eyre!("node `{receiver_id}` channel full")),
        Err(_) => {
            dataflow.subscribe_channels.remove(receiver_id);
            Err(eyre::eyre!("node `{receiver_id}` channel closed"))
        }
    }
}

/// If `receiver_id` has finished, disable its restart policy and notify it
/// that all inputs are closed.
///
/// "Finished" is [`RunningDataflow::should_signal_all_inputs_closed`], which
/// under `--exit-when-nodes-finish` means its data inputs have closed even
/// while a timer keeps ticking.
///
/// Shared drain-completion tail of [`close_input`] and [`break_input`]; a
/// no-op if the node still has open/broken inputs. Restart bookkeeping is
/// independent from the node event channel; only the `AllInputsClosed` send
/// requires a live channel.
fn signal_all_inputs_closed_if_drained(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    clock: &HLC,
) -> eyre::Result<()> {
    // As at the subscribe site: either "nothing left open" (pre-existing
    // behavior, and true for a source) or "drained" (the node we are about
    // to tell to finish) disables restart.
    let should_disable_restart = (dataflow.open_inputs(receiver_id).is_empty()
        || dataflow.is_drained(receiver_id))
        && !dataflow.has_broken_input(receiver_id);
    if should_disable_restart && let Some(node) = dataflow.running_nodes.get_mut(receiver_id) {
        node.disable_restart();
    }

    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
        return Ok(());
    };
    if dataflow.is_finished(receiver_id)
        && match send_with_timestamp(channel, NodeEvent::AllInputsClosed, clock) {
            Ok(true) => true,
            Ok(false) => return Err(eyre::eyre!("node `{receiver_id}` channel full")),
            Err(_) => {
                dataflow.subscribe_channels.remove(receiver_id);
                return Err(eyre::eyre!("node `{receiver_id}` channel closed"));
            }
        }
    {
        dataflow.inc_pending(receiver_id);
        dataflow
            .all_inputs_closed_at
            .insert(receiver_id.clone(), Instant::now());
    }
    Ok(())
}

/// Circuit-breaker version of close_input: closes the input but keeps it recoverable.
/// The input is moved to `broken_inputs` before calling this function.
pub(crate) fn break_input(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    input_id: &DataId,
    clock: &HLC,
) {
    if let Some(open_inputs) = dataflow.open_inputs.get_mut(receiver_id)
        && !open_inputs.remove(input_id)
    {
        // The input already left `open_inputs` (e.g. it was closed before this
        // timeout fired). The caller inserted a `broken_inputs` record before
        // calling us; a no-op break would orphan it, permanently pinning
        // `has_broken_input` true. Roll that insert back so we don't leave the
        // node undrainable (#2968).
        dataflow
            .broken_inputs
            .remove(&(receiver_id.clone(), input_id.clone()));
        return;
    }
    if let Err(err) = send_input_closed_strict(dataflow, receiver_id, input_id, clock) {
        tracing::warn!("failed to break input `{receiver_id}/{input_id}`: {err:?}");
    }

    if let Err(err) = signal_all_inputs_closed_if_drained(dataflow, receiver_id, clock) {
        tracing::warn!("failed to signal drained input `{receiver_id}/{input_id}`: {err:?}");
    }
}
