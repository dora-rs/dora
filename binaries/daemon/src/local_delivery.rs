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
    sync::{
        Arc, Mutex, PoisonError, atomic,
        atomic::{AtomicBool, AtomicU64},
    },
    time::Instant,
};
use tokio::sync::mpsc;

/// A delivery the daemon loop could not complete without dropping, on an
/// edge whose input declares `queue_policy: backpressure`.
///
/// The receiver's event channel is full (or below its control-event
/// headroom), and the daemon loop must never wait for it — every node's
/// traffic funnels through that loop. So the event is handed back to the
/// *producer's* listener task (`SendOut::deferred_reply`), which waits for
/// room on `drained`, sends it, and holds the producer's next request until
/// it is in. Only that producer's `send_output` stalls, which is what
/// backpressure means; the drop-with-a-warning that used to happen here was
/// the last silent loss on the daemon path (dora-rs/dora#3397, #3439). The
/// producer's listener asks for this only on outputs that have a
/// backpressure consumer (`output_routing::backpressured_outputs`).
#[derive(Debug)]
pub(crate) struct DeferredDelivery {
    pub receiver: NodeId,
    /// The receiver's input the event is for; checked against
    /// `DrainSignal::closed_inputs` before the event is sent.
    pub input: DataId,
    pub channel: mpsc::Sender<Timestamped<NodeEvent>>,
    /// The receiver's pending-message counter, bumped once the event is in.
    pub pending: Option<Arc<AtomicU64>>,
    /// The receiver's side of the hold.
    pub drained: Arc<DrainSignal>,
    pub event: Timestamped<NodeEvent>,
}

/// What a producer held for a full backpressure receiver shares with that
/// receiver's listener (`RunningDataflow::drain_signals`). One per listener,
/// so a restarted node starts with a fresh one.
#[derive(Debug, Default)]
pub(crate) struct DrainSignal {
    /// Fires when the receiver's listener takes an event out of its channel.
    pub notify: tokio::sync::Notify,
    /// Set when a held delivery to this receiver ran into the stall limit:
    /// the receiver is wedged, or blocked on its own producer in a
    /// backpressure cycle. While set, deliveries to its full channel are
    /// dropped and counted instead of holding their producer for another
    /// stall limit each — once is enough to know (dora-rs/dora#3601).
    /// Cleared once the receiver has drained its channel back to the
    /// headroom a held delivery waits for.
    pub gave_up: AtomicBool,
    /// Set when the node dropped its event stream deliberately, so a held
    /// delivery that finds its channel closed afterwards is not a loss.
    pub stream_dropped: AtomicBool,
    /// Inputs of this receiver the daemon loop has closed for good. A held
    /// delivery checks it and sends under the same lock, and `close_input`
    /// adds to it before sending `InputClosed`, so a held message either
    /// lands before the close or not at all: its producer crashing while it
    /// is held must not put an `Input` behind that input's `InputClosed`
    /// (dora-rs/dora#3619).
    pub closed_inputs: Mutex<BTreeSet<DataId>>,
}

impl DrainSignal {
    /// Marks `input` closed for held deliveries, and wakes them so they
    /// give up on it now rather than after waiting for room.
    pub fn close_input(&self, input: &DataId) {
        self.closed_inputs
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(input.clone());
        self.notify.notify_waiters();
    }

    /// Undoes [`Self::close_input`] for an input a reload maps again.
    pub fn reopen_input(&self, input: &DataId) {
        self.closed_inputs
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .remove(input);
    }
}

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

/// Offers `event` to `receiver_id`'s event channel, keeping the
/// control-event headroom: data never takes the last
/// `CONTROL_EVENT_HEADROOM` slots. Returns whether the receiver has it —
/// delivered, or deferred to the producer's listener (see
/// [`DeferredDelivery`]), which counts the same for the caller's bookkeeping:
/// the producer is alive and produced, which is what the input deadline and
/// the circuit breaker watch. A closed channel is noted in `closed`; a full
/// one with no way to defer is a counted drop.
#[allow(clippy::too_many_arguments)]
fn offer_event<'a>(
    dataflow: &RunningDataflow,
    receiver_id: &'a NodeId,
    input_id: &DataId,
    output_id: &OutputId,
    channel: &mpsc::Sender<Timestamped<NodeEvent>>,
    event: Timestamped<NodeEvent>,
    deferred: Option<&mut Vec<DeferredDelivery>>,
    ft_stats: Option<&FaultToleranceStats>,
    closed: &mut Vec<&'a NodeId>,
) -> bool {
    let event = if channel.capacity() < CONTROL_EVENT_HEADROOM {
        event
    } else {
        match channel.try_send(event) {
            Ok(()) => {
                dataflow.inc_pending(receiver_id);
                return true;
            }
            Err(mpsc::error::TrySendError::Closed(_)) => {
                closed.push(receiver_id);
                return false;
            }
            Err(mpsc::error::TrySendError::Full(event)) => event,
        }
    };
    let requires_backpressure = dataflow.input_requires_backpressure(receiver_id, input_id);
    let deferred = deferred
        .filter(|_| requires_backpressure)
        .and_then(|deferred| Some((deferred, dataflow.drain_signals.get(receiver_id)?.clone())))
        // A receiver that already let a held delivery run into the stall
        // limit is not waited for again until it drains.
        .filter(|(_, drained)| !drained.gave_up.load(atomic::Ordering::Acquire));
    match deferred {
        Some((deferred, drained)) => {
            tracing::debug!(
                node = %receiver_id,
                input = %input_id,
                "event channel full ({}/{}), holding the producer of `{}` until the receiver \
                 makes room (queue_policy: backpressure)",
                channel.capacity(),
                NODE_EVENT_CHANNEL_CAPACITY,
                output_id.1,
            );
            deferred.push(DeferredDelivery {
                receiver: receiver_id.clone(),
                input: input_id.clone(),
                channel: channel.clone(),
                pending: dataflow.pending_messages.get(receiver_id).cloned(),
                drained,
                event,
            });
            true
        }
        None => {
            tracing::warn!(
                node = %receiver_id,
                "event channel full ({}/{}), dropping message (node is too slow)",
                channel.capacity(),
                NODE_EVENT_CHANNEL_CAPACITY,
            );
            if let Some(stats) = ft_stats {
                stats.record_drop(1, requires_backpressure);
            }
            false
        }
    }
}

/// `deferred` is where deliveries to a full backpressure receiver go instead
/// of being dropped (see [`DeferredDelivery`]); `None` means no one can wait
/// for room on the caller's behalf (a remote forward, a benchmark), and such
/// a delivery is dropped and counted like any other.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn send_output_to_local_receivers(
    output_id: &OutputId,
    dataflow: &mut RunningDataflow,
    metadata: &metadata::Metadata,
    data: Option<DataMessage>,
    clock: &HLC,
    ft_stats: Option<&FaultToleranceStats>,
    need_data_bytes: bool,
    mut deferred: Option<&mut Vec<DeferredDelivery>>,
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
            let event = Timestamped {
                inner: NodeEvent::Input {
                    id: input_id.clone(),
                    metadata: metadata_arc
                        .get_or_insert_with(|| Arc::new(metadata.clone()))
                        .clone(),
                    data: data.clone(),
                },
                timestamp,
            };
            let accepted = offer_event(
                dataflow,
                receiver_id,
                input_id,
                output_id,
                channel,
                event,
                deferred.as_deref_mut(),
                ft_stats,
                &mut closed,
            );
            if !accepted {
                continue;
            }
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
        } else if dataflow.running_nodes.contains_key(receiver_id)
            && !dataflow.dropped_event_streams.contains(receiver_id)
        {
            // The receiver is registered in `mappings` AND still a live node,
            // but has no daemon event stream and did not drop it deliberately:
            // its channel was dropped by a crash that never re-subscribed or a
            // closed listener. This is the silent-routing-loss mode of
            // dora-rs/dora#3201 — the producer's send still "succeeds" but the
            // consumer receives nothing, indefinitely. Make it visible, once
            // per edge; the marker is cleared on (re)subscribe so an edge that
            // drops its stream again after reconnecting gets a fresh warning.
            //
            // A consumer that finished normally sends `EventStreamDropped` (so
            // it is in `dropped_event_streams`) but is still in `running_nodes`
            // until its process exit is observed. Producing to it in that
            // window is expected, not a fault, so it falls through to the
            // debug arm below instead of a misleading "failed to re-subscribe"
            // warning (dora-rs/dora#3556).
            //
            // Every such message is a counted drop — and a lost one on a
            // backpressure input — so a run that promised delivery
            // (`fail_on_lost_backpressure_messages`) cannot pass over it.
            if let Some(stats) = ft_stats {
                stats.record_drop(
                    1,
                    dataflow.input_requires_backpressure(receiver_id, input_id),
                );
            }
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
            // The receiver has no daemon event stream and either already exited
            // (`running_nodes` has no entry) or finished normally and dropped
            // its stream while its process exit is still pending (still in
            // `running_nodes`, in `dropped_event_streams` — dora-rs/dora#3556).
            // Either way its receiver-edge mapping outlives the node until the
            // dataflow finishes (lib.rs: handle_node_stop_inner), so an upstream
            // that keeps sending still reaches here — expected, not a restart
            // failure, so debug only.
            let finished_but_running = dataflow.running_nodes.contains_key(receiver_id);
            tracing::debug!(
                receiver = %receiver_id,
                input = %input_id,
                output = %output_id.1,
                "dropping `{}/{}` to `{receiver_id}`: node has {} (no daemon \
                 event stream)",
                output_id.0,
                output_id.1,
                if finished_but_running {
                    "finished but not yet exited"
                } else {
                    "exited"
                },
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
) {
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
        return;
    }

    // Before `InputClosed` goes out, so a message still held for this input
    // cannot land behind it (dora-rs/dora#3619).
    if let Some(drained) = dataflow.drain_signals.get(receiver_id) {
        drained.close_input(input_id);
    }

    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id)
        && was_open
        && send_with_timestamp(
            channel,
            NodeEvent::InputClosed {
                id: input_id.clone(),
            },
            clock,
        )
        .ok()
            == Some(true)
    {
        dataflow.inc_pending(receiver_id);
    }

    signal_all_inputs_closed_if_drained(dataflow, receiver_id, clock);
}

/// If `receiver_id` has finished, disable its restart policy and notify it
/// that all inputs are closed.
///
/// "Finished" is [`RunningDataflow::should_signal_all_inputs_closed`], which
/// under `--exit-when-nodes-finish` means its data inputs have closed even
/// while a timer keeps ticking.
///
/// Shared drain-completion tail of [`close_input`] and [`break_input`]; a
/// no-op if the node still has open/broken inputs or has no subscribe channel.
pub(crate) fn signal_all_inputs_closed_if_drained(
    dataflow: &mut RunningDataflow,
    receiver_id: &NodeId,
    clock: &HLC,
) {
    let Some(channel) = dataflow.subscribe_channels.get(receiver_id) else {
        return;
    };
    // As at the subscribe site: either "nothing left open" (pre-existing
    // behavior, and true for a source) or "drained" (the node we are about
    // to tell to finish) disables restart.
    if (dataflow.open_inputs(receiver_id).is_empty() || dataflow.is_drained(receiver_id))
        && !dataflow.has_broken_input(receiver_id)
        && let Some(node) = dataflow.running_nodes.get_mut(receiver_id)
    {
        node.disable_restart();
    }
    if dataflow.is_finished(receiver_id)
        && send_with_timestamp(channel, NodeEvent::AllInputsClosed, clock).ok() == Some(true)
    {
        dataflow.inc_pending(receiver_id);
        dataflow
            .all_inputs_closed_at
            .insert(receiver_id.clone(), Instant::now());
    }
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
    if let Some(channel) = dataflow.subscribe_channels.get(receiver_id)
        && send_with_timestamp(
            channel,
            NodeEvent::InputClosed {
                id: input_id.clone(),
            },
            clock,
        )
        .ok()
            == Some(true)
    {
        dataflow.inc_pending(receiver_id);
    }

    signal_all_inputs_closed_if_drained(dataflow, receiver_id, clock);
}
