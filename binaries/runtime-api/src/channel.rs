use dora_core::config::DataId;
use dora_message::config::{DEFAULT_QUEUE_SIZE, QueuePolicy};
use dora_message::metadata::{GOAL_ID, GOAL_STATUS, REQUEST_ID, carries_pattern_correlation};
use dora_node_api::Event;
use futures::{
    FutureExt,
    future::{self, FusedFuture},
};
use std::collections::{BTreeMap, VecDeque};

pub fn channel(
    runtime: &tokio::runtime::Handle,
    queue_sizes: BTreeMap<DataId, (usize, QueuePolicy)>,
) -> (flume::Sender<Event>, flume::Receiver<Event>) {
    let (incoming_tx, incoming_rx) = flume::bounded(10);
    let (outgoing_tx, outgoing_rx) = flume::bounded(0);

    runtime.spawn(async {
        let mut buffer = InputBuffer::new(queue_sizes);
        buffer.run(incoming_rx, outgoing_tx).await;
    });

    (incoming_tx, outgoing_rx)
}

struct InputBuffer {
    queue: VecDeque<Event>,
    /// Effective cap and current queue occupancy per input ID.
    inputs: BTreeMap<DataId, InputState>,
}

struct InputState {
    /// Pre-computed effective cap.
    cap: usize,
    policy: QueuePolicy,
    /// Number of this input's events in `queue`, kept in step with it so the
    /// per-push cap check does not have to scan the queue.
    queued: usize,
    /// Running count of this input's events dropped at the cap; drives the
    /// rate-limited drop log (see [`should_log_drop`]).
    dropped: u64,
}

impl InputState {
    fn new(size: usize, policy: QueuePolicy) -> Self {
        Self {
            cap: policy.effective_cap(size),
            policy,
            queued: 0,
            dropped: 0,
        }
    }
}

impl InputBuffer {
    pub fn new(queue_sizes: BTreeMap<DataId, (usize, QueuePolicy)>) -> Self {
        let inputs = queue_sizes
            .into_iter()
            .map(|(id, (size, policy))| (id, InputState::new(size, policy)))
            .collect();
        Self {
            queue: VecDeque::new(),
            inputs,
        }
    }

    pub async fn run(&mut self, incoming: flume::Receiver<Event>, outgoing: flume::Sender<Event>) {
        let mut send_out_buf = future::Fuse::terminated();
        let mut incoming_closed = false;
        loop {
            let next_incoming = if incoming_closed {
                future::Fuse::terminated()
            } else {
                incoming.recv_async().fuse()
            };
            match future::select(next_incoming, send_out_buf).await {
                future::Either::Left((event, mut send_out)) => {
                    match event {
                        Ok(event) => {
                            // received a new event -> push it to the queue
                            self.add_event(event);

                            // if outgoing queue is empty, fill it again
                            if send_out.is_terminated() {
                                send_out = self.send_next_queued(&outgoing);
                            }
                        }
                        Err(flume::RecvError::Disconnected) => {
                            incoming_closed = true;
                        }
                    }

                    // reassign the send_out future, which might be still in progress
                    send_out_buf = send_out;
                }
                future::Either::Right((send_result, _)) => match send_result {
                    Ok(()) => {
                        send_out_buf = self.send_next_queued(&outgoing);
                    }
                    Err(flume::SendError(_)) => break,
                },
            };
            if incoming_closed && send_out_buf.is_terminated() && self.queue.is_empty() {
                break;
            }
        }
    }

    fn send_next_queued<'a>(
        &mut self,
        outgoing: &'a flume::Sender<Event>,
    ) -> future::Fuse<flume::r#async::SendFut<'a, Event>> {
        match self.queue.pop_front() {
            Some(next) => {
                if let Event::Input { id, .. } = &next {
                    self.forget_queued_input(id);
                }
                outgoing.send_async(next).fuse()
            }
            None => future::Fuse::terminated(),
        }
    }

    fn forget_queued_input(&mut self, id: &DataId) {
        let state = self.inputs.get_mut(id);
        debug_assert!(
            state.as_ref().is_some_and(|state| state.queued > 0),
            "queued count for `{id}` out of step with the queue"
        );
        if let Some(state) = state {
            state.queued = state.queued.saturating_sub(1);
        }
    }

    /// Queue `event`, dropping the oldest event of the same input if that
    /// puts it over its cap. Returns whether a drop was logged.
    fn add_event(&mut self, event: Event) -> bool {
        let Event::Input { id, .. } = &event else {
            // Not an input (e.g. `InputClosed`) — nothing to cap.
            self.queue.push_back(event);
            return false;
        };

        // Only the pushed input can now exceed its cap: every other input was
        // already within cap before this push, and this push leaves their
        // counts unchanged.
        let state = match self.inputs.get_mut(id) {
            Some(state) => state,
            None => {
                // An input the operator never declared a queue size for (e.g. a
                // routing/`dora node connect` edge delivering an undeclared
                // input). Fall back to the default cap so drop-oldest still
                // bounds this input's memory, mirroring the node-side scheduler
                // (`Scheduler::add_event`). Without a cap here, a stalled
                // `on_event` lets this input's queue grow without bound (OOM).
                // Record it so the warning fires once, not per message.
                tracing::warn!(
                    "no queue size configured for operator input `{id}`; \
                     using default size {DEFAULT_QUEUE_SIZE}"
                );
                self.inputs
                    .entry(id.clone())
                    .or_insert_with(|| InputState::new(DEFAULT_QUEUE_SIZE, QueuePolicy::default()))
            }
        };
        // A counter rather than a scan of the queue: once the operator lags,
        // the queue holds up to the cap (10x `queue_size` under backpressure),
        // and counting it on every push made each message cost a pass over the
        // whole backlog exactly when the runtime is behind.
        state.queued += 1;
        let mut logged_drop = false;
        if state.queued > state.cap {
            // Make room before pushing, so the newest `cap` (this event
            // included) stay queued in FIFO order. `effective_cap` is always
            // >= 1, so a single push can only put an input one over.
            //
            // A dropped service/action event leaves its caller waiting
            // forever, so spare those as the node-side scheduler does
            // (`select_eviction`).
            let dropped = match eviction_candidates(&self.queue, id) {
                (_, Some(ordinary)) => {
                    self.queue.remove(ordinary);
                    Some("oldest")
                }
                // Only correlations are queued and the incoming event is
                // ordinary: drop it instead.
                (Some(_), None) if !is_correlated(&event) => Some("incoming"),
                // Only correlations on both sides: one has to go. Every such
                // drop breaks a request, so each one is logged.
                (Some(oldest), None) => {
                    if let Some(Event::Input { metadata, .. }) = self.queue.remove(oldest) {
                        let params = &metadata.parameters;
                        tracing::error!(
                            input = %id,
                            request_id = ?params.get(REQUEST_ID),
                            goal_id = ?params.get(GOAL_ID),
                            goal_status = ?params.get(GOAL_STATUS),
                            "operator queue full of correlated messages; dropping oldest \
                             correlation. This breaks the service/action request-response \
                             contract. Consider increasing queue_size or switching this \
                             input to `queue_policy: backpressure`."
                        );
                    }
                    Some("oldest")
                }
                // The counter ran ahead of the queue and nothing can be
                // evicted; the decrement below still keeps it at the cap.
                (None, _) => {
                    debug_assert!(false, "queued count for `{id}` exceeds the queue");
                    None
                }
            };
            state.queued -= 1;
            if let Some(which) = dropped {
                // An operator slower than its input sits at the cap, so this
                // runs for every incoming message. Log at a power-of-two
                // cadence, like the node-side scheduler, instead of once or
                // twice per message.
                state.dropped = state.dropped.saturating_add(1);
                if should_log_drop(state.dropped) {
                    logged_drop = true;
                    let total = state.dropped;
                    if let QueuePolicy::Backpressure = state.policy {
                        tracing::error!(
                            "backpressure input `{id}` hit hard cap, dropping the {which} \
                             event to prevent OOM ({total} dropped so far)"
                        );
                    } else {
                        tracing::warn!(
                            "dropped the {which} operator input `{id}` because its queue \
                             was full ({total} dropped so far)"
                        );
                    }
                }
                if which == "incoming" {
                    return logged_drop;
                }
            }
        }
        self.queue.push_back(event);
        logged_drop
    }
}

/// Whether the drop that brings an input's running drop count to `count`
/// should be logged: at 1, 2, 4, 8, … so a lasting overload keeps being
/// reported, but only O(log N) times for N drops. Mirrors the node-side
/// `should_warn_drop` in `dora-node-api`.
fn should_log_drop(count: u64) -> bool {
    count.is_power_of_two()
}

/// Whether `event` is part of a service or action exchange: it carries a
/// `request_id`, `goal_id` or `goal_status`.
fn is_correlated(event: &Event) -> bool {
    matches!(event, Event::Input { metadata, .. } if carries_pattern_correlation(&metadata.parameters))
}

/// The indices of the oldest queued event of input `id`, and of the oldest
/// one that is not a service/action correlation, found in one pass.
fn eviction_candidates(queue: &VecDeque<Event>, id: &DataId) -> (Option<usize>, Option<usize>) {
    // The oldest occurrence is usually near the front of the queue; only a
    // low-rate input queued behind a large backlog of others pays for a longer
    // search.
    let mut oldest = None;
    for (index, event) in queue.iter().enumerate() {
        if matches!(event, Event::Input { id: other, .. } if other == id) {
            oldest.get_or_insert(index);
            if !is_correlated(event) {
                return (oldest, Some(index));
            }
        }
    }
    (oldest, None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::new_empty_array;
    use arrow::datatypes::DataType;
    use dora_message::metadata::{Metadata, Parameter};
    use dora_node_api::DoraArray;

    fn closed_event(id: &str) -> Event {
        Event::InputClosed {
            id: DataId::from(id.to_string()),
        }
    }

    fn input_event(id: &str) -> Event {
        Event::Input {
            id: DataId::from(id.to_string()),
            metadata: Metadata::new(dora_core::uhlc::HLC::default().new_timestamp()),
            data: DoraArray::from(new_empty_array(&DataType::Null)),
        }
    }

    fn queued_ids(buffer: &InputBuffer) -> Vec<String> {
        buffer
            .queue
            .iter()
            .map(|e| match e {
                Event::Input { id, .. } | Event::InputClosed { id } => id.to_string(),
                _ => panic!("unexpected queue entry"),
            })
            .collect()
    }

    fn queued(buffer: &InputBuffer, id: &DataId) -> Option<usize> {
        buffer.inputs.get(id).map(|state| state.queued)
    }

    // The per-input counters the cap check reads must stay equal to what is
    // actually queued: incremented on push, decremented when an event is
    // handed to the operator and when an over-cap input is dropped. A counter
    // that drifted high would drop inputs below the cap; one that drifted low
    // would let the queue grow past it.
    #[test]
    fn queued_input_counts_track_queue_contents() {
        let mut caps = BTreeMap::new();
        caps.insert(
            DataId::from("a".to_string()),
            (2usize, QueuePolicy::DropOldest),
        );
        let mut buffer = InputBuffer::new(caps);
        let a = DataId::from("a".to_string());
        let b = DataId::from("b".to_string());

        buffer.add_event(input_event("a"));
        buffer.add_event(closed_event("b"));
        buffer.add_event(input_event("b"));
        buffer.add_event(input_event("a"));
        buffer.add_event(input_event("a")); // over cap: drops the oldest `a`
        assert_eq!(queued_ids(&buffer), ["b", "b", "a", "a"]);
        assert_eq!(queued(&buffer, &a), Some(2));
        assert_eq!(queued(&buffer, &b), Some(1));

        // Hand every event out; the counters must drain to zero.
        let (tx, _rx) = flume::unbounded();
        for _ in 0..4 {
            assert!(!buffer.send_next_queued(&tx).is_terminated());
        }
        assert!(buffer.send_next_queued(&tx).is_terminated());
        assert_eq!(queued(&buffer, &a), Some(0));
        assert_eq!(queued(&buffer, &b), Some(0));
    }

    fn correlated_input_event(id: &str, request_id: &str) -> Event {
        let mut event = input_event(id);
        if let Event::Input { metadata, .. } = &mut event {
            metadata.parameters.insert(
                REQUEST_ID.to_string(),
                Parameter::String(request_id.to_string()),
            );
        }
        event
    }

    fn request_ids(buffer: &InputBuffer) -> Vec<Option<String>> {
        buffer
            .queue
            .iter()
            .map(|e| match e {
                Event::Input { metadata, .. } => match metadata.parameters.get(REQUEST_ID) {
                    Some(Parameter::String(r)) => Some(r.clone()),
                    _ => None,
                },
                _ => None,
            })
            .collect()
    }

    fn buffer_with_cap(id: &str, cap: usize) -> InputBuffer {
        let mut caps = BTreeMap::new();
        caps.insert(DataId::from(id.to_string()), (cap, QueuePolicy::DropOldest));
        InputBuffer::new(caps)
    }

    // A service response must not be evicted while an ordinary event of the
    // same input can go instead, matching the node-side scheduler.
    #[test]
    fn over_cap_evicts_an_ordinary_event_before_a_correlated_one() {
        let mut buffer = buffer_with_cap("x", 2);
        buffer.add_event(correlated_input_event("x", "A"));
        buffer.add_event(input_event("x"));
        buffer.add_event(input_event("x"));
        assert_eq!(request_ids(&buffer), [Some("A".into()), None]);
        assert_eq!(queued(&buffer, &DataId::from("x".to_string())), Some(2));
    }

    // With only correlations queued, an ordinary incoming event is the one
    // dropped.
    #[test]
    fn over_cap_drops_ordinary_incoming_when_only_correlations_are_queued() {
        let mut buffer = buffer_with_cap("x", 1);
        buffer.add_event(correlated_input_event("x", "A"));
        buffer.add_event(input_event("x"));
        assert_eq!(request_ids(&buffer), [Some("A".into())]);
        assert_eq!(queued(&buffer, &DataId::from("x".to_string())), Some(1));
    }

    // When both sides are correlations the oldest still has to go, so the
    // queue stays bounded.
    #[test]
    fn over_cap_drops_oldest_correlation_when_nothing_else_is_left() {
        let mut buffer = buffer_with_cap("x", 1);
        buffer.add_event(correlated_input_event("x", "A"));
        buffer.add_event(correlated_input_event("x", "B"));
        assert_eq!(request_ids(&buffer), [Some("B".into())]);
        assert_eq!(queued(&buffer, &DataId::from("x".to_string())), Some(1));
    }

    // Integration-level guard for #2483: driving the *real* path
    // (`add_event` -> `eviction_candidates`) under a stalled consumer must keep
    // the deque physically bounded to the cap. The stall is modelled by never
    // draining the queue (no `send_next_queued`).
    #[test]
    fn drop_oldest_inputs_keeps_deque_bounded_under_stall() {
        let cap = 2usize;
        let mut caps = BTreeMap::new();
        caps.insert(
            DataId::from("x".to_string()),
            (cap, QueuePolicy::DropOldest),
        );
        let mut buffer = InputBuffer::new(caps);

        // Feed far more inputs than the cap without ever draining the queue.
        let total = 50;
        for _ in 0..total {
            buffer.add_event(input_event("x"));
        }

        assert_eq!(
            buffer.queue.len(),
            cap,
            "stalled consumer must keep the deque bounded to the cap"
        );
    }

    // Each input must be capped independently: pushing many events of one input
    // must drop only *that* input's oldest surplus, never another input's
    // events, and must keep the newest `cap` of the offending input in FIFO
    // order. This pins the optimized `add_event` (which now inspects
    // only the just-pushed input instead of scanning a map over all inputs) to
    // the same observable behavior as a full per-input scan.
    #[test]
    fn drop_oldest_inputs_caps_each_input_independently() {
        let mut caps = BTreeMap::new();
        caps.insert(
            DataId::from("a".to_string()),
            (1usize, QueuePolicy::DropOldest),
        );
        caps.insert(
            DataId::from("b".to_string()),
            (2usize, QueuePolicy::DropOldest),
        );
        let mut buffer = InputBuffer::new(caps);

        // Interleave inputs without ever draining the queue.
        for id in ["a", "b", "a", "b", "b", "a", "b"] {
            buffer.add_event(input_event(id));
        }

        let ids = queued_ids(&buffer);

        // `a` capped to 1, `b` capped to 2 — independently — and the surviving
        // events keep their FIFO order (newest per input kept).
        assert_eq!(
            ids,
            ["b", "a", "b"],
            "each input must be capped independently, keeping the newest in FIFO order"
        );
    }

    // An input with no configured queue size (not present in `config.inputs`,
    // e.g. delivered by a routing/`dora node connect` edge) must still be
    // bounded: `add_event` falls back to `DEFAULT_QUEUE_SIZE` instead
    // of returning uncapped. Without the fallback, a stalled consumer accretes
    // this input's events without bound (OOM). Mirrors the node-side scheduler.
    #[test]
    fn unconfigured_input_falls_back_to_default_cap_under_stall() {
        // No caps configured at all.
        let mut buffer = InputBuffer::new(BTreeMap::new());

        // Feed far more than the default cap without ever draining the queue.
        for _ in 0..(DEFAULT_QUEUE_SIZE + 50) {
            buffer.add_event(input_event("undeclared"));
        }

        assert_eq!(
            buffer.queue.len(),
            DEFAULT_QUEUE_SIZE,
            "an unconfigured input must be bounded to the default cap, not grow unbounded"
        );
        // The fallback is recorded so it is computed once, not per message.
        assert!(
            buffer
                .inputs
                .contains_key(&DataId::from("undeclared".to_string())),
            "the default cap must be recorded for the unconfigured input"
        );
    }

    // An operator slower than its input sits at the cap, so every push drops
    // one event. The drop log must follow a power-of-two cadence over the
    // running count, not fire once (or twice) per message.
    #[test]
    fn drops_are_counted_and_logged_at_power_of_two_cadence() {
        let id = DataId::from("fast".to_string());
        let mut buffer =
            InputBuffer::new(BTreeMap::from([(id.clone(), (2, QueuePolicy::DropOldest))]));

        let logged = (0..1002)
            .filter(|_| buffer.add_event(input_event("fast")))
            .count();

        assert_eq!(buffer.inputs[&id].dropped, 1000);
        // 1, 2, 4, …, 512: ten log lines for a thousand drops.
        assert_eq!(logged, 10);
    }
}
