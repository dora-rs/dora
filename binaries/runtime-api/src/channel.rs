use dora_core::config::DataId;
use dora_message::config::{DEFAULT_QUEUE_SIZE, QueuePolicy};
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
}

impl InputState {
    fn new(size: usize, policy: QueuePolicy) -> Self {
        Self {
            cap: policy.effective_cap(size),
            policy,
            queued: 0,
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

    fn add_event(&mut self, event: Event) {
        let Event::Input { id, .. } = &event else {
            // Not an input (e.g. `InputClosed`) — nothing to cap.
            self.queue.push_back(event);
            return;
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
        let over_cap = state.queued.saturating_sub(state.cap);
        if over_cap > 0 {
            // Make room before pushing, so the newest `cap` (this event
            // included) stay queued in FIFO order. `effective_cap` is always
            // >= 1, so a single push can only put an input one over.
            if let QueuePolicy::Backpressure = state.policy {
                tracing::error!(
                    "backpressure input `{id}` hit hard cap, dropping oldest to prevent OOM"
                );
            }
            let dropped = drop_oldest(&mut self.queue, id, over_cap);
            debug_assert_eq!(
                dropped, over_cap,
                "queued count for `{id}` exceeds the queue"
            );
            state.queued -= dropped;
            tracing::warn!("dropped {dropped} operator inputs because event queue was too full");
        }
        self.queue.push_back(event);
    }
}

/// Remove up to `count` of the oldest queued events of input `id`, returning
/// how many were removed.
fn drop_oldest(queue: &mut VecDeque<Event>, id: &DataId, count: usize) -> usize {
    let mut dropped = 0;
    while dropped < count {
        // The oldest occurrence is usually near the front of the queue; only a
        // low-rate input queued behind a large backlog of others pays for a
        // longer search and shift.
        let oldest = queue
            .iter()
            .position(|event| matches!(event, Event::Input { id: other, .. } if other == id));
        let Some(index) = oldest else { break };
        queue.remove(index);
        dropped += 1;
    }
    dropped
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::new_empty_array;
    use arrow::datatypes::DataType;
    use dora_message::metadata::Metadata;
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

    // Integration-level guard for #2483: driving the *real* path
    // (`add_event` -> `drop_oldest`) under a stalled consumer must keep
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
}
