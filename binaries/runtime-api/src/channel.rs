use dora_core::config::DataId;
use dora_message::config::QueuePolicy;
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
    queue: VecDeque<Option<Event>>,
    /// Pre-computed effective cap per input ID.
    effective_caps: BTreeMap<DataId, (usize, QueuePolicy)>,
}

impl InputBuffer {
    pub fn new(queue_sizes: BTreeMap<DataId, (usize, QueuePolicy)>) -> Self {
        let effective_caps = queue_sizes
            .into_iter()
            .map(|(id, (size, policy))| (id, (policy.effective_cap(size), policy)))
            .collect();
        Self {
            queue: VecDeque::new(),
            effective_caps,
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
        loop {
            match self.queue.pop_front() {
                Some(Some(next)) => break outgoing.send_async(next).fuse(),
                Some(None) => {
                    // dropped event, try again with next one
                }
                None => break future::Fuse::terminated(),
            }
        }
    }

    fn add_event(&mut self, event: Event) {
        self.queue.push_back(Some(event));

        // Cap the queue after *every* single push. `drop_oldest_inputs` relies
        // on this cadence — it only re-checks the just-pushed input — so exactly
        // one event must be enqueued between calls.
        self.drop_oldest_inputs();
    }

    fn drop_oldest_inputs(&mut self) {
        // Only the input of the just-pushed event can now exceed its cap: every
        // other input was already within cap before this push (the invariant
        // this method maintains on every `add_event`), and pushing a single
        // event leaves their counts unchanged. So there is no need to allocate
        // and populate a per-input map over *every* configured input on each
        // message — we only look at the newest event's input.
        let Some(Some(Event::Input { id, .. })) = self.queue.back() else {
            // Newest event isn't an input (e.g. `InputClosed`) — nothing to cap.
            return;
        };
        let (cap, policy) = match self.effective_caps.get(id) {
            Some((cap, policy)) => (*cap, *policy),
            None => {
                tracing::warn!("no queue size known for received operator input `{id}`");
                return;
            }
        };

        // Count this input's live events. In steady state (consumer keeping up)
        // the deque is near-empty, so this is a handful of comparisons and no
        // allocation — the common path never touches the heap.
        let live = self
            .queue
            .iter()
            .filter(|event| matches!(event, Some(Event::Input { id: other, .. }) if other == id))
            .count();
        if live <= cap {
            return;
        }

        // Over cap (rare overflow path). Clone the id once so the immutable
        // borrow of `self.queue` above is released before we mutate it, then
        // drop the oldest surplus occurrences (front to back), keeping the
        // newest `cap`. `effective_cap` is always >= 1, so a single push can
        // only put us one over, but stay robust to any surplus.
        let id = id.clone();
        let to_drop = live - cap;
        if let QueuePolicy::Backpressure = policy {
            tracing::error!(
                "backpressure input `{id}` hit hard cap, dropping oldest to prevent OOM"
            );
        }
        let mut dropped = 0;
        for event in self.queue.iter_mut() {
            if dropped == to_drop {
                break;
            }
            if matches!(event, Some(Event::Input { id: other, .. }) if *other == id) {
                *event = None;
                dropped += 1;
            }
        }

        self.compact();
        tracing::warn!("dropped {dropped} operator inputs because event queue was too full");
    }

    /// Physically remove the `None` tombstones left by [`Self::drop_oldest_inputs`].
    ///
    /// Without this, a stalled operator (slow `on_event`) makes the deque grow
    /// by one `Option::None` slot per received input forever — the number of
    /// live `Some` events stays capped, but the tombstones are only ever cleared
    /// by `pop_front` in `send_next_queued`, which never runs while the outgoing
    /// send is pending. That defeats the bounded-memory guarantee of the
    /// drop-oldest policy and leaks until OOM. `retain` preserves the FIFO order
    /// of the surviving events.
    fn compact(&mut self) {
        self.queue.retain(Option::is_some);
    }
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

    // The compaction that `drop_oldest_inputs` runs after evicting over-cap
    // inputs must *physically* remove the `None` tombstones, not just leave them
    // in place. Otherwise a stalled consumer accumulates one tombstone per
    // dropped input without bound (they are only ever cleared by `pop_front` in
    // `send_next_queued`, which does not run while the outgoing send is pending)
    // — an unbounded memory leak that defeats the drop-oldest policy. This
    // exercises the compaction primitive directly with the metadata-free
    // `Event::InputClosed` stand-in (`compact` only distinguishes `Some`/`None`).
    #[test]
    fn compact_removes_tombstones_preserving_order() {
        let mut buffer = InputBuffer::new(BTreeMap::new());
        buffer.queue = VecDeque::from([
            None,
            Some(closed_event("a")),
            None,
            None,
            Some(closed_event("b")),
            None,
        ]);

        buffer.compact();

        assert_eq!(
            buffer.queue.len(),
            2,
            "every `None` tombstone must be removed, not merely nulled out"
        );
        let ids: Vec<String> = buffer
            .queue
            .iter()
            .map(|e| match e {
                Some(Event::InputClosed { id }) => id.to_string(),
                _ => panic!("unexpected queue entry after compaction"),
            })
            .collect();
        assert_eq!(
            ids,
            ["a", "b"],
            "surviving events must keep their FIFO order"
        );
    }

    // Integration-level guard for the fix in #2483: driving the *real* path
    // (`add_event` -> `drop_oldest_inputs` -> `compact`) under a stalled
    // consumer must keep the deque physically bounded, not just cap the number
    // of live `Some` events. The stall is modelled by never draining the queue
    // (no `send_next_queued`/`pop_front`), which is exactly when the tombstones
    // would otherwise accumulate. Unlike `compact_removes_tombstones_preserving_order`,
    // this routes through the fix site, so deleting the `self.compact();` call
    // in `drop_oldest_inputs` makes the length assertion fail (regression test
    // for #2680 — the previous test left that call site unguarded/mutable).
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

        // With `compact()`, the deque holds only the `cap` live events and no
        // tombstones. Without it, every drop past the cap would leave a `None`
        // behind, growing the deque roughly linearly with `total`.
        assert_eq!(
            buffer.queue.len(),
            cap,
            "stalled consumer must not accumulate tombstones (deque must stay bounded to the cap)"
        );
        assert!(
            buffer.queue.iter().all(Option::is_some),
            "no `None` tombstones may remain after compaction"
        );
    }

    // Each input must be capped independently: pushing many events of one input
    // must drop only *that* input's oldest surplus, never another input's
    // events, and must keep the newest `cap` of the offending input in FIFO
    // order. This pins the optimized `drop_oldest_inputs` (which now inspects
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

        let ids: Vec<String> = buffer
            .queue
            .iter()
            .map(|e| match e {
                Some(Event::Input { id, .. }) => id.to_string(),
                _ => panic!("unexpected queue entry"),
            })
            .collect();

        // `a` capped to 1, `b` capped to 2 — independently — with no tombstones,
        // and the surviving events keep their FIFO order (newest per input kept).
        assert_eq!(
            ids,
            ["b", "a", "b"],
            "each input must be capped independently, keeping the newest in FIFO order"
        );
    }
}
