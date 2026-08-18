use std::{
    collections::{HashMap, VecDeque},
    sync::LazyLock,
};

use dora_message::{
    config::{DEFAULT_QUEUE_SIZE, QueuePolicy},
    daemon_to_node::NodeEvent,
    id::DataId,
    metadata::{
        GOAL_ID, GOAL_STATUS, MetadataParameters, REQUEST_ID, carries_pattern_correlation,
        get_string_param,
    },
};

use super::thread::EventItem;

/// The metadata parameters carried by an input-bearing event, or `None` for
/// events with no metadata (Stop, reload, input-closed, ...).
///
/// This is the single place that decides which `EventItem` variants carry
/// parameters, so the eviction guard (`is_correlated`) and its diagnostics
/// (`log_correlation_drop`) can never disagree on that classification.
fn event_parameters(event: &EventItem) -> Option<&MetadataParameters> {
    match event {
        EventItem::NodeEvent {
            event: NodeEvent::Input { metadata, .. },
            ..
        } => Some(&metadata.parameters),
        EventItem::ZenohInput { metadata, .. } => Some(&metadata.parameters),
        _ => None,
    }
}

/// Returns `true` if the event carries request/response or action correlation
/// metadata (`request_id`, `goal_id`, or `goal_status`).
///
/// These keys bind a message to a specific service request or action goal.
/// Silently dropping such a message breaks the correlation contract — the
/// client waits forever for a response or result that never arrives
/// (dora-rs/adora#145).
fn is_correlated(event: &EventItem) -> bool {
    // Delegate to the canonical definition in `dora_message` so the scheduler's
    // eviction guard can never disagree with the send-side / receive-side /
    // daemon-debug layers about which keys mark a message as pattern-correlated
    // (see `carries_pattern_correlation`). Duplicating the key list here risked
    // silently dropping service/action messages if a new correlation key were
    // added to only one copy.
    event_parameters(event).is_some_and(carries_pattern_correlation)
}

/// Returns `true` if the event is the daemon's [`Stop`][NodeEvent::Stop]
/// shutdown signal.
///
/// `Stop` must never be silently evicted from a full queue: dropping it makes
/// the node miss shutdown entirely and keep running until the daemon's
/// force-kill grace deadline. Since at most one `Stop` is ever in flight,
/// making it eviction-immune cannot let the queue grow unbounded.
fn is_stop(event: &EventItem) -> bool {
    matches!(
        event,
        EventItem::NodeEvent {
            event: NodeEvent::Stop,
            ..
        }
    )
}

/// Outcome of `select_eviction`.
enum Eviction {
    /// Remove event at this index from the queue and push the incoming event.
    RemoveAt(usize),
    /// The queue holds only events that must be preserved (correlated and/or
    /// the `Stop`) and the incoming event is an ordinary one — drop the
    /// incoming event instead of breaking a correlation or losing `Stop`.
    DropIncoming,
    /// The queue is entirely correlated (plus possibly the `Stop`) and the
    /// incoming event must also be preserved — drop the oldest *correlated*
    /// event at this index with a loud error log, never the `Stop`.
    DropCorrelatedLoud(usize),
}

/// Choose which event to drop when the queue is at capacity.
///
/// Prefers sacrificing ordinary (non-correlated, non-`Stop`) events so that
/// service responses, action results, and the shutdown signal survive. See
/// `is_correlated` for the metadata keys that mark an event as part of a
/// pattern, and `is_stop` for why `Stop` is eviction-immune.
fn select_eviction(queue: &VecDeque<EventItem>, incoming: &EventItem) -> Eviction {
    // 1. Sacrifice the oldest ordinary event, never a correlation or `Stop`.
    if let Some(idx) = queue.iter().position(|e| !is_correlated(e) && !is_stop(e)) {
        return Eviction::RemoveAt(idx);
    }
    // 2. Nothing ordinary left to sacrifice. If the incoming event is itself
    //    ordinary, drop it rather than a correlation or the `Stop`.
    if !is_correlated(incoming) && !is_stop(incoming) {
        return Eviction::DropIncoming;
    }
    // 3. Both the queue and the incoming event must be preserved. Drop the
    //    oldest *correlated* event (loudly), keeping the `Stop` intact. If the
    //    queue somehow contains no correlated event (only a `Stop`, which is
    //    at most one), fall back to dropping the incoming event.
    match queue.iter().position(|e| !is_stop(e)) {
        Some(idx) => Eviction::DropCorrelatedLoud(idx),
        None => Eviction::DropIncoming,
    }
}

/// Emit a loud error when a correlated event has to be dropped because
/// everything in the queue is also correlated. Identifies the correlation
/// keys so operators can trace the affected request/goal.
fn log_correlation_drop(event_id: &DataId, dropped: &EventItem) {
    let Some(params) = event_parameters(dropped) else {
        return;
    };
    let request_id = get_string_param(params, REQUEST_ID);
    let goal_id = get_string_param(params, GOAL_ID);
    let goal_status = get_string_param(params, GOAL_STATUS);
    tracing::error!(
        input = %event_id,
        ?request_id,
        ?goal_id,
        ?goal_status,
        "queue full of correlated messages; dropping oldest correlation. \
         This breaks the service/action request-response contract. \
         Consider increasing queue_size or switching this input to \
         `queue_policy: backpressure`."
    );
}
pub(crate) const NON_INPUT_EVENT: &str = "dora.non_input_event";

/// Shared [`DataId`] for [`NON_INPUT_EVENT`], so the hot `add_event`/`next`
/// paths don't have to allocate a fresh `String` on every call.
static NON_INPUT_EVENT_ID: LazyLock<DataId> =
    LazyLock::new(|| DataId::from(NON_INPUT_EVENT.to_string()));

/// This scheduler will make sure that there is fairness between inputs.
///
/// The scheduler reorders events in the following way:
///
/// - **Non-input events are prioritized**
///   
///   If the node received any events that are not input events, they are returned first. The
///   intention of this reordering is that the nodes can react quickly to dataflow-related events
///   even when their input queues are very full.
///   
///   This reordering has some side effects that might be unexpected:
///   - An [`InputClosed`][super::Event::InputClosed] event might be yielded before the last
///     input events of that ID.
///     
///     Usually, an `InputClosed` event indicates that there won't be any subsequent inputs
///     of a certain ID. This invariant does not hold anymore for a scheduled event stream.
///   - The [`Stop`][super::Event::Stop] event might not be the last event of the stream anymore.
///     
///     Usually, the `Stop` event is the last event that is sent to a node before the event stream
///     is closed. Because of the reordering, the stream might return more events after a `Stop`
///     event.
/// - **Input events are grouped by ID** and yielded in a **least-recently used order (by ID)**.
///
///   The scheduler keeps a separate queue for each input ID, where the incoming input events are
///   placed in their chronological order. When yielding the next event, the scheduler iterates over
///   these queues in least-recently used order. This means that the queue corresponding to the
///   last yielded event will be checked last. The scheduler will return the oldest event from the
///   first non-empty queue.
///
///   The side effect of this change is that inputs events of different IDs are no longer in their
///   chronological order. This might lead to unexpected results for input events that are caused by
///   each other.
///
/// ## Example 1
/// Consider the case that one input has a very high frequency and another one with a very slow
/// frequency. The event stream will always alternate between the two inputs when each input is
/// available.
/// Without the scheduling, the high-frequency input would be returned much more often.
///
/// ## Example 2
/// Again, let's consider the case that one input has a very high frequency and the other has a
/// very slow frequency. This time, we define a small maximum queue sizes for the low-frequency
/// input, but a large queue size for the high-frequency one.
/// Using the scheduler, the event stream will always alternate between high and low-frequency
/// inputs as long as inputs of both types are available.
///
/// Without scheduling, the low-frequency input might never be yielded before
/// it's dropped because there is almost always an older high-frequency input available that is
/// yielded first. Once the low-frequency input would be the next one chronologically, it might
/// have been dropped already because the node received newer low-frequency inputs in the
/// meantime (the queue length is small). At this point, the next-oldest input is a high-frequency
/// input again.
///
/// ## Example 3
/// Consider a high-frequency camera input and a low-frequency bounding box input, which is based
/// on the latest camera image. The dataflow YAML file specifies a large queue size for the camera
/// input and a small queue size for the bounding box input.
///
/// With scheduling, the number of
/// buffered camera inputs might grow over time. As a result the camera inputs yielded from the
/// stream (in oldest-first order) are not synchronized with the bounding box inputs anymore. So
/// the node receives an up-to-date bounding box, but a considerably outdated image.
///
/// Without scheduling, the events are returned in chronological order. This time, the bounding
/// box might be slightly outdated if the camera sent new images before the bounding box was
/// ready. However, the time difference between the two input types is independent of the
/// queue size this time.
///
/// (If a perfect matching bounding box is required, we recommend to forward the input image as
/// part of the bounding box output. This way, the receiving node only needs to subscribe to one
/// input so no mismatches can happen.)
#[derive(Debug)]
pub struct Scheduler {
    /// Tracks the last-used event ID
    last_used: VecDeque<DataId>,
    /// Tracks events per ID
    event_queues: HashMap<DataId, (usize, VecDeque<EventItem>)>,
    /// Queue policies per input ID
    queue_policies: HashMap<DataId, QueuePolicy>,
    /// Drop counters per input ID
    dropped: HashMap<DataId, u64>,
}

impl Scheduler {
    pub(crate) fn with_policies(
        event_queues: HashMap<DataId, (usize, VecDeque<EventItem>)>,
        queue_policies: HashMap<DataId, QueuePolicy>,
    ) -> Self {
        let topic = VecDeque::from_iter(
            event_queues
                .keys()
                .filter(|t| **t != *NON_INPUT_EVENT_ID)
                .cloned(),
        );
        Self {
            last_used: topic,
            event_queues,
            queue_policies,
            dropped: HashMap::new(),
        }
    }

    /// Returns and resets the accumulated drop counts per input ID.
    pub fn drain_drop_counts(&mut self) -> HashMap<DataId, u64> {
        std::mem::take(&mut self.dropped)
    }

    pub(crate) fn add_event(&mut self, event: EventItem) {
        let (event_id, should_flush) = match &event {
            EventItem::NodeEvent {
                event: NodeEvent::Input { id, metadata, .. },
                ..
            } => {
                let flush = dora_message::metadata::get_bool_param(
                    &metadata.parameters,
                    dora_message::metadata::FLUSH,
                ) == Some(true);
                (id, flush)
            }
            EventItem::ZenohInput { id, metadata, .. } => {
                let flush = dora_message::metadata::get_bool_param(
                    &metadata.parameters,
                    dora_message::metadata::FLUSH,
                ) == Some(true);
                (id, flush)
            }
            _ => (&*NON_INPUT_EVENT_ID, false),
        };

        // Flush older queued messages when flush=true is present.
        //
        // Streaming pattern's `flush: true` means "discard stale stream chunks".
        // It must NOT wipe service responses or action results that happen to
        // share the same input, because those carry `request_id` / `goal_id` /
        // `goal_status` correlations whose senders are waiting for them
        // (dora-rs/adora#146). Use the same correlation predicate that the
        // drop_oldest path uses and retain correlated events across the flush.
        // Also retain the `Stop` shutdown signal (eviction-immune everywhere,
        // see `is_stop`): flush normally targets a per-input queue and `Stop`
        // lives under `NON_INPUT_EVENT_ID`, but the two collide if an input is
        // literally named `dora.non_input_event`, which `validate_data_id`
        // permits — so guard the flush path too rather than rely on that.
        if should_flush && let Some((_size, queue)) = self.event_queues.get_mut(event_id) {
            let before = queue.len();
            queue.retain(|e| is_correlated(e) || is_stop(e));
            let drained = before - queue.len();
            if drained > 0 {
                tracing::debug!(
                    "Flushed {drained} queued event(s) for input `{event_id}` (flush signal)"
                );
            }
            if !queue.is_empty() {
                tracing::debug!(
                    input = %event_id,
                    preserved = queue.len(),
                    "flush signal retained correlated (request_id/goal_id) events"
                );
            }
        }

        // Enforce queue size limit.
        //
        // The queue is normally preconfigured for every input at construction
        // (see `with_policies`), so look it up by reference first to avoid
        // cloning the `DataId` key on every event. Only the rare unconfigured
        // input path needs to allocate an owned key for insertion.
        if !self.event_queues.contains_key(event_id) {
            tracing::warn!(
                "no queue config for input `{event_id}`, using default size {DEFAULT_QUEUE_SIZE}"
            );
            self.last_used.push_back(event_id.clone());
            self.event_queues
                .insert(event_id.clone(), (DEFAULT_QUEUE_SIZE, Default::default()));
        }
        let Some((size, queue)) = self.event_queues.get_mut(event_id) else {
            // Unreachable: the entry was just inserted above when missing.
            return;
        };

        let policy = self
            .queue_policies
            .get(event_id)
            .copied()
            .unwrap_or_default();

        let cap = policy.effective_cap(*size);
        if queue.len() >= cap {
            if policy == QueuePolicy::Backpressure {
                tracing::error!(
                    "Backpressure input `{event_id}` hit hard cap ({cap}), \
                     dropping oldest to prevent OOM"
                );
            } else {
                tracing::warn!("Discarding event for input `{event_id}` due to queue size limit");
            }
            *self.dropped.entry(event_id.clone()).or_insert(0) += 1;
            match select_eviction(queue, &event) {
                Eviction::RemoveAt(idx) => {
                    queue.remove(idx);
                }
                Eviction::DropIncoming => {
                    // Queue is entirely correlated; preserve correlations
                    // by dropping the incoming (non-correlated) event.
                    return;
                }
                Eviction::DropCorrelatedLoud(idx) => {
                    if let Some(dropped) = queue.remove(idx) {
                        log_correlation_drop(event_id, &dropped);
                    }
                }
            }
        }
        queue.push_back(event);
    }

    pub(crate) fn next(&mut self) -> Option<EventItem> {
        // Retrieve message from the non input event first that have priority over input message.
        if let Some((_size, queue)) = self.event_queues.get_mut(&*NON_INPUT_EVENT_ID)
            && let Some(event) = queue.pop_front()
        {
            return Some(event);
        }

        // Yield from the first non-empty input queue in least-recently-used
        // order: `last_used` is a VecDeque of input IDs, and the ID we serve
        // from is rotated to the back below so the others get a turn next.
        for index in 0..self.last_used.len() {
            let id = &self.last_used[index];
            if let Some((_size, queue)) = self.event_queues.get_mut(id)
                && let Some(event) = queue.pop_front()
            {
                // Put last used at last
                if let Some(id) = self.last_used.remove(index) {
                    self.last_used.push_back(id);
                }
                return Some(event);
            }
        }

        None
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.event_queues
            .iter()
            .all(|(_id, (_size, queue))| queue.is_empty())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uhlc;
    use dora_message::{
        daemon_to_node::NodeEvent,
        metadata::{FLUSH, Metadata, MetadataParameters, Parameter},
    };

    fn make_input(id: &str, params: MetadataParameters) -> EventItem {
        let ts = uhlc::HLC::default().new_timestamp();
        let metadata = Metadata::from_parameters(ts, params);
        EventItem::NodeEvent {
            event: NodeEvent::Input {
                id: DataId::from(id.to_string()),
                metadata: std::sync::Arc::new(metadata),
                data: None,
            },
        }
    }

    fn make_stop() -> EventItem {
        EventItem::NodeEvent {
            event: NodeEvent::Stop,
        }
    }

    fn make_input_closed(id: &str) -> EventItem {
        EventItem::NodeEvent {
            event: NodeEvent::InputClosed {
                id: DataId::from(id.to_string()),
            },
        }
    }

    fn make_scheduler(audio_capacity: usize) -> (Scheduler, DataId) {
        let id = DataId::from("audio".to_string());
        let mut queues = HashMap::new();
        queues.insert(id.clone(), (audio_capacity, VecDeque::new()));
        queues.insert(
            DataId::from(NON_INPUT_EVENT.to_string()),
            (10, VecDeque::new()),
        );
        (Scheduler::with_policies(queues, HashMap::new()), id)
    }

    #[test]
    fn flush_clears_older_queued_events() {
        let (mut sched, id) = make_scheduler(10);

        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        assert_eq!(sched.event_queues[&id].1.len(), 3);

        // Flush should clear the 3 older events, then insert itself
        let mut flush_params = MetadataParameters::new();
        flush_params.insert(FLUSH.into(), Parameter::Bool(true));
        sched.add_event(make_input("audio", flush_params));

        assert_eq!(sched.event_queues[&id].1.len(), 1);
    }

    #[test]
    fn non_flush_does_not_clear_queue() {
        let (mut sched, id) = make_scheduler(10);

        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        assert_eq!(sched.event_queues[&id].1.len(), 3);
    }

    #[test]
    fn flush_false_does_not_clear_queue() {
        let (mut sched, id) = make_scheduler(10);

        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));

        let mut params = MetadataParameters::new();
        params.insert(FLUSH.into(), Parameter::Bool(false));
        sched.add_event(make_input("audio", params));

        assert_eq!(sched.event_queues[&id].1.len(), 3);
    }

    #[test]
    fn flush_with_queue_size_one_retains_flush_message() {
        let (mut sched, id) = make_scheduler(1);

        sched.add_event(make_input("audio", MetadataParameters::new()));
        assert_eq!(sched.event_queues[&id].1.len(), 1);

        // Flush clears the queue, then the flush message itself is inserted
        let mut flush_params = MetadataParameters::new();
        flush_params.insert(FLUSH.into(), Parameter::Bool(true));
        sched.add_event(make_input("audio", flush_params));

        // The flush message should survive (queue was cleared to 0, then inserted)
        assert_eq!(sched.event_queues[&id].1.len(), 1);
    }

    #[test]
    fn drop_oldest_tracks_drop_count() {
        let (mut sched, id) = make_scheduler(2);

        // Fill to capacity
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        assert_eq!(sched.event_queues[&id].1.len(), 2);

        // Overflow by 3 more
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));

        // Queue stays at capacity
        assert_eq!(sched.event_queues[&id].1.len(), 2);

        // 3 drops counted
        let counts = sched.drain_drop_counts();
        assert_eq!(counts.get(&id), Some(&3));

        // After drain, counts reset
        let counts = sched.drain_drop_counts();
        assert!(counts.is_empty());
    }

    // ---- dora-rs/adora#146: flush: true must not wipe correlated messages ----

    #[test]
    fn flush_retains_correlated_events() {
        // Queue holds a service response (request_id) and two stream chunks.
        // A flush signal should drop the stream chunks but keep the response.
        let (mut sched, id) = make_scheduler(10);

        sched.add_event(make_input("audio", with_request_id("req-1")));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        assert_eq!(sched.event_queues[&id].1.len(), 3);

        let mut flush_params = MetadataParameters::new();
        flush_params.insert(FLUSH.into(), Parameter::Bool(true));
        sched.add_event(make_input("audio", flush_params));

        let queue = &sched.event_queues[&id].1;
        // Expect: [req-1, flush_message]
        assert_eq!(queue.len(), 2);
        assert!(
            queue
                .iter()
                .any(|e| request_id_of(e).as_deref() == Some("req-1")),
            "service response with request_id was wiped by flush"
        );
    }

    #[test]
    fn flush_retains_goal_id_events() {
        // Same preservation via goal_id.
        let (mut sched, id) = make_scheduler(10);

        let mut goal_params = MetadataParameters::new();
        goal_params.insert(GOAL_ID.into(), Parameter::String("goal-7".to_string()));
        sched.add_event(make_input("audio", goal_params));
        sched.add_event(make_input("audio", MetadataParameters::new()));

        let mut flush_params = MetadataParameters::new();
        flush_params.insert(FLUSH.into(), Parameter::Bool(true));
        sched.add_event(make_input("audio", flush_params));

        let queue = &sched.event_queues[&id].1;
        // Expect: [goal-7, flush_message]
        assert_eq!(queue.len(), 2);
        let has_goal = queue.iter().any(|e| {
            let EventItem::NodeEvent {
                event: NodeEvent::Input { metadata, .. },
                ..
            } = e
            else {
                return false;
            };
            get_string_param(&metadata.parameters, GOAL_ID) == Some("goal-7")
        });
        assert!(has_goal, "action result with goal_id was wiped by flush");
    }

    #[test]
    fn flush_with_all_correlated_queue_keeps_everything() {
        // All queued events are correlations. Flush should preserve them all,
        // then admit the flush message itself.
        let (mut sched, id) = make_scheduler(10);

        sched.add_event(make_input("audio", with_request_id("req-1")));
        sched.add_event(make_input("audio", with_request_id("req-2")));
        sched.add_event(make_input("audio", with_request_id("req-3")));

        let mut flush_params = MetadataParameters::new();
        flush_params.insert(FLUSH.into(), Parameter::Bool(true));
        sched.add_event(make_input("audio", flush_params));

        // Expect: [req-1, req-2, req-3, flush_message]
        assert_eq!(sched.event_queues[&id].1.len(), 4);
    }

    // ---- dora-rs/adora#145: drop_oldest must not silently drop correlated messages ----

    /// Helper: extract request_id from an event's metadata, if any.
    fn request_id_of(event: &EventItem) -> Option<String> {
        let EventItem::NodeEvent {
            event: NodeEvent::Input { metadata, .. },
            ..
        } = event
        else {
            return None;
        };
        get_string_param(&metadata.parameters, REQUEST_ID).map(|s| s.to_string())
    }

    fn with_request_id(id: &str) -> MetadataParameters {
        let mut params = MetadataParameters::new();
        params.insert(REQUEST_ID.into(), Parameter::String(id.to_string()));
        params
    }

    #[test]
    fn drop_oldest_preserves_correlated_when_non_correlated_present() {
        // Queue has [correlated(req-1), non-correlated, non-correlated]
        // Adding one more should drop a non-correlated event, not req-1.
        let (mut sched, id) = make_scheduler(3);

        sched.add_event(make_input("audio", with_request_id("req-1")));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));

        let queue = &sched.event_queues[&id].1;
        assert_eq!(queue.len(), 3);
        // req-1 must still be somewhere in the queue
        assert!(
            queue
                .iter()
                .any(|e| request_id_of(e).as_deref() == Some("req-1")),
            "correlated message was dropped even though non-correlated events were available"
        );
    }

    #[test]
    fn drop_oldest_drops_middle_non_correlated_to_save_front_correlated() {
        // Queue: [req-1 (correlated), B, req-2 (correlated)]
        // Adding C should drop B (the only non-correlated), not req-1.
        let (mut sched, id) = make_scheduler(3);

        sched.add_event(make_input("audio", with_request_id("req-1")));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", with_request_id("req-2")));
        sched.add_event(make_input("audio", MetadataParameters::new()));

        let queue = &sched.event_queues[&id].1;
        assert_eq!(queue.len(), 3);
        assert!(
            queue
                .iter()
                .any(|e| request_id_of(e).as_deref() == Some("req-1"))
        );
        assert!(
            queue
                .iter()
                .any(|e| request_id_of(e).as_deref() == Some("req-2"))
        );
    }

    #[test]
    fn drop_oldest_drops_incoming_if_queue_is_fully_correlated_and_incoming_is_not() {
        // Queue: [req-1, req-2] (both correlated). Incoming is non-correlated.
        // The correlations must survive; incoming gets dropped instead.
        let (mut sched, id) = make_scheduler(2);

        sched.add_event(make_input("audio", with_request_id("req-1")));
        sched.add_event(make_input("audio", with_request_id("req-2")));
        sched.add_event(make_input("audio", MetadataParameters::new()));

        let queue = &sched.event_queues[&id].1;
        assert_eq!(queue.len(), 2);
        let ids: Vec<_> = queue.iter().filter_map(request_id_of).collect();
        assert_eq!(ids, vec!["req-1".to_string(), "req-2".to_string()]);

        // Drop counter still increments — we rejected a message.
        let counts = sched.drain_drop_counts();
        assert_eq!(counts.get(&id), Some(&1));
    }

    #[test]
    fn drop_oldest_drops_front_loudly_when_both_queue_and_incoming_are_correlated() {
        // Queue: [req-1, req-2] (both correlated). Incoming is req-3.
        // Unavoidable drop — the oldest correlation (req-1) is evicted.
        let (mut sched, id) = make_scheduler(2);

        sched.add_event(make_input("audio", with_request_id("req-1")));
        sched.add_event(make_input("audio", with_request_id("req-2")));
        sched.add_event(make_input("audio", with_request_id("req-3")));

        let queue = &sched.event_queues[&id].1;
        assert_eq!(queue.len(), 2);
        let ids: Vec<_> = queue.iter().filter_map(request_id_of).collect();
        assert_eq!(ids, vec!["req-2".to_string(), "req-3".to_string()]);
    }

    #[test]
    fn drop_oldest_goal_id_is_also_preserved() {
        // Same preservation as request_id, but via goal_id.
        let (mut sched, id) = make_scheduler(2);

        let mut goal_params = MetadataParameters::new();
        goal_params.insert(GOAL_ID.into(), Parameter::String("goal-42".to_string()));

        sched.add_event(make_input("audio", goal_params));
        sched.add_event(make_input("audio", MetadataParameters::new()));
        sched.add_event(make_input("audio", MetadataParameters::new()));

        let queue = &sched.event_queues[&id].1;
        assert_eq!(queue.len(), 2);
        let has_goal = queue.iter().any(|e| {
            let EventItem::NodeEvent {
                event: NodeEvent::Input { metadata, .. },
                ..
            } = e
            else {
                return false;
            };
            get_string_param(&metadata.parameters, GOAL_ID) == Some("goal-42")
        });
        assert!(
            has_goal,
            "goal-42 was dropped despite having non-correlated events to drop"
        );
    }

    // The non-input queue holds every lifecycle event (`Stop`, `InputClosed`,
    // `Error`, ...) under one cap. Overflow must never evict the `Stop`
    // shutdown signal: dropping it makes the node miss shutdown and run until
    // the daemon force-kills it. A `Stop` buffered while the node is busy must
    // survive a flood of other non-input events that overflows the queue many
    // times over.
    #[test]
    fn stop_survives_non_input_queue_overflow() {
        // `make_scheduler` gives the non-input queue a cap of 10.
        let (mut sched, _id) = make_scheduler(10);

        sched.add_event(make_stop());
        for i in 0..100 {
            sched.add_event(make_input_closed(&format!("in-{i}")));
        }

        let non_input = &sched.event_queues[&*NON_INPUT_EVENT_ID].1;
        assert_eq!(non_input.len(), 10, "non-input queue must stay bounded");
        assert!(
            non_input.iter().any(is_stop),
            "the Stop event must survive non-input queue overflow"
        );
    }

    // The incoming event is a `Stop` and the queue is already full of ordinary
    // non-input events: the `Stop` must be admitted (evicting an ordinary
    // event), not dropped as the overflow victim.
    #[test]
    fn incoming_stop_is_admitted_into_a_full_non_input_queue() {
        let (mut sched, _id) = make_scheduler(10);

        for i in 0..10 {
            sched.add_event(make_input_closed(&format!("in-{i}")));
        }
        sched.add_event(make_stop());

        let non_input = &sched.event_queues[&*NON_INPUT_EVENT_ID].1;
        assert_eq!(non_input.len(), 10, "non-input queue must stay bounded");
        assert!(
            non_input.iter().any(is_stop),
            "an incoming Stop must be admitted into a full non-input queue"
        );
    }

    // The flush path (`retain`) is a second eviction site. It normally targets
    // a per-input queue, but an input literally named `dora.non_input_event`
    // (which `validate_data_id` permits) collides with the queue where `Stop`
    // lives. Flush must still preserve the `Stop` shutdown signal there.
    #[test]
    fn flush_retains_stop_when_targeting_the_non_input_queue() {
        let (mut sched, _id) = make_scheduler(10);

        sched.add_event(make_stop());
        sched.add_event(make_input_closed("x"));

        let mut flush_params = MetadataParameters::new();
        flush_params.insert(FLUSH.into(), Parameter::Bool(true));
        sched.add_event(make_input(NON_INPUT_EVENT, flush_params));

        let non_input = &sched.event_queues[&*NON_INPUT_EVENT_ID].1;
        assert!(
            non_input.iter().any(is_stop),
            "flush must not evict the Stop shutdown signal"
        );
    }

    #[test]
    fn backpressure_policy_prevents_drops() {
        let id = DataId::from("commands".to_string());
        let mut queues = HashMap::new();
        queues.insert(id.clone(), (2, VecDeque::new()));
        queues.insert(
            DataId::from(NON_INPUT_EVENT.to_string()),
            (10, VecDeque::new()),
        );
        let policies = HashMap::from([(id.clone(), QueuePolicy::Backpressure)]);
        let mut sched = Scheduler::with_policies(queues, policies);

        // Fill past capacity — backpressure should let queue grow
        sched.add_event(make_input("commands", MetadataParameters::new()));
        sched.add_event(make_input("commands", MetadataParameters::new()));
        sched.add_event(make_input("commands", MetadataParameters::new()));
        sched.add_event(make_input("commands", MetadataParameters::new()));

        // Queue grew beyond configured size (no drops)
        assert_eq!(sched.event_queues[&id].1.len(), 4);

        // Zero drops
        let counts = sched.drain_drop_counts();
        assert!(counts.is_empty());
    }

    // ---- issue #2212: log_correlation_drop must also fire for ZenohInput ----

    fn make_zenoh_input(id: &str, params: MetadataParameters) -> EventItem {
        let ts = uhlc::HLC::default().new_timestamp();
        let metadata = Metadata::from_parameters(ts, params);
        use dora_arrow_convert::IntoArrow;
        EventItem::ZenohInput {
            id: DataId::from(id.to_string()),
            metadata: std::sync::Arc::new(metadata),
            data: ().into_arrow().into(),
        }
    }

    /// Helper: extract request_id from a ZenohInput event's metadata, if any.
    fn request_id_of_zenoh(event: &EventItem) -> Option<String> {
        let EventItem::ZenohInput { metadata, .. } = event else {
            return None;
        };
        get_string_param(&metadata.parameters, REQUEST_ID).map(|s| s.to_string())
    }

    #[test]
    fn zenoh_drop_oldest_drops_front_loudly_when_both_queue_and_incoming_are_correlated() {
        // Mirrors `drop_oldest_drops_front_loudly_when_both_queue_and_incoming_are_correlated`
        // but uses ZenohInput items, exercising the second match arm of
        // `log_correlation_drop` (issue #2212).
        //
        // Queue: [req-1, req-2] (both ZenohInput + correlated). Incoming is req-3.
        // Unavoidable drop — the oldest correlation (req-1) is evicted.
        let (mut sched, id) = make_scheduler(2);

        sched.add_event(make_zenoh_input("audio", with_request_id("req-1")));
        sched.add_event(make_zenoh_input("audio", with_request_id("req-2")));
        sched.add_event(make_zenoh_input("audio", with_request_id("req-3")));

        let queue = &sched.event_queues[&id].1;
        assert_eq!(queue.len(), 2);
        let ids: Vec<_> = queue.iter().filter_map(request_id_of_zenoh).collect();
        assert_eq!(ids, vec!["req-2".to_string(), "req-3".to_string()]);
    }

    #[test]
    fn zenoh_drop_oldest_preserves_correlated_when_non_correlated_present() {
        // ZenohInput mirror of `drop_oldest_preserves_correlated_when_non_correlated_present`.
        // A correlated ZenohInput must not be evicted when non-correlated items are available.
        let (mut sched, id) = make_scheduler(3);

        sched.add_event(make_zenoh_input("audio", with_request_id("req-1")));
        sched.add_event(make_zenoh_input("audio", MetadataParameters::new()));
        sched.add_event(make_zenoh_input("audio", MetadataParameters::new()));
        sched.add_event(make_zenoh_input("audio", MetadataParameters::new()));

        let queue = &sched.event_queues[&id].1;
        assert_eq!(queue.len(), 3);
        assert!(
            queue
                .iter()
                .any(|e| request_id_of_zenoh(e).as_deref() == Some("req-1")),
            "correlated ZenohInput was dropped even though non-correlated events were available"
        );
    }
}
