use std::collections::{HashMap, VecDeque};

use adora_message::{
    config::{DEFAULT_QUEUE_SIZE, QueuePolicy},
    daemon_to_node::NodeEvent,
    id::DataId,
    metadata::{GOAL_ID, GOAL_STATUS, REQUEST_ID, get_string_param},
};

use super::thread::EventItem;

/// Returns `true` if the event carries request/response or action correlation
/// metadata (`request_id`, `goal_id`, or `goal_status`).
///
/// These keys bind a message to a specific service request or action goal.
/// Silently dropping such a message breaks the correlation contract — the
/// client waits forever for a response or result that never arrives
/// (dora-rs/adora#145).
fn is_correlated(event: &EventItem) -> bool {
    let EventItem::NodeEvent {
        event: NodeEvent::Input { metadata, .. },
        ..
    } = event
    else {
        return false;
    };
    let params = &metadata.parameters;
    params.contains_key(REQUEST_ID)
        || params.contains_key(GOAL_ID)
        || params.contains_key(GOAL_STATUS)
}

/// Outcome of `select_eviction`.
enum Eviction {
    /// Remove event at this index from the queue and push the incoming event.
    RemoveAt(usize),
    /// The queue is entirely correlated and the incoming event is not —
    /// drop the incoming event instead of breaking a correlation.
    DropIncoming,
    /// The queue is entirely correlated and the incoming event is also
    /// correlated — drop the oldest (front) event with a loud error log.
    DropFrontLoud,
}

/// Choose which event to drop when the queue is at capacity.
///
/// Prefers sacrificing non-correlated events so that service responses and
/// action results survive. See `is_correlated` for the metadata keys that
/// mark an event as part of a pattern.
fn select_eviction(queue: &VecDeque<EventItem>, incoming: &EventItem) -> Eviction {
    if let Some(idx) = queue.iter().position(|e| !is_correlated(e)) {
        return Eviction::RemoveAt(idx);
    }
    if !is_correlated(incoming) {
        return Eviction::DropIncoming;
    }
    Eviction::DropFrontLoud
}

/// Emit a loud error when a correlated event has to be dropped because
/// everything in the queue is also correlated. Identifies the correlation
/// keys so operators can trace the affected request/goal.
fn log_correlation_drop(event_id: &DataId, dropped: &EventItem) {
    let EventItem::NodeEvent {
        event: NodeEvent::Input { metadata, .. },
        ..
    } = dropped
    else {
        return;
    };
    let params = &metadata.parameters;
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
pub(crate) const NON_INPUT_EVENT: &str = "adora.non_input_event";

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
                .filter(|t| **t != DataId::from(NON_INPUT_EVENT.to_string()))
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
                let flush = adora_message::metadata::get_bool_param(
                    &metadata.parameters,
                    adora_message::metadata::FLUSH,
                ) == Some(true);
                (id, flush)
            }
            _ => (&DataId::from(NON_INPUT_EVENT.to_string()), false),
        };

        // Flush older queued messages when flush=true is present.
        //
        // Streaming pattern's `flush: true` means "discard stale stream chunks".
        // It must NOT wipe service responses or action results that happen to
        // share the same input, because those carry `request_id` / `goal_id` /
        // `goal_status` correlations whose senders are waiting for them
        // (dora-rs/adora#146). Use the same correlation predicate that the
        // drop_oldest path uses and retain correlated events across the flush.
        if should_flush && let Some((_size, queue)) = self.event_queues.get_mut(event_id) {
            let before = queue.len();
            queue.retain(is_correlated);
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

        // Enforce queue size limit
        let (size, queue) = self
            .event_queues
            .entry(event_id.clone())
            .or_insert_with(|| {
                tracing::warn!(
                    "no queue config for input `{event_id}`, using default size {DEFAULT_QUEUE_SIZE}"
                );
                self.last_used.push_back(event_id.clone());
                (DEFAULT_QUEUE_SIZE, Default::default())
            });

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
                Eviction::DropFrontLoud => {
                    if let Some(front) = queue.pop_front() {
                        log_correlation_drop(event_id, &front);
                    }
                }
            }
        }
        queue.push_back(event);
    }

    pub(crate) fn next(&mut self) -> Option<EventItem> {
        // Retrieve message from the non input event first that have priority over input message.
        if let Some((_size, queue)) = self
            .event_queues
            .get_mut(&DataId::from(NON_INPUT_EVENT.to_string()))
            && let Some(event) = queue.pop_front()
        {
            return Some(event);
        }

        // Process the ID with the oldest timestamp using BTreeMap Ordering
        for (index, id) in self.last_used.clone().iter().enumerate() {
            if let Some((_size, queue)) = self.event_queues.get_mut(id)
                && let Some(event) = queue.pop_front()
            {
                // Put last used at last
                self.last_used.remove(index);
                self.last_used.push_back(id.clone());
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
    use adora_message::{
        daemon_to_node::NodeEvent,
        metadata::{ArrowTypeInfo, FLUSH, Metadata, MetadataParameters, Parameter},
    };
    use arrow_schema::DataType;

    fn make_input(id: &str, params: MetadataParameters) -> EventItem {
        let type_info = ArrowTypeInfo {
            data_type: DataType::Null,
            len: 0,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![],
            child_data: vec![],
            field_names: None,
            schema_hash: None,
        };
        let ts = uhlc::HLC::default().new_timestamp();
        let metadata = Metadata::from_parameters(ts, type_info, params);
        let (tx, _rx) = flume::bounded(1);
        EventItem::NodeEvent {
            event: NodeEvent::Input {
                id: DataId::from(id.to_string()),
                metadata: std::sync::Arc::new(metadata),
                data: None,
            },
            ack_channel: tx,
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
}
