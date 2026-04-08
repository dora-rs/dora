use std::collections::{HashMap, VecDeque};

use adora_message::{
    config::{DEFAULT_QUEUE_SIZE, QueuePolicy},
    daemon_to_node::NodeEvent,
    id::DataId,
};

use super::thread::EventItem;
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

        // Flush older queued messages when flush=true is present
        if should_flush && let Some((_size, queue)) = self.event_queues.get_mut(event_id) {
            let drained = queue.len();
            queue.clear();
            if drained > 0 {
                tracing::debug!(
                    "Flushed {drained} queued event(s) for input `{event_id}` (flush signal)"
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
            queue.pop_front();
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
