use std::collections::{HashMap, VecDeque};

use dora_message::{daemon_to_node::NodeEvent, id::DataId};

use super::thread::EventItem;
pub(crate) const NON_INPUT_EVENT: &str = "dora/non_input_event";

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
}

impl Scheduler {
    pub(crate) fn new(event_queues: HashMap<DataId, (usize, VecDeque<EventItem>)>) -> Self {
        let topic = VecDeque::from_iter(
            event_queues
                .keys()
                .filter(|t| **t != DataId::from(NON_INPUT_EVENT.to_string()))
                .cloned(),
        );
        Self {
            last_used: topic,
            event_queues,
        }
    }

    pub(crate) fn add_event(&mut self, event: EventItem) {
        let event_id = match &event {
            EventItem::NodeEvent {
                event:
                    NodeEvent::Input {
                        id,
                        metadata: _,
                        data: _,
                    },
                ack_channel: _,
            } => id,
            _ => &DataId::from(NON_INPUT_EVENT.to_string()),
        };

        // Enforce queue size limit
        if let Some((size, queue)) = self.event_queues.get_mut(event_id) {
            // Remove the oldest event if at limit
            if &queue.len() >= size {
                tracing::debug!("Discarding event for input `{event_id}` due to queue size limit");
                queue.pop_front();
            }
            queue.push_back(event);
        } else {
            unimplemented!("Received an event that was not in the definition event id description.")
        }
    }

    pub(crate) fn next(&mut self) -> Option<EventItem> {
        // Retrieve message from the non input event first that have priority over input message.
        if let Some((_size, queue)) = self
            .event_queues
            .get_mut(&DataId::from(NON_INPUT_EVENT.to_string()))
        {
            if let Some(event) = queue.pop_front() {
                return Some(event);
            }
        }

        // Process the ID with the oldest timestamp using BTreeMap Ordering
        for (index, id) in self.last_used.clone().iter().enumerate() {
            if let Some((_size, queue)) = self.event_queues.get_mut(id) {
                if let Some(event) = queue.pop_front() {
                    // Put last used at last
                    self.last_used.remove(index);
                    self.last_used.push_back(id.clone());
                    return Some(event);
                }
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
