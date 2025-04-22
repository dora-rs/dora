use std::collections::{HashMap, VecDeque};

use dora_message::{daemon_to_node::NodeEvent, id::DataId};

use super::thread::EventItem;
pub const NON_INPUT_EVENT: &str = "dora/non_input_event";

// This scheduler will make sure that there is fairness between
// inputs.
//
// It is going to always make sure that the input that has not been used for the longest period is the first one to be used next.
//
// Ex:
// In the case that one input has a very high frequency and another one with a very slow frequency.\
//
// The Node will always alternate between the two inputs when each input is available
// Avoiding one input to be overwhelmingly present.
//
#[derive(Debug)]
pub struct Scheduler {
    last_used: VecDeque<DataId>, // Tracks the last-used event ID
    event_queues: HashMap<DataId, (usize, VecDeque<EventItem>)>, // Tracks events per ID
}

impl Scheduler {
    pub fn new(event_queues: HashMap<DataId, (usize, VecDeque<EventItem>)>) -> Self {
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

    pub fn add_event(&mut self, event: EventItem) {
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

    pub fn next(&mut self) -> Option<EventItem> {
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

    pub fn is_empty(&self) -> bool {
        self.event_queues
            .iter()
            .all(|(_id, (_size, queue))| queue.is_empty())
    }
}
