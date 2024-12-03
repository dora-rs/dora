use std::collections::{HashMap, VecDeque};

use dora_message::{daemon_to_node::NodeEvent, id::DataId};

use super::thread::EventItem;

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
    event_queues: HashMap<DataId, VecDeque<EventItem>>, // Tracks events per ID
    queue_limit: HashMap<DataId, usize>, // Maximum size of the queue per ID
}

impl Scheduler {
    pub fn new(queue_limit: HashMap<DataId, usize>) -> Self {
        Self {
            last_used: VecDeque::new(),
            event_queues: HashMap::new(),
            queue_limit: queue_limit,
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
            } => id.clone(),
            _ => DataId::from("non_input_event".to_string()),
        };

        // Enforce queue size limit
        if let Some(queue) = self.event_queues.get_mut(&event_id) {
            if &queue.len() >= self.queue_limit.get(&event_id).unwrap_or(&1) {
                queue.pop_front(); // Remove the oldest event if at limit
            }
            queue.push_back(event);
        } else {
            self.event_queues
                .insert(event_id.clone(), VecDeque::from([event]));
            self.last_used.push_front(event_id.clone());
        }
    }

    pub fn next(&mut self) -> Option<EventItem> {
        // Process the ID with the oldest timestamp using BTreMap Ordering

        for (index, id) in self.last_used.clone().iter().enumerate() {
            if let Some(queue) = self.event_queues.get_mut(id) {
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
            .all(|(_id, queue)| queue.is_empty())
    }
}
