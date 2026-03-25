use std::collections::HashMap;

use adora_arrow_convert::ArrowData;
use adora_core::config::DataId;

use super::event::Event;

/// Tracks the health state and last known value of each input.
///
/// Use this helper to implement graceful degradation when upstream nodes
/// time out (via `input_timeout`). It caches the last received [`ArrowData`]
/// per input so your node can fall back to stale data instead of crashing.
///
/// The cache is bounded by the number of distinct input IDs declared in the
/// dataflow YAML. Since input IDs are fixed at dataflow compile time, the
/// cache cannot grow unboundedly.
///
/// # Example
///
/// ```ignore
/// let mut tracker = InputTracker::new();
/// while let Some(event) = events.recv().await {
///     tracker.process_event(&event);
///     match event {
///         Event::Input { id, data, .. } => { /* use fresh data */ }
///         Event::InputClosed { id } => {
///             if let Some(stale) = tracker.last_value(&id) {
///                 /* degrade gracefully using cached value */
///             }
///         }
///         _ => {}
///     }
/// }
/// ```
pub struct InputTracker {
    states: HashMap<DataId, InputState>,
    cache: HashMap<DataId, ArrowData>,
}

/// Health state of a tracked input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputState {
    /// The input is receiving data normally.
    Healthy,
    /// The input was closed (upstream exited or timed out).
    Closed,
}

impl InputTracker {
    /// Create a new, empty tracker.
    pub fn new() -> Self {
        Self {
            states: HashMap::new(),
            cache: HashMap::new(),
        }
    }

    /// Update tracker state from an event. Returns `true` if the event
    /// was relevant to input tracking (Input, InputClosed, or InputRecovered).
    pub fn process_event(&mut self, event: &Event) -> bool {
        match event {
            Event::Input { id, data, .. } => {
                self.states.insert(id.clone(), InputState::Healthy);
                self.cache.insert(id.clone(), ArrowData(data.0.clone()));
                true
            }
            Event::InputClosed { id } => {
                self.states.insert(id.clone(), InputState::Closed);
                // Cache preserved -- node can still read last_value
                true
            }
            Event::InputRecovered { id } => {
                self.states.insert(id.clone(), InputState::Healthy);
                true
            }
            _ => false,
        }
    }

    /// Get the current state of an input, if tracked.
    pub fn state(&self, id: &DataId) -> Option<InputState> {
        self.states.get(id).copied()
    }

    /// Check if an input is currently closed (timed out or upstream exited).
    pub fn is_closed(&self, id: &DataId) -> bool {
        self.states.get(id) == Some(&InputState::Closed)
    }

    /// Get the last received value for an input. Available even when closed.
    pub fn last_value(&self, id: &DataId) -> Option<&ArrowData> {
        self.cache.get(id)
    }

    /// Return all inputs that are currently closed.
    pub fn closed_inputs(&self) -> Vec<&DataId> {
        self.states
            .iter()
            .filter(|(_, s)| **s == InputState::Closed)
            .map(|(id, _)| id)
            .collect()
    }

    /// Check if any tracked input is currently closed.
    pub fn any_closed(&self) -> bool {
        self.states.values().any(|s| *s == InputState::Closed)
    }
}

impl Default for InputTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use adora_message::metadata::{ArrowTypeInfo, Metadata};
    use arrow::array::new_empty_array;
    use arrow::datatypes::DataType;

    fn empty_data() -> ArrowData {
        ArrowData(new_empty_array(&DataType::Null))
    }

    fn test_metadata() -> Metadata {
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
        Metadata::new(adora_core::uhlc::HLC::default().new_timestamp(), type_info)
    }

    fn make_input(id: &str, data: ArrowData) -> Event {
        Event::Input {
            id: id.into(),
            metadata: test_metadata(),
            data,
        }
    }

    #[test]
    fn tracks_healthy_input() {
        let mut t = InputTracker::new();
        assert!(t.process_event(&make_input("a", empty_data())));
        assert_eq!(t.state(&"a".into()), Some(InputState::Healthy));
        assert!(!t.is_closed(&"a".into()));
        assert!(t.last_value(&"a".into()).is_some());
    }

    #[test]
    fn tracks_closed_preserves_cache() {
        let mut t = InputTracker::new();
        t.process_event(&make_input("a", empty_data()));
        t.process_event(&Event::InputClosed { id: "a".into() });

        assert_eq!(t.state(&"a".into()), Some(InputState::Closed));
        assert!(t.is_closed(&"a".into()));
        assert!(t.last_value(&"a".into()).is_some());
        assert!(t.any_closed());
        assert_eq!(t.closed_inputs().len(), 1);
    }

    #[test]
    fn tracks_recovery() {
        let mut t = InputTracker::new();
        t.process_event(&make_input("a", empty_data()));
        t.process_event(&Event::InputClosed { id: "a".into() });
        t.process_event(&Event::InputRecovered { id: "a".into() });

        assert_eq!(t.state(&"a".into()), Some(InputState::Healthy));
        assert!(!t.any_closed());
    }

    #[test]
    fn ignores_irrelevant_events() {
        let mut t = InputTracker::new();
        assert!(!t.process_event(&Event::Stop(super::super::event::StopCause::Manual)));
    }
}
