use std::collections::HashMap;

use dora_arrow_convert::ArrowData;
use dora_core::config::{DataId, NodeId};

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
    /// Optional input → source node map. When provided, `NodeRestarted`
    /// events transition any `Closed` inputs sourced from the restarted
    /// node back to `Healthy`. Without it the tracker has no way to know
    /// which inputs a given upstream node feeds, so `NodeRestarted`
    /// leaves state unchanged (dora-rs/adora#148).
    source_map: HashMap<DataId, NodeId>,
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
    ///
    /// Without a source map, `NodeRestarted` events are acknowledged
    /// (return `true` from `process_event`) but do not mutate state,
    /// because the tracker cannot determine which inputs a given node
    /// feeds. If you use fault-tolerance restart policies, prefer
    /// [`InputTracker::with_source_map`] so closed inputs recover
    /// automatically when the upstream comes back up.
    pub fn new() -> Self {
        Self {
            states: HashMap::new(),
            cache: HashMap::new(),
            source_map: HashMap::new(),
        }
    }

    /// Create a tracker that knows which input is fed by which upstream
    /// node. With this map populated, `NodeRestarted { id }` events
    /// transition any `Closed` inputs sourced from `id` back to `Healthy`.
    ///
    /// The map can be built from the dataflow's `input_config` at
    /// initialization time — for each user input `(data_id, source_node)`
    /// insert the pair here.
    pub fn with_source_map(source_map: HashMap<DataId, NodeId>) -> Self {
        Self {
            states: HashMap::new(),
            cache: HashMap::new(),
            source_map,
        }
    }

    /// Update tracker state from an event. Returns `true` if the event
    /// was relevant to input tracking (`Input`, `InputClosed`,
    /// `InputRecovered`, or `NodeRestarted`).
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
            Event::NodeRestarted { id: restarted } => {
                // Transition any closed inputs fed by the restarted node
                // back to Healthy. Cached last_value is preserved so the
                // node can still fall back to stale data between restart
                // and first post-restart message.
                for (input_id, source) in &self.source_map {
                    if source == restarted && self.states.get(input_id) == Some(&InputState::Closed)
                    {
                        self.states.insert(input_id.clone(), InputState::Healthy);
                    }
                }
                // Always return true to signal relevance even when the
                // tracker has no source map: callers inspecting the
                // boolean can still detect that a lifecycle event arrived.
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
    use arrow::array::new_empty_array;
    use arrow::datatypes::DataType;
    use dora_message::metadata::{ArrowTypeInfo, Metadata};

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
        Metadata::new(dora_core::uhlc::HLC::default().new_timestamp(), type_info)
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

    // ---- dora-rs/adora#148: NodeRestarted handling ----

    #[test]
    fn node_restarted_without_source_map_is_acknowledged_but_noop() {
        // Without a source map the tracker has no way to know which inputs
        // the restarted node feeds, so state is unchanged — but the event
        // must still be reported as "relevant" (returns true) so callers
        // that inspect the boolean don't treat the lifecycle signal as noise.
        let mut t = InputTracker::new();
        t.process_event(&make_input("a", empty_data()));
        t.process_event(&Event::InputClosed { id: "a".into() });
        assert!(t.is_closed(&"a".into()));

        let relevant = t.process_event(&Event::NodeRestarted {
            id: NodeId::from("upstream".to_string()),
        });
        assert!(relevant, "NodeRestarted should be reported as relevant");
        // State untouched: no source map, no safe way to transition.
        assert!(t.is_closed(&"a".into()));
    }

    #[test]
    fn node_restarted_with_source_map_recovers_matching_closed_inputs() {
        let mut source_map = HashMap::new();
        source_map.insert(
            DataId::from("sensor".to_string()),
            NodeId::from("camera".to_string()),
        );
        source_map.insert(
            DataId::from("telemetry".to_string()),
            NodeId::from("camera".to_string()),
        );
        source_map.insert(
            DataId::from("config".to_string()),
            NodeId::from("other".to_string()),
        );
        let mut t = InputTracker::with_source_map(source_map);

        // Close all three inputs.
        t.process_event(&Event::InputClosed {
            id: "sensor".into(),
        });
        t.process_event(&Event::InputClosed {
            id: "telemetry".into(),
        });
        t.process_event(&Event::InputClosed {
            id: "config".into(),
        });
        assert_eq!(t.closed_inputs().len(), 3);

        // Only the `camera`-sourced inputs should recover.
        assert!(t.process_event(&Event::NodeRestarted {
            id: NodeId::from("camera".to_string()),
        }));

        assert_eq!(t.state(&"sensor".into()), Some(InputState::Healthy));
        assert_eq!(t.state(&"telemetry".into()), Some(InputState::Healthy));
        assert_eq!(t.state(&"config".into()), Some(InputState::Closed));
    }

    #[test]
    fn node_restarted_preserves_last_value_cache() {
        // After a restart, cached stale data must still be readable so
        // nodes can fall back to it between restart and first new message.
        let mut source_map = HashMap::new();
        source_map.insert(
            DataId::from("sensor".to_string()),
            NodeId::from("camera".to_string()),
        );
        let mut t = InputTracker::with_source_map(source_map);

        t.process_event(&make_input("sensor", empty_data()));
        t.process_event(&Event::InputClosed {
            id: "sensor".into(),
        });
        assert!(t.last_value(&"sensor".into()).is_some());

        t.process_event(&Event::NodeRestarted {
            id: NodeId::from("camera".to_string()),
        });

        assert_eq!(t.state(&"sensor".into()), Some(InputState::Healthy));
        assert!(
            t.last_value(&"sensor".into()).is_some(),
            "cached value should survive restart so nodes can degrade gracefully"
        );
    }

    #[test]
    fn node_restarted_leaves_healthy_inputs_alone() {
        // A NodeRestarted event must not downgrade already-healthy inputs.
        let mut source_map = HashMap::new();
        source_map.insert(
            DataId::from("sensor".to_string()),
            NodeId::from("camera".to_string()),
        );
        let mut t = InputTracker::with_source_map(source_map);

        t.process_event(&make_input("sensor", empty_data()));
        assert_eq!(t.state(&"sensor".into()), Some(InputState::Healthy));

        t.process_event(&Event::NodeRestarted {
            id: NodeId::from("camera".to_string()),
        });
        assert_eq!(t.state(&"sensor".into()), Some(InputState::Healthy));
    }
}
