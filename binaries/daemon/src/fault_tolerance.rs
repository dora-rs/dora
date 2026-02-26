//! Fault tolerance tracking types.

use std::{
    collections::BTreeMap,
    sync::atomic::{self, AtomicU64},
};

use adora_core::config::NodeId;

/// Atomic counters for fault tolerance events, visible in periodic health check logs.
#[derive(Default)]
pub(crate) struct FaultToleranceStats {
    pub restarts: AtomicU64,
    pub health_check_kills: AtomicU64,
    pub input_timeouts: AtomicU64,
    pub circuit_breaker_recoveries: AtomicU64,
}

impl FaultToleranceStats {
    pub fn any_nonzero(&self) -> bool {
        self.restarts.load(atomic::Ordering::Relaxed) > 0
            || self.health_check_kills.load(atomic::Ordering::Relaxed) > 0
            || self.input_timeouts.load(atomic::Ordering::Relaxed) > 0
            || self
                .circuit_breaker_recoveries
                .load(atomic::Ordering::Relaxed)
                > 0
    }
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CascadingErrorCauses {
    pub(crate) caused_by: BTreeMap<NodeId, NodeId>,
}

impl CascadingErrorCauses {
    #[allow(dead_code)]
    pub fn experienced_cascading_error(&self, node: &NodeId) -> bool {
        self.caused_by.contains_key(node)
    }

    /// Return the ID of the node that caused a cascading error for the given node, if any.
    pub fn error_caused_by(&self, node: &NodeId) -> Option<&NodeId> {
        self.caused_by.get(node)
    }

    pub fn report_cascading_error(&mut self, causing_node: NodeId, affected_node: NodeId) {
        self.caused_by.entry(affected_node).or_insert(causing_node);
    }
}
