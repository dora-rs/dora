//! Fault tolerance tracking types.

use std::{
    collections::BTreeMap,
    sync::atomic::{self, AtomicU64},
};

use dora_core::config::NodeId;

/// Atomic counters for fault tolerance events, visible in periodic health check logs.
#[derive(Debug, Default)]
pub(crate) struct FaultToleranceStats {
    pub restarts: AtomicU64,
    pub health_check_kills: AtomicU64,
    pub startup_timeout_kills: AtomicU64,
    pub input_timeouts: AtomicU64,
    pub circuit_breaker_recoveries: AtomicU64,
    /// Data messages the daemon path discarded because a receiver's event
    /// channel was full. For a `drop_oldest` input that is the receiver's
    /// choice, and a `warn!` per drop is all it gets.
    pub dropped_messages: AtomicU64,
    /// The subset of `dropped_messages` on inputs that declared
    /// `queue_policy: backpressure`: a promise not to drop that the daemon
    /// could not keep, because no local producer could be held (a remote
    /// forward, a producer that exited mid-wait, a receiver that made no
    /// progress for `BACKPRESSURE_STALL_LIMIT`). A run that asked to be
    /// lossless — `dora replay` — fails on a nonzero count
    /// (dora-rs/dora#3397).
    pub lost_backpressure_messages: AtomicU64,
}

impl FaultToleranceStats {
    /// Records `n` data messages the daemon path dropped, on an input that
    /// `requires_backpressure` or not.
    pub fn record_drop(&self, n: u64, requires_backpressure: bool) {
        self.dropped_messages
            .fetch_add(n, atomic::Ordering::Relaxed);
        if requires_backpressure {
            self.lost_backpressure_messages
                .fetch_add(n, atomic::Ordering::Relaxed);
        }
    }

    pub fn any_nonzero(&self) -> bool {
        self.restarts.load(atomic::Ordering::Relaxed) > 0
            || self.health_check_kills.load(atomic::Ordering::Relaxed) > 0
            || self.startup_timeout_kills.load(atomic::Ordering::Relaxed) > 0
            || self.input_timeouts.load(atomic::Ordering::Relaxed) > 0
            || self
                .circuit_breaker_recoveries
                .load(atomic::Ordering::Relaxed)
                > 0
            || self.dropped_messages.load(atomic::Ordering::Relaxed) > 0
    }
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CascadingErrorCauses {
    pub(crate) caused_by: BTreeMap<NodeId, NodeId>,
}

impl CascadingErrorCauses {
    /// Return the ID of the node that caused a cascading error for the given node, if any.
    pub fn error_caused_by(&self, node: &NodeId) -> Option<&NodeId> {
        self.caused_by.get(node)
    }

    pub fn report_cascading_error(&mut self, causing_node: NodeId, affected_node: NodeId) {
        self.caused_by.entry(affected_node).or_insert(causing_node);
    }

    /// Forget a recorded cascading cause for `node`. Used when the id's
    /// process incarnation is replaced (dora-rs/dora#2927): entries are
    /// otherwise never removed, so a stale cohort-era cause would be
    /// attributed to the successor's own failures.
    pub fn forget(&mut self, node: &NodeId) {
        self.caused_by.remove(node);
    }
}
