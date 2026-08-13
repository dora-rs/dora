//! Cross-process memory-pool cleanup coordination.
//!
//! When any node calls `free_memory_pool`, the daemon sends a targeted
//! `NodeEvent::FreeMemoryPool` to every node that registered or read the
//! pool.  The event thread intercepts this event (without forwarding it to
//! user code) and pushes the pool id into a pending-cleanup set.  The
//! language binding drains this set before yielding the next user-visible
//! event, so that GPU buffers, transit buffers, and shmem mappings are
//! released regardless of which node initiated the free.

use std::collections::HashSet;
use std::sync::Mutex;

/// Pending free notifications from the daemon.
///
/// Written by the event-stream thread, drained by the language binding
/// (e.g. the Python node API) before returning the next user event.
/// A `HashSet` is used so that duplicate pushes (e.g. multiple event
/// streams in the same process) are naturally deduplicated.
static PENDING_FREES: std::sync::LazyLock<Mutex<HashSet<String>>> =
    std::sync::LazyLock::new(|| Mutex::new(HashSet::new()));

/// Push a pool id onto the pending-cleanup set (called from the event thread).
pub(crate) fn push_freed_pool(shared_memory_id: String) {
    PENDING_FREES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .insert(shared_memory_id);
}

/// Drain and return all pending pool ids for cleanup.
///
/// The caller is responsible for releasing per-process resources
/// (GPU buffers, transit buffers, shmem mappings) for each returned id.
pub fn drain_freed_pools() -> Vec<String> {
    PENDING_FREES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .drain()
        .collect()
}
