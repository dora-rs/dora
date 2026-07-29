//! Cross-process memory-pool cleanup coordination.
//!
//! When any node calls `free_memory_pool`, the daemon broadcasts a
//! `NodeEvent::FreeMemoryPool` to every node in the dataflow.  The event
//! thread intercepts this event (without forwarding it to user code) and
//! pushes the pool id into a pending-cleanup list.  The language binding
//! drains this list before yielding the next user-visible event, so that
//! GPU buffers, transit buffers, and shmem mappings are released regardless
//! of which node initiated the free.

use std::collections::VecDeque;
use std::sync::Mutex;

/// Pending free notifications from the daemon.
///
/// Written by the event-stream thread, drained by the language binding
/// (e.g. the Python node API) before returning the next user event.
static PENDING_FREES: std::sync::LazyLock<Mutex<VecDeque<String>>> =
    std::sync::LazyLock::new(|| Mutex::new(VecDeque::new()));

/// Push a pool id onto the pending-cleanup list (called from the event thread).
pub(crate) fn push_freed_pool(shared_memory_id: String) {
    PENDING_FREES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .push_back(shared_memory_id);
}

/// Drain and return all pending pool ids for cleanup.
///
/// The caller is responsible for releasing per-process resources
/// (GPU buffers, transit buffers, shmem mappings) for each returned id.
pub fn drain_freed_pools() -> Vec<String> {
    PENDING_FREES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .drain(..)
        .collect()
}
