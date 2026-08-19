//! Drop notifications for the daemon's opaque extension table.
//!
//! When any node drops an extension key, the daemon sends
//! [`NodeEvent::ExtensionDropped`](dora_message::daemon_to_node::NodeEvent::ExtensionDropped)
//! to every node that stored or loaded it. That event is out-of-band — it is
//! not a dataflow input, and surfacing it to user code would mean every node
//! author had to match on an event they did not ask for. So the event-stream
//! thread consumes it into this process-global queue, and the extension's own
//! code drains it whenever it next runs.
//!
//! Process-global rather than per-node because a language binding holds its
//! caches the same way: one process, one set of mappings, regardless of how
//! many `DoraNode`s live in it.

use std::collections::VecDeque;
use std::sync::{LazyLock, Mutex};

/// Bound on the queue. A consumer that never drains must not grow it without
/// limit; dropping the oldest entry is safe because a missed notification only
/// means the extension releases that resource later (on its own `drop`) rather
/// than promptly.
const MAX_PENDING: usize = 4096;

type Dropped = (String, String);

static DROPPED: LazyLock<Mutex<VecDeque<Dropped>>> = LazyLock::new(|| Mutex::new(VecDeque::new()));

/// Record a dropped `(namespace, key)`. Called by the event-stream thread.
pub(crate) fn push_dropped(namespace: String, key: String) {
    let mut queue = DROPPED.lock().unwrap_or_else(|e| e.into_inner());
    if queue.len() >= MAX_PENDING {
        queue.pop_front();
    }
    queue.push_back((namespace, key));
}

/// Take every pending drop notification for `namespace`, leaving the rest.
///
/// Scoped by namespace so two extensions in one process cannot swallow each
/// other's notifications.
pub fn drain_dropped_keys(namespace: &str) -> Vec<String> {
    let mut queue = DROPPED.lock().unwrap_or_else(|e| e.into_inner());
    let mut taken = Vec::new();
    queue.retain(|(ns, key)| {
        if ns == namespace {
            taken.push(key.clone());
            false
        } else {
            true
        }
    });
    taken
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The queue is process-global by design, so these tests would clobber
    /// each other under cargo's default thread-per-test. Serialize them and
    /// start each from empty.
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    fn guard() -> std::sync::MutexGuard<'static, ()> {
        let g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        DROPPED.lock().unwrap_or_else(|e| e.into_inner()).clear();
        g
    }

    #[test]
    fn drain_returns_only_the_requested_namespace() {
        let _g = guard();
        push_dropped("pool".into(), "a".into());
        push_dropped("other".into(), "b".into());
        push_dropped("pool".into(), "c".into());

        assert_eq!(drain_dropped_keys("pool"), vec!["a", "c"]);
        // The other namespace's entry survived rather than being consumed.
        assert_eq!(drain_dropped_keys("other"), vec!["b"]);
    }

    #[test]
    fn drain_is_exhaustive() {
        let _g = guard();
        push_dropped("pool".into(), "a".into());
        assert_eq!(drain_dropped_keys("pool"), vec!["a"]);
        assert!(drain_dropped_keys("pool").is_empty());
    }

    #[test]
    fn queue_is_bounded_and_drops_oldest() {
        let _g = guard();
        for i in 0..MAX_PENDING + 10 {
            push_dropped("pool".into(), i.to_string());
        }
        let drained = drain_dropped_keys("pool");
        assert_eq!(drained.len(), MAX_PENDING);
        // The oldest ten were evicted, so the window starts at 10.
        assert_eq!(drained.first().map(String::as_str), Some("10"));
    }
}
