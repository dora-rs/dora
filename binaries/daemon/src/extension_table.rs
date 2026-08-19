//! Dataflow-scoped store of opaque values for out-of-tree extensions.
//!
//! The daemon brokers lifetime and nothing else. It never interprets a
//! namespace, key or value — an extension (a transport living outside this
//! repo) uses the table to hand a descriptor from one node to another and to
//! learn when that descriptor is withdrawn.
//!
//! What the daemon contributes that a node cannot do for itself:
//!
//! - **Reclamation.** A node that crashes cannot clean up after itself. The
//!   daemon drops its entries on exit and on dataflow finish, which is the
//!   whole reason this is daemon-side rather than a plain dataflow message
//!   (dora-rs/dora#2881 is what that failure mode looks like).
//! - **Notification.** Everyone who touched a key learns when it goes away,
//!   so per-process resources keyed on it can be released promptly rather
//!   than at process exit.

use std::collections::{BTreeSet, HashMap};

use dora_message::id::NodeId;

/// Identifies one entry. Dataflow-scoped so two dataflows on the same daemon
/// cannot see or clobber each other's keys.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ExtensionKey {
    pub dataflow_id: String,
    pub namespace: String,
    pub key: String,
}

#[derive(Debug, Clone)]
struct Entry {
    value: Vec<u8>,
    /// The node that stored it. Its exit reclaims the entry.
    ///
    /// A bare id, not (id, incarnation): a node that is removed and re-added
    /// under the same id inherits whatever its predecessor stored, because
    /// the daemon cannot tell the two apart here. That costs the removed
    /// incarnation's stop-grace-window entries their prompt release — they
    /// live until dataflow finish (dora-rs/dora#3177). Making ownership
    /// generation-aware would close it; it needs the storing incarnation's
    /// generation threaded down to `store`, which the daemon drops before
    /// the `ExtensionStore` arm today.
    owner: NodeId,
    /// Everyone who stored or loaded it, and so must be told when it is
    /// dropped. Includes the owner.
    touched_by: BTreeSet<NodeId>,
}

/// Per-daemon table. Not `Sync`-wrapped here — the daemon owns it behind
/// `&mut self`, like the rest of its state.
#[derive(Debug, Default)]
pub struct ExtensionTable {
    entries: HashMap<ExtensionKey, Entry>,
    /// Live entry count per `dataflow_id`, kept in step with `entries` so the
    /// per-dataflow cap check in [`store`](Self::store) is O(1) instead of an
    /// O(total-entries) scan of every key across every dataflow. Invariant: a
    /// dataflow's value equals the number of `entries` keys with that
    /// `dataflow_id`, and the key is absent once that count reaches zero.
    per_dataflow_count: HashMap<String, usize>,
}

/// Upper bound on entries per dataflow, so a looping node cannot exhaust the
/// daemon's memory with a store-per-frame. Chosen to sit well above any
/// plausible working set while still bounding a runaway.
pub const MAX_ENTRIES_PER_DATAFLOW: usize = 8192;

impl ExtensionTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// Store or overwrite `key`.
    ///
    /// Overwriting is allowed only by the owner: a second node claiming an
    /// existing key would otherwise be able to redirect every reader of it.
    /// Returns `Err` on a foreign overwrite or when the dataflow is at cap.
    pub fn store(
        &mut self,
        key: ExtensionKey,
        value: Vec<u8>,
        owner: &NodeId,
    ) -> Result<(), String> {
        match self.entries.get_mut(&key) {
            Some(entry) => {
                if &entry.owner != owner {
                    return Err(format!(
                        "extension key `{}/{}` is owned by node `{}`; node `{}` may not overwrite it",
                        key.namespace, key.key, entry.owner, owner
                    ));
                }
                entry.value = value;
                Ok(())
            }
            None => {
                let count = self
                    .per_dataflow_count
                    .get(&key.dataflow_id)
                    .copied()
                    .unwrap_or(0);
                if count >= MAX_ENTRIES_PER_DATAFLOW {
                    return Err(format!(
                        "extension table full for this dataflow ({MAX_ENTRIES_PER_DATAFLOW} entries); \
                         node `{owner}` cannot store `{}/{}`",
                        key.namespace, key.key
                    ));
                }
                *self
                    .per_dataflow_count
                    .entry(key.dataflow_id.clone())
                    .or_insert(0) += 1;
                self.entries.insert(
                    key,
                    Entry {
                        value,
                        owner: owner.clone(),
                        touched_by: BTreeSet::from([owner.clone()]),
                    },
                );
                Ok(())
            }
        }
    }

    /// Read `key`, recording `reader` as someone to notify on drop.
    ///
    /// The reader is recorded even on a plain read, because the point of the
    /// notification is to let it release whatever it derived from the value.
    pub fn load(&mut self, key: &ExtensionKey, reader: &NodeId) -> Option<Vec<u8>> {
        let entry = self.entries.get_mut(key)?;
        entry.touched_by.insert(reader.clone());
        Some(entry.value.clone())
    }

    /// Remove `key`, returning everyone who must be told.
    ///
    /// `None` if it was not present, so a duplicate drop is distinguishable
    /// from a real one and does not produce a second broadcast.
    pub fn drop_key(&mut self, key: &ExtensionKey) -> Option<BTreeSet<NodeId>> {
        let entry = self.entries.remove(key)?;
        self.decrement(&key.dataflow_id);
        Some(entry.touched_by)
    }

    /// Decrement the live-entry counter for `dataflow_id`, dropping the counter
    /// key entirely at zero so an idle dataflow leaves no residue. Call exactly
    /// once per entry removed from `entries`.
    fn decrement(&mut self, dataflow_id: &str) {
        if let Some(count) = self.per_dataflow_count.get_mut(dataflow_id) {
            *count -= 1;
            if *count == 0 {
                self.per_dataflow_count.remove(dataflow_id);
            }
        }
    }

    /// Drop everything owned by `node` in `dataflow_id`, for when it exits.
    ///
    /// Returns each dropped key with its notification set. Entries the node
    /// merely *read* are left alone: they belong to someone still running.
    pub fn reclaim_owner(
        &mut self,
        dataflow_id: &str,
        node: &NodeId,
    ) -> Vec<(ExtensionKey, BTreeSet<NodeId>)> {
        let doomed: Vec<ExtensionKey> = self
            .entries
            .iter()
            .filter(|(k, e)| k.dataflow_id == dataflow_id && &e.owner == node)
            .map(|(k, _)| k.clone())
            .collect();
        doomed
            .into_iter()
            .filter_map(|k| {
                let touched = self.entries.remove(&k)?.touched_by;
                self.decrement(&k.dataflow_id);
                Some((k, touched))
            })
            .collect()
    }

    /// Drop every entry for a finished dataflow. No notifications: its nodes
    /// are gone and its channels are closed by the time this runs.
    pub fn reclaim_dataflow(&mut self, dataflow_id: &str) -> usize {
        let before = self.entries.len();
        self.entries.retain(|k, _| k.dataflow_id != dataflow_id);
        self.per_dataflow_count.remove(dataflow_id);
        before - self.entries.len()
    }

    #[cfg(test)]
    pub fn len(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(name: &str) -> NodeId {
        name.to_string().into()
    }

    fn key(ns: &str, k: &str) -> ExtensionKey {
        ExtensionKey {
            dataflow_id: "df".into(),
            namespace: ns.into(),
            key: k.into(),
        }
    }

    #[test]
    fn store_then_load_round_trips() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"meta".to_vec(), &node("sender"))
            .unwrap();
        assert_eq!(
            t.load(&key("pool", "a"), &node("recv")),
            Some(b"meta".to_vec())
        );
    }

    #[test]
    fn load_of_absent_key_is_none() {
        let mut t = ExtensionTable::new();
        assert_eq!(t.load(&key("pool", "nope"), &node("recv")), None);
    }

    #[test]
    fn namespaces_do_not_collide() {
        let mut t = ExtensionTable::new();
        t.store(key("a", "k"), b"one".to_vec(), &node("n")).unwrap();
        t.store(key("b", "k"), b"two".to_vec(), &node("n")).unwrap();
        assert_eq!(t.load(&key("a", "k"), &node("n")), Some(b"one".to_vec()));
        assert_eq!(t.load(&key("b", "k"), &node("n")), Some(b"two".to_vec()));
    }

    #[test]
    fn dataflows_do_not_collide() {
        let mut t = ExtensionTable::new();
        let mut other = key("pool", "k");
        other.dataflow_id = "other-df".into();
        t.store(key("pool", "k"), b"mine".to_vec(), &node("n"))
            .unwrap();
        t.store(other.clone(), b"theirs".to_vec(), &node("n"))
            .unwrap();
        assert_eq!(
            t.load(&key("pool", "k"), &node("n")),
            Some(b"mine".to_vec())
        );
        assert_eq!(t.load(&other, &node("n")), Some(b"theirs".to_vec()));
    }

    #[test]
    fn a_foreign_node_cannot_overwrite() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"mine".to_vec(), &node("owner"))
            .unwrap();
        let err = t
            .store(key("pool", "a"), b"hijacked".to_vec(), &node("attacker"))
            .unwrap_err();
        assert!(err.contains("owned by node `owner`"), "{err}");
        // The original value survived the attempt.
        assert_eq!(
            t.load(&key("pool", "a"), &node("owner")),
            Some(b"mine".to_vec())
        );
    }

    #[test]
    fn the_owner_may_overwrite() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"v1".to_vec(), &node("owner"))
            .unwrap();
        t.store(key("pool", "a"), b"v2".to_vec(), &node("owner"))
            .unwrap();
        assert_eq!(
            t.load(&key("pool", "a"), &node("owner")),
            Some(b"v2".to_vec())
        );
    }

    #[test]
    fn drop_reports_every_node_that_touched_the_key() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"v".to_vec(), &node("owner"))
            .unwrap();
        t.load(&key("pool", "a"), &node("reader1"));
        t.load(&key("pool", "a"), &node("reader2"));

        let notified = t.drop_key(&key("pool", "a")).expect("was present");
        assert_eq!(
            notified,
            BTreeSet::from([node("owner"), node("reader1"), node("reader2")])
        );
    }

    #[test]
    fn dropping_twice_notifies_once() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"v".to_vec(), &node("owner"))
            .unwrap();
        assert!(t.drop_key(&key("pool", "a")).is_some());
        // The second drop is a no-op, not a second broadcast.
        assert!(t.drop_key(&key("pool", "a")).is_none());
    }

    #[test]
    fn owner_exit_reclaims_its_entries_and_notifies_readers() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"v".to_vec(), &node("owner"))
            .unwrap();
        t.load(&key("pool", "a"), &node("reader"));

        let reclaimed = t.reclaim_owner("df", &node("owner"));
        assert_eq!(reclaimed.len(), 1);
        assert_eq!(reclaimed[0].0, key("pool", "a"));
        assert!(reclaimed[0].1.contains(&node("reader")));
        assert_eq!(t.len(), 0);
    }

    #[test]
    fn a_readers_exit_does_not_reclaim_the_owners_entry() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"v".to_vec(), &node("owner"))
            .unwrap();
        t.load(&key("pool", "a"), &node("reader"));

        // The reader leaving must not take a live sender's descriptor with it.
        assert!(t.reclaim_owner("df", &node("reader")).is_empty());
        assert_eq!(t.len(), 1);
    }

    #[test]
    fn owner_exit_is_scoped_to_its_own_dataflow() {
        let mut t = ExtensionTable::new();
        let mut other = key("pool", "a");
        other.dataflow_id = "other-df".into();
        t.store(key("pool", "a"), b"v".to_vec(), &node("owner"))
            .unwrap();
        t.store(other, b"v".to_vec(), &node("owner")).unwrap();

        assert_eq!(t.reclaim_owner("df", &node("owner")).len(), 1);
        // Same node id in another dataflow keeps its entry.
        assert_eq!(t.len(), 1);
    }

    #[test]
    fn dataflow_finish_reclaims_everything_it_owns() {
        let mut t = ExtensionTable::new();
        let mut other = key("pool", "keep");
        other.dataflow_id = "other-df".into();
        t.store(key("pool", "a"), b"v".to_vec(), &node("n"))
            .unwrap();
        t.store(key("pool", "b"), b"v".to_vec(), &node("m"))
            .unwrap();
        t.store(other, b"v".to_vec(), &node("n")).unwrap();

        assert_eq!(t.reclaim_dataflow("df"), 2);
        assert_eq!(t.len(), 1);
    }

    #[test]
    fn the_per_dataflow_cap_is_enforced_and_scoped() {
        let mut t = ExtensionTable::new();
        for i in 0..MAX_ENTRIES_PER_DATAFLOW {
            t.store(key("pool", &i.to_string()), b"v".to_vec(), &node("n"))
                .unwrap();
        }
        let err = t
            .store(key("pool", "one-too-many"), b"v".to_vec(), &node("n"))
            .unwrap_err();
        assert!(err.contains("extension table full"), "{err}");

        // A different dataflow still has its own budget.
        let mut other = key("pool", "fresh");
        other.dataflow_id = "other-df".into();
        assert!(t.store(other, b"v".to_vec(), &node("n")).is_ok());
    }

    #[test]
    fn overwriting_at_cap_still_works() {
        let mut t = ExtensionTable::new();
        for i in 0..MAX_ENTRIES_PER_DATAFLOW {
            t.store(key("pool", &i.to_string()), b"v".to_vec(), &node("n"))
                .unwrap();
        }
        // At cap, replacing an existing key adds nothing, so it must not be
        // rejected — otherwise a steady-state writer wedges at the boundary.
        assert!(
            t.store(key("pool", "0"), b"v2".to_vec(), &node("n"))
                .is_ok()
        );
    }

    /// Recompute the per-dataflow counts from `entries` and assert the
    /// incrementally maintained `per_dataflow_count` matches exactly — no
    /// drift and no zero-valued residue. This is the invariant the O(1) cap
    /// check in `store` relies on.
    fn assert_count_invariant(t: &ExtensionTable) {
        let mut recomputed: HashMap<String, usize> = HashMap::new();
        for k in t.entries.keys() {
            *recomputed.entry(k.dataflow_id.clone()).or_insert(0) += 1;
        }
        assert_eq!(
            t.per_dataflow_count, recomputed,
            "per_dataflow_count drifted from entries"
        );
    }

    #[test]
    fn dropping_an_entry_at_cap_frees_a_slot() {
        let mut t = ExtensionTable::new();
        for i in 0..MAX_ENTRIES_PER_DATAFLOW {
            t.store(key("pool", &i.to_string()), b"v".to_vec(), &node("n"))
                .unwrap();
        }
        // At cap, a fresh key is rejected...
        assert!(
            t.store(key("pool", "extra"), b"v".to_vec(), &node("n"))
                .is_err()
        );
        // ...but dropping one frees exactly one slot, which requires the O(1)
        // counter to have been decremented on removal.
        assert!(t.drop_key(&key("pool", "0")).is_some());
        assert!(
            t.store(key("pool", "extra"), b"v".to_vec(), &node("n"))
                .is_ok()
        );
        assert_count_invariant(&t);
    }

    #[test]
    fn reclaim_paths_keep_the_count_in_step() {
        let mut t = ExtensionTable::new();
        t.store(key("pool", "a"), b"v".to_vec(), &node("owner"))
            .unwrap();
        t.store(key("pool", "b"), b"v".to_vec(), &node("owner"))
            .unwrap();
        let mut other = key("pool", "c");
        other.dataflow_id = "other-df".into();
        t.store(other, b"v".to_vec(), &node("owner")).unwrap();
        assert_count_invariant(&t);

        // Owner exit reclaims that owner's entries in "df".
        t.reclaim_owner("df", &node("owner"));
        assert_count_invariant(&t);
        // "df" is now empty, so its counter key must be gone (no zero residue).
        assert!(!t.per_dataflow_count.contains_key("df"));

        // Dataflow finish clears the remaining dataflow's counter entirely.
        t.reclaim_dataflow("other-df");
        assert_count_invariant(&t);
        assert!(t.per_dataflow_count.is_empty());
    }
}
