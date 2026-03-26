use std::collections::{BTreeMap, VecDeque};

use dora_message::{DataflowId, common::DaemonId, daemon_to_daemon::StateUpdate};

/// Number of state operations retained per dataflow for incremental catch-up.
pub const DEFAULT_LOG_CAPACITY: usize = 1024;

#[derive(Debug, Default)]
pub struct DataflowReplicationState {
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) local_revision: u64,
    pub(crate) kv: BTreeMap<String, Vec<u8>>,
    pub(crate) operation_log: VecDeque<StateUpdate>,
    pub(crate) known_revisions: BTreeMap<DaemonId, u64>,
}

impl DataflowReplicationState {
    fn apply_update(&mut self, update: &StateUpdate) {
        match &update.value {
            Some(value) => {
                self.kv.insert(update.key.clone(), value.clone());
            }
            None => {
                self.kv.remove(&update.key);
            }
        }
    }
}

#[derive(Debug)]
pub struct ReplicationLogStore {
    log_capacity: usize,
    states: dashmap::DashMap<DataflowId, DataflowReplicationState>,
}

impl Default for ReplicationLogStore {
    fn default() -> Self {
        Self::new(DEFAULT_LOG_CAPACITY)
    }
}

impl ReplicationLogStore {
    pub fn new(log_capacity: usize) -> Self {
        Self {
            log_capacity,
            states: Default::default(),
        }
    }

    pub fn ensure_dataflow(&self, dataflow_id: DataflowId) {
        self.states.entry(dataflow_id).or_default();
    }

    pub fn remove_dataflow(&self, dataflow_id: DataflowId) {
        self.states.remove(&dataflow_id);
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn apply_local_update(
        &self,
        dataflow_id: DataflowId,
        source_daemon_id: DaemonId,
        key: String,
        value: Option<Vec<u8>>,
    ) -> StateUpdate {
        let mut state = self.states.entry(dataflow_id).or_default();
        let state_ref = state.value_mut();

        state_ref.local_revision = state_ref.local_revision.saturating_add(1);
        let update = StateUpdate {
            dataflow_id,
            source_daemon_id: source_daemon_id.clone(),
            revision: state_ref.local_revision,
            key,
            value,
        };
        state_ref.apply_update(&update);
        state_ref
            .known_revisions
            .insert(source_daemon_id, update.revision);

        state_ref.operation_log.push_back(update.clone());
        while state_ref.operation_log.len() > self.log_capacity {
            state_ref.operation_log.pop_front();
        }
        update
    }

    /// Applies a remote update if it advances the known revision.
    pub fn apply_remote_update(&self, update: &StateUpdate) -> bool {
        let mut state = self.states.entry(update.dataflow_id).or_default();
        let state_ref = state.value_mut();

        let known = state_ref
            .known_revisions
            .get(&update.source_daemon_id)
            .copied()
            .unwrap_or(0);
        if update.revision <= known {
            return false;
        }

        state_ref.apply_update(update);
        state_ref
            .known_revisions
            .insert(update.source_daemon_id.clone(), update.revision);
        state_ref.operation_log.push_back(update.clone());
        while state_ref.operation_log.len() > self.log_capacity {
            state_ref.operation_log.pop_front();
        }
        true
    }

    pub fn known_revisions(&self, dataflow_id: DataflowId) -> BTreeMap<DaemonId, u64> {
        self.states
            .get(&dataflow_id)
            .map(|entry| entry.known_revisions.clone())
            .unwrap_or_default()
    }

    pub fn updates_since(
        &self,
        dataflow_id: DataflowId,
        source_daemon_id: &DaemonId,
        since_revision: u64,
    ) -> Vec<StateUpdate> {
        self.states
            .get(&dataflow_id)
            .map(|entry| {
                entry
                    .operation_log
                    .iter()
                    .filter(|u| {
                        &u.source_daemon_id == source_daemon_id && u.revision > since_revision
                    })
                    .cloned()
                    .collect()
            })
            .unwrap_or_default()
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn values_snapshot(&self, dataflow_id: DataflowId) -> BTreeMap<String, Vec<u8>> {
        self.states
            .get(&dataflow_id)
            .map(|entry| entry.kv.clone())
            .unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::ReplicationLogStore;
    use dora_message::{common::DaemonId, daemon_to_daemon::InterDaemonEvent};
    use uuid::Uuid;

    #[test]
    fn catch_up_after_rejoin_replays_only_missing_updates() {
        let dataflow_id = Uuid::new_v4();
        let daemon_a = DaemonId::new(Some("A".to_owned()));
        let daemon_b = DaemonId::new(Some("B".to_owned()));

        let store_a = ReplicationLogStore::new(64);
        let store_b = ReplicationLogStore::new(64);

        // A produces first two updates.
        let u1 = store_a.apply_local_update(
            dataflow_id,
            daemon_a.clone(),
            "k1".to_owned(),
            Some(b"v1".to_vec()),
        );
        let u2 = store_a.apply_local_update(
            dataflow_id,
            daemon_a.clone(),
            "k2".to_owned(),
            Some(b"v2".to_vec()),
        );

        // B receives only these two (before disconnect).
        assert!(store_b.apply_remote_update(&u1));
        assert!(store_b.apply_remote_update(&u2));
        assert_eq!(
            store_b
                .known_revisions(dataflow_id)
                .get(&daemon_a)
                .copied()
                .unwrap_or(0),
            2
        );

        // A keeps producing while B is disconnected.
        let _u3 = store_a.apply_local_update(
            dataflow_id,
            daemon_a.clone(),
            "k1".to_owned(),
            Some(b"v1b".to_vec()),
        );
        let _u4 = store_a.apply_local_update(
            dataflow_id,
            daemon_a.clone(),
            "k3".to_owned(),
            Some(b"v3".to_vec()),
        );

        // B rejoins and asks for deltas since what it knows for A.
        let known_revisions = store_b.known_revisions(dataflow_id);
        let request = InterDaemonEvent::StateCatchUpRequest {
            dataflow_id,
            requester_daemon_id: daemon_b.clone(),
            known_revisions,
        };

        let updates = match request {
            InterDaemonEvent::StateCatchUpRequest {
                known_revisions, ..
            } => {
                let since = known_revisions.get(&daemon_a).copied().unwrap_or(0);
                store_a.updates_since(dataflow_id, &daemon_a, since)
            }
            _ => unreachable!(),
        };
        assert_eq!(updates.len(), 2);
        assert_eq!(updates[0].revision, 3);
        assert_eq!(updates[1].revision, 4);

        for update in &updates {
            assert!(store_b.apply_remote_update(update));
        }

        let snapshot = store_b.values_snapshot(dataflow_id);
        assert_eq!(snapshot.get("k1").cloned(), Some(b"v1b".to_vec()));
        assert_eq!(snapshot.get("k2").cloned(), Some(b"v2".to_vec()));
        assert_eq!(snapshot.get("k3").cloned(), Some(b"v3".to_vec()));
        assert_eq!(
            store_b
                .known_revisions(dataflow_id)
                .get(&daemon_a)
                .copied()
                .unwrap_or(0),
            4
        );
    }
}
