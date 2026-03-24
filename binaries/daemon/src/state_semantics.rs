use std::collections::BTreeMap;

use dora_message::{
    DataflowId, common::DaemonId, daemon_to_daemon::StateUpdate, daemon_to_node::StateWriteResult,
};

#[derive(Debug, Clone)]
struct VersionedValue {
    value: Option<Vec<u8>>,
    revision: u64,
    source_daemon_id: DaemonId,
}

#[derive(Debug, Default)]
struct DataflowState {
    keys: BTreeMap<String, VersionedValue>,
}

#[derive(Debug, Default)]
pub struct StateStore {
    dataflows: dashmap::DashMap<DataflowId, DataflowState>,
}

impl StateStore {
    pub fn ensure_dataflow(&self, dataflow_id: DataflowId) {
        self.dataflows.entry(dataflow_id).or_default();
    }

    pub fn remove_dataflow(&self, dataflow_id: DataflowId) {
        self.dataflows.remove(&dataflow_id);
    }

    pub fn get(&self, dataflow_id: DataflowId, key: &str) -> (Option<Vec<u8>>, u64) {
        let Some(dataflow) = self.dataflows.get(&dataflow_id) else {
            return (None, 0);
        };
        let Some(value) = dataflow.keys.get(key) else {
            return (None, 0);
        };
        (value.value.clone(), value.revision)
    }

    pub fn set(
        &self,
        dataflow_id: DataflowId,
        key: String,
        value: Vec<u8>,
        source_daemon_id: DaemonId,
    ) -> (StateWriteResult, StateUpdate) {
        self.write_internal(dataflow_id, key, Some(value), None, source_daemon_id)
    }

    pub fn compare_and_set(
        &self,
        dataflow_id: DataflowId,
        key: String,
        expected_revision: u64,
        value: Option<Vec<u8>>,
        source_daemon_id: DaemonId,
    ) -> (StateWriteResult, Option<StateUpdate>) {
        let (result, update) = self.write_internal(
            dataflow_id,
            key,
            value,
            Some(expected_revision),
            source_daemon_id,
        );
        match result {
            StateWriteResult::Applied { .. } => (result, Some(update)),
            StateWriteResult::Conflict { .. } => (result, None),
        }
    }

    fn write_internal(
        &self,
        dataflow_id: DataflowId,
        key: String,
        value: Option<Vec<u8>>,
        expected_revision: Option<u64>,
        source_daemon_id: DaemonId,
    ) -> (StateWriteResult, StateUpdate) {
        let mut dataflow = self.dataflows.entry(dataflow_id).or_default();
        let current = dataflow.keys.get(&key).cloned();
        let current_revision = current.as_ref().map(|v| v.revision).unwrap_or(0);
        let current_value = current.as_ref().and_then(|v| v.value.clone());

        if let Some(expected) = expected_revision {
            if expected != current_revision {
                return (
                    StateWriteResult::Conflict {
                        value: current_value,
                        revision: current_revision,
                    },
                    StateUpdate {
                        dataflow_id,
                        key,
                        value,
                        revision: current_revision,
                        source_daemon_id,
                    },
                );
            }
        }

        let next_revision = current_revision.saturating_add(1);
        let update = StateUpdate {
            dataflow_id,
            key: key.clone(),
            value: value.clone(),
            revision: next_revision,
            source_daemon_id: source_daemon_id.clone(),
        };
        dataflow.keys.insert(
            key,
            VersionedValue {
                value: value.clone(),
                revision: next_revision,
                source_daemon_id,
            },
        );
        (
            StateWriteResult::Applied {
                value,
                revision: next_revision,
            },
            update,
        )
    }

    /// Apply replicated state update and return whether local state changed.
    pub fn apply_replicated_update(&self, update: StateUpdate) -> bool {
        let mut dataflow = self.dataflows.entry(update.dataflow_id).or_default();
        let current = dataflow.keys.get(&update.key).cloned();
        let should_apply = match current {
            None => true,
            Some(current) => {
                if update.revision > current.revision {
                    true
                } else if update.revision < current.revision {
                    false
                } else {
                    update.source_daemon_id > current.source_daemon_id
                }
            }
        };

        if should_apply {
            dataflow.keys.insert(
                update.key,
                VersionedValue {
                    value: update.value,
                    revision: update.revision,
                    source_daemon_id: update.source_daemon_id,
                },
            );
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::StateStore;
    use dora_message::common::DaemonId;
    use uuid::Uuid;

    #[test]
    fn concurrent_writes_converge_deterministically_across_daemons() {
        let dataflow_id = Uuid::new_v4();
        let daemon_a = DaemonId::new(Some("A".to_owned()));
        let daemon_b = DaemonId::new(Some("B".to_owned()));
        let key = "shared-key".to_owned();

        let store_a = StateStore::default();
        let store_b = StateStore::default();
        store_a.ensure_dataflow(dataflow_id);
        store_b.ensure_dataflow(dataflow_id);

        let (_, rev0) = store_a.get(dataflow_id, &key);
        assert_eq!(rev0, 0);

        let (res_a, update_a) = store_a.compare_and_set(
            dataflow_id,
            key.clone(),
            0,
            Some(b"from-a".to_vec()),
            daemon_a.clone(),
        );
        let (res_b, update_b) = store_b.compare_and_set(
            dataflow_id,
            key.clone(),
            0,
            Some(b"from-b".to_vec()),
            daemon_b.clone(),
        );
        assert!(matches!(
            res_a,
            dora_message::daemon_to_node::StateWriteResult::Applied { revision: 1, .. }
        ));
        assert!(matches!(
            res_b,
            dora_message::daemon_to_node::StateWriteResult::Applied { revision: 1, .. }
        ));
        let update_a = update_a.expect("A CAS should apply");
        let update_b = update_b.expect("B CAS should apply");

        // Cross-replicate concurrent writes in opposite orders.
        store_a.apply_replicated_update(update_b.clone());
        store_b.apply_replicated_update(update_a.clone());

        let (value_a, revision_a) = store_a.get(dataflow_id, &key);
        let (value_b, revision_b) = store_b.get(dataflow_id, &key);
        assert_eq!(revision_a, 1);
        assert_eq!(revision_b, 1);
        assert_eq!(value_a, value_b);
        // Tie-break is deterministic by daemon id for equal revisions.
        assert_eq!(value_a, Some(b"from-b".to_vec()));

        // stale CAS must fail with conflict
        let (stale_result, stale_update) = store_a.compare_and_set(
            dataflow_id,
            key.clone(),
            0,
            Some(b"stale".to_vec()),
            daemon_a,
        );
        assert!(stale_update.is_none());
        assert!(matches!(
            stale_result,
            dora_message::daemon_to_node::StateWriteResult::Conflict { revision: 1, .. }
        ));
    }
}
