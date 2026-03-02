use std::collections::BTreeMap;
use std::sync::RwLock;

use adora_message::common::DaemonId;
use eyre::{Result, eyre};
use uuid::Uuid;

use crate::{BuildRecord, CoordinatorStore, DaemonInfo, DataflowRecord};

/// In-memory implementation of [`CoordinatorStore`].
///
/// State lives only in-process and is lost on restart.
/// This is the default store used by the coordinator.
pub struct InMemoryStore {
    daemons: RwLock<BTreeMap<DaemonId, DaemonInfo>>,
    dataflows: RwLock<BTreeMap<Uuid, DataflowRecord>>,
    builds: RwLock<BTreeMap<Uuid, BuildRecord>>,
}

impl InMemoryStore {
    pub fn new() -> Self {
        Self {
            daemons: RwLock::new(BTreeMap::new()),
            dataflows: RwLock::new(BTreeMap::new()),
            builds: RwLock::new(BTreeMap::new()),
        }
    }
}

impl Default for InMemoryStore {
    fn default() -> Self {
        Self::new()
    }
}

impl CoordinatorStore for InMemoryStore {
    // -- Daemon registry --

    fn register_daemon(&self, info: DaemonInfo) -> Result<()> {
        let mut daemons = self
            .daemons
            .write()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        daemons.insert(info.daemon_id.clone(), info);
        Ok(())
    }

    fn unregister_daemon(&self, id: &DaemonId) -> Result<()> {
        let mut daemons = self
            .daemons
            .write()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        daemons.remove(id);
        Ok(())
    }

    fn list_daemons(&self) -> Result<Vec<DaemonInfo>> {
        let daemons = self
            .daemons
            .read()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        Ok(daemons.values().cloned().collect())
    }

    fn get_daemon(&self, id: &DaemonId) -> Result<Option<DaemonInfo>> {
        let daemons = self
            .daemons
            .read()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        Ok(daemons.get(id).cloned())
    }

    fn get_daemon_by_machine(&self, machine_id: &str) -> Result<Option<DaemonId>> {
        let daemons = self
            .daemons
            .read()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        Ok(daemons
            .keys()
            .find(|id| id.matches_machine_id(machine_id))
            .cloned())
    }

    // -- Dataflow state --

    fn put_dataflow(&self, record: &DataflowRecord) -> Result<()> {
        let mut dataflows = self
            .dataflows
            .write()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        dataflows.insert(record.uuid, record.clone());
        Ok(())
    }

    fn get_dataflow(&self, uuid: &Uuid) -> Result<Option<DataflowRecord>> {
        let dataflows = self
            .dataflows
            .read()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        Ok(dataflows.get(uuid).cloned())
    }

    fn list_dataflows(&self) -> Result<Vec<DataflowRecord>> {
        let dataflows = self
            .dataflows
            .read()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        Ok(dataflows.values().cloned().collect())
    }

    fn delete_dataflow(&self, uuid: &Uuid) -> Result<()> {
        let mut dataflows = self
            .dataflows
            .write()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        dataflows.remove(uuid);
        Ok(())
    }

    // -- Build state --

    fn put_build(&self, record: &BuildRecord) -> Result<()> {
        let mut builds = self
            .builds
            .write()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        builds.insert(record.build_id, record.clone());
        Ok(())
    }

    fn get_build(&self, build_id: &Uuid) -> Result<Option<BuildRecord>> {
        let builds = self
            .builds
            .read()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        Ok(builds.get(build_id).cloned())
    }

    fn list_builds(&self) -> Result<Vec<BuildRecord>> {
        let builds = self
            .builds
            .read()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        Ok(builds.values().cloned().collect())
    }

    fn delete_build(&self, build_id: &Uuid) -> Result<()> {
        let mut builds = self
            .builds
            .write()
            .map_err(|e| eyre!("lock poisoned: {e}"))?;
        builds.remove(build_id);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BuildStatus, DataflowStatus};

    #[test]
    fn daemon_crud() {
        let store = InMemoryStore::new();
        let id = DaemonId::new(Some("m1".into()));
        let info = DaemonInfo {
            daemon_id: id.clone(),
            machine_id: Some("m1".into()),
            labels: Default::default(),
        };

        store.register_daemon(info).unwrap();
        assert!(store.get_daemon(&id).unwrap().is_some());
        assert_eq!(store.list_daemons().unwrap().len(), 1);
        assert!(store.get_daemon_by_machine("m1").unwrap().is_some());

        store.unregister_daemon(&id).unwrap();
        assert!(store.get_daemon(&id).unwrap().is_none());
    }

    #[test]
    fn dataflow_crud() {
        let store = InMemoryStore::new();
        let uuid = Uuid::new_v4();
        let record = DataflowRecord {
            uuid,
            name: Some("test".into()),
            descriptor_json: "{}".into(),
            status: DataflowStatus::Pending,
            daemon_ids: vec![],
            generation: 0,
            created_at: 0,
            updated_at: 0,
        };

        store.put_dataflow(&record).unwrap();
        assert_eq!(store.list_dataflows().unwrap().len(), 1);

        let loaded = store.get_dataflow(&uuid).unwrap().unwrap();
        assert_eq!(loaded.name.as_deref(), Some("test"));

        store.delete_dataflow(&uuid).unwrap();
        assert!(store.get_dataflow(&uuid).unwrap().is_none());
    }

    #[test]
    fn build_crud() {
        let store = InMemoryStore::new();
        let id = Uuid::new_v4();
        let record = BuildRecord {
            build_id: id,
            status: BuildStatus::Pending,
            errors: vec![],
            created_at: 0,
            updated_at: 0,
        };

        store.put_build(&record).unwrap();
        assert!(store.get_build(&id).unwrap().is_some());

        store.delete_build(&id).unwrap();
        assert!(store.get_build(&id).unwrap().is_none());
    }
}
