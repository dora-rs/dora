use std::collections::BTreeMap;
use std::sync::RwLock;

use adora_message::common::DaemonId;
use eyre::{Result, eyre};

use crate::{CoordinatorStore, DaemonInfo};

/// In-memory implementation of [`CoordinatorStore`].
///
/// This wraps the same data structures that the coordinator already uses,
/// behind a trait so it can be swapped for a persistent store later.
pub struct InMemoryStore {
    daemons: RwLock<BTreeMap<DaemonId, DaemonInfo>>,
}

impl InMemoryStore {
    pub fn new() -> Self {
        Self {
            daemons: RwLock::new(BTreeMap::new()),
        }
    }
}

impl Default for InMemoryStore {
    fn default() -> Self {
        Self::new()
    }
}

impl CoordinatorStore for InMemoryStore {
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
}
