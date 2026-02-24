mod in_memory;

pub use in_memory::InMemoryStore;

use adora_message::common::DaemonId;
use eyre::Result;

/// Minimal information about a connected daemon (transport-agnostic).
#[derive(Debug, Clone)]
pub struct DaemonInfo {
    pub daemon_id: DaemonId,
    pub machine_id: Option<String>,
}

/// Trait abstracting coordinator state storage.
///
/// The initial implementation is in-memory (same data as today).
/// Later, this can be swapped for etcd/sqlite to enable stateless coordinators.
pub trait CoordinatorStore: Send + Sync {
    // -- Daemon registry --

    fn register_daemon(&self, info: DaemonInfo) -> Result<()>;
    fn unregister_daemon(&self, id: &DaemonId) -> Result<()>;
    fn list_daemons(&self) -> Result<Vec<DaemonInfo>>;
    fn get_daemon(&self, id: &DaemonId) -> Result<Option<DaemonInfo>>;
    fn get_daemon_by_machine(&self, machine_id: &str) -> Result<Option<DaemonId>>;
}
