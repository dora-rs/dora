mod in_memory;

#[cfg(feature = "redb-backend")]
mod redb_store;

pub use in_memory::InMemoryStore;
#[cfg(feature = "redb-backend")]
pub use redb_store::RedbStore;

use adora_message::common::DaemonId;
use eyre::Result;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

// ---------------------------------------------------------------------------
// Persistable types
// ---------------------------------------------------------------------------

/// Minimal information about a connected daemon (transport-agnostic).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonInfo {
    pub daemon_id: DaemonId,
    pub machine_id: Option<String>,
}

/// Persistable dataflow record (desired state + observed status).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataflowRecord {
    pub uuid: Uuid,
    pub name: Option<String>,
    /// Serialized `Descriptor` as JSON -- kept as a string so the store
    /// crate does not depend on the full descriptor type.
    pub descriptor_json: String,
    pub status: DataflowStatus,
    pub daemon_ids: Vec<DaemonId>,
    /// Monotonically increasing version; bumped on every persist.
    pub generation: u64,
    /// Unix epoch milliseconds.
    pub created_at: u64,
    /// Unix epoch milliseconds.
    pub updated_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DataflowStatus {
    Pending,
    Running,
    Stopping,
    Succeeded,
    Failed { error: String },
}

/// Persistable build record.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildRecord {
    pub build_id: Uuid,
    pub status: BuildStatus,
    pub errors: Vec<String>,
    /// Unix epoch milliseconds.
    pub created_at: u64,
    /// Unix epoch milliseconds.
    pub updated_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BuildStatus {
    Pending,
    Succeeded,
    Failed,
}

// ---------------------------------------------------------------------------
// Store trait
// ---------------------------------------------------------------------------

/// Trait abstracting coordinator state storage.
///
/// The `InMemoryStore` implementation preserves the current in-memory behavior.
/// The `RedbStore` implementation (behind the `redb-backend` feature) persists
/// state to disk so the coordinator can recover after a restart.
pub trait CoordinatorStore: Send + Sync {
    // -- Daemon registry --

    fn register_daemon(&self, info: DaemonInfo) -> Result<()>;
    fn unregister_daemon(&self, id: &DaemonId) -> Result<()>;
    fn list_daemons(&self) -> Result<Vec<DaemonInfo>>;
    fn get_daemon(&self, id: &DaemonId) -> Result<Option<DaemonInfo>>;
    fn get_daemon_by_machine(&self, machine_id: &str) -> Result<Option<DaemonId>>;

    // -- Dataflow state --

    fn put_dataflow(&self, record: &DataflowRecord) -> Result<()>;
    fn get_dataflow(&self, uuid: &Uuid) -> Result<Option<DataflowRecord>>;
    fn list_dataflows(&self) -> Result<Vec<DataflowRecord>>;
    fn delete_dataflow(&self, uuid: &Uuid) -> Result<()>;

    // -- Build state --
    // NOTE: Not yet wired into the coordinator event loop (Phase 2).

    fn put_build(&self, record: &BuildRecord) -> Result<()>;
    fn get_build(&self, build_id: &Uuid) -> Result<Option<BuildRecord>>;
    fn list_builds(&self) -> Result<Vec<BuildRecord>>;
    fn delete_build(&self, build_id: &Uuid) -> Result<()>;
}
