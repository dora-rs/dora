mod in_memory;

#[cfg(feature = "redb-backend")]
mod redb_store;

pub use in_memory::InMemoryStore;

/// Maximum allowed length for a param key (bytes).
pub const MAX_PARAM_KEY_BYTES: usize = 256;
/// Maximum allowed size for a serialized param value (bytes).
pub const MAX_PARAM_VALUE_BYTES: usize = 65_536;

/// Validate param key and value size limits.
pub fn validate_param_limits(key: &str, value: &[u8]) -> Result<()> {
    if key.len() > MAX_PARAM_KEY_BYTES {
        eyre::bail!("param key too long (max {MAX_PARAM_KEY_BYTES} bytes)");
    }
    if value.len() > MAX_PARAM_VALUE_BYTES {
        eyre::bail!("param value too large (max {MAX_PARAM_VALUE_BYTES} bytes)");
    }
    Ok(())
}
#[cfg(feature = "redb-backend")]
pub use redb_store::RedbStore;

use adora_message::common::DaemonId;
use adora_message::id::NodeId;
use eyre::Result;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use uuid::Uuid;

// ---------------------------------------------------------------------------
// Persistable types
// ---------------------------------------------------------------------------

/// Minimal information about a connected daemon (transport-agnostic).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonInfo {
    pub daemon_id: DaemonId,
    pub machine_id: Option<String>,
    #[serde(default)]
    pub labels: BTreeMap<String, String>,
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

    // -- Node parameters --

    /// Set a runtime parameter for a node in a dataflow.
    ///
    /// Enforces `MAX_PARAM_KEY_BYTES` and `MAX_PARAM_VALUE_BYTES` limits.
    fn put_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
        value: &[u8],
    ) -> Result<()>;

    /// Get a single runtime parameter.
    fn get_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
    ) -> Result<Option<Vec<u8>>>;

    /// List all runtime parameters for a node.
    fn list_node_params(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
    ) -> Result<Vec<(String, Vec<u8>)>>;

    /// Delete a single runtime parameter.
    fn delete_node_param(&self, dataflow_id: &Uuid, node_id: &NodeId, key: &str) -> Result<()>;
}
