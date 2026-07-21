mod in_memory;

#[cfg(feature = "redb-backend")]
mod redb_store;

pub use in_memory::InMemoryStore;

/// Maximum allowed length for a param key (bytes).
pub const MAX_PARAM_KEY_BYTES: usize = 256;
/// Maximum allowed size for a serialized param value (bytes).
pub const MAX_PARAM_VALUE_BYTES: usize = 65_536;

/// Validate param key and value limits.
///
/// This is the single source of truth shared by every [`CoordinatorStore`]
/// backend, so all backends accept or reject identical input. In particular
/// the null byte is rejected here because the redb backend uses it as an
/// internal composite-key separator (see `redb_store::KEY_SEPARATOR`); without
/// this check `put_node_param` with a `\0`-containing key would succeed on the
/// in-memory backend but fail on redb.
pub fn validate_param_limits(key: &str, value: &[u8]) -> Result<()> {
    if key.len() > MAX_PARAM_KEY_BYTES {
        eyre::bail!("param key too long (max {MAX_PARAM_KEY_BYTES} bytes)");
    }
    if key.contains('\0') {
        eyre::bail!("param key must not contain null bytes");
    }
    if value.len() > MAX_PARAM_VALUE_BYTES {
        eyre::bail!("param value too large (max {MAX_PARAM_VALUE_BYTES} bytes)");
    }
    Ok(())
}
#[cfg(feature = "redb-backend")]
pub use redb_store::RedbStore;

use dora_message::common::DaemonId;
use dora_message::id::NodeId;
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
    /// Per-node daemon assignment: node_id -> daemon machine_id.
    /// Used for state reconstruction after coordinator restart.
    #[serde(default)]
    pub node_to_daemon: BTreeMap<String, String>,
    /// Whether the dataflow was started with Python UV support.
    #[serde(default)]
    pub uv: bool,
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
    /// Coordinator restarted; waiting for daemons to reconnect and report.
    /// Transitions to Running once all daemons reconnect, or Failed after timeout.
    Recovering,
    Stopping,
    Succeeded,
    Failed {
        error: String,
        /// Marks the failure as terminal -- subsequent daemon reports
        /// must NOT promote the record back to `Running` via the
        /// reconcile path. Set by coordinator-side failure paths
        /// (e.g. the spawn-timeout watchdog) where the verdict has
        /// already been delivered to the user via `wait_for_spawn`
        /// and resurrection would create an inconsistent
        /// store-vs-CLI view across coordinator restarts.
        ///
        /// `#[serde(default)]` documents the intended semantics for a
        /// missing value, but with the bincode encoding used by
        /// `RedbStore` it does NOT make old bytes decodable on its own
        /// (bincode is not self-describing, so a missing trailing field
        /// fails to decode rather than falling back to `Default`). Records
        /// written before this field existed require a `SCHEMA_VERSION`
        /// bump (see `redb_store.rs`) so old databases are rejected at
        /// `open()` instead of silently losing rows at decode time.
        /// Rescue of [#1593](https://github.com/dora-rs/dora/pull/1593).
        #[serde(default)]
        terminal: bool,
    },
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
    /// Delete a dataflow record. Cascades to every `put_node_param`
    /// row that belongs to the same `uuid` so callers (e.g.
    /// `dora clean`) don't have to walk node params separately and
    /// orphan param rows can't accumulate in long-lived stores.
    ///
    /// The redb backend performs the cascade atomically in one write
    /// transaction; the in-memory backend uses one lock per table
    /// (consistent with the rest of the impl), which is safe because
    /// the coordinator dispatches events serially.
    ///
    /// Returns `Ok(())` if no record exists for `uuid`.
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
