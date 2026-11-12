//! **Internal to dora — not a public API.**
//!
//! This crate is published to crates.io only because cargo requires every
//! dependency of a published crate to be published; `dora-coordinator`
//! depends on it. It is not covered by dora's 1.0 stability
//! guarantee and may change in any release, including a patch.
//!
//! Depend on it directly at your own risk. See the "Stability scope at 1.0"
//! section of `docs/api-rust.md`.
//!
mod in_memory;

#[cfg(feature = "redb-backend")]
mod redb_store;

pub use in_memory::InMemoryStore;

/// Marker prefix of the error produced by `RedbStore::open` when the store on
/// disk is unusable by this binary: either the record schema version differs,
/// or the file predates the redb v3 on-disk format (#2449). Both are recovered
/// the same way -- archive the old file and start fresh -- so they share one
/// marker. The CLI (`dora up`) matches coordinator stderr against this exact
/// string to decide whether to suggest `--recreate-store`, so any rewording
/// must update both sites together (covered end-to-end by
/// `tests/ws-cli-e2e.rs::e2e_up_reports_incompatible_coordinator_store`).
pub const SCHEMA_MISMATCH_MARKER: &str = "redb schema version mismatch";

/// Marker prefix of the error produced by `RedbStore::open` when the store
/// file is damaged: redb's integrity check fails, the file is cut short, or
/// it is not a redb database at all. Recovery is the same as for
/// [`SCHEMA_MISMATCH_MARKER`] -- archive the file and start fresh -- so
/// `dora up` matches this string too and suggests `--recreate-store`. Any
/// rewording must update both sites together (covered end-to-end by
/// `tests/ws-cli-e2e.rs::e2e_up_reports_corrupt_coordinator_store`).
pub const STORE_CORRUPT_MARKER: &str = "redb store is corrupt";

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
///
/// # Examples
///
/// ```
/// use dora_coordinator_store::{validate_param_limits, MAX_PARAM_KEY_BYTES};
///
/// // Ordinary keys and values are accepted.
/// assert!(validate_param_limits("robot_pose", b"{...}").is_ok());
///
/// // A key exactly at the limit is fine; one byte over is rejected.
/// assert!(validate_param_limits(&"k".repeat(MAX_PARAM_KEY_BYTES), b"v").is_ok());
/// assert!(validate_param_limits(&"k".repeat(MAX_PARAM_KEY_BYTES + 1), b"v").is_err());
///
/// // A null byte in the key is rejected so every backend behaves identically
/// // (redb uses `\0` as a composite-key separator).
/// assert!(validate_param_limits("a\0b", b"v").is_err());
/// ```
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
    /// Per-node daemon assignment: node_id -> the assigned daemon's
    /// `DaemonId` display string (`<machine_id>-<uuid>`, or a bare uuid for
    /// an unnamed daemon), not its machine id.
    ///
    /// Informational only: recovery after a coordinator restart rebuilds the
    /// assignment from the nodes each reconnecting daemon reports, not from
    /// this field.
    #[serde(default)]
    pub node_to_daemon: BTreeMap<String, String>,
    /// Whether the dataflow was started with Python UV support.
    #[serde(default)]
    pub uv: bool,
    /// Whether the start barrier (`AllNodesReady`) has already been broadcast.
    ///
    /// Persisted because the in-memory `RunningDataflow` is destroyed whenever
    /// every daemon running the dataflow disconnects (orphan reclaim) or the
    /// coordinator restarts. A daemon that missed the broadcast and reconnects
    /// after that point would otherwise never be replayed it -- and it cannot
    /// prompt a fresh one, because daemon-side `reported_init_to_coordinator`
    /// is never reset (dora-rs/dora#2998).
    #[serde(default)]
    pub ready_barrier_released: bool,
    /// The verdict the barrier carried: nodes that exited before subscribing.
    ///
    /// Persisted alongside the flag so a replay after reconstruction repeats
    /// the *outcome*, not a blank success. Replaying an empty list for a
    /// barrier that actually failed would start a dataflow the coordinator had
    /// already given up on.
    #[serde(default)]
    pub barrier_exited_before_subscribe: Vec<String>,
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
        /// missing value, but with the postcard encoding used by
        /// `RedbStore` it does NOT make old bytes decodable on its own
        /// (postcard is not self-describing, so a missing trailing field
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
    /// row and param tombstone that belongs to the same `uuid` so callers (e.g.
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
    /// Clears the key's tombstone, if any, in the same atomic step.
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
    ///
    /// If the key was set, a tombstone for it is recorded in the same atomic
    /// step, with a fresh token, so a full param replay can tell a daemon that
    /// missed the delete to remove the key from its node. A tombstone lasts
    /// until a replay delivers the delete (`clear_node_param_tombstone`), the
    /// key is set again, or the dataflow is deleted. A key that was never set
    /// gets none (no node can have it), and deleting an already deleted key
    /// keeps its tombstone and token.
    fn delete_node_param(&self, dataflow_id: &Uuid, node_id: &NodeId, key: &str) -> Result<()>;

    /// List the tombstoned param keys of a node with their tokens (see
    /// `delete_node_param`).
    fn list_node_param_tombstones(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
    ) -> Result<Vec<(String, Uuid)>>;

    /// Remove a tombstone, but only if it still has `token`: one written
    /// since the token was listed (the key was set and deleted again) stays.
    fn clear_node_param_tombstone(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
        token: Uuid,
    ) -> Result<()>;
}

/// Tombstone behavior every backend must share, run from each backend's tests.
#[cfg(test)]
pub(crate) mod tombstone_tests {
    use super::*;

    pub(crate) fn tombstoned_keys(
        store: &dyn CoordinatorStore,
        df: &Uuid,
        node: &NodeId,
    ) -> Vec<String> {
        store
            .list_node_param_tombstones(df, node)
            .unwrap()
            .into_iter()
            .map(|(key, _)| key)
            .collect()
    }

    /// A tombstone is cleared only with the token it was listed with: one
    /// written by a later delete of the same key survives a stale clear.
    pub(crate) fn clear_matches_only_its_token(store: &dyn CoordinatorStore) {
        let df = Uuid::new_v4();
        let node: NodeId = "sensor".to_string().into();
        let token = |store: &dyn CoordinatorStore| {
            let tombstones = store.list_node_param_tombstones(&df, &node).unwrap();
            assert_eq!(tombstones.len(), 1, "{tombstones:?}");
            tombstones[0].1
        };
        store.put_node_param(&df, &node, "gain", b"1").unwrap();
        store.delete_node_param(&df, &node, "gain").unwrap();
        let first = token(store);
        store.delete_node_param(&df, &node, "gain").unwrap();
        assert_eq!(token(store), first, "re-deleting keeps the token");

        store.put_node_param(&df, &node, "gain", b"2").unwrap();
        store.delete_node_param(&df, &node, "gain").unwrap();
        let second = token(store);
        assert_ne!(second, first, "a new delete gets a new token");
        store
            .clear_node_param_tombstone(&df, &node, "gain", first)
            .unwrap();
        assert_eq!(token(store), second, "a stale clear is a no-op");

        store
            .clear_node_param_tombstone(&df, &node, "gain", second)
            .unwrap();
        assert!(tombstoned_keys(store, &df, &node).is_empty());
    }

    /// Deleting a set key tombstones it, deleting a key that was never set
    /// does not, and setting the key again clears the tombstone.
    pub(crate) fn delete_records_a_tombstone_that_put_clears(store: &dyn CoordinatorStore) {
        let df = Uuid::new_v4();
        let node: NodeId = "sensor".to_string().into();
        let other: NodeId = "sensor2".to_string().into();
        store.put_node_param(&df, &node, "gain", b"1").unwrap();
        store.put_node_param(&df, &other, "gain", b"1").unwrap();

        store.delete_node_param(&df, &node, "gain").unwrap();
        store.delete_node_param(&df, &node, "never_set").unwrap();
        assert_eq!(tombstoned_keys(store, &df, &node), vec!["gain".to_string()]);
        assert!(tombstoned_keys(store, &df, &other).is_empty());

        store.put_node_param(&df, &node, "gain", b"2").unwrap();
        assert!(tombstoned_keys(store, &df, &node).is_empty());
    }
}
