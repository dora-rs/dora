use std::path::Path;

use dora_message::common::DaemonId;
use eyre::{Result, WrapErr, eyre};
use redb::{Database, ReadableTable, TableDefinition};
use uuid::Uuid;

use dora_message::id::NodeId;

use crate::{BuildRecord, CoordinatorStore, DaemonInfo, DataflowRecord, validate_param_limits};

const META: TableDefinition<&str, u32> = TableDefinition::new("meta");
const DAEMONS: TableDefinition<&[u8], &[u8]> = TableDefinition::new("daemons");
const DATAFLOWS: TableDefinition<&[u8], &[u8]> = TableDefinition::new("dataflows");
const BUILDS: TableDefinition<&[u8], &[u8]> = TableDefinition::new("builds");
/// Node params table. Key: "{uuid}/{node_id}/{param_key}", Value: JSON bytes.
const NODE_PARAMS: TableDefinition<&str, &[u8]> = TableDefinition::new("node_params");

/// Bump this when the serialization format of any stored record changes.
const SCHEMA_VERSION: u32 = 2; // Sprint 9: added node_to_daemon, uv, Recovering status
const SCHEMA_VERSION_KEY: &str = "schema_version";

/// Run `f` with umask set to `0o077` (owner-only) on Unix, restoring afterwards.
/// This ensures files/dirs created inside `f` have restrictive permissions from
/// the start, closing the TOCTOU window between creation and `set_permissions`.
#[cfg(unix)]
fn with_restrictive_umask<T>(f: impl FnOnce() -> T) -> T {
    // SAFETY: umask is always safe to call and has no undefined behavior.
    let old = unsafe { libc::umask(0o077) };
    let result = f();
    unsafe { libc::umask(old) };
    result
}

#[cfg(not(unix))]
fn with_restrictive_umask<T>(f: impl FnOnce() -> T) -> T {
    f()
}

/// Persistent [`CoordinatorStore`] backed by [redb](https://docs.rs/redb).
///
/// All state is written to a single file on disk. The store is crash-safe:
/// redb uses copy-on-write B-trees that survive unexpected process termination.
pub struct RedbStore {
    db: Database,
}

impl RedbStore {
    /// Open (or create) a redb database at the given path.
    ///
    /// On first open, writes the current [`SCHEMA_VERSION`] into a `meta`
    /// table. On subsequent opens, validates that the stored version matches
    /// the compiled-in version and returns an error if they differ.
    pub fn open(path: &Path) -> Result<Self> {
        let db = with_restrictive_umask(|| Database::create(path))
            .wrap_err("failed to open redb database")?;

        // Restrict file permissions to owner-only on Unix.
        // NOTE: On Windows, file permissions are governed by ACLs and not
        // enforced here. Administrators should use NTFS ACLs to restrict
        // access to the database file in production deployments.
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))
                .wrap_err("failed to set redb file permissions to 0600")?;
        }

        // Ensure tables exist and check schema version.
        let txn = db
            .begin_write()
            .wrap_err("failed to begin write transaction")?;
        {
            let mut meta = txn
                .open_table(META)
                .wrap_err("failed to create meta table")?;
            let stored = meta.get(SCHEMA_VERSION_KEY)?.map(|v| v.value());
            match stored {
                Some(v) if v != SCHEMA_VERSION => {
                    return Err(eyre!(
                        "redb schema version mismatch: database at `{}` has v{v}, \
                         but this binary expects v{SCHEMA_VERSION}. \
                         Delete the file and restart to create a fresh database, \
                         or use `--store memory` to bypass persistence.",
                        path.display()
                    ));
                }
                Some(_) => {} // version matches
                None => {
                    meta.insert(SCHEMA_VERSION_KEY, SCHEMA_VERSION)?;
                }
            }
        }
        txn.open_table(DAEMONS)
            .wrap_err("failed to create daemons table")?;
        txn.open_table(DATAFLOWS)
            .wrap_err("failed to create dataflows table")?;
        txn.open_table(BUILDS)
            .wrap_err("failed to create builds table")?;
        txn.open_table(NODE_PARAMS)
            .wrap_err("failed to create node_params table")?;
        txn.commit().wrap_err("failed to commit table creation")?;

        Ok(Self { db })
    }
}

// ---------------------------------------------------------------------------
// Serialisation helpers
// ---------------------------------------------------------------------------

fn encode<T: serde::Serialize>(value: &T) -> Result<Vec<u8>> {
    bincode::serde::encode_to_vec(value, bincode::config::standard())
        .map_err(|e| eyre!("encode error: {e}"))
}

/// Decode uses a separate config with a 64 MiB limit that `encode` intentionally
/// omits: encoding our own data needs no guard, but decoding potentially corrupt
/// data must cap allocation sizes to prevent OOM from malformed varint length
/// prefixes. The limit type is a const generic in bincode 2.x, so it cannot
/// share a single `const` with the unlimited encode config.
fn decode<T: serde::de::DeserializeOwned>(bytes: &[u8]) -> Result<T> {
    let config = bincode::config::standard().with_limit::<{ 64 * 1024 * 1024 }>();
    let (val, _) =
        bincode::serde::decode_from_slice(bytes, config).map_err(|e| eyre!("decode error: {e}"))?;
    Ok(val)
}

// ---------------------------------------------------------------------------
// CoordinatorStore implementation
// ---------------------------------------------------------------------------

impl CoordinatorStore for RedbStore {
    // -- Daemon registry --

    fn register_daemon(&self, info: DaemonInfo) -> Result<()> {
        let key = encode(&info.daemon_id)?;
        let value = encode(&info)?;
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(DAEMONS)?;
            table.insert(key.as_slice(), value.as_slice())?;
        }
        txn.commit()?;
        Ok(())
    }

    fn unregister_daemon(&self, id: &DaemonId) -> Result<()> {
        let key = encode(id)?;
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(DAEMONS)?;
            table.remove(key.as_slice())?;
        }
        txn.commit()?;
        Ok(())
    }

    fn list_daemons(&self) -> Result<Vec<DaemonInfo>> {
        let txn = self.db.begin_read()?;
        let table = txn.open_table(DAEMONS)?;
        let mut result = Vec::new();
        for entry in table.iter()? {
            let (_, value) = entry?;
            result.push(decode(value.value())?);
        }
        Ok(result)
    }

    fn get_daemon(&self, id: &DaemonId) -> Result<Option<DaemonInfo>> {
        let key = encode(id)?;
        let txn = self.db.begin_read()?;
        let table = txn.open_table(DAEMONS)?;
        match table.get(key.as_slice())? {
            Some(value) => Ok(Some(decode(value.value())?)),
            None => Ok(None),
        }
    }

    fn get_daemon_by_machine(&self, machine_id: &str) -> Result<Option<DaemonId>> {
        // Scan -- daemon count is small (single digits).
        let daemons = self.list_daemons()?;
        Ok(daemons
            .into_iter()
            .find(|info| info.daemon_id.matches_machine_id(machine_id))
            .map(|info| info.daemon_id))
    }

    // -- Dataflow state --

    fn put_dataflow(&self, record: &DataflowRecord) -> Result<()> {
        let key: &[u8] = record.uuid.as_bytes();
        let value = encode(record)?;
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(DATAFLOWS)?;
            table.insert(key, value.as_slice())?;
        }
        txn.commit()?;
        Ok(())
    }

    fn get_dataflow(&self, uuid: &Uuid) -> Result<Option<DataflowRecord>> {
        let key: &[u8] = uuid.as_bytes();
        let txn = self.db.begin_read()?;
        let table = txn.open_table(DATAFLOWS)?;
        match table.get(key)? {
            Some(value) => Ok(Some(decode(value.value())?)),
            None => Ok(None),
        }
    }

    fn list_dataflows(&self) -> Result<Vec<DataflowRecord>> {
        let txn = self.db.begin_read()?;
        let table = txn.open_table(DATAFLOWS)?;
        let mut result = Vec::new();
        for entry in table.iter()? {
            let (_, value) = entry?;
            result.push(decode(value.value())?);
        }
        Ok(result)
    }

    fn delete_dataflow(&self, uuid: &Uuid) -> Result<()> {
        let key: &[u8] = uuid.as_bytes();
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(DATAFLOWS)?;
            table.remove(key)?;
        }
        txn.commit()?;
        Ok(())
    }

    // -- Build state --

    fn put_build(&self, record: &BuildRecord) -> Result<()> {
        let key: &[u8] = record.build_id.as_bytes();
        let value = encode(record)?;
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(BUILDS)?;
            table.insert(key, value.as_slice())?;
        }
        txn.commit()?;
        Ok(())
    }

    fn get_build(&self, build_id: &Uuid) -> Result<Option<BuildRecord>> {
        let key: &[u8] = build_id.as_bytes();
        let txn = self.db.begin_read()?;
        let table = txn.open_table(BUILDS)?;
        match table.get(key)? {
            Some(value) => Ok(Some(decode(value.value())?)),
            None => Ok(None),
        }
    }

    fn list_builds(&self) -> Result<Vec<BuildRecord>> {
        let txn = self.db.begin_read()?;
        let table = txn.open_table(BUILDS)?;
        let mut result = Vec::new();
        for entry in table.iter()? {
            let (_, value) = entry?;
            result.push(decode(value.value())?);
        }
        Ok(result)
    }

    fn delete_build(&self, build_id: &Uuid) -> Result<()> {
        let key: &[u8] = build_id.as_bytes();
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(BUILDS)?;
            table.remove(key)?;
        }
        txn.commit()?;
        Ok(())
    }

    // -- Node parameters --

    fn put_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
        value: &[u8],
    ) -> Result<()> {
        validate_param_limits(key, value)?;
        let param_key = make_param_key(dataflow_id, node_id, key)?;
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(NODE_PARAMS)?;
            table.insert(param_key.as_str(), value)?;
        }
        txn.commit()?;
        Ok(())
    }

    fn get_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
    ) -> Result<Option<Vec<u8>>> {
        let param_key = make_param_key(dataflow_id, node_id, key)?;
        let txn = self.db.begin_read()?;
        let table = txn.open_table(NODE_PARAMS)?;
        match table.get(param_key.as_str())? {
            Some(value) => Ok(Some(value.value().to_vec())),
            None => Ok(None),
        }
    }

    fn list_node_params(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
    ) -> Result<Vec<(String, Vec<u8>)>> {
        let prefix = make_param_prefix(dataflow_id, node_id)?;
        let txn = self.db.begin_read()?;
        let table = txn.open_table(NODE_PARAMS)?;
        let mut result = Vec::new();
        // Range scan: keys are sorted, so we start at the prefix and stop
        // when keys no longer match.
        for entry in table.range(prefix.as_str()..)? {
            let (key, value) = entry?;
            let key_str = key.value();
            match key_str.strip_prefix(&prefix) {
                Some(param_key) => {
                    result.push((param_key.to_string(), value.value().to_vec()));
                }
                None => break,
            }
        }
        Ok(result)
    }

    fn delete_node_param(&self, dataflow_id: &Uuid, node_id: &NodeId, key: &str) -> Result<()> {
        let param_key = make_param_key(dataflow_id, node_id, key)?;
        let txn = self.db.begin_write()?;
        {
            let mut table = txn.open_table(NODE_PARAMS)?;
            table.remove(param_key.as_str())?;
        }
        txn.commit()?;
        Ok(())
    }
}

/// Separator used in redb composite keys. Node IDs and param keys must not
/// contain this character to prevent key collisions.
const KEY_SEPARATOR: char = '\0';

/// Check that a string component does not contain the key separator.
fn check_no_separator(s: &str, label: &str) -> Result<()> {
    if s.contains(KEY_SEPARATOR) {
        eyre::bail!("{label} must not contain null bytes");
    }
    Ok(())
}

/// Build a composite redb key: `"{dataflow_id}\0{node_id}\0{key}"`.
fn make_param_key(dataflow_id: &Uuid, node_id: &NodeId, key: &str) -> Result<String> {
    let node_str: &str = AsRef::<str>::as_ref(node_id);
    check_no_separator(node_str, "node_id")?;
    check_no_separator(key, "param key")?;
    Ok(format!(
        "{dataflow_id}{sep}{node_str}{sep}{key}",
        sep = KEY_SEPARATOR
    ))
}

/// Build the prefix for listing params: `"{dataflow_id}\0{node_id}\0"`.
fn make_param_prefix(dataflow_id: &Uuid, node_id: &NodeId) -> Result<String> {
    let node_str: &str = AsRef::<str>::as_ref(node_id);
    check_no_separator(node_str, "node_id")?;
    Ok(format!(
        "{dataflow_id}{sep}{node_str}{sep}",
        sep = KEY_SEPARATOR
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_store() -> (RedbStore, tempfile::TempDir) {
        let dir = tempfile::tempdir().unwrap();
        let store = RedbStore::open(&dir.path().join("test.redb")).unwrap();
        (store, dir)
    }

    #[test]
    fn daemon_crud() {
        let (store, _dir) = temp_store();
        let id = DaemonId::new(Some("machine-1".into()));
        let info = DaemonInfo {
            daemon_id: id.clone(),
            machine_id: Some("machine-1".into()),
            labels: Default::default(),
        };

        store.register_daemon(info.clone()).unwrap();
        assert!(store.get_daemon(&id).unwrap().is_some());
        assert_eq!(store.list_daemons().unwrap().len(), 1);
        assert!(store.get_daemon_by_machine("machine-1").unwrap().is_some());

        store.unregister_daemon(&id).unwrap();
        assert!(store.get_daemon(&id).unwrap().is_none());
        assert!(store.list_daemons().unwrap().is_empty());
    }

    #[test]
    fn dataflow_crud() {
        let (store, _dir) = temp_store();
        let uuid = Uuid::new_v4();
        let record = DataflowRecord {
            uuid,
            name: Some("test-flow".into()),
            descriptor_json: "nodes: []".into(),
            status: crate::DataflowStatus::Pending,
            daemon_ids: vec![],
            generation: 0,
            created_at: 1000,
            updated_at: 1000,
            node_to_daemon: Default::default(),
            uv: false,
        };

        store.put_dataflow(&record).unwrap();
        let loaded = store.get_dataflow(&uuid).unwrap().unwrap();
        assert_eq!(loaded.uuid, uuid);
        assert_eq!(loaded.name.as_deref(), Some("test-flow"));
        assert_eq!(loaded.status, crate::DataflowStatus::Pending);

        assert_eq!(store.list_dataflows().unwrap().len(), 1);

        store.delete_dataflow(&uuid).unwrap();
        assert!(store.get_dataflow(&uuid).unwrap().is_none());
    }

    #[test]
    fn build_crud() {
        let (store, _dir) = temp_store();
        let id = Uuid::new_v4();
        let record = BuildRecord {
            build_id: id,
            status: crate::BuildStatus::Pending,
            errors: vec![],
            created_at: 2000,
            updated_at: 2000,
        };

        store.put_build(&record).unwrap();
        let loaded = store.get_build(&id).unwrap().unwrap();
        assert_eq!(loaded.build_id, id);
        assert_eq!(loaded.status, crate::BuildStatus::Pending);

        store.delete_build(&id).unwrap();
        assert!(store.get_build(&id).unwrap().is_none());
    }

    #[test]
    fn dataflow_update_persists() {
        let (store, _dir) = temp_store();
        let uuid = Uuid::new_v4();
        let mut record = DataflowRecord {
            uuid,
            name: None,
            descriptor_json: "nodes: []".into(),
            status: crate::DataflowStatus::Pending,
            daemon_ids: vec![],
            generation: 0,
            created_at: 1000,
            updated_at: 1000,
            node_to_daemon: Default::default(),
            uv: false,
        };

        store.put_dataflow(&record).unwrap();

        record.status = crate::DataflowStatus::Running;
        record.generation = 1;
        record.updated_at = 2000;
        store.put_dataflow(&record).unwrap();

        let loaded = store.get_dataflow(&uuid).unwrap().unwrap();
        assert_eq!(loaded.status, crate::DataflowStatus::Running);
        assert_eq!(loaded.generation, 1);
    }

    #[test]
    fn persistence_across_reopen() {
        let dir = tempfile::tempdir().unwrap();
        let db_path = dir.path().join("persist.redb");
        let uuid = Uuid::new_v4();

        {
            let store = RedbStore::open(&db_path).unwrap();
            let record = DataflowRecord {
                uuid,
                name: Some("persistent".into()),
                descriptor_json: "nodes: []".into(),
                status: crate::DataflowStatus::Running,
                daemon_ids: vec![],
                generation: 1,
                created_at: 1000,
                updated_at: 2000,
                node_to_daemon: Default::default(),
                uv: false,
            };
            store.put_dataflow(&record).unwrap();
        }

        // Reopen the database -- simulates coordinator restart.
        {
            let store = RedbStore::open(&db_path).unwrap();
            let loaded = store.get_dataflow(&uuid).unwrap().unwrap();
            assert_eq!(loaded.name.as_deref(), Some("persistent"));
            assert_eq!(loaded.status, crate::DataflowStatus::Running);
        }
    }

    #[test]
    fn reopen_same_version_succeeds() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("version.redb");
        RedbStore::open(&path).unwrap();
        // Reopening with the same SCHEMA_VERSION should succeed.
        RedbStore::open(&path).unwrap();
    }

    #[test]
    fn param_crud() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "camera".to_string().into();

        store.put_node_param(&df, &node, "fps", b"30").unwrap();
        assert_eq!(
            store.get_node_param(&df, &node, "fps").unwrap(),
            Some(b"30".to_vec())
        );

        store
            .put_node_param(&df, &node, "resolution", b"1080")
            .unwrap();
        let params = store.list_node_params(&df, &node).unwrap();
        assert_eq!(params.len(), 2);

        store.delete_node_param(&df, &node, "fps").unwrap();
        assert!(store.get_node_param(&df, &node, "fps").unwrap().is_none());
        assert_eq!(store.list_node_params(&df, &node).unwrap().len(), 1);
    }

    #[test]
    fn schema_version_mismatch_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("mismatch.redb");

        // Create a database and write a different schema version directly.
        {
            let db = Database::create(&path).unwrap();
            let txn = db.begin_write().unwrap();
            {
                let mut meta = txn.open_table(META).unwrap();
                meta.insert(SCHEMA_VERSION_KEY, SCHEMA_VERSION + 1).unwrap();
            }
            txn.commit().unwrap();
        }

        // RedbStore::open should refuse due to version mismatch.
        match RedbStore::open(&path) {
            Ok(_) => panic!("expected schema version mismatch error"),
            Err(e) => {
                let msg = format!("{e}");
                assert!(
                    msg.contains("schema version mismatch"),
                    "expected schema version mismatch error, got: {msg}"
                );
            }
        }
    }

    // --- Focused tests for redb_store edge cases (added 2026-04-08) ---
    //
    // Added during the POC's follow-up case study on the persistence
    // layer. 52 of 59 missed mutants in dora-coordinator-store were in
    // this file. These tests cover boundary conditions, separator
    // handling, and prefix-scan behavior that the basic CRUD tests
    // did not exercise.

    use crate::{MAX_PARAM_KEY_BYTES, MAX_PARAM_VALUE_BYTES};

    /// Empty param KEY: currently allowed and stored under the prefix-only
    /// marker. Pins the behavior. If this test fails because empty keys
    /// start being rejected, update the doc on `put_node_param` and the
    /// failure case in `list_node_params` to match.
    #[test]
    fn param_empty_key_is_accepted_and_round_trips() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "camera".to_string().into();

        store.put_node_param(&df, &node, "", b"value").unwrap();
        let got = store.get_node_param(&df, &node, "").unwrap();
        assert_eq!(got, Some(b"value".to_vec()));

        // list_node_params should also return it under key ""
        let list = store.list_node_params(&df, &node).unwrap();
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].0, "");
        assert_eq!(list[0].1, b"value");
    }

    /// Empty param VALUE: allowed (a common "unset" sentinel).
    #[test]
    fn param_empty_value_round_trips() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "n".to_string().into();
        store.put_node_param(&df, &node, "k", b"").unwrap();
        assert_eq!(store.get_node_param(&df, &node, "k").unwrap(), Some(vec![]));
    }

    /// Param key at exactly MAX_PARAM_KEY_BYTES is accepted; one byte
    /// over is rejected. Pins the inclusive/exclusive boundary.
    #[test]
    fn param_key_length_boundary() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "n".to_string().into();

        let max_key = "a".repeat(MAX_PARAM_KEY_BYTES);
        store.put_node_param(&df, &node, &max_key, b"ok").unwrap();
        assert_eq!(
            store.get_node_param(&df, &node, &max_key).unwrap(),
            Some(b"ok".to_vec())
        );

        let too_long = "a".repeat(MAX_PARAM_KEY_BYTES + 1);
        let err = store
            .put_node_param(&df, &node, &too_long, b"bad")
            .unwrap_err();
        assert!(
            err.to_string().contains("too long"),
            "expected 'too long' error, got: {err}"
        );
    }

    /// Param value at exactly MAX_PARAM_VALUE_BYTES is accepted; one byte
    /// over is rejected. Pins the inclusive/exclusive boundary.
    #[test]
    fn param_value_length_boundary() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "n".to_string().into();

        let max_val = vec![0u8; MAX_PARAM_VALUE_BYTES];
        store.put_node_param(&df, &node, "k", &max_val).unwrap();
        assert_eq!(
            store
                .get_node_param(&df, &node, "k")
                .unwrap()
                .map(|v| v.len()),
            Some(MAX_PARAM_VALUE_BYTES)
        );

        let too_large = vec![0u8; MAX_PARAM_VALUE_BYTES + 1];
        let err = store
            .put_node_param(&df, &node, "k2", &too_large)
            .unwrap_err();
        assert!(
            err.to_string().contains("too large"),
            "expected 'too large' error, got: {err}"
        );
    }

    /// Null-byte attack on node id is caught at the NodeId type boundary,
    /// before reaching redb_store. This test pins the layered defense:
    /// NodeId validation via `FromStr` rejects null bytes (and other
    /// invalid chars) at construction, so `check_no_separator` in
    /// `make_param_key` is effectively dead code for node_id — a
    /// deliberate belt-and-braces check.
    ///
    /// NOTE on the API: do NOT use `NodeId::try_from("...".to_string())`
    /// for untrusted input. Despite the doc comment on `From<String>`,
    /// `TryFrom` is the auto-derived blanket impl which delegates to
    /// `From` — and `From<String>` PANICS on invalid input. The actual
    /// safe path is `s.parse::<NodeId>()` which uses `FromStr` and
    /// returns `Result<Self, InvalidId>`. Discovered 2026-04-08.
    #[test]
    fn node_id_with_null_byte_is_rejected_at_type_boundary() {
        use std::str::FromStr;
        let result = NodeId::from_str("foo\0bar");
        assert!(
            result.is_err(),
            "NodeId::from_str must reject strings containing null bytes"
        );
    }

    /// Param key containing a null byte is rejected.
    #[test]
    fn param_key_with_null_byte_is_rejected() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "n".to_string().into();
        let err = store.put_node_param(&df, &node, "k\0ey", b"v").unwrap_err();
        assert!(
            err.to_string().contains("null bytes"),
            "expected 'null bytes' error, got: {err}"
        );
    }

    /// list_node_params must isolate params across dataflows: putting a
    /// param under df_a/node_x must not show up when listing df_b/node_x.
    #[test]
    fn list_node_params_isolates_across_dataflows() {
        let (store, _dir) = temp_store();
        let df_a = Uuid::new_v4();
        let df_b = Uuid::new_v4();
        let node: NodeId = "shared".to_string().into();

        store.put_node_param(&df_a, &node, "k1", b"a").unwrap();
        store.put_node_param(&df_b, &node, "k2", b"b").unwrap();

        let list_a = store.list_node_params(&df_a, &node).unwrap();
        let list_b = store.list_node_params(&df_b, &node).unwrap();
        assert_eq!(list_a.len(), 1);
        assert_eq!(list_a[0].0, "k1");
        assert_eq!(list_b.len(), 1);
        assert_eq!(list_b[0].0, "k2");
    }

    /// list_node_params must isolate params across nodes in the same
    /// dataflow: the prefix scan must stop at the node boundary.
    /// This exercises the strip_prefix `None => break` path in
    /// redb_store.rs list_node_params.
    #[test]
    fn list_node_params_isolates_across_nodes_in_same_dataflow() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node_a: NodeId = "a".to_string().into();
        let node_b: NodeId = "b".to_string().into();

        store.put_node_param(&df, &node_a, "k1", b"va").unwrap();
        store.put_node_param(&df, &node_b, "k2", b"vb").unwrap();

        let list_a = store.list_node_params(&df, &node_a).unwrap();
        assert_eq!(list_a.len(), 1);
        assert_eq!(list_a[0].0, "k1");
        assert_eq!(list_a[0].1, b"va");
    }

    /// list_node_params must isolate params when one node_id is a prefix
    /// of another (e.g., "node" vs "node_extended"). The null byte
    /// separator ensures this, but the test pins the behavior.
    #[test]
    fn list_node_params_isolates_when_node_id_is_prefix_of_another() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "node".to_string().into();
        let node_long: NodeId = "node_extended".to_string().into();

        store.put_node_param(&df, &node, "k", b"short").unwrap();
        store.put_node_param(&df, &node_long, "k", b"long").unwrap();

        let list = store.list_node_params(&df, &node).unwrap();
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].1, b"short");

        let list_long = store.list_node_params(&df, &node_long).unwrap();
        assert_eq!(list_long.len(), 1);
        assert_eq!(list_long[0].1, b"long");
    }

    /// list_node_params returns empty when no params exist (no accidental
    /// returns from unrelated rows).
    #[test]
    fn list_node_params_empty_when_no_params() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "lonely".to_string().into();

        // Populate unrelated data to verify the scan still returns empty.
        let other_df = Uuid::new_v4();
        let other_node: NodeId = "busy".to_string().into();
        store
            .put_node_param(&other_df, &other_node, "k", b"v")
            .unwrap();

        let list = store.list_node_params(&df, &node).unwrap();
        assert!(list.is_empty());
    }

    /// list_node_params returns all keys under a node, sorted
    /// alphabetically (redb range scan is sorted).
    #[test]
    fn list_node_params_returns_sorted_keys() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "n".to_string().into();

        store.put_node_param(&df, &node, "z", b"3").unwrap();
        store.put_node_param(&df, &node, "a", b"1").unwrap();
        store.put_node_param(&df, &node, "m", b"2").unwrap();

        let keys: Vec<String> = store
            .list_node_params(&df, &node)
            .unwrap()
            .into_iter()
            .map(|(k, _)| k)
            .collect();
        assert_eq!(keys, vec!["a", "m", "z"]);
    }

    /// register_daemon is idempotent: registering twice with the same id
    /// updates rather than duplicates.
    #[test]
    fn register_daemon_is_idempotent_update() {
        let (store, _dir) = temp_store();
        let id = DaemonId::new(Some("host".into()));

        let info_v1 = DaemonInfo {
            daemon_id: id.clone(),
            machine_id: Some("host".into()),
            labels: [("env".to_string(), "dev".to_string())].into(),
        };
        store.register_daemon(info_v1).unwrap();

        let info_v2 = DaemonInfo {
            daemon_id: id.clone(),
            machine_id: Some("host".into()),
            labels: [("env".to_string(), "prod".to_string())].into(),
        };
        store.register_daemon(info_v2).unwrap();

        // Should be exactly one daemon, with the updated label.
        let daemons = store.list_daemons().unwrap();
        assert_eq!(daemons.len(), 1);
        assert_eq!(
            daemons[0].labels.get("env").map(String::as_str),
            Some("prod")
        );
    }

    /// delete_dataflow on a non-existent uuid is a no-op, not an error.
    #[test]
    fn delete_dataflow_missing_is_not_error() {
        let (store, _dir) = temp_store();
        let uuid = Uuid::new_v4();
        // Nothing was stored. delete should still succeed.
        store.delete_dataflow(&uuid).unwrap();
    }

    /// delete_node_param on a non-existent key is a no-op.
    #[test]
    fn delete_node_param_missing_is_not_error() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "n".to_string().into();
        store.delete_node_param(&df, &node, "never-set").unwrap();
    }

    /// After delete, get returns None and subsequent put works (the key
    /// can be reused). Pins redb-specific tombstone vs reinsert behavior.
    #[test]
    fn delete_then_reinsert_works() {
        let (store, _dir) = temp_store();
        let df = Uuid::new_v4();
        let node: NodeId = "n".to_string().into();

        store.put_node_param(&df, &node, "k", b"v1").unwrap();
        store.delete_node_param(&df, &node, "k").unwrap();
        assert!(store.get_node_param(&df, &node, "k").unwrap().is_none());
        store.put_node_param(&df, &node, "k", b"v2").unwrap();
        assert_eq!(
            store.get_node_param(&df, &node, "k").unwrap(),
            Some(b"v2".to_vec())
        );
    }

    /// Schema version exactly equal to SCHEMA_VERSION succeeds on reopen.
    /// Complements `schema_version_mismatch_is_rejected` by pinning the
    /// match-arm boundary: guard `v != SCHEMA_VERSION` must be FALSE for
    /// the currently compiled version.
    #[test]
    fn reopen_exact_schema_version_succeeds() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("exact.redb");

        // First open writes SCHEMA_VERSION; second must accept it.
        let _s1 = RedbStore::open(&path).unwrap();
        drop(_s1);
        let _s2 = RedbStore::open(&path).unwrap();
    }

    /// Schema version SCHEMA_VERSION - 1 is rejected (older file).
    /// Complements the "+1" test already above. Both directions pinned.
    #[test]
    fn older_schema_version_is_rejected() {
        if SCHEMA_VERSION == 0 {
            return; // can't represent v-1
        }
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("older.redb");
        {
            let db = Database::create(&path).unwrap();
            let txn = db.begin_write().unwrap();
            {
                let mut meta = txn.open_table(META).unwrap();
                meta.insert(SCHEMA_VERSION_KEY, SCHEMA_VERSION - 1).unwrap();
            }
            txn.commit().unwrap();
        }
        match RedbStore::open(&path) {
            Ok(_) => panic!("expected schema version mismatch error"),
            Err(err) => assert!(
                err.to_string().contains("schema version mismatch"),
                "expected schema version mismatch, got: {err}"
            ),
        }
    }

    /// A corrupt bincode blob in the dataflows table surfaces as an error
    /// on the read path, not a panic. This is the file-corruption scenario
    /// from plan-fault-injection.md section 3.3 at the API level.
    #[test]
    fn corrupt_dataflow_record_returns_error_not_panic() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("corrupt.redb");
        let uuid = Uuid::new_v4();

        // Open store, write a garbage blob directly into DATAFLOWS, close.
        {
            let store = RedbStore::open(&path).unwrap();
            let txn = store.db.begin_write().unwrap();
            {
                let mut table = txn.open_table(DATAFLOWS).unwrap();
                let garbage: &[u8] = b"not a valid bincode record XXXXXXXX";
                let key: &[u8] = uuid.as_bytes();
                table.insert(key, garbage).unwrap();
            }
            txn.commit().unwrap();
        }

        // Reopen and attempt to read. get_dataflow should return an error
        // (decode failure), not panic and not silently succeed.
        let store = RedbStore::open(&path).unwrap();
        let result = store.get_dataflow(&uuid);
        assert!(
            result.is_err(),
            "expected decode error on corrupt blob, got Ok({:?})",
            result.ok()
        );
        let err_msg = result.unwrap_err().to_string();
        assert!(
            err_msg.contains("decode"),
            "expected error mentioning 'decode', got: {err_msg}"
        );
    }
}
