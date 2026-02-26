use std::path::Path;

use adora_message::common::DaemonId;
use eyre::{Result, WrapErr, eyre};
use redb::{Database, ReadableTable, TableDefinition};
use uuid::Uuid;

use crate::{BuildRecord, CoordinatorStore, DaemonInfo, DataflowRecord};

const META: TableDefinition<&str, u32> = TableDefinition::new("meta");
const DAEMONS: TableDefinition<&[u8], &[u8]> = TableDefinition::new("daemons");
const DATAFLOWS: TableDefinition<&[u8], &[u8]> = TableDefinition::new("dataflows");
const BUILDS: TableDefinition<&[u8], &[u8]> = TableDefinition::new("builds");

/// Bump this when the serialization format of any stored record changes.
const SCHEMA_VERSION: u32 = 1;
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
}
