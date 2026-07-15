use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Identifier for a memory pool buffer, scoped by dataflow.
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct MemoryPoolId {
    /// The dataflow that owns this pool.
    pub dataflow_id: String,
    /// The per-node buffer identifier.
    pub id: String,
}

/// Metadata for a memory pool tensor.
///
/// # Cross-process safety
///
/// `ptr` is the virtual address of the tensor data **in the registering
/// process**. It is meaningless in any other process — consumers **must**
/// retrieve the data pointer via `shared_memory_name` (opening the shmem
/// file and reading the DORADMA header for `data_offset`), not via `ptr`.
#[derive(Debug, Clone)]
pub struct MemoryPoolMetadata {
    /// Raw pointer to tensor data in the registering process's address space.
    /// Only valid in the registering process; cross-process consumers must
    /// use `shared_memory_name` instead.
    pub ptr: u64,
    /// Size in bytes.
    pub size: usize,
    /// Data type as string (for example, "int64" or "float32").
    pub dtype: String,
    /// Shape of the tensor.
    pub shape: Vec<usize>,
    /// Whether the memory is pinned and registered with CUDA.
    pub is_pinned: bool,
    /// Shared memory name for cross-process access.
    pub shared_memory_name: Option<String>,
    /// Buffer ID used for lifecycle tracking and cleanup.
    pub buffer_id: Option<String>,
    /// Pool type: "cpu" or "cuda", indicating whether the receiver is a CUDA device.
    pub pinned_type: Option<String>,
    /// Whether an IPC handle was exported by the sender at registration.
    /// When true, the receiver reads from the GPU buffer via IPC, not from /dev/shm.
    pub ipc_present: bool,
}

/// Entry in the memory pool table.
#[derive(Debug, Clone)]
pub struct MemoryPoolEntry {
    /// Metadata about the tensor.
    pub metadata: MemoryPoolMetadata,
    /// Node that registered this memory.
    pub registered_by: String,
}

/// Result summary for daemon shutdown cleanup.
#[derive(Debug, Clone, Copy, Default)]
pub struct CleanupSummary {
    pub unreleased_count: usize,
    pub released_count: usize,
}

/// Manager for memory pool allocations.
#[derive(Clone)]
pub struct MemoryPoolManager {
    /// Table mapping memory pool IDs to their entries.
    memory_pool_table: Arc<Mutex<HashMap<MemoryPoolId, MemoryPoolEntry>>>,
}

impl MemoryPoolManager {
    pub fn new() -> Self {
        Self {
            memory_pool_table: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Lock the pool table, recovering from poison.
    ///
    /// Poisoning should never happen in practice (no panics inside lock
    /// guards), but degrading gracefully is preferable to crashing the
    /// daemon on an edge case.
    fn lock_table(&self) -> std::sync::MutexGuard<'_, HashMap<MemoryPoolId, MemoryPoolEntry>> {
        self.memory_pool_table
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Register a memory pool with the given ID and metadata.
    pub fn register_memory_pool(
        &self,
        id: MemoryPoolId,
        metadata: MemoryPoolMetadata,
        registered_by: String,
    ) -> Result<(), String> {
        let mut table = self.lock_table();

        if table.contains_key(&id) {
            return Err(format!("Memory pool with ID {} already registered", id.id));
        }

        table.insert(
            id,
            MemoryPoolEntry {
                metadata,
                registered_by,
            },
        );

        Ok(())
    }

    /// Get the current number of entries in the memory pool table.
    pub fn table_size(&self) -> usize {
        let table = self.lock_table();
        table.len()
    }

    /// Read memory pool metadata by ID.
    ///
    /// `requested_by` is the node ID of the caller, used for audit logging.
    /// Cross-node reads are allowed (receivers must read senders' pools)
    /// but logged at debug level for diagnostics.
    pub fn read_memory_pool(
        &self,
        id: &MemoryPoolId,
        requested_by: &str,
    ) -> Option<MemoryPoolMetadata> {
        let table = self.lock_table();
        table.get(id).map(|entry| {
            if entry.registered_by != requested_by {
                tracing::debug!(
                    "memory pool {} (registered by {}) read by {}",
                    id.id,
                    entry.registered_by,
                    requested_by,
                );
            }
            entry.metadata.clone()
        })
    }

    /// Free memory pool by ID.
    ///
    /// Any node may free a pool — the normal lifecycle is that the
    /// registering (sender) node creates the pool and a reading
    /// (receiver) node releases it after use.
    ///
    /// Removes the entry from the table and attempts to clean up the
    /// underlying shared memory.
    pub fn free_memory_pool(
        &self,
        id: &MemoryPoolId,
        requested_by: &str,
    ) -> Result<MemoryPoolMetadata, String> {
        let mut table = self.lock_table();

        let entry = table
            .remove(id)
            .ok_or_else(|| "memory pool not found".to_string())?;

        if entry.registered_by != requested_by {
            tracing::debug!(
                "memory pool {} (registered by {}) freed by {}",
                id.id,
                entry.registered_by,
                requested_by,
            );
        }

        if let Some(shm_name) = &entry.metadata.shared_memory_name
            && !shm_name.is_empty()
        {
            self.free_shared_memory(shm_name)?;
        }

        Ok(entry.metadata)
    }

    fn free_shared_memory(&self, shm_name: &str) -> Result<(), String> {
        // Sanity-checks to avoid path traversal: an attacker-supplied
        // shared_memory_name must stay within the expected /dev/shm name space.
        if !shm_name.starts_with("dora_pool_") || shm_name.contains('/') || shm_name.contains("..")
        {
            return Err(format!(
                "shared_memory_name `{}` does not match expected dora_pool_ prefix",
                shm_name,
            ));
        }

        #[cfg(target_os = "linux")]
        {
            let shm_path = format!("/dev/shm/{}", shm_name);
            match std::fs::remove_file(&shm_path) {
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => {
                    tracing::warn!(
                        "Failed to unlink shared memory file {}: {}. The file may still be in use by other processes.",
                        shm_path,
                        e
                    );
                }
            }
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(format!(
                "memory-pool transport is unavailable on this platform; cannot clean up shared memory `{}`",
                shm_name
            ))
        }
    }

    /// Sweep orphaned shared-memory segments from a previous crash or
    /// SIGKILL of the same dataflow.
    ///
    /// `dataflow_id` scopes the sweep — only files matching
    /// `dora_pool_{dataflow_id}_*` are removed.  This is safe even when
    /// other daemons are running on the same host, because dataflow IDs
    /// are UUIDs and no two daemons run the same one concurrently.
    pub fn cleanup_orphans(dataflow_id: &str) {
        #[cfg(target_os = "linux")]
        {
            let prefix = format!("dora_pool_{}_", dataflow_id);
            if let Ok(entries) = std::fs::read_dir("/dev/shm") {
                for entry in entries.flatten() {
                    let name = entry.file_name();
                    let name = name.to_string_lossy();
                    if name.starts_with(&prefix)
                        && let Err(err) = std::fs::remove_file(entry.path())
                        && err.kind() != std::io::ErrorKind::NotFound
                    {
                        tracing::debug!("Orphan sweep: could not unlink {}: {}", name, err);
                    }
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = dataflow_id;
        }
    }

    /// Cleanup all memory pools on shutdown.
    pub fn cleanup_all(&self) -> Result<CleanupSummary, Vec<String>> {
        let mut table = self.lock_table();
        let ids: Vec<MemoryPoolId> = table.keys().cloned().collect();
        let unreleased_count = ids.len();
        let mut errors = Vec::new();

        if unreleased_count > 0 {
            tracing::info!(
                "Detected {} unreleased memory pool, releasing...",
                unreleased_count
            );
        }

        for id in &ids {
            if let Some(entry) = table.remove(id)
                && let Some(shm_name) = &entry.metadata.shared_memory_name
                && !shm_name.is_empty()
                && let Err(err) = self.free_shared_memory(shm_name)
            {
                errors.push(err);
            }
        }

        let released_count = unreleased_count - errors.len();

        if errors.is_empty() {
            if unreleased_count > 0 {
                tracing::info!(
                    "Successfully released {} unreleased memory pools!",
                    released_count
                );
            }
            Ok(CleanupSummary {
                unreleased_count,
                released_count,
            })
        } else {
            tracing::warn!(
                "Released {} of {} unreleased memory pools; {} failed",
                released_count,
                unreleased_count,
                errors.len()
            );
            Err(errors)
        }
    }
}

impl Default for MemoryPoolManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_metadata() -> MemoryPoolMetadata {
        MemoryPoolMetadata {
            ptr: 0,
            size: 1024,
            dtype: "float32".into(),
            shape: vec![256],
            is_pinned: false,
            shared_memory_name: None,
            buffer_id: None,
            pinned_type: None,
            ipc_present: false,
        }
    }

    fn make_id(name: &str) -> MemoryPoolId {
        MemoryPoolId {
            dataflow_id: "test_df".to_string(),
            id: name.to_string(),
        }
    }

    #[test]
    fn register_and_read() {
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        let meta = make_metadata();

        mgr.register_memory_pool(id.clone(), meta.clone(), "node_a".into())
            .unwrap();
        let read = mgr
            .read_memory_pool(&id, "node_a")
            .expect("pool should exist");

        assert_eq!(read.ptr, meta.ptr);
        assert_eq!(read.size, meta.size);
        assert_eq!(read.dtype, meta.dtype);
        assert_eq!(read.shape, meta.shape);
    }

    #[test]
    fn double_register_fails() {
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        let meta = make_metadata();

        mgr.register_memory_pool(id.clone(), meta.clone(), "node_a".into())
            .unwrap();
        let err = mgr
            .register_memory_pool(id, meta, "node_a".into())
            .unwrap_err();

        assert!(err.contains("already registered"));
    }

    #[test]
    fn cross_owner_free_succeeds() {
        // Cross-node free is the normal lifecycle: sender registers,
        // receiver frees. Both operations must succeed.
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        let meta = make_metadata();

        mgr.register_memory_pool(id.clone(), meta, "node_a".into())
            .unwrap();
        // A different node frees the pool — this must succeed.
        mgr.free_memory_pool(&id, "node_b").unwrap();

        // Pool should be gone after successful free.
        assert!(mgr.read_memory_pool(&id, "node_b").is_none());
    }

    #[test]
    fn double_free_second_fails() {
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        let meta = make_metadata();

        mgr.register_memory_pool(id.clone(), meta, "node_a".into())
            .unwrap();
        mgr.free_memory_pool(&id, "node_a").unwrap();

        let err = mgr.free_memory_pool(&id, "node_a").unwrap_err();
        assert!(err.contains("memory pool not found"));
    }

    #[test]
    fn cleanup_all_tracks_counts() {
        let mgr = MemoryPoolManager::new();

        for i in 0..3 {
            mgr.register_memory_pool(
                make_id(&format!("pool-{}", i)),
                make_metadata(),
                "node_a".into(),
            )
            .unwrap();
        }

        let summary = mgr.cleanup_all().unwrap();
        assert_eq!(summary.unreleased_count, 3);
        assert_eq!(summary.released_count, 3);
        assert_eq!(mgr.table_size(), 0);
    }

    #[test]
    fn cleanup_all_reports_partial_release_on_failure() {
        let mgr = MemoryPoolManager::new();

        // One entry frees cleanly (no backing shmem name)...
        mgr.register_memory_pool(make_id("ok"), make_metadata(), "node_a".into())
            .unwrap();
        // ...and one whose shared_memory_name fails the `dora_pool_` validation
        // in `free_shared_memory`, so its release errors.
        let mut bad_meta = make_metadata();
        bad_meta.shared_memory_name = Some("invalid_name".to_string());
        mgr.register_memory_pool(make_id("bad"), bad_meta, "node_a".into())
            .unwrap();

        // cleanup_all must surface the failure (rather than silently claiming
        // "Successfully released") and still drain every entry from the table.
        let errors = mgr.cleanup_all().unwrap_err();
        assert_eq!(errors.len(), 1, "exactly one free should have failed");
        assert_eq!(mgr.table_size(), 0, "all entries must be removed");
    }

    #[test]
    fn table_size_tracks_entries() {
        let mgr = MemoryPoolManager::new();
        assert_eq!(mgr.table_size(), 0);

        let id = make_id("pool-1");
        mgr.register_memory_pool(id.clone(), make_metadata(), "node_a".into())
            .unwrap();
        assert_eq!(mgr.table_size(), 1);

        mgr.free_memory_pool(&id, "node_a").unwrap();
        assert_eq!(mgr.table_size(), 0);
    }

    #[test]
    fn poison_recovery_lock_table() {
        let mgr = MemoryPoolManager::new();
        // lock_table is private but accessible from a child test module.
        // Verify it returns a guard, and that operations work after release.
        {
            let _guard = mgr.lock_table();
            // Guard held; drop it before testing further operations.
        }
        assert_eq!(mgr.table_size(), 0);
    }
}
