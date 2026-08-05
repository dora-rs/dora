use std::collections::{HashMap, HashSet};
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
#[derive(Debug, Clone, Default)]
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
    /// Whether an IPC handle was successfully exported to the shmem header
    /// (byte 24).  True (1) means the pool's GPU buffer is accessible via
    /// cudaIpcOpenMemHandle; the shmem data region may be header-only in this case.
    pub ipc_present: Option<bool>,
    /// Pool type: "cpu" or "cuda", indicating whether the receiver is a CUDA device.
    pub pinned_type: Option<String>,
}

/// Entry in the memory pool table.
#[derive(Debug, Clone)]
pub struct MemoryPoolEntry {
    /// Metadata about the tensor.
    pub metadata: MemoryPoolMetadata,
    /// Node that registered this memory.
    pub registered_by: String,
    /// All nodes that have accessed this pool (registered or read).
    /// Used to send targeted cleanup notifications on free.
    pub touched_by: HashSet<String>,
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
    /// Cross-machine pools this daemon participates in:
    /// pool id -> (peer machine id, dataflow id).
    ///
    /// Unlike the main table these entries describe *mirrors* (pools that
    /// live on another machine's /dev/shm), so they never carry a
    /// `MemoryPoolEntry` and are tracked separately. Written on register
    /// ack (origin side) and on mirror creation (mirror side); read on
    /// every write (is_cross / forward gate) and on free (peer routing);
    /// drained by `cleanup_all` on daemon exit.
    cross_pools: Arc<Mutex<HashMap<String, (String, String)>>>,
}

impl MemoryPoolManager {
    pub fn new() -> Self {
        Self {
            memory_pool_table: Arc::new(Mutex::new(HashMap::new())),
            cross_pools: Arc::new(Mutex::new(HashMap::new())),
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

        let mut touched = HashSet::new();
        touched.insert(registered_by.clone());
        table.insert(
            id,
            MemoryPoolEntry {
                metadata,
                registered_by,
                touched_by: touched,
            },
        );

        Ok(())
    }

    /// Get the current number of entries in the memory pool table.
    pub fn table_size(&self) -> usize {
        let table = self.lock_table();
        table.len()
    }

    fn lock_cross_pools(&self) -> std::sync::MutexGuard<'_, HashMap<String, (String, String)>> {
        self.cross_pools
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Record a cross-machine pool: `pool_id` mirrors to/from `peer_machine`.
    ///
    /// Called on both sides: the origin records `{pool -> target}` when the
    /// register ack arrives, the mirror records `{pool -> origin}` after
    /// creating the mirror segment.
    pub fn register_cross_pool(&self, pool_id: String, peer_machine: String, dataflow_id: String) {
        self.lock_cross_pools()
            .insert(pool_id, (peer_machine, dataflow_id));
    }

    /// Forget a cross-machine pool (called on free).
    pub fn unregister_cross_pool(&self, pool_id: &str) -> Option<(String, String)> {
        self.lock_cross_pools().remove(pool_id)
    }

    /// The pool's peer machine (the machine it mirrors to/from), if any.
    pub fn cross_peer(&self, pool_id: &str) -> Option<String> {
        self.lock_cross_pools()
            .get(pool_id)
            .map(|(peer, _)| peer.clone())
    }

    /// Whether `pool_id` is a cross-machine pool this daemon participates in.
    pub fn is_cross(&self, pool_id: &str) -> bool {
        self.lock_cross_pools().contains_key(pool_id)
    }

    /// Machine-qualified OS id of a cross-machine pool mirror:
    /// `dora_pool_{machine_id}_{dataflow_id}_{node_id}_{counter}`, derived
    /// from a `pool_{node_id}_{counter}` shaped `shared_memory_id`. Returns
    /// `None` when the id does not match that shape. Single source of truth
    /// for the qualified name — create, write, and the free paths must agree
    /// or a mirror leaks under a name nobody unlinks.
    pub fn cross_pool_shmem_name(
        machine_id: &str,
        dataflow_id: &str,
        shared_memory_id: &str,
    ) -> Option<String> {
        let (node_id, counter) = shared_memory_id
            .strip_prefix("pool_")
            .and_then(|s| s.rsplit_once('_'))?;
        Some(format!(
            "dora_pool_{machine_id}_{dataflow_id}_{node_id}_{counter}"
        ))
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
        let mut table = self.lock_table();
        table.get_mut(id).map(|entry| {
            if entry.registered_by != requested_by {
                tracing::debug!(
                    "memory pool {} (registered by {}) read by {}",
                    id.id,
                    entry.registered_by,
                    requested_by,
                );
            }
            entry.touched_by.insert(requested_by.to_string());
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
    ) -> Result<(MemoryPoolMetadata, HashSet<String>), String> {
        // Only the table removal needs the lock. Releasing it before the
        // shared-memory unlink below keeps the `remove_file` syscall out of the
        // critical section, so concurrent `register`/`read`/`free` calls on
        // other pools don't serialize behind this pool's filesystem work.
        let entry = {
            let mut table = self.lock_table();
            table
                .remove(id)
                .ok_or_else(|| "memory pool not found".to_string())?
        };

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

        Ok((entry.metadata, entry.touched_by))
    }

    fn free_shared_memory(&self, shm_name: &str) -> Result<(), String> {
        // Sanity-checks to avoid path traversal: an attacker-supplied
        // shared_memory_name must stay within the expected /dev/shm name
        // space. No `dora_pool_` prefix requirement — explicit names via
        // `register_memory_pool(name=...)` may be arbitrary (checked at
        // registration), only '/' and '..' are rejected here.
        if shm_name.contains('/') || shm_name.contains("..") {
            return Err(format!(
                "shared_memory_name `{shm_name}` is invalid: must not contain '/' or '..'",
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
    /// `dataflow_id` scopes the sweep.  Segments appear under two naming
    /// formats:
    ///
    /// - local pool:            `dora_pool_{dataflow_id}_{node_id}_{counter}`
    /// - cross-machine mirror:  `dora_pool_{machine_id}_{dataflow_id}_{node_id}_{counter}`
    ///
    /// Both are removed (mirrors are machine-qualified, so the daemon cannot
    /// know the machine prefix in advance).  This is safe even when other
    /// daemons are running on the same host, because dataflow IDs are UUIDs
    /// and no two daemons run the same one concurrently; matching the
    /// dataflow id only as an underscore-delimited segment (not a bare
    /// substring) means a foreign dataflow's segments or unrelated /dev/shm
    /// files can never be swept.
    pub fn cleanup_orphans(dataflow_id: &str) {
        #[cfg(target_os = "linux")]
        {
            let unqualified_prefix = format!("dora_pool_{}_", dataflow_id);
            let qualified_segment = format!("_{}_", dataflow_id);
            match std::fs::read_dir("/dev/shm") {
                Ok(entries) => {
                    for entry in entries.flatten() {
                        let name = entry.file_name();
                        let name = name.to_string_lossy();
                        let is_this_dataflow = name.starts_with("dora_pool_")
                            && (name.starts_with(&unqualified_prefix)
                                || name.contains(&qualified_segment));
                        if is_this_dataflow
                            && let Err(err) = std::fs::remove_file(entry.path())
                            && err.kind() != std::io::ErrorKind::NotFound
                        {
                            tracing::debug!(
                                "Orphan sweep: could not unlink {} at {}: {}",
                                name,
                                entry.path().display(),
                                err
                            );
                        }
                    }
                }
                Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
                Err(err) => {
                    tracing::debug!("Orphan sweep: failed to read /dev/shm directory: {}", err);
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = dataflow_id;
        }
    }

    /// Cleanup all memory pools on shutdown.
    ///
    /// `machine_id` is this daemon's own machine id: cross-machine mirror
    /// segments are derived from it, so each daemon only ever unlinks
    /// segments that live on its own machine (entries pointing at another
    /// machine resolve to a name that does not exist locally — a no-op).
    pub fn cleanup_all(&self, machine_id: &str) -> Result<CleanupSummary, Vec<String>> {
        let mut table = self.lock_table();
        // Drain the table in one move instead of cloning every key into a
        // `Vec` only to look each one back up and remove it. The guard is held
        // for the rest of the function (matching the previous locking window);
        // `free_shared_memory` does not re-enter the table.
        let drained = std::mem::take(&mut *table);
        let unreleased_count = drained.len();
        let mut errors = Vec::new();

        if unreleased_count > 0 {
            tracing::info!(
                "Detected {} unreleased memory pool, releasing...",
                unreleased_count
            );
        }

        for (_id, entry) in drained {
            if let Some(shm_name) = &entry.metadata.shared_memory_name
                && !shm_name.is_empty()
                && let Err(err) = self.free_shared_memory(shm_name)
            {
                errors.push(err);
            }
        }

        // Cross-machine mirrors: drain the cross table and unlink every
        // segment whose name resolves on this machine. Linux keeps pools in
        // /dev/shm; the mirror handle is dropped owner-less
        // (`set_owner(false)`), so the name is only removable by file unlink.
        // Non-Linux platforms have no such segments — skip (the drain above
        // still clears the table, and cross_peer callers see the freed state).
        #[cfg(target_os = "linux")]
        {
            let cross = std::mem::take(&mut *self.lock_cross_pools());
            if !cross.is_empty() {
                tracing::info!(
                    "Releasing {} cross-machine mirror segment(s) on shutdown",
                    cross.len()
                );
            }
            for (pool_id, (_peer, dataflow_id)) in &cross {
                let Some(shm_name) = Self::cross_pool_shmem_name(machine_id, dataflow_id, pool_id)
                else {
                    continue;
                };
                // Absent locally (the mirror lives on another machine's
                // /dev/shm) is the normal case for origin-side entries — not
                // an error.
                match std::fs::remove_file(format!("/dev/shm/{shm_name}")) {
                    Ok(()) => tracing::debug!("released cross-machine mirror {shm_name}"),
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                    Err(e) => {
                        tracing::warn!("failed to remove mirror {shm_name} on shutdown: {e}");
                    }
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            self.lock_cross_pools().clear();
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
            ipc_present: None,
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
    fn concurrent_frees_across_pools_all_succeed() {
        // `free_memory_pool` only holds the table lock for the removal, not
        // across the shared-memory unlink. Frees on distinct pools must proceed
        // independently and leave the table empty. Metadata carries no backing
        // segment (`shared_memory_name: None`) so the test stays cross-platform.
        let mgr = MemoryPoolManager::new();
        let n = 16;
        for i in 0..n {
            mgr.register_memory_pool(
                make_id(&format!("pool-{i}")),
                make_metadata(),
                "node_a".into(),
            )
            .unwrap();
        }

        let handles: Vec<_> = (0..n)
            .map(|i| {
                let mgr = mgr.clone();
                std::thread::spawn(move || {
                    mgr.free_memory_pool(&make_id(&format!("pool-{i}")), "node_b")
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap().expect("free should succeed");
        }
        assert_eq!(mgr.table_size(), 0);
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

        let summary = mgr.cleanup_all("").unwrap();
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
        // ...and one whose shared_memory_name fails the path-traversal
        // validation in `free_shared_memory`, so its release errors.
        let mut bad_meta = make_metadata();
        bad_meta.shared_memory_name = Some("invalid/name".to_string());
        mgr.register_memory_pool(make_id("bad"), bad_meta, "node_a".into())
            .unwrap();

        // cleanup_all must surface the failure (rather than silently claiming
        // "Successfully released") and still drain every entry from the table.
        let errors = mgr.cleanup_all("").unwrap_err();
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

    #[test]
    fn cleanup_orphans_runs_without_panic() {
        // Sweep should run cleanly without panicking regardless of platform.
        MemoryPoolManager::cleanup_orphans("test-dataflow-uuid");
    }

    /// Regression test: the orphan sweep must remove both the unqualified
    /// local segment (`dora_pool_{df}_*`) and the machine-qualified
    /// cross-machine mirror (`dora_pool_{machine}_{df}_*`), while never
    /// touching another dataflow's segments.
    #[test]
    #[cfg(target_os = "linux")]
    fn cleanup_orphans_removes_local_and_machine_qualified_segments() {
        use std::fs;
        use std::path::PathBuf;

        let dataflow_id = "cleanup-orphans-test-df";
        let segments = [
            // (name, expected to be swept)
            (
                format!("dora_pool_{dataflow_id}_node_0"), // local pool
                true,
            ),
            (
                format!("dora_pool_machine-1_{dataflow_id}_node_1"), // mirror
                true,
            ),
            (format!("dora_pool_other-df_node_0"), false), // foreign
        ];

        let mut created: Vec<PathBuf> = Vec::new();
        for (name, _) in &segments {
            let path = PathBuf::from("/dev/shm").join(name);
            if fs::write(&path, b"x").is_ok() {
                created.push(path);
            }
        }
        // Best-effort cleanup even if an assertion fails below.
        struct RemoveOnDrop(Vec<PathBuf>);
        impl Drop for RemoveOnDrop {
            fn drop(&mut self) {
                for path in &self.0 {
                    let _ = fs::remove_file(path);
                }
            }
        }
        let _guard = RemoveOnDrop(created.clone());

        MemoryPoolManager::cleanup_orphans(dataflow_id);

        for (i, (_name, expected_swept)) in segments.iter().enumerate() {
            assert_eq!(
                !created[i].exists(),
                *expected_swept,
                "segment {i} sweep mismatch (name: {})",
                segments[i].0,
            );
        }
    }
}

#[cfg(test)]
mod cross_pool_tests {
    use super::*;

    /// `cleanup_all` must unlink mirror segments that resolve on this
    /// machine (via this daemon's own machine id) and leave segments
    /// belonging to other machines untouched.
    #[test]
    #[cfg(target_os = "linux")]
    fn cleanup_all_removes_only_own_machine_mirrors() {
        let mgr = MemoryPoolManager::new();
        let df = "11111111-2222-3333-4444-555555555555";

        // a mirror segment that lives on this machine ("B")
        let own = MemoryPoolManager::cross_pool_shmem_name("B", df, "pool_node_0").unwrap();
        std::fs::write(format!("/dev/shm/{own}"), vec![0u8; 1024]).unwrap();
        mgr.register_cross_pool("pool_node_0".into(), "A".into(), df.into());

        // a mirror segment that lives on another machine ("C") — the
        // origin-side entry for it must NOT cause a local unlink
        let foreign = MemoryPoolManager::cross_pool_shmem_name("C", df, "pool_node_1").unwrap();
        std::fs::write(format!("/dev/shm/{foreign}"), vec![0u8; 1024]).unwrap();
        mgr.register_cross_pool("pool_node_1".into(), "C".into(), df.into());

        let _ = mgr.cleanup_all("B");

        assert!(
            !std::path::Path::new(&format!("/dev/shm/{own}")).exists(),
            "own-machine mirror should be unlinked"
        );
        assert!(
            std::path::Path::new(&format!("/dev/shm/{foreign}")).exists(),
            "foreign-machine mirror must survive"
        );

        // test hygiene
        let _ = std::fs::remove_file(format!("/dev/shm/{foreign}"));
    }

    /// Register / query / unregister lifecycle of the cross table.
    #[test]
    fn cross_pool_lifecycle() {
        let mgr = MemoryPoolManager::new();
        assert!(!mgr.is_cross("pool_node_0"));
        assert_eq!(mgr.cross_peer("pool_node_0"), None);

        mgr.register_cross_pool("pool_node_0".into(), "B".into(), "df".into());
        assert!(mgr.is_cross("pool_node_0"));
        assert_eq!(mgr.cross_peer("pool_node_0").as_deref(), Some("B"));

        let removed = mgr.unregister_cross_pool("pool_node_0");
        assert_eq!(removed.as_ref().map(|(peer, _)| peer.as_str()), Some("B"));
        assert!(!mgr.is_cross("pool_node_0"));
    }
}
