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
    /// Nodes that have not opened this pool (yet) but could still learn its
    /// id from a message in flight — the registering node's downstream
    /// consumers as of registration time. Together with `touched_by` this is
    /// the set of nodes that may still reference the pool; see
    /// [`MemoryPoolManager::reclaim_unreachable`].
    pub potential_readers: HashSet<String>,
}

impl MemoryPoolEntry {
    /// Whether any node that could still reference this pool is alive.
    fn reachable(&self, is_live: &impl Fn(&str) -> bool) -> bool {
        self.touched_by
            .iter()
            .chain(&self.potential_readers)
            .any(|node| is_live(node))
    }
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
    ///
    /// `potential_readers` lists the nodes that may still ask for this pool
    /// without having opened it yet (the registrar's downstream consumers).
    /// It keeps the pool alive until none of them can use it any more — see
    /// [`Self::reclaim_unreachable`].
    pub fn register_memory_pool(
        &self,
        id: MemoryPoolId,
        metadata: MemoryPoolMetadata,
        registered_by: String,
        potential_readers: HashSet<String>,
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
                potential_readers,
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

        self.release_segment(&entry.metadata)?;

        Ok((entry.metadata, entry.touched_by))
    }

    /// Unlink an entry's backing segment, if it has one.
    fn release_segment(&self, metadata: &MemoryPoolMetadata) -> Result<(), String> {
        match &metadata.shared_memory_name {
            Some(name) if !name.is_empty() => self.free_shared_memory(name),
            _ => Ok(()),
        }
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

    /// Release every pool of `dataflow_id` that no live node can reach any
    /// more, and return the released ids.
    ///
    /// A pool deliberately outlives the node that registered it: the normal
    /// lifecycle is that a sender registers a pool and a *receiver* reads
    /// and frees it, possibly well after the sender exited. Dropping a
    /// node's pools the moment it goes away would therefore break in-flight
    /// transfers. Instead every entry records who can still reach it — the
    /// nodes that already opened it (`touched_by`) plus the registrar's
    /// downstream consumers (`potential_readers`), which can still learn the
    /// pool id from a message that is already on its way. The pool is
    /// released once none of those nodes is running any more, which for a
    /// removed or crashed node happens as soon as the last of its consumers
    /// is gone instead of only at dataflow teardown (dora-rs/dora#2881).
    ///
    /// Because a released pool has no live node in `touched_by`, there is
    /// nobody left to send a `FreeMemoryPool` notification to — unlike
    /// `free_memory_pool`, which must tell the other holders to drop their
    /// per-process buffers.
    ///
    /// `is_live` reports whether a node is still running **on this daemon**.
    /// Nodes on other daemons never qualify, and must not: pools are
    /// host-local shared memory and every daemon keeps its own table, so a
    /// remote consumer can never open this one's segments. It is called
    /// with the pool table locked, so it must stay cheap and must not
    /// re-enter the manager.
    pub fn reclaim_unreachable(
        &self,
        dataflow_id: &str,
        is_live: impl Fn(&str) -> bool,
    ) -> Vec<MemoryPoolId> {
        self.release_matching(dataflow_id, |entry| !entry.reachable(&is_live))
    }

    /// Release every pool of a finished dataflow, and return the released
    /// ids.
    ///
    /// Unlike [`Self::reclaim_unreachable`] this needs no liveness check:
    /// the dataflow is over, so none of its nodes can ask for a pool any
    /// more. Without it a daemon that outlives the dataflow (`dora up`)
    /// would hold every unfreed pool until it exits (dora-rs/dora#2881).
    pub fn cleanup_dataflow(&self, dataflow_id: &str) -> Vec<MemoryPoolId> {
        let released = self.release_matching(dataflow_id, |_| true);
        if !released.is_empty() {
            tracing::info!(
                "Detected {} unreleased memory pool of finished dataflow {dataflow_id}, releasing...",
                released.len(),
            );
        }
        released
    }

    /// Drop the entries of `dataflow_id` that `should_release` accepts and
    /// unlink their segments, returning the released ids.
    ///
    /// The table lock is released before the unlinks, matching
    /// `free_memory_pool`: the `remove_file` syscalls must not serialize
    /// unrelated pool operations.
    fn release_matching(
        &self,
        dataflow_id: &str,
        mut should_release: impl FnMut(&MemoryPoolEntry) -> bool,
    ) -> Vec<MemoryPoolId> {
        let released: Vec<(MemoryPoolId, MemoryPoolEntry)> = {
            let mut table = self.lock_table();
            table
                .extract_if(|id, entry| id.dataflow_id == dataflow_id && should_release(entry))
                .collect()
        };

        released
            .into_iter()
            .map(|(id, entry)| {
                if let Err(err) = self.release_segment(&entry.metadata) {
                    tracing::warn!("failed to release memory pool {}: {err}", id.id);
                }
                id
            })
            .collect()
    }

    /// Sweep orphaned shared-memory segments of `dataflow_id` that belong to
    /// one of *this daemon's* nodes.
    ///
    /// Catches what the pool table cannot: a segment whose creator died
    /// between `shm_open` and `register_memory_pool` has no entry to release.
    ///
    /// Ownership matters because `/dev/shm` is host-wide while the segment
    /// name carries only the *dataflow* id: sweeping by dataflow id alone
    /// reaches every daemon on the host. Two daemons serving one dataflow on
    /// one machine (a `machine:` split) is a supported deployment, and there
    /// the daemon that finishes first would unlink the other's *live*
    /// segments, leaving a consumer over there with `ENOENT`. Both call sites
    /// are per-daemon: the spawn-time sweep races the peer daemon's spawn,
    /// the finish-time one runs while the peer may still be going. A node id
    /// belongs to at most one daemon, so `is_local_node` keeps the sweep
    /// host-safe without narrowing what it can reclaim.
    ///
    /// Segments are named `dora_pool_{dataflow_id}_{node_id}_{counter}`. Node
    /// ids may contain `_`, the counter never does, so the last component
    /// splits off the node id; a name that does not fit the pattern belongs
    /// to nobody identifiable and is left alone.
    pub fn cleanup_orphans(dataflow_id: &str, is_local_node: impl Fn(&str) -> bool) {
        #[cfg(target_os = "linux")]
        {
            let prefix = format!("dora_pool_{}_", dataflow_id);
            match std::fs::read_dir("/dev/shm") {
                Ok(entries) => {
                    for entry in entries.flatten() {
                        let name = entry.file_name();
                        let name = name.to_string_lossy();
                        let Some((node_id, counter)) = name
                            .strip_prefix(&prefix)
                            .and_then(|rest| rest.rsplit_once('_'))
                        else {
                            continue;
                        };
                        if counter.parse::<u64>().is_err() || !is_local_node(node_id) {
                            continue;
                        }
                        if let Err(err) = std::fs::remove_file(entry.path())
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
            let _ = (dataflow_id, is_local_node);
        }
    }

    /// Cleanup all memory pools on shutdown.
    pub fn cleanup_all(&self) -> Result<CleanupSummary, Vec<String>> {
        // Drain the table in one move instead of cloning every key into a
        // `Vec` only to look each one back up and remove it, and release the
        // guard before the unlinks below (see `release_matching`).
        let drained = std::mem::take(&mut *self.lock_table());
        let unreleased_count = drained.len();
        let mut errors = Vec::new();

        if unreleased_count > 0 {
            tracing::info!(
                "Detected {} unreleased memory pool, releasing...",
                unreleased_count
            );
        }

        for (_id, entry) in drained {
            if let Err(err) = self.release_segment(&entry.metadata) {
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
            ipc_present: None,
        }
    }

    fn make_id(name: &str) -> MemoryPoolId {
        MemoryPoolId {
            dataflow_id: "test_df".to_string(),
            id: name.to_string(),
        }
    }

    /// The registrar's downstream consumers at registration time.
    fn readers(nodes: &[&str]) -> HashSet<String> {
        nodes.iter().map(|n| n.to_string()).collect()
    }

    #[test]
    fn register_and_read() {
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        let meta = make_metadata();

        mgr.register_memory_pool(id.clone(), meta.clone(), "node_a".into(), readers(&[]))
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

        mgr.register_memory_pool(id.clone(), meta.clone(), "node_a".into(), readers(&[]))
            .unwrap();
        let err = mgr
            .register_memory_pool(id, meta, "node_a".into(), readers(&[]))
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

        mgr.register_memory_pool(id.clone(), meta, "node_a".into(), readers(&[]))
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

        mgr.register_memory_pool(id.clone(), meta, "node_a".into(), readers(&[]))
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
                readers(&[]),
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
                readers(&[]),
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
        mgr.register_memory_pool(
            make_id("ok"),
            make_metadata(),
            "node_a".into(),
            readers(&[]),
        )
        .unwrap();
        // ...and one whose shared_memory_name fails the `dora_pool_` validation
        // in `free_shared_memory`, so its release errors.
        let mut bad_meta = make_metadata();
        bad_meta.shared_memory_name = Some("invalid_name".to_string());
        mgr.register_memory_pool(make_id("bad"), bad_meta, "node_a".into(), readers(&[]))
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
        mgr.register_memory_pool(id.clone(), make_metadata(), "node_a".into(), readers(&[]))
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
        MemoryPoolManager::cleanup_orphans("test-dataflow-uuid", |_| true);
    }

    /// Two daemons on one host serve one dataflow, so both see the other's
    /// segments under the same `dora_pool_{dataflow_id}_` prefix. The sweep
    /// must unlink only its own daemon's nodes, or a finishing daemon
    /// destroys a live segment the peer daemon's consumer has not mapped
    /// yet (dora-rs/dora#2881 review).
    #[cfg(target_os = "linux")]
    #[test]
    fn cleanup_orphans_leaves_another_daemons_segments_alone() {
        // Unique per run, so concurrent tests cannot collide in /dev/shm.
        let dataflow_id = format!("test-df-{}-scoped", std::process::id());
        let segment = |name: &str| std::path::PathBuf::from(format!("/dev/shm/{name}"));
        // (segment name, must survive the sweep)
        let files = [
            (format!("dora_pool_{dataflow_id}_local_0"), false),
            // A node id may itself contain '_', so only the last component
            // is the counter: this is node `local_with_underscore`, not
            // `local_with` or `local`.
            (
                format!("dora_pool_{dataflow_id}_local_with_underscore_7"),
                false,
            ),
            (format!("dora_pool_{dataflow_id}_remote_0"), true),
            // Not `{node}_{counter}` — attributable to nobody, so kept.
            (format!("dora_pool_{dataflow_id}_local_notacounter"), true),
            // Another dataflow entirely, even for a local node id.
            (format!("dora_pool_{dataflow_id}other_local_0"), true),
        ];
        for (file, _) in &files {
            std::fs::write(segment(file), b"x").unwrap();
        }

        MemoryPoolManager::cleanup_orphans(&dataflow_id, |node| {
            matches!(node, "local" | "local_with_underscore")
        });

        let outcome: Vec<(&str, bool)> = files
            .iter()
            .map(|(file, _)| (file.as_str(), segment(file).exists()))
            .collect();
        for (file, _) in &files {
            let _ = std::fs::remove_file(segment(file));
        }
        for ((file, expected_kept), (_, kept)) in files.iter().zip(&outcome) {
            assert_eq!(kept, expected_kept, "{file}");
        }
    }

    // -- dora#2881: pools must be reclaimed once nothing can reach them --

    #[test]
    fn registrar_exit_keeps_a_pool_a_live_reader_can_still_reach() {
        // The normal lifecycle: the sender registers a pool, sends its id
        // downstream and exits. The receiver has not opened the pool yet,
        // so `touched_by` is still just the sender — freeing here would
        // break the in-flight transfer.
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        mgr.register_memory_pool(
            id.clone(),
            make_metadata(),
            "sender".into(),
            readers(&["recv"]),
        )
        .unwrap();

        let released = mgr.reclaim_unreachable("test_df", |node| node == "recv");

        assert!(
            released.is_empty(),
            "pool must survive: `recv` can still read it"
        );
        assert!(mgr.read_memory_pool(&id, "recv").is_some());
    }

    #[test]
    fn pool_is_released_once_no_live_node_can_reach_it() {
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        mgr.register_memory_pool(
            id.clone(),
            make_metadata(),
            "sender".into(),
            readers(&["recv"]),
        )
        .unwrap();

        // Both the sender and its only downstream consumer are gone.
        let released = mgr.reclaim_unreachable("test_df", |_| false);

        assert_eq!(released, vec![id.clone()]);
        assert_eq!(mgr.table_size(), 0);
    }

    #[test]
    fn a_live_reader_that_never_was_a_potential_reader_keeps_the_pool() {
        // `potential_readers` is a snapshot of the topology at registration
        // time; a node that actually opened the pool must keep it alive on
        // its own, even if it is not in that snapshot.
        let mgr = MemoryPoolManager::new();
        let id = make_id("pool-1");
        mgr.register_memory_pool(id.clone(), make_metadata(), "sender".into(), readers(&[]))
            .unwrap();
        mgr.read_memory_pool(&id, "late_reader").unwrap();

        let released = mgr.reclaim_unreachable("test_df", |node| node == "late_reader");

        assert!(released.is_empty());
        assert_eq!(mgr.table_size(), 1);
    }

    #[test]
    fn reclaim_is_scoped_to_one_dataflow() {
        let mgr = MemoryPoolManager::new();
        let mine = make_id("pool-1");
        let other = MemoryPoolId {
            dataflow_id: "other_df".to_string(),
            id: "pool-1".to_string(),
        };
        mgr.register_memory_pool(mine.clone(), make_metadata(), "sender".into(), readers(&[]))
            .unwrap();
        mgr.register_memory_pool(
            other.clone(),
            make_metadata(),
            "sender".into(),
            readers(&[]),
        )
        .unwrap();

        let released = mgr.reclaim_unreachable("test_df", |_| false);

        assert_eq!(released, vec![mine]);
        assert!(
            mgr.read_memory_pool(&other, "sender").is_some(),
            "a same-named pool of another dataflow must not be touched"
        );
    }

    #[test]
    fn cleanup_dataflow_releases_every_pool_of_that_dataflow() {
        let mgr = MemoryPoolManager::new();
        for i in 0..3 {
            mgr.register_memory_pool(
                make_id(&format!("pool-{i}")),
                make_metadata(),
                "sender".into(),
                readers(&["recv"]),
            )
            .unwrap();
        }
        let other = MemoryPoolId {
            dataflow_id: "other_df".to_string(),
            id: "pool-1".to_string(),
        };
        mgr.register_memory_pool(
            other.clone(),
            make_metadata(),
            "sender".into(),
            readers(&[]),
        )
        .unwrap();

        // A finished dataflow has no nodes left, so liveness plays no role.
        let released = mgr.cleanup_dataflow("test_df");

        assert_eq!(released.len(), 3);
        assert_eq!(mgr.table_size(), 1, "the other dataflow's pool must remain");
    }
}
