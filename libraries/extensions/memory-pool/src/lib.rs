use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Identifier for a memory pool buffer.
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct MemoryPoolId {
    /// Unique identifier for the memory pool buffer.
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
    /// Process ID that allocated the memory, when known.
    pub allocator_pid: Option<u32>,
    /// Whether CUDA host memory registration was performed.
    pub cuda_registered: bool,
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
    /// Only the node that registered the pool may free it. Cross-node
    /// free attempts are rejected with an error to prevent accidental
    /// disruption of another node's data path.
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
            // Re-insert before returning error — don't orphan the entry
            let msg = format!(
                "memory pool {} is owned by {}, not {}",
                id.id, entry.registered_by, requested_by,
            );
            table.insert(id.clone(), entry);
            return Err(msg);
        }

        if let Some(shm_name) = &entry.metadata.shared_memory_name {
            if !shm_name.is_empty() {
                self.free_shared_memory(shm_name)?;
            }
        }

        Ok(entry.metadata)
    }

    fn free_shared_memory(&self, shm_name: &str) -> Result<(), String> {
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

    /// Get all memory pool entries for cleanup on shutdown.
    pub fn get_all_entries(&self) -> Vec<(MemoryPoolId, MemoryPoolEntry)> {
        let table = self.lock_table();
        table
            .iter()
            .map(|(id, entry)| (id.clone(), entry.clone()))
            .collect()
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
            if let Some(entry) = table.remove(id) {
                if let Some(shm_name) = &entry.metadata.shared_memory_name {
                    if !shm_name.is_empty() {
                        if let Err(err) = self.free_shared_memory(shm_name) {
                            errors.push(err);
                        }
                    }
                }
            }
        }

        if unreleased_count > 0 {
            tracing::info!(
                "Successfully released {} unreleased memory pools!",
                unreleased_count
            );
        }

        if errors.is_empty() {
            Ok(CleanupSummary {
                unreleased_count,
                released_count: unreleased_count,
            })
        } else {
            Err(errors)
        }
    }
}

impl Default for MemoryPoolManager {
    fn default() -> Self {
        Self::new()
    }
}
