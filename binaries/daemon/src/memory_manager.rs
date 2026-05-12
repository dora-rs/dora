use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Identifier for memory pool buffer
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct MemoryPoolId {
    /// Unique identifier for the memory pool buffer
    pub id: String,
}

/// Metadata for memory pool tensor
#[derive(Debug, Clone)]
pub struct MemoryPoolMetadata {
    /// Pointer to the memory pool buffer (in registering process)
    pub ptr: u64,
    /// Size in bytes
    pub size: usize,
    /// Data type as string (e.g., "int64", "float32")
    pub dtype: String,
    /// Shape of the tensor
    pub shape: Vec<usize>,
    /// Whether the memory is pinned (registered with CUDA)
    pub is_pinned: bool,
    /// Shared memory name for cross-process access
    pub shared_memory_name: Option<String>,
    /// Buffer ID for tracking and cleanup
    pub buffer_id: Option<String>,
    /// Process ID that allocated the memory (if known)
    pub allocator_pid: Option<u32>,
    /// Whether CUDA host memory registration was performed
    pub cuda_registered: bool,
}

/// Entry in the memory pool table
#[derive(Debug, Clone)]
pub struct MemoryPoolEntry {
    /// Metadata about the tensor
    pub metadata: MemoryPoolMetadata,
    /// Whether the memory is currently in use
    pub in_use: bool,
    /// Node that registered this memory
    pub registered_by: String,
}

/// Manager for memory pool allocations
#[derive(Clone)]
pub struct MemoryPoolManager {
    /// Table mapping memory pool IDs to their entries
    memory_pool_table: Arc<Mutex<HashMap<MemoryPoolId, MemoryPoolEntry>>>,
}

impl MemoryPoolManager {
    pub fn new() -> Self {
        Self {
            memory_pool_table: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Register a memory pool with the given ID and metadata
    pub fn register_memory_pool(
        &self,
        id: MemoryPoolId,
        metadata: MemoryPoolMetadata,
        registered_by: String,
    ) -> Result<(), String> {
        let mut table = self.memory_pool_table.lock().unwrap();

        if table.contains_key(&id) {
            return Err(format!("Memory pool with ID {} already registered", id.id));
        }

        table.insert(
            id,
            MemoryPoolEntry {
                metadata,
                in_use: true,
                registered_by,
            },
        );

        Ok(())
    }

    /// Get the current number of entries in the memory pool table
    pub fn table_size(&self) -> usize {
        let table = self.memory_pool_table.lock().unwrap();
        table.len()
    }

    /// Read memory pool metadata by ID
    pub fn read_memory_pool(&self, id: &MemoryPoolId) -> Option<MemoryPoolMetadata> {
        let table = self.memory_pool_table.lock().unwrap();
        table.get(id).map(|entry| entry.metadata.clone())
    }

    /// Free memory pool by ID.
    ///
    /// Removes the entry from the table and attempts to clean up the
    /// underlying shared memory (pool-mode shmems are skipped here
    /// and cleaned up at daemon shutdown in `cleanup_all`).
    ///
    /// Returns `Ok(metadata)` on success, `Err` if not found.
    pub fn free_memory_pool(&self, id: &MemoryPoolId) -> Result<MemoryPoolMetadata, String> {
        let mut table = self.memory_pool_table.lock().unwrap();

        let entry = table.remove(id).ok_or_else(|| "memory pool not found".to_string())?;

        // Try to free the shared memory if it exists
        if let Some(shm_name) = &entry.metadata.shared_memory_name {
            if !shm_name.is_empty() {
                let _ = self.free_shared_memory(shm_name);
            }
        }

        Ok(entry.metadata)
    }

    /// Attempt to free shared memory.
    ///
    /// Pool-mode shmems (`dora_pool_*`) are managed by the ring buffer lifecycle
    /// and must NOT be unlinked during normal free — the sender reuses them via
    /// `create()`→`open()` fallback. They are cleaned up at daemon shutdown in
    /// `cleanup_all` via direct filesystem unlink.
    fn free_shared_memory(&self, shm_name: &str) -> Result<(), String> {
        // Skip unlink for pool-mode shmems — persistent ring buffer
        if shm_name.starts_with("dora_pool_") {
            return Ok(());
        }

        // Try to unlink shared memory file on Linux
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
        }

        #[cfg(not(target_os = "linux"))]
        {}

        Ok(())
    }

    /// Get all memory pool entries (for cleanup on shutdown)
    pub fn get_all_entries(&self) -> Vec<(MemoryPoolId, MemoryPoolEntry)> {
        let table = self.memory_pool_table.lock().unwrap();
        table
            .iter()
            .map(|(id, entry)| (id.clone(), entry.clone()))
            .collect()
    }

    /// Cleanup all memory pools on shutdown.
    ///
    /// Counts unfreed entries, logs info about them, frees them, and logs the result.
    /// This is called automatically when the daemon exits.
    pub fn cleanup_all(&self) -> Result<(), Vec<String>> {
        let mut table = self.memory_pool_table.lock().unwrap();
        let ids: Vec<MemoryPoolId> = table.keys().cloned().collect();

        // Count entries that were not freed by the node (all remaining entries)
        let unfreed_count = ids.len();

        if unfreed_count > 0 {
            tracing::info!(
                "memory manager检测到{}条未释放的页锁内存数据，正在释放......",
                unfreed_count
            );
        }

        for id in &ids {
            if let Some(entry) = table.remove(id) {
                if let Some(shm_name) = &entry.metadata.shared_memory_name {
                    if !shm_name.is_empty() {
                        let _ = self.free_shared_memory(shm_name);
                    }
                }
            }
        }

        // Clean up pool-mode shmems (dora_pool_*) that were skipped by
        // free_shared_memory during normal operation. These are persistent
        // buffers and must be unlinked at daemon shutdown.
        #[cfg(target_os = "linux")]
        {
            if let Ok(entries) = std::fs::read_dir("/dev/shm") {
                for entry in entries.flatten() {
                    let file_name = entry.file_name();
                    let name = file_name.to_string_lossy();
                    if name.starts_with("dora_pool_") {
                        let _ = std::fs::remove_file(entry.path());
                    }
                }
            }
        }

        if unfreed_count > 0 {
            tracing::info!(
                "memory manager已成功释放{}条未释放的页锁内存数据",
                unfreed_count
            );
        }

        Ok(())
    }
}
