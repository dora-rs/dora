use std::collections::HashMap;
use std::sync::{Arc, Mutex};

/// Identifier for pinned memory buffer
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct PinnedMemoryId {
    /// Unique identifier for the pinned memory buffer
    pub id: String,
}

/// Metadata for pinned memory tensor
#[derive(Debug, Clone)]
pub struct PinnedMemoryMetadata {
    /// Pointer to the pinned memory buffer (in registering process)
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

/// Entry in the pinned memory table
#[derive(Debug, Clone)]
pub struct PinnedMemoryEntry {
    /// Metadata about the tensor
    pub metadata: PinnedMemoryMetadata,
    /// Whether the memory is currently in use
    pub in_use: bool,
    /// Node that registered this memory
    pub registered_by: String,
    /// Whether this entry has been freed (kept for double-free detection)
    pub freed: bool,
}

/// Manager for pinned memory allocations
#[derive(Clone)]
pub struct MemoryManager {
    /// Table mapping pinned memory IDs to their entries
    pinned_memory_table: Arc<Mutex<HashMap<PinnedMemoryId, PinnedMemoryEntry>>>,
}

impl MemoryManager {
    /// Create a new MemoryManager
    pub fn new() -> Self {
        Self {
            pinned_memory_table: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Register pinned memory with the given ID and metadata
    pub fn register_pinned_memory(
        &self,
        id: PinnedMemoryId,
        metadata: PinnedMemoryMetadata,
        registered_by: String,
    ) -> Result<(), String> {
        let mut table = self.pinned_memory_table.lock().unwrap();

        if table.contains_key(&id) {
            return Err(format!(
                "Pinned memory with ID {} already registered",
                id.id
            ));
        }

        table.insert(
            id,
            PinnedMemoryEntry {
                metadata,
                in_use: true,
                registered_by,
                freed: false,
            },
        );

        Ok(())
    }

    /// Read pinned memory metadata by ID
    pub fn read_pinned_memory(&self, id: &PinnedMemoryId) -> Option<PinnedMemoryMetadata> {
        let table = self.pinned_memory_table.lock().unwrap();
        table.get(id).map(|entry| entry.metadata.clone())
    }

    /// Free pinned memory by ID.
    ///
    /// The entry is kept in the table (marked as freed) rather than removed,
    /// so that subsequent operations can detect double-free and retrieve
    /// metadata (e.g. size) for proper warning messages.
    ///
    /// Returns `Ok(metadata)` on first free, `Err("already freed,size=N")` on double-free.
    pub fn free_pinned_memory(&self, id: &PinnedMemoryId) -> Result<PinnedMemoryMetadata, String> {
        let mut table = self.pinned_memory_table.lock().unwrap();

        let entry = match table.get_mut(id) {
            Some(entry) if !entry.freed => {
                // First free: mark as freed
                entry.freed = true;
                entry.clone()
            }
            Some(entry) => {
                // Double-free: return error with size from stored metadata
                let size = entry.metadata.size;
                return Err(format!("already freed,size={}", size));
            }
            None => {
                return Err("already freed,size=0".to_string());
            }
        };

        // Try to free the shared memory if it exists
        if let Some(shm_name) = &entry.metadata.shared_memory_name {
            if !shm_name.is_empty() {
                let _ = self.free_shared_memory(shm_name, &entry.metadata);
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
    fn free_shared_memory(
        &self,
        shm_name: &str,
        _metadata: &PinnedMemoryMetadata,
    ) -> Result<(), String> {
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

    /// Get all pinned memory entries (for cleanup on shutdown)
    pub fn get_all_entries(&self) -> Vec<(PinnedMemoryId, PinnedMemoryEntry)> {
        let table = self.pinned_memory_table.lock().unwrap();
        table
            .iter()
            .map(|(id, entry)| (id.clone(), entry.clone()))
            .collect()
    }

    /// Cleanup all pinned memory on shutdown.
    ///
    /// Counts unfreed entries, logs info about them, frees them, and logs the result.
    /// This is called automatically when the daemon exits.
    pub fn cleanup_all(&self) -> Result<(), Vec<String>> {
        let mut table = self.pinned_memory_table.lock().unwrap();
        let ids: Vec<PinnedMemoryId> = table.keys().cloned().collect();

        // Count entries that were not freed by the node
        let unfreed_count = table
            .values()
            .filter(|entry| !entry.freed)
            .count();

        if unfreed_count > 0 {
            tracing::info!(
                "memory manager检测到{}条未释放的页锁内存数据，正在释放......",
                unfreed_count
            );
        }

        for id in &ids {
            if let Some(entry) = table.remove(id) {
                if entry.freed {
                    continue;
                }
                if let Some(shm_name) = &entry.metadata.shared_memory_name {
                    if !shm_name.is_empty() {
                        let _ = self.free_shared_memory(shm_name, &entry.metadata);
                    }
                }
            }
        }

        // Clean up pool-mode shmems (dora_pool_0..N) that were skipped by
        // free_shared_memory during normal operation. These are persistent ring
        // buffers and must be unlinked at daemon shutdown.
        #[cfg(target_os = "linux")]
        {
            for i in 0..3 {
                let shm_name = format!("dora_pool_{}", i);
                let shm_path = format!("/dev/shm/{}", shm_name);
                let _ = std::fs::remove_file(&shm_path);
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
