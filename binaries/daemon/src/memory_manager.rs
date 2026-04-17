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
            return Err(format!("Pinned memory with ID {} already registered", id.id));
        }

        table.insert(id, PinnedMemoryEntry {
            metadata,
            in_use: true,
            registered_by,
        });

        Ok(())
    }

    /// Read pinned memory metadata by ID
    pub fn read_pinned_memory(&self, id: &PinnedMemoryId) -> Option<PinnedMemoryMetadata> {
        let table = self.pinned_memory_table.lock().unwrap();
        table.get(id).map(|entry| entry.metadata.clone())
    }

    /// Free pinned memory by ID
    pub fn free_pinned_memory(&self, id: &PinnedMemoryId) -> Result<(), String> {
        let mut table = self.pinned_memory_table.lock().unwrap();

        // Get the entry before removing it
        let entry = match table.remove(id) {
            Some(entry) => entry,
            None => return Err(format!("Pinned memory with ID {} not found", id.id)),
        };

        // Try to free the shared memory if it exists
        // Note: Daemon should not free shared memory directly as it may still be in use
        // and CUDA memory registration needs to be unregistered by the allocating process.
        // The allocating process (Python) is responsible for cleanup via free_pinned_buffer.
        // We just remove the entry from the table here.
        // if let Some(shm_name) = &entry.metadata.shared_memory_name {
        //     if !shm_name.is_empty() {
        //         // Attempt to free the shared memory
        //         // Note: This is a best-effort attempt since the daemon may not have
        //         // direct access to the shared memory or CUDA context
        //         let _ = self.free_shared_memory(shm_name, &entry.metadata);
        //     }
        // }

        Ok(())
    }

    /// Attempt to free shared memory
    fn free_shared_memory(&self, shm_name: &str, metadata: &PinnedMemoryMetadata) -> Result<(), String> {
        tracing::error!("free_shared_memory CALLED! This should not happen. shm_name={}, pid={:?}", shm_name, metadata.allocator_pid);
        // Note: Directly unlinking shared memory from /dev/shm may not work correctly
        // because:
        // 1. The shared memory might still be in use by other processes
        // 2. CUDA pinned memory registration needs to be unregistered in the same process
        // 3. File permissions might prevent the daemon from accessing the shared memory

        // Instead of trying to unlink directly, we log a warning and rely on
        // the allocating process to clean up the memory properly.
        // The allocating process should:
        // 1. Call cudaHostUnregister if cuda_registered is true
        // 2. Close and unlink the SharedMemory object

        tracing::warn!(
            "Shared memory {} needs to be freed by allocating process (PID: {:?}). \
            Daemon cannot safely free CUDA-registered memory from another process.",
            shm_name,
            metadata.allocator_pid
        );

        // We should NOT clean up the shared memory file from the daemon.
        // The allocating process (Python) is responsible for cleanup via free_pinned_buffer.
        // Removing the file while it's still in use by other processes causes errors.
        // Just log a message and return.
        tracing::debug!("Shared memory {} should be freed by allocating process (PID: {:?})", shm_name, metadata.allocator_pid);
        Ok(())

        // #[cfg(not(target_os = "linux"))]
        // {
        //     tracing::debug!("Cannot directly unlink shared memory on this platform: {}", shm_name);
        //     Ok(())
        // }
    }

    /// Get all pinned memory entries (for cleanup on shutdown)
    pub fn get_all_entries(&self) -> Vec<(PinnedMemoryId, PinnedMemoryEntry)> {
        let table = self.pinned_memory_table.lock().unwrap();
        table.iter().map(|(id, entry)| (id.clone(), entry.clone())).collect()
    }

    /// Cleanup all pinned memory on shutdown
    pub fn cleanup_all(&self) -> Result<(), Vec<String>> {
        let mut table = self.pinned_memory_table.lock().unwrap();
        let mut errors = Vec::new();
        let ids: Vec<PinnedMemoryId> = table.keys().cloned().collect();

        for id in ids {
            if let Err(e) = self.free_pinned_memory(&id) {
                errors.push(format!("Failed to free pinned memory {}: {}", id.id, e));
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}