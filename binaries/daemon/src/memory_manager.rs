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

        if table.remove(id).is_none() {
            return Err(format!("Pinned memory with ID {} not found", id.id));
        }

        Ok(())
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