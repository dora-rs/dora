use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;
use dora_message::metadata::ArrowTypeInfo;
use eyre::Context;
use shared_memory_extended::{Shmem, ShmemConf};

use crate::arrow_utils::{buffer_into_arrow_array, decode_arrow_ipc};

pub enum RawData {
    Empty,
    Vec(AVec<u8, ConstAlign<128>>),
    SharedMemory(SharedMemoryData),
}

impl RawData {
    pub fn into_arrow_array(
        self,
        type_info: &ArrowTypeInfo,
        is_ipc: bool,
    ) -> eyre::Result<arrow::array::ArrayData> {
        if is_ipc {
            return self.decode_ipc();
        }

        let raw_buffer = match self {
            RawData::Empty => return Ok(().into_arrow().into()),
            RawData::Vec(data) => {
                let ptr = NonNull::new(data.as_ptr() as *mut _).unwrap();
                let len = data.len();

                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data)) }
            }
            RawData::SharedMemory(data) => {
                let ptr = NonNull::new(data.data.as_ptr() as *mut _).unwrap();
                let len = data.data.len();

                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data)) }
            }
        };

        buffer_into_arrow_array(&raw_buffer, type_info)
    }

    fn decode_ipc(&self) -> eyre::Result<arrow::array::ArrayData> {
        let bytes: &[u8] = match self {
            RawData::Empty => return Ok(().into_arrow().into()),
            RawData::Vec(data) => data,
            RawData::SharedMemory(data) => &data.data,
        };
        decode_arrow_ipc(bytes)
    }
}

impl std::fmt::Debug for RawData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}

pub struct SharedMemoryData {
    pub data: MappedInputData,
    pub _drop: flume::Sender<()>,
}

pub struct MappedInputData {
    memory: Box<Shmem>,
    len: usize,
}

impl MappedInputData {
    pub(crate) unsafe fn map(shared_memory_id: &str, len: usize) -> eyre::Result<Self> {
        let memory = Box::new(
            ShmemConf::new()
                .os_id(shared_memory_id)
                .writable(false)
                .open()
                .wrap_err("failed to map shared memory input")?,
        );
        if len > memory.len() {
            eyre::bail!(
                "mapped input data length ({len}) exceeds shared memory region size ({})",
                memory.len()
            );
        }
        Ok(MappedInputData { memory, len })
    }
}

impl std::ops::Deref for MappedInputData {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { &self.memory.as_slice()[..self.len] }
    }
}

// SAFETY: MappedInputData wraps a `Box<Shmem>` that represents a read-only
// mapping of a POSIX shared memory region. Soundness of Send + Sync relies
// on protocol invariants that Rust cannot verify:
//
// 1. The region is opened with `writable(false)` in `map()` above, so this
//    reader cannot write to the memory.
// 2. The sender (daemon or another node) is required by dora's wire
//    protocol to finish writing and signal readiness BEFORE this reader
//    maps the region, and not to reuse the region UNTIL this reader drops
//    its `SharedMemoryData` (the drop token in the parent struct signals
//    completion via a flume channel).
// 3. Multiple threads concurrently dereferencing the same MappedInputData
//    via `&MappedInputData` is safe under the above protocol: all threads
//    see the same immutable bytes.
//
// Violating invariant (2) — e.g., a buggy daemon that reuses a region
// before the reader's drop token is observed — would produce torn reads
// or inconsistent data. This would be a protocol bug, not a Rust soundness
// bug, but the consequences are the same from the user's perspective.
//
// Flagged by the 2026-03-21 audit as a potentially unsound Sync impl.
// After review on 2026-04-08: the impl is sound modulo the protocol, but
// the fragility is real. See `plan-zenoh-shared-memory.md` for the
// proposed long-term fix: replace this whole layer with Zenoh SHM, which
// handles shared-memory lifecycle through its own tracking.
unsafe impl Send for MappedInputData {}
unsafe impl Sync for MappedInputData {}
