//! The tensor-pool transport: segment layout, seqlock, CUDA helpers and the
//! four operations exposed to Python.
//!
//! **Not covered by dora's 1.0 compatibility guarantees** — opt-in extension,
//! known open defects, no GPU coverage in CI. See `../../README.md`.
//!
//! Reaches dora only through the generic extension channel (see [`crate::seam`]
//! and dora's `docs/extensions.md`). Everything in this file — the `unsafe`
//! pointer arithmetic over the DORADMA header, the seqlock, the embedded
//! `libcudart` ctypes module, the transport selection — stays on this side of
//! that boundary. If a change here seems to need a dora request named after
//! this transport, widen the generic channel instead.

use std::collections::{HashMap, HashSet};
use std::sync::LazyLock;

use eyre::Context;

use arrow::array::{Array, BinaryArray, StringArray};
use arrow::pyarrow::{FromPyArrow, ToPyArrow};
use dora_message::metadata::Parameter;
use dora_node_api::dora_core::config::NodeId;
use dora_node_api::{DataflowId, DoraNode};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyModule};
use shared_memory_extended::ShmemConf;

// ==================== pool_prelude ====================
/// Pre-compiled CUDA DMA helper module. Compiled once (at first use) and reused
/// across all iterations to eliminate per-call PyModule::from_code overhead.
/// Maintains persistent state: pinned host pointers, pre-allocated GPU buffers.
static CUDA_HELPERS: LazyLock<std::sync::Mutex<Option<Py<PyModule>>>> =
    LazyLock::new(|| std::sync::Mutex::new(None));

/// Counter to make pinned memory buffer IDs unique across registrations.
static PINNED_COUNTER: LazyLock<std::sync::Mutex<u64>> = LazyLock::new(|| std::sync::Mutex::new(0));

/// Maximum number of freed pool buffer IDs to remember at once. This is a
/// single budget shared across every peer the process reads from, not a
/// per-stream allowance — once the cap is exceeded the oldest entries are
/// evicted rather than keeping every ID for the life of the process.
const FREED_POOL_IDS_CAP: usize = 4096;

/// Tracks freed pool buffer IDs so the DORADMA fast path can detect
/// read-after-free. Bounded to `FREED_POOL_IDS_CAP` entries (oldest evicted
/// first) so long-running nodes doing register->write->free every frame
/// don't leak memory indefinitely.
///
/// The cap is one shared budget across *all* peers the process reads
/// from, not a per-stream recency window: a high-rate sender's frees can
/// evict a low-rate sender's tombstones well before the low-rate sender's
/// own next free (e.g. a 60Hz peer can cycle the whole cap in ~68s,
/// evicting a 1Hz peer's entries long before they'd naturally expire). An
/// evicted tombstone is harmless — it just makes a stale fast-path read
/// fall back to the existing `warn_missing_tensor_pool`/daemon path
/// instead of being caught here.
static FREED_POOL_IDS: LazyLock<std::sync::Mutex<FreedPoolIds>> =
    LazyLock::new(|| std::sync::Mutex::new(FreedPoolIds::default()));

/// Bounded, insertion-ordered set of freed pool buffer IDs. `set` gives
/// O(1) membership checks; `order` tracks insertion order so the oldest
/// entry can be evicted once `FREED_POOL_IDS_CAP` is exceeded.
#[derive(Default)]
struct FreedPoolIds {
    set: HashSet<String>,
    order: std::collections::VecDeque<String>,
}

impl FreedPoolIds {
    fn insert(&mut self, id: String) {
        if self.set.insert(id.clone()) {
            self.order.push_back(id);
            while self.order.len() > FREED_POOL_IDS_CAP {
                if let Some(oldest) = self.order.pop_front() {
                    self.set.remove(&oldest);
                }
            }
        }
    }

    fn contains(&self, id: &str) -> bool {
        self.set.contains(id)
    }

    fn remove(&mut self, id: &str) {
        if self.set.remove(id) {
            self.order.retain(|x| x != id);
        }
    }

    fn len(&self) -> usize {
        self.set.len()
    }
}

#[cfg(test)]
mod freed_pool_ids_tests {
    use super::{FREED_POOL_IDS_CAP, FreedPoolIds};

    /// Regression test: without a cap, inserting one ID per frame in a
    /// long-running node grows `FREED_POOL_IDS` forever. Inserting well
    /// past the cap must keep the set bounded instead of leaking.
    #[test]
    fn insert_past_cap_does_not_grow_unbounded() {
        let mut freed = FreedPoolIds::default();
        for i in 0..(FREED_POOL_IDS_CAP * 4) {
            freed.insert(format!("pool_node_{i}"));
        }
        assert_eq!(freed.len(), FREED_POOL_IDS_CAP);
    }

    /// Once past the cap, the oldest entries must be evicted first so the
    /// most recently freed buffers (the ones a read-after-free check would
    /// actually care about) stay tracked.
    #[test]
    fn insert_past_cap_evicts_oldest_first() {
        let mut freed = FreedPoolIds::default();
        for i in 0..(FREED_POOL_IDS_CAP * 2) {
            freed.insert(format!("pool_node_{i}"));
        }
        assert!(
            !freed.contains("pool_node_0"),
            "oldest entry should have been evicted"
        );
        let newest = format!("pool_node_{}", FREED_POOL_IDS_CAP * 2 - 1);
        assert!(
            freed.contains(&newest),
            "most recently freed entry should still be tracked"
        );
    }

    /// Duplicate inserts of an already-tracked ID must not double-count
    /// against the cap or push a second copy into the eviction order.
    #[test]
    fn duplicate_insert_is_idempotent() {
        let mut freed = FreedPoolIds::default();
        freed.insert("pool_node_0".to_string());
        freed.insert("pool_node_0".to_string());
        assert_eq!(freed.len(), 1);
    }

    #[test]
    fn remove_drops_membership_and_order_entry() {
        let mut freed = FreedPoolIds::default();
        freed.insert("pool_node_0".to_string());
        freed.remove("pool_node_0");
        assert!(!freed.contains("pool_node_0"));
        assert_eq!(freed.len(), 0);
        // Re-inserting after a remove must not be blocked by a stale
        // order-queue entry left behind by `remove`.
        freed.insert("pool_node_0".to_string());
        assert_eq!(freed.len(), 1);
    }
}

/// Per-pool persistent state.
/// Keeping Shmem alive prevents munmap, preserving stable mmap addresses
/// for pool-hit detection across `register_tensor_pool` calls.
///
/// # Safety
/// `Shmem` is not `Send + Sync` due to raw pointer fields, but `PoolSlot`
/// is always stored behind a `Mutex` and `_shmem` is never accessed after
/// initialization — it is only kept alive for its destructor.
struct PoolSlot {
    _shmem: shared_memory_extended::Shmem,
    base: u64,
    size: usize,
    is_pinned: bool,
    /// CPU page-locked transit buffer for cross-device GPU transfers
    /// without P2P (e.g. RTX 5090).  0 means no transit path.
    transit_ptr: u64,
    /// The GPU device index where the pool buffer was allocated.
    pool_device: i32,
}

unsafe impl Send for PoolSlot {}
unsafe impl Sync for PoolSlot {}

/// Persistent pool storage — stable mmap addresses for zero-copy detection.
/// Keyed by counter (unique per registration), supports unlimited pools.
static PINNED_POOL: LazyLock<std::sync::Mutex<HashMap<u64, PoolSlot>>> =
    LazyLock::new(|| std::sync::Mutex::new(HashMap::new()));

/// Persistent transit-buffer metadata for GPU pools.
/// Survives `PINNED_POOL` cache-miss so the write fast path can recover
/// `transit_ptr` and `pool_device` even when the `PoolSlot` has been evicted.
/// Keyed by counter, populated during `register_tensor_pool`, cleared in
/// `free_tensor_pool`.  `transit_ptr=0` means no transit buffer (same-device
/// or P2P path).
static TRANSIT_META: LazyLock<std::sync::Mutex<HashMap<u64, (u64, i32)>>> =
    LazyLock::new(|| std::sync::Mutex::new(HashMap::new()));

/// Receiver-side GPU cache per pool.
/// Keeps Shmem alive to prevent munmap, preserving stable mmap addresses
/// and valid GPU VAs for zero-copy reads across iterations.
struct RecvGpuSlot {
    _shmem: shared_memory_extended::Shmem,
    gpu_va: u64,       // device VA from cudaHostGetDevicePointer, 0 if IPC path
    gpu_buf: u64,      // IPC-opened GPU DRAM pointer, 0 if GPU VA path
    host_base: u64,    // original host ptr passed to cudaHostRegister
    gpu_buf_size: u64, // GPU buffer byte size from first import (baseline)
}

/// Daemon-trusted GPU buffer sizes, keyed by buffer_id.
/// Populated from daemon metadata when the fallback path successfully
/// re-imports a GPU IPC handle.  The fast path (`try_doradma_read`)
/// validates the (world-writable) shmem `size` against this cache —
/// if `size` exceeds the trusted capacity, the read is rejected and
/// the caller falls back to the daemon.
static GPU_BUF_SIZES: LazyLock<std::sync::Mutex<HashMap<String, u64>>> =
    LazyLock::new(|| std::sync::Mutex::new(HashMap::new()));
unsafe impl Send for RecvGpuSlot {}
unsafe impl Sync for RecvGpuSlot {}

/// Receiver-side per-pool cache keeping Shmem alive + GPU VA for zero-copy reads.
/// Set up lazily in try_doradma_read: open shmem, cudaHostRegister,
/// cudaHostGetDevicePointer, then cache both Shmem and GPU VA.
static RECV_GPU_VA: LazyLock<std::sync::Mutex<HashMap<String, RecvGpuSlot>>> =
    LazyLock::new(|| std::sync::Mutex::new(HashMap::new()));

/// Receiver-side per-pool Shmem cache for CPU receivers.
/// Keeps Shmem alive to prevent munmap of the CPU pointer returned
/// by the as_cuda=False path in try_doradma_read.
struct RecvCpuSlot {
    _shmem: shared_memory_extended::Shmem,
    base: u64,
}
unsafe impl Send for RecvCpuSlot {}
unsafe impl Sync for RecvCpuSlot {}

/// Receiver-side per-pool cache keeping Shmem alive for CPU zero-copy reads.
/// Without this cache, the Shmem handle drops at the end of try_doradma_read,
/// triggering munmap and making the returned CPU pointer a dangling pointer.
static RECV_CPU_SHMEM: LazyLock<std::sync::Mutex<HashMap<String, RecvCpuSlot>>> =
    LazyLock::new(|| std::sync::Mutex::new(HashMap::new()));

// ==================== cross-machine state ====================

/// Receiver-side per-pool HtoD staging cache (cross-machine GPU pools).
/// Separate from RECV_GPU_VA on purpose: that cache's free path calls
/// `_ipc_close` on `gpu_buf`, which would destroy a cudaMalloc'd pointer.
struct RecvGpuHtodSlot {
    _shmem: shared_memory_extended::Shmem,
    host_base: u64, // mirror base (cudaHostRegister'd), for free-path unpin
    gpu_buf: u64,   // pooled GPU DRAM target (cudaMalloc'd, keyed by buffer_id)
    gpu_buf_size: u64,
}
unsafe impl Send for RecvGpuHtodSlot {}
unsafe impl Sync for RecvGpuHtodSlot {}

static RECV_GPU_HTOD: LazyLock<std::sync::Mutex<HashMap<String, RecvGpuHtodSlot>>> =
    LazyLock::new(|| std::sync::Mutex::new(HashMap::new()));

/// Pools registered without a `machine` — they have no mirror on another
/// host, so the write path skips the daemon push for them.
static NO_MIRROR_POOLS: LazyLock<std::sync::Mutex<std::collections::HashSet<String>>> =
    LazyLock::new(|| std::sync::Mutex::new(std::collections::HashSet::new()));

/// Pools whose reader confirmed same-host direct access (register ack
/// `direct == true`): the receiver opens this machine's segment directly,
/// so a daemon push would only feed a mirror nobody reads. The write path
/// skips the push for them.
static DIRECT_POOLS: LazyLock<std::sync::Mutex<std::collections::HashSet<String>>> =
    LazyLock::new(|| std::sync::Mutex::new(std::collections::HashSet::new()));

/// Whether a cross-machine write must push the frame through the daemon.
/// GPU pools with an IPC handle (ipc_present == 1) hold their data in the
/// IPC buffer the receiver imports directly — skip. Pools registered
/// without a `machine` (no mirror) or with a same-host direct reader
/// (DIRECT_POOLS) skip too.
fn should_push_mirror(buffer_id: &str, ipc_present: u64) -> bool {
    if ipc_present == 1 {
        return false;
    }
    let skip_push = {
        let pools = NO_MIRROR_POOLS.lock().unwrap_or_else(|e| e.into_inner());
        pools.contains(buffer_id)
            || DIRECT_POOLS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .contains(buffer_id)
    };
    !skip_push
}

/// Roll a failed cross-machine registration back to a clean local state:
/// unregister the pinned shmem, free any GPU/transit buffers, forget the
/// transit metadata, tombstone the buffer id, and unlink the local segment.
fn rollback_local_pool(
    shmem: &mut shared_memory_extended::Shmem,
    shmem_ptr: u64,
    receiver_is_cuda: bool,
    pool_counter: u64,
    buffer_id: String,
    py: Python<'_>,
) {
    if !receiver_is_cuda && let Ok(helpers) = get_cuda_helpers(py) {
        let bound = helpers.bind(py);
        let _ = bound.call_method1("_unregister_host", (shmem_ptr,));
    }
    {
        let mut pool = PINNED_POOL.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(slot) = pool.remove(&pool_counter) {
            // Defensive: the slot is normally stored after this point, but
            // if it exists free its GPU-side resources too.
            if let Ok(helpers) = get_cuda_helpers(py) {
                let bound = helpers.bind(py);
                let _ = bound.call_method1("_free_gpu_buf", (pool_counter,));
                if slot.transit_ptr != 0 {
                    let _ = bound.call_method1("_free_transit", (slot.transit_ptr,));
                }
            }
        }
    }
    TRANSIT_META
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .remove(&pool_counter);
    FREED_POOL_IDS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .insert(buffer_id);
    // Unlink the local shmem segment (drop with owner=true removes it).
    shmem.set_owner(true);
}

/// DORADMA shared-memory header layout:
///
/// Offset  Size   Field
/// 0       8      magic      — b"DORADMA\x00"
/// 8       8      json_len   — u64 LE, metadata JSON byte length
/// 16      8      data_off   — u64 LE, byte offset of tensor data from shmem base
/// 24      8      ipc_flag   — u64 LE, 1 when ipc_handle is valid
/// 32      64     ipc_handle — CUDA IPC mem handle (only valid if ipc_flag == 1)
/// 96      8      write_gen  — u64 LE, seqlock: even = complete, odd = writing
/// 104     152    reserved
/// 256     N      json       — padded-to-256-byte-alignment metadata JSON
/// 256+N   M      data       — tensor payload
const DORADMA_HEADER_SIZE: usize = 256;
const DORADMA_MAGIC: &[u8; 8] = b"DORADMA\x00";
const DORADMA_METADATA_ALIGN: usize = 256;

/// Crossover where pinned-DMA bandwidth overtakes pageable copy +
/// cudaHostRegister/unregister fixed cost (~100 µs).  Determined by
/// ablation study (2026-06-27): pageable faster below, pinned faster
/// above.  Shared by `register_tensor_pool` and `write_tensor_pool`.
const DMA_PIN_THRESHOLD_BYTES: usize = 25 * 1024 * 1024;

/// Returns `true` when the source tensor should be pinned before DMA.
///
/// Pinning is a property of the *source* pointer: `cudaHostRegister` only
/// makes sense for host (CPU) memory, and the pin/unpin fixed cost
/// (~100 µs) is only worth paying when the tensor is large enough that
/// the DMA bandwidth gain outweighs it.
///
/// # Unit-testable
///
/// The decision is pure integer logic — no CUDA runtime calls — so the
/// boundary (25 MiB ± 1 byte) can be exercised in CI even without a GPU.
#[inline]
const fn should_pin(is_cuda: bool, size: usize) -> bool {
    !is_cuda && size > DMA_PIN_THRESHOLD_BYTES
}

#[cfg(test)]
mod pin_tests {
    use super::*;

    #[test]
    fn pin_cpu_source_above_threshold() {
        // CPU source, 25 MiB + 1 byte → should pin
        assert!(should_pin(false, 25 * 1024 * 1024 + 1));
        // CPU source, 100 MiB → should pin
        assert!(should_pin(false, 100 * 1024 * 1024));
    }

    #[test]
    fn pin_cpu_source_below_threshold() {
        // CPU source, exactly at threshold → should NOT pin (> not >=)
        assert!(!should_pin(false, 25 * 1024 * 1024));
        // CPU source, 1 byte below → should NOT pin
        assert!(!should_pin(false, 25 * 1024 * 1024 - 1));
        // CPU source, tiny → should NOT pin
        assert!(!should_pin(false, 1));
    }

    #[test]
    fn pin_cuda_source_never_pins() {
        // CUDA source regardless of size → never pin
        assert!(!should_pin(true, 0));
        assert!(!should_pin(true, 25 * 1024 * 1024));
        assert!(!should_pin(true, 100 * 1024 * 1024));
        assert!(!should_pin(true, 1024 * 1024 * 1024));
    }

    #[test]
    fn pin_zero_size_cpu() {
        // Zero-size CPU tensor → below threshold, don't pin
        assert!(!should_pin(false, 0));
    }
}

// ---------------------------------------------------------------------------
// GPU transport-path classification — pure decision logic extractable
// from CUDA-runtime-embedded code so the full matrix can be exercised in
// CI without a GPU.  Same pattern as `should_pin` above.
// ---------------------------------------------------------------------------

/// Which transport path a GPU-pool registration (write-time) selects.
///
/// Pure logic — no CUDA runtime calls.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum TransportPath {
    /// Buffer on sender device, plain DtoD memcpy (same-device or CPU source).
    SameDeviceDtoD,
    /// Cross-device with P2P peer access enabled.
    P2PPeerAccess,
    /// Cross-device without P2P — CPU page-locked transit (DtoH → HtoD).
    HostStagingTransit,
}

/// Classify which transport path a GPU-pool write should take at
/// registration time.
///
/// Decision matrix (2³ = 8 cases, `is_cuda_source` dominates):
///
/// | src CUDA | same dev | P2P | path                |
/// |----------|----------|-----|---------------------|
/// | false    | *        | *   | `SameDeviceDtoD`    |
/// | true     | true     | *   | `SameDeviceDtoD`    |
/// | true     | false    | yes | `P2PPeerAccess`     |
/// | true     | false    | no  | `HostStagingTransit`|
#[inline]
fn classify_transport(
    sender_device: i32,
    receiver_device: i32,
    p2p_available: bool,
    is_cuda_source: bool,
) -> TransportPath {
    if !is_cuda_source {
        return TransportPath::SameDeviceDtoD;
    }
    if sender_device == receiver_device {
        return TransportPath::SameDeviceDtoD;
    }
    if p2p_available {
        return TransportPath::P2PPeerAccess;
    }
    TransportPath::HostStagingTransit
}

/// Which write path `write_tensor_pool` dispatches to for a given frame.
///
/// The fast and slow write paths both branch on the same 2×2×2 matrix
/// (`ipc_present` × `is_cuda` × `transit_ptr`); extracting the
/// classification makes the 5 reachable paths explicit and testable.
#[derive(Debug, PartialEq, Eq)]
enum WritePath {
    /// CPU source → GPU pool via `dma_copy` (ipc_present=1, !is_cuda).
    CpuToGpuPoolDma,
    /// GPU source → GPU pool via transit (ipc_present=1, is_cuda, transit_ptr≠0).
    GpuToGpuPoolTransit,
    /// GPU source → GPU pool via plain DtoD `_cuda_memcpy_gpu_buf`
    /// (ipc_present=1, is_cuda, transit_ptr=0).
    GpuToGpuPoolDtoD,
    /// GPU source → shmem data region via `cudaMemcpy` (ipc_present≠1, is_cuda).
    GpuToShmem,
    /// CPU source → shmem data region via `ptr::copy_nonoverlapping`
    /// (ipc_present≠1, !is_cuda).
    CpuToShmem,
}

/// Classify which write path to take.
#[inline]
fn classify_write_path(ipc_present: u64, is_cuda: bool, transit_ptr: u64) -> WritePath {
    if ipc_present == 1 {
        if is_cuda {
            if transit_ptr != 0 {
                WritePath::GpuToGpuPoolTransit
            } else {
                WritePath::GpuToGpuPoolDtoD
            }
        } else {
            WritePath::CpuToGpuPoolDma
        }
    } else if is_cuda {
        WritePath::GpuToShmem
    } else {
        WritePath::CpuToShmem
    }
}

/// Result of validating a GPU-pool read `size` against the daemon-trusted
/// capacity cache (`GPU_BUF_SIZES`) and the first-import baseline
/// (`RecvGpuSlot::gpu_buf_size`).
#[derive(Debug, PartialEq, Eq)]
enum CapacityCheck {
    /// Size is within the trusted bound.
    Ok,
    /// Size exceeds the trusted capacity → reject this read.
    ExceedsTrustedSize,
    /// No daemon-trusted entry and no cached baseline → reject first import.
    NoTrustedEntry,
}

/// Validate the read `size` for a GPU-pool buffer against the daemon-trusted
/// capacity and the cached first-import baseline.
///
/// Resolution order: `trusted_sizes` (daemon metadata) → `cached_gpu_buf_size`
/// (first-import baseline) → reject.
///
/// The daemon rejects zero-size pools at registration, so a resolved capacity
/// of zero is unreachable in normal operation.  If it does occur (daemon bug
/// or memory corruption), the `size > capped` check fails closed — any
/// non-zero `size` triggers `ExceedsTrustedSize` rather than silently skipping
/// validation.
#[inline]
fn check_capacity_gpu_pool(
    trusted_sizes: Option<u64>,
    cached_gpu_buf_size: Option<u64>,
    size: u64,
) -> CapacityCheck {
    let cap = trusted_sizes.or(cached_gpu_buf_size);
    match cap {
        None => CapacityCheck::NoTrustedEntry,
        Some(capped) if size > capped => CapacityCheck::ExceedsTrustedSize,
        Some(_) => CapacityCheck::Ok,
    }
}

#[cfg(test)]
mod transport_tests {
    use super::*;

    // -- classify_transport -------------------------------------------------

    #[test]
    fn same_device_no_transit() {
        // Same GPU — never transit, regardless of P2P
        assert_eq!(
            classify_transport(0, 0, false, true),
            TransportPath::SameDeviceDtoD
        );
        assert_eq!(
            classify_transport(1, 1, true, true),
            TransportPath::SameDeviceDtoD
        );
    }

    #[test]
    fn cpu_source_never_transit() {
        // CPU→GPU always uses dma_copy, no transit needed
        assert_eq!(
            classify_transport(0, 1, false, false),
            TransportPath::SameDeviceDtoD
        );
        assert_eq!(
            classify_transport(0, 2, true, false),
            TransportPath::SameDeviceDtoD
        );
    }

    #[test]
    fn cross_device_with_p2p() {
        assert_eq!(
            classify_transport(0, 1, true, true),
            TransportPath::P2PPeerAccess
        );
    }

    #[test]
    fn cross_device_no_p2p_uses_transit() {
        // This is the RTX 5090 / Blackwell path — the flag-ship non-P2P
        // fallback that must NOT be dead code.
        assert_eq!(
            classify_transport(0, 1, false, true),
            TransportPath::HostStagingTransit
        );
        assert_eq!(
            classify_transport(2, 0, false, true),
            TransportPath::HostStagingTransit
        );
    }

    #[test]
    fn classify_transport_full_8_case_matrix() {
        let cases: &[((i32, i32, bool, bool), TransportPath)] = &[
            // (src_dev, dst_dev, p2p, is_cuda) → expected
            ((0, 0, false, false), TransportPath::SameDeviceDtoD),
            ((0, 0, false, true), TransportPath::SameDeviceDtoD),
            ((0, 0, true, false), TransportPath::SameDeviceDtoD),
            ((0, 0, true, true), TransportPath::SameDeviceDtoD),
            ((0, 1, false, false), TransportPath::SameDeviceDtoD),
            ((0, 1, false, true), TransportPath::HostStagingTransit),
            ((0, 1, true, false), TransportPath::SameDeviceDtoD),
            ((0, 1, true, true), TransportPath::P2PPeerAccess),
        ];
        for ((s, r, p2p, cuda), expected) in cases {
            let got = classify_transport(*s, *r, *p2p, *cuda);
            assert_eq!(
                got, *expected,
                "classify_transport(s={s}, r={r}, p2p={p2p}, cuda={cuda}) → {got:?}, expected {expected:?}"
            );
        }
    }

    // -- classify_write_path -------------------------------------------------

    #[test]
    fn write_path_cpu_to_gpu_pool_dma() {
        assert_eq!(classify_write_path(1, false, 0), WritePath::CpuToGpuPoolDma);
        // transit_ptr is irrelevant when !is_cuda
        assert_eq!(
            classify_write_path(1, false, 0xDEAD),
            WritePath::CpuToGpuPoolDma
        );
    }

    #[test]
    fn write_path_gpu_to_gpu_pool_transit() {
        assert_eq!(
            classify_write_path(1, true, 1),
            WritePath::GpuToGpuPoolTransit
        );
        assert_eq!(
            classify_write_path(1, true, 0xDEAD_BEEF),
            WritePath::GpuToGpuPoolTransit
        );
    }

    #[test]
    fn write_path_gpu_to_gpu_pool_dtod() {
        assert_eq!(classify_write_path(1, true, 0), WritePath::GpuToGpuPoolDtoD);
    }

    #[test]
    fn write_path_gpu_to_shmem() {
        assert_eq!(classify_write_path(0, true, 0), WritePath::GpuToShmem);
        assert_eq!(classify_write_path(0, true, 1), WritePath::GpuToShmem);
    }

    #[test]
    fn write_path_cpu_to_shmem() {
        assert_eq!(classify_write_path(0, false, 0), WritePath::CpuToShmem);
        assert_eq!(classify_write_path(0, false, 1), WritePath::CpuToShmem);
    }

    #[test]
    fn write_path_cache_miss_defaults_to_dtod() {
        // When the write fast path hits a cache miss and constructs a
        // fresh PoolSlot with transit_ptr=0, the dispatch must fall
        // through to plain DtoD — NOT transit.  This documents the
        // current behaviour; if cache-miss transit recovery is added
        // later, this test must be updated.
        let path = classify_write_path(1, true, 0);
        assert_eq!(path, WritePath::GpuToGpuPoolDtoD);
    }

    #[test]
    fn write_path_full_matrix() {
        // 2×2×2 = 8 cases; 6 reachable (ipc_present=1 && transit_ptr≠0
        // for a CPU source is semantically unreachable because transit is
        // only allocated on the CUDA-registration path).
        let cases: &[(u64, bool, u64, WritePath)] = &[
            (1, false, 0, WritePath::CpuToGpuPoolDma),
            (1, false, 1, WritePath::CpuToGpuPoolDma),
            (1, true, 0, WritePath::GpuToGpuPoolDtoD),
            (1, true, 1, WritePath::GpuToGpuPoolTransit),
            (0, false, 0, WritePath::CpuToShmem),
            (0, false, 1, WritePath::CpuToShmem),
            (0, true, 0, WritePath::GpuToShmem),
            (0, true, 1, WritePath::GpuToShmem),
        ];
        for (ipc, cuda, tp, expected) in cases {
            let got = classify_write_path(*ipc, *cuda, *tp);
            assert_eq!(
                got, *expected,
                "classify_write_path(ipc={ipc}, cuda={cuda}, tp={tp}) → {got:?}, expected {expected:?}"
            );
        }
    }

    // -- check_capacity_gpu_pool --------------------------------------------

    #[test]
    fn capacity_ok_within_bounds() {
        // Daemon-trusted cap present, size fits
        assert_eq!(
            check_capacity_gpu_pool(Some(4096), None, 4096),
            CapacityCheck::Ok
        );
        assert_eq!(
            check_capacity_gpu_pool(Some(4096), None, 1),
            CapacityCheck::Ok
        );
    }

    #[test]
    fn capacity_exceeds_trusted_size() {
        assert_eq!(
            check_capacity_gpu_pool(Some(4096), None, 4097),
            CapacityCheck::ExceedsTrustedSize
        );
    }

    #[test]
    fn capacity_fallback_to_gpu_buf_size() {
        // No daemon entry, but cached first-import baseline exists
        assert_eq!(
            check_capacity_gpu_pool(None, Some(4096), 2048),
            CapacityCheck::Ok
        );
        assert_eq!(
            check_capacity_gpu_pool(None, Some(4096), 4097),
            CapacityCheck::ExceedsTrustedSize
        );
    }

    #[test]
    fn capacity_no_trusted_entry_rejects() {
        // Neither daemon nor cached baseline — must fail closed
        assert_eq!(
            check_capacity_gpu_pool(None, None, 1024),
            CapacityCheck::NoTrustedEntry
        );
    }

    #[test]
    fn capacity_zero_trusted_cap_allows_zero_size_only() {
        // The daemon rejects zero-size pools at registration, so
        // a trusted capacity of 0 is unreachable in normal operation.
        // If it does occur, fail closed: any non-zero size must be
        // rejected rather than silently skipping validation.
        assert_eq!(check_capacity_gpu_pool(Some(0), None, 0), CapacityCheck::Ok);
        assert_eq!(
            check_capacity_gpu_pool(Some(0), None, 1024 * 1024),
            CapacityCheck::ExceedsTrustedSize
        );
    }

    #[test]
    fn capacity_zero_cached_buf_size_fails_closed() {
        // Same reasoning as above: a cached baseline of 0 is
        // unreachable; fail closed on any non-zero read.
        assert_eq!(
            check_capacity_gpu_pool(None, Some(0), 1024),
            CapacityCheck::ExceedsTrustedSize
        );
    }
}

/// Get (or compile) the persistent CUDA DMA helper module.
///
/// Compiled once at first use and reused across all subsequent iterations.
/// The module maintains internal state for pinned host pointers and GPU buffers,
/// eliminating per-call cudaHostRegister/cudaMalloc/cudaFree/shm_open overhead.
fn get_cuda_helpers(py: Python<'_>) -> Result<Py<PyModule>, String> {
    let mut guard = CUDA_HELPERS.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(ref module) = *guard {
        return Ok(module.clone_ref(py));
    }

    let code = r#"
import ctypes
_lib = ctypes.CDLL('libcudart.so')

_lib.cudaHostRegister.restype = ctypes.c_int
_lib.cudaHostRegister.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint]

_lib.cudaHostUnregister.restype = ctypes.c_int
_lib.cudaHostUnregister.argtypes = [ctypes.c_void_p]

_lib.cudaMalloc.restype = ctypes.c_int
_lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]

_lib.cudaMemcpy.restype = ctypes.c_int
_lib.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]

_lib.cudaFree.restype = ctypes.c_int
_lib.cudaFree.argtypes = [ctypes.c_void_p]

_lib.cudaDeviceSynchronize.restype = ctypes.c_int

_lib.cudaHostGetDevicePointer.restype = ctypes.c_int
_lib.cudaHostGetDevicePointer.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_void_p, ctypes.c_uint]

cudaMemcpyHostToDevice = 1

# IPC handle struct — must be a Structure subclass so ctypes passes it
# by value (64 bytes on the stack) to cudaIpcOpenMemHandle.
class _CudaIpcMemHandle(ctypes.Structure):
    _fields_ = [("reserved", ctypes.c_byte * 64)]

_lib.cudaIpcGetMemHandle.restype = ctypes.c_int
_lib.cudaIpcGetMemHandle.argtypes = [ctypes.POINTER(_CudaIpcMemHandle), ctypes.c_void_p]

_lib.cudaIpcOpenMemHandle.restype = ctypes.c_int
_lib.cudaIpcOpenMemHandle.argtypes = [ctypes.POINTER(ctypes.c_void_p), _CudaIpcMemHandle, ctypes.c_uint]

_lib.cudaIpcCloseMemHandle.restype = ctypes.c_int
_lib.cudaIpcCloseMemHandle.argtypes = [ctypes.c_void_p]

# Persistent state: per-slot GPU buffer cache
_gpu_bufs = {}    # slot -> (d_ptr, size)

def _register_host(ptr, size):
    """Pin host memory. Idempotent — error 712 (already registered) is ok."""
    err = _lib.cudaHostRegister(ctypes.c_void_p(ptr), size, 0)
    if err != 0 and err != 712:
        raise RuntimeError(f'cudaHostRegister(0x{ptr:x}, {size}) failed: {err}')

def _unregister_host(ptr):
    """Unpin host memory."""
    err = _lib.cudaHostUnregister(ctypes.c_void_p(ptr))
    if err != 0 and err != 713:
        raise RuntimeError(f'cudaHostUnregister(0x{ptr:x}) failed: {err}')

def _get_device_ptr(host_ptr):
    """Get a GPU VA for a pinned host memory region."""
    d_ptr = ctypes.c_void_p()
    err = _lib.cudaHostGetDevicePointer(ctypes.byref(d_ptr), ctypes.c_void_p(host_ptr), 0)
    if err != 0:
        raise RuntimeError(f'cudaHostGetDevicePointer(0x{host_ptr:x}) failed: {err}')
    return d_ptr.value


# P2P and device query bindings (CUDA runtime).
_lib.cudaGetDevice.restype = ctypes.c_int
_lib.cudaGetDevice.argtypes = [ctypes.POINTER(ctypes.c_int)]
_lib.cudaDeviceCanAccessPeer.restype = ctypes.c_int
_lib.cudaDeviceCanAccessPeer.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_int]
_lib.cudaDeviceEnablePeerAccess.restype = ctypes.c_int
_lib.cudaDeviceEnablePeerAccess.argtypes = [ctypes.c_int, ctypes.c_uint]
_lib.cudaSetDevice.restype = ctypes.c_int
_lib.cudaSetDevice.argtypes = [ctypes.c_int]

# Page-locked host allocation for cross-device staging buffers.
_lib.cudaHostAlloc.restype = ctypes.c_int
_lib.cudaHostAlloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t, ctypes.c_uint]
_lib.cudaFreeHost.restype = ctypes.c_int
_lib.cudaFreeHost.argtypes = [ctypes.c_void_p]

_p2p_enabled_pairs = set()    # {(src, dst)} pairs already peer-enabled

def _alloc_transit(size):
    """Allocate a page-locked CPU buffer for cross-device staging.
    Returns the pointer (as int), or 0 on failure."""
    ptr = ctypes.c_void_p()
    if _lib.cudaHostAlloc(ctypes.byref(ptr), size, 0) == 0:
        return ptr.value
    return 0

def _free_transit(ptr):
    """Free a page-locked CPU transit buffer."""
    if ptr:
        _lib.cudaFreeHost(ctypes.c_void_p(ptr))

def _can_access_peer(src, dst):
    """Check if src GPU can P2P-access dst GPU. Returns bool."""
    can = ctypes.c_int(0)
    if _lib.cudaDeviceCanAccessPeer(ctypes.byref(can), src, dst) == 0:
        return can.value != 0
    return False

def _set_cuda_device(idx):
    """Set the current CUDA device.  No-op if *idx* < 0."""
    if idx >= 0:
        _lib.cudaSetDevice(idx)

def _get_cuda_device():
    """Return the current CUDA device index, or -1 on failure."""
    dev = ctypes.c_int()
    if _lib.cudaGetDevice(ctypes.byref(dev)) == 0:
        return dev.value
    return -1

def _transit_copy(src_ptr, src_dev, transit_ptr, dst_ptr, dst_dev, size):
    """Copy via CPU transit: src(GPU) → DtoH → transit → HtoD → dst(GPU).
    Returns True on success.  Restores the caller's current CUDA device on exit."""
    saved = ctypes.c_int()
    _lib.cudaGetDevice(ctypes.byref(saved))
    # GPU src → CPU transit
    _lib.cudaSetDevice(src_dev)
    err = _lib.cudaMemcpy(ctypes.c_void_p(transit_ptr), ctypes.c_void_p(src_ptr), size, 2)
    if err != 0:
        _lib.cudaSetDevice(saved.value)
        return False
    _lib.cudaDeviceSynchronize()
    # CPU transit → GPU dst
    _lib.cudaSetDevice(dst_dev)
    err = _lib.cudaMemcpy(ctypes.c_void_p(dst_ptr), ctypes.c_void_p(transit_ptr), size, 1)
    _lib.cudaDeviceSynchronize()
    _lib.cudaSetDevice(saved.value)
    return err == 0

def _transit_copy_gpu_buf(slot, src_ptr, src_dev, transit_ptr, dst_dev, size):
    """Same as _transit_copy but looks up the pool's GPU buffer by slot.
    Raises on a missing/undersized slot or a failed copy so the caller
    can surface the error instead of silently delivering stale data."""
    if slot not in _gpu_bufs:
        raise RuntimeError(f"GPU pool buffer for slot {slot} not initialised")
    dst, capacity = _gpu_bufs[slot]
    if size > capacity:
        raise RuntimeError(
            f"write size {size} exceeds GPU pool buffer capacity {capacity} (slot={slot})"
        )
    if not _transit_copy(src_ptr, src_dev, transit_ptr, dst, dst_dev, size):
        raise RuntimeError(f"transit copy into GPU pool buffer failed (slot={slot})")

def _ensure_p2p_pair(a, b):
    """Enable bidirectional P2P access for the (a, b) GPU pair only.

    Scoped to the sender/receiver devices actually used by a transfer —
    enabling P2P across *all* GPU pairs would lazily create a CUDA context
    (hundreds of MB) on every visible device, including ones not in this
    dataflow.  Idempotent per pair.
    """
    if a == b or a < 0 or b < 0:
        return
    if (a, b) in _p2p_enabled_pairs:
        return
    # Save the current device so we can restore it after enabling peer
    # access — otherwise later CUDA operations (cudaMalloc,
    # cudaIpcOpenMemHandle) would run on the last enabled src device.
    saved = ctypes.c_int()
    _lib.cudaGetDevice(ctypes.byref(saved))
    for src, dst in ((a, b), (b, a)):
        can = ctypes.c_int(0)
        if _lib.cudaDeviceCanAccessPeer(ctypes.byref(can), src, dst) == 0 and can.value:
            _lib.cudaSetDevice(src)
            _lib.cudaDeviceEnablePeerAccess(dst, 0)
        _p2p_enabled_pairs.add((src, dst))
    _lib.cudaSetDevice(saved.value)

def _cuda_memcpy(dst, src, size, kind):
    """cudaMemcpy wrapper. kind: 1=H2D, 2=D2H, 3=D2D."""
    err = _lib.cudaMemcpy(ctypes.c_void_p(dst), ctypes.c_void_p(src), size, kind)
    if err != 0:
        raise RuntimeError(f"cudaMemcpy(0x{dst:x}, 0x{src:x}, {size}, {kind}) failed: {err}")
    _lib.cudaDeviceSynchronize()

def _cuda_memcpy_gpu_buf(slot, src_ptr, size):
    """Copy *size* bytes from *src_ptr* (GPU) into the pool's pinned GPU buffer
    identified by *slot*.  Used by write_tensor_pool when both source and pool
    buffer are GPU-resident (same-device DtoD copy)."""
    if slot not in _gpu_bufs:
        raise RuntimeError(f"GPU pool buffer for slot {slot} not initialised")
    dst, capacity = _gpu_bufs[slot]
    if size > capacity:
        # The GPU buffer was sized at registration and its IPC handle is
        # already exported; growing it would invalidate the receiver's
        # imported handle.  Reject rather than overflow the allocation.
        raise RuntimeError(
            f"write size {size} exceeds GPU pool buffer capacity {capacity} (slot={slot})"
        )
    err = _lib.cudaMemcpy(ctypes.c_void_p(dst), ctypes.c_void_p(src_ptr), size, 3)
    if err != 0:
        raise RuntimeError(f"cudaMemcpy GPU buf DtoD (slot={slot}, 0x{dst:x}←0x{src_ptr:x}, {size}B) failed: {err}")
    _lib.cudaDeviceSynchronize()

def _get_gpu_buf(slot, size):
    """Get or allocate a GPU buffer for the given slot. Reuses when size matches."""
    if slot in _gpu_bufs and _gpu_bufs[slot][1] >= size:
        return _gpu_bufs[slot][0]
    if slot in _gpu_bufs:
        _lib.cudaFree(ctypes.c_void_p(_gpu_bufs[slot][0]))
    d_ptr = ctypes.c_void_p()
    err = _lib.cudaMalloc(ctypes.byref(d_ptr), size)
    if err != 0:
        raise RuntimeError(f'cudaMalloc({size}) failed: {err}')
    _gpu_bufs[slot] = (d_ptr.value, size)
    return d_ptr.value

def _free_gpu_buf(slot):
    """Free the pooled GPU buffer for the given slot."""
    if slot in _gpu_bufs:
        _lib.cudaFree(ctypes.c_void_p(_gpu_bufs[slot][0]))
        del _gpu_bufs[slot]

def _ipc_export(d_ptr):
    """Export GPU memory for cross-process sharing. Returns 64-byte handle."""
    handle = _CudaIpcMemHandle()
    err = _lib.cudaIpcGetMemHandle(ctypes.byref(handle), ctypes.c_void_p(d_ptr))
    if err != 0:
        raise RuntimeError(f'cudaIpcGetMemHandle(0x{d_ptr:x}) failed: {err}')
    return bytes(handle)

def _ipc_import(handle_bytes):
    """Import GPU memory from another process. Returns device pointer.

    cudaIpcOpenMemHandle takes the handle struct BY VALUE (64 bytes on
    the stack). _CudaIpcMemHandle.from_buffer_copy reconstructs the struct
    and ctypes copies it onto the stack for the call.
    """
    handle = _CudaIpcMemHandle.from_buffer_copy(handle_bytes)
    d_ptr = ctypes.c_void_p()
    err = _lib.cudaIpcOpenMemHandle(ctypes.byref(d_ptr), handle, 1)
    if err != 0:
        raise RuntimeError(f'cudaIpcOpenMemHandle failed: {err}')
    _lib.cudaDeviceSynchronize()
    return d_ptr.value

def _ipc_close(d_ptr):
    """Close an IPC memory handle opened by _ipc_import.
    Frees the GPU-side mapping without freeing the underlying allocation."""
    err = _lib.cudaIpcCloseMemHandle(ctypes.c_void_p(d_ptr))
    if err != 0:
        raise RuntimeError(f'cudaIpcCloseMemHandle(0x{d_ptr:x}) failed: {err}')

def dma_copy(ptr, size, slot, no_dma):
    """DMA transfer from host to pre-allocated GPU buffer.

    Copies via cudaMemcpyHtoD (DMA engine).  When *no_dma* is false
    (the default), the source memory is pinned (cudaHostRegister)
    before the copy and unpinned after — this is the fast path for
    large tensors where pinned-DMA bandwidth outweighs the pin/unpin
    fixed cost.  When *no_dma* is true, pin/unpin is skipped, using
    pageable memory (faster for small tensors where pin overhead
    dominates).

    Returns the device pointer of the pooled GPU buffer.

    Raises RuntimeError if *size* exceeds the existing buffer capacity
    and the buffer cannot be grown (GPU pool buffers are IPC-exported;
    reallocation would invalidate the receiver's imported handle).
    """
    if slot in _gpu_bufs:
        _, capacity = _gpu_bufs[slot]
        if size > capacity:
            raise RuntimeError(
                f"write size {size} exceeds GPU pool buffer capacity {capacity} (slot={slot})"
                " — GPU buffer cannot be grown (IPC handle already exported)"
            )
    if not no_dma:
        _register_host(ptr, size)
    try:
        d_ptr = _get_gpu_buf(slot, size)
        err = _lib.cudaMemcpy(
            ctypes.c_void_p(d_ptr),
            ctypes.c_void_p(ptr),
            size,
            cudaMemcpyHostToDevice,
        )
        if err != 0:
            raise RuntimeError(f'cudaMemcpy failed: {err}')
        _lib.cudaDeviceSynchronize()
    finally:
        if not no_dma:
            _unregister_host(ptr)
    return d_ptr

"#;

    let code_cstr = std::ffi::CString::new(code).map_err(|e| format!("CString: {}", e))?;
    let bound_module = PyModule::from_code(
        py,
        code_cstr.as_c_str(),
        c"_cuda_helpers.py",
        c"_cuda_helpers",
    )
    .map_err(|e| format!("CUDA helper compile failed: {}", e))?;
    let module_ref: Py<PyModule> = bound_module.unbind();
    *guard = Some(module_ref.clone_ref(py));
    Ok(module_ref)
}

/// Read 8 consecutive bytes from `p` as a little-endian u64,
/// without an `.unwrap()` call (avoids the CI unwrap budget).
fn read_header_u64(p: *const u8) -> u64 {
    const { assert!(std::mem::size_of::<u64>() == 8) };
    let mut buf = [0u8; 8];
    unsafe { std::ptr::copy_nonoverlapping(p, buf.as_mut_ptr(), 8) };
    u64::from_le_bytes(buf)
}

fn parse_tensor_pool_id(tensor_pool_id: Py<PyAny>, py: Python<'_>) -> eyre::Result<String> {
    let array_data = arrow::array::ArrayData::from_pyarrow_bound(tensor_pool_id.bind(py))?;
    let array = arrow::array::make_array(array_data);

    if let Some(string_array) = array.as_any().downcast_ref::<StringArray>() {
        if string_array.len() != 1 {
            eyre::bail!(
                "expected string array with exactly one element, got {}",
                string_array.len()
            );
        }
        Ok(string_array.value(0).to_string())
    } else if let Some(binary_array) = array.as_any().downcast_ref::<BinaryArray>() {
        if binary_array.len() != 1 {
            eyre::bail!(
                "expected binary array with exactly one element, got {}",
                binary_array.len()
            );
        }
        Ok(String::from_utf8(binary_array.value(0).to_vec())?)
    } else {
        eyre::bail!(
            "tensor_pool_id must be a string or binary array, got {:?}",
            array.data_type()
        )
    }
}

fn warn_missing_tensor_pool(node_id: &NodeId, action: &str, buffer_id: &str) {
    tracing::warn!(
        "[{}] Attempt to {} tensor pool [{}] failed - reason: pool does not exist. Operation aborted.",
        node_id,
        action,
        buffer_id
    );
}

// ==================== seqlock ====================
/// Begins a tensor-pool seqlock write at `gen_ptr` (header offset 96)
/// **if the generation is even**.  Returns the **even** pre-write
/// generation — i.e., the generation value before the write cycle
/// began, which is always even.
///
/// If the generation is already odd (leftover from a previous failed
/// write), the begin-increment is skipped and the **previous** even
/// generation (`cur - 1`) is returned.  This ensures that
/// `seqlock_end`'s `pre + 2` always produces an even generation,
/// avoiding a permanent parity inversion.
unsafe fn seqlock_begin_if_even(gen_ptr: *mut u64) -> u64 {
    unsafe {
        let cur = std::ptr::read_volatile(gen_ptr);
        if cur.is_multiple_of(2) {
            std::ptr::write_volatile(gen_ptr, cur + 1);
            std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        }
        cur & !1 // always return the even baseline
    }
}

/// Closes a tensor-pool seqlock write (header offset 96).
///
/// Advances the generation to `pre_write_gen + 2` (even = "complete").
/// The caller must only invoke this on a successful copy.  GPU pool write
/// paths leave gen odd on failure (in-place writes cannot roll back to a
/// clean previous frame; double-buffering is deferred to a follow-up PR).
/// The `copy_ok == false` rollback branch is retained for the helper's
/// contract but is dead code in production — see the leave-gen-odd blocks
/// in `write_tensor_pool`.
unsafe fn seqlock_end(gen_ptr: *mut u64, pre_write_gen: u64, copy_ok: bool) {
    unsafe {
        if copy_ok {
            std::ptr::write_volatile(gen_ptr, pre_write_gen.wrapping_add(2));
        } else {
            std::ptr::write_volatile(gen_ptr, pre_write_gen);
        }
        std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
    }
}

#[cfg(test)]
mod seqlock_tests {
    #[test]
    fn begin_if_even_when_even_flips_to_odd() {
        let mut generation: u64 = 10;
        unsafe {
            super::seqlock_begin_if_even(&mut generation);
        }
        assert_eq!(generation, 11, "even generation must be flipped to odd");
    }

    #[test]
    fn begin_if_even_when_odd_returns_even_baseline() {
        let mut generation: u64 = 11;
        let pre;
        unsafe {
            pre = super::seqlock_begin_if_even(&mut generation);
        }
        assert_eq!(generation, 11, "odd generation must stay odd (skip begin)");
        assert_eq!(pre, 10, "pre-write baseline must be even (cur - 1)");
        assert_eq!(pre % 2, 0);
    }

    #[test]
    fn begin_if_even_then_end_always_produces_even() {
        // Simulates: gen stuck odd (11) from failure → begin returns 10
        // → end does 10+2=12 which is even → pool recovers.
        let mut generation: u64 = 11;
        let pre;
        unsafe {
            pre = super::seqlock_begin_if_even(&mut generation);
        }
        unsafe {
            super::seqlock_end(&mut generation, pre, true);
        }
        assert_eq!(generation, 12, "pre(10)+2=12 is even → recovery");
        assert_eq!(generation % 2, 0);
    }

    /// Regression test for #2436: a successful copy publishes the new
    /// frame by advancing the generation to pre+2 (even = complete).
    #[test]
    fn end_success_advances_to_even() {
        let mut generation: u64 = 11; // pre_write(10) + 1
        unsafe {
            super::seqlock_end(&mut generation, 10, true);
        }
        assert_eq!(generation, 12);
        assert_eq!(generation % 2, 0, "generation must be even on success");
    }

    /// Regression test for #2436: on failure the generation is rolled
    /// back to pre_write_gen (even) so consumers see the previous
    /// valid frame, not a torn/incomplete one.
    #[test]
    fn end_failure_rolls_back_to_even() {
        let mut generation: u64 = 11; // pre_write(10) + 1
        unsafe {
            super::seqlock_end(&mut generation, 10, false);
        }
        assert_eq!(generation, 10);
        assert_eq!(generation % 2, 0, "rolled-back generation must be even");
    }

    /// The very first write (pre_write == 0) must not underflow.
    #[test]
    fn end_first_write_failure_rolls_back_to_zero() {
        let mut generation: u64 = 1;
        unsafe {
            super::seqlock_end(&mut generation, 0, false);
        }
        assert_eq!(generation, 0);
    }
}

// ==================== the transport ====================

/// One node's view of the pool transport.
///
/// Holds what the methods below need from dora — nothing more. Constructed
/// per call by the thin `#[pymethods]` wrappers in `dora-node-api-python`,
/// because PyO3 cannot add methods to its `Node` from another crate.
pub struct Pool<'a> {
    pub node: &'a mut DoraNode,
    pub node_id: NodeId,
    pub dataflow_id: DataflowId,
}

impl Pool<'_> {
    // ==================== process_pending_frees ====================
    /// Process any tensor pools that were freed by another node.
    pub fn process_pending_tensor_pool_frees(&mut self, py: Python) {
        for shared_memory_id in crate::seam::drain_dropped(crate::seam::NAMESPACE) {
            let buffer_id = shared_memory_id;
            // Receiver-side cleanup (IPC handles, shmem mappings).
            {
                if let Some(slot) = RECV_GPU_VA
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&buffer_id)
                {
                    if slot.gpu_buf != 0 {
                        if let Ok(helpers) = get_cuda_helpers(py) {
                            let bound = helpers.bind(py);
                            let _ = bound.call_method1("_ipc_close", (slot.gpu_buf,));
                        }
                    } else if slot.gpu_va != 0 {
                        // Host-registered mapping (effective_as_cuda branch):
                        // must cudaHostUnregister before munmap.  _unregister_host
                        // requires the original host pointer (shmem base), not the
                        // device VA returned by cudaHostGetDevicePointer — passing
                        // the device VA makes cudaHostUnregister fail and leaks the
                        // pin over an address range that then gets munmap'd.
                        if let Ok(helpers) = get_cuda_helpers(py) {
                            let bound = helpers.bind(py);
                            let _ = bound.call_method1("_unregister_host", (slot.host_base,));
                        }
                    }
                    // Drop slot → munmap
                }
            }
            RECV_CPU_SHMEM
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .remove(&buffer_id);

            // Sender-side cleanup (PINNED_POOL, GPU/transit buffers).
            // Guard against cross-process counter aliasing: buffer ids are
            // pool_{node_id}_{counter}.  Extract the owner segment (between
            // "pool_" and the final "_<counter>") and require an exact
            // equality match — a prefix check (starts_with) would alias
            // across node ids that are prefixes of each other (e.g. cam /
            // cam_left).
            if let Some(owner_and_counter) = buffer_id.strip_prefix("pool_")
                && let Some((owner, counter_str)) = owner_and_counter.rsplit_once('_')
                && owner == self.node_id.as_ref()
                && let Ok(c) = counter_str.parse::<u64>()
                && let Some(slot) = PINNED_POOL
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&c)
            {
                if let Ok(helpers) = get_cuda_helpers(py) {
                    let bound = helpers.bind(py);
                    let _ = bound.call_method1("_unregister_host", (slot.base,));
                    let _ = bound.call_method1("_free_gpu_buf", (c,));
                    if slot.transit_ptr != 0 {
                        let _ = bound.call_method1("_free_transit", (slot.transit_ptr,));
                    }
                }
                TRANSIT_META
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&c);
            }
        }
    }

    // ==================== pool_api_methods ====================

    // === Tensor Pool API ===

    /// Register a shared tensor pool for zero-copy tensor transfer.
    ///
    /// The returned pool ID can be shared across nodes (e.g. via a Dora output)
    /// so that a receiver can call [`read_tensor_pool`] and [`free_tensor_pool`]
    /// on it.
    ///
    /// # Concurrency / safety
    ///
    /// **This pool provides no internal mutual exclusion for data bytes.**
    /// The writer must not begin a new [`write_tensor_pool`] while a receiver is
    /// still consuming the previous tensor. Callers that share a pool across
    /// nodes MUST enforce a **turn-based discipline** — for example, by waiting
    /// for a `next_require` round-trip from the receiver before writing again.
    /// The bundled `examples/tensor-pool/` dataflows demonstrate this pattern.
    ///
    /// The on-segment seqlock guards metadata integrity (header fields written
    /// once at registration) and detects in-flight overwrites, but it does
    /// **not** block the writer from starting a new write while a consumer
    /// holds a zero-copy tensor. Skipping the turn-based discipline risks
    /// torn data at the consumer.
    /// Register a tensor pool.
    ///
    /// `machine`: target machine id for a cross-machine pool — the daemon
    /// mirrors the pool there (coordinator resolves the machine; sync
    /// confirm). `None` registers a local pool with no mirror. `name`:
    /// explicit `/dev/shm` segment name (dora-owned `dora_pool_*`
    /// namespace only); `None` derives a machine-qualified name from
    /// `DORA_MACHINE_ID` when set.
    pub fn register_tensor_pool(
        &mut self,
        tensor_info: &Bound<'_, PyDict>,
        device: String,
        machine: Option<String>,
        name: Option<String>,
        py: Python,
    ) -> eyre::Result<Py<PyAny>> {
        let ptr_val: u64 = tensor_info
            .get_item("ptr")?
            .ok_or_else(|| eyre::eyre!("missing ptr"))?
            .extract()?;
        let size: usize = tensor_info
            .get_item("size")?
            .ok_or_else(|| eyre::eyre!("missing size"))?
            .extract()?;
        let dtype: String = tensor_info
            .get_item("dtype")?
            .ok_or_else(|| eyre::eyre!("missing dtype"))?
            .extract()?;
        let shape_list: Vec<i64> = tensor_info
            .get_item("shape")?
            .ok_or_else(|| eyre::eyre!("missing shape"))?
            .extract()?;
        let tensor_device: String = tensor_info
            .get_item("device")?
            .ok_or_else(|| eyre::eyre!("missing device"))?
            .extract()?;

        let is_cuda = tensor_device.starts_with("cuda");
        let receiver_is_cuda = device.starts_with("cuda");
        // Cross-machine registration: the pool gets a mirror on the target
        // machine. A GPU receiver's cross-machine pool is staged through the
        // shmem data region (receiver HtoD), never through an IPC handle —
        // CUDA IPC handles are only valid within one host.
        let cross_machine = machine.is_some();
        let cpu_mode = !receiver_is_cuda;
        // Auto-select pinning: key off the source device — pinning only
        // matters when the source is CPU (cudaHostRegister would raise on a
        // device pointer; prevented by the !is_cuda guard above).
        let is_pinned = should_pin(is_cuda, size);
        let pinned_type = if cpu_mode { "cpu" } else { "cuda" };

        if ptr_val == 0 {
            eyre::bail!("Invalid source pointer (NULL)");
        }
        if size == 0 || size > 1024 * 1024 * 1024 {
            eyre::bail!("Invalid size: {} bytes", size);
        }
        if cfg!(not(target_os = "linux")) {
            eyre::bail!(
                "tensor-pool transport requires Linux (uses /dev/shm). \
                 This platform is not supported."
            );
        }

        // Generate unique pool counter for this registration
        let pool_counter = {
            let mut c = PINNED_COUNTER.lock().unwrap_or_else(|e| e.into_inner());
            *c += 1;
            *c
        };
        // Segment name: an explicit `name` is used verbatim (after the
        // path-traversal checks below); otherwise the name is generated
        // with the local machine id when known, so multi-dataflow /
        // multi-node deployments cannot collide and leftover segments are
        // attributable to their owner machine.
        let shmem_name = match &name {
            Some(name) => {
                if name.is_empty()
                    || name.contains('/')
                    || name.contains("..")
                    || name.len() > 128
                    || !name.starts_with("dora_pool_")
                {
                    eyre::bail!(
                        "invalid tensor pool name `{name}`: must be non-empty, start with \
                         `dora_pool_` (the dora-owned /dev/shm namespace), without '/' or \
                         '..', and at most 128 chars"
                    );
                }
                name.clone()
            }
            None => {
                let machine_id = std::env::var("DORA_MACHINE_ID").unwrap_or_default();
                if machine_id.is_empty() {
                    format!(
                        "dora_pool_{}_{}_{}",
                        self.dataflow_id, self.node_id, pool_counter
                    )
                } else {
                    format!(
                        "dora_pool_{}_{}_{}_{}",
                        machine_id, self.dataflow_id, self.node_id, pool_counter
                    )
                }
            }
        };

        let header_meta = PyDict::new(py);
        header_meta.set_item("size", size)?;
        header_meta.set_item("dtype", &dtype)?;
        header_meta.set_item("shape", shape_list.clone())?;
        header_meta.set_item("pinned_type", pinned_type)?;

        let json_bytes = py
            .import("json")
            .wrap_err("failed to import json")?
            .call_method1("dumps", (header_meta,))
            .wrap_err("failed to serialize metadata to JSON")?
            .extract::<String>()
            .wrap_err("failed to extract JSON string")?
            .into_bytes();
        let json_len = json_bytes.len();
        let padded_json_len = json_len.div_ceil(DORADMA_METADATA_ALIGN) * DORADMA_METADATA_ALIGN;
        let data_offset = DORADMA_HEADER_SIZE + padded_json_len;
        // GPU receivers read tensor data from the IPC-exported GPU buffer,
        // not from the shmem data region.  Allocate only the header portion
        // (metadata + IPC handle + seqlock) — a few hundred bytes instead of
        // 80 MB.  This also lets us skip cudaHostRegister on a useless data
        // region.  Cross-machine GPU pools are the exception: CUDA IPC
        // handles are host-local, so the mirror stores the data region and
        // the receiver stages it HtoD — the segment must be full-size or
        // the registration push cannot read the frame (see the push
        // condition below).
        let total_size = if receiver_is_cuda && !cross_machine {
            data_offset
        } else {
            data_offset + size
        };

        // Create shared memory
        let mut shmem = ShmemConf::new()
            .os_id(&shmem_name)
            .size(total_size)
            .writable(true)
            .create()
            .wrap_err_with(|| {
                format!(
                    "failed to create pool shared memory `{}` (name collision with another node or leftover segment)",
                    shmem_name
                )
            })?;
        let shmem_ptr = unsafe { shmem.as_slice_mut().as_mut_ptr() };

        // Pin the shmem for DMA only when the receiver reads from it
        // (CPU receivers).  GPU receivers never touch the shmem data
        // region, and the header-only shmem is too small (< 1 page) to
        // benefit from pinning.
        if !receiver_is_cuda && let Ok(helpers) = get_cuda_helpers(py) {
            let bound = helpers.bind(py);
            let _ = bound.call_method1("_register_host", (shmem_ptr as u64, total_size));
        }

        shmem.set_owner(false);

        // Write DORADMA header
        unsafe {
            std::ptr::copy_nonoverlapping(DORADMA_MAGIC.as_ptr(), shmem_ptr, 8);
            let json_len_le = (json_len as u64).to_le_bytes();
            std::ptr::copy_nonoverlapping(json_len_le.as_ptr(), shmem_ptr.add(8), 8);
            let data_off_le = (data_offset as u64).to_le_bytes();
            std::ptr::copy_nonoverlapping(data_off_le.as_ptr(), shmem_ptr.add(16), 8);
            std::ptr::copy_nonoverlapping(
                json_bytes.as_ptr(),
                shmem_ptr.add(DORADMA_HEADER_SIZE),
                json_len,
            );
        }

        // Initialize seqlock write_gen at header[96] to 0 (even = complete)
        unsafe {
            std::ptr::write(shmem_ptr.add(96) as *mut u64, 0u64);
        }

        // Seqlock: increment generation to odd (write-in-progress)
        unsafe {
            let gen_ptr = shmem_ptr.add(96) as *mut u64;
            let old_gen = std::ptr::read_volatile(gen_ptr);
            std::ptr::write_volatile(gen_ptr, old_gen + 1);
            std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        }

        // Copy tensor data to shmem — only when the receiver will
        // actually read it.  GPU receivers import the pool GPU buffer
        // via the IPC handle in the DORADMA header and never touch the
        // shmem data region; skipping this copy for them eliminates
        // a redundant CPU-memcpy or GPU-DtoH transfer on every
        // registration (cpu2cuda and cuda2cuda respectively).
        if !receiver_is_cuda {
            // The DtoH copy must publish either a fully-initialized data
            // region or nothing — uninitialized shmem exposed as a valid
            // frame is data corruption.  Both a failed cudaMemcpy and a
            // missing CUDA helper module are treated as copy failures.
            if is_cuda {
                // A missing CUDA helper module counts as a failed copy: the
                // consumer must see the whole region or nothing.
                let dtoh_copy_ok = if let Ok(helpers) = get_cuda_helpers(py) {
                    let bound = helpers.bind(py);
                    bound
                        .call_method1(
                            "_cuda_memcpy",
                            (shmem_ptr as u64 + data_offset as u64, ptr_val, size, 2u32),
                        )
                        .is_ok()
                } else {
                    false
                };
                if !dtoh_copy_ok {
                    // The matching `_register_host` above is unconditional (it
                    // runs whenever `!receiver_is_cuda`), so the unregister must
                    // be too — gating it on `!is_pinned` would leak the pin if
                    // `should_pin` is ever tuned to pin CUDA sources.
                    if let Ok(helpers) = get_cuda_helpers(py) {
                        let bound = helpers.bind(py);
                        let _ = bound.call_method1("_unregister_host", (shmem_ptr as u64,));
                    }
                    shmem.set_owner(true);
                    eyre::bail!(
                        "[{}] register_tensor_pool: DtoH copy failed ({} → CPU shmem, {} bytes)",
                        self.node_id,
                        tensor_device,
                        size
                    );
                }
            } else {
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        ptr_val as *const u8,
                        shmem_ptr.add(data_offset),
                        size,
                    );
                }
            }
        }

        // Cross-machine: mirror the pool on the target machine via the
        // daemon (coordinator resolves the machine; sync confirm).  The
        // mirror is independent of the local receiver's device — the
        // target machine's pool is always CPU-style DORADMA shmem.
        // Failure (unresolved machine, remote pool creation failure, ack
        // timeout, or a broken daemon channel — including the
        // interactive / integration-testing mocks, which reject
        // cross-machine registration) is a warn-and-no-op: the daemon has
        // already logged a warning; we roll back the local pool and
        // return None rather than crash.
        let buffer_id = format!("pool_{}_{}", self.node_id, pool_counter);
        // Tracks whether the GPU pool buffer + IPC handle were successfully
        // set up.  Declared here (before the cross-machine ack handling) so
        // the same-host IPC export below can set it; the GPU allocation block
        // and the bail check below read it too.
        let mut ipc_written = false;
        if machine.is_none() {
            NO_MIRROR_POOLS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(buffer_id.clone());
        }
        if let Some(target_machine) = machine {
            let buffer_id = format!("pool_{}_{}", self.node_id, pool_counter);
            // register_cross_machine_pool returns `Result<Result<(), String>,
            // eyre::Error>`: the outer Err is a transport failure (daemon
            // channel closed, interactive / integration-testing mock mode),
            // the inner Err the daemon-reported mirror failure.  Map the
            // transport Err to the same String type so the two failure
            // channels merge below.
            let result = self
                .node
                .register_cross_machine_pool(
                    buffer_id.clone(),
                    shmem_name.clone(),
                    size,
                    dtype.clone(),
                    shape_list.clone(),
                    // The mirror's consumer is the receiver — relay the
                    // RECEIVER device, not the source device, so the
                    // mirror's pinned_type matches how the receiver reads
                    // it (cpu = data region, cuda = HtoD staging).
                    device.clone(),
                    target_machine,
                )
                .map_err(|e| format!("{e:?}"));
            match result {
                Ok((Ok(()), direct)) => {
                    if direct {
                        DIRECT_POOLS
                            .lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .insert(buffer_id.clone());
                        // Same-host direct access: the receiver can open this
                        // machine's CUDA IPC handle.  Cross-machine GPU pools
                        // skip IPC export at registration (handles are
                        // host-local), but on a same-host deployment the
                        // export is valid — it turns the staging path
                        // (DtoH → zenoh → mirror → HtoD) into zero-copy IPC
                        // reads.  Export now, best-effort: stage the initial
                        // frame into the pooled GPU buffer and publish the
                        // handle; the write path switches automatically
                        // (classify_write_path sees ipc_present == 1) and
                        // the mirror push stops (should_push_mirror).  Any
                        // failure keeps the staging path (ipc_written stays
                        // false) — the pool still works.
                        if receiver_is_cuda
                            && !ipc_written
                            && let Ok(helpers) = get_cuda_helpers(py)
                        {
                            {
                                let bound = helpers.bind(py);
                                let receiver_device_idx = device
                                    .rsplit(':')
                                    .next()
                                    .and_then(|s| s.parse::<i32>().ok())
                                    .unwrap_or(0);
                                let saved_dev = bound
                                    .call_method0("_get_cuda_device")
                                    .and_then(|r| r.extract::<i32>())
                                    .unwrap_or(0);
                                let _ =
                                    bound.call_method1("_set_cuda_device", (receiver_device_idx,));
                                let gpu_ptr_opt = bound
                                    .call_method1("_get_gpu_buf", (pool_counter, size))
                                    .and_then(|r| r.extract::<u64>())
                                    .ok();
                                let _ = bound.call_method1("_set_cuda_device", (saved_dev,));
                                if let Some(gpu_ptr) = gpu_ptr_opt {
                                    // Stage the initial frame into the GPU
                                    // buffer (the data region already holds
                                    // it from the registration copy above).
                                    let htod_ok = bound
                                        .call_method1(
                                            "_cuda_memcpy",
                                            (
                                                gpu_ptr,
                                                shmem_ptr as u64 + data_offset as u64,
                                                size,
                                                1u32,
                                            ),
                                        )
                                        .is_ok();
                                    if htod_ok
                                        && let Ok(handle) = bound
                                            .call_method1("_ipc_export", (gpu_ptr,))
                                            .and_then(|r| r.extract::<Vec<u8>>())
                                        && handle.len() == 64
                                    {
                                        unsafe {
                                            std::ptr::copy_nonoverlapping(
                                                handle.as_ptr(),
                                                shmem_ptr.add(32),
                                                64,
                                            );
                                            std::ptr::write(shmem_ptr.add(24) as *mut u64, 1u64);
                                        }
                                        ipc_written = true;
                                    }
                                }
                            }
                        }
                    }
                    // Local pool stays; the daemon recorded CROSS_POOLS.
                    // Push the registered tensor through the daemon so the
                    // mirror pool is populated before the first explicit
                    // write. The receiver's first read blocks on this data
                    // (flow control: the receiver cannot send next_require
                    // until it has the pool data), so the push must happen
                    // at registration — the write path alone would deadlock
                    // on message 1. Only for CPU receivers: GPU pools
                    // travel via the IPC handle, which the daemon path
                    // cannot carry. Local pools (no `machine`) skip this —
                    // their receivers read the local shmem directly.
                    // A cross-machine GPU pool stages its data region too
                    // (no IPC handle crosses hosts), so it pushes as well —
                    // unless the same-host IPC export above succeeded: then
                    // the receiver reads the GPU buffer directly and the
                    // push would only feed a mirror nobody reads (same
                    // semantics as should_push_mirror on the write path).
                    if (!receiver_is_cuda || cross_machine) && !ipc_written {
                        self.push_mirror_update(&buffer_id, size, "register_tensor_pool");
                    }
                }
                Ok((Err(msg), _)) | Err(msg) => {
                    tracing::warn!(
                        "[{}] register_tensor_pool: cross-machine mirror failed for {}: {msg}",
                        self.node_id,
                        buffer_id
                    );
                    rollback_local_pool(
                        &mut shmem,
                        shmem_ptr as u64,
                        receiver_is_cuda,
                        pool_counter,
                        buffer_id,
                        py,
                    );
                    return Ok(py.None());
                }
            }
        }

        // GPU pool: allocate GPU buffer on current device, copy data, export
        // IPC handle for cross-process zero-copy access.  When the source
        // tensor is also on CUDA (GPU→GPU), the source and pool buffer are on
        // the same device (sender's current CUDA device), so a plain DtoD
        // memcpy suffices.  When the source is CPU, `dma_copy` does a pinned
        // host→device DMA copy (existing path).
        // Resolve sender and receiver device indices for cross-device detection.
        let sender_device_idx = tensor_device
            .strip_prefix("cuda")
            .and_then(|s| s.strip_prefix(':'))
            .and_then(|s| s.parse::<i32>().ok())
            .unwrap_or(0);
        let receiver_device_idx = device
            .strip_prefix("cuda")
            .and_then(|s| s.strip_prefix(':'))
            .and_then(|s| s.parse::<i32>().ok())
            .unwrap_or(0);
        let cross_device = sender_device_idx != receiver_device_idx;
        let mut transit_ptr: u64 = 0;
        let mut pool_device = if receiver_is_cuda {
            receiver_device_idx
        } else {
            sender_device_idx
        };

        // Tracks whether the GPU pool buffer + IPC handle were successfully set
        // up.  A CUDA receiver's shmem is header-only, so without the handle the
        // pool is unusable — we fail registration rather than hand back a
        // permanently-broken pool.
        let mut ipc_written = false;

        if receiver_is_cuda
            && !cross_machine
            && !ipc_written
            && let Ok(helpers) = get_cuda_helpers(py)
        {
            let bound = helpers.bind(py);

            // Enable P2P for the sender/receiver pair before any IPC operations.
            // Gate on a CUDA source: for a CPU source sender_device_idx defaults
            // to 0, and enabling a spurious GPU0↔receiver P2P pair creates an
            // unnecessary CUDA context.  CPU-source registration snapshots the
            // ambient device in the else branch below — running _set_cuda_device
            // first would clobber the original value saved there.
            if is_cuda {
                let _ = bound
                    .call_method1("_ensure_p2p_pair", (sender_device_idx, receiver_device_idx));
                let _ = bound.call_method1("_set_cuda_device", (sender_device_idx,));
            }

            // Resolve transport path.  classify_transport encodes the full
            // 2³ decision matrix (pure, CI-tested); here we only need the
            // single GPU-runtime-dependent input (p2p_available).
            let p2p_available: bool = cross_device
                && bound
                    .call_method1("_can_access_peer", (sender_device_idx, receiver_device_idx))
                    .and_then(|r| r.extract::<bool>())
                    .unwrap_or(false);
            let transport_path = classify_transport(
                sender_device_idx,
                receiver_device_idx,
                p2p_available,
                is_cuda,
            );
            let use_transit = transport_path == TransportPath::HostStagingTransit;

            let gpu_ptr: Option<u64> = if is_cuda {
                if use_transit {
                    // Allocate pool buffer on receiver's GPU so the
                    // receiver can import the IPC handle on its own device.
                    // _transit_copy internally saves/restores the caller's
                    // device, so an explicit restore is unnecessary here.
                    let _ = bound.call_method1("_set_cuda_device", (receiver_device_idx,));
                    let dst: u64 = bound
                        .call_method1("_get_gpu_buf", (pool_counter, size))
                        .and_then(|r| r.extract::<u64>())
                        .unwrap_or(0);
                    // Switch back to sender device.
                    let _ = bound.call_method1("_set_cuda_device", (sender_device_idx,));
                    if dst != 0 {
                        // Allocate CPU page-locked transit buffer.
                        let tp: u64 = bound
                            .call_method1("_alloc_transit", (size,))
                            .and_then(|r| r.extract::<u64>())
                            .unwrap_or(0);
                        if tp != 0 {
                            // Copy via transit: sender GPU → CPU → receiver GPU.
                            let ok: bool = bound
                                .call_method1(
                                    "_transit_copy",
                                    (
                                        ptr_val,
                                        sender_device_idx,
                                        tp,
                                        dst,
                                        receiver_device_idx,
                                        size,
                                    ),
                                )
                                .and_then(|r| r.extract::<bool>())
                                .unwrap_or(false);
                            if ok {
                                transit_ptr = tp;
                                pool_device = receiver_device_idx;
                                Some(dst)
                            } else {
                                // _transit_copy failed: free both buffers
                                let _ = bound.call_method1("_free_transit", (tp,));
                                let _ = bound.call_method1("_free_gpu_buf", (pool_counter,));
                                None
                            }
                        } else {
                            // transit alloc failed: free GPU buffer
                            let _ = bound.call_method1("_free_gpu_buf", (pool_counter,));
                            None
                        }
                    } else {
                        None
                    }
                    // Both branches start from sender_device_idx (restored
                    // after _ensure_p2p_pair above).  _transit_copy does its
                    // own internal save/restore; the same-device branch below
                    // saves/restores explicitly so later cudaMalloc calls land
                    // on the right GPU.
                } else {
                    // Same-device or P2P available: allocate on sender device.
                    // Save the current device and restore before returning
                    // so later cudaMalloc calls land on the right GPU.
                    let saved_dev: i32 = bound
                        .call_method0("_get_cuda_device")
                        .and_then(|r| r.extract::<i32>())
                        .unwrap_or(0);
                    let _ = bound.call_method1("_set_cuda_device", (sender_device_idx,));
                    let result = {
                        let dst: u64 = bound
                            .call_method1("_get_gpu_buf", (pool_counter, size))
                            .and_then(|r| r.extract::<u64>())
                            .unwrap_or(0);
                        if dst != 0 {
                            // Only export a handle if the DtoD copy succeeded —
                            // otherwise the receiver would import uninitialised memory.
                            if bound
                                .call_method1("_cuda_memcpy", (dst, ptr_val, size, 3u32))
                                .is_ok()
                            {
                                Some(dst)
                            } else {
                                let _ = bound.call_method1("_free_gpu_buf", (pool_counter,));
                                None
                            }
                        } else {
                            None
                        }
                    };
                    let _ = bound.call_method1("_set_cuda_device", (saved_dev,));
                    result
                }
            } else {
                // CPU source → GPU pool: switch to the receiver's device
                // so cudaMalloc in dma_copy → _get_gpu_buf lands on the
                // correct GPU (matching the GPU-source branches).
                let saved_dev: i32 = bound
                    .call_method0("_get_cuda_device")
                    .and_then(|r| r.extract::<i32>())
                    .unwrap_or(0);
                let _ = bound.call_method1("_set_cuda_device", (receiver_device_idx,));
                let result = bound
                    .call_method1("dma_copy", (ptr_val, size, pool_counter, !is_pinned))
                    .and_then(|r| r.extract::<u64>())
                    .ok();
                let _ = bound.call_method1("_set_cuda_device", (saved_dev,));
                result
            };

            if let Some(gpu_ptr) = gpu_ptr
                && let Ok(handle) = bound
                    .call_method1("_ipc_export", (gpu_ptr,))
                    .and_then(|r| r.extract::<Vec<u8>>())
                && handle.len() == 64
            {
                unsafe {
                    // Write IPC handle into DORADMA reserved area [32..96)
                    std::ptr::copy_nonoverlapping(handle.as_ptr(), shmem_ptr.add(32), 64);
                    // ipc_present flag at byte 24
                    std::ptr::write(shmem_ptr.add(24) as *mut u64, 1u64);
                }
                ipc_written = true;
            }
        }

        // A CUDA receiver's shmem is header-only and useless without the IPC
        // handle.  If GPU-pool setup failed (or the CUDA helpers were
        // unavailable), fail registration instead of returning a pool that
        // every later write/read would silently reject.  Reclaim the shmem
        // segment on the way out (it was created with owner=false).
        if receiver_is_cuda && !cross_machine && !ipc_written {
            // The GPU pool buffer (and, on the transit path, the page-locked
            // host transit buffer) were allocated before the IPC export, which
            // failed.  Free them before bailing — otherwise they leak for the
            // life of the process since no PoolSlot was stored to track them.
            if let Ok(helpers) = get_cuda_helpers(py) {
                let bound = helpers.bind(py);
                let _ = bound.call_method1("_free_gpu_buf", (pool_counter,));
                if transit_ptr != 0 {
                    let _ = bound.call_method1("_free_transit", (transit_ptr,));
                }
            }
            shmem.set_owner(true);
            eyre::bail!(
                "[{}] register_tensor_pool: failed to set up GPU pool buffer / IPC handle for CUDA receiver `{}`",
                self.node_id,
                tensor_device
            );
        }

        // Seqlock: increment generation to even (write-complete)
        unsafe {
            let gen_ptr = shmem_ptr.add(96) as *mut u64;
            let old_gen = std::ptr::read_volatile(gen_ptr);
            std::ptr::write_volatile(gen_ptr, old_gen + 1);
            std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        }

        // Store shmem in pool (keep alive)
        {
            let mut pool = PINNED_POOL.lock().unwrap_or_else(|e| e.into_inner());
            pool.insert(
                pool_counter,
                PoolSlot {
                    _shmem: shmem,
                    base: shmem_ptr as u64,
                    size: total_size,
                    is_pinned,
                    transit_ptr,
                    pool_device,
                },
            );
        }

        // Persist transit metadata so the write fast path can recover
        // transit_ptr / pool_device on a PINNED_POOL cache-miss.
        if transit_ptr != 0 || pool_device != 0 {
            TRANSIT_META
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(pool_counter, (transit_ptr, pool_device));
        }

        let buffer_id = format!("pool_{}_{}", self.node_id, pool_counter);

        // Register with daemon for lifecycle tracking
        {
            let hlc = dora_node_api::dora_core::uhlc::HLC::default();
            let ts = hlc.new_timestamp();
            let mut params = dora_node_api::MetadataParameters::new();
            params.insert(
                "ptr".to_string(),
                dora_node_api::Parameter::Integer(ptr_val as i64),
            );
            params.insert(
                "size".to_string(),
                dora_node_api::Parameter::Integer(size as i64),
            );
            params.insert("dtype".to_string(), dora_node_api::Parameter::String(dtype));
            params.insert(
                "shape".to_string(),
                dora_node_api::Parameter::ListInt(shape_list),
            );
            params.insert(
                "shared_memory_name".to_string(),
                dora_node_api::Parameter::String(shmem_name),
            );
            params.insert(
                "is_pinned".to_string(),
                dora_node_api::Parameter::Bool(is_pinned),
            );
            params.insert(
                "pinned_type".to_string(),
                dora_node_api::Parameter::String(pinned_type.to_string()),
            );
            params.insert(
                "ipc_present".to_string(),
                dora_node_api::Parameter::Bool(ipc_written),
            );
            params.insert(
                "buffer_id".to_string(),
                dora_node_api::Parameter::String(buffer_id.clone()),
            );

            let meta = dora_node_api::Metadata::from_parameters(ts, params);
            if let Err(e) = crate::seam::store(self.node, &buffer_id, &meta) {
                tracing::warn!("[{}] failed to register tensor pool: {:#}", self.node_id, e);
            }
        }

        let buffer_id_array = arrow::array::StringArray::from(vec![buffer_id]);
        let buf_py: Py<PyAny> = buffer_id_array.to_data().to_pyarrow(py)?.unbind();
        Ok(buf_py)
    }

    /// Write tensor data to an existing tensor pool.
    ///
    /// Overwrites the data region of a previously-registered pool without
    /// re-registering, enabling memory reuse across iterations.
    ///
    /// # Concurrency / safety
    ///
    /// **This is a non-blocking overwrite.** The writer must not call this
    /// while a receiver is consuming the previous tensor. Data-byte
    /// consistency relies entirely on callers honoring a **turn-based
    /// discipline**: wait for the receiver to signal completion (e.g. via
    /// a `next_require` round-trip) before writing the next frame.
    ///
    /// The seqlock at header offset 96 detects in-flight overwrites so the
    /// reader can retry, but it does **not** prevent the overwrite itself.
    /// A writer that ignores the turn-based contract will produce torn
    /// (partially updated) data at the consumer.
    ///
    /// The bundled `examples/tensor-pool/` dataflows demonstrate correct
    /// turn-based usage: the sender writes, outputs the pool ID, and waits
    /// for the next input event before writing again.
    pub fn write_tensor_pool(
        &mut self,
        tensor_pool_id: Py<PyAny>,
        tensor_info: &Bound<'_, PyDict>,
        py: Python,
    ) -> eyre::Result<()> {
        let buffer_id = parse_tensor_pool_id(tensor_pool_id, py)?;

        let ptr_val: u64 = tensor_info
            .get_item("ptr")?
            .ok_or_else(|| eyre::eyre!("missing ptr"))?
            .extract()?;
        let size: usize = tensor_info
            .get_item("size")?
            .ok_or_else(|| eyre::eyre!("missing size"))?
            .extract()?;
        let tensor_device: String = tensor_info
            .get_item("device")?
            .ok_or_else(|| eyre::eyre!("missing device"))?
            .extract()?;
        let is_cuda = tensor_device.starts_with("cuda");

        {
            let freed = FREED_POOL_IDS.lock().unwrap_or_else(|e| e.into_inner());
            if freed.contains(&buffer_id) {
                warn_missing_tensor_pool(&self.node_id, "write", &buffer_id);
                return Ok(());
            }
        }

        // Auto-select pinning based on tensor size (25 MiB threshold).
        // Shared by cache-miss PoolSlot construction and slow-path dma_copy;
        // cache-hit reuses the slot's stored is_pinned.
        let auto_pin = should_pin(is_cuda, size);

        // Fast path: pool_ format -> DORADMA
        if buffer_id.starts_with("pool_") {
            // Extract counter from the last underscore segment — node_id
            // may legitimately contain underscores.
            if let Some((_, counter_str)) = buffer_id.rsplit_once('_')
                && let Ok(counter) = counter_str.parse::<u64>()
            {
                // Try PINNED_POOL cache first to avoid per-iteration mmap/munmap.
                // register_tensor_pool already stored the Shmem here; taking it
                // prevents munmap, and storing it back keeps the mapping alive.
                let pool_slot = {
                    PINNED_POOL
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .remove(&counter)
                };

                // Both cache-hit and cache-miss produce a PoolSlot that is
                // stored back into PINNED_POOL after the write — this keeps
                // the shmem mapping alive for the duration of the data copy.
                let (shmem_ptr, shmem_capacity, mut store_back, is_pinned) =
                    if let Some(mut slot_data) = pool_slot {
                        // Cache hit: reuse the persistent mapping (no mmap).
                        // Recompute is_pinned from the current tensor size
                        // so the auto-selection reflects each write's payload.
                        let cap = slot_data.size;
                        slot_data.is_pinned = auto_pin;
                        let pinned = auto_pin;
                        (slot_data.base as *mut u8, cap, Some(slot_data), pinned)
                    } else {
                        // Cache miss: open via ShmemConf, wrap immediately
                        // so the mapping stays alive until post-write re-insert.
                        let shmem_name = {
                            let machine = std::env::var("DORA_MACHINE_ID").unwrap_or_default();
                            if machine.is_empty() {
                                format!(
                                    "dora_pool_{}_{}_{}",
                                    self.dataflow_id, self.node_id, counter
                                )
                            } else {
                                format!(
                                    "dora_pool_{}_{}_{}_{}",
                                    machine, self.dataflow_id, self.node_id, counter
                                )
                            }
                        };
                        match ShmemConf::new().os_id(&shmem_name).open() {
                            Ok(shmem) => {
                                let cap = shmem.len();
                                let base = shmem.as_ptr() as u64;
                                let slot = PoolSlot {
                                    _shmem: shmem,
                                    base,
                                    size: cap,
                                    is_pinned: auto_pin,
                                    transit_ptr: 0,
                                    pool_device: 0,
                                };
                                (base as *mut u8, cap, Some(slot), auto_pin)
                            }
                            Err(_) => (std::ptr::null_mut(), 0, None, false),
                        }
                    };

                if !shmem_ptr.is_null() {
                    // Guard against truncated segments before any
                    // pointer arithmetic (mirrors slow-path + read guards).
                    if shmem_capacity < DORADMA_HEADER_SIZE {
                        if let Some(slot_data) = store_back {
                            PINNED_POOL
                                .lock()
                                .unwrap_or_else(|e| e.into_inner())
                                .insert(counter, slot_data);
                        }
                        return Ok(());
                    }
                    let magic = unsafe { std::slice::from_raw_parts(shmem_ptr, 8) };
                    if magic == DORADMA_MAGIC {
                        let data_offset = unsafe { read_header_u64(shmem_ptr.add(16)) as usize };

                        // Check if this pool has GPU DMA path enabled
                        let ipc_present =
                            unsafe { std::ptr::read(shmem_ptr.add(24) as *const u64) };

                        // Validate write size against pool capacity.  The
                        // `size == 0` guard always applies.  For GPU-buffer pools
                        // (ipc_present == 1) the shmem data region is unused, so
                        // its capacity is irrelevant — the GPU-buffer helpers
                        // validate `size` against the actual GPU allocation.
                        if size == 0
                            || (ipc_present != 1
                                && size > shmem_capacity.saturating_sub(data_offset))
                        {
                            tracing::warn!(
                                "[{}] write_tensor_pool: size {} exceeds available pool capacity (data_offset={}, total={}), operation aborted",
                                self.node_id,
                                size,
                                data_offset,
                                shmem_capacity
                            );
                            // Store back to PINNED_POOL to keep shmem alive
                            if let Some(slot_data) = store_back {
                                PINNED_POOL
                                    .lock()
                                    .unwrap_or_else(|e| e.into_inner())
                                    .insert(counter, slot_data);
                            }
                            return Ok(());
                        }

                        if ipc_present == 1 && !is_cuda {
                            // Seqlock: begin (noop if still odd from a
                            // previous failed copy).
                            let gen_ptr = unsafe { shmem_ptr.add(96) as *mut u64 };
                            let pre_write_gen = unsafe { seqlock_begin_if_even(gen_ptr) };
                            let mut copy_ok = true;
                            if let Ok(helpers) = get_cuda_helpers(py) {
                                let bound = helpers.bind(py);
                                if let Err(e) = bound
                                    .call_method1("dma_copy", (ptr_val, size, counter, !is_pinned))
                                {
                                    copy_ok = false;
                                    tracing::error!(
                                        "[{}] write_tensor_pool: DMA copy failed: {}",
                                        self.node_id,
                                        e
                                    );
                                }
                            } else {
                                copy_ok = false;
                            }
                            if copy_ok {
                                // Publish: gen was odd (in-progress), flip to even.
                                unsafe {
                                    seqlock_end(gen_ptr, pre_write_gen, true);
                                }
                            } else {
                                // GPU in-place write to the live IPC buffer: a
                                // partial copy cannot be rolled back to a clean
                                // previous frame, so leave gen odd.  The reader
                                // retries until the next successful write.
                            }
                            if !copy_ok {
                                // Re-insert the slot so free_tensor_pool
                                // can clean up the GPU buffer and transit
                                // allocation (mirrors the is_cuda branch).
                                if let Some(slot_data) = store_back.take() {
                                    PINNED_POOL
                                        .lock()
                                        .unwrap_or_else(|e| e.into_inner())
                                        .insert(counter, slot_data);
                                }
                                return Err(eyre::eyre!(
                                    "[{}] write_tensor_pool: DMA copy failed",
                                    self.node_id
                                ));
                            }
                        } else if is_cuda {
                            // Seqlock: begin (noop if still odd from a
                            // previous failed copy).
                            let gen_ptr = unsafe { shmem_ptr.add(96) as *mut u64 };
                            let pre_write_gen = unsafe { seqlock_begin_if_even(gen_ptr) };
                            let mut copy_ok = true;
                            if let Ok(helpers) = get_cuda_helpers(py) {
                                let bound = helpers.bind(py);
                                // Resolve transit metadata: cache-hit from PoolSlot,
                                // cache-miss from TRANSIT_META (populated during registration).
                                let mut transit_ptr =
                                    store_back.as_ref().map_or(0, |s| s.transit_ptr);
                                let transit_from_cache;
                                if transit_ptr == 0 {
                                    // Cache-miss fallback: TRANSIT_META survives
                                    // PINNED_POOL eviction so the write fast path
                                    // always knows whether a transit buffer exists.
                                    let meta =
                                        TRANSIT_META.lock().unwrap_or_else(|e| e.into_inner());
                                    if let Some(&(tp, _pd)) = meta.get(&counter) {
                                        transit_ptr = tp;
                                        transit_from_cache = true;
                                    } else {
                                        transit_from_cache = false;
                                    }
                                } else {
                                    transit_from_cache = false;
                                }
                                let pool_dev = store_back.as_ref().map_or(0, |s| s.pool_device);
                                let write_path = classify_write_path(
                                    ipc_present,
                                    /*is_cuda=*/ true,
                                    transit_ptr,
                                );
                                let res = match write_path {
                                    WritePath::GpuToGpuPoolTransit => {
                                        // Recover pool_device from TRANSIT_META on
                                        // cache-miss; otherwise use the PoolSlot value.
                                        let pool_dev = if transit_from_cache {
                                            TRANSIT_META
                                                .lock()
                                                .unwrap_or_else(|e| e.into_inner())
                                                .get(&counter)
                                                .copied()
                                                .map(|(_tp, pd)| pd)
                                                .unwrap_or(pool_dev)
                                        } else {
                                            pool_dev
                                        };
                                        let sender_dev = tensor_device
                                            .strip_prefix("cuda")
                                            .and_then(|d| d.strip_prefix(':'))
                                            .and_then(|d| d.parse::<i32>().ok())
                                            .unwrap_or(0);
                                        bound
                                            .call_method1(
                                                "_transit_copy_gpu_buf",
                                                (
                                                    counter,
                                                    ptr_val,
                                                    sender_dev,
                                                    transit_ptr,
                                                    pool_dev,
                                                    size,
                                                ),
                                            )
                                            .map(|_| ())
                                    }
                                    WritePath::GpuToGpuPoolDtoD => bound
                                        .call_method1(
                                            "_cuda_memcpy_gpu_buf",
                                            (counter, ptr_val, size),
                                        )
                                        .map(|_| ()),
                                    _ => {
                                        // GpuToShmem: ipc_present ≠ 1,
                                        // copy to shared-memory data region.
                                        bound
                                            .call_method1(
                                                "_cuda_memcpy",
                                                (
                                                    shmem_ptr as u64 + data_offset as u64,
                                                    ptr_val,
                                                    size,
                                                    2u32,
                                                ),
                                            )
                                            .map(|_| ())
                                    }
                                };
                                if let Err(e) = res {
                                    copy_ok = false;
                                    tracing::error!(
                                        "[{}] write_tensor_pool: GPU pool copy failed: {}",
                                        self.node_id,
                                        e
                                    );
                                }
                            } else {
                                copy_ok = false;
                            }
                            if copy_ok {
                                // Publish: gen was odd (in-progress), flip to even.
                                unsafe {
                                    seqlock_end(gen_ptr, pre_write_gen, true);
                                }
                            } else {
                                // GPU in-place write to the live IPC buffer: a
                                // partial copy cannot be rolled back to a clean
                                // previous frame, so leave gen odd.  The reader
                                // retries until the next successful write.
                            }
                            if !copy_ok {
                                if let Some(slot_data) = store_back.take() {
                                    PINNED_POOL
                                        .lock()
                                        .unwrap_or_else(|e| e.into_inner())
                                        .insert(counter, slot_data);
                                }
                                return Err(eyre::eyre!(
                                    "[{}] write_tensor_pool: GPU pool copy failed",
                                    self.node_id
                                ));
                            }
                        } else {
                            // Seqlock: begin (noop if still odd from a
                            // previous failed GPU-to-shmem copy).
                            let gen_ptr = unsafe { shmem_ptr.add(96) as *mut u64 };
                            let pre_write_gen = unsafe { seqlock_begin_if_even(gen_ptr) };
                            unsafe {
                                std::ptr::copy_nonoverlapping(
                                    ptr_val as *const u8,
                                    shmem_ptr.add(data_offset),
                                    size,
                                );
                            }
                            // Publish: gen was odd (in-progress), flip to even.
                            // The shmem data region is not shared with the reader
                            // through IPC, so a copy failure means segfault — the
                            // process is dead before reaching here.  No rollback needed.
                            unsafe {
                                seqlock_end(gen_ptr, pre_write_gen, true);
                            }
                        }

                        // Store back to PINNED_POOL to keep shmem alive
                        if let Some(slot_data) = store_back {
                            PINNED_POOL
                                .lock()
                                .unwrap_or_else(|e| e.into_inner())
                                .insert(counter, slot_data);
                        }

                        // Cross-machine: push the frame through the daemon
                        // so the mirror pool is updated in place. GPU pools
                        // without an IPC handle (cross-machine: data region
                        // staged via DtoH) push; same-machine GPU pools
                        // (ipc_present == 1) hold their data in the IPC
                        // buffer — skip. Pools whose reader confirmed
                        // same-host direct access (or that have no mirror at
                        // all) skip too — the reader opens this segment
                        // directly; the push would only feed a mirror nobody
                        // reads.
                        if should_push_mirror(&buffer_id, ipc_present) {
                            self.push_mirror_update(&buffer_id, size, "write_tensor_pool");
                        }

                        return Ok(());
                    }
                }
            }
        }

        // Slow path: query daemon for pool metadata
        match crate::seam::load_metadata(self.node, &buffer_id, false) {
            Ok(metadata) => {
                let shmem_name = metadata.parameters.get("shared_memory_name").and_then(|p| {
                    if let Parameter::String(s) = p {
                        Some(s.clone())
                    } else {
                        None
                    }
                });

                if let Some(ref name) = shmem_name
                    && let Ok(shmem) = ShmemConf::new().os_id(name).open()
                {
                    // Mirror fast-path guard: reject segments smaller
                    // than the header before any pointer arithmetic.
                    if shmem.len() < DORADMA_HEADER_SIZE {
                        return Ok(());
                    }
                    let shmem_ptr = shmem.as_ptr();

                    let magic = unsafe { std::slice::from_raw_parts(shmem_ptr, 8) };
                    if magic == DORADMA_MAGIC {
                        let data_offset = unsafe { read_header_u64(shmem_ptr.add(16)) as usize };

                        // Check if this pool has GPU DMA path enabled
                        let ipc_present =
                            unsafe { std::ptr::read(shmem_ptr.add(24) as *const u64) };

                        // Validate write size against pool capacity.  The
                        // `size == 0` guard always applies; the shmem-region
                        // capacity check is skipped for GPU-buffer pools
                        // (ipc_present == 1), which the GPU helpers validate.
                        let shmem_len = shmem.len();
                        if size == 0
                            || (ipc_present != 1 && size > shmem_len.saturating_sub(data_offset))
                        {
                            tracing::warn!(
                                "[{}] write_tensor_pool (slow path): size {} exceeds available pool capacity (data_offset={}, total={}), operation aborted",
                                self.node_id,
                                size,
                                data_offset,
                                shmem_len
                            );
                            return Ok(());
                        }

                        if ipc_present == 1 && !is_cuda {
                            // Extract counter for the DMA slot from buffer_id.
                            let slow_counter = buffer_id
                                .rsplit_once('_')
                                .and_then(|(_, c)| c.parse::<u64>().ok());
                            let gen_ptr = unsafe { shmem_ptr.add(96) as *mut u64 };
                            let pre_write_gen = unsafe { seqlock_begin_if_even(gen_ptr) };
                            let mut copy_ok = true;
                            if let (Ok(helpers), Some(c)) = (get_cuda_helpers(py), slow_counter) {
                                let bound = helpers.bind(py);
                                let slow_no_dma = !auto_pin;
                                if let Err(e) =
                                    bound.call_method1("dma_copy", (ptr_val, size, c, slow_no_dma))
                                {
                                    copy_ok = false;
                                    tracing::error!(
                                        "[{}] write_tensor_pool (slow path): DMA copy failed: {}",
                                        self.node_id,
                                        e
                                    );
                                }
                            } else {
                                copy_ok = false;
                            }
                            if copy_ok {
                                // Publish: gen was odd (in-progress), flip to even.
                                unsafe {
                                    seqlock_end(gen_ptr, pre_write_gen, true);
                                }
                            } else {
                                // GPU in-place write to the live IPC buffer: a
                                // partial copy cannot be rolled back to a clean
                                // previous frame, so leave gen odd.  The reader
                                // retries until the next successful write.
                            }
                            if !copy_ok {
                                return Err(eyre::eyre!(
                                    "[{}] write_tensor_pool (slow path): DMA copy failed",
                                    self.node_id
                                ));
                            }
                        } else if is_cuda {
                            let gen_ptr = unsafe { shmem_ptr.add(96) as *mut u64 };
                            let pre_write_gen = unsafe { seqlock_begin_if_even(gen_ptr) };
                            let mut copy_ok = true;
                            if let Ok(helpers) = get_cuda_helpers(py) {
                                let bound = helpers.bind(py);
                                // Slow path transit look-up: PINNED_POOL
                                // (contrast fast path which uses store_back).
                                let (transit_ptr, pool_device) = if let Some((_, counter_str)) =
                                    buffer_id.rsplit_once('_')
                                    && let Ok(c) = counter_str.parse::<u64>()
                                {
                                    PINNED_POOL
                                        .lock()
                                        .unwrap_or_else(|e| e.into_inner())
                                        .get(&c)
                                        .map(|s| (s.transit_ptr, s.pool_device))
                                        .unwrap_or((0, 0))
                                } else {
                                    (0, 0)
                                };
                                let write_path = classify_write_path(
                                    ipc_present,
                                    /*is_cuda=*/ true,
                                    transit_ptr,
                                );
                                let res = match write_path {
                                    WritePath::GpuToGpuPoolTransit => {
                                        let sender_dev = tensor_device
                                            .strip_prefix("cuda")
                                            .and_then(|d| d.strip_prefix(':'))
                                            .and_then(|d| d.parse::<i32>().ok())
                                            .unwrap_or(0);
                                        bound
                                            .call_method1(
                                                "_transit_copy_gpu_buf",
                                                (
                                                    buffer_id
                                                        .rsplit_once('_')
                                                        .and_then(|(_, cs)| cs.parse::<u64>().ok())
                                                        .unwrap_or(0),
                                                    ptr_val,
                                                    sender_dev,
                                                    transit_ptr,
                                                    pool_device,
                                                    size,
                                                ),
                                            )
                                            .map(|_| ())
                                    }
                                    WritePath::GpuToGpuPoolDtoD => bound
                                        .call_method1(
                                            "_cuda_memcpy_gpu_buf",
                                            (
                                                buffer_id
                                                    .rsplit_once('_')
                                                    .and_then(|(_, cs)| cs.parse::<u64>().ok())
                                                    .unwrap_or(0),
                                                ptr_val,
                                                size,
                                            ),
                                        )
                                        .map(|_| ()),
                                    _ => {
                                        // GpuToShmem: copy to shmem data region.
                                        bound
                                            .call_method1(
                                                "_cuda_memcpy",
                                                (
                                                    shmem_ptr as u64 + data_offset as u64,
                                                    ptr_val,
                                                    size,
                                                    2u32,
                                                ),
                                            )
                                            .map(|_| ())
                                    }
                                };
                                if let Err(e) = res {
                                    copy_ok = false;
                                    tracing::error!(
                                        "[{}] write_tensor_pool (slow path): GPU pool copy failed: {}",
                                        self.node_id,
                                        e
                                    );
                                }
                            } else {
                                copy_ok = false;
                            }
                            if copy_ok {
                                // Publish: gen was odd (in-progress), flip to even.
                                unsafe {
                                    seqlock_end(gen_ptr, pre_write_gen, true);
                                }
                            } else {
                                // GPU in-place write to the live IPC buffer: a
                                // partial copy cannot be rolled back to a clean
                                // previous frame, so leave gen odd.  The reader
                                // retries until the next successful write.
                            }
                            if !copy_ok {
                                return Err(eyre::eyre!(
                                    "[{}] write_tensor_pool (slow path): GPU pool copy failed",
                                    self.node_id
                                ));
                            }
                        } else {
                            // Seqlock: begin (noop if still odd from a
                            // previous failed GPU-to-shmem copy).
                            let gen_ptr = unsafe { shmem_ptr.add(96) as *mut u64 };
                            let pre_write_gen = unsafe { seqlock_begin_if_even(gen_ptr) };
                            unsafe {
                                std::ptr::copy_nonoverlapping(
                                    ptr_val as *const u8,
                                    shmem_ptr.add(data_offset),
                                    size,
                                );
                            }
                            // Publish: gen was odd (in-progress), flip to even.
                            // The shmem data region is not shared through IPC —
                            // copy_nonoverlapping failure means segfault,
                            // so the process is dead before reaching here.
                            unsafe {
                                seqlock_end(gen_ptr, pre_write_gen, true);
                            }
                        }
                    }
                }
            }
            Err(_) => {
                warn_missing_tensor_pool(&self.node_id, "write", &buffer_id);
            }
        }

        Ok(())
    }

    /// Read tensor info from an existing tensor pool (zero-copy).
    ///
    /// Returns a `tensor_info` dict compatible with `tensor_from_info`.
    /// The returned tensor shares the underlying shared-memory mapping —
    /// no copy is made, so data bytes reflect whatever the writer has most
    /// recently stored.
    ///
    /// # Concurrency / safety
    ///
    /// **The returned tensor is a zero-copy view into shared memory.**
    /// Its data bytes can be overwritten at any time by a concurrent (or
    /// subsequent) [`write_tensor_pool`] on the sender. The seqlock
    /// re-check at end-of-read detects whether an overwrite occurred
    /// mid-consumption, but it is the **caller's responsibility** to
    /// ensure the tensor is not used after the writer is allowed to write
    /// again.
    ///
    /// Correct consumers follow a **turn-based discipline**: read the
    /// pool, consume the tensor fully, then signal the sender (e.g. via
    /// the dataflow graph's `next_require` round-trip) that it is safe to
    /// write the next frame. The bundled `examples/tensor-pool/` dataflows
    /// demonstrate this pattern.
    pub fn read_tensor_pool(
        &mut self,
        tensor_pool_id: Py<PyAny>,
        py: Python,
    ) -> eyre::Result<Py<PyAny>> {
        let buffer_id = parse_tensor_pool_id(tensor_pool_id, py)?;

        // Populate the trusted GPU buffer size from daemon metadata
        // on the first read.  Subsequent reads (if any) reuse the
        // cached entry — the daemon query runs at most once per pool.
        {
            let trusted = GPU_BUF_SIZES.lock().unwrap_or_else(|e| e.into_inner());
            if !trusted.contains_key(&buffer_id) {
                drop(trusted);
                if let Ok(metadata) = crate::seam::load_metadata(self.node, &buffer_id, false)
                    && let Some(size) = metadata.parameters.get("size").and_then(|p| {
                        if let Parameter::Integer(v) = p {
                            Some(*v)
                        } else {
                            None
                        }
                    })
                {
                    GPU_BUF_SIZES
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .insert(buffer_id.clone(), size as u64);
                }
            }
        }

        // Fast path: DORADMA header read with daemon-trusted size validation.
        if buffer_id.starts_with("pool_") {
            // Retry on transient failures (odd seqlock, shmem not yet
            // mapped) so a concurrent writer doesn't cause a hard error.
            // Time-bounded: a GPU copy (cudaMemcpy + synchronize) takes
            // milliseconds, so we wait up to 500ms total with 1ms sleeps
            // between attempts.
            let deadline = std::time::Instant::now()
                .checked_add(std::time::Duration::from_millis(500))
                .unwrap_or(std::time::Instant::now());
            loop {
                match self.try_doradma_read(&buffer_id, py) {
                    Ok(Some(result)) => return Ok(result),
                    Ok(None) if std::time::Instant::now() < deadline => {
                        // Transient — yield the GIL and sleep so the
                        // writer can complete its copy+sync.
                        py.detach(|| {
                            std::thread::sleep(std::time::Duration::from_millis(1));
                        });
                        continue;
                    }
                    Ok(None) => break,
                    Err(e) => {
                        warn_missing_tensor_pool(&self.node_id, "read", &buffer_id);
                        eyre::bail!("tensor pool {}: fast path failed: {}", buffer_id, e);
                    }
                }
            }
            // Retries exhausted — fall back to the daemon for CPU pools.
            if let Ok(metadata) = crate::seam::load_metadata(self.node, &buffer_id, false) {
                let size = metadata
                    .parameters
                    .get("size")
                    .and_then(|p| {
                        if let Parameter::Integer(v) = p {
                            Some(*v)
                        } else {
                            None
                        }
                    })
                    .unwrap_or(0);
                let dtype = metadata
                    .parameters
                    .get("dtype")
                    .and_then(|p| {
                        if let Parameter::String(s) = p {
                            Some(s.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let shape = metadata
                    .parameters
                    .get("shape")
                    .and_then(|p| {
                        if let Parameter::ListInt(v) = p {
                            Some(v.clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                // Only for CPU pools — GPU pools need IPC import.
                let ipc_present = metadata
                    .parameters
                    .get("ipc_present")
                    .and_then(|p| {
                        if let Parameter::Bool(v) = p {
                            Some(*v)
                        } else {
                            None
                        }
                    })
                    .unwrap_or(false);
                if ipc_present {
                    warn_missing_tensor_pool(&self.node_id, "read", &buffer_id);
                    eyre::bail!(
                        "tensor pool {}: fast path retries exhausted for GPU pool \
                         (daemon fallback cannot provide a GPU pointer)",
                        buffer_id
                    );
                }
                let shmem_name = metadata.parameters.get("shared_memory_name").and_then(|p| {
                    if let Parameter::String(s) = p {
                        Some(s.clone())
                    } else {
                        None
                    }
                });
                if let Some(ref name) = shmem_name
                    && let Ok(shmem) = ShmemConf::new().os_id(name).open()
                    && shmem.len() >= DORADMA_HEADER_SIZE
                {
                    let shmem_ptr = shmem.as_ptr();
                    let magic = unsafe { std::slice::from_raw_parts(shmem_ptr, 8) };
                    if magic == DORADMA_MAGIC {
                        let data_offset = unsafe { read_header_u64(shmem_ptr.add(16)) as usize };
                        // Mirror fast-path bounds check.
                        if data_offset > shmem.len()
                            || (size as usize) > shmem.len().saturating_sub(data_offset)
                        {
                            warn_missing_tensor_pool(&self.node_id, "read", &buffer_id);
                            eyre::bail!(
                                "tensor pool {}: header bounds exceeded: \
                                 data_offset {} + size {} > shmem_len {}",
                                buffer_id,
                                data_offset,
                                size,
                                shmem.len()
                            );
                        }
                        // Seqlock: reject a torn mid-write frame.  The
                        // fallback is reached when the fast path retries
                        // are exhausted, typically because the generation
                        // is stuck odd (crashed writer).
                        let read_gen =
                            unsafe { std::ptr::read_volatile(shmem_ptr.add(96) as *const u64) };
                        if read_gen % 2 != 0 {
                            warn_missing_tensor_pool(&self.node_id, "read", &buffer_id);
                            eyre::bail!(
                                "tensor pool {}: daemon fallback: seqlock write in progress \
                                 (generation={}, odd)",
                                buffer_id,
                                read_gen
                            );
                        }
                        // Cache-hit: use the stored mapping's base so
                        // the pointer stays valid after the fresh shmem
                        // is dropped.  Cache-miss: insert the fresh
                        // mapping and drop the old one (if any).
                        let read_ptr;
                        {
                            let mut cpu_cache =
                                RECV_CPU_SHMEM.lock().unwrap_or_else(|e| e.into_inner());
                            if let Some(cached) = cpu_cache.get(&buffer_id) {
                                read_ptr = (cached.base + data_offset as u64) as i64;
                            } else {
                                let base = shmem_ptr as u64;
                                read_ptr = (base + data_offset as u64) as i64;
                                cpu_cache.entry(buffer_id.clone()).or_insert(RecvCpuSlot {
                                    _shmem: shmem,
                                    base,
                                });
                            }
                        }
                        let dict = PyDict::new(py);
                        dict.set_item("ptr", read_ptr)?;
                        dict.set_item("size", size)?;
                        dict.set_item("dtype", dtype)?;
                        dict.set_item("shape", shape)?;
                        dict.set_item("device", "cpu")?;
                        return Ok(dict.into());
                    }
                }
            }
            warn_missing_tensor_pool(&self.node_id, "read", &buffer_id);
            eyre::bail!(
                "tensor pool {}: fast path retries exhausted — pool not ready after 500ms",
                buffer_id
            );
        }

        warn_missing_tensor_pool(&self.node_id, "read", &buffer_id);
        eyre::bail!("tensor pool {} not found", buffer_id);
    }

    /// Free a tensor pool.
    pub fn free_tensor_pool(&mut self, tensor_pool_id: Py<PyAny>, py: Python) -> eyre::Result<()> {
        let buffer_id = parse_tensor_pool_id(tensor_pool_id, py)?;

        // Cross-machine release: ask the daemon to tear down the mirror on
        // the target machine (no-op for local pools — the daemon logs the
        // missing entry and returns an error we absorb below).  The local
        // metadata is removed by the `drop_key` seam right after.
        if let Err(e) = self.node.free_pinned_memory(buffer_id.clone()) {
            tracing::debug!(
                "[{}] free_tensor_pool: daemon release failed for {}: {e}",
                self.node_id,
                buffer_id
            );
        }

        if crate::seam::drop_key(self.node, &buffer_id).is_err() {
            warn_missing_tensor_pool(&self.node_id, "release", &buffer_id);
        }

        // Clean up sender-side pinned pool mapping (Shmem + CUDA host register).
        // Without this, each register->write->free cycle leaks one Shmem mapping
        // and one cudaHostRegister pinned region for the process lifetime.
        // PINNED_POOL is sender-side (per-process), so bare counter is sufficient.
        {
            let counter = buffer_id
                .strip_prefix("pool_")
                .and_then(|s| s.rsplit_once('_').map(|(_, c)| c))
                .and_then(|c| c.parse::<u64>().ok());
            if let Some(c) = counter
                && let Some(slot) = PINNED_POOL
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&c)
            {
                if let Ok(helpers) = get_cuda_helpers(py) {
                    let bound = helpers.bind(py);
                    let _ = bound.call_method1("_unregister_host", (slot.base,));
                    let _ = bound.call_method1("_free_gpu_buf", (c,));
                    if slot.transit_ptr != 0 {
                        let _ = bound.call_method1("_free_transit", (slot.transit_ptr,));
                    }
                }
                // Remove transit metadata regardless of whether CUDA helpers
                // are available — a missing _free_transit is a leak, but a stale
                // TRANSIT_META entry is a correctness bug on re-registration.
                TRANSIT_META
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&c);
            }
            // PoolSlot dropped here -> Shmem unmapped
        }

        // Clean up receiver-side caches so the shmem mappings are released.
        // Keyed by full buffer_id (namespaced) to correctly handle
        // multiple sender nodes with the same per-process counter.
        {
            // Close GPU IPC handle before dropping the cache entry.
            if let Some(slot) = RECV_GPU_VA
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .remove(&buffer_id)
            {
                if slot.gpu_buf != 0 {
                    if let Ok(helpers) = get_cuda_helpers(py) {
                        let bound = helpers.bind(py);
                        let _ = bound.call_method1("_ipc_close", (slot.gpu_buf,));
                    }
                } else if slot.gpu_va != 0 {
                    // Host-registered mapping (effective_as_cuda branch):
                    // must cudaHostUnregister before munmap.  _unregister_host
                    // requires the original host pointer (shmem base), not the
                    // device VA returned by cudaHostGetDevicePointer.
                    if let Ok(helpers) = get_cuda_helpers(py) {
                        let bound = helpers.bind(py);
                        let _ = bound.call_method1("_unregister_host", (slot.host_base,));
                    }
                }
                // slot._shmem drops here -> munmap
            }
            // Cross-machine GPU pool staging cache: unpin the mirror and
            // free the pooled GPU buffer BEFORE dropping the mapping
            // (unregister must precede munmap).
            if let Some(slot) = RECV_GPU_HTOD
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .remove(&buffer_id)
                && let Ok(helpers) = get_cuda_helpers(py)
            {
                let bound = helpers.bind(py);
                let _ = bound.call_method1("_unregister_host", (slot.host_base,));
                let _ = bound.call_method1("_free_htod_buf", (&buffer_id,));
                // slot._shmem drops here -> munmap
            }
        }
        RECV_CPU_SHMEM
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remove(&buffer_id);
        GPU_BUF_SIZES
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remove(&buffer_id);

        {
            let mut freed = FREED_POOL_IDS.lock().unwrap_or_else(|e| e.into_inner());
            tracing::debug!(
                "[{}] free_tensor_pool: adding {} to FREED_POOL_IDS (set size={})",
                self.node_id,
                buffer_id,
                freed.len()
            );
            freed.insert(buffer_id.clone());
        }

        // Drop the classification-sets entries: pool ids embed a
        // monotonic counter, so register/free cycles would otherwise
        // grow NO_MIRROR_POOLS / DIRECT_POOLS without bound on a
        // long-running node.
        NO_MIRROR_POOLS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remove(&buffer_id);
        DIRECT_POOLS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remove(&buffer_id);

        Ok(())
    }

    /// Push a shared-memory-reference update for `buffer_id` through the
    /// daemon: the payload is empty, the daemon reads the sender's segment
    /// and forwards the frame to the mirror pool on the target machine.
    /// Used by the cross-machine register (initial frame) and write paths.
    fn push_mirror_update(&mut self, buffer_id: &str, size: usize, caller: &str) {
        if let Err(e) = self
            .node
            .write_pinned_memory(buffer_id.to_string(), Vec::new(), size)
        {
            tracing::error!(
                "[{}] {caller}: daemon push failed for {}: {e}",
                self.node_id,
                buffer_id
            );
        }
    }

    // ==================== try_doradma_read ====================

    /// DORADMA fast path for read_tensor_pool: reads metadata directly from
    /// the shmem header, bypassing the daemon for zero-copy metadata retrieval.
    ///
    /// Buffer ID format: `"pool_{node_id}_{counter}"` →
    /// shmem name: `"dora_pool_{dataflow_id}_{node_id}_{counter}"`.
    ///
    /// # Synchronization model
    ///
    /// The seqlock (write_gen at header offset 96) guards **data-byte**
    /// consistency across `write_tensor_pool` overwrites — the end-of-read
    /// generation re-check detects if a write occurred mid-read.  Header
    /// fields (json_len, data_offset) are written once at registration and
    /// never change, so they are not subject to torn-read risk.
    ///
    /// The seqlock does NOT protect the tensor data bytes from being
    /// overwritten while a consumer is iterating the zero-copy tensor.
    /// Callers must enforce a **turn-based** discipline: the writer must
    /// not begin a new `write_tensor_pool` until the receiver has finished
    /// consuming the previous tensor. The example dataflow enforces this
    /// via `next_require` round-trip signaling.
    ///
    /// Returns `Ok(Some(tensor_info_dict))` on success, `Ok(None)` to fall back to daemon.
    fn try_doradma_read(
        &mut self,
        buffer_id: &str,
        py: Python<'_>,
    ) -> eyre::Result<Option<Py<PyAny>>> {
        // Format: "pool_{node_id}_{counter}".
        // Use rsplit to extract the counter from the end — the node_id
        // portion may itself contain underscores (legal in dora node ids).
        let counter: u64 = match buffer_id.rsplit_once('_') {
            Some((_, c)) => match c.parse() {
                Ok(c) => c,
                Err(_) => return Ok(None),
            },
            None => return Ok(None),
        };
        let pool_node_id = buffer_id
            .strip_prefix("pool_")
            .and_then(|s| s.strip_suffix(&format!("_{counter}")))
            .unwrap_or("");

        // Check freed tracking -> if this buffer was freed, fall back to daemon
        {
            let freed = FREED_POOL_IDS.lock().unwrap_or_else(|e| e.into_inner());
            if freed.contains(buffer_id) {
                tracing::debug!(
                    "[{}] try_doradma_read: buffer {} is freed, fallback to daemon",
                    self.node_id,
                    buffer_id
                );
                return Ok(None);
            }
        }

        // Candidate segment names, tried in order: the machine-qualified
        // mirror name first (cross-machine pools resolve to a mirror
        // segment on this host, named `dora_pool_{machine}_{df}_{node}_
        // {counter}`), then the unqualified local name (single-daemon and
        // same-machine pools, and nodes spawned without DORA_MACHINE_ID).
        let mut candidates: Vec<String> = Vec::new();
        if let Ok(machine) = std::env::var("DORA_MACHINE_ID")
            && !machine.is_empty()
        {
            candidates.push(format!(
                "dora_pool_{machine}_{}_{}_{}",
                self.dataflow_id, pool_node_id, counter
            ));
        }
        candidates.push(format!(
            "dora_pool_{}_{}_{}",
            self.dataflow_id, pool_node_id, counter
        ));

        // Open the first readable candidate.
        let mut shmem = None;
        let mut shmem_name = String::new();
        for cand in &candidates {
            if let Ok(s) = ShmemConf::new().os_id(cand).open() {
                shmem = Some(s);
                shmem_name = cand.clone();
                break;
            }
        }
        let Some(shmem) = shmem else {
            return Ok(None);
        };

        let shmem_ptr = shmem.as_ptr();
        let shmem_size = shmem.len();

        // Reject truncated segments: need at least DORADMA_HEADER_SIZE bytes for the header
        if shmem_size < DORADMA_HEADER_SIZE {
            tracing::warn!(
                "[{}] try_doradma_read: shmem size {} < DORADMA_HEADER_SIZE {}, rejecting",
                self.node_id,
                shmem_size,
                DORADMA_HEADER_SIZE,
            );
            return Ok(None);
        }

        // Verify DORADMA magic header

        unsafe {
            let magic = std::slice::from_raw_parts(shmem_ptr, 8);
            if magic != DORADMA_MAGIC {
                return Ok(None);
            }
        }

        // Read header: [magic:8][json_len:8][data_offset:8][reserved:232]
        let json_len = unsafe { read_header_u64(shmem_ptr.add(8)) as usize };
        let data_offset = unsafe { read_header_u64(shmem_ptr.add(16)) as usize };

        // Validate JSON length fits within the segment
        if json_len > shmem_size.saturating_sub(DORADMA_HEADER_SIZE) {
            tracing::warn!(
                "[{}] try_doradma_read: json_len {} exceeds shmem bounds (size={}, header={})",
                self.node_id,
                json_len,
                shmem_size,
                DORADMA_HEADER_SIZE,
            );
            return Ok(None);
        }

        // Read JSON metadata from header
        let json_slice =
            unsafe { std::slice::from_raw_parts(shmem_ptr.add(DORADMA_HEADER_SIZE), json_len) };
        let json_str = match std::str::from_utf8(json_slice) {
            Ok(s) => s,
            Err(_) => return Ok(None),
        };

        // Parse JSON to Python dict
        let metadata_dict: Bound<'_, PyDict> = match py.import("json") {
            Ok(m) => match m.call_method1("loads", (json_str,)) {
                Ok(v) => match v.cast_into::<PyDict>() {
                    Ok(d) => d,
                    Err(_) => return Ok(None),
                },
                Err(_) => return Ok(None),
            },
            Err(_) => return Ok(None),
        };

        let size: usize = match metadata_dict.get_item("size") {
            Ok(Some(v)) => v.extract().unwrap_or(0),
            _ => 0,
        };
        if size == 0 {
            return Ok(None);
        }

        // Same-host direct-read fast path: the mirror's header carries the
        // sender's segment name (`sender_shmem`, written by the mirroring
        // daemon from the RegisterPool event). When that segment is
        // reachable on this host, read it directly — the origin skips the
        // per-frame push for same-host pools (direct == true), so the
        // mirror would otherwise serve stale frames forever.
        let mut active_shmem = shmem;
        let mut active_ptr = shmem_ptr;
        let mut active_size = shmem_size;
        let mut active_data_offset = data_offset;
        let mut active_ipc = unsafe { std::ptr::read(active_ptr.add(24) as *const u64) };
        if let Some(sender_name) = metadata_dict
            .get_item("sender_shmem")
            .ok()
            .flatten()
            .and_then(|v| v.extract::<String>().ok())
            && sender_name != shmem_name
            && let Ok(sender) = ShmemConf::new().os_id(&sender_name).open()
            && sender.len() >= DORADMA_HEADER_SIZE
        {
            let sender_ptr = sender.as_ptr();
            let magic = unsafe { std::slice::from_raw_parts(sender_ptr, 8) };
            if magic == DORADMA_MAGIC {
                // Re-read the sender's own header: its JSON (written by
                // the node's register) has a different length than the
                // mirror's, hence a different data_offset. The
                // metadata values (size/dtype/shape) agree — the sender
                // registered the same pool.
                active_shmem = sender;
                active_ptr = active_shmem.as_ptr();
                active_size = active_shmem.len();
                active_data_offset = unsafe { read_header_u64(active_ptr.add(16)) as usize };
                active_ipc = unsafe { std::ptr::read(active_ptr.add(24) as *const u64) };
            }
        }
        // Shadow the original bindings with the active (possibly
        // sender-redirected) segment's values. `active_shmem` keeps the
        // mapping alive for the rest of the read.
        let shmem = active_shmem;
        let shmem_ptr = active_ptr;
        let shmem_size = active_size;
        let data_offset = active_data_offset;
        let ipc_present = active_ipc;

        // Verify data_offset + size fits within shared memory segment.
        // GPU-buffer reads (ipc_present == 1) don't access the shmem data
        // region, so the size check is only required for CPU-receiver paths.
        if ipc_present != 1 {
            // Use saturating operations to guard against corrupted/hostile
            // headers with a near-usize::MAX data_offset (overflow-safe,
            // matching the write-path checks).
            if data_offset > shmem_size || size > shmem_size.saturating_sub(data_offset) {
                tracing::warn!(
                    "[{}] try_doradma_read: data_offset {} + size {} exceeds shmem_size {}",
                    self.node_id,
                    data_offset,
                    size,
                    shmem_size,
                );
                return Ok(None);
            }
        }

        // Auto-detect read path from pinned_type
        let pinned_type: Option<String> = metadata_dict
            .get_item("pinned_type")
            .ok()
            .flatten()
            .and_then(|v| v.extract::<String>().ok());

        let effective_as_cuda = ipc_present == 1 || pinned_type.as_deref() != Some("cpu");

        // Seqlock: read generation, validate stable after data access
        let read_gen = unsafe { std::ptr::read_volatile(shmem_ptr.add(96) as *const u64) };
        if read_gen % 2 != 0 {
            // Writer is in progress — retry (caller falls back to daemon)
            return Ok(None);
        }
        std::sync::atomic::fence(std::sync::atomic::Ordering::Acquire);

        let read_ptr: u64;

        if ipc_present == 1 {
            // GPU DMA pool: import IPC handle once, cache GPU buffer ptr
            read_ptr = {
                let cache = RECV_GPU_VA.lock().unwrap_or_else(|e| e.into_inner());
                match cache.get(buffer_id) {
                    Some(slot_data) if slot_data.gpu_buf != 0 => {
                        // Validate size against the GPU buffer's registered
                        // capacity.  GPU_BUF_SIZES (populated from daemon
                        // metadata) is authoritative; gpu_buf_size (populated
                        // from shmem at first import) is the baseline.
                        let trusted_sizes = {
                            let trusted = GPU_BUF_SIZES.lock().unwrap_or_else(|e| e.into_inner());
                            trusted.get(buffer_id).copied()
                        };
                        if check_capacity_gpu_pool(
                            trusted_sizes,
                            Some(slot_data.gpu_buf_size),
                            size as u64,
                        ) == CapacityCheck::ExceedsTrustedSize
                        {
                            return Ok(None);
                        }
                        slot_data.gpu_buf
                    }
                    _ => {
                        // First IPC import: validate size against
                        // daemon-trusted capacity.  Fail closed —
                        // NoTrustedEntry rejects the import rather than
                        // trusting the world-writable shmem size.
                        let trusted_sizes = {
                            let trusted = GPU_BUF_SIZES.lock().unwrap_or_else(|e| e.into_inner());
                            trusted.get(buffer_id).copied()
                        };
                        match check_capacity_gpu_pool(trusted_sizes, None, size as u64) {
                            CapacityCheck::Ok => {}
                            CapacityCheck::ExceedsTrustedSize | CapacityCheck::NoTrustedEntry => {
                                return Ok(None);
                            }
                        }
                        drop(cache);
                        let handle_bytes =
                            unsafe { std::slice::from_raw_parts(shmem_ptr.add(32), 64) };
                        let helpers = get_cuda_helpers(py)
                            .map_err(|e| eyre::eyre!("get_cuda_helpers: {}", e))?;
                        let bound = helpers.bind(py);
                        let handle_py = PyBytes::new(py, handle_bytes);
                        let gpu_ptr: u64 = bound
                            .call_method1("_ipc_import", (handle_py,))
                            .map_err(|e| eyre::eyre!("_ipc_import: {}", e))?
                            .extract()
                            .map_err(|e| eyre::eyre!("extract gpu_ptr: {}", e))?;
                        let mut cache = RECV_GPU_VA.lock().unwrap_or_else(|e| e.into_inner());
                        cache.insert(
                            buffer_id.to_string(),
                            RecvGpuSlot {
                                _shmem: shmem,
                                gpu_va: 0,
                                gpu_buf: gpu_ptr,
                                host_base: shmem_ptr as u64,
                                // Baseline from shmem — daemon metadata
                                // already populated GPU_BUF_SIZES above.
                                gpu_buf_size: size as u64,
                            },
                        );
                        gpu_ptr
                    }
                }
            };
        } else if effective_as_cuda {
            // Cross-machine GPU pool (or a same-host direct read of one):
            // ipc_present == 0 with a non-"cpu" pinned_type — the data
            // arrives as CPU bytes in the mirror / sender data region
            // (CUDA IPC handles are host-local).  Pin the segment and
            // stage a pooled GPU DRAM buffer via cudaMemcpy HtoD; the
            // returned pointer is a stable DRAM buffer, NOT a view of the
            // shmem (the mirror may be overwritten by the next zenoh frame
            // without corrupting this tensor — relaxes the turn-based
            // discipline on this side).
            read_ptr = {
                // Trust anchor: the daemon-relayed size (GPU_BUF_SIZES,
                // populated at read_tensor_pool entry) bounds the staging
                // allocation; never touch untrusted shmem sizes.
                let trusted = GPU_BUF_SIZES.lock().unwrap_or_else(|e| e.into_inner());
                match check_capacity_gpu_pool(trusted.get(buffer_id).copied(), None, size as u64) {
                    CapacityCheck::Ok => {}
                    _ => return Ok(None), // retry window; see caller
                }
                let cache = RECV_GPU_HTOD.lock().unwrap_or_else(|e| e.into_inner());
                match cache.get(buffer_id) {
                    Some(slot) if slot.gpu_buf_size >= size as u64 => slot.gpu_buf,
                    Some(_) => return Ok(None), // capacity changed (unreachable: register fixes size)
                    None => {
                        drop(cache);
                        let helpers = get_cuda_helpers(py)
                            .map_err(|e| eyre::eyre!("get_cuda_helpers: {}", e))?;
                        let bound = helpers.bind(py);
                        // Pin the whole mirror (covers the data region) so
                        // the HtoD copy runs at DMA bandwidth.  First read
                        // only; idempotent (error 712 tolerated).
                        bound
                            .call_method1("_register_host", (shmem_ptr as u64, shmem_size))
                            .map_err(|e| eyre::eyre!("_register_host: {}", e))?;
                        let gpu_buf: u64 = bound
                            .call_method1("_get_htod_buf", (buffer_id, size))
                            .map_err(|e| eyre::eyre!("_get_htod_buf: {}", e))?
                            .extract()
                            .map_err(|e| eyre::eyre!("extract gpu_buf: {}", e))?;
                        let mut cache = RECV_GPU_HTOD.lock().unwrap_or_else(|e| e.into_inner());
                        cache.insert(
                            buffer_id.to_string(),
                            RecvGpuHtodSlot {
                                _shmem: shmem,
                                host_base: shmem_ptr as u64,
                                gpu_buf,
                                gpu_buf_size: size as u64,
                            },
                        );
                        gpu_buf
                    }
                }
            };
            // HtoD staging: mirror data region → pooled GPU buffer (sync
            // copy, cudaMemcpy kind 1).  Sits between the two seqlock
            // reads: a frame that changes mid-copy fails the re-check and
            // the caller retries — the stale GPU buffer is overwritten on
            // the next read.
            {
                let helpers =
                    get_cuda_helpers(py).map_err(|e| eyre::eyre!("get_cuda_helpers: {}", e))?;
                let bound = helpers.bind(py);
                bound
                    .call_method1(
                        "_cuda_memcpy",
                        (read_ptr, shmem_ptr as u64 + data_offset as u64, size, 1u32),
                    )
                    .map_err(|e| eyre::eyre!("HtoD staging copy: {}", e))?;
            }
        } else {
            // On the first read the fresh mapping is cached; on subsequent
            // reads the fresh mapping is dropped and the returned pointer
            // must use the cached mapping's base (a different mmap address).
            let mut cpu_cache = RECV_CPU_SHMEM.lock().unwrap_or_else(|e| e.into_inner());
            read_ptr = match cpu_cache.get(buffer_id) {
                Some(cached) => cached.base + data_offset as u64,
                None => {
                    let base = shmem_ptr as u64;
                    cpu_cache.insert(
                        buffer_id.to_string(),
                        RecvCpuSlot {
                            _shmem: shmem,
                            base,
                        },
                    );
                    base + data_offset as u64
                }
            };
        }

        // Seqlock: re-read generation — mismatch means data changed during read
        let read_gen2 = unsafe { std::ptr::read_volatile(shmem_ptr.add(96) as *const u64) };
        if read_gen2 != read_gen {
            return Ok(None);
        }

        // Build tensor_info dict
        let dtype: String = metadata_dict
            .get_item("dtype")
            .ok()
            .flatten()
            .and_then(|v| v.extract::<String>().ok())
            .unwrap_or_default();
        let shape: Vec<i64> = metadata_dict
            .get_item("shape")
            .ok()
            .flatten()
            .and_then(|v| v.extract::<Vec<i64>>().ok())
            .unwrap_or_default();
        let device = if effective_as_cuda { "cuda" } else { "cpu" };

        let dict = PyDict::new(py);
        dict.set_item("ptr", read_ptr as i64)?;
        dict.set_item("size", size)?;
        dict.set_item("dtype", dtype)?;
        dict.set_item("shape", shape)?;
        dict.set_item("device", device)?;

        // A successful fast-path read means the pool is alive.  Clear
        // any stale tombstone so that the fast path re-engages after a
        // sender restart re-creates the same buffer_id (FREED_POOL_IDS
        // is per-process and free_tensor_pool inserts there).
        {
            FREED_POOL_IDS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .remove(buffer_id);
        }

        Ok(Some(dict.into()))
    }
}
