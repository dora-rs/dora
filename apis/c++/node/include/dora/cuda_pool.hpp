// Optional CUDA helper for the Dora memory-pool transport.
//
// This header is NOT included by dora-node-api.h and does not itself include
// it. Include this file only from a node that already links the CUDA
// runtime; nothing in the Dora binding depends on CUDA, and a node that never
// includes this file needs no CUDA toolchain — neither does the Rust build
// that generates the bridge.
//
// Why the whole segment and not just the payload: `cudaHostRegister` requires
// a page-aligned address, but a pool's payload starts at
// `256 + padded_metadata_len`, which is only 256-byte aligned. So register
// from the segment's mapping base and offset into it afterwards. The bridge
// hands you exactly those two pieces, already paired so you cannot combine a
// base with the wrong pool's offset:
//
//   - producer side: `pool_shm_base(pool)` + `pool_segment_bytes(pool)` for
//     the base and length, `pool_payload_offset(pool)` for the payload's
//     offset into that mapping.
//   - consumer side: `view_mapping(view, base, bytes)` for the base and
//     length, `view_payload_offset(view, offset)` for the offset.
//
// `view_payload_offset` is a predicate that returns false — leaving `offset`
// untouched — for an `ipc` pool, whose payload lives in device memory rather
// than in this host mapping, and for a pool that has already been freed.
// **Check the return value before using the offset** — on false it is left at
// whatever you initialized it to, and adding that to `m.device` lands in the
// DORADMA header (magic, `json_len`, `data_offset`, the seqlock generation)
// instead of the payload. A kernel writing through a pointer built that way
// corrupts the segment silently; it does not crash.
//
// `pool_payload_offset` needs no such check: a pool you created is never
// `ipc` (registration refuses it) and never freed while you hold it, so it
// always has a payload in its own mapping.
//
// Producer side, once at setup (never per frame — see `map_pool`'s note on
// cost):
//
//   dora::cuda::MappedPool m;
//   if (dora::cuda::map_pool(reinterpret_cast<void *>(pool_shm_base(pool)),
//                             pool_segment_bytes(pool), m)) {
//       void *device_payload =
//           static_cast<char *>(m.device) + pool_payload_offset(pool);
//       // ... hand device_payload to a kernel ...
//   }
//
// Consumer side, once per view (a fresh `view_mapping` on every frame would
// pay the registration cost per frame; map once when the view arrives and
// reuse the mapping across reads):
//
//   dora::cuda::MappedPool m;
//   std::uint64_t base = 0;
//   std::size_t bytes = 0;
//   if (view_mapping(view, base, bytes) &&
//       dora::cuda::map_pool(reinterpret_cast<void *>(base), bytes, m)) {
//       std::size_t offset = 0;
//       if (view_payload_offset(view, offset)) {
//           const void *device_payload = static_cast<char *>(m.device) + offset;
//           // ... hand device_payload to a kernel ...
//       }
//   }
//
// On an integrated GPU this is the only zero-copy route: host and device
// share the same physical memory, `cudaIpcGetMemHandle` is unsupported, and
// the mapped host pages are directly addressable from a kernel through
// `m.device`.
//
// This header does mapping and unmapping only. Streams, kernels, and any
// `cudaMemcpy` stay in the caller.

#pragma once

#include <cstddef>

#include <cuda_runtime.h>

namespace dora::cuda {

/// The same bytes under two addresses, once `map_pool` has registered them.
///
/// A handle, not an owner: copying a `MappedPool` copies the addresses, not
/// the registration, so both copies believe they must unmap it. Only ever
/// unmap the one you called `map_pool` into.
struct MappedPool {
    void *host = nullptr;
    void *device = nullptr;
    size_t bytes = 0;
};

/// True when device `device_index` is known to share memory with the host
/// (an integrated GPU). Such a device has no CUDA IPC, so a pool feeding it
/// must use the `unified` transport rather than `ipc`.
///
/// False means "not known to be integrated" — not "confirmed discrete". It
/// covers both a real discrete GPU and a query that failed (no CUDA device,
/// driver not loaded), and those two cases warrant different responses: pick
/// `ipc` for the former, treat the latter as "cannot select a transport"
/// rather than silently defaulting to a transport that cannot work. Call
/// `cudaGetDeviceProperties` directly if you need to tell them apart.
///
/// Checks device 0 by default. `map_pool` below acts on the CUDA *current*
/// device instead (`cudaHostGetDevicePointer` has no device-index parameter),
/// so on a single-GPU box the two agree; on a multi-GPU host, set the current
/// device to `device_index` before calling `map_pool` if you rely on this
/// check for it.
inline bool is_integrated_gpu(int device_index = 0) {
    cudaDeviceProp props{};
    if (cudaGetDeviceProperties(&props, device_index) != cudaSuccess) {
        return false;
    }
    return props.integrated != 0;
}

/// Page-lock `bytes` at `shm_base` and take its device alias.
///
/// `shm_base` must be the pool's mapping base (`pool_shm_base` /
/// `view_mapping`), not the payload start — see the file comment for why.
///
/// `cudaHostRegister` walks and pins every page in the segment, which is slow
/// relative to a frame tick on a segment sized for real payloads. Call this
/// once per pool — at setup on the producer side, once per view on the
/// consumer side — and reuse the resulting `MappedPool` across frames. Never
/// call it per frame.
///
/// Returns false and leaves `out` untouched on any CUDA error, including a
/// null or zero-length input. On failure, the CUDA runtime's last-error slot
/// is cleared before returning (see `out_error`'s doc) so a failed mapping
/// does not surface as a stale error on the caller's next unrelated
/// `cudaGetLastError()` — the standard post-kernel-launch check.
///
/// `out_error`, if non-null, receives the `cudaError_t` that caused the
/// failure (`cudaSuccess` on success). "No CUDA device", "already
/// registered", and "out of memory" are all distinguishable through it;
/// without it they all collapse to the same `false`.
inline bool map_pool(void *shm_base, size_t bytes, MappedPool &out,
                      cudaError_t *out_error = nullptr) {
    if (shm_base == nullptr || bytes == 0) {
        if (out_error != nullptr) {
            *out_error = cudaErrorInvalidValue;
        }
        return false;
    }
    cudaError_t err = cudaHostRegister(shm_base, bytes, cudaHostRegisterMapped);
    if (err != cudaSuccess) {
        cudaGetLastError(); // clear the sticky error before it reaches the caller's own checks
        if (out_error != nullptr) {
            *out_error = err;
        }
        return false;
    }
    void *device = nullptr;
    err = cudaHostGetDevicePointer(&device, shm_base, 0);
    if (err != cudaSuccess) {
        cudaHostUnregister(shm_base);
        cudaGetLastError(); // same: leave nothing stale behind a failed mapping
        if (out_error != nullptr) {
            *out_error = err;
        }
        return false;
    }
    out.host = shm_base;
    out.device = device;
    out.bytes = bytes;
    if (out_error != nullptr) {
        *out_error = cudaSuccess;
    }
    return true;
}

/// Release a mapping.
///
/// Call this before the segment is unlinked, not after — `cudaHostUnregister`
/// on pages no longer backed by the shared-memory file is undefined — **and**
/// before you drop the pool/view handle, whichever comes first: dropping the
/// `rust::Box` on the Rust side unmaps the segment out from under a still-
/// registered mapping exactly as an unlink does. `take_freed_pools()` exists
/// to give you the first "before" on a poll loop — check it on the node's
/// tick and unmap any pool it names before the next call that could free the
/// segment — but it says nothing about the second: an ordinary "done with
/// this view, dropping the handle now" path needs the same unmap first, and
/// nothing enforces that for you.
///
/// This is a plain function rather than an RAII guard because the right
/// release point is a `take_freed_pools()` tick, not a C++ scope exit — a
/// scope guard would either fire too early (the mapping is still in use when
/// the guard's scope ends) or force a lifetime onto the mapping that the pool
/// itself does not have. `PoolWriteGuard` in `dora/memory_pool.hpp` closes a
/// write cycle that genuinely is scope-bound; this is not that.
///
/// Safe to call twice on the same `MappedPool` and safe on a never-mapped
/// one: both leave `m` zeroed rather than double-unregistering. Not safe on
/// two different `MappedPool` copies of the same mapping — see the struct's
/// doc.
inline void unmap_pool(MappedPool &m) {
    if (m.host != nullptr) {
        cudaHostUnregister(m.host);
    }
    m.host = nullptr;
    m.device = nullptr;
    m.bytes = 0;
}

} // namespace dora::cuda
