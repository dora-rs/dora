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
//     the base and length, `pool_payload_offset(pool, offset)` for the
//     payload's offset into that mapping.
//   - consumer side: `view_mapping(view, base, bytes)` for the base and
//     length, `view_payload_offset(view, offset)` for the offset.
//
// `pool_payload_offset` and `view_payload_offset` are predicates that return
// false — leaving `offset` untouched — for an `ipc` pool: its payload lives
// in device memory, not in this host mapping, so there is no host offset to
// report and nothing here to map.
//
//   dora::cuda::MappedPool m;
//   if (dora::cuda::map_pool(reinterpret_cast<void *>(pool_shm_base(pool)),
//                             pool_segment_bytes(pool), m)) {
//       std::size_t offset = 0;
//       pool_payload_offset(pool, offset);
//       void *device_payload = static_cast<char *>(m.device) + offset;
//       // ... hand device_payload to a kernel ...
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

/// A segment registered with the CUDA driver: the same bytes under two
/// addresses.
struct MappedPool {
    void *host = nullptr;
    void *device = nullptr;
    size_t bytes = 0;
};

/// True when device `device_index` shares memory with the host (an
/// integrated GPU). Such a device has no CUDA IPC, so a pool feeding it must
/// use the `unified` transport rather than `ipc`.
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
/// Returns false and leaves `out` untouched on any CUDA error, including a
/// null or zero-length input.
inline bool map_pool(void *shm_base, size_t bytes, MappedPool &out) {
    if (shm_base == nullptr || bytes == 0) {
        return false;
    }
    if (cudaHostRegister(shm_base, bytes, cudaHostRegisterMapped) != cudaSuccess) {
        return false;
    }
    void *device = nullptr;
    if (cudaHostGetDevicePointer(&device, shm_base, 0) != cudaSuccess) {
        cudaHostUnregister(shm_base);
        return false;
    }
    out.host = shm_base;
    out.device = device;
    out.bytes = bytes;
    return true;
}

/// Release a mapping.
///
/// Call this before the segment is unlinked, not after: `cudaHostUnregister`
/// on pages that are no longer backed by the shared-memory file is undefined.
/// `take_freed_pools()` exists to give you that "before" — poll it on the
/// node's tick and unmap any pool it names before the next call that could
/// free the segment.
///
/// Safe to call twice and safe on a never-mapped `MappedPool`: both leave
/// `m` zeroed rather than double-unregistering.
inline void unmap_pool(MappedPool &m) {
    if (m.host != nullptr) {
        cudaHostUnregister(m.host);
    }
    m.host = nullptr;
    m.device = nullptr;
    m.bytes = 0;
}

} // namespace dora::cuda
