// Compile coverage for `dora/cuda_pool.hpp`, mirroring the compile-only guard
// `memory_pool_header_compiles` gives `dora/memory_pool.hpp` in this same
// directory. The `cuda_pool_header_compiles` test in `src/lib.rs` compiles
// this file with `nvcc` against the headers `build.rs` just installed, and
// skips cleanly (does not fail) when no CUDA toolchain is on this machine —
// the crate's tests must not require CUDA.
//
// Nothing here is linked or run: `nvcc -c` is enough to make the compiler
// walk every declaration in the header, which is what a `.hpp`-only file
// needs to rot-proof it.

#include <cstddef>
#include <type_traits>

#include "dora/cuda_pool.hpp"

// A plain bag of host/device pointers and a length: default-constructible so
// a caller can declare one before knowing whether `map_pool` will succeed,
// and needing no special copy/move handling because it owns nothing that
// `unmap_pool` doesn't already put back to null on every path.
static_assert(std::is_default_constructible<dora::cuda::MappedPool>::value,
              "MappedPool must be default-constructible: map_pool takes an out-parameter, so a "
              "caller declares one before knowing whether mapping will succeed");
static_assert(std::is_trivially_copyable<dora::cuda::MappedPool>::value,
              "MappedPool is host/device pointers plus a length; it should need no special copy "
              "semantics");

// The shape every caller uses it in: declare, attempt to map, use the device
// pointer only if mapping succeeded, unmap on every exit path. `unmap_pool`
// has to tolerate being called on a `MappedPool` that never mapped
// successfully, and to tolerate a second call on the *same* handle (it is not
// safe on two independently-mapped or copied handles for the same mapping;
// see `MappedPool`'s doc).
bool dora_cuda_pool_header_compile_check(void *shm_base, std::size_t bytes) {
    dora::cuda::MappedPool m;
    cudaError_t map_err = cudaSuccess;
    const bool mapped = dora::cuda::map_pool(shm_base, bytes, m, &map_err);
    void *device = mapped ? m.device : nullptr;
    dora::cuda::unmap_pool(m);
    dora::cuda::unmap_pool(m);  // must be safe to call twice on the same handle
    return device != nullptr && map_err == cudaSuccess;
}

// `out_error` is optional: the common case (a caller that only branches on
// success/failure) must still compile without it.
bool dora_cuda_pool_header_compile_check_no_out_error(void *shm_base, std::size_t bytes) {
    dora::cuda::MappedPool m;
    const bool mapped = dora::cuda::map_pool(shm_base, bytes, m);
    dora::cuda::unmap_pool(m);
    return mapped;
}

// `is_integrated_gpu` takes no pool at all — it is a property of the device,
// checked once up front to choose a transport, not something wired to a
// mapping.
int dora_cuda_pool_is_integrated_gpu_compile_check() {
    return dora::cuda::is_integrated_gpu() ? 1 : 0;
}
