// RAII wrapper around the memory-pool write cycle.
//
// This header is CUDA-free and depends on nothing but the generated bridge
// header. The optional `dora/cuda_pool.hpp` maps a pool for the GPU; the two
// are independent, and a CPU node needs only this one.
//
// # Why the raw calls are not the documented path
//
// `pool_begin_write` marks the pool's generation odd; `pool_end_write` marks
// it even again. Between the two, every reader treats the payload as torn and
// discards it — which is the point. But an early `return`, a `break`, or a
// thrown exception between the two leaves the generation odd *permanently*:
// the pool is then unreadable to every consumer until some later write closes
// a cycle, and nothing on the Rust side recovers it, because the pool handle
// lives on this side of the bridge and Rust never sees the abandoned cycle.
//
// `PoolWriteGuard` closes the cycle from its destructor, which runs on all
// three of those paths. Publishing is opt-in: the frame is marked complete
// only if `commit()` was called, so an abandoned write is published as
// incomplete rather than as data. There is deliberately no rollback — the
// payload is written in place, so by the time a write fails the previous
// frame is already gone.

#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "dora-node-api.h"

namespace dora {

/// Holds a pool's write cycle open for the lifetime of the guard.
///
/// ```cpp
/// {
///     dora::PoolWriteGuard write(pool);
///     std::memcpy(write.data(), frame, write.size());  // may throw
///     write.commit();                                  // publishes the frame
/// }   // an uncommitted guard marks the frame incomplete instead
/// ```
///
/// The guard is what makes an in-place fill safe. The one-call alternative,
/// `write_memory_pool(pool, data)`, copies a buffer you already hold and
/// brackets its own cycle in Rust — so it is safe on its own, but it must
/// **not** be called while a guard is alive: `pool_begin_write` refuses a
/// second cycle, and the copy would fail.
///
/// Not copyable and not movable: two objects closing one cycle would publish
/// the frame twice, and the second `pool_end_write` would bump the generation
/// over whatever the producer wrote next. `operator new` is deleted for the
/// same reason a stack guard is the point — a heap guard outlives the scope
/// that owns the pool as easily as it outlives nothing at all.
///
/// Do not move or free the pool while a guard is alive. The guard holds a
/// reference to the caller's `rust::Box`, so moving the box out from under it
/// or calling `free_memory_pool` leaves the destructor closing a cycle on a
/// null box.
class PoolWriteGuard {
  public:
    /// Opens a write cycle on `pool`.
    ///
    /// Throws `std::runtime_error` if the pool has no payload in its mapping
    /// (an `ipc` pool, whose data is in device memory) or if a cycle is
    /// already open on this handle. Nothing is left open when it throws.
    explicit PoolWriteGuard(::rust::Box<::DoraMemoryPool> &pool) : pool_(pool) {
        ::std::uint64_t payload_ptr = 0;
        ::std::size_t payload_len = 0;
        if (!::pool_payload(pool_, payload_ptr, payload_len)) {
            throw ::std::runtime_error(
                "dora::PoolWriteGuard: pool `" + ::std::string(::pool_id(pool_)) +
                "` has no payload in its shared-memory mapping (transport `" +
                ::std::string(::pool_transport(pool_)) + "`), so there is nothing here to write");
        }
        ::DoraResult result = ::pool_begin_write(pool_);
        if (!result.error.empty()) {
            throw ::std::runtime_error("dora::PoolWriteGuard: " + ::std::string(result.error));
        }
        open_ = true;
        data_ = reinterpret_cast<::std::uint8_t *>(static_cast<::std::uintptr_t>(payload_ptr));
        size_ = payload_len;
    }

    /// Closes the cycle if `commit()` did not. The frame is marked incomplete,
    /// so readers reject it until the next successful write.
    ~PoolWriteGuard() { close(false); }

    PoolWriteGuard(const PoolWriteGuard &) = delete;
    PoolWriteGuard &operator=(const PoolWriteGuard &) = delete;
    PoolWriteGuard(PoolWriteGuard &&) = delete;
    PoolWriteGuard &operator=(PoolWriteGuard &&) = delete;

    /// A guard is only ever a scope-bound stack object; see the class note.
    static void *operator new(::std::size_t) = delete;

    /// Publishes the frame and closes the cycle. Idempotent; after it returns,
    /// the destructor does nothing and `data()` is null.
    void commit() noexcept { close(true); }

    /// Start of the payload while the cycle is open, and null once `commit()`
    /// or destruction has closed it. Null rather than stale on purpose: a write
    /// through a pointer kept past the cycle lands in the payload with no
    /// generation bump around it, so every reader publishes it as a complete
    /// frame. A null dereference stops there instead.
    ::std::uint8_t *data() const noexcept { return data_; }

    /// Bytes of payload in the mapping — the only bound for a write through
    /// `data()`, and zero once the cycle is closed. Never size a copy from the
    /// pool's shape: it is advisory, and an unrecognized dtype makes a
    /// shape-derived product an under-estimate.
    ::std::size_t size() const noexcept { return size_; }

    /// True until `commit()` or destruction closes the cycle.
    bool open() const noexcept { return open_; }

  private:
    void close(bool ok) noexcept {
        if (open_) {
            open_ = false;
            data_ = nullptr;
            size_ = 0;
            ::pool_end_write(pool_, ok);
        }
    }

    ::rust::Box<::DoraMemoryPool> &pool_;
    ::std::uint8_t *data_ = nullptr;
    ::std::size_t size_ = 0;
    bool open_ = false;
};

}  // namespace dora
