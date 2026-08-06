// Compile coverage for `dora/memory_pool.hpp`.
//
// The crate ships its C++ sources for a consumer to build rather than
// compiling them here, so nothing in this repository would otherwise notice
// the header rotting. The `memory_pool_header_compiles` test in `src/lib.rs`
// is what compiles this file, against the headers `build.rs` just installed.
//
// Nothing here is linked or run: the guard is exercised for its type
// properties and for the shape of the code it exists to make safe.

#include <cstddef>
#include <cstring>
#include <type_traits>
#include <utility>

#include "dora/memory_pool.hpp"

using PoolRef = ::rust::Box<::DoraMemoryPool> &;

static_assert(!std::is_copy_constructible<dora::PoolWriteGuard>::value,
              "PoolWriteGuard must not be copyable: two guards would close one cycle twice, "
              "and the second pool_end_write would bump the generation over the next frame");
static_assert(!std::is_copy_assignable<dora::PoolWriteGuard>::value,
              "PoolWriteGuard must not be copy-assignable");
static_assert(!std::is_move_constructible<dora::PoolWriteGuard>::value,
              "PoolWriteGuard must not be movable: the cycle belongs to the scope that opened it");
static_assert(!std::is_move_assignable<dora::PoolWriteGuard>::value,
              "PoolWriteGuard must not be move-assignable");
static_assert(!std::is_default_constructible<dora::PoolWriteGuard>::value,
              "PoolWriteGuard must not be default-constructible: a guard without a pool has no "
              "cycle to close");

// `new PoolWriteGuard(pool)` must be ill-formed. Probing the expression rather
// than `T::operator new` is what distinguishes "deleted" from "never
// declared" — the latter is what a class that simply forgot the deletion looks
// like, and it must not pass.
template <typename T, typename = void>
struct is_heap_allocatable : std::false_type {};
template <typename T>
struct is_heap_allocatable<T, std::void_t<decltype(new T(std::declval<PoolRef>()))>>
    : std::true_type {};

static_assert(!is_heap_allocatable<dora::PoolWriteGuard>::value,
              "PoolWriteGuard::operator new must stay deleted: a heap guard outlives the scope "
              "that owns the pool");

// The shape the guard exists for: an early return out of the middle of a write
// cycle, which the destructor has to close.
bool dora_memory_pool_header_compile_check(PoolRef pool, const void *src, std::size_t len) {
    dora::PoolWriteGuard write(pool);
    if (write.size() < len) {
        return false;  // ~PoolWriteGuard closes the cycle, frame marked incomplete
    }
    std::memcpy(write.data(), src, len);
    write.commit();
    return true;
}
