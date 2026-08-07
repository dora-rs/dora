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
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

#include "dora/memory_pool.hpp"

using PoolRef = ::rust::Box<::DoraMemoryPool> &;
using ViewRef = const ::rust::Box<::DoraMemoryPoolView> &;

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

// The read guard carries the same restrictions as the write guard: it holds a
// reference to the caller's box, so it must not be copied, moved, or heaped
// out of the scope that owns the view.
static_assert(!std::is_copy_constructible<dora::PoolReadGuard>::value,
              "PoolReadGuard must not be copyable");
static_assert(!std::is_copy_assignable<dora::PoolReadGuard>::value,
              "PoolReadGuard must not be copy-assignable");
static_assert(!std::is_move_constructible<dora::PoolReadGuard>::value,
              "PoolReadGuard must not be movable");
static_assert(!std::is_move_assignable<dora::PoolReadGuard>::value,
              "PoolReadGuard must not be move-assignable");
static_assert(!std::is_default_constructible<dora::PoolReadGuard>::value,
              "PoolReadGuard must not be default-constructible: a guard without a view has no "
              "generation to sample");

template <typename T, typename = void>
struct is_view_heap_allocatable : std::false_type {};
template <typename T>
struct is_view_heap_allocatable<T, std::void_t<decltype(new T(std::declval<ViewRef>()))>>
    : std::true_type {};

static_assert(!is_view_heap_allocatable<dora::PoolReadGuard>::value,
              "PoolReadGuard::operator new must stay deleted: a heap guard outlives the scope "
              "that owns the view");

// Placement new is the same hazard wearing a caller-owned buffer, and it is
// blocked by the same single declaration: a class-scope `operator new` hides
// every global form, so `new (buffer) Guard(...)` never reaches ::operator new.
// These probe the property, not the line that provides it — they fire when a
// guard declares no `operator new` at all, which is what a class that forgot
// the deletion looks like.
template <typename T, typename A, typename = void>
struct is_placement_constructible : std::false_type {};
template <typename T, typename A>
struct is_placement_constructible<
    T, A, std::void_t<decltype(new (std::declval<void *>()) T(std::declval<A>()))>>
    : std::true_type {};

static_assert(!is_placement_constructible<dora::PoolWriteGuard, PoolRef>::value,
              "PoolWriteGuard must not be placement-constructible: a guard built into a "
              "caller-owned buffer outlives its scope exactly as a heap one does");
static_assert(!is_placement_constructible<dora::PoolReadGuard, ViewRef>::value,
              "PoolReadGuard must not be placement-constructible");

// The probe must be able to say yes, or it proves nothing about either guard.
struct PlacementProbe {
    explicit PlacementProbe(int) {}
};
static_assert(is_placement_constructible<PlacementProbe, int>::value,
              "the placement probe must detect a type that IS placement-constructible");

// `data()` must not hand out a writable pointer: the reader does not own the
// frame, and a store through it would land in the payload with no generation
// bump around it, so every other reader would publish it as a complete frame.
static_assert(std::is_same<decltype(std::declval<const dora::PoolReadGuard &>().data()),
                           const std::uint8_t *>::value,
              "PoolReadGuard::data() must be a pointer to const");

// The opening sample must be unreachable as a value: it lives in an opaque
// rust::Box that C++ can neither construct nor copy out of the guard.
static_assert(!std::is_constructible<::rust::Box<::DoraPoolRead>>::value,
              "DoraPoolRead must not be constructible on the C++ side: a forged token would "
              "bracket a read that never happened");

// The zero-copy shape: work in place, then throw the result away if the
// bracket did not hold.
bool dora_memory_pool_read_guard_compile_check(ViewRef view, std::uint64_t *checksum) {
    dora::PoolReadGuard read(view);
    std::uint64_t sum = 0;
    for (std::size_t i = 0; i < read.size(); ++i) {
        sum += read.data()[i];
    }
    if (!read.valid()) {
        return false;  // the frame was overwritten mid-scan; `sum` is garbage
    }
    *checksum = sum;
    return true;
}

// The copying shape. The outcome is a three-state `enum class`, so the caller
// has to say what it does about a pool that will never yield bytes.
bool dora_memory_pool_try_read_compile_check(ViewRef view, std::vector<std::uint8_t> &out) {
    switch (dora::try_read_pool(view, out)) {
        case dora::PoolReadOutcome::Copied:
            return true;
        case dora::PoolReadOutcome::Torn:
            return false;  // retry on the next event
        case dora::PoolReadOutcome::Unavailable:
            return false;  // ipc or freed: retrying would spin forever
    }
    return false;
}

// The bug the enum exists to stop: `while (!try_read_pool(...)) {}` spins
// forever on an `ipc` view, whose payload is in device memory and will never
// arrive in this mapping. An `enum class` has no conversion to bool, so the
// loop is ill-formed rather than merely wrong.
template <typename T, typename = void>
struct is_negatable : std::false_type {};
template <typename T>
struct is_negatable<T, std::void_t<decltype(!std::declval<T>())>> : std::true_type {};

static_assert(is_negatable<bool>::value, "the probe must detect a type that IS negatable");
static_assert(!is_negatable<dora::PoolReadOutcome>::value,
              "try_read_pool's outcome must not be usable as a bool: `while (!try_read_pool(view, "
              "buf)) {}` spins forever on an ipc or freed view");
