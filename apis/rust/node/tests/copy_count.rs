//! Copy-count harness for the send-side encoders (plan W0-D).
//!
//! Counts heap bytes allocated *during* each encode strategy with a counting
//! global allocator. This measures whether a strategy materializes a
//! **payload-sized intermediate buffer** — the tell-tale of an extra payload
//! copy. The destination sample is allocated *outside* the measured region (as
//! in production, where it is the SHM sample), so:
//!
//! | strategy                       | payload copies | intermediate alloc |
//! |--------------------------------|----------------|--------------------|
//! | construct in place (write dst) | 0              | ~0                 |
//! | raw `copy_array_into_sample`   | 1 (into dst)   | ~0                 |
//! | IPC fast `encode_ipc_into`     | 1 (into dst)   | ~0                 |
//! | IPC fallback `encode_ipc_to_vec` | 2 (array->Vec->dst) | >= payload   |
//!
//! The first three write straight into the caller's buffer and never allocate a
//! second payload-sized buffer; the fallback allocates a full payload-sized
//! `Vec` (the official writer's staging buffer). This is the regression guard
//! for the #1745 two-copy SHM path that W0-A removed and the fast-vs-fallback
//! distinction W0-B introduced.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use dora_node_api::arrow::array::{Array, Float32Array};
use dora_node_api::arrow_utils::ipc_encode::{
    encode_ipc_into, encode_ipc_to_vec, ipc_fast_path_len,
};
use dora_node_api::arrow_utils::{copy_array_into_sample, required_data_size};

/// Bytes allocated since process start (only the growth, for reallocs).
static ALLOCATED: AtomicUsize = AtomicUsize::new(0);

struct CountingAlloc;

// SAFETY: forwards every operation to the system allocator unchanged; the only
// added behavior is a relaxed atomic add, which cannot affect memory safety.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATED.fetch_add(layout.size(), Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATED.fetch_add(layout.size(), Ordering::Relaxed);
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if new_size > layout.size() {
            ALLOCATED.fetch_add(new_size - layout.size(), Ordering::Relaxed);
        }
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: CountingAlloc = CountingAlloc;

/// Run `f` and return the heap bytes it allocated. Single-threaded test, so no
/// other allocations interleave between the two counter reads.
fn allocated_during<R>(f: impl FnOnce() -> R) -> (usize, R) {
    let before = ALLOCATED.load(Ordering::Relaxed);
    let r = f();
    let after = ALLOCATED.load(Ordering::Relaxed);
    (after.wrapping_sub(before), r)
}

const ELEMENTS: usize = 1_048_576; // 1M f32 = 4 MiB payload
const PAYLOAD: usize = ELEMENTS * 4;

fn big_array() -> dora_node_api::arrow::array::ArrayData {
    Float32Array::from((0..ELEMENTS).map(|i| i as f32).collect::<Vec<_>>()).into_data()
}

/// Generous ceiling for "no payload-sized intermediate": headers, the flatbuffer
/// builder, and the small per-buffer metadata vectors are all far below this.
const NO_PAYLOAD_INTERMEDIATE: usize = PAYLOAD / 8;

#[test]
fn construct_in_place_allocates_no_payload_intermediate() {
    // Build the message straight into a pre-allocated sample (0 copies from a
    // source array — the write *is* the construction).
    let mut sample = vec![0u8; PAYLOAD];
    sample.iter_mut().for_each(|b| *b = 0); // pre-fault

    let (allocated, ()) = allocated_during(|| {
        for (i, chunk) in sample.chunks_exact_mut(4).enumerate() {
            chunk.copy_from_slice(&(i as f32).to_le_bytes());
        }
    });
    assert!(
        allocated < NO_PAYLOAD_INTERMEDIATE,
        "construct-in-place allocated {allocated} bytes (>= {NO_PAYLOAD_INTERMEDIATE}); \
         expected no payload-sized intermediate"
    );
}

#[test]
fn raw_copy_allocates_no_payload_intermediate() {
    let data = big_array();
    let mut sample = vec![0u8; required_data_size(&data)];
    sample.iter_mut().for_each(|b| *b = 0); // pre-fault

    let (allocated, _info) = allocated_during(|| copy_array_into_sample(&mut sample, &data));
    assert!(
        allocated < NO_PAYLOAD_INTERMEDIATE,
        "raw copy_array_into_sample allocated {allocated} bytes (>= {NO_PAYLOAD_INTERMEDIATE}); \
         the payload should be copied once, straight into the sample"
    );
}

#[test]
fn ipc_fast_allocates_no_payload_intermediate() {
    let data = big_array();
    let len = ipc_fast_path_len(&data).expect("primitive array is fast-path eligible");
    let mut sample = vec![0u8; len];
    sample.iter_mut().for_each(|b| *b = 0); // pre-fault

    let (allocated, result) = allocated_during(|| encode_ipc_into(&data, &mut sample));
    result.expect("fast-path encode");
    assert!(
        allocated < NO_PAYLOAD_INTERMEDIATE,
        "IPC fast path allocated {allocated} bytes (>= {NO_PAYLOAD_INTERMEDIATE}); \
         it must write the payload straight into the sample with no body Vec"
    );
}

#[test]
fn ipc_fallback_allocates_payload_intermediate() {
    // Force the fallback with a Float32 array (it IS fast-path eligible, but
    // `encode_ipc_to_vec` always uses the official writer, which stages the body
    // in a payload-sized Vec — the second copy).
    let data = big_array();

    let (allocated, vec) = allocated_during(|| encode_ipc_to_vec(&data).unwrap());
    assert!(
        vec.len() >= PAYLOAD,
        "fallback stream {} should contain the whole payload",
        vec.len()
    );
    assert!(
        allocated >= PAYLOAD,
        "IPC fallback allocated only {allocated} bytes (< {PAYLOAD}); \
         the official writer is expected to stage a payload-sized Vec"
    );
}
