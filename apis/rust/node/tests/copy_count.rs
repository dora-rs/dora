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
//! | IPC fast `encode_ipc_into`     | 1 (into dst)   | ~0                 |
//! | IPC fallback `encode_ipc_to_vec` | 2 (array->Vec->dst) | >= payload   |
//!
//! The first two write straight into the caller's buffer and never allocate a
//! second payload-sized buffer; the fallback allocates a full payload-sized
//! `Vec` (the official writer's staging buffer). This is the regression guard
//! for the #1745 two-copy SHM path that W0-A removed and the fast-vs-fallback
//! distinction W0-B introduced.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use dora_node_api::arrow::array::{Array, Float32Array};
use dora_node_api::arrow_utils::ipc_encode::{
    encode_ipc_into, encode_ipc_to_vec, encode_uint8_ipc_header, ipc_fast_path_len, uint8_ipc_len,
};

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

/// All strategies are measured in ONE test on purpose: the allocation counter
/// is process-global, so two measured regions running on parallel test threads
/// would pollute each other's deltas. A single test runs alone in this binary,
/// keeping every `allocated_during` window free of concurrent allocations. (If
/// you add another `#[test]` here, gate both with a shared mutex or run
/// `--test-threads=1`.)
#[test]
fn send_strategies_intermediate_allocation() {
    // --- construct in place: build straight into a pre-allocated sample (0
    // copies from a source array — the write *is* the construction). ---
    {
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

    // --- send_output_raw / Python: pre-write the UInt8 IPC header into a
    // pre-allocated sample, then fill the data region in place. The whole stream
    // is constructed in the sample with NO payload-sized intermediate. ---
    {
        let total = uint8_ipc_len(PAYLOAD).unwrap();
        let mut sample = vec![0u8; total];
        sample.iter_mut().for_each(|b| *b = 0); // pre-fault
        let (allocated, ()) = allocated_during(|| {
            let offset = encode_uint8_ipc_header(&mut sample, PAYLOAD).unwrap();
            // The caller writes the data straight into the wire payload.
            for (i, b) in sample[offset..offset + PAYLOAD].iter_mut().enumerate() {
                *b = i as u8;
            }
        });
        assert!(
            allocated < NO_PAYLOAD_INTERMEDIATE,
            "send_output_raw construct-in-place allocated {allocated} bytes \
             (>= {NO_PAYLOAD_INTERMEDIATE}); the header is tiny and the data is \
             written in place, so there must be no payload-sized intermediate"
        );
    }

    // --- IPC fast path: writes the payload straight into the sample, no body Vec. ---
    {
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

    // --- IPC fallback: the official writer stages the body in a payload-sized
    // Vec (the second copy). ---
    {
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
}
