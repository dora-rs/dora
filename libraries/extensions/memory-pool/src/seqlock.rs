//! Single-writer / many-reader seqlock over the `write_gen` field at offset 96
//! of a `DORADMA` segment.
//!
//! Even means "the payload is complete", odd means "a writer is mid-write".
//! A reader samples the generation, copies, samples again, and keeps the copy
//! only if both samples are equal and even.
//!
//! # Safety
//!
//! Every function takes a raw pointer into a mapped segment. The caller must
//! guarantee the pointer is 8-byte aligned, within the mapping, and that the
//! mapping outlives the call.

use std::sync::atomic::{Ordering, fence};

/// Open a write cycle and return the **even** baseline generation to hand back
/// to [`end_write`].
///
/// If the generation is already odd — a previous writer died mid-write — the
/// increment is skipped and the previous even value is returned, so
/// `end_write`'s `pre + 2` still lands on an even generation.
///
/// # Safety
/// See the module docs.
pub unsafe fn begin_write(gen_ptr: *mut u64) -> u64 {
    unsafe {
        let cur = std::ptr::read_volatile(gen_ptr);
        if cur.is_multiple_of(2) {
            std::ptr::write_volatile(gen_ptr, cur + 1);
            fence(Ordering::Release);
        }
        cur & !1
    }
}

/// Close a write cycle: publish `pre_write_gen + 2` on success, or roll back to
/// `pre_write_gen` when the payload could not be completed.
///
/// # Safety
/// See the module docs.
pub unsafe fn end_write(gen_ptr: *mut u64, pre_write_gen: u64, ok: bool) {
    unsafe {
        fence(Ordering::Release);
        let next = if ok {
            pre_write_gen.wrapping_add(2)
        } else {
            pre_write_gen
        };
        std::ptr::write_volatile(gen_ptr, next);
        fence(Ordering::Release);
    }
}

/// Sample the current generation.
///
/// # Safety
/// See the module docs.
pub unsafe fn read_gen(gen_ptr: *const u64) -> u64 {
    unsafe {
        let g = std::ptr::read_volatile(gen_ptr);
        fence(Ordering::Acquire);
        g
    }
}

/// True when a generation value denotes a complete payload.
pub fn is_complete(generation: u64) -> bool {
    generation.is_multiple_of(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(v: u64) -> Box<u64> {
        Box::new(v)
    }

    #[test]
    fn begin_makes_it_odd_and_end_makes_it_even() {
        let mut g = cell(0);
        let pre = unsafe { begin_write(&mut *g) };
        assert_eq!(pre, 0);
        assert_eq!(*g, 1, "generation must be odd while writing");
        unsafe { end_write(&mut *g, pre, true) };
        assert_eq!(*g, 2, "a successful write must publish the next even gen");
    }

    #[test]
    fn failed_write_rolls_back_to_the_previous_even_gen() {
        let mut g = cell(4);
        let pre = unsafe { begin_write(&mut *g) };
        unsafe { end_write(&mut *g, pre, false) };
        assert_eq!(*g, 4, "a failed write must not publish a new generation");
    }

    /// A writer killed mid-write leaves the generation odd. The next writer
    /// must recover to an even baseline instead of inverting parity forever.
    #[test]
    fn begin_recovers_from_a_leftover_odd_generation() {
        let mut g = cell(7);
        let pre = unsafe { begin_write(&mut *g) };
        assert_eq!(pre, 6, "baseline must be the previous even value");
        unsafe { end_write(&mut *g, pre, true) };
        assert_eq!(*g, 8);
        assert!(is_complete(*g));
    }

    #[test]
    fn is_complete_tracks_parity() {
        assert!(is_complete(0));
        assert!(is_complete(2));
        assert!(!is_complete(1));
        assert!(!is_complete(3));
    }

    #[test]
    fn read_gen_sees_the_current_value() {
        let g = cell(12);
        assert_eq!(unsafe { read_gen(&*g) }, 12);
    }
}
