//! Single-writer / many-reader seqlock over the `write_gen` field at offset 96
//! of a `DORADMA` segment.
//!
//! Even means "the payload is complete", odd means "a writer is mid-write". A
//! reader takes an opening sample with [`begin_read`], copies the payload, and
//! keeps the copy only if [`read_completed`] agrees.
//!
//! # Two reader edges, two functions
//!
//! The opening and closing samples need different ordering:
//! - opening: `gen load -> payload loads` — nothing may read the payload
//!   before the generation is sampled.
//! - closing: `payload loads -> gen load` — the closing sample may not be
//!   hoisted above the payload reads it is meant to bracket.
//!
//! A plain acquire load gives the first edge for free (acquire orders
//! *later* operations in this thread after it) but gives nothing for the
//! second (acquire does not hold *earlier* operations before it).
//! [`read_completed`] therefore pairs a standalone acquire fence *before* a
//! relaxed load, rather than reusing an acquire load.
//!
//! Getting that backwards — closing the read with a second [`begin_read`] —
//! produces code that passes every test, because both forms load the same
//! value and differ only in a fence. The type system is the only thing that
//! can catch it, so [`begin_read`] returns an opaque [`OpeningSample`] rather
//! than a `u64`, and the closing load lives inside [`read_completed`] where a
//! caller cannot substitute for it.
//!
//! # Atomics, not volatile
//!
//! Every access goes through `AtomicU64::from_ptr` rather than
//! `read_volatile`/`write_volatile` with a bystander `fence`. `volatile`
//! stops the compiler from eliding or duplicating an access, but it gives no
//! synchronizes-with relation under Rust's memory model — a fence has
//! nothing to attach to without an atomic operation next to it. `AtomicU64`
//! has the same size, alignment and layout as `u64` and is lock-free on
//! every target this crate supports, so the wire format (a naturally
//! aligned little-endian `u64` at byte 96) is unchanged. Interop with the
//! Python binding's plain volatile access is unaffected by this change: that
//! side's correctness already rests on naturally aligned 8-byte accesses
//! being single-copy-atomic on the platforms both bindings run on, which is
//! independent of which side uses `AtomicU64`.
//!
//! # Safety
//!
//! Every function takes a raw pointer into a mapped segment. The caller must
//! guarantee:
//! - single-writer exclusivity: at most one writer calls [`begin_write`] /
//!   [`end_write`] on a given location at a time;
//! - no live Rust reference (`&u64` / `&mut u64`) aliases the location for
//!   the duration of the call — go through the pointer only;
//! - the location is valid for reads (and, for the writer, valid for
//!   writes) and holds an initialized `u64`;
//! - [`end_write`] receives exactly the value its matching [`begin_write`]
//!   returned.
//!
//! Alignment is not a caller obligation to track: `AtomicU64::from_ptr`
//! requires 8-byte alignment, and byte offset 96 of a page-aligned
//! `DORADMA` mapping satisfies that by construction.

use std::sync::atomic::{AtomicU64, Ordering, fence};

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
    let atomic = unsafe { AtomicU64::from_ptr(gen_ptr) };
    let cur = atomic.load(Ordering::Relaxed);
    if cur.is_multiple_of(2) {
        atomic.store(cur + 1, Ordering::Relaxed);
    }
    // Unconditional, and a fence rather than a release store: a release store
    // would order what precedes it, whereas the edge needed here is
    // `odd marker -> payload stores`, which only a fence after the store
    // provides. On the recovery path (generation already odd, left by a writer
    // that died mid-write) nothing is stored at all, and the fence instead
    // orders the load above against the caller's payload stores — so another
    // thread cannot observe those stores without also observing the odd marker.
    fence(Ordering::Release);
    cur & !1
}

/// Close a write cycle.
///
/// On success, publishes `pre_write_gen + 2` — the next even generation,
/// meaning "payload complete". On failure, does **nothing**: the generation
/// is left exactly as `begin_write` set it, odd. There is no previous frame
/// to roll back to — this is an in-place write, so a failed write has
/// already destroyed whatever payload was there. Publishing an even
/// generation over a payload the write never finished would tell readers a
/// torn buffer is safe to copy; leaving the generation odd makes every
/// reader reject the pool until the next successful write.
///
/// # Safety
/// See the module docs.
pub unsafe fn end_write(gen_ptr: *mut u64, pre_write_gen: u64, ok: bool) {
    if !ok {
        return;
    }
    let atomic = unsafe { AtomicU64::from_ptr(gen_ptr) };
    atomic.store(pre_write_gen.wrapping_add(2), Ordering::Release);
}

/// The opening generation sample of a read, from [`begin_read`].
///
/// Opaque on purpose: the inner value is not reachable, so it cannot be
/// forged, arrived at by arithmetic, or handed to something expecting the
/// closing load. The only thing it can be used for is [`read_completed`].
///
/// Deliberately **not** `PartialEq`. Comparing two samples by hand is exactly
/// the mistake this type exists to prevent — it is how a reader ends up
/// closing its read with a second [`begin_read`], which loads the same value
/// with the wrong fence and so passes every test while admitting torn frames
/// on a weakly-ordered CPU. Without `PartialEq` that code does not compile.
#[derive(Debug, Clone, Copy)]
pub struct OpeningSample(u64);

impl OpeningSample {
    /// True when the sample itself denotes a complete payload. A reader may
    /// skip the copy entirely when this is false — [`read_completed`] would
    /// reject it regardless.
    pub fn is_complete(self) -> bool {
        is_complete(self.0)
    }
}

/// Take the opening generation sample, before reading the payload.
///
/// The acquire load orders every later operation in this thread after it,
/// which is exactly the `gen load -> payload loads` edge a reader needs at
/// the start of a read.
///
/// # Safety
/// See the module docs.
pub unsafe fn begin_read(gen_ptr: *mut u64) -> OpeningSample {
    let atomic = unsafe { AtomicU64::from_ptr(gen_ptr) };
    OpeningSample(atomic.load(Ordering::Acquire))
}

/// Close a read: true when the payload copied since `opening` is intact.
///
/// Takes the closing sample and applies both acceptance rules in one place —
/// the opening sample must be even, and the two samples must be equal.
///
/// An acquire load only orders what comes *after* it; it does nothing to
/// stop the load itself from being hoisted above the payload reads it is
/// meant to close out. A standalone acquire fence placed *before* a relaxed
/// load gives the `payload loads -> gen load` edge instead. That is why this
/// function exists rather than a second sampling function the caller
/// compares by hand: the wrong load here is invisible to every test.
///
/// # Safety
/// See the module docs.
pub unsafe fn read_completed(gen_ptr: *mut u64, opening: OpeningSample) -> bool {
    let atomic = unsafe { AtomicU64::from_ptr(gen_ptr) };
    fence(Ordering::Acquire);
    opening.is_complete() && atomic.load(Ordering::Relaxed) == opening.0
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
    fn failed_write_leaves_the_generation_odd_so_readers_reject() {
        let mut g = cell(4);
        let pre = unsafe { begin_write(&mut *g) };
        unsafe { end_write(&mut *g, pre, false) };
        assert_eq!(
            *g, 5,
            "a failed write must leave the generation exactly as begin_write set it, odd"
        );
        assert!(!is_complete(*g));
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
    fn an_undisturbed_read_completes() {
        let mut g = cell(4);
        let opening = unsafe { begin_read(&mut *g) };
        assert!(opening.is_complete());
        assert!(unsafe { read_completed(&mut *g, opening) });
    }

    #[test]
    fn a_write_that_starts_mid_read_is_rejected() {
        let mut g = cell(4);
        let opening = unsafe { begin_read(&mut *g) };
        let pre = unsafe { begin_write(&mut *g) };
        assert!(
            !unsafe { read_completed(&mut *g, opening) },
            "a write in progress must not be accepted"
        );
        unsafe { end_write(&mut *g, pre, true) };
        assert!(
            !unsafe { read_completed(&mut *g, opening) },
            "the closing sample must see the new generation, not the opening one"
        );
    }

    /// A reader that opens mid-write holds an odd sample. Nothing else
    /// changes the generation, so both samples agree — parity is the only
    /// thing that can reject it.
    #[test]
    fn an_odd_opening_sample_is_rejected_even_though_both_samples_agree() {
        let mut g = cell(4);
        let _pre = unsafe { begin_write(&mut *g) };
        let opening = unsafe { begin_read(&mut *g) };
        assert!(!opening.is_complete());
        assert!(
            !unsafe { read_completed(&mut *g, opening) },
            "an odd opening sample must never be accepted"
        );
    }

    #[test]
    fn the_generation_wraps_without_panicking() {
        let mut g = cell(u64::MAX - 1);
        let pre = unsafe { begin_write(&mut *g) };
        assert_eq!(*g, u64::MAX);
        unsafe { end_write(&mut *g, pre, true) };
        assert_eq!(*g, 0);
        assert!(is_complete(*g));
    }
}
