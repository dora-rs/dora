use std::{mem::ManuallyDrop, ops::Deref};

use super::traits::{FFIFromRust, FFIToRust};

#[repr(C)]
#[derive(Debug)]
pub struct FFISeq<T> {
    data: *mut T,
    size: usize,
    capacity: usize,
}

impl<T> FFISeq<T> {
    /// Extracts a slice.
    pub fn as_slice(&self) -> &[T] {
        self
    }

    /// Returns the length of the sequence.
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the sequence has a length of 0.
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<T> FFIToRust for FFISeq<T>
where
    T: FFIToRust,
{
    type Target = Vec<T::Target>;

    unsafe fn to_rust(&self) -> Self::Target {
        self.iter().map(|v| unsafe { v.to_rust() }).collect()
    }
}

macro_rules! impl_traits_to_primitive {
    ($type: ty) => {
        impl FFIToRust for FFISeq<$type> {
            type Target = Vec<$type>;

            unsafe fn to_rust(&self) -> Self::Target {
                self.iter().cloned().collect()
            }
        }
    };
}

impl_traits_to_primitive!(i8);
impl_traits_to_primitive!(i16);
impl_traits_to_primitive!(i32);
impl_traits_to_primitive!(i64);
impl_traits_to_primitive!(u8);
impl_traits_to_primitive!(u16);
impl_traits_to_primitive!(u32);
impl_traits_to_primitive!(u64);
impl_traits_to_primitive!(f32);
impl_traits_to_primitive!(f64);
impl_traits_to_primitive!(bool);

impl<T> Deref for FFISeq<T> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        if self.size == 0 {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.data, self.len()) }
    }
}

impl<T> AsRef<[T]> for FFISeq<T> {
    fn as_ref(&self) -> &[T] {
        self
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct OwnedFFISeq<T> {
    data: *mut T,
    size: usize,
    capacity: usize,
}

impl<T> OwnedFFISeq<T> {
    /// Extracts a slice.
    pub fn as_slice(&self) -> &[T] {
        // `from_rust` stores a null `data` pointer for an empty sequence, and
        // `slice::from_raw_parts` is UB with a null pointer even for length 0.
        // Guard the empty case, mirroring `FFISeq::deref`.
        if self.size == 0 {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.data, self.len()) }
    }

    /// Returns the length of the sequence.
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the sequence has a length of 0.
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<T> FFIFromRust for OwnedFFISeq<T>
where
    T: FFIFromRust,
{
    type From = Vec<T::From>;

    unsafe fn from_rust(vec: &Self::From) -> Self {
        if vec.is_empty() {
            Self {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            }
        } else {
            let mut new_vec = vec
                .iter()
                .map(|v| unsafe { FFIFromRust::from_rust(v) })
                .collect::<Vec<_>>();
            new_vec.shrink_to_fit();
            assert_eq!(new_vec.len(), new_vec.capacity());
            let mut new_vec = ManuallyDrop::new(new_vec);
            Self {
                data: new_vec.as_mut_ptr(),
                size: new_vec.len(),
                capacity: new_vec.len(),
            }
        }
    }
}

impl<T> Drop for OwnedFFISeq<T> {
    fn drop(&mut self) {
        // `from_rust` stores a null `data` pointer for an empty sequence (as
        // rosidl requires). `Vec::from_raw_parts(null, 0, 0)` is undefined
        // behavior — it feeds the null pointer to `Unique::new_unchecked`,
        // which aborts under debug assertions — so skip reconstruction when
        // there is nothing to free. Mirrors the `as_slice` empty-case guard.
        if self.data.is_null() {
            return;
        }
        unsafe { Vec::from_raw_parts(self.data, self.size, self.capacity) };
    }
}

/// Temporally borrowed buffer from `Vec<T>`
#[repr(C)]
#[derive(Debug)]
pub struct RefFFISeq<T> {
    data: *mut T,
    size: usize,
    capacity: usize,
}

impl<T> RefFFISeq<T> {
    /// Extracts a slice.
    pub fn as_slice(&self) -> &[T] {
        // `from_rust` stores a null `data` pointer for an empty sequence, and
        // `slice::from_raw_parts` is UB with a null pointer even for length 0.
        // Guard the empty case, mirroring `FFISeq::deref`.
        if self.size == 0 {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.data, self.len()) }
    }

    /// Returns the length of the sequence.
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the sequence has a length of 0.
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<T> FFIFromRust for RefFFISeq<T> {
    type From = Vec<T>;

    unsafe fn from_rust(vec: &Self::From) -> Self {
        if vec.is_empty() {
            Self {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            }
        } else {
            Self {
                data: vec.as_ptr() as *mut _,
                size: vec.len(),
                capacity: vec.len(),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // `from_rust` deliberately stores a null `data` pointer for an empty
    // sequence (see the `vec.is_empty()` branches). `slice::from_raw_parts`
    // is undefined behavior when given a null pointer even for a zero
    // length, so `as_slice` must special-case the empty sequence — exactly
    // as `FFISeq::deref` already does. These tests exercise the empty and
    // non-empty paths of that guard.
    //
    // `RefFFISeq` is used because it borrows (no `Drop`) and its `from_rust`
    // is unconstrained in `T`, so the empty case can be built safely.

    #[test]
    fn ref_seq_as_slice_empty_is_safe() {
        let empty: Vec<i32> = Vec::new();
        let seq = unsafe { RefFFISeq::from_rust(&empty) };
        assert!(seq.is_empty());
        assert_eq!(seq.as_slice(), &[] as &[i32]);
    }

    #[test]
    fn ref_seq_as_slice_non_empty_roundtrips() {
        let data = vec![1_i32, 2, 3];
        let seq = unsafe { RefFFISeq::from_rust(&data) };
        assert_eq!(seq.as_slice(), data.as_slice());
    }

    #[test]
    fn owned_seq_as_slice_empty_is_safe() {
        // Build the empty representation `from_rust` produces (null data,
        // size 0) directly, since `OwnedFFISeq::from_rust` requires
        // `T: FFIFromRust`. Letting it drop at end of scope also exercises the
        // `Drop` guard (`Vec::from_raw_parts` must not run on the null
        // pointer), so no `mem::forget` is needed.
        let seq: OwnedFFISeq<i32> = OwnedFFISeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        assert!(seq.is_empty());
        assert_eq!(seq.as_slice(), &[] as &[i32]);
    }

    #[test]
    fn owned_seq_empty_drop_is_safe() {
        // `from_rust` stores a null `data` pointer for an empty sequence.
        // `Drop` must not feed it to `Vec::from_raw_parts`, which reaches
        // `Unique::new_unchecked(null)` and aborts under debug assertions.
        // Building the empty representation and dropping it exercises the
        // guard directly (this is the more reachable case: `OwnedFFISeq` is
        // what msg-gen emits for every non-primitive sequence field).
        let seq: OwnedFFISeq<i32> = OwnedFFISeq {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        drop(seq);
    }

    #[test]
    fn owned_seq_non_empty_drop_frees() {
        // The guard only skips the null/empty case: a non-null sequence must
        // still be reconstructed and freed. Build one from a leaked `Vec`
        // (matching what `from_rust` stores) and let it drop; miri/ASan would
        // flag a leak or double-free if the guard were too broad.
        let mut v = ManuallyDrop::new(vec![1_i32, 2, 3]);
        let seq: OwnedFFISeq<i32> = OwnedFFISeq {
            data: v.as_mut_ptr(),
            size: v.len(),
            capacity: v.capacity(),
        };
        drop(seq);
    }
}
