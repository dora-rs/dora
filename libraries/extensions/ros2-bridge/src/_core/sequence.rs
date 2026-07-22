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
        // `slice::from_raw_parts` requires a non-null (aligned) pointer even for
        // a zero length — so guard the empty case, matching `FFISeq::deref`.
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
        // `from_rust` stores a null `data` pointer for an empty input (see the
        // `vec.is_empty()` branch above). `Vec::from_raw_parts` requires a
        // pointer that came from a `Vec` allocation, and `Vec`'s internal
        // pointer is a `NonNull`; passing null is library-level UB (flagged by
        // Miri) even though a 0-capacity `Vec` performs no deallocation. There
        // is nothing to free in that case, so skip the reconstruction entirely.
        if self.capacity == 0 {
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
        // `slice::from_raw_parts` requires a non-null (aligned) pointer even for
        // a zero length — so guard the empty case, matching `FFISeq::deref`.
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
    use super::super::string::OwnedFFIString;
    use super::*;

    /// Regression test: `from_rust` stores a null `data` pointer for an empty
    /// sequence, so neither `as_slice` (`slice::from_raw_parts(null, 0)`) nor
    /// `Drop` (`Vec::from_raw_parts(null, 0, 0)`) may reconstruct from it —
    /// both are library-level UB that Miri flags even though a zero-length/
    /// zero-capacity reconstruction touches no memory. Run under
    /// `cargo +nightly miri test` to exercise the checks.
    #[test]
    fn empty_owned_seq_slice_and_drop_without_ub() {
        let empty: Vec<String> = Vec::new();
        // SAFETY: `empty` is a valid `Vec<String>` (the `From` type of
        // `OwnedFFIString`); the produced sequence borrows nothing from it.
        let seq: OwnedFFISeq<OwnedFFIString> = unsafe { FFIFromRust::from_rust(&empty) };
        assert!(seq.is_empty());
        assert_eq!(seq.len(), 0);
        // `as_slice` on the empty (null-backed) sequence must not deref null.
        assert!(seq.as_slice().is_empty());
        // Dropping here must not hit `Vec::from_raw_parts(null, 0, 0)`.
        drop(seq);
    }

    /// `RefFFISeq` shares the null-for-empty representation, so its `as_slice`
    /// must also guard the empty case rather than deref a null pointer.
    #[test]
    fn empty_ref_seq_slice_without_ub() {
        let empty: Vec<u8> = Vec::new();
        // SAFETY: `empty` outlives the borrow held by the produced sequence.
        let seq: RefFFISeq<u8> = unsafe { FFIFromRust::from_rust(&empty) };
        assert!(seq.is_empty());
        assert!(seq.as_slice().is_empty());
    }
}
