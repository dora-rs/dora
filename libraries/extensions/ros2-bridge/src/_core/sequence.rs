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
        self.iter().map(|v| v.to_rust()).collect()
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
                .map(|v| FFIFromRust::from_rust(v))
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
