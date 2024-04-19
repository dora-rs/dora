use std::{
    ffi::{CStr, CString},
    ops::{Deref, DerefMut},
    os::raw::c_char,
};

use widestring::{U16CStr, U16CString};

use super::traits::{FFIFromRust, FFIToRust};

#[derive(
    Debug,
    Default,
    Clone,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    serde::Serialize,
    serde::Deserialize,
)]
#[serde(from = "Vec<u16>", into = "Vec<u16>")]
#[repr(transparent)]
pub struct U16String(widestring::U16String);

impl U16String {
    pub fn new() -> Self {
        Self(widestring::U16String::new())
    }

    #[allow(clippy::should_implement_trait)]
    pub fn from_str(arg: &str) -> U16String {
        Self(widestring::U16String::from_str(arg))
    }
}

impl Deref for U16String {
    type Target = widestring::U16String;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for U16String {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl AsRef<widestring::U16Str> for U16String {
    fn as_ref(&self) -> &widestring::U16Str {
        self.0.as_ref()
    }
}

impl From<U16String> for Vec<u16> {
    fn from(value: U16String) -> Self {
        value.0.into_vec()
    }
}

impl From<Vec<u16>> for U16String {
    fn from(value: Vec<u16>) -> Self {
        Self(value.into())
    }
}

/// An array of 8-bit characters terminated by a null character.
#[repr(C)]
#[derive(Debug)]
pub struct FFIString {
    data: *mut c_char,
    size: usize,
    capacity: usize,
}

impl FFIString {
    /// Returns the length of the string (excluding the null byte)
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the string has a length of 0.
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub unsafe fn to_str(&self) -> Result<&str, std::str::Utf8Error> {
        if self.is_empty() {
            Ok("")
        } else {
            CStr::from_ptr(self.data).to_str()
        }
    }
}

impl FFIToRust for FFIString {
    type Target = String;

    unsafe fn to_rust(&self) -> Self::Target {
        self.to_str().expect("CStr::to_str failed").to_string()
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct OwnedFFIString {
    data: *mut c_char,
    size: usize,
    capacity: usize,
}

impl OwnedFFIString {
    /// Returns the length of the string (excluding the null byte)
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the string has a length of 0.
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl FFIFromRust for OwnedFFIString {
    type From = String;

    unsafe fn from_rust(string: &Self::From) -> Self {
        let cstring = CString::new(string.clone()).expect("CString::new failed");
        let len = cstring.as_bytes().len();
        Self {
            data: cstring.into_raw(),
            size: len,
            capacity: len + 1,
        }
    }
}

impl Drop for OwnedFFIString {
    fn drop(&mut self) {
        unsafe {
            std::mem::drop(CString::from_raw(self.data));
        }
    }
}

/// An array of 16-bit characters terminated by a null character.
#[repr(C)]
#[derive(Debug)]
pub struct FFIWString {
    data: *mut u16,
    size: usize,
    capacity: usize,
}

impl FFIWString {
    /// Returns the length of the string (excluding the null byte)
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the sequence has a length of 0.
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl FFIToRust for FFIWString {
    type Target = U16String;

    unsafe fn to_rust(&self) -> Self::Target {
        if self.is_empty() {
            Self::Target::new()
        } else {
            U16String(U16CStr::from_ptr_str(self.data).to_ustring())
        }
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct OwnedFFIWString {
    data: *mut u16,
    size: usize,
    capacity: usize,
}

impl OwnedFFIWString {
    /// Returns the length of the string (excluding the null byte)
    pub const fn len(&self) -> usize {
        self.size
    }

    /// Returns `true` if the sequence has a length of 0.
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl FFIFromRust for OwnedFFIWString {
    type From = U16String;

    unsafe fn from_rust(string: &Self::From) -> Self {
        let cstring = U16CString::from_ustr(string).expect("U16CString::new failed");
        let len = cstring.len();
        Self {
            data: cstring.into_raw(),
            size: len,
            capacity: len + 1,
        }
    }
}

impl Drop for OwnedFFIWString {
    fn drop(&mut self) {
        unsafe {
            std::mem::drop(U16CString::from_raw(self.data));
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn owned_ffi_string_new() {
        let string = "abcde".into();
        let cstring = unsafe { OwnedFFIString::from_rust(&string) };
        let native_string = FFIString {
            data: cstring.data,
            size: cstring.size,
            capacity: cstring.capacity,
        };

        assert_eq!(string, unsafe { native_string.to_rust() });
    }

    #[test]
    fn owned_ffi_wstring_new() {
        let wstring = U16String::from_str("あいうえお");
        let cwstring = unsafe { OwnedFFIWString::from_rust(&wstring) };
        let native_wstring = FFIWString {
            data: cwstring.data,
            size: cwstring.size,
            capacity: cwstring.capacity,
        };

        assert_eq!(wstring, unsafe { native_wstring.to_rust() });
    }
}
