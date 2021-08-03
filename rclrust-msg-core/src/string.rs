use std::ffi::{CStr, CString};
use std::os::raw::c_char;

use widestring::{U16CStr, U16CString, U16String};

use crate::traits::{FFIFromRust, FFIToRust, ZeroInit};

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
}

impl ZeroInit for FFIString {
    fn zero_init() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        }
    }
}

impl FFIToRust for FFIString {
    type Target = String;

    unsafe fn to_rust(&self) -> Self::Target {
        if self.is_empty() {
            "".to_string()
        } else {
            CStr::from_ptr(self.data)
                .to_str()
                .expect("CStr::to_str failed")
                .to_string()
        }
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

impl ZeroInit for OwnedFFIString {
    fn zero_init() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        }
    }
}

impl FFIFromRust for OwnedFFIString {
    type From = String;

    fn from_rust(string: &Self::From) -> Self {
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
            CString::from_raw(self.data);
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

impl ZeroInit for FFIWString {
    fn zero_init() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        }
    }
}

impl FFIToRust for FFIWString {
    type Target = U16String;

    unsafe fn to_rust(&self) -> Self::Target {
        if self.is_empty() {
            Self::Target::new()
        } else {
            U16CStr::from_ptr_str(self.data).to_ustring()
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

impl ZeroInit for OwnedFFIWString {
    fn zero_init() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        }
    }
}

impl FFIFromRust for OwnedFFIWString {
    type From = U16String;

    fn from_rust(string: &Self::From) -> Self {
        let cstring = U16CString::new(string.clone()).expect("U16CString::new failed");
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
        unsafe { U16CString::from_raw(self.data) };
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn owned_ffi_string_new() {
        let string = "abcde".into();
        let cstring = OwnedFFIString::from_rust(&string);
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
        let cwstring = OwnedFFIWString::from_rust(&wstring);
        let native_wstring = FFIWString {
            data: cwstring.data,
            size: cwstring.size,
            capacity: cwstring.capacity,
        };

        assert_eq!(wstring, unsafe { native_wstring.to_rust() });
    }
}
