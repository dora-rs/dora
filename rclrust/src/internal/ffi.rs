use std::ffi::{CStr, CString};
use std::os::raw::c_char;

pub unsafe trait SizedFromCChar: Sized {
    unsafe fn from_c_char(ptr: *const c_char) -> Option<Self>;
}

unsafe impl SizedFromCChar for CString {
    unsafe fn from_c_char(ptr: *const c_char) -> Option<Self> {
        CStr::from_c_char(ptr).map(|v| v.into())
    }
}

unsafe impl SizedFromCChar for String {
    unsafe fn from_c_char(ptr: *const c_char) -> Option<Self> {
        str::from_c_char(ptr).map(|v| v.into())
    }
}

pub unsafe trait FromCChar {
    unsafe fn from_c_char<'a>(ptr: *const c_char) -> Option<&'a Self>;
}

unsafe impl FromCChar for CStr {
    unsafe fn from_c_char<'a>(ptr: *const c_char) -> Option<&'a Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self::from_ptr(ptr))
        }
    }
}

unsafe impl FromCChar for str {
    unsafe fn from_c_char<'a>(ptr: *const c_char) -> Option<&'a Self> {
        CStr::from_c_char(ptr).map(|v| v.to_str().expect("expect UTF-8 string"))
    }
}
