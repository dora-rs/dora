//! API in rcutils/types/*.h

use std::os::raw::c_char;

pub type rcutils_ret_t = std::os::raw::c_int;

use super::rcutils_allocator_t;

#[repr(C)]
#[derive(Debug)]
pub struct rcutils_string_array_t {
    pub size: usize,
    pub data: *mut *mut c_char,
    pub allocator: rcutils_allocator_t,
}

extern "C" {
    /// Return an empty string array struct.
    pub fn rcutils_get_zero_initialized_string_array() -> rcutils_string_array_t;

    /// Initialize a string array with a given size.
    pub fn rcutils_string_array_init(
        string_array: *mut rcutils_string_array_t,
        size: usize,
        allocator: *const rcutils_allocator_t,
    ) -> rcutils_ret_t;

    /// Finalize a string array, reclaiming all resources.
    pub fn rcutils_string_array_fini(string_array: *mut rcutils_string_array_t) -> rcutils_ret_t;
}

#[repr(C)]
#[derive(Debug)]
struct rcutils_string_map_impl_t {
    _unused: [u8; 0],
}

#[repr(C)]
#[derive(Debug)]
pub struct rcutils_string_map_t {
    impl_: *mut rcutils_string_map_impl_t,
}

extern "C" {
    pub fn rcutils_get_zero_initialized_string_map() -> rcutils_string_map_t;

    /// Initialize a rcutils_string_map_t, allocating space for given capacity.
    pub fn rcutils_string_map_init(
        string_map: *mut rcutils_string_map_t,
        initial_capacity: usize,
        allocator: rcutils_allocator_t,
    ) -> rcutils_ret_t;

    /// Finalize the previously initialized string map struct.
    pub fn rcutils_string_map_fini(string_map: *mut rcutils_string_map_t) -> rcutils_ret_t;
}

#[repr(C)]
#[derive(Debug)]
pub struct rcutils_uint8_array_t {
    pub buffer: *mut u8,
    pub buffer_length: usize,
    pub buffer_capacity: usize,
    pub allocator: rcutils_allocator_t,
}

extern "C" {
    /// Return a zero initialized uint8 array struct.
    pub fn rcutils_get_zero_initialized_uint8_array() -> rcutils_uint8_array_t;

    /// Initialize a zero initialized uint8 array struct.
    pub fn rcutils_uint8_array_init(
        uint8_array: *mut rcutils_uint8_array_t,
        buffer_capacity: usize,
        allocator: *const rcutils_allocator_t,
    ) -> rcutils_ret_t;

    /// Finalize a uint8 array struct.
    pub fn rcutils_uint8_array_fini(uint8_array: *mut rcutils_uint8_array_t) -> rcutils_ret_t;
}
