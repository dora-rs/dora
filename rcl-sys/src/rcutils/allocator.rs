//! API in rcutils/allocator.h

use std::os::raw::c_void;

/// Encapsulation of an allocator.
#[repr(C)]
#[derive(Debug)]
pub struct rcutils_allocator_t {
    /// Allocate memory, given a size and the `state` pointer.
    pub allocate: Option<unsafe extern "C" fn(size: usize, state: *mut c_void) -> *mut c_void>,
    /// Deallocate previously allocated memory, mimicking free().
    pub deallocate: Option<unsafe extern "C" fn(pointer: *mut c_void, state: *mut c_void)>,
    /// Reallocate if possible, otherwise it deallocates and allocates.
    pub reallocate: Option<
        unsafe extern "C" fn(pointer: *mut c_void, size: usize, state: *mut c_void) -> *mut c_void,
    >,
    /// Allocate memory with all elements set to zero, given a number of elements and their size.
    pub zero_allocate: Option<
        unsafe extern "C" fn(
            number_of_elements: usize,
            size_of_element: usize,
            state: *mut c_void,
        ) -> *mut c_void,
    >,
    /// Implementation defined state storage.
    pub state: *mut c_void,
}

extern "C" {
    /// Return a zero initialized allocator.
    pub fn rcutils_get_zero_initialized_allocator() -> rcutils_allocator_t;

    /// Return a properly initialized rcutils_allocator_t with default values.
    pub fn rcutils_get_default_allocator() -> rcutils_allocator_t;

    /// Return true if the given allocator has non-null function pointers.
    pub fn rcutils_allocator_is_valid(allocator: *const rcutils_allocator_t) -> bool;

    /// Emulate the behavior of [reallocf](https://linux.die.net/man/3/reallocf).
    pub fn rcutils_reallocf(
        pointer: *mut c_void,
        size: usize,
        allocator: *mut rcutils_allocator_t,
    ) -> *mut c_void;
}
