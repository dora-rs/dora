//! API in rmw/names_and_types.h

use super::rmw_ret_t;
use crate::rcutils::{rcutils_allocator_t, rcutils_string_array_t};

/// Associative array of topic or service names and types.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_names_and_types_t {
    /// Array of names
    pub names: rcutils_string_array_t,
    /// Dynamic array of arrays of type names, with the same length as `names`
    pub types: *mut rcutils_string_array_t,
}

extern "C" {
    /// Return a zero initialized array of names and types.
    pub fn rmw_get_zero_initialized_names_and_types() -> rmw_names_and_types_t;

    /// Check that the given `names_and_types` array is zero initialized.
    pub fn rmw_names_and_types_check_zero(names_and_types: *mut rmw_names_and_types_t)
        -> rmw_ret_t;

    /// Initialize an array of names and types.
    pub fn rmw_names_and_types_init(
        names_and_types: *mut rmw_names_and_types_t,
        size: usize,
        allocator: *mut rcutils_allocator_t,
    ) -> rmw_ret_t;

    /// Finalize an array of names and types.
    pub fn rmw_names_and_types_fini(names_and_types: *mut rmw_names_and_types_t) -> rmw_ret_t;
}
