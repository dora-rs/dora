//! API in rcl/init_options.h

use crate::*;

#[repr(C)]
#[derive(Debug)]
struct rcl_init_options_impl_t {
    _unused: [u8; 0],
}
/// Encapsulation of init options and implementation defined init options.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_init_options_t {
    /// Implementation specific pointer.
    impl_: *mut rcl_init_options_impl_t,
}

extern "C" {
    /// Return a zero initialized rcl_init_options_t struct.
    pub fn rcl_get_zero_initialized_init_options() -> rcl_init_options_t;

    /// Initialize given init_options with the default values and implementation specific values.
    pub fn rcl_init_options_init(
        init_options: *mut rcl_init_options_t,
        allocator: rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize the given init_options.
    pub fn rcl_init_options_fini(init_options: *mut rcl_init_options_t) -> rcl_ret_t;
}
