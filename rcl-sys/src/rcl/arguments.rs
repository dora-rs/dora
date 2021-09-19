//! API in rcl/arguments.h

use crate::*;

#[repr(C)]
#[derive(Debug)]
struct rcl_arguments_impl_t {
    _unused: [u8; 0],
}

/// Hold output of parsing command line arguments.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_arguments_t {
    /// Private implementation pointer.
    impl_: *mut rcl_arguments_impl_t,
}

extern "C" {
    /// Return all parameter overrides parsed from the command line.
    pub fn rcl_arguments_get_param_overrides(
        arguments: *const rcl_arguments_t,
        parameter_overrides: *mut *mut rcl_params_t,
    ) -> rcl_ret_t;
}
