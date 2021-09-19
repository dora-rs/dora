//! API in rcl/init.h

use std::os::raw::{c_char, c_int};

use crate::*;

extern "C" {
    /// Initialization of rcl.
    pub fn rcl_init(
        argc: c_int,
        argv: *const *const c_char,
        options: *const rcl_init_options_t,
        context: *mut rcl_context_t,
    ) -> rcl_ret_t;

    /// Shutdown a given rcl context.
    pub fn rcl_shutdown(context: *mut rcl_context_t) -> rcl_ret_t;
}
