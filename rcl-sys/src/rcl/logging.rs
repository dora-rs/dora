//! API in rcl/logging.h

use std::os::raw::{c_char, c_int};

use crate::*;

pub type rcl_logging_output_handler_t = rcutils_logging_output_handler_t;

extern "C" {
    /// Configure the logging system.
    pub fn rcl_logging_configure(
        global_args: *const rcl_arguments_t,
        allocator: *const rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Configure the logging system with the provided output handler.
    pub fn rcl_logging_configure_with_output_handler(
        global_args: *const rcl_arguments_t,
        allocator: *const rcl_allocator_t,
        output_handler: rcl_logging_output_handler_t,
    ) -> rcl_ret_t;

    /// This function should be called to tear down the logging setup by the configure function.
    pub fn rcl_logging_fini() -> rcl_ret_t;

    /// See if logging rosout is enabled.
    pub fn rcl_logging_rosout_enabled() -> bool;

    /// Default output handler used by rcl.
    pub fn rcl_logging_multiple_output_handler(
        location: *const rcutils_log_location_t,
        severity: c_int,
        name: *const c_char,
        timestamp: rcutils_time_point_value_t,
        format: *const c_char,
        args: *mut va_list,
    );
}
