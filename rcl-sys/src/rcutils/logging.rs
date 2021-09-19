//! API in rcutils/logging.h

use std::os::raw::{c_char, c_int};

use super::{rcutils_allocator_t, rcutils_ret_t, rcutils_time_point_value_t};
use crate::va_list;

extern "C" {
    /// The flag if the logging system has been initialized.
    pub static mut g_rcutils_logging_initialized: bool;

    /// Initialize the logging system using the specified allocator.
    pub fn rcutils_logging_initialize_with_allocator(
        allocator: rcutils_allocator_t,
    ) -> rcutils_ret_t;

    /// Initialize the logging system.
    pub fn rcutils_logging_initialize() -> rcutils_ret_t;

    /// Shutdown the logging system.
    pub fn rcutils_logging_shutdown() -> rcutils_ret_t;
}

/// The structure identifying the caller location in the source code.
#[repr(C)]
#[derive(Debug)]
pub struct rcutils_log_location_t {
    /// The name of the function containing the log call.
    pub function_name: *const c_char,
    /// The name of the source file containing the log call.
    pub file_name: *const c_char,
    /// The line number containing the log call.
    pub line_number: usize,
}

#[repr(i32)]
/// The severity levels of log messages / loggers.
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum RcutilsLogSeverity {
    /// The unset log level
    Unset = 0,
    /// The debug log level
    Debug = 10,
    /// The info log level
    Info = 20,
    /// The warn log level
    Warn = 30,
    /// The error log level
    Error = 40,
    /// The fatal log level
    Fatal = 50,
}

impl RcutilsLogSeverity {
    pub fn try_from_int(from: i32) -> Option<Self> {
        Some(match from {
            v if v == Self::Unset as i32 => Self::Unset,
            v if v == Self::Debug as i32 => Self::Debug,
            v if v == Self::Info as i32 => Self::Info,
            v if v == Self::Warn as i32 => Self::Warn,
            v if v == Self::Error as i32 => Self::Error,
            v if v == Self::Fatal as i32 => Self::Fatal,
            _ => return None,
        })
    }
}

impl From<RcutilsLogSeverity> for i32 {
    fn from(from: RcutilsLogSeverity) -> Self {
        from as Self
    }
}

/// The function signature to log messages.
pub type rcutils_logging_output_handler_t = Option<
    unsafe extern "C" fn(
        arg1: *const rcutils_log_location_t,
        arg2: c_int,
        arg3: *const c_char,
        arg4: rcutils_time_point_value_t,
        arg5: *const c_char,
        arg6: *mut va_list,
    ),
>;
extern "C" {
    /// Get the default level for loggers.
    pub fn rcutils_logging_get_default_logger_level() -> c_int;

    /// Set the default severity level for loggers.
    pub fn rcutils_logging_set_default_logger_level(level: c_int);

    /// Get the severity level for a logger.
    pub fn rcutils_logging_get_logger_level(name: *const c_char) -> c_int;

    /// Set the severity level for a logger.
    pub fn rcutils_logging_set_logger_level(name: *const c_char, level: c_int) -> rcutils_ret_t;

    /// Determine if a logger is enabled for a severity level.
    pub fn rcutils_logging_logger_is_enabled_for(name: *const c_char, severity: c_int) -> bool;

    /// Determine the effective level for a logger.
    pub fn rcutils_logging_get_logger_effective_level(name: *const c_char) -> c_int;

    /// Log a message.
    pub fn rcutils_log(
        location: *const rcutils_log_location_t,
        severity: c_int,
        name: *const c_char,
        format: *const c_char,
    );

    /// The default output handler outputs log messages to the standard streams.
    pub fn rcutils_logging_console_output_handler(
        location: *const rcutils_log_location_t,
        severity: c_int,
        name: *const c_char,
        timestamp: rcutils_time_point_value_t,
        format: *const c_char,
        args: *mut va_list,
    );
}
