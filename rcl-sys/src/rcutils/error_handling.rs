//! API in rcutils/error_handling.h

use std::os::raw::c_char;

/// "18446744073709551615"
const RCUTILS_ERROR_STATE_LINE_NUMBER_STR_MAX_LENGTH: usize = 20;
/// ', at ' + ':'
const RCUTILS_ERROR_FORMATTING_CHARACTERS: usize = 6;

/// max formatted string length
const RCUTILS_ERROR_MESSAGE_MAX_LENGTH: usize = 1024;

/// adjustable max length for user defined error message
/// remember "chained" errors will include previously specified file paths
/// e.g. "some error, at /path/to/a.c:42, at /path/to/b.c:42"
const RCUTILS_ERROR_STATE_MESSAGE_MAX_LENGTH: usize = 768;

const RCUTILS_ERROR_STATE_FILE_MAX_LENGTH: usize = RCUTILS_ERROR_MESSAGE_MAX_LENGTH
    - RCUTILS_ERROR_STATE_MESSAGE_MAX_LENGTH
    - RCUTILS_ERROR_STATE_LINE_NUMBER_STR_MAX_LENGTH
    - RCUTILS_ERROR_FORMATTING_CHARACTERS
    - 1;

/// Struct which encapsulates the error state set by `RCUTILS_SET_ERROR_MSG()`.
#[repr(C)]
#[derive(Debug)]
pub struct rcutils_error_state_t {
    /// User message storage, limited to RCUTILS_ERROR_STATE_MESSAGE_MAX_LENGTH characters.
    pub message: [c_char; RCUTILS_ERROR_STATE_MESSAGE_MAX_LENGTH],
    /// File name, limited to what's left from RCUTILS_ERROR_STATE_MAX_SIZE characters
    /// after subtracting storage for others.
    pub file: [c_char; RCUTILS_ERROR_STATE_FILE_MAX_LENGTH],
    /// Line number of error.
    pub line_number: u64,
}

extern "C" {
    /// Return an rcutils_error_state_t which was set with rcutils_set_error_state().
    pub fn rcutils_get_error_state() -> *const rcutils_error_state_t;

    /// Reset the error state by clearing any previously set error state.
    pub fn rcutils_reset_error();
}
