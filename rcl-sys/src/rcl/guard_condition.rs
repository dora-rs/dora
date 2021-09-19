//! API in rcl/guard_condition.h

use crate::*;

/// Internal rcl guard condition implementation struct.
#[repr(C)]
#[derive(Debug)]
struct rcl_guard_condition_impl_t {
    _unused: [u8; 0],
}

/// Handle for a rcl guard condition.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_guard_condition_t {
    /// Context associated with this guard condition.
    pub context: *mut rcl_context_t,
    /// Pointer to the guard condition implementation
    impl_: *mut rcl_guard_condition_impl_t,
}

/// Options available for a rcl guard condition.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_guard_condition_options_t {
    /// Custom allocator for the guard condition, used for internal allocations.
    pub allocator: rcl_allocator_t,
}

extern "C" {
    /// Return a rcl_guard_condition_t struct with members set to `NULL`.
    pub fn rcl_get_zero_initialized_guard_condition() -> rcl_guard_condition_t;

    /// Initialize a rcl guard_condition.
    pub fn rcl_guard_condition_init(
        guard_condition: *mut rcl_guard_condition_t,
        context: *mut rcl_context_t,
        options: rcl_guard_condition_options_t,
    ) -> rcl_ret_t;

    /// Finalize a rcl_guard_condition_t.
    pub fn rcl_guard_condition_fini(guard_condition: *mut rcl_guard_condition_t) -> rcl_ret_t;

    /// Return the default options in a rcl_guard_condition_options_t struct.
    pub fn rcl_guard_condition_get_default_options() -> rcl_guard_condition_options_t;

    /// Trigger a rcl guard condition.
    pub fn rcl_trigger_guard_condition(guard_condition: *mut rcl_guard_condition_t) -> rcl_ret_t;

    /// Return the guard condition options.
    pub fn rcl_guard_condition_get_options(
        guard_condition: *const rcl_guard_condition_t,
    ) -> *const rcl_guard_condition_options_t;
}
