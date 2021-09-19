//! API in rcl/timer.h

use crate::*;

#[repr(C)]
#[derive(Debug)]
struct rcl_timer_impl_t {
    _unused: [u8; 0],
}
/// Structure which encapsulates a ROS Timer.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_timer_t {
    /// Private implementation pointer.
    impl_: *mut rcl_timer_impl_t,
}

/// User callback signature for timers.
pub type rcl_timer_callback_t = Option<unsafe extern "C" fn(arg1: *mut rcl_timer_t, arg2: i64)>;

extern "C" {
    /// Return a zero initialized timer.
    pub fn rcl_get_zero_initialized_timer() -> rcl_timer_t;

    /// Initialize a timer.
    pub fn rcl_timer_init(
        timer: *mut rcl_timer_t,
        clock: *mut rcl_clock_t,
        context: *mut rcl_context_t,
        period: i64,
        callback: rcl_timer_callback_t,
        allocator: rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize a timer.
    pub fn rcl_timer_fini(timer: *mut rcl_timer_t) -> rcl_ret_t;

    /// Call the timer's callback and set the last call time.
    pub fn rcl_timer_call(timer: *mut rcl_timer_t) -> rcl_ret_t;

    /// Retrieve the clock of the timer.
    pub fn rcl_timer_clock(timer: *mut rcl_timer_t, clock: *mut *mut rcl_clock_t) -> rcl_ret_t;

    /// Calculates whether or not the timer should be called.
    pub fn rcl_timer_is_ready(timer: *const rcl_timer_t, is_ready: *mut bool) -> rcl_ret_t;

    /// Calculate and retrieve the time until the next call in nanoseconds.
    pub fn rcl_timer_get_time_until_next_call(
        timer: *const rcl_timer_t,
        time_until_next_call: *mut i64,
    ) -> rcl_ret_t;

    /// Retrieve the time since the previous call to rcl_timer_call() occurred.
    pub fn rcl_timer_get_time_since_last_call(
        timer: *const rcl_timer_t,
        time_since_last_call: *mut i64,
    ) -> rcl_ret_t;

    /// Retrieve the period of the timer.
    pub fn rcl_timer_get_period(timer: *const rcl_timer_t, period: *mut i64) -> rcl_ret_t;

    /// Exchange the period of the timer and return the previous period.
    pub fn rcl_timer_exchange_period(
        timer: *const rcl_timer_t,
        new_period: i64,
        old_period: *mut i64,
    ) -> rcl_ret_t;

    /// Return the current timer callback.
    pub fn rcl_timer_get_callback(timer: *const rcl_timer_t) -> rcl_timer_callback_t;

    /// Exchange the current timer callback and return the current callback.
    pub fn rcl_timer_exchange_callback(
        timer: *mut rcl_timer_t,
        new_callback: rcl_timer_callback_t,
    ) -> rcl_timer_callback_t;

    /// Cancel a timer.
    pub fn rcl_timer_cancel(timer: *mut rcl_timer_t) -> rcl_ret_t;

    /// Retrieve the canceled state of a timer.
    pub fn rcl_timer_is_canceled(timer: *const rcl_timer_t, is_canceled: *mut bool) -> rcl_ret_t;

    /// Reset a timer.
    pub fn rcl_timer_reset(timer: *mut rcl_timer_t) -> rcl_ret_t;

    /// Return the allocator for the timer.
    pub fn rcl_timer_get_allocator(timer: *const rcl_timer_t) -> *const rcl_allocator_t;

    /// Retrieve a guard condition used by the timer to wake the waitset when using ROSTime.
    pub fn rcl_timer_get_guard_condition(timer: *const rcl_timer_t) -> *mut rcl_guard_condition_t;
}
