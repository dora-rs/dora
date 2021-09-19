//! API in rcl/time.h

use std::os::raw::c_void;

use crate::*;

/// A single point in time, measured in nanoseconds since the Unix epoch.
pub type rcl_time_point_value_t = rcutils_time_point_value_t;
/// A duration of time, measured in nanoseconds.
pub type rcl_duration_value_t = rcutils_duration_value_t;
#[repr(u32)]

/// Time source type, used to indicate the source of a time measurement.
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum RclClockType {
    Uninitialized = 0,
    RosTime = 1,
    SystemTime = 2,
    SteadyTime = 3,
}

/// A duration of time, measured in nanoseconds and its source.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_duration_t {
    /// Duration in nanoseconds and its source.
    pub nanoseconds: rcl_duration_value_t,
}

#[repr(u32)]
/// Enumeration to describe the type of time jump.
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum rcl_clock_change_t {
    /// The source before and after the jump is ROS_TIME.
    RCL_ROS_TIME_NO_CHANGE = 1,
    /// The source switched to ROS_TIME from SYSTEM_TIME.
    RCL_ROS_TIME_ACTIVATED = 2,
    /// The source switched to SYSTEM_TIME from ROS_TIME.
    RCL_ROS_TIME_DEACTIVATED = 3,
    /// The source before and after the jump is SYSTEM_TIME.
    RCL_SYSTEM_TIME_NO_CHANGE = 4,
}

/// Struct to describe a jump in time.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_time_jump_t {
    /// Indicate whether or not the source of time changed.
    pub clock_change: rcl_clock_change_t,
    /// The new time minus the last time before the jump.
    pub delta: rcl_duration_t,
}

/// Signature of a time jump callback.
pub type rcl_jump_callback_t = Option<
    unsafe extern "C" fn(
        time_jump: *const rcl_time_jump_t,
        before_jump: bool,
        user_data: *mut c_void,
    ),
>;

/// Describe the prerequisites for calling a time jump callback.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_jump_threshold_t {
    /// True to call callback when the clock type changes.
    pub on_clock_change: bool,
    /// A positive duration indicating the minimum jump forwards to be considered exceeded, or zero
    /// to disable.
    pub min_forward: rcl_duration_t,
    /// A negative duration indicating the minimum jump backwards to be considered exceeded, or zero
    /// to disable.
    pub min_backward: rcl_duration_t,
}

/// Struct to describe an added callback.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_jump_callback_info_t {
    /// Callback to fucntion.
    pub callback: rcl_jump_callback_t,
    /// Threshold to decide when to call the callback.
    pub threshold: rcl_jump_threshold_t,
    /// Pointer passed to the callback.
    pub user_data: *mut c_void,
}

/// Encapsulation of a time source.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_clock_t {
    /// Clock type
    pub type_: RclClockType,
    /// An array of added jump callbacks.
    pub jump_callbacks: *mut rcl_jump_callback_info_t,
    /// Number of callbacks in jump_callbacks.
    pub num_jump_callbacks: usize,
    /// Pointer to get_now function
    pub get_now: Option<
        unsafe extern "C" fn(data: *mut c_void, now: *mut rcl_time_point_value_t) -> rcl_ret_t,
    >,
    /// Clock storage
    pub data: *mut c_void,
    /// Custom allocator used for internal allocations.
    pub allocator: rcl_allocator_t,
}

/// A single point in time, measured in nanoseconds, the reference point is based on the source.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_time_point_t {
    /// Nanoseconds of the point in time
    pub nanoseconds: rcl_time_point_value_t,
    /// Clock type of the point in time
    pub clock_type: RclClockType,
}

extern "C" {
    /// Check if the clock has valid values.
    pub fn rcl_clock_valid(clock: *mut rcl_clock_t) -> bool;

    /// Initialize a clock based on the passed type.
    pub fn rcl_clock_init(
        clock_type: RclClockType,
        clock: *mut rcl_clock_t,
        allocator: *mut rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize a clock.
    pub fn rcl_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t;

    /// Initialize a clock as a RCL_ROS_TIME time source.
    pub fn rcl_ros_clock_init(
        clock: *mut rcl_clock_t,
        allocator: *mut rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize a clock as a `RCL_ROS_TIME` time source.
    pub fn rcl_ros_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t;

    /// Initialize a clock as a `RCL_STEADY_TIME` time source.
    pub fn rcl_steady_clock_init(
        clock: *mut rcl_clock_t,
        allocator: *mut rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize a clock as a `RCL_STEADY_TIME` time source.
    pub fn rcl_steady_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t;

    /// Initialize a clock as a `RCL_SYSTEM_TIME` time source.
    pub fn rcl_system_clock_init(
        clock: *mut rcl_clock_t,
        allocator: *mut rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize a clock as a `RCL_SYSTEM_TIME` time source.
    pub fn rcl_system_clock_fini(clock: *mut rcl_clock_t) -> rcl_ret_t;

    /// Compute the difference between two time points
    pub fn rcl_difference_times(
        start: *mut rcl_time_point_t,
        finish: *mut rcl_time_point_t,
        delta: *mut rcl_duration_t,
    ) -> rcl_ret_t;

    /// Fill the time point value with the current value of the associated clock.
    pub fn rcl_clock_get_now(
        clock: *mut rcl_clock_t,
        time_point_value: *mut rcl_time_point_value_t,
    ) -> rcl_ret_t;

    /// Enable the ROS time abstraction override.
    pub fn rcl_enable_ros_time_override(clock: *mut rcl_clock_t) -> rcl_ret_t;

    /// Disable the ROS time abstraction override.
    pub fn rcl_disable_ros_time_override(clock: *mut rcl_clock_t) -> rcl_ret_t;

    /// Check if the `RCL_ROS_TIME` time source has the override enabled.
    pub fn rcl_is_enabled_ros_time_override(
        clock: *mut rcl_clock_t,
        is_enabled: *mut bool,
    ) -> rcl_ret_t;

    /// Set the current time for this `RCL_ROS_TIME` time source.
    pub fn rcl_set_ros_time_override(
        clock: *mut rcl_clock_t,
        time_value: rcl_time_point_value_t,
    ) -> rcl_ret_t;

    /// Add a callback to be called when a time jump exceeds a threshold.
    pub fn rcl_clock_add_jump_callback(
        clock: *mut rcl_clock_t,
        threshold: rcl_jump_threshold_t,
        callback: rcl_jump_callback_t,
        user_data: *mut c_void,
    ) -> rcl_ret_t;

    /// Remove a previously added time jump callback.
    pub fn rcl_clock_remove_jump_callback(
        clock: *mut rcl_clock_t,
        callback: rcl_jump_callback_t,
        user_data: *mut c_void,
    ) -> rcl_ret_t;
}
