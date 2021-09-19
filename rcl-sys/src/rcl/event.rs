//! API in rcl/event.h

use std::os::raw::c_void;

use crate::*;

/// Enumeration of all of the publisher events that may fire.
#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum rcl_publisher_event_type_t {
    RCL_PUBLISHER_OFFERED_DEADLINE_MISSED = 0,
    RCL_PUBLISHER_LIVELINESS_LOST = 1,
    RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS = 2,
}

/// Enumeration of all of the subscription events that may fire.
#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum rcl_subscription_event_type_t {
    RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED = 0,
    RCL_SUBSCRIPTION_LIVELINESS_CHANGED = 1,
    RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS = 2,
    #[cfg(feature = "galactic+")]
    RCL_SUBSCRIPTION_MESSAGE_LOST = 3,
}

/// Internal rcl implementation struct.
#[repr(C)]
#[derive(Debug)]
struct rcl_event_impl_t {
    _unused: [u8; 0],
}
/// Structure which encapsulates a ROS QoS event handle.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_event_t {
    /// Pointer to the event implementation
    impl_: *mut rcl_event_impl_t,
}

extern "C" {
    /// Return a [rcl_event_t] struct with members set to `NULL`.
    pub fn rcl_get_zero_initialized_event() -> rcl_event_t;

    /// Initialize an [rcl_event_t] with a publisher.
    pub fn rcl_publisher_event_init(
        event: *mut rcl_event_t,
        publisher: *const rcl_publisher_t,
        event_type: rcl_publisher_event_type_t,
    ) -> rcl_ret_t;

    /// Initialize an [rcl_event_t] with a subscription.
    pub fn rcl_subscription_event_init(
        event: *mut rcl_event_t,
        subscription: *const rcl_subscription_t,
        event_type: rcl_subscription_event_type_t,
    ) -> rcl_ret_t;

    /// Take an event from the event handle.
    pub fn rcl_take_event(event: *const rcl_event_t, event_info: *mut c_void) -> rcl_ret_t;

    /// Finalize an event.
    pub fn rcl_event_fini(event: *mut rcl_event_t) -> rcl_ret_t;
}
