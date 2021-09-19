//! API in rcl/subscription.rs

use std::os::raw::{c_char, c_void};

use crate::*;

/// Internal rcl implementation struct.
#[repr(C)]
#[derive(Debug)]
struct rcl_subscription_impl_t {
    _unused: [u8; 0],
}

/// Structure which encapsulates a ROS Subscription.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_subscription_t {
    /// Pointer to the subscription implementation
    impl_: *mut rcl_subscription_impl_t,
}

/// Options available for a rcl subscription.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_subscription_options_t {
    /// Middleware quality of service settings for the subscription.
    pub qos: rmw_qos_profile_t,
    /// Custom allocator for the subscription, used for incidental allocations.
    pub allocator: rcl_allocator_t,
    /// rmw specific subscription options, e.g. the rmw implementation specific payload.
    pub rmw_subscription_options: rmw_subscription_options_t,
}

extern "C" {
    /// Return a [rcl_subscription_t] struct with members set to `NULL`.
    pub fn rcl_get_zero_initialized_subscription() -> rcl_subscription_t;

    /// Initialize a ROS subscription.
    pub fn rcl_subscription_init(
        subscription: *mut rcl_subscription_t,
        node: *const rcl_node_t,
        type_support: *const rosidl_message_type_support_t,
        topic_name: *const c_char,
        options: *const rcl_subscription_options_t,
    ) -> rcl_ret_t;

    /// Finalize a [rcl_subscription_t].
    pub fn rcl_subscription_fini(
        subscription: *mut rcl_subscription_t,
        node: *mut rcl_node_t,
    ) -> rcl_ret_t;

    /// Return the default subscription options in a [rcl_subscription_options_t].
    pub fn rcl_subscription_get_default_options() -> rcl_subscription_options_t;

    /// Take a ROS message from a topic using a rcl subscription.
    pub fn rcl_take(
        subscription: *const rcl_subscription_t,
        ros_message: *mut c_void,
        message_info: *mut rmw_message_info_t,
        allocation: *mut rmw_subscription_allocation_t,
    ) -> rcl_ret_t;

    /// Take a serialized raw message from a topic using a rcl subscription.
    pub fn rcl_take_serialized_message(
        subscription: *const rcl_subscription_t,
        serialized_message: *mut rcl_serialized_message_t,
        message_info: *mut rmw_message_info_t,
        allocation: *mut rmw_subscription_allocation_t,
    ) -> rcl_ret_t;

    /// " Take a loaned message from a topic using a rcl subscription.
    pub fn rcl_take_loaned_message(
        subscription: *const rcl_subscription_t,
        loaned_message: *mut *mut c_void,
        message_info: *mut rmw_message_info_t,
        allocation: *mut rmw_subscription_allocation_t,
    ) -> rcl_ret_t;

    /// Return a loaned message from a topic using a rcl subscription.
    pub fn rcl_return_loaned_message_from_subscription(
        subscription: *const rcl_subscription_t,
        loaned_message: *mut c_void,
    ) -> rcl_ret_t;

    /// Get the topic name for the subscription.
    pub fn rcl_subscription_get_topic_name(
        subscription: *const rcl_subscription_t,
    ) -> *const c_char;

    /// Return the rcl subscription options.
    pub fn rcl_subscription_get_options(
        subscription: *const rcl_subscription_t,
    ) -> *const rcl_subscription_options_t;

    /// Check that the subscription is valid."]
    pub fn rcl_subscription_is_valid(subscription: *const rcl_subscription_t) -> bool;

    /// Get the number of publishers matched to a subscription.
    pub fn rcl_subscription_get_publisher_count(
        subscription: *const rcl_subscription_t,
        publisher_count: *mut usize,
    ) -> rmw_ret_t;

    /// Get the actual qos settings of the subscription.
    pub fn rcl_subscription_get_actual_qos(
        subscription: *const rcl_subscription_t,
    ) -> *const rmw_qos_profile_t;

    /// Check if subscription instance can loan messages.
    pub fn rcl_subscription_can_loan_messages(subscription: *const rcl_subscription_t) -> bool;
}
