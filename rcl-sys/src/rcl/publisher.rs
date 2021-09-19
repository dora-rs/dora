//! API in rcl/publisher.h

use std::os::raw::{c_char, c_void};

use crate::*;

/// Internal rcl publisher implementation struct.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_publisher_impl_t {
    _unused: [u8; 0],
}

/// Structure which encapsulates a ROS Publisher.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_publisher_t {
    /// Pointer to the publisher implementation
    impl_: *mut rcl_publisher_impl_t,
}

/// Options available for a rcl publisher.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_publisher_options_t {
    /// Middleware quality of service settings for the publisher.
    pub qos: rmw_qos_profile_t,
    /// Custom allocator for the publisher, used for incidental allocations.
    pub allocator: rcl_allocator_t,
    // rmw specific publisher options, e.g. the rmw implementation specific payload.
    pub rmw_publisher_options: rmw_publisher_options_t,
}

extern "C" {
    /// Return a [rcl_publisher_t] struct with members set to `NULL`.
    pub fn rcl_get_zero_initialized_publisher() -> rcl_publisher_t;

    /// Initialize a rcl publisher.
    pub fn rcl_publisher_init(
        publisher: *mut rcl_publisher_t,
        node: *const rcl_node_t,
        type_support: *const rosidl_message_type_support_t,
        topic_name: *const c_char,
        options: *const rcl_publisher_options_t,
    ) -> rcl_ret_t;

    /// Finalize a rcl_publisher_t.
    pub fn rcl_publisher_fini(publisher: *mut rcl_publisher_t, node: *mut rcl_node_t) -> rcl_ret_t;

    /// Return the default publisher options in a [rcl_publisher_options_t].
    pub fn rcl_publisher_get_default_options() -> rcl_publisher_options_t;

    /// Borrow a loaned message.
    pub fn rcl_borrow_loaned_message(
        publisher: *const rcl_publisher_t,
        type_support: *const rosidl_message_type_support_t,
        ros_message: *mut *mut c_void,
    ) -> rcl_ret_t;

    /// Return a loaned message previously borrowed from a publisher.
    pub fn rcl_return_loaned_message_from_publisher(
        publisher: *const rcl_publisher_t,
        loaned_message: *mut c_void,
    ) -> rcl_ret_t;

    /// Publish a ROS message on a topic using a publisher.
    pub fn rcl_publish(
        publisher: *const rcl_publisher_t,
        ros_message: *const c_void,
        allocation: *mut rmw_publisher_allocation_t,
    ) -> rcl_ret_t;

    /// Publish a serialized message on a topic using a publisher.
    pub fn rcl_publish_serialized_message(
        publisher: *const rcl_publisher_t,
        serialized_message: *const rcl_serialized_message_t,
        allocation: *mut rmw_publisher_allocation_t,
    ) -> rcl_ret_t;

    /// Publish a loaned message on a topic using a publisher.
    pub fn rcl_publish_loaned_message(
        publisher: *const rcl_publisher_t,
        ros_message: *mut c_void,
        allocation: *mut rmw_publisher_allocation_t,
    ) -> rcl_ret_t;

    /// Manually assert that this Publisher is alive (for `RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC`)
    pub fn rcl_publisher_assert_liveliness(publisher: *const rcl_publisher_t) -> rcl_ret_t;

    /// Get the topic name for the publisher.
    pub fn rcl_publisher_get_topic_name(publisher: *const rcl_publisher_t) -> *const c_char;

    /// Return the rcl publisher options.
    pub fn rcl_publisher_get_options(
        publisher: *const rcl_publisher_t,
    ) -> *const rcl_publisher_options_t;

    /// Return the context associated with this publisher.
    pub fn rcl_publisher_get_context(publisher: *const rcl_publisher_t) -> *mut rcl_context_t;

    /// Return the context associated with this publisher.
    pub fn rcl_publisher_is_valid(publisher: *const rcl_publisher_t) -> bool;

    /// Return true if the publisher is valid, otherwise false.
    pub fn rcl_publisher_is_valid_except_context(publisher: *const rcl_publisher_t) -> bool;

    /// Get the number of subscriptions matched to a publisher.
    pub fn rcl_publisher_get_subscription_count(
        publisher: *const rcl_publisher_t,
        subscription_count: *mut usize,
    ) -> rcl_ret_t;

    /// Get the actual qos settings of the publisher.
    pub fn rcl_publisher_get_actual_qos(
        publisher: *const rcl_publisher_t,
    ) -> *const rmw_qos_profile_t;

    /// Check if publisher instance can loan messages.
    pub fn rcl_publisher_can_loan_messages(publisher: *const rcl_publisher_t) -> bool;
}
