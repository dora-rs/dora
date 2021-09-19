//! API in rcl/service.h
//!
//! skip
//! - rcl_service_get_rmw_handle

use std::os::raw::{c_char, c_void};

use crate::*;

/// Internal rcl implementation struct.
#[repr(C)]
#[derive(Debug)]
struct rcl_service_impl_t {
    _unused: [u8; 0],
}

/// Structure which encapsulates a ROS Service.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_service_t {
    /// Pointer to the service implementation
    impl_: *mut rcl_service_impl_t,
}

/// Options available for a [rcl_service_t].
#[repr(C)]
#[derive(Debug)]
pub struct rcl_service_options_t {
    /// Middleware quality of service settings for the service.
    pub qos: rmw_qos_profile_t,
    /// Custom allocator for the service, used for incidental allocations.
    pub allocator: rcl_allocator_t,
}

extern "C" {
    /// Return a [rcl_service_t] struct with members set to `NULL`.
    pub fn rcl_get_zero_initialized_service() -> rcl_service_t;

    /// Initialize a rcl service.
    pub fn rcl_service_init(
        service: *mut rcl_service_t,
        node: *const rcl_node_t,
        type_support: *const rosidl_service_type_support_t,
        service_name: *const c_char,
        options: *const rcl_service_options_t,
    ) -> rcl_ret_t;

    /// Finalize a [rcl_service_t].
    pub fn rcl_service_fini(service: *mut rcl_service_t, node: *mut rcl_node_t) -> rcl_ret_t;

    /// Return the default service options in a [rcl_service_options_t].
    pub fn rcl_service_get_default_options() -> rcl_service_options_t;

    /// Take a pending ROS request using a rcl service.
    pub fn rcl_take_request_with_info(
        service: *const rcl_service_t,
        request_header: *mut rmw_service_info_t,
        ros_request: *mut c_void,
    ) -> rcl_ret_t;

    /// backwards compatibility version that takes a request_id only
    pub fn rcl_take_request(
        service: *const rcl_service_t,
        request_header: *mut rmw_request_id_t,
        ros_request: *mut c_void,
    ) -> rcl_ret_t;

    /// Send a ROS response to a client using a service.
    pub fn rcl_send_response(
        service: *const rcl_service_t,
        response_header: *mut rmw_request_id_t,
        ros_response: *mut c_void,
    ) -> rcl_ret_t;

    /// Get the topic name for the service.
    pub fn rcl_service_get_service_name(service: *const rcl_service_t) -> *const c_char;

    /// Return the rcl service options.
    pub fn rcl_service_get_options(service: *const rcl_service_t) -> *const rcl_service_options_t;

    /// Check that the service is valid.
    pub fn rcl_service_is_valid(service: *const rcl_service_t) -> bool;
}
