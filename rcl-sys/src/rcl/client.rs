//! API in rcl/client.h

use std::os::raw::{c_char, c_void};

use crate::*;

/// Internal rcl client implementation struct.
#[repr(C)]
#[derive(Debug)]
struct rcl_client_impl_t {
    _unused: [u8; 0],
}

/// Structure which encapsulates a ROS Client.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_client_t {
    /// Pointer to the client implementation
    impl_: *mut rcl_client_impl_t,
}

/// Options available for a [rcl_client_t].
#[repr(C)]
#[derive(Debug)]
pub struct rcl_client_options_t {
    /// Middleware quality of service settings for the client.
    pub qos: rmw_qos_profile_t,
    /// Custom allocator for the client, used for incidental allocations.
    pub allocator: rcl_allocator_t,
}

extern "C" {
    /// Return a [rcl_client_t] struct with members set to `NULL`.
    pub fn rcl_get_zero_initialized_client() -> rcl_client_t;

    /// Initialize a rcl client.
    pub fn rcl_client_init(
        client: *mut rcl_client_t,
        node: *const rcl_node_t,
        type_support: *const rosidl_service_type_support_t,
        service_name: *const c_char,
        options: *const rcl_client_options_t,
    ) -> rcl_ret_t;

    /// Finalize a 'rcl_client_t'.
    pub fn rcl_client_fini(client: *mut rcl_client_t, node: *mut rcl_node_t) -> rcl_ret_t;

    /// Return the default client options in a [rcl_client_options_t].
    pub fn rcl_client_get_default_options() -> rcl_client_options_t;

    /// Send a ROS request using a client.
    pub fn rcl_send_request(
        client: *const rcl_client_t,
        ros_request: *const c_void,
        sequence_number: *mut i64,
    ) -> rcl_ret_t;

    /// Take a ROS response using a client
    pub fn rcl_take_response_with_info(
        client: *const rcl_client_t,
        request_header: *mut rmw_service_info_t,
        ros_response: *mut c_void,
    ) -> rcl_ret_t;

    /// backwards compatibility function that takes a rmw_request_id_t only
    pub fn rcl_take_response(
        client: *const rcl_client_t,
        request_header: *mut rmw_request_id_t,
        ros_response: *mut c_void,
    ) -> rcl_ret_t;

    /// Get the name of the service that this client will request a response from.
    pub fn rcl_client_get_service_name(client: *const rcl_client_t) -> *const c_char;

    /// Return the rcl client options.
    pub fn rcl_client_get_options(client: *const rcl_client_t) -> *const rcl_client_options_t;

    /// Check that the client is valid.
    pub fn rcl_client_is_valid(client: *const rcl_client_t) -> bool;
}
