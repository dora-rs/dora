//! API in rcl/graph.h

use std::os::raw::c_char;

use crate::*;
pub use crate::{
    rmw_get_zero_initialized_names_and_types as rcl_get_zero_initialized_names_and_types,
    rmw_get_zero_initialized_topic_endpoint_info_array as rcl_get_zero_initialized_topic_endpoint_info_array,
    rmw_names_and_types_t as rcl_names_and_types_t,
    rmw_topic_endpoint_info_array_fini as rcl_topic_endpoint_info_array_fini,
    rmw_topic_endpoint_info_array_t as rcl_topic_endpoint_info_array_t,
    rmw_topic_endpoint_info_t as rcl_topic_endpoint_info_t,
};

extern "C" {
    /// Return a list of topic names and types for publishers associated with a node.
    pub fn rcl_get_publisher_names_and_types_by_node(
        node: *const rcl_node_t,
        allocator: *mut rcl_allocator_t,
        no_demangle: bool,
        node_name: *const c_char,
        node_namespace: *const c_char,
        topic_names_and_types: *mut rcl_names_and_types_t,
    ) -> rcl_ret_t;

    /// Return a list of topic names and types for subscriptions associated with a node.
    pub fn rcl_get_subscriber_names_and_types_by_node(
        node: *const rcl_node_t,
        allocator: *mut rcl_allocator_t,
        no_demangle: bool,
        node_name: *const c_char,
        node_namespace: *const c_char,
        topic_names_and_types: *mut rcl_names_and_types_t,
    ) -> rcl_ret_t;

    /// Return a list of service names and types associated with a node.
    pub fn rcl_get_service_names_and_types_by_node(
        node: *const rcl_node_t,
        allocator: *mut rcl_allocator_t,
        node_name: *const c_char,
        node_namespace: *const c_char,
        service_names_and_types: *mut rcl_names_and_types_t,
    ) -> rcl_ret_t;

    /// Return a list of service client names and types associated with a node.
    pub fn rcl_get_client_names_and_types_by_node(
        node: *const rcl_node_t,
        allocator: *mut rcl_allocator_t,
        node_name: *const c_char,
        node_namespace: *const c_char,
        service_names_and_types: *mut rcl_names_and_types_t,
    ) -> rcl_ret_t;

    /// Return a list of topic names and their types.
    pub fn rcl_get_topic_names_and_types(
        node: *const rcl_node_t,
        allocator: *mut rcl_allocator_t,
        no_demangle: bool,
        topic_names_and_types: *mut rcl_names_and_types_t,
    ) -> rcl_ret_t;

    /// Return a list of service names and their types.
    pub fn rcl_get_service_names_and_types(
        node: *const rcl_node_t,
        allocator: *mut rcl_allocator_t,
        service_names_and_types: *mut rcl_names_and_types_t,
    ) -> rcl_ret_t;

    /// Initialize a rcl_names_and_types_t object.
    pub fn rcl_names_and_types_init(
        names_and_types: *mut rcl_names_and_types_t,
        size: usize,
        allocator: *mut rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize a rcl_names_and_types_t object.
    pub fn rcl_names_and_types_fini(names_and_types: *mut rcl_names_and_types_t) -> rcl_ret_t;

    /// Return a list of available nodes in the ROS graph.
    pub fn rcl_get_node_names(
        node: *const rcl_node_t,
        allocator: rcl_allocator_t,
        node_names: *mut rcutils_string_array_t,
        node_namespaces: *mut rcutils_string_array_t,
    ) -> rcl_ret_t;

    /// Return a list of available nodes in the ROS graph, including their enclave names.
    pub fn rcl_get_node_names_with_enclaves(
        node: *const rcl_node_t,
        allocator: rcl_allocator_t,
        node_names: *mut rcutils_string_array_t,
        node_namespaces: *mut rcutils_string_array_t,
        enclaves: *mut rcutils_string_array_t,
    ) -> rcl_ret_t;

    /// Return the number of publishers on a given topic.
    pub fn rcl_count_publishers(
        node: *const rcl_node_t,
        topic_name: *const c_char,
        count: *mut usize,
    ) -> rcl_ret_t;

    /// Return the number of subscriptions on a given topic.
    pub fn rcl_count_subscribers(
        node: *const rcl_node_t,
        topic_name: *const c_char,
        count: *mut usize,
    ) -> rcl_ret_t;

    /// Return a list of all publishers to a topic.
    pub fn rcl_get_publishers_info_by_topic(
        node: *const rcl_node_t,
        allocator: *mut rcutils_allocator_t,
        topic_name: *const c_char,
        no_mangle: bool,
        publishers_info: *mut rcl_topic_endpoint_info_array_t,
    ) -> rcl_ret_t;

    /// Return a list of all subscriptions to a topic.
    pub fn rcl_get_subscriptions_info_by_topic(
        node: *const rcl_node_t,
        allocator: *mut rcutils_allocator_t,
        topic_name: *const c_char,
        no_mangle: bool,
        subscriptions_info: *mut rcl_topic_endpoint_info_array_t,
    ) -> rcl_ret_t;

    /// Check if a service server is available for the given service client.
    pub fn rcl_service_server_is_available(
        node: *const rcl_node_t,
        client: *const rcl_client_t,
        is_available: *mut bool,
    ) -> rcl_ret_t;
}
