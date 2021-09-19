//! API for rcl/node.h
//!
//! skip
//! - rcl_node_get_rmw_handle

use std::os::raw::c_char;

use crate::*;

#[repr(C)]
#[derive(Debug)]
pub struct rcl_node_impl_t {
    _unused: [u8; 0],
}
/// Structure which encapsulates a ROS Node.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_node_t {
    /// Context associated with this node.
    context: *mut rcl_context_t,
    /// Private implementation pointer.
    impl_: *mut rcl_node_impl_t,
}

extern "C" {
    /// Return a [rcl_node_t] struct with members initialized to `NULL`.
    pub fn rcl_get_zero_initialized_node() -> rcl_node_t;

    /// Initialize a ROS node.
    pub fn rcl_node_init(
        node: *mut rcl_node_t,
        name: *const c_char,
        namespace_: *const c_char,
        context: *mut rcl_context_t,
        options: *const rcl_node_options_t,
    ) -> rcl_ret_t;

    /// Finalize a [rcl_node_t].
    pub fn rcl_node_fini(node: *mut rcl_node_t) -> rcl_ret_t;

    /// Return `true` if the node is valid, else `false`.
    pub fn rcl_node_is_valid(node: *const rcl_node_t) -> bool;

    /// Return true if node is valid, except for the context being valid.
    pub fn rcl_node_is_valid_except_context(node: *const rcl_node_t) -> bool;

    /// Return the name of the node.
    pub fn rcl_node_get_name(node: *const rcl_node_t) -> *const c_char;

    /// Return the namespace of the node.
    pub fn rcl_node_get_namespace(node: *const rcl_node_t) -> *const c_char;

    /// Return the fully qualified name of the node.
    pub fn rcl_node_get_fully_qualified_name(node: *const rcl_node_t) -> *const c_char;

    /// Return the rcl node options.
    pub fn rcl_node_get_options(node: *const rcl_node_t) -> *const rcl_node_options_t;

    /// Return the ROS domain ID that the node is using.
    pub fn rcl_node_get_domain_id(node: *const rcl_node_t, domain_id: *mut usize) -> rcl_ret_t;

    /// Return the associated rcl instance id.
    pub fn rcl_node_get_rcl_instance_id(node: *const rcl_node_t) -> u64;

    /// Return a guard condition which is triggered when the ROS graph changes.
    pub fn rcl_node_get_graph_guard_condition(
        node: *const rcl_node_t,
    ) -> *const rcl_guard_condition_t;

    /// Return the logger name of the node.
    pub fn rcl_node_get_logger_name(node: *const rcl_node_t) -> *const c_char;
}
