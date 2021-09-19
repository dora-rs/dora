//! API in rcl/node_options.h

use crate::*;

/// Structure which encapsulates the options for creating a rcl_node_t.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_node_options_t {
    /// If set, then this value overrides the ROS_DOMAIN_ID environment variable.
    pub domain_id: usize,
    /// Custom allocator used for internal allocations.
    pub allocator: rcl_allocator_t,
    /// If false then only use arguments in this struct, otherwise use global arguments also.
    pub use_global_arguments: bool,
    /// Command line arguments that apply only to this node.
    pub arguments: rcl_arguments_t,
    /// Flag to enable rosout for this node
    pub enable_rosout: bool,
}

extern "C" {
    /// Return the default node options in a [rcl_node_options_t].
    pub fn rcl_node_get_default_options() -> rcl_node_options_t;

    /// Copy one options structure into another.
    pub fn rcl_node_options_copy(
        options: *const rcl_node_options_t,
        options_out: *mut rcl_node_options_t,
    ) -> rcl_ret_t;

    /// Finalize the given node_options.
    pub fn rcl_node_options_fini(options: *mut rcl_node_options_t) -> rcl_ret_t;
}
