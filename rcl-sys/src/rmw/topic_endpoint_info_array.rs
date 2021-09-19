//! API in rmw/topic_endpoint_info_array.h

use super::{rmw_ret_t, rmw_topic_endpoint_info_t};
use crate::rcutils::rcutils_allocator_t;

/// Array of topic endpoint information
#[repr(C)]
#[derive(Debug)]
pub struct rmw_topic_endpoint_info_array_t {
    /// Size of the array.
    pub size: usize,
    /// Contiguous storage for topic endpoint information elements.
    pub info_array: *mut rmw_topic_endpoint_info_t,
}

extern "C" {
    /// Return a zero initialized array of topic endpoint information.
    pub fn rmw_get_zero_initialized_topic_endpoint_info_array() -> rmw_topic_endpoint_info_array_t;

    /// Initialize an array of topic endpoint information.
    pub fn rmw_topic_endpoint_info_array_init_with_size(
        topic_endpoint_info_array: *mut rmw_topic_endpoint_info_array_t,
        size: usize,
        allocator: *mut rcutils_allocator_t,
    ) -> rmw_ret_t;

    /// Finalize an array of topic endpoint information.
    pub fn rmw_topic_endpoint_info_array_fini(
        topic_endpoint_info_array: *mut rmw_topic_endpoint_info_array_t,
        allocator: *mut rcutils_allocator_t,
    ) -> rmw_ret_t;
}
