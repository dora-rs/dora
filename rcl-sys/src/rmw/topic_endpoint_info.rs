//! API in rmw/topic_endpoint_info.h

use std::os::raw::c_char;

use crate::rmw::{rmw_endpoint_type_t, rmw_qos_profile_t, RMW_GID_STORAGE_SIZE};

/// A data structure that encapsulates the node name, node namespace,
/// topic_type, gid, and qos_profile of publishers and subscriptions
/// for a topic.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_topic_endpoint_info_t {
    /// Name of the node
    pub node_name: *const c_char,
    /// Namespace of the node
    pub node_namespace: *const c_char,
    /// The associated topic type
    pub topic_type: *const c_char,
    /// The endpoint type
    pub endpoint_type: rmw_endpoint_type_t,
    /// The GID of the endpoint
    pub endpoint_gid: [u8; RMW_GID_STORAGE_SIZE],
    /// QoS profile of the endpoint
    pub qos_profile: rmw_qos_profile_t,
}
