//! Messages exchanged directly between nodes, bypassing the daemon.
//!
//! Used for low-latency delivery of small messages (<4KB) between
//! nodes on the same machine.

use aligned_vec::{AVec, ConstAlign};

use crate::{id::DataId, metadata::Metadata};

/// Message sent directly from a sender node to a receiver node.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DirectMessage {
    pub input_id: DataId,
    pub metadata: Metadata,
    pub data: AVec<u8, ConstAlign<128>>,
}

/// Routing information for a direct node-to-node connection.
///
/// Returned by the daemon in response to a `QueryDirectRoutes` request,
/// telling the sender which receiver nodes to connect to directly.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DirectRouteInfo {
    pub output_id: DataId,
    pub input_id: DataId,
    pub receiver_addr: std::net::SocketAddr,
}
