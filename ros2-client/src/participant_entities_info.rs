use serde::{Deserialize, Serialize};

use crate::{gid::Gid, node_entities_info::NodeEntitiesInfo};

/// Information structure for other DomainParticipants in ROS2 network
///
/// See [ParticipantEntitiesInfo](https://github.com/ros2/rmw_dds_common/blob/master/rmw_dds_common/msg/ParticipantEntitiesInfo.msg) in ROS2.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParticipantEntitiesInfo {
  guid: Gid,
  nodes: Vec<NodeEntitiesInfo>,
}

impl ParticipantEntitiesInfo {
  pub fn new(guid: Gid, nodes: Vec<NodeEntitiesInfo>) -> ParticipantEntitiesInfo {
    ParticipantEntitiesInfo { guid, nodes }
  }

  pub fn guid(&self) -> Gid {
    self.guid
  }

  pub fn into_nodes(self) -> Vec<NodeEntitiesInfo> {
    self.nodes
  }

  pub fn nodes(&self) -> &Vec<NodeEntitiesInfo> {
    &self.nodes
  }

  pub fn nodes_mut(&mut self) -> &mut Vec<NodeEntitiesInfo> {
    &mut self.nodes
  }
}
