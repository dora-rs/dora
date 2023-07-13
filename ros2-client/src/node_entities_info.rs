use serde::{Deserialize, Serialize};

use crate::gid::Gid;

/// Information about a node in ROS2 network
///
/// See [NodeEntitiesInfo](https://github.com/ros2/rmw_dds_common/blob/master/rmw_dds_common/msg/NodeEntitiesInfo.msg)
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NodeEntitiesInfo {
  node_namespace: String,
  node_name: String,
  reader_guid: Vec<Gid>,
  writer_guid: Vec<Gid>,
}

impl NodeEntitiesInfo {
  pub fn new(name: String, namespace: String) -> NodeEntitiesInfo {
    NodeEntitiesInfo {
      node_namespace: namespace,
      node_name: name,
      reader_guid: Vec::new(),
      writer_guid: Vec::new(),
    }
  }

  pub fn namespace(&self) -> &str {
    &self.node_namespace
  }

  pub fn name(&self) -> &str {
    &self.node_name
  }

  pub fn get_reader_gid(&self) -> Vec<Gid> {
    self.reader_guid.clone()
  }

  pub fn get_writer_gid(&self) -> Vec<Gid> {
    self.writer_guid.clone()
  }

  /// Full name of the node namespace + name eg. /some_node
  pub fn get_full_name(&self) -> String {
    let mut name = self.node_namespace.clone();
    name.push_str(&self.node_name);
    name
  }

  pub fn add_writer(&mut self, gid: Gid) {
    if !self.writer_guid.contains(&gid) {
      self.writer_guid.push(gid);
    }
  }

  pub fn add_reader(&mut self, gid: Gid) {
    if !self.reader_guid.contains(&gid) {
      self.reader_guid.push(gid);
    }
  }

  /// Clears all reader and writer guids
  pub fn clear_all(&mut self) {
    self.reader_guid.clear();
    self.writer_guid.clear();
  }
}
