use serde::{Deserialize, Serialize};
use cdr_encoding_size::CdrEncodingSize;

use crate::{
  dds::key::Key,
  structure::{guid::GUID, time::Timestamp},
};

/// Analog of DDS GUID in ROS2 builtin datastructures
#[derive(
  Debug, Copy, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, CdrEncodingSize,
)]
pub struct Gid {
  data: [u8; 24],
}

impl Gid {
  pub fn from_guid(guid: GUID) -> Self {
    let mut data: [u8; 24] = [0; 24];
    data[..12].clone_from_slice(&guid.prefix.bytes);
    data[12..15].clone_from_slice(&guid.entity_id.entity_key);
    data[15..16].clone_from_slice(&[u8::from(guid.entity_id.entity_kind)]);
    Self { data }
  }
}

impl Key for Gid {}

/// Information about the node in ROS2 network
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NodeInfo {
  node_namespace: String,
  node_name: String,
  reader_guid: Vec<Gid>,
  writer_guid: Vec<Gid>,
}

impl NodeInfo {
  pub fn new(name: String, namespace: String) -> Self {
    Self {
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

/// Information structure for other DomainParticipants in ROS2 network
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ROSParticipantInfo {
  guid: Gid,
  nodes: Vec<NodeInfo>,
}

impl ROSParticipantInfo {
  pub fn new(guid: Gid, nodes: Vec<NodeInfo>) -> Self {
    Self { guid, nodes }
  }

  pub fn guid(&self) -> Gid {
    self.guid
  }

  pub fn into_nodes(self) -> Vec<NodeInfo> {
    self.nodes
  }

  pub fn nodes(&self) -> &Vec<NodeInfo> {
    &self.nodes
  }

  pub fn nodes_mut(&mut self) -> &mut Vec<NodeInfo> {
    &mut self.nodes
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterEvents {
  timestamp: Timestamp,
  // fully qualified path
  node: String,
  new_parameters: Vec<Parameter>,
  changed_parameters: Vec<Parameter>,
  deleted_parameters: Vec<Parameter>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Parameter {
  name: String,
  value: ParameterValue,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterValue {
  ptype: u8,
  boolean_value: bool,
  int_value: i64,
  double_value: f64,
  string_value: String,
  byte_array: Vec<u8>,
  bool_array: Vec<bool>,
  int_array: Vec<i64>,
  double_array: Vec<f64>,
  string_array: Vec<String>,
}

/// Rosout message structure, received from RosParticipant rosout reader
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Log {
  timestamp: Timestamp,
  level: u8,
  name: String,
  msg: String,
  file: String,
  function: String,
  line: u32,
}

impl Log {
  /// Timestamp when rosout message was sent
  pub fn get_timestamp(&self) -> &Timestamp {
    &self.timestamp
  }

  /// Rosout level
  pub fn get_level(&self) -> u8 {
    self.level
  }

  /// Name of the rosout message
  pub fn name(&self) -> &str {
    &self.name
  }

  /// Actual message
  pub fn get_msg(&self) -> &str {
    &self.msg
  }

  pub fn get_file(&self) -> &str {
    &self.file
  }

  pub fn get_function(&self) -> &str {
    &self.function
  }

  pub fn get_line(&self) -> u32 {
    self.line
  }
}
