use rustdds::{
  discovery::DiscoveredTopicData,
  ros2::builtin_datatypes::{NodeInfo, ROSParticipantInfo},
};

#[derive(Debug)]
pub enum DataUpdate {
  NewROSParticipantFound { participant: ROSParticipantInfo },
  DiscoveredTopics { topics: Vec<DiscoveredTopicData> },
  DiscoveredNodes { nodes: Vec<NodeInfo> },
}

#[derive(Debug)]
pub enum RosCommand {
  StopRosLoop,
}
