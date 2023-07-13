use rustdds::{
  discovery::DiscoveredTopicData,
  ros2::builtin_datatypes::{NodeInfo, ROSParticipantInfo},
};

pub fn get_topics_list_view_strings(discovered_topic_datas: &[DiscoveredTopicData]) -> Vec<String> {
  let mut strings = vec![];
  for topic in discovered_topic_datas {
    strings.push(format!("{:?}", topic.topic_name()));
  }
  strings
}

pub fn get_topic_view_strings(topic: &DiscoveredTopicData) -> Vec<String> {
  let mut strings = vec![];
  strings.push(format!("name: {:?}", topic.topic_name()));
  strings.push(format!("type_name: {:?}", topic.type_name()));
  strings.push(format!("durability: {:?}", topic.topic_data.durability));
  strings.push(format!("deadline: {:?}", topic.topic_data.deadline));
  strings.push(format!(
    "latency_budget: {:?}",
    topic.topic_data.latency_budget
  ));
  strings.push(format!("liveliness: {:?}", topic.topic_data.liveliness));
  strings.push(format!("reliability: {:?}", topic.topic_data.reliability));
  strings.push(format!("lifespan: {:?}", topic.topic_data.lifespan));
  strings.push(format!(
    "destination_order: {:?}",
    topic.topic_data.destination_order
  ));
  strings.push(format!("presentation: {:?}", topic.topic_data.presentation));
  strings.push(format!("history: {:?}", topic.topic_data.history));
  strings.push(format!(
    "resource_limits: {:?}",
    topic.topic_data.resource_limits
  ));
  strings.push(format!("ownership: {:?}", topic.topic_data.ownership));

  strings
}

pub fn get_participant_list_view_strings(participants: &[ROSParticipantInfo]) -> Vec<String> {
  let mut strings = vec![];
  for participant in participants {
    strings.push(format!("{:?}", participant.guid()));
  }
  strings
}

pub fn get_participant_view_strings(participant_info: &ROSParticipantInfo) -> Vec<String> {
  let mut strings = vec![];
  strings.push(format!("guid: {:?}", participant_info.guid()));
  strings.push("nodes: ".to_string());
  for node in participant_info.nodes() {
    strings.push(format!("   name: {:?}", node.get_full_name()));
  }
  strings
}

pub fn get_node_list_strings(nodes: &[NodeInfo]) -> Vec<String> {
  let mut strings = vec![];
  for node in nodes {
    strings.push(format!("{:?}", node.get_full_name()));
  }
  strings
}

pub fn get_node_view_strings(node_info: &NodeInfo) -> Vec<String> {
  let mut strings = vec![];

  strings.push(format!("name: {:?}", node_info.name()));
  strings.push(format!("namespace: {:?}", node_info.namespace()));
  strings.push("readers: ".to_string());
  for reader_gid in node_info.get_reader_gid() {
    strings.push(format!("  {reader_gid:?}"));
  }
  strings.push("writers: ".to_string());
  for writer_gid in node_info.get_reader_gid() {
    strings.push(format!("  {writer_gid:?}"));
  }
  strings
}

// pub fn get_external_node_info_strings(participant: &RosParticipant) ->
// Vec<String> {   let node_infos =
// participant.get_all_discovered_external_ros_node_infos();   let mut strings =
// vec![];   for (_gid, info_vec) in node_infos {
//     for node_info in info_vec {
//       strings.push(format!(
//         "name: {:?} namespace {:?} ",
//         node_info.name(),
//         node_info.namespace()
//       ));
//     }
//   }
//   strings
// }
