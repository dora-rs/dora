use std::{
  collections::{HashMap, HashSet},
  sync::{Arc, Mutex},
};

use log::{error, info};
use mio_06::Evented;
use serde::{de::DeserializeOwned, Serialize};

use crate::{
  dds::{
    adapters::{no_key, with_key},
    key::{Key, Keyed},
    pubsub::{Publisher, Subscriber},
    qos::QosPolicies,
    result::Error,
    topic::{Topic, TopicKind},
  },
  discovery::sedp_messages::DiscoveredTopicData,
  no_key::{datareader::DataReader as NoKeyDataReader, datawriter::DataWriter as NoKeyDataWriter},
  structure::{entity::RTPSEntity, guid::GUID},
  DomainParticipant,
};
use super::{
  builtin_datatypes::{Gid, Log, NodeInfo, ParameterEvents, ROSParticipantInfo},
  builtin_topics::{ParameterEventsTopic, ROSDiscoveryTopic, RosOutTopic},
  KeyedRosPublisher, KeyedRosSubscriber, RosPublisher, RosSubscriber,
};

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

/// [`RosParticipant`] sends and receives other
/// participants information in ROS2 network
#[derive(Clone)]
pub struct RosParticipant {
  inner: Arc<Mutex<RosParticipantInner>>,
}

impl RosParticipant {
  pub fn new() -> Result<Self, Error> {
    Self::from_domain_participant(DomainParticipant::new(0)?)
  }

  pub fn from_domain_participant(domain_participant: DomainParticipant) -> Result<Self, Error> {
    let i = RosParticipantInner::from_domain_participant(domain_participant)?;
    Ok(Self {
      inner: Arc::new(Mutex::new(i)),
    })
  }
  /// Create a new ROS2 node
  pub fn new_ros_node(
    &self,
    name: &str,
    namespace: &str,
    options: NodeOptions,
  ) -> Result<RosNode, Error> {
    RosNode::new(name, namespace, options, self.clone())
  }
  pub fn handle_node_read(&mut self) -> Vec<ROSParticipantInfo> {
    self.inner.lock().unwrap().handle_node_read()
  }
  /// Clears all nodes and updates our RosParticipantInfo to ROS2 network
  pub fn clear(&mut self) {
    self.inner.lock().unwrap().clear();
  }

  pub fn domain_id(&self) -> u16 {
    self.inner.lock().unwrap().domain_participant.domain_id()
  }

  pub fn discovered_topics(&self) -> Vec<DiscoveredTopicData> {
    self.domain_participant().discovered_topics()
  }

  pub fn add_node_info(&mut self, node_info: NodeInfo) {
    self.inner.lock().unwrap().add_node_info(node_info);
  }

  pub fn remove_node_info(&mut self, node_info: &NodeInfo) {
    self.inner.lock().unwrap().remove_node_info(node_info);
  }

  pub fn get_all_discovered_external_ros_node_infos(&self) -> HashMap<Gid, Vec<NodeInfo>> {
    self.inner.lock().unwrap().external_nodes.clone()
  }

  pub fn get_all_discovered_local_ros_node_infos(&self) -> HashMap<String, NodeInfo> {
    self.inner.lock().unwrap().nodes.clone()
  }

  /// Gets our current participant info we have sent to ROS2 network
  pub fn get_ros_participant_info(&self) -> ROSParticipantInfo {
    self.inner.lock().unwrap().get_ros_participant_info()
  }

  fn get_parameter_events_topic(&self) -> Topic {
    self
      .inner
      .lock()
      .unwrap()
      .ros_parameter_events_topic
      .clone()
  }

  fn get_rosout_topic(&self) -> Topic {
    self.inner.lock().unwrap().ros_rosout_topic.clone()
  }

  fn get_ros_discovery_publisher(&self) -> Publisher {
    self.inner.lock().unwrap().ros_discovery_publisher.clone()
  }

  fn get_ros_discovery_subscriber(&self) -> Subscriber {
    self.inner.lock().unwrap().ros_discovery_subscriber.clone()
  }

  fn domain_participant(&self) -> DomainParticipant {
    self.inner.lock().unwrap().domain_participant.clone()
  }
}

struct RosParticipantInner {
  nodes: HashMap<String, NodeInfo>,
  external_nodes: HashMap<Gid, Vec<NodeInfo>>,
  node_reader: NoKeyDataReader<ROSParticipantInfo>,
  node_writer: NoKeyDataWriter<ROSParticipantInfo>,

  domain_participant: DomainParticipant,
  #[allow(dead_code)] // technically not needed after initialization
  ros_discovery_topic: Topic,
  ros_discovery_publisher: Publisher,
  ros_discovery_subscriber: Subscriber,

  ros_parameter_events_topic: Topic,
  ros_rosout_topic: Topic,
}

impl RosParticipantInner {
  // "new"
  pub fn from_domain_participant(domain_participant: DomainParticipant) -> Result<Self, Error> {
    let ros_discovery_topic = domain_participant.create_topic(
      ROSDiscoveryTopic::topic_name().to_string(),
      ROSDiscoveryTopic::type_name().to_string(),
      &ROSDiscoveryTopic::qos(),
      TopicKind::NoKey,
    )?;

    let ros_discovery_publisher = domain_participant.create_publisher(&ROSDiscoveryTopic::qos())?;
    let ros_discovery_subscriber =
      domain_participant.create_subscriber(&ROSDiscoveryTopic::qos())?;

    let ros_parameter_events_topic = domain_participant.create_topic(
      ParameterEventsTopic::topic_name().to_string(),
      ParameterEventsTopic::type_name().to_string(),
      &ParameterEventsTopic::qos(),
      TopicKind::NoKey,
    )?;

    let ros_rosout_topic = domain_participant.create_topic(
      RosOutTopic::topic_name().to_string(),
      RosOutTopic::type_name().to_string(),
      &RosOutTopic::qos(),
      TopicKind::NoKey,
    )?;

    let node_reader =
      ros_discovery_subscriber.create_datareader_no_key(&ros_discovery_topic, None)?;

    let node_writer =
      ros_discovery_publisher.create_datawriter_no_key(&ros_discovery_topic, None)?;

    Ok(Self {
      nodes: HashMap::new(),
      external_nodes: HashMap::new(),
      node_reader,
      node_writer,

      domain_participant,
      ros_discovery_topic,
      ros_discovery_publisher,
      ros_discovery_subscriber,
      ros_parameter_events_topic,
      ros_rosout_topic,
    })
  }

  /// Gets our current participant info we have sent to ROS2 network
  pub fn get_ros_participant_info(&self) -> ROSParticipantInfo {
    ROSParticipantInfo::new(
      Gid::from_guid(self.domain_participant.guid()),
      self.nodes.values().cloned().collect(),
    )
  }

  // Adds new NodeInfo and updates our RosParticipantInfo to ROS2 network
  fn add_node_info(&mut self, mut node_info: NodeInfo) {
    node_info.add_reader(Gid::from_guid(self.node_reader.guid()));
    node_info.add_writer(Gid::from_guid(self.node_writer.guid()));

    self.nodes.insert(node_info.get_full_name(), node_info);
    self.broadcast_node_infos();
  }

  /// Removes NodeInfo and updates our RosParticipantInfo to ROS2 network
  fn remove_node_info(&mut self, node_info: &NodeInfo) {
    self.nodes.remove(&node_info.get_full_name());
    self.broadcast_node_infos();
  }

  /// Clears all nodes and updates our RosParticipantInfo to ROS2 network
  pub fn clear(&mut self) {
    if !self.nodes.is_empty() {
      self.nodes.clear();
      self.broadcast_node_infos();
    }
  }

  fn broadcast_node_infos(&self) {
    match self
      .node_writer
      .write(self.get_ros_participant_info(), None)
    {
      Ok(_) => (),
      Err(e) => error!("Failed to write into node_writer {e:?}"),
    }
  }

  /// Fetches all unread ROSParticipantInfos we have received
  pub fn handle_node_read(&mut self) -> Vec<ROSParticipantInfo> {
    let mut pts = Vec::new();
    while let Ok(Some(sample)) = self.node_reader.take_next_sample() {
      let rpi = sample.value();
      match self.external_nodes.get_mut(&rpi.guid()) {
        Some(rpi2) => {
          *rpi2 = rpi.nodes().clone();
        }
        None => {
          self.external_nodes.insert(rpi.guid(), rpi.nodes().clone());
        }
      };
      pts.push(rpi.clone());
    }
    pts
  }

  //rustdds::ros2::ros_node::RosParticipantInner
  //external_nodes: HashMap<Gid, Vec<NodeInfo, Global>, RandomState>

  /*
  pub fn get_all_discovered_ros_node_infos(&self) -> HashMap<Gid, Vec<NodeInfo>> {
    //let mut pts = Vec::new();
    self.external_nodes.clone()
  }
  */
}

use mio_06 as mio;

impl Evented for RosParticipant {
  fn register(
    &self,
    poll: &mio::Poll,
    token: mio::Token,
    interest: mio::Ready,
    opts: mio::PollOpt,
  ) -> std::io::Result<()> {
    poll.register(
      &self.inner.lock().unwrap().node_reader,
      token,
      interest,
      opts,
    )
  }

  fn reregister(
    &self,
    poll: &mio::Poll,
    token: mio::Token,
    interest: mio::Ready,
    opts: mio::PollOpt,
  ) -> std::io::Result<()> {
    poll.reregister(
      &self.inner.lock().unwrap().node_reader,
      token,
      interest,
      opts,
    )
  }

  fn deregister(&self, poll: &mio::Poll) -> std::io::Result<()> {
    poll.deregister(&self.inner.lock().unwrap().node_reader)
  }
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

/// Configuration of [RosNode](struct.RosNode.html)
pub struct NodeOptions {
  enable_rosout: bool,
}

impl NodeOptions {
  /// # Arguments
  ///
  /// * `enable_rosout` -  Wheter or not ros logging is enabled (rosout writer)
  pub fn new(/* domain_id: u16, */ enable_rosout: bool) -> Self {
    Self { enable_rosout }
  }
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

/// Node in ROS2 network. Holds necessary readers and writers for rosout and
/// parameter events topics internally. Should be constructed using
/// [builder](struct.RosNodeBuilder.html).
pub struct RosNode {
  // node info
  name: String,
  namespace: String,
  options: NodeOptions,

  ros_participant: RosParticipant,

  // dynamic
  readers: HashSet<GUID>,
  writers: HashSet<GUID>,

  // builtin writers and readers
  rosout_writer: Option<NoKeyDataWriter<Log>>,
  //rosout_reader: Option<NoKeyDataReader<Log>>, // TODO
  parameter_events_writer: NoKeyDataWriter<ParameterEvents>,
}

impl RosNode {
  fn new(
    name: &str,
    namespace: &str,
    options: NodeOptions,
    ros_participant: RosParticipant,
  ) -> Result<Self, Error> {
    let paramtopic = ros_participant.get_parameter_events_topic();
    let rosout_topic = ros_participant.get_rosout_topic();

    let rosout_writer = if options.enable_rosout {
      Some(
        ros_participant
          .get_ros_discovery_publisher()
          .create_datawriter_no_key(&rosout_topic, None)?,
      )
    } else {
      None
    };

    let parameter_events_writer = ros_participant
      .get_ros_discovery_publisher()
      .create_datawriter_no_key(&paramtopic, None)?;

    Ok(Self {
      name: String::from(name),
      namespace: String::from(namespace),
      options,
      ros_participant,
      readers: HashSet::new(),
      writers: HashSet::new(),
      rosout_writer,
      //rosout_reader: None, // TODO
      parameter_events_writer,
    })
  }

  // Generates ROS2 node info from added readers and writers.
  fn generate_node_info(&self) -> NodeInfo {
    let mut node_info = NodeInfo::new(self.name.clone(), self.namespace.clone());

    node_info.add_writer(Gid::from_guid(self.parameter_events_writer.guid()));
    if let Some(row) = &self.rosout_writer {
      node_info.add_writer(Gid::from_guid(row.guid()));
    }

    for reader in &self.readers {
      node_info.add_reader(Gid::from_guid(*reader));
    }

    for writer in &self.writers {
      node_info.add_writer(Gid::from_guid(*writer));
    }

    node_info
  }

  fn add_reader(&mut self, reader: GUID) {
    self.readers.insert(reader);
    self
      .ros_participant
      .add_node_info(self.generate_node_info());
  }

  pub fn remove_reader(&mut self, reader: &GUID) {
    self.readers.remove(reader);
    self
      .ros_participant
      .add_node_info(self.generate_node_info());
  }

  fn add_writer(&mut self, writer: GUID) {
    self.writers.insert(writer);
    self
      .ros_participant
      .add_node_info(self.generate_node_info());
  }

  pub fn remove_writer(&mut self, writer: &GUID) {
    self.writers.remove(writer);
    self
      .ros_participant
      .add_node_info(self.generate_node_info());
  }

  /// Clears both all reader and writer guids from this node.
  pub fn clear_node(&mut self) {
    self.readers.clear();
    self.writers.clear();
    self
      .ros_participant
      .add_node_info(self.generate_node_info());
  }

  pub fn name(&self) -> &str {
    &self.name
  }

  pub fn namespace(&self) -> &str {
    &self.namespace
  }

  pub fn get_fully_qualified_name(&self) -> String {
    let mut nn = self.name.clone();
    nn.push_str(&self.namespace);
    nn
  }

  pub fn get_options(&self) -> &NodeOptions {
    &self.options
  }

  pub fn get_domain_id(&self) -> u16 {
    self.ros_participant.domain_id()
  }

  /// Creates ROS2 topic and handles necessary conversions from DDS to ROS2
  ///
  /// # Arguments
  ///
  /// * `domain_participant` -
  ///   [DomainParticipant](../dds/struct.DomainParticipant.html)
  /// * `name` - Name of the topic
  /// * `type_name` - What type the topic holds in string form
  /// * `qos` - Quality of Service parameters for the topic (not restricted only
  ///   to ROS2)
  /// * `topic_kind` - Does the topic have a key (multiple DDS instances)? NoKey
  ///   or WithKey
  ///
  ///  
  ///   [summary of all rules for topic and service names in ROS 2](https://design.ros2.org/articles/topic_and_service_names.html)
  ///   (as of Dec 2020)
  ///
  /// * must not be empty
  /// * may contain alphanumeric characters ([0-9|a-z|A-Z]), underscores (_), or
  ///   forward slashes (/)
  /// * may use balanced curly braces ({}) for substitutions
  /// * may start with a tilde (~), the private namespace substitution character
  /// * must not start with a numeric character ([0-9])
  /// * must not end with a forward slash (/)
  /// * must not contain any number of repeated forward slashes (/)
  /// * must not contain any number of repeated underscores (_)
  /// * must separate a tilde (~) from the rest of the name with a forward slash
  ///   (/), i.e. ~/foo not ~foo
  /// * must have balanced curly braces ({}) when used, i.e. {sub}/foo but not
  ///   {sub/foo nor /foo}
  pub fn create_ros_topic(
    &self,
    name: &str,
    type_name: String,
    qos: &QosPolicies,
    topic_kind: TopicKind,
  ) -> Result<Topic, Error> {
    if name.is_empty() {
      return Error::bad_parameter("Topic name must not be empty.");
    }
    // TODO: Implement the rest of the rules.

    let mut oname = "rt/".to_owned();
    let name_stripped = name.strip_prefix('/').unwrap_or(name); // avoid double slash in name
    oname.push_str(name_stripped);
    info!("Creating topic, DDS name: {}", oname);
    let topic = self
      .ros_participant
      .domain_participant()
      .create_topic(oname, type_name, qos, topic_kind)?;
    info!("Created topic");
    Ok(topic)
  }

  /// Creates ROS2 Subscriber to no key topic.
  ///
  /// # Arguments
  ///
  /// * `topic` - Reference to topic created with `create_ros_topic`.
  /// * `qos` - Should take [QOS](../dds/qos/struct.QosPolicies.html) and use if
  ///   it's compatible with topics QOS. `None` indicates the use of Topics QOS.
  pub fn create_ros_nokey_subscriber<
    D: DeserializeOwned + 'static,
    DA: no_key::DeserializerAdapter<D>,
  >(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<RosSubscriber<D, DA>, Error> {
    let sub = self
      .ros_participant
      .get_ros_discovery_subscriber()
      .create_datareader_no_key::<D, DA>(topic, qos)?;
    self.add_reader(sub.guid());
    Ok(sub)
  }

  /// Creates ROS2 Subscriber to [Keyed](../dds/traits/trait.Keyed.html) topic.
  ///
  /// # Arguments
  ///
  /// * `topic` - Reference to topic created with `create_ros_topic`.
  /// * `qos` - Should take [QOS](../dds/qos/struct.QosPolicies.html) and use it
  ///   if it's compatible with topics QOS. `None` indicates the use of Topics
  ///   QOS.
  pub fn create_ros_subscriber<D, DA: with_key::DeserializerAdapter<D>>(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<KeyedRosSubscriber<D, DA>, Error>
  where
    D: Keyed + DeserializeOwned + 'static,
    D::K: Key,
  {
    let sub = self
      .ros_participant
      .get_ros_discovery_subscriber()
      .create_datareader::<D, DA>(topic, qos)?;
    self.add_reader(sub.guid());
    Ok(sub)
  }

  /// Creates ROS2 Publisher to no key topic.
  ///
  /// # Arguments
  ///
  /// * `topic` - Reference to topic created with `create_ros_topic`.
  /// * `qos` - Should take [QOS](../dds/qos/struct.QosPolicies.html) and use it
  ///   if it's compatible with topics QOS. `None` indicates the use of Topics
  ///   QOS.
  pub fn create_ros_nokey_publisher<D: Serialize, SA: no_key::SerializerAdapter<D>>(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<RosPublisher<D, SA>, Error> {
    let p = self
      .ros_participant
      .get_ros_discovery_publisher()
      .create_datawriter_no_key(topic, qos)?;
    self.add_writer(p.guid());
    Ok(p)
  }

  /// Creates ROS2 Publisher to [Keyed](../dds/traits/trait.Keyed.html) topic.
  ///
  /// # Arguments
  ///
  /// * `topic` - Reference to topic created with `create_ros_topic`.
  /// * `qos` - Should take [QOS](../dds/qos/struct.QosPolicies.html) and use it
  ///   if it's compatible with topics QOS. `None` indicates the use of Topics
  ///   QOS.
  pub fn create_ros_publisher<D, SA: with_key::SerializerAdapter<D>>(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<KeyedRosPublisher<D, SA>, Error>
  where
    D: Keyed + Serialize,
    D::K: Key,
  {
    let p = self
      .ros_participant
      .get_ros_discovery_publisher()
      .create_datawriter(topic, qos)?;
    self.add_writer(p.guid());
    Ok(p)
  }
}
