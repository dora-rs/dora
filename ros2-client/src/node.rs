use std::collections::HashSet;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use serde::{de::DeserializeOwned, Serialize};
use rustdds::*;

use crate::{
  action::*,
  context::Context,
  gid::Gid,
  log::Log,
  message::MessageTypeName,
  node_entities_info::NodeEntitiesInfo,
  parameters::*,
  pubsub::{Publisher, Subscription},
  service::{Client, Server, Service, ServiceMapping},
  SubscriptionUntyped,
};

/// Configuration of [Node]
/// This is a builder-like struct.
pub struct NodeOptions {
  #[allow(dead_code)]
  cli_args: Vec<String>,
  #[allow(dead_code)]
  use_global_arguments: bool, // process-wide command line args
  enable_rosout: bool, // use rosout topic for logging?
  #[allow(dead_code)]
  start_parameter_services: bool,
  #[allow(dead_code)]
  parameter_overrides: Vec<Parameter>,
  #[allow(dead_code)]
  allow_undeclared_parameters: bool,
  #[allow(dead_code)]
  automatically_declare_parameters_from_overrides: bool,
  // The NodeOptions struct does not contain
  // node_name, context, or namespace, because
  // they ae always needed and have no reasonable default.
}

impl NodeOptions {
  /// Get a default NodeOptions
  pub fn new() -> NodeOptions {
    // These defaults are from rclpy reference
    // https://docs.ros2.org/latest/api/rclpy/api/node.html
    NodeOptions {
      cli_args: Vec::new(),
      use_global_arguments: true,
      enable_rosout: true,
      start_parameter_services: true,
      parameter_overrides: Vec::new(),
      allow_undeclared_parameters: false,
      automatically_declare_parameters_from_overrides: false,
    }
  }
  pub fn enable_rosout(self, enable: bool) -> NodeOptions {
    NodeOptions {
      enable_rosout: enable,
      ..self
    }
  }
}

impl Default for NodeOptions {
  fn default() -> Self {
    Self::new()
  }
}
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

/// Node in ROS2 network. Holds necessary readers and writers for rosout and
/// parameter events topics internally.
///
/// These are produced by a [`Context`].
pub struct Node {
  // node info
  name: String,
  namespace: String,
  options: NodeOptions,

  pub(crate) ros_context: Context,

  // dynamic
  readers: HashSet<GUID>,
  writers: HashSet<GUID>,

  // builtin writers and readers
  rosout_writer: Option<Publisher<Log>>,
  #[allow(dead_code)]
  rosout_reader: Option<Subscription<Log>>,
  parameter_events_writer: Publisher<raw::ParameterEvent>,
}

impl Node {
  pub(crate) fn new(
    name: &str,
    namespace: &str,
    options: NodeOptions,
    ros_context: Context,
  ) -> Result<Node, dds::CreateError> {
    let paramtopic = ros_context.get_parameter_events_topic();
    let rosout_topic = ros_context.get_rosout_topic();

    let rosout_writer = if options.enable_rosout {
      Some(
        // topic already has QoS defined
        ros_context.create_publisher(&rosout_topic, None)?,
      )
    } else {
      None
    };

    let parameter_events_writer = ros_context
      // topic already has QoS defined
      .create_publisher(&paramtopic, None)?;

    Ok(Node {
      name: String::from(name),
      namespace: String::from(namespace),
      options,
      ros_context,
      readers: HashSet::new(),
      writers: HashSet::new(),
      rosout_writer,
      rosout_reader: None, //TODO
      parameter_events_writer,
    })
  }

  // Generates ROS2 node info from added readers and writers.
  fn generate_node_info(&self) -> NodeEntitiesInfo {
    let mut node_info = NodeEntitiesInfo::new(self.name.clone(), self.namespace.clone());

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
    self.ros_context.add_node_info(self.generate_node_info());
  }

  pub fn remove_reader(&mut self, reader: &GUID) {
    self.readers.remove(reader);
    self.ros_context.add_node_info(self.generate_node_info());
  }

  fn add_writer(&mut self, writer: GUID) {
    self.writers.insert(writer);
    self.ros_context.add_node_info(self.generate_node_info());
  }

  pub fn remove_writer(&mut self, writer: &GUID) {
    self.writers.remove(writer);
    self.ros_context.add_node_info(self.generate_node_info());
  }

  /// Clears both all reader and writer guids from this node.
  pub fn clear_node(&mut self) {
    self.readers.clear();
    self.writers.clear();
    self.ros_context.add_node_info(self.generate_node_info());
  }

  pub fn name(&self) -> &str {
    &self.name
  }

  pub fn namespace(&self) -> &str {
    &self.namespace
  }

  pub fn fully_qualified_name(&self) -> String {
    let mut nn = self.name.clone();
    nn.push_str(&self.namespace);
    nn
  }

  pub fn options(&self) -> &NodeOptions {
    &self.options
  }

  pub fn domain_id(&self) -> u16 {
    self.ros_context.domain_id()
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
  pub fn create_topic(
    &self,
    name: &str,
    type_name: String,
    qos: &QosPolicies,
  ) -> Result<Topic, dds::CreateError> {
    let oname = Self::check_name_and_add_prefix("rt/".to_owned(), name)?;
    info!("Creating topic, DDS name: {}", oname);
    let topic = self.ros_context.domain_participant().create_topic(
      oname,
      type_name,
      qos,
      TopicKind::NoKey,
    )?;
    // ROS2 does not use WithKey topics, so always NoKey
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
  pub fn create_subscription<D: DeserializeOwned + 'static>(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<Subscription<D>, dds::CreateError>
  where
    D: 'static,
  {
    let sub = self.ros_context.create_subscription(topic, qos)?;
    self.add_reader(sub.guid());
    Ok(sub)
  }

  fn check_name_and_add_prefix(mut prefix: String, name: &str) -> Result<String, dds::CreateError> {
    if name.is_empty() {
      return Err(dds::CreateError::BadParameter {
        reason: "Topic name must not be empty.".into(),
      });
    }
    // TODO: Implement the rest of the ROS2 name rules.
    // See https://design.ros2.org/articles/topic_and_service_names.html
    // --> "ROS 2 Topic and Service Name Constraints"

    // avoid double slash in name
    prefix.push_str(name.strip_prefix('/').unwrap_or(name));
    Ok(prefix)
  }

  /// Creates ROS2 Publisher to no key topic.
  ///
  /// # Arguments
  ///
  /// * `topic` - Reference to topic created with `create_ros_topic`.
  /// * `qos` - Should take [QOS](../dds/qos/struct.QosPolicies.html) and use it
  ///   if it's compatible with topics QOS. `None` indicates the use of Topics
  ///   QOS.
  pub fn create_publisher<D: Serialize>(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<Publisher<D>, dds::CreateError> {
    let p = self.ros_context.create_publisher(topic, qos)?;
    self.add_writer(p.guid());
    Ok(p)
  }

  pub(crate) fn create_simpledatareader<D, DA>(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<no_key::SimpleDataReader<D, DA>, dds::CreateError>
  where
    D: 'static,
    DA: rustdds::no_key::DeserializerAdapter<D> + 'static,
  {
    self.ros_context.create_simpledatareader(topic, qos)
  }

  pub(crate) fn create_datawriter<D, SA>(
    &mut self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<no_key::DataWriter<D, SA>, dds::CreateError>
  where
    SA: rustdds::no_key::SerializerAdapter<D>,
  {
    self.ros_context.create_datawriter(topic, qos)
  }

  /// Creates ROS2 Service Client
  ///
  /// # Arguments
  ///
  /// * `service_mapping` - ServiceMapping to be used
  /// * `service_name` -
  /// * `qos`-
  pub fn create_client<S>(
    &mut self,
    service_mapping: ServiceMapping,
    service_name: &str,
    request_type_name: &str,
    response_type_name: &str,
    request_qos: QosPolicies,
    response_qos: QosPolicies,
  ) -> Result<Client<S>, dds::CreateError>
  where
    S: Service + 'static,
    S::Request: Clone,
  {
    // Add rq/ and rr/ prefixes as documented in
    // https://design.ros2.org/articles/topic_and_service_names.html
    // Where are the suffixes documented?
    // And why "Reply" and not "Response" ?
    let rq_name =
      Self::check_name_and_add_prefix("rq/".to_owned(), &(service_name.to_owned() + "Request"))?;
    let rs_name =
      Self::check_name_and_add_prefix("rr/".to_owned(), &(service_name.to_owned() + "Reply"))?;

    let rq_topic = self.ros_context.domain_participant().create_topic(
      rq_name,
      request_type_name.to_string(),
      &request_qos,
      TopicKind::NoKey,
    )?;
    let rs_topic = self.ros_context.domain_participant().create_topic(
      rs_name,
      response_type_name.to_string(),
      &response_qos,
      TopicKind::NoKey,
    )?;

    let c = Client::<S>::new(
      service_mapping,
      self,
      &rq_topic,
      &rs_topic,
      Some(request_qos),
      Some(response_qos),
    )?;

    Ok(c)
  }

  /// Creates ROS2 Service Server
  ///
  /// # Arguments
  ///
  /// * `service_mapping` - ServiceMapping to be used. See
  ///   [`Self.create_client`].
  /// * `service_name` -
  /// * `qos`-
  pub fn create_server<S>(
    &mut self,
    service_mapping: ServiceMapping,
    service_name: &str,
    request_type_name: &str,
    response_type_name: &str,
    request_qos: QosPolicies,
    response_qos: QosPolicies,
  ) -> Result<Server<S>, dds::CreateError>
  where
    S: Service + 'static,
    S::Request: Clone,
  {
    let rq_name =
      Self::check_name_and_add_prefix("rq/".to_owned(), &(service_name.to_owned() + "Request"))?;
    let rs_name =
      Self::check_name_and_add_prefix("rr/".to_owned(), &(service_name.to_owned() + "Reply"))?;

    let rq_topic = self.ros_context.domain_participant().create_topic(
      rq_name,
      request_type_name.to_string(),
      &request_qos,
      TopicKind::NoKey,
    )?;
    let rs_topic = self.ros_context.domain_participant().create_topic(
      rs_name,
      response_type_name.to_string(),
      &response_qos,
      TopicKind::NoKey,
    )?;

    let s = Server::<S>::new(
      service_mapping,
      self,
      &rq_topic,
      &rs_topic,
      Some(request_qos),
      Some(response_qos),
    )?;

    Ok(s)
  }

  pub fn create_action_client<A>(
    &mut self,
    service_mapping: ServiceMapping,
    action_name: &str,
    action_type_name: &MessageTypeName,
    action_qos: ActionClientQosPolicies,
  ) -> Result<ActionClient<A>, dds::CreateError>
  where
    A: ActionTypes + 'static,
  {
    // action name is e.g. "/turtle1/rotate_absolute"
    // action type name is e.g. "turtlesim/action/RotateAbsolute"

    let goal_service_name = action_name.to_owned() + "/_action/send_goal";
    let goal_service_req_type = action_type_name.dds_action_type() + "_SendGoal_Request_";
    let goal_service_resp_type = action_type_name.dds_action_type() + "_SendGoal_Response_";
    let my_goal_client = self.create_client(
      service_mapping,
      &goal_service_name,
      &goal_service_req_type,
      &goal_service_resp_type,
      action_qos.goal_service.clone(),
      action_qos.goal_service,
    )?;

    let cancel_service_name = action_name.to_owned() + "/_action/cancel_goal";
    let cancel_goal_type = MessageTypeName::new("action_msgs", "CancelGoal");
    let cancel_service_req_type = cancel_goal_type.dds_request_type();
    let cancel_service_resp_type = cancel_goal_type.dds_response_type();
    let my_cancel_client = self.create_client(
      service_mapping,
      &cancel_service_name,
      &cancel_service_req_type,
      &cancel_service_resp_type,
      action_qos.cancel_service.clone(),
      action_qos.cancel_service,
    )?;

    let result_service_name = action_name.to_owned() + "/_action/get_result";
    let result_service_req_type = action_type_name.dds_action_type() + "_GetResult_Request_";
    let result_service_resp_type = action_type_name.dds_action_type() + "_GetResult_Response_";
    let my_result_client = self.create_client(
      service_mapping,
      &result_service_name,
      &result_service_req_type,
      &result_service_resp_type,
      action_qos.result_service.clone(),
      action_qos.result_service,
    )?;

    let feedback_topic_name = action_name.to_owned() + "/_action/feedback";
    let feedback_topic_type = action_type_name.dds_action_type() + "_FeedbackMessage_";
    let feedback_topic = self.create_topic(
      &feedback_topic_name,
      feedback_topic_type,
      &action_qos.feedback_subscription,
    )?;
    let my_feedback_subscription =
      self.create_subscription(&feedback_topic, Some(action_qos.feedback_subscription))?;

    let status_topic_name = action_name.to_owned() + "/_action/status";
    let status_topic_type = MessageTypeName::new("action_msgs", "GoalStatusArray").dds_msg_type();
    let status_topic = self.create_topic(
      &status_topic_name,
      status_topic_type,
      &action_qos.status_subscription,
    )?;
    let my_status_subscription =
      self.create_subscription(&status_topic, Some(action_qos.status_subscription))?;

    Ok(ActionClient {
      my_goal_client,
      my_cancel_client,
      my_result_client,
      my_feedback_subscription,
      my_status_subscription,
      my_action_name: action_name.to_owned(),
    })
  }

  pub fn create_action_server<A>(
    &mut self,
    service_mapping: ServiceMapping,
    action_name: &str,
    action_type_name: &MessageTypeName,
    action_qos: ActionServerQosPolicies,
  ) -> Result<ActionServer<A>, dds::CreateError>
  where
    A: ActionTypes + 'static,
  {
    let goal_service_name = action_name.to_owned() + "/_action/send_goal";
    let goal_service_req_type = action_type_name.dds_action_type() + "_SendGoal_Request_";
    let goal_service_resp_type = action_type_name.dds_action_type() + "_SendGoal_Response_";
    let my_goal_server = self.create_server(
      service_mapping,
      &goal_service_name,
      &goal_service_req_type,
      &goal_service_resp_type,
      action_qos.goal_service.clone(),
      action_qos.goal_service,
    )?;

    let cancel_service_name = action_name.to_owned() + "/_action/cancel_goal";
    let cancel_goal_type = MessageTypeName::new("action_msgs", "CancelGoal");
    let cancel_service_req_type = cancel_goal_type.dds_request_type();
    let cancel_service_resp_type = cancel_goal_type.dds_response_type();
    let my_cancel_server = self.create_server(
      service_mapping,
      &cancel_service_name,
      &cancel_service_req_type,
      &cancel_service_resp_type,
      action_qos.cancel_service.clone(),
      action_qos.cancel_service,
    )?;

    let result_service_name = action_name.to_owned() + "/_action/get_result";
    let result_service_req_type = action_type_name.dds_action_type() + "_GetResult_Request_";
    let result_service_resp_type = action_type_name.dds_action_type() + "_GetResult_Response_";
    let my_result_server = self.create_server(
      service_mapping,
      &result_service_name,
      &result_service_req_type,
      &result_service_resp_type,
      action_qos.result_service.clone(),
      action_qos.result_service,
    )?;

    let feedback_topic_name = action_name.to_owned() + "/_action/feedback";
    let feedback_topic_type = action_type_name.dds_action_type() + "_FeedbackMessage_";
    let feedback_topic = self.create_topic(
      &feedback_topic_name,
      feedback_topic_type,
      &action_qos.feedback_publisher,
    )?;
    let my_feedback_publisher =
      self.create_publisher(&feedback_topic, Some(action_qos.feedback_publisher))?;

    let status_topic_name = action_name.to_owned() + "/_action/status";
    let status_topic_type = MessageTypeName::new("action_msgs", "GoalStatusArray").dds_msg_type();
    let status_topic = self.create_topic(
      &status_topic_name,
      status_topic_type,
      &action_qos.status_publisher,
    )?;
    let my_status_publisher =
      self.create_publisher(&status_topic, Some(action_qos.status_publisher))?;

    Ok(ActionServer {
      my_goal_server,
      my_cancel_server,
      my_result_server,
      my_feedback_publisher,
      my_status_publisher,
      my_action_name: action_name.to_owned(),
    })
  }
}
