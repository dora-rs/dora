use std::{fmt::Debug, sync::Arc};

use crate::dds::{
  dds_entity::DDSEntity,
  participant::{DomainParticipant, DomainParticipantWeak},
  qos::{HasQoSPolicy, QosPolicies},
  typedesc::TypeDesc,
};
pub use crate::structure::topic_kind::TopicKind;

/// Trait approximation of DDS 2.2.2.3.1 TopicDescription Class
///
/// Now it is utterly useless, but if we ever add ContentFilteredTopic or
/// MultiTopic, then it may turn out to be useful.
pub trait TopicDescription {
  fn participant(&self) -> Option<DomainParticipant>;
  fn get_type(&self) -> TypeDesc; // This replaces type_name() from spec
  fn name(&self) -> String;
}

/// DDS Topic
///
/// DDS Specification, Section 2.2.1.2 Conceptual outline:
/// > Topic objects conceptually fit between publications and subscriptions.
/// > Publications must be known in such a way that
/// > subscriptions can refer to them unambiguously. A Topic is meant to fulfill
/// that purpose: it associates a name (unique in the > domain), a data-type,
/// and QoS related to the data itself.
///
/// Topics can be created (or found) using a [`DomainParticipant`].
///
/// # Examples
///
/// ```
/// use rustdds::*;
///
/// let domain_participant = DomainParticipant::new(0).unwrap();
/// let qos = QosPolicyBuilder::new().build();
/// let topic = domain_participant
///       .create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey)
///       .unwrap();
/// ```
#[derive(Clone)]
pub struct Topic {
  //TODO: Do we really need set_qos operation?
  // Maybe not. Let's make Topic immutable.
  inner: Arc<InnerTopic>,
}

impl Topic {
  pub(crate) fn new(
    my_domainparticipant: &DomainParticipantWeak,
    my_name: String,
    my_typedesc: TypeDesc,
    my_qos_policies: &QosPolicies,
    topic_kind: TopicKind,
  ) -> Self {
    Self {
      inner: Arc::new(InnerTopic::new(
        my_domainparticipant,
        my_name,
        my_typedesc,
        my_qos_policies,
        topic_kind,
      )),
    }
  }

  // This is private, because it is made public via the TopicDescription trait
  fn participant(&self) -> Option<DomainParticipant> {
    self.inner.participant()
  }

  // This is private, because it is made public via the TopicDescription trait
  fn get_type(&self) -> TypeDesc {
    self.inner.get_type()
  }

  // This is private, because it is made public via the TopicDescription trait
  fn name(&self) -> String {
    self.inner.name()
  }

  /// Gets Topics TopicKind
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::*;
  ///
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// let topic = domain_participant
  ///     .create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey)
  ///     .unwrap();
  /// assert_eq!(topic.kind(), TopicKind::WithKey);
  /// ```
  pub fn kind(&self) -> TopicKind {
    self.inner.kind()
  }
  /*
  // DDS spec 2.2.2.3.2 Topic Class
  // specifies only method get_inconsistent_topic_status
  // TODO: implement
  pub(crate) fn get_inconsistent_topic_status() -> Result<InconsistentTopicStatus> {
    unimplemented!()
  }
  */
}

impl PartialEq for Topic {
  fn eq(&self, other: &Self) -> bool {
    self.inner == other.inner
  }
}

impl Debug for Topic {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    self.inner.fmt(f)
  }
}

/// Implements some default topic interfaces functions defined in DDS spec
impl TopicDescription for Topic {
  /// Gets [DomainParticipant](struct.DomainParticipant.html) if it is still
  /// alive.
  fn participant(&self) -> Option<DomainParticipant> {
    self.participant()
  }

  /// Gets type description of this Topic
  fn get_type(&self) -> TypeDesc {
    self.get_type()
  }

  /// Gets name of this topic
  fn name(&self) -> String {
    self.name()
  }
}

impl HasQoSPolicy for Topic {
  fn qos(&self) -> QosPolicies {
    self.inner.qos()
  }
}

//impl DDSEntity for Topic {}

// -------------------------------- InnerTopic -----------------------------

#[derive(Clone)]
pub struct InnerTopic {
  my_domainparticipant: DomainParticipantWeak,
  my_name: String,
  my_typedesc: TypeDesc,
  my_qos_policies: QosPolicies,
  topic_kind: TopicKind, // WITH_KEY or NO_KEY
}

impl InnerTopic {
  // visibility pub(crate), because only DomainParticipant should be able to
  // create new Topic objects from an application point of view.
  fn new(
    my_domainparticipant: &DomainParticipantWeak,
    my_name: String,
    my_typedesc: TypeDesc,
    my_qos_policies: &QosPolicies,
    topic_kind: TopicKind,
  ) -> Self {
    Self {
      my_domainparticipant: my_domainparticipant.clone(),
      my_name,
      my_typedesc,
      my_qos_policies: my_qos_policies.clone(),
      topic_kind,
    }
  }

  fn participant(&self) -> Option<DomainParticipant> {
    self.my_domainparticipant.clone().upgrade()
  }

  fn get_type(&self) -> TypeDesc {
    self.my_typedesc.clone()
  }

  fn name(&self) -> String {
    self.my_name.to_string()
  }

  pub fn kind(&self) -> TopicKind {
    self.topic_kind
  }
  /*
  pub(crate) fn get_inconsistent_topic_status() -> Result<TopicStatus> {
    unimplemented!()
  } */
}

impl PartialEq for InnerTopic {
  fn eq(&self, other: &Self) -> bool {
    self.participant() == other.participant()
      && self.get_type() == other.get_type()
      && self.name() == other.name()
      && self.qos() == other.qos()
      && self.topic_kind == other.topic_kind
  }
}

impl Debug for InnerTopic {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.write_fmt(format_args!("{:?}", self.participant()))?;
    f.write_fmt(format_args!("Topic name: {}", self.name()))?;
    f.write_fmt(format_args!("Topic type: {:?}", self.get_type()))?;
    f.write_fmt(format_args!("Topic QoS: {:?} ", self.qos()))
  }
}

/// Implements some default topic interfaces functions defined in DDS spec
impl TopicDescription for InnerTopic {
  /// Gets [DomainParticipant](struct.DomainParticipant.html) if it is still
  /// alive.
  fn participant(&self) -> Option<DomainParticipant> {
    self.participant()
  }

  /// Gets type description of this Topic
  fn get_type(&self) -> TypeDesc {
    self.get_type()
  }

  /// Gets name of this topic
  fn name(&self) -> String {
    self.name()
  }
}

impl HasQoSPolicy for InnerTopic {
  fn qos(&self) -> QosPolicies {
    self.my_qos_policies.clone()
  }
}

impl DDSEntity for InnerTopic {}
