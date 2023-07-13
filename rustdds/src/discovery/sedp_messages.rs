use std::time::Instant;

use serde::{Deserialize, Deserializer, Serialize, Serializer};
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use bytes::Bytes;
use speedy::{Readable, Writable};
use chrono::{DateTime, Utc};
use cdr_encoding_size::*;

use crate::{
  dds::{
    adapters::with_key::SerializerAdapter,
    participant::DomainParticipant,
    qos::{
      policy::{
        Deadline, DestinationOrder, Durability, History, LatencyBudget, Lifespan, Liveliness,
        Ownership, Presentation, Reliability, ResourceLimits, TimeBasedFilter,
      },
      HasQoSPolicy, QosPolicies,
    },
    topic::{Topic, TopicDescription},
    with_key::datawriter::DataWriter,
  },
  discovery::content_filter_property::ContentFilterProperty,
  messages::submessages::elements::{parameter::Parameter, parameter_list::ParameterList},
  network::{constant::user_traffic_unicast_port, util::get_local_unicast_locators},
  rtps::{rtps_reader_proxy::RtpsReaderProxy, rtps_writer_proxy::RtpsWriterProxy},
  security::EndpointSecurityInfo,
  serialization::{
    pl_cdr_adapters::{
      PlCdrDeserialize, PlCdrDeserializeError, PlCdrSerialize, PlCdrSerializeError,
    },
    representation_identifier::RepresentationIdentifier,
    speedy_pl_cdr_helpers::*,
  },
  structure::{
    entity::RTPSEntity,
    guid::{GuidPrefix, GUID},
    locator,
    locator::Locator,
    parameter_id::ParameterId,
  },
  Key, Keyed,
};
#[cfg(test)]
use crate::structure::guid::EntityKind;

macro_rules! fake_implement_serialize_and_deserialize {
  ($t:ty) => {
    impl Serialize for $t {
      fn serialize<S>(&self, _serializer: S) -> Result<S::Ok, S::Error>
      where
        S: Serializer,
      {
        unimplemented!()
      }
    }
    impl<'de> Deserialize<'de> for $t {
      fn deserialize<D>(_deserializer: D) -> Result<$t, D::Error>
      where
        D: Deserializer<'de>,
      {
        unimplemented!()
      }
    }
  };
}

// We need a wrapper to distinguish between Participant and Endpoint GUIDs.
// They need to be distinguished, because the PL_CDR serialization is different:
// ParameterId is different.

#[allow(non_camel_case_types)]
#[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy, CdrEncodingSize, Hash, Serialize)]
pub struct Endpoint_GUID(pub GUID);

impl Key for Endpoint_GUID {}

impl PlCdrDeserialize for Endpoint_GUID {
  fn from_pl_cdr_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<Self, PlCdrDeserializeError> {
    let ctx = pl_cdr_rep_id_to_speedy_d(encoding)?;
    let pl = ParameterList::read_from_buffer_with_ctx(ctx, input_bytes)?;
    let pl_map = pl.to_map();

    let guid: GUID = get_first_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_ENDPOINT_GUID,
      "Endpoint GUID",
    )?;

    Ok(Endpoint_GUID(guid))
  }
}

impl PlCdrSerialize for Endpoint_GUID {
  fn to_pl_cdr_bytes(
    &self,
    encoding: RepresentationIdentifier,
  ) -> Result<Bytes, PlCdrSerializeError> {
    let mut pl = ParameterList::new();
    let ctx = pl_cdr_rep_id_to_speedy(encoding)?;
    macro_rules! emit {
      ($pid:ident, $member:expr, $type:ty) => {
        pl.push(Parameter::new(ParameterId::$pid, {
          let m: &$type = $member;
          m.write_to_vec_with_ctx(ctx)?
        }))
      };
    }
    emit!(PID_ENDPOINT_GUID, &self.0, GUID);
    let bytes = pl.serialize_to_bytes(ctx)?;
    Ok(bytes)
  }
}

// Topic data contains all topic related
// (including reader and writer data structures for serialization and
// deserialization)

/// Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReaderProxy {
  pub remote_reader_guid: GUID,
  pub expects_inline_qos: bool,
  pub unicast_locator_list: Vec<Locator>,
  pub multicast_locator_list: Vec<Locator>,
}

impl ReaderProxy {
  pub fn new(
    guid: GUID,
    expects_inline_qos: bool,
    unicast_locator_list: Vec<Locator>,
    multicast_locator_list: Vec<Locator>,
  ) -> Self {
    Self {
      remote_reader_guid: guid,
      expects_inline_qos,
      unicast_locator_list,
      multicast_locator_list,
    }
  }
}

impl From<RtpsReaderProxy> for ReaderProxy {
  fn from(rtps_reader_proxy: RtpsReaderProxy) -> Self {
    Self {
      remote_reader_guid: rtps_reader_proxy.remote_reader_guid,
      expects_inline_qos: rtps_reader_proxy.expects_in_line_qos,
      unicast_locator_list: rtps_reader_proxy.unicast_locator_list,
      multicast_locator_list: rtps_reader_proxy.multicast_locator_list,
    }
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

/// DDS SubscriptionBuiltinTopicData
/// Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SubscriptionBuiltinTopicData {
  key: GUID,
  participant_key: Option<GUID>,
  topic_name: String,
  type_name: String,
  durability: Option<Durability>,
  deadline: Option<Deadline>,
  latency_budget: Option<LatencyBudget>,
  liveliness: Option<Liveliness>,
  reliability: Option<Reliability>,
  ownership: Option<Ownership>,
  destination_order: Option<DestinationOrder>,
  // pub user_data: Option<UserData>,
  time_based_filter: Option<TimeBasedFilter>,
  presentation: Option<Presentation>,
  // pub partition: Option<Partition>,
  // pub topic_data: Option<TopicData>,
  // pub group_data: Option<GroupData>,
  // pub durability_service: Option<DurabilityService>,
  lifespan: Option<Lifespan>,

  // From spec Remote Procedure Call over DDS:
  service_instance_name: Option<String>,
  related_datawriter_key: Option<GUID>,
  topic_aliases: Option<Vec<String>>, /* Option is a bit redundant, but it indicates if the
                                       * parameter was present or not */
  // DDS Security:
  security_info: Option<EndpointSecurityInfo>,
}

impl SubscriptionBuiltinTopicData {
  pub fn new(
    key: GUID,
    participant_key: Option<GUID>,
    topic_name: String,
    type_name: String,
    qos: &QosPolicies,
    security_info: Option<EndpointSecurityInfo>,
  ) -> Self {
    let mut sbtd = Self {
      key,
      participant_key,
      topic_name,
      type_name,
      // QoS
      durability: None,
      deadline: None,
      latency_budget: None,
      liveliness: None,
      reliability: None,
      ownership: None,
      destination_order: None,
      time_based_filter: None,
      presentation: None,
      lifespan: None,
      // DDS-RPC
      // TODO: these are not implemented
      service_instance_name: None,  // Note: Not implemented
      related_datawriter_key: None, // Note: Not implemented
      topic_aliases: None,          // Note: Not implemented
      // DDS Security
      security_info,
    };

    sbtd.set_qos(qos);
    sbtd
  }

  pub fn key(&self) -> GUID {
    self.key
  }

  pub fn participant_key(&self) -> &Option<GUID> {
    &self.participant_key
  }

  pub fn topic_name(&self) -> &String {
    &self.topic_name
  }

  pub fn type_name(&self) -> &String {
    &self.type_name
  }

  pub fn security_info(&self) -> &Option<EndpointSecurityInfo> {
    &self.security_info
  }

  pub fn set_qos(&mut self, qos: &QosPolicies) {
    self.durability = qos.durability;
    self.deadline = qos.deadline;
    self.latency_budget = qos.latency_budget;
    self.liveliness = qos.liveliness;
    self.reliability = qos.reliability;
    self.ownership = qos.ownership;
    self.destination_order = qos.destination_order;
    self.time_based_filter = qos.time_based_filter;
    self.presentation = qos.presentation;
    self.lifespan = qos.lifespan;
    // history does not exist
    // resource_limits does not exist
  }

  pub fn qos(&self) -> QosPolicies {
    QosPolicies {
      durability: self.durability,
      presentation: self.presentation,
      deadline: self.deadline,
      latency_budget: self.latency_budget,
      ownership: self.ownership,
      liveliness: self.liveliness,
      time_based_filter: self.time_based_filter,
      reliability: self.reliability,
      destination_order: self.destination_order,
      history: None, // SubscriptionBuiltinTopicData does not contain History QoS
      resource_limits: None, // nor Resource Limits, see Figure 8.30 in RTPS spec 2.5
      lifespan: self.lifespan,
    }
  }

  pub fn to_topic_data(&self) -> TopicBuiltinTopicData {
    // TODO: See the corresponding function in PublicationBuiltinTopicData
    TopicBuiltinTopicData::new(
      None,
      self.topic_name.clone(),
      self.type_name.clone(),
      &self.qos(),
    )
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

/// Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredReaderData {
  pub reader_proxy: ReaderProxy,
  pub subscription_topic_data: SubscriptionBuiltinTopicData,
  pub content_filter: Option<ContentFilterProperty>,
}

fake_implement_serialize_and_deserialize! {DiscoveredReaderData}

impl DiscoveredReaderData {
  // This is for generating test data only
  #[cfg(test)]
  pub fn default(topic_name: String, type_name: String) -> Self {
    let rguid = GUID::dummy_test_guid(EntityKind::READER_WITH_KEY_BUILT_IN);
    let reader_proxy = ReaderProxy::new(rguid, false, vec![], vec![]);
    let subscription_topic_data = SubscriptionBuiltinTopicData::new(
      rguid,
      None,
      topic_name,
      type_name,
      &QosPolicies::builder().build(),
      None,
    );
    Self {
      reader_proxy,
      subscription_topic_data,
      content_filter: None,
    }
  }
}

impl Keyed for DiscoveredReaderData {
  type K = Endpoint_GUID;
  fn key(&self) -> Self::K {
    Endpoint_GUID(self.subscription_topic_data.key)
  }
}

impl PlCdrDeserialize for DiscoveredReaderData {
  fn from_pl_cdr_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<Self, PlCdrDeserializeError> {
    let ctx = pl_cdr_rep_id_to_speedy_d(encoding)?;
    let pl = ParameterList::read_from_buffer_with_ctx(ctx, input_bytes)?;
    let pl_map = pl.to_map();

    let guid: GUID = get_first_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_ENDPOINT_GUID,
      "Endpoint GUID",
    )?;
    let participant_guid: Option<GUID> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PARTICIPANT_GUID,
      "Participant GUID",
    )?;

    let expects_inline_qos : bool = // This one has default value false
      get_option_from_pl_map(&pl_map, ctx, ParameterId::PID_EXPECTS_INLINE_QOS, "Expects inline Qos")?
      .unwrap_or(false);

    let unicast_locator_list: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_DEFAULT_UNICAST_LOCATOR,
      "unicast locators",
    )?;
    let multicast_locator_list: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_DEFAULT_MULTICAST_LOCATOR,
      "multicast locators",
    )?;

    let topic_name : String = // Note the serialized type is StringWithNul
      get_first_from_pl_map::< _ , StringWithNul>(&pl_map, ctx, ParameterId::PID_TOPIC_NAME, "topic name")?
      .into();
    let type_name : String = // Note the serialized type is StringWithNul
      get_first_from_pl_map::< _ , StringWithNul>(&pl_map, ctx, ParameterId::PID_TYPE_NAME, "type name")?
      .into();

    let content_filter: Option<ContentFilterProperty> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_CONTENT_FILTER_PROPERTY,
      "content filter",
    )
    .map_err(|e| {
      warn!(
        "Content filter was: {:?}",
        pl_map.get(&ParameterId::PID_CONTENT_FILTER_PROPERTY)
      );
      e
    })?;

    let security_info: Option<EndpointSecurityInfo> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_ENDPOINT_SECURITY_INFO,
      "endpoint security info",
    )?;

    let qos = QosPolicies::from_parameter_list(ctx, &pl_map)?;

    Ok(DiscoveredReaderData {
      reader_proxy: ReaderProxy::new(
        guid,
        expects_inline_qos,
        unicast_locator_list,
        multicast_locator_list,
      ),
      subscription_topic_data: SubscriptionBuiltinTopicData::new(
        guid,
        participant_guid,
        topic_name,
        type_name,
        &qos,
        security_info,
      ),
      content_filter,
    })
  }
}

impl PlCdrSerialize for DiscoveredReaderData {
  fn to_pl_cdr_bytes(
    &self,
    encoding: RepresentationIdentifier,
  ) -> Result<Bytes, PlCdrSerializeError> {
    let DiscoveredReaderData {
      reader_proxy:
        ReaderProxy {
          remote_reader_guid,
          expects_inline_qos,
          unicast_locator_list,
          multicast_locator_list,
        },
      subscription_topic_data:
        sbtd @ SubscriptionBuiltinTopicData {
          key,
          participant_key,
          topic_name,
          type_name,

          // QoS handled separately
          durability: _,
          deadline: _,
          latency_budget: _,
          liveliness: _,
          reliability: _,
          ownership: _,
          destination_order: _,
          time_based_filter: _,
          presentation: _,
          lifespan: _,

          service_instance_name,
          related_datawriter_key,
          topic_aliases,
          security_info, // TODO: missing implementation
        },
      content_filter,
    } = self;

    let mut pl = ParameterList::new();
    let qos = sbtd.qos();

    let ctx = pl_cdr_rep_id_to_speedy(encoding)?;

    macro_rules! emit {
      ($pid:ident, $member:expr, $type:ty) => {
        pl.push(Parameter::new(ParameterId::$pid, {
          let m: &$type = $member;
          m.write_to_vec_with_ctx(ctx)?
        }))
      };
    }
    macro_rules! emit_option {
      ($pid:ident, $member:expr, $type:ty) => {
        if let Some(m) = $member {
          emit!($pid, m, $type)
        }
      };
    }

    // ReaderProxy
    emit!(PID_EXPECTS_INLINE_QOS, expects_inline_qos, bool);

    // Note that this GUID can be in two places
    emit!(PID_ENDPOINT_GUID, remote_reader_guid, GUID);
    if remote_reader_guid != key {
      // sanity check
      warn!("DiscoveredReaderData: Inconsistent GUID: {remote_reader_guid:?} vs {key:?}");
    }

    for loc in unicast_locator_list {
      emit!(
        PID_DEFAULT_UNICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }
    for loc in multicast_locator_list {
      emit!(
        PID_DEFAULT_MULTICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }

    // SubscriptionBuiltinTopicData
    emit_option!(PID_PARTICIPANT_GUID, participant_key, GUID);
    emit!(PID_TOPIC_NAME, &topic_name.clone().into(), StringWithNul);
    emit!(PID_TYPE_NAME, &type_name.clone().into(), StringWithNul);
    pl.parameters.append(&mut qos.to_parameter_list(ctx)?);
    emit_option!(
      PID_SERVICE_INSTANCE_NAME,
      &service_instance_name.clone().map(|e| e.into()),
      StringWithNul
    );
    emit_option!(PID_RELATED_ENTITY_GUID, related_datawriter_key, GUID);
    // TODO: Should topic alias be on parameter with vector or multiple
    // parameters with one alias name each?
    for topic_alias in topic_aliases.as_ref().unwrap_or(&Vec::new()) {
      emit!(
        PID_TOPIC_ALIASES,
        &topic_alias.clone().into(),
        StringWithNul
      );
    }
    emit_option!(
      PID_CONTENT_FILTER_PROPERTY,
      content_filter,
      ContentFilterProperty
    );

    emit_option!(
      PID_ENDPOINT_SECURITY_INFO,
      security_info,
      EndpointSecurityInfo
    );

    let bytes = pl.serialize_to_bytes(ctx)?;
    Ok(bytes)
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

/// Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WriterProxy {
  pub remote_writer_guid: GUID,
  pub unicast_locator_list: Vec<Locator>,
  pub multicast_locator_list: Vec<Locator>,
  pub data_max_size_serialized: Option<u32>,
}

impl WriterProxy {
  pub fn new(
    guid: GUID,
    multicast_locator_list: Vec<Locator>,
    unicast_locator_list: Vec<Locator>,
  ) -> Self {
    Self {
      remote_writer_guid: guid,
      unicast_locator_list,
      multicast_locator_list,
      data_max_size_serialized: None,
    }
  }
}

impl From<RtpsWriterProxy> for WriterProxy {
  fn from(rtps_writer_proxy: RtpsWriterProxy) -> Self {
    WriterProxy {
      remote_writer_guid: rtps_writer_proxy.remote_writer_guid,
      unicast_locator_list: rtps_writer_proxy.unicast_locator_list,
      multicast_locator_list: rtps_writer_proxy.multicast_locator_list,
      data_max_size_serialized: None,
    }
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

/// Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PublicationBuiltinTopicData {
  pub key: GUID, // endpoint GUID
  pub participant_key: Option<GUID>,
  pub topic_name: String, // TODO: Convert to method for symmetry with SubscriptionBuiltinTopicData
  pub type_name: String,
  pub durability: Option<Durability>,
  pub deadline: Option<Deadline>,
  pub latency_budget: Option<LatencyBudget>,
  pub liveliness: Option<Liveliness>,
  pub reliability: Option<Reliability>,
  pub lifespan: Option<Lifespan>,
  pub time_based_filter: Option<TimeBasedFilter>,
  pub ownership: Option<Ownership>,
  pub destination_order: Option<DestinationOrder>,
  pub presentation: Option<Presentation>,

  // From Remote Procedure Call over DDS:
  pub service_instance_name: Option<String>,
  pub related_datareader_key: Option<GUID>,
  pub topic_aliases: Option<Vec<String>>, /* Option is a bit redundant, but it indicates
                                           * if the parameter was present or not */
  // DDS Security:
  pub security_info: Option<EndpointSecurityInfo>,
}

impl PublicationBuiltinTopicData {
  pub fn new(
    guid: GUID,
    participant_guid: Option<GUID>,
    topic_name: String,
    type_name: String,
    security_info: Option<EndpointSecurityInfo>,
  ) -> Self {
    Self {
      key: guid,
      participant_key: participant_guid,
      topic_name,
      type_name,

      durability: None,
      deadline: None,
      latency_budget: None,
      liveliness: None,
      reliability: None,
      lifespan: None,
      time_based_filter: None,
      ownership: None,
      destination_order: None,
      presentation: None,

      service_instance_name: None,  // TODO: These are not supported/used
      related_datareader_key: None, // TODO
      topic_aliases: None,          // TODO

      security_info,
    }
  }

  pub fn new_with_qos(
    guid: GUID,
    participant_guid: Option<GUID>,
    topic_name: String,
    type_name: String,
    qos: &QosPolicies,
    security_info: Option<EndpointSecurityInfo>,
  ) -> Self {
    let mut s = Self::new(guid, participant_guid, topic_name, type_name, security_info);
    s.set_qos(qos);
    s
  }

  pub fn set_qos(&mut self, qos: &QosPolicies) {
    self.durability = qos.durability;
    self.deadline = qos.deadline;
    self.latency_budget = qos.latency_budget;
    self.liveliness = qos.liveliness;
    self.reliability = qos.reliability;
    self.lifespan = qos.lifespan;
    self.time_based_filter = qos.time_based_filter;
    self.ownership = qos.ownership;
    self.destination_order = qos.destination_order;
    self.presentation = qos.presentation;
  }

  pub fn qos(&self) -> QosPolicies {
    QosPolicies {
      durability: self.durability,
      presentation: self.presentation,
      deadline: self.deadline,
      latency_budget: self.latency_budget,
      ownership: self.ownership,
      liveliness: self.liveliness,
      time_based_filter: self.time_based_filter,
      reliability: self.reliability,
      destination_order: self.destination_order,
      history: None,         // PublicationBuiltinTopicData does not contain History QoS
      resource_limits: None, // nor Resource Limits, see Figure 8.30 in RTPS spec 2.5
      lifespan: self.lifespan,
    }
  }

  pub fn to_topic_data(&self) -> TopicBuiltinTopicData {
    TopicBuiltinTopicData::new(
      None, // This would be topic GUID or BuiltinInTopicKey_t. What is it and who defines it?
      // According to various googled sources, it is either 3x u32 or 4x u32
      // or a GUID or a GuidPrefix. Does it even matter?
      // TODO: Find out.
      self.topic_name.clone(),
      self.type_name.clone(),
      &self.qos(),
    )
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

/// Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiscoveredWriterData {
  pub last_updated: Instant, // last_updated is not serialized

  pub writer_proxy: WriterProxy,
  pub publication_topic_data: PublicationBuiltinTopicData,
}

fake_implement_serialize_and_deserialize! {DiscoveredWriterData}

impl Keyed for DiscoveredWriterData {
  type K = Endpoint_GUID;

  fn key(&self) -> Self::K {
    Endpoint_GUID(self.publication_topic_data.key)
  }
}

impl DiscoveredWriterData {
  pub fn new<D: Keyed, SA: SerializerAdapter<D>>(
    writer: &DataWriter<D, SA>,
    topic: &Topic,
    dp: &DomainParticipant,
    security_info: Option<EndpointSecurityInfo>,
  ) -> Self {
    let unicast_port = user_traffic_unicast_port(dp.domain_id(), dp.participant_id());
    let unicast_addresses = get_local_unicast_locators(unicast_port);
    // TODO: Why empty vector below? No multicast?
    let writer_proxy = WriterProxy::new(writer.guid(), vec![], unicast_addresses);
    let publication_topic_data = PublicationBuiltinTopicData::new_with_qos(
      writer.guid(),
      Some(dp.guid()),
      topic.name(),
      topic.get_type().name().to_string(),
      &writer.qos(),
      security_info,
    );

    Self {
      last_updated: Instant::now(),
      writer_proxy,
      publication_topic_data,
    }
  }
}

impl PlCdrDeserialize for DiscoveredWriterData {
  fn from_pl_cdr_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<Self, PlCdrDeserializeError> {
    let ctx = pl_cdr_rep_id_to_speedy_d(encoding)?;
    let pl = ParameterList::read_from_buffer_with_ctx(ctx, input_bytes)?;
    let pl_map = pl.to_map();

    let guid: GUID = get_first_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_ENDPOINT_GUID,
      "Endpoint GUID",
    )?;
    let participant_guid: Option<GUID> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PARTICIPANT_GUID,
      "Participant GUID",
    )?;

    let unicast_locator_list: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_DEFAULT_UNICAST_LOCATOR,
      "unicast locators",
    )?;
    let multicast_locator_list: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_DEFAULT_MULTICAST_LOCATOR,
      "multicast locators",
    )?;

    let topic_name : String = // Note the serialized type is StringWithNul
      get_first_from_pl_map::< _ , StringWithNul>(&pl_map, ctx, ParameterId::PID_TOPIC_NAME, "topic name")?
      .into();
    let type_name : String = // Note the serialized type is StringWithNul
      get_first_from_pl_map::< _ , StringWithNul>(&pl_map, ctx, ParameterId::PID_TYPE_NAME, "type name")?
      .into();

    let data_max_size_serialized: Option<u32> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_TYPE_MAX_SIZE_SERIALIZED,
      "Max size serialized",
    )?;
    let security_info: Option<EndpointSecurityInfo> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_ENDPOINT_SECURITY_INFO,
      "endpoint security info",
    )?;

    let qos = QosPolicies::from_parameter_list(ctx, &pl_map)?;

    Ok(DiscoveredWriterData {
      last_updated: Instant::now(),
      writer_proxy: WriterProxy {
        remote_writer_guid: guid,
        unicast_locator_list,
        multicast_locator_list,
        data_max_size_serialized,
      },
      publication_topic_data: PublicationBuiltinTopicData::new_with_qos(
        guid,
        participant_guid,
        topic_name,
        type_name,
        &qos,
        security_info,
      ),
    })
  }
}

impl PlCdrSerialize for DiscoveredWriterData {
  fn to_pl_cdr_bytes(
    &self,
    encoding: RepresentationIdentifier,
  ) -> Result<Bytes, PlCdrSerializeError> {
    let DiscoveredWriterData {
      last_updated: _, // This is never serialized
      writer_proxy:
        WriterProxy {
          remote_writer_guid,
          unicast_locator_list,
          multicast_locator_list,
          data_max_size_serialized,
        },
      publication_topic_data:
        pbtd @ PublicationBuiltinTopicData {
          key,
          participant_key,
          topic_name,
          type_name,

          // QoS handled separately
          durability: _,
          deadline: _,
          latency_budget: _,
          liveliness: _,
          reliability: _,
          ownership: _,
          destination_order: _,
          time_based_filter: _,
          presentation: _,
          lifespan: _,

          service_instance_name,
          related_datareader_key,
          topic_aliases,
          security_info,
        },
    } = self;

    let mut pl = ParameterList::new();
    let qos = pbtd.qos();

    let ctx = pl_cdr_rep_id_to_speedy(encoding)?;

    macro_rules! emit {
      ($pid:ident, $member:expr, $type:ty) => {
        pl.push(Parameter::new(ParameterId::$pid, {
          let m: &$type = $member;
          m.write_to_vec_with_ctx(ctx)?
        }))
      };
    }
    macro_rules! emit_option {
      ($pid:ident, $member:expr, $type:ty) => {
        if let Some(m) = $member {
          emit!($pid, m, $type)
        }
      };
    }

    // ReaderProxy
    emit_option!(PID_TYPE_MAX_SIZE_SERIALIZED, data_max_size_serialized, u32);

    // Note that this GUID can be in two places
    emit!(PID_ENDPOINT_GUID, remote_writer_guid, GUID);
    if remote_writer_guid != key {
      // sanity check
      warn!("DiscoveredWriterData: Inconsistent GUID: {remote_writer_guid:?} vs {key:?}");
    }

    for loc in unicast_locator_list {
      emit!(
        PID_DEFAULT_UNICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }
    for loc in multicast_locator_list {
      emit!(
        PID_DEFAULT_MULTICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }

    // SubscriptionBuiltinTopicData
    emit_option!(PID_PARTICIPANT_GUID, participant_key, GUID);
    emit!(PID_TOPIC_NAME, &topic_name.clone().into(), StringWithNul);
    emit!(PID_TYPE_NAME, &type_name.clone().into(), StringWithNul);
    pl.parameters.append(&mut qos.to_parameter_list(ctx)?);
    emit_option!(
      PID_SERVICE_INSTANCE_NAME,
      &service_instance_name.clone().map(|e| e.into()),
      StringWithNul
    );
    emit_option!(PID_RELATED_ENTITY_GUID, related_datareader_key, GUID);
    // TODO: Should topic alias be on parameter with vector or multiple
    // parameters with one alias name each?
    for topic_alias in topic_aliases.as_ref().unwrap_or(&Vec::new()) {
      emit!(
        PID_TOPIC_ALIASES,
        &topic_alias.clone().into(),
        StringWithNul
      );
    }
    emit_option!(
      PID_ENDPOINT_SECURITY_INFO,
      security_info,
      EndpointSecurityInfo
    );

    let bytes = pl.serialize_to_bytes(ctx)?;
    Ok(bytes)
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

/// Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, PartialEq, Eq, Clone)]
pub struct TopicBuiltinTopicData {
  pub key: Option<GUID>,
  pub name: String,
  pub type_name: String,
  pub durability: Option<Durability>,
  pub deadline: Option<Deadline>,
  pub latency_budget: Option<LatencyBudget>,
  pub liveliness: Option<Liveliness>,
  pub reliability: Option<Reliability>,
  pub lifespan: Option<Lifespan>,
  pub destination_order: Option<DestinationOrder>,
  pub presentation: Option<Presentation>,
  pub history: Option<History>,
  pub resource_limits: Option<ResourceLimits>,
  pub ownership: Option<Ownership>,
}

impl TopicBuiltinTopicData {
  pub fn new(key: Option<GUID>, name: String, type_name: String, qos: &QosPolicies) -> Self {
    Self {
      key,
      name,
      type_name,
      durability: qos.durability(),
      deadline: qos.deadline(),
      latency_budget: qos.latency_budget(),
      liveliness: qos.liveliness(),
      reliability: qos.reliability(),
      lifespan: qos.lifespan(),
      destination_order: qos.destination_order(),
      presentation: qos.presentation(),
      history: qos.history(),
      resource_limits: qos.resource_limits(),
      ownership: qos.ownership(),
    }
  }
}

impl HasQoSPolicy for TopicBuiltinTopicData {
  fn qos(&self) -> QosPolicies {
    QosPolicies {
      durability: self.durability,
      presentation: self.presentation,
      deadline: self.deadline,
      latency_budget: self.latency_budget,
      ownership: self.ownership,
      liveliness: self.liveliness,
      time_based_filter: None,
      reliability: self.reliability,
      destination_order: self.destination_order,
      history: self.history,
      resource_limits: self.resource_limits,
      lifespan: self.lifespan,
    }
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

/// DDS Spec defined DiscoveredTopicData with extra updated time attribute.
/// Practically this is gotten from
/// [DomainParticipant](../participant/struct.DomainParticipant.html) during
/// runtime Type specified in RTPS v2.3 spec Figure 8.30
#[derive(Debug, PartialEq, Eq, Clone)]
pub struct DiscoveredTopicData {
  updated_time: DateTime<Utc>,
  pub topic_data: TopicBuiltinTopicData,
}

fake_implement_serialize_and_deserialize! {DiscoveredTopicData}

impl DiscoveredTopicData {
  pub fn new(updated_time: DateTime<Utc>, topic_data: TopicBuiltinTopicData) -> Self {
    Self {
      updated_time,
      topic_data,
    }
  }

  pub fn topic_name(&self) -> &String {
    &self.topic_data.name
  }

  pub fn type_name(&self) -> &String {
    &self.topic_data.type_name
  }
}

impl Keyed for DiscoveredTopicData {
  type K = Endpoint_GUID;

  fn key(&self) -> Self::K {
    Endpoint_GUID(match self.topic_data.key {
      Some(k) => k,
      None => GUID::GUID_UNKNOWN,
    })
  }
}

impl PlCdrDeserialize for DiscoveredTopicData {
  fn from_pl_cdr_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<Self, PlCdrDeserializeError> {
    let ctx = pl_cdr_rep_id_to_speedy_d(encoding)?;
    let pl = ParameterList::read_from_buffer_with_ctx(ctx, input_bytes)?;
    let pl_map = pl.to_map();

    let key: Option<GUID> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_ENDPOINT_GUID,
      "Endpoint GUID",
    )?;

    let name : String = // Note the serialized type is StringWithNul
      get_first_from_pl_map::< _ , StringWithNul>(&pl_map, ctx, ParameterId::PID_TOPIC_NAME, "topic name")?
      .into();
    let type_name : String = // Note the serialized type is StringWithNul
      get_first_from_pl_map::< _ , StringWithNul>(&pl_map, ctx, ParameterId::PID_TYPE_NAME, "type name")?
      .into();

    let qos = QosPolicies::from_parameter_list(ctx, &pl_map)?;

    let topic_data = TopicBuiltinTopicData::new(key, name, type_name, &qos);

    Ok(DiscoveredTopicData {
      updated_time: Utc::now(),
      topic_data,
    })
  }
}

impl PlCdrSerialize for DiscoveredTopicData {
  fn to_pl_cdr_bytes(
    &self,
    encoding: RepresentationIdentifier,
  ) -> Result<Bytes, PlCdrSerializeError> {
    let DiscoveredTopicData {
      updated_time: _, // This is never serialized
      topic_data:
        td @ TopicBuiltinTopicData {
          key,
          name,
          type_name,

          // QoS handled separately
          durability: _,
          deadline: _,
          history: _,
          latency_budget: _,
          liveliness: _,
          reliability: _,
          ownership: _,
          destination_order: _,
          presentation: _,
          lifespan: _,
          resource_limits: _,
        },
    } = self;

    let mut pl = ParameterList::new();
    let qos = td.qos();

    let ctx = pl_cdr_rep_id_to_speedy(encoding)?;

    macro_rules! emit {
      ($pid:ident, $member:expr, $type:ty) => {
        pl.push(Parameter::new(ParameterId::$pid, {
          let m: &$type = $member;
          m.write_to_vec_with_ctx(ctx)?
        }))
      };
    }
    macro_rules! emit_option {
      ($pid:ident, $member:expr, $type:ty) => {
        if let Some(m) = $member {
          emit!($pid, m, $type)
        }
      };
    }

    // Also Topics have GUIDs, but apparently not mandatory?
    emit_option!(PID_ENDPOINT_GUID, key, GUID);
    emit!(PID_TOPIC_NAME, &name.clone().into(), StringWithNul);
    emit!(PID_TYPE_NAME, &type_name.clone().into(), StringWithNul);
    pl.parameters.append(&mut qos.to_parameter_list(ctx)?);

    let bytes = pl.serialize_to_bytes(ctx)?;
    Ok(bytes)
  }
}

// =======================================================================
// =======================================================================
// =======================================================================

#[derive(
  Copy, Clone, Debug, PartialOrd, PartialEq, Ord, Eq, Hash, Serialize, Deserialize, CdrEncodingSize,
)]
pub struct ParticipantMessageDataKind {
  value: [u8; 4],
}

impl ParticipantMessageDataKind {
  #[allow(dead_code)] // This is defined in the spec, but currently unused.
  pub const UNKNOWN: Self = Self {
    value: [0x00, 0x00, 0x00, 0x00],
  };
  pub const AUTOMATIC_LIVELINESS_UPDATE: Self = Self {
    value: [0x00, 0x00, 0x00, 0x01],
  };
  pub const MANUAL_LIVELINESS_UPDATE: Self = Self {
    value: [0x00, 0x00, 0x00, 0x02],
  };
}

// =======================================================================
// =======================================================================
// =======================================================================

// This will be sent using ordinary CDR, not PL_CDR, so derive Serialize &
// Deserialize
#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
pub struct ParticipantMessageData {
  pub guid: GuidPrefix,
  pub kind: ParticipantMessageDataKind,
  pub data: Vec<u8>, // normally this should be empty
}

impl Keyed for ParticipantMessageData {
  type K = ParticipantMessageDataKey;

  fn key(&self) -> Self::K {
    ParticipantMessageDataKey(self.guid, self.kind)
  }
}

#[derive(
  Debug, PartialEq, Eq, Clone, Copy, Hash, Ord, PartialOrd, Serialize, Deserialize, CdrEncodingSize,
)]
pub struct ParticipantMessageDataKey(GuidPrefix, ParticipantMessageDataKind);

impl Key for ParticipantMessageDataKey {}

// =======================================================================
// =======================================================================
// =======================================================================

#[cfg(test)]
mod tests {
  use byteorder::LittleEndian;
  use bytes::Bytes;
  use log::info;
  use test_log::test; // to capture logging macros run by test cases

  use crate::dds::adapters::no_key::SerializerAdapter;
  use super::*;
  // use crate::serialization::cdr_serializer::to_little_endian_binary;
  use crate::{
    dds::adapters::no_key::DeserializerAdapter,
    rtps::Message,
    serialization::pl_cdr_adapters::*,
    test::test_data::{
      content_filter_data, publication_builtin_topic_data, reader_proxy_data,
      subscription_builtin_topic_data, topic_data, writer_proxy_data,
    },
    RepresentationIdentifier,
  };

  /* do not test separate ser/deser of components, as these are never seen on wire individually
    #[test]
    fn td_reader_proxy_ser_deser() {
      let reader_proxy = reader_proxy_data().unwrap();

      let sdata = to_bytes::<ReaderProxy, LittleEndian>(&reader_proxy).unwrap();
      let reader_proxy2: ReaderProxy =
        PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE).unwrap();
      assert_eq!(reader_proxy, reader_proxy2);
      let sdata2 = to_bytes::<ReaderProxy, LittleEndian>(&reader_proxy2).unwrap();
      assert_eq!(sdata, sdata2);
    }

    #[test]
    fn td_writer_proxy_ser_deser() {
      let writer_proxy = writer_proxy_data().unwrap();

      let sdata = to_bytes::<WriterProxy, LittleEndian>(&writer_proxy).unwrap();
      let writer_proxy2: WriterProxy =
        PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE).unwrap();
      assert_eq!(writer_proxy, writer_proxy2);
      let sdata2 = to_bytes::<WriterProxy, LittleEndian>(&writer_proxy2).unwrap();
      assert_eq!(sdata, sdata2);
    }

    #[test]
    fn td_subscription_builtin_topic_data_ser_deser() {
      let sub_topic_data = subscription_builtin_topic_data().unwrap();

      let sdata = to_bytes::<SubscriptionBuiltinTopicData, LittleEndian>(&sub_topic_data).unwrap();
      let sub_topic_data2: SubscriptionBuiltinTopicData =
        PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE).unwrap();
      assert_eq!(sub_topic_data, sub_topic_data2);
      let sdata2 = to_bytes::<SubscriptionBuiltinTopicData, LittleEndian>(&sub_topic_data2).unwrap();
      assert_eq!(sdata, sdata2);
    }

    #[test]
    fn td_publication_builtin_topic_data_ser_deser() {
      let pub_topic_data = publication_builtin_topic_data().unwrap();

      let sdata = to_bytes::<PublicationBuiltinTopicData, LittleEndian>(&pub_topic_data).unwrap();
      let pub_topic_data2: PublicationBuiltinTopicData =
        PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE).unwrap();
      assert_eq!(pub_topic_data, pub_topic_data2);
      let sdata2 = to_bytes::<PublicationBuiltinTopicData, LittleEndian>(&pub_topic_data2).unwrap();
      assert_eq!(sdata, sdata2);
    }
  */
  #[test]
  fn td_discovered_reader_data_ser_deser() {
    let mut reader_proxy = reader_proxy_data().unwrap();
    let sub_topic_data = subscription_builtin_topic_data().unwrap();
    reader_proxy.remote_reader_guid = sub_topic_data.key;
    let content_filter = content_filter_data().unwrap();

    let drd = DiscoveredReaderData {
      reader_proxy,
      subscription_topic_data: sub_topic_data,
      content_filter: Some(content_filter),
    };

    // serialize
    let sdata = drd
      .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
      .unwrap();
    info!(
      "td_discovered_reader_data_ser_deser serialized data is {:x?}",
      sdata
    );
    // deserialize back
    let drd2: DiscoveredReaderData =
      PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE)
        .unwrap_or_else(|e| {
          println!("DiscoveredReaderData deserialize fail: {e:?}");
          panic!();
        });

    // check objects are equal
    assert_eq!(drd, drd2);
    let sdata2 =
      PlCdrSerializerAdapter::<DiscoveredReaderData, LittleEndian>::to_bytes(&drd2).unwrap();
    assert_eq!(sdata, sdata2);

    let raw_data = Bytes::from_static(&[
      0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x00, 0x00, 0x39, 0xbc, 0xd6, 0xb1, 0x4f, 0xa2, 0x49,
      0x72, 0x81, 0x7d, 0xd4, 0x54, 0x09, 0x01, 0x08, 0x00, 0xa8, 0x3b, 0x56, 0x5f, 0xfa, 0xa6,
      0xa9, 0x75, 0x15, 0x05, 0xdc, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0xc7, 0x00,
      0x00, 0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
      0x43, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x10, 0x00, 0x39, 0xbc, 0xd6,
      0xb1, 0x4f, 0xa2, 0x49, 0x72, 0x81, 0x7d, 0xd4, 0x54, 0x00, 0x00, 0x01, 0xc1, 0x5a, 0x00,
      0x10, 0x00, 0x39, 0xbc, 0xd6, 0xb1, 0x4f, 0xa2, 0x49, 0x72, 0x81, 0x7d, 0xd4, 0x54, 0x4f,
      0xe7, 0xe7, 0x07, 0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xfd, 0x1c, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x8e,
      0xc9, 0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xfd, 0x1c, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0x45, 0x14, 0x2f,
      0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xfd, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xac, 0x11, 0x00, 0x01, 0x30, 0x00, 0x18,
      0x00, 0x01, 0x00, 0x00, 0x00, 0xe9, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0xff, 0x00, 0x01, 0x05, 0x00, 0x0c, 0x00, 0x07,
      0x00, 0x00, 0x00, 0x53, 0x71, 0x75, 0x61, 0x72, 0x65, 0x00, 0x00, 0x07, 0x00, 0x0c, 0x00,
      0x07, 0x00, 0x00, 0x00, 0x53, 0x71, 0x75, 0x61, 0x72, 0x65, 0x00, 0x00, 0x01, 0x00, 0x00,
      0x00,
    ]);

    let msg = Message::read_from_buffer(&raw_data).unwrap();
    info!("{:?}", msg);
  }

  #[test]
  fn td_discovered_writer_data_ser_deser() {
    let mut writer_proxy = writer_proxy_data().unwrap();
    let pub_topic_data = publication_builtin_topic_data().unwrap();
    writer_proxy.remote_writer_guid = pub_topic_data.key;

    let dwd = DiscoveredWriterData {
      last_updated: Instant::now(),
      writer_proxy,
      publication_topic_data: pub_topic_data,
    };

    let sdata = dwd
      .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
      .unwrap();
    let mut dwd2: DiscoveredWriterData =
      PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE).unwrap();
    // last updated is not serialized thus copying value for correct result
    dwd2.last_updated = dwd.last_updated;

    info!(
      "td_discovered_writer_data_ser_deser serialized data is {:x?}",
      sdata
    );

    assert_eq!(dwd, dwd2);
    let sdata2 =
      PlCdrSerializerAdapter::<DiscoveredWriterData, LittleEndian>::to_bytes(&dwd2).unwrap();
    assert_eq!(sdata, sdata2);
  }

  // Do not test ser/deser. This is never seen on the wire out of
  // DiscoveredTopicData #[test]
  // fn td_topic_data_ser_deser() {
  //   let topic_data = topic_data().unwrap();

  //   let sdata = to_bytes::<TopicBuiltinTopicData,
  // LittleEndian>(&topic_data).unwrap();   let topic_data2:
  // TopicBuiltinTopicData =     PlCdrDeserializerAdapter::from_bytes(&sdata,
  // RepresentationIdentifier::PL_CDR_LE).unwrap();   assert_eq!(topic_data,
  // topic_data2);   let sdata2 = to_bytes::<TopicBuiltinTopicData,
  // LittleEndian>(&topic_data2).unwrap();   assert_eq!(sdata, sdata2);
  // }

  #[test]
  fn td_discovered_topic_data_ser_deser() {
    let topic_data = topic_data().unwrap();

    let dtd = DiscoveredTopicData::new(Utc::now(), topic_data);

    let sdata = dtd
      .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
      .unwrap();
    let dtd2: DiscoveredTopicData =
      PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE).unwrap();
    assert_eq!(dtd.topic_data, dtd2.topic_data);
    let sdata2 =
      PlCdrSerializerAdapter::<DiscoveredTopicData, LittleEndian>::to_bytes(&dtd2).unwrap();
    assert_eq!(sdata, sdata2);
  }

  // TODO: somehow get some actual bytes of ParticipantMessageData
  // #[test]
  // fn td_participant_message_data_ser_deser() {
  //   let mut pmd_file = File::open("participant_message_data.bin").unwrap();
  //   let mut buffer: [u8; 1024] = [0; 1024];
  //   let _len = pmd_file.read(&mut buffer).unwrap();

  //   let rpi = CDRDeserializerAdapter::<ParticipantMessageData>::from_bytes(
  //     &buffer,
  //     RepresentationIdentifier::CDR_LE,
  //   )
  //   .unwrap();

  //   let _data2 = to_bytes::<ParticipantMessageData,
  // LittleEndian>(&rpi).unwrap(); }
}
