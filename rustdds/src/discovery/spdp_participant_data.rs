use std::collections::HashMap;

use serde::{Deserialize, Deserializer, Serialize, Serializer};
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use mio_06::Token;
use speedy::{Readable, Writable};
use chrono::Utc;
use bytes::Bytes;
use cdr_encoding_size::CdrEncodingSize;

use crate::{
  dds::{participant::DomainParticipant, qos, qos::QosPolicies},
  messages::{
    protocol_version::ProtocolVersion,
    submessages::elements::{parameter::Parameter, parameter_list::ParameterList},
    vendor_id::VendorId,
  },
  network::constant::*,
  rtps::{rtps_reader_proxy::RtpsReaderProxy, rtps_writer_proxy::RtpsWriterProxy},
  security::{
    access_control::PermissionsToken, authentication::IdentityToken, ParticipantSecurityInfo,
  },
  serialization::{pl_cdr_adapters::*, speedy_pl_cdr_helpers::*},
  structure::{
    duration::Duration,
    entity::RTPSEntity,
    guid::{EntityId, GUID},
    locator,
    locator::Locator,
    parameter_id::ParameterId,
  },
  Key, Keyed, RepresentationIdentifier,
};
use super::builtin_endpoint::{BuiltinEndpointQos, BuiltinEndpointSet};

// This type is used by Discovery to communicate the presence and properties
// of DomainParticipants. It is sent over topic "DCPSParticipant".
// The type is called "ParticipantBuiltinTopicData" in DDS-Security Spec, e.g.
// Section 7.4.1.4.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpdpDiscoveredParticipantData {
  pub updated_time: chrono::DateTime<Utc>,
  pub protocol_version: ProtocolVersion,
  pub vendor_id: VendorId,
  pub expects_inline_qos: bool,
  pub participant_guid: GUID,
  pub metatraffic_unicast_locators: Vec<Locator>,
  pub metatraffic_multicast_locators: Vec<Locator>,
  pub default_unicast_locators: Vec<Locator>,
  pub default_multicast_locators: Vec<Locator>,
  pub available_builtin_endpoints: BuiltinEndpointSet,
  pub lease_duration: Option<Duration>,
  pub manual_liveliness_count: i32,
  pub builtin_endpoint_qos: Option<BuiltinEndpointQos>,
  pub entity_name: Option<String>,

  // security
  pub identity_token: Option<IdentityToken>,
  pub permissions_token: Option<PermissionsToken>,
  pub property: Option<qos::policy::Property>,
  pub security_info: Option<ParticipantSecurityInfo>,
}

impl SpdpDiscoveredParticipantData {
  pub(crate) fn as_reader_proxy(
    &self,
    is_metatraffic: bool,
    entity_id: Option<EntityId>,
  ) -> RtpsReaderProxy {
    let remote_reader_guid = GUID::new_with_prefix_and_id(
      self.participant_guid.prefix,
      match entity_id {
        Some(id) => id,
        None => EntityId::SPDP_BUILTIN_PARTICIPANT_READER,
      },
    );

    let mut proxy = RtpsReaderProxy::new(
      remote_reader_guid,
      QosPolicies::qos_none(), // TODO: What is the correct QoS value here?
    );
    proxy.expects_in_line_qos = self.expects_inline_qos;

    if !is_metatraffic {
      proxy.multicast_locator_list = self.default_multicast_locators.clone();
      proxy.unicast_locator_list = self.default_unicast_locators.clone();
    } else {
      proxy.multicast_locator_list = self.metatraffic_multicast_locators.clone();
      proxy.unicast_locator_list = self.metatraffic_unicast_locators.clone();
    }

    proxy
  }

  pub(crate) fn as_writer_proxy(
    &self,
    is_metatraffic: bool,
    entity_id: Option<EntityId>,
  ) -> RtpsWriterProxy {
    let remote_writer_guid = GUID::new_with_prefix_and_id(
      self.participant_guid.prefix,
      match entity_id {
        Some(id) => id,
        None => EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER,
      },
    );

    let mut proxy = RtpsWriterProxy::new(
      remote_writer_guid,
      Vec::new(),
      Vec::new(),
      EntityId::UNKNOWN,
    );

    if is_metatraffic {
      // TODO: possible multicast addresses
      proxy.unicast_locator_list = self.metatraffic_unicast_locators.clone();
    } else {
      // TODO: possible multicast addresses
      proxy.unicast_locator_list = self.default_unicast_locators.clone();
    }

    proxy
  }

  pub fn from_local_participant(
    participant: &DomainParticipant,
    self_locators: &HashMap<Token, Vec<Locator>>,
    lease_duration: Duration,
  ) -> Self {
    let metatraffic_multicast_locators = self_locators
      .get(&DISCOVERY_MUL_LISTENER_TOKEN)
      .cloned()
      .unwrap_or_default();

    let metatraffic_unicast_locators = self_locators
      .get(&DISCOVERY_LISTENER_TOKEN)
      .cloned()
      .unwrap_or_default();

    let default_multicast_locators = self_locators
      .get(&USER_TRAFFIC_MUL_LISTENER_TOKEN)
      .cloned()
      .unwrap_or_default();

    let default_unicast_locators = self_locators
      .get(&USER_TRAFFIC_LISTENER_TOKEN)
      .cloned()
      .unwrap_or_default();

    let builtin_endpoints = BuiltinEndpointSet::PARTICIPANT_ANNOUNCER
      | BuiltinEndpointSet::PARTICIPANT_DETECTOR
      | BuiltinEndpointSet::PUBLICATIONS_ANNOUNCER
      | BuiltinEndpointSet::PUBLICATIONS_DETECTOR
      | BuiltinEndpointSet::SUBSCRIPTIONS_ANNOUNCER
      | BuiltinEndpointSet::SUBSCRIPTIONS_DETECTOR
      | BuiltinEndpointSet::PARTICIPANT_MESSAGE_DATA_WRITER
      | BuiltinEndpointSet::PARTICIPANT_MESSAGE_DATA_READER
      | BuiltinEndpointSet::TOPICS_ANNOUNCER
      | BuiltinEndpointSet::TOPICS_DETECTOR;

    Self {
      updated_time: Utc::now(),
      protocol_version: ProtocolVersion::PROTOCOLVERSION_2_3,
      vendor_id: VendorId::THIS_IMPLEMENTATION,
      expects_inline_qos: false,
      participant_guid: participant.guid(),
      metatraffic_unicast_locators,
      metatraffic_multicast_locators,
      default_unicast_locators,
      default_multicast_locators,
      available_builtin_endpoints: BuiltinEndpointSet::from_u32(builtin_endpoints),
      lease_duration: Some(lease_duration),
      manual_liveliness_count: 0,
      builtin_endpoint_qos: None,
      entity_name: None,

      // DDS Security
      identity_token: None,    // TODO: Generate(?) one
      permissions_token: None, // TODO
      property: None,
      security_info: None,
    }
  }
}

// fake implementations. Real serialization is done using PlCdrSerialize
impl Serialize for SpdpDiscoveredParticipantData {
  fn serialize<S>(&self, _serializer: S) -> std::result::Result<S::Ok, S::Error>
  where
    S: Serializer,
  {
    unimplemented!()
  }
}
impl<'de> Deserialize<'de> for SpdpDiscoveredParticipantData {
  fn deserialize<D>(
    _deserializer: D,
  ) -> std::result::Result<SpdpDiscoveredParticipantData, D::Error>
  where
    D: Deserializer<'de>,
  {
    unimplemented!()
  }
}

impl PlCdrDeserialize for SpdpDiscoveredParticipantData {
  fn from_pl_cdr_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<Self, PlCdrDeserializeError> {
    let ctx = pl_cdr_rep_id_to_speedy_d(encoding)?;
    let pl = ParameterList::read_from_buffer_with_ctx(ctx, input_bytes)?;
    let pl_map = pl.to_map();
    let protocol_version: ProtocolVersion = get_first_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PROTOCOL_VERSION,
      "Protocol Version",
    )?;
    let vendor_id: VendorId =
      get_first_from_pl_map(&pl_map, ctx, ParameterId::PID_VENDOR_ID, "Vendor Id")?;
    let expects_inline_qos : bool = // This one has default value false
      get_option_from_pl_map(&pl_map, ctx, ParameterId::PID_EXPECTS_INLINE_QOS, "Expects inline Qos")?
      .unwrap_or(false);
    let participant_guid: GUID = get_first_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PARTICIPANT_GUID,
      "Participant GUID",
    )?;

    let metatraffic_unicast_locators: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR,
      "Metatraffic unicast locators",
    )?;
    let metatraffic_multicast_locators: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_METATRAFFIC_MULTICAST_LOCATOR,
      "Metatraffic multicast locators",
    )?;
    let default_unicast_locators: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_DEFAULT_UNICAST_LOCATOR,
      "Default unicast locators",
    )?;
    let default_multicast_locators: Vec<Locator> = get_all_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_DEFAULT_MULTICAST_LOCATOR,
      "Default multicast locators",
    )?;

    let lease_duration: Option<Duration> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PARTICIPANT_LEASE_DURATION,
      "participant lease duration",
    )?;
    let manual_liveliness_count : i32 =  // Default value is 0. TODO: What is the meaning of this?
      get_option_from_pl_map(&pl_map, ctx, ParameterId::PID_PARTICIPANT_MANUAL_LIVELINESS_COUNT, "Manual liveness count")?
      .unwrap_or(0);
    let available_builtin_endpoints: BuiltinEndpointSet = get_first_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_BUILTIN_ENDPOINT_SET,
      "Available builtin endpoints",
    )?;
    let builtin_endpoint_qos: Option<BuiltinEndpointQos> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_BUILTIN_ENDPOINT_QOS,
      "Builtin Endpoint Qos",
    )?;

    let entity_name : Option<String> = // Note the serialized type is StringWithNul
      get_option_from_pl_map::< _ , StringWithNul>(&pl_map, ctx, ParameterId::PID_ENTITY_NAME, "entity name")?
      .map( String::from );

    // DDS security
    let identity_token: Option<IdentityToken> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_IDENTITY_TOKEN,
      "identity token",
    )?;
    let permissions_token: Option<PermissionsToken> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PERMISSIONS_TOKEN,
      "permissions token",
    )?;
    let property: Option<qos::policy::Property> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PROPERTY_LIST,
      "property list",
    )?;
    let security_info: Option<ParticipantSecurityInfo> = get_option_from_pl_map(
      &pl_map,
      ctx,
      ParameterId::PID_PARTICIPANT_SECURITY_INFO,
      "participant security info",
    )?;

    Ok(Self {
      updated_time: Utc::now(),
      protocol_version,
      vendor_id,
      expects_inline_qos,
      participant_guid,
      metatraffic_unicast_locators,
      metatraffic_multicast_locators,
      default_unicast_locators,
      default_multicast_locators,
      available_builtin_endpoints,
      lease_duration,
      manual_liveliness_count,
      builtin_endpoint_qos,
      entity_name,

      identity_token,
      permissions_token,
      property,
      security_info,
    })
  }
}

impl PlCdrSerialize for SpdpDiscoveredParticipantData {
  fn to_pl_cdr_bytes(
    &self,
    encoding: RepresentationIdentifier,
  ) -> Result<Bytes, PlCdrSerializeError> {
    // This "unnecessary" binding is to trigger a warning if we forget to
    // serialize any fields.
    let Self {
      updated_time: _, // except this field. It is not serialized.
      protocol_version,
      vendor_id,
      expects_inline_qos,
      participant_guid,
      metatraffic_unicast_locators,
      metatraffic_multicast_locators,
      default_unicast_locators,
      default_multicast_locators,
      available_builtin_endpoints,
      lease_duration,
      manual_liveliness_count,
      builtin_endpoint_qos,
      entity_name,

      // DDS security
      identity_token,    // TODO
      permissions_token, // TODO
      property,          // TODO
      security_info,     // TODO
    } = self;

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
    macro_rules! emit_option {
      ($pid:ident, $member:expr, $type:ty) => {
        if let Some(m) = $member {
          emit!($pid, m, $type)
        }
      };
    }

    emit!(PID_PROTOCOL_VERSION, protocol_version, ProtocolVersion);
    emit!(PID_VENDOR_ID, vendor_id, VendorId);
    emit!(PID_EXPECTS_INLINE_QOS, expects_inline_qos, bool);
    emit!(PID_PARTICIPANT_GUID, participant_guid, GUID);
    for loc in metatraffic_unicast_locators {
      emit!(
        PID_METATRAFFIC_UNICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }
    for loc in metatraffic_multicast_locators {
      emit!(
        PID_METATRAFFIC_MULTICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }
    for loc in default_unicast_locators {
      emit!(
        PID_DEFAULT_UNICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }
    for loc in default_multicast_locators {
      emit!(
        PID_DEFAULT_MULTICAST_LOCATOR,
        &locator::repr::Locator::from(*loc),
        locator::repr::Locator
      );
    }
    emit!(
      PID_BUILTIN_ENDPOINT_SET,
      available_builtin_endpoints,
      BuiltinEndpointSet
    );
    emit_option!(PID_PARTICIPANT_LEASE_DURATION, lease_duration, Duration);
    emit!(
      PID_PARTICIPANT_MANUAL_LIVELINESS_COUNT,
      manual_liveliness_count,
      i32
    );
    emit_option!(
      PID_BUILTIN_ENDPOINT_QOS,
      builtin_endpoint_qos,
      BuiltinEndpointQos
    );

    // Here we need to serialize as StringWithNul, as String is Speedy built-in,
    // and does not follow CDR encoding.
    let entity_name_n: Option<StringWithNul> = entity_name.clone().map(|e| e.into());
    emit_option!(PID_ENTITY_NAME, &entity_name_n, StringWithNul);

    // DDS security
    emit_option!(PID_IDENTITY_TOKEN, identity_token, IdentityToken);
    emit_option!(PID_PERMISSIONS_TOKEN, permissions_token, PermissionsToken);
    emit_option!(PID_PROPERTY_LIST, property, qos::policy::Property);
    emit_option!(
      PID_PARTICIPANT_SECURITY_INFO,
      security_info,
      ParticipantSecurityInfo
    );

    let bytes = pl.serialize_to_bytes(ctx)?;

    Ok(bytes)
  }
}

// We need a wrapper to distinguish between Participant and Endpoint GUIDs.
#[allow(non_camel_case_types)]
#[derive(
  PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy, Serialize, Deserialize, CdrEncodingSize, Hash,
)]
pub struct Participant_GUID(pub GUID);

impl Key for Participant_GUID {}

impl Keyed for SpdpDiscoveredParticipantData {
  type K = Participant_GUID;
  fn key(&self) -> Self::K {
    Participant_GUID(self.participant_guid)
  }
}

impl PlCdrDeserialize for Participant_GUID {
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
      ParameterId::PID_PARTICIPANT_GUID,
      "Participant GUID",
    )?;

    Ok(Participant_GUID(guid))
  }
}

impl PlCdrSerialize for Participant_GUID {
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
    emit!(PID_PARTICIPANT_GUID, &self.0, GUID);
    let bytes = pl.serialize_to_bytes(ctx)?;
    Ok(bytes)
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::{
    dds::adapters::no_key::DeserializerAdapter,
    messages::submessages::submessages::WriterSubmessage,
    rtps::{submessage::*, Message},
    test::test_data::*,
    RepresentationIdentifier,
  };

  #[test]
  fn pdata_deserialize_serialize() {
    let data = spdp_participant_data_raw();

    let rtpsmsg = Message::read_from_buffer(&data).unwrap();
    let submsgs = rtpsmsg.submessages();

    for submsg in &submsgs {
      match &submsg.body {
        SubmessageBody::Writer(v) => match v {
          WriterSubmessage::Data(d, _) => {
            let participant_data: SpdpDiscoveredParticipantData =
              PlCdrDeserializerAdapter::from_bytes(
                &d.serialized_payload.as_ref().unwrap().value,
                RepresentationIdentifier::PL_CDR_LE,
              )
              .unwrap();
            let sdata = participant_data
              .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
              .unwrap();
            eprintln!("message data = {:?}", &data);
            eprintln!(
              "payload    = {:?}",
              &d.serialized_payload.as_ref().unwrap().value.to_vec()
            );
            eprintln!("deserialized  = {:?}", &participant_data);
            eprintln!("serialized = {:?}", &sdata);
            // order cannot be known at this point
            // assert_eq!(
            //  sdata.len(),
            //  d.serialized_payload.as_ref().unwrap().value.len()
            //);

            let mut participant_data_2: SpdpDiscoveredParticipantData =
              PlCdrDeserializerAdapter::from_bytes(&sdata, RepresentationIdentifier::PL_CDR_LE)
                .unwrap();
            // force timestamps to be the same, as these are not serialized/deserialized,
            // but stamped during deserialization
            participant_data_2.updated_time = participant_data.updated_time;

            eprintln!("again deserialized = {:?}", &participant_data_2);
            let _sdata_2 = participant_data
              .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
              .unwrap();
            // now the order of bytes should be the same
            assert_eq!(&participant_data_2, &participant_data);
          }

          _ => continue,
        },
        SubmessageBody::Interpreter(_) => (),
        _ => continue,
      }
    }
  }

  #[test]
  fn deserialize_evil_spdp_fuzz() {
    use hex_literal::hex;
    let data = Bytes::copy_from_slice(&hex!(
      "
    52 54 50 53
    02 02 ff ff 01 0f 45 d2 b3 f5 58 b9 01 00 00 00
    15 07 1e 00 00 00 10 00 00 00 00 00 00 01 00 c2
    00 00 00 00 00 00 00 00 01 00 00 00 00 02 44 d5
    cf 7a
    "
    ));

    let rtpsmsg = Message::read_from_buffer(&data).unwrap();
    let submsgs = rtpsmsg.submessages();

    for submsg in &submsgs {
      match &submsg.body {
        SubmessageBody::Writer(v) => match v {
          WriterSubmessage::Data(d, _) => {
            let participant_data: Result<SpdpDiscoveredParticipantData, PlCdrDeserializeError> =
              PlCdrDeserializerAdapter::from_bytes(
                &d.serialized_payload.as_ref().unwrap().value,
                RepresentationIdentifier::PL_CDR_LE,
              );
            eprintln!("message data = {:?}", &data);
            eprintln!(
              "payload    = {:?}",
              &d.serialized_payload.as_ref().unwrap().value.to_vec()
            );
            eprintln!("deserialized  = {:?}", &participant_data);
          }

          _ => continue,
        },
        SubmessageBody::Interpreter(_) => (),
        _ => continue,
      }
    }
  }
  #[test]
  fn deserialize_evil_spdp_fuzz_2() {
    // https://github.com/jhelovuo/RustDDS/issues/279
    use hex_literal::hex;
    let data = Bytes::copy_from_slice(&hex!(
      "
      52 54 50 53
      02 02 ff ff 01 0f 45 d2 b3 f5 58 b9 01 00 00 00
      15 05 19 00 00 00 10 00 00 00 00 00 00 01 00 c2
      00 00 00 00 02 00 00 00 00 03 90 fe c7
    "
    ));

    let rtpsmsg = Message::read_from_buffer(&data).unwrap();
    let submsgs = rtpsmsg.submessages();

    for submsg in &submsgs {
      match &submsg.body {
        SubmessageBody::Writer(v) => match v {
          WriterSubmessage::Data(d, _) => {
            let participant_data: Result<SpdpDiscoveredParticipantData, PlCdrDeserializeError> =
              PlCdrDeserializerAdapter::from_bytes(
                &d.serialized_payload.as_ref().unwrap().value,
                RepresentationIdentifier::PL_CDR_LE,
              );
            eprintln!("message data = {:?}", &data);
            eprintln!(
              "payload    = {:?}",
              &d.serialized_payload.as_ref().unwrap().value.to_vec()
            );
            eprintln!("deserialized  = {:?}", &participant_data);
          }

          _ => continue,
        },
        SubmessageBody::Interpreter(_) => (),
        _ => continue,
      }
    }
  }

  #[test]
  fn deserialize_evil_spdp_fuzz_3() {
    // https://github.com/jhelovuo/RustDDS/issues/281
    use hex_literal::hex;
    let data = Bytes::copy_from_slice(&hex!(
      "
      52 54 50 53
      02 02 ff ff 01 0f 45 d2 b3 f5 58 b9 01 00 00 00
      15 05 00 00 00 00 32 00 00 00 00 00 00 01 00 c2
      00 00 00 00 02 00 00 00 00 03 00 00 77 00 04 00
      00 00 00 00
    "
    ));

    let rtpsmsg = match Message::read_from_buffer(&data) {
      Ok(m) => m,
      Err(e) => {
        eprintln!("{e}");
        return;
      }
    };
    let submsgs = rtpsmsg.submessages();

    for submsg in &submsgs {
      match &submsg.body {
        SubmessageBody::Writer(v) => match v {
          WriterSubmessage::Data(d, _) => {
            let participant_data: Result<SpdpDiscoveredParticipantData, PlCdrDeserializeError> =
              PlCdrDeserializerAdapter::from_bytes(
                &d.serialized_payload.as_ref().unwrap().value,
                RepresentationIdentifier::PL_CDR_LE,
              );
            eprintln!("message data = {:?}", &data);
            eprintln!(
              "payload    = {:?}",
              &d.serialized_payload.as_ref().unwrap().value.to_vec()
            );
            eprintln!("deserialized  = {:?}", &participant_data);
          }

          _ => continue,
        },
        SubmessageBody::Interpreter(_) => (),
        _ => continue,
      }
    }
  }
}
