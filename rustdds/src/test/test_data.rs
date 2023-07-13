pub(crate) fn spdp_participant_data_raw() -> Bytes {
  const DATA: [u8; 204] = [
    // Offset 0x00000000 to 0x00000203
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x09, 0x01, 0x08, 0x00, 0x0e, 0x15, 0xf3, 0x5e, 0x00, 0x28, 0x74, 0xd2,
    0x15, 0x05, 0xa8, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0xc7, 0x00, 0x01, 0x00, 0xc2,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x15, 0x00, 0x04, 0x00,
    0x02, 0x03, 0x00, 0x00, 0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00, 0x50, 0x00, 0x10, 0x00,
    0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1,
    0x32, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf4, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x8e, 0x68, 0x31, 0x00, 0x18, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xf5, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x8e, 0x68, 0x02, 0x00, 0x08, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x58, 0x00, 0x04, 0x00, 0x3f, 0x0c, 0x3f, 0x0c, 0x62, 0x00, 0x18, 0x00,
    0x14, 0x00, 0x00, 0x00, 0x66, 0x61, 0x73, 0x74, 0x72, 0x74, 0x70, 0x73, 0x50, 0x61, 0x72, 0x74,
    0x69, 0x63, 0x69, 0x70, 0x61, 0x6e, 0x74, 0x00, 0x01, 0x00, 0x00, 0x00,
  ];

  Bytes::from_static(&DATA)
}

pub(crate) fn spdp_subscription_data_raw() -> Bytes {
  const DATA: [u8; 248] = [
    // Offset 0x00000000 to 0x00000247
    0x52, 0x54, 0x50, 0x53, 0x02, 0x04, 0x01, 0x03, 0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d, 0x31, 0xa2,
    0x28, 0x20, 0x02, 0x08, 0x09, 0x01, 0x08, 0x00, 0x17, 0x15, 0xf3, 0x5e, 0x35, 0x07, 0x08, 0xc2,
    0x15, 0x05, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xc2,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x05, 0x00, 0x0c, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x53, 0x71, 0x75, 0x61, 0x72, 0x65, 0x00, 0x00, 0x07, 0x00, 0x10, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x53, 0x68, 0x61, 0x70, 0x65, 0x54, 0x79, 0x70, 0x65, 0x00, 0x00, 0x00,
    0x1a, 0x00, 0x0c, 0x00, 0x01, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x7f,
    0x5a, 0x00, 0x10, 0x00, 0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d, 0x31, 0xa2, 0x28, 0x20, 0x02, 0x08,
    0x00, 0x00, 0x00, 0x07, 0x30, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xe9, 0x1c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0xff, 0x00, 0x02,
    0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00, 0xa6, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x8e, 0xc9, 0x2f, 0x00, 0x18, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xa6, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0x45, 0x14, 0x2f, 0x00, 0x18, 0x00, 0x01, 0x00, 0x00, 0x00,
    0xa6, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xac, 0x11, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
  ];
  Bytes::from_static(&DATA)
}

pub(crate) fn spdp_publication_data_raw() -> Bytes {
  const DATA: [u8; 352] = [
    // Offset 0x00000000 to 0x00000351
    0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x0e, 0x01, 0x0c, 0x00, 0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d, 0x31, 0xa2,
    0x28, 0x20, 0x02, 0x08, 0x09, 0x01, 0x08, 0x00, 0x12, 0x15, 0xf3, 0x5e, 0x00, 0xc8, 0xa9, 0xfa,
    0x15, 0x05, 0x0c, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0xc7, 0x00, 0x00, 0x03, 0xc2,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x2f, 0x00, 0x18, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xf5, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x8e, 0x68, 0x50, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x99, 0x06,
    0x78, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1, 0x05, 0x00, 0x0c, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x53, 0x71, 0x75, 0x61, 0x72, 0x65, 0x00, 0x00, 0x07, 0x00, 0x10, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x53, 0x68, 0x61, 0x70, 0x65, 0x54, 0x79, 0x70, 0x65, 0x00, 0x00, 0x00,
    0x70, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x02, 0x5a, 0x00, 0x10, 0x00, 0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x60, 0x00, 0x04, 0x00, 0x5f, 0x01, 0x00, 0x00,
    0x15, 0x00, 0x04, 0x00, 0x02, 0x03, 0x00, 0x00, 0x16, 0x00, 0x04, 0x00, 0x01, 0x0f, 0x00, 0x00,
    0x1d, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f,
    0xff, 0xff, 0xff, 0xff, 0x27, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1b, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff,
    0x1a, 0x00, 0x0c, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9a, 0x99, 0x99, 0x19,
    0x2b, 0x00, 0x08, 0x00, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x04, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x25, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x07, 0x01, 0x1c, 0x00, 0x00, 0x00, 0x03, 0xc7, 0x00, 0x00, 0x03, 0xc2, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
  ];

  Bytes::from_static(&DATA)
}

use std::{net::SocketAddr, time::Duration as StdDuration};

use bytes::Bytes;
use speedy::{Endianness, Writable};
// use serde::Serialize;
// use byteorder::LittleEndian;
use enumflags2::BitFlags;

use crate::{
  dds::{
    adapters::no_key::DeserializerAdapter,
    qos::{
      policy::{
        Deadline, DestinationOrder, Durability, History, LatencyBudget, Lifespan, Liveliness,
        Ownership, Presentation, PresentationAccessScope, Reliability, ResourceLimits,
        TimeBasedFilter,
      },
      QosPolicyBuilder,
    },
  },
  discovery::{
    content_filter_property::ContentFilterProperty,
    sedp_messages::{
      PublicationBuiltinTopicData, ReaderProxy, SubscriptionBuiltinTopicData,
      TopicBuiltinTopicData, WriterProxy,
    },
    spdp_participant_data::SpdpDiscoveredParticipantData,
  },
  messages::{
    header::Header,
    submessages::{
      elements::serialized_payload::SerializedPayload,
      submessages::{Data, SubmessageHeader, SubmessageKind, WriterSubmessage, *},
    },
  },
  rtps::{Message, Submessage, SubmessageBody},
  serialization::pl_cdr_adapters::*,
  structure::{
    duration::Duration,
    guid::{EntityId, EntityKind, GUID},
    locator::Locator,
    sequence_number::SequenceNumber,
  },
  RepresentationIdentifier,
};

pub(crate) fn spdp_participant_msg() -> Message {
  let data = spdp_participant_data_raw();

  Message::read_from_buffer(&data).unwrap()
}

pub(crate) fn spdp_subscription_msg() -> Message {
  let data = spdp_subscription_data_raw();

  Message::read_from_buffer(&data).unwrap()
}

pub(crate) fn spdp_publication_msg() -> Message {
  let data = spdp_publication_data_raw();

  Message::read_from_buffer(&data).unwrap()
}

pub(crate) fn spdp_participant_msg_mod(port: u16) -> Message {
  let mut tdata: Message = spdp_participant_msg();
  let mut data;
  for submsg in &mut tdata.submessages {
    let mut submsglen = submsg.header.content_length;
    match &mut submsg.body {
      SubmessageBody::Writer(v) => match v {
        WriterSubmessage::Data(d, _) => {
          let mut participant_data: SpdpDiscoveredParticipantData =
            PlCdrDeserializerAdapter::<SpdpDiscoveredParticipantData>::from_bytes(
              &d.serialized_payload.as_ref().unwrap().value,
              RepresentationIdentifier::PL_CDR_LE,
            )
            .unwrap();
          participant_data.metatraffic_unicast_locators[0] =
            Locator::from(SocketAddr::new("127.0.0.1".parse().unwrap(), port));
          participant_data.metatraffic_multicast_locators.clear();
          participant_data.default_unicast_locators.clear();
          participant_data.default_multicast_locators.clear();

          let datalen = d.serialized_payload.as_ref().unwrap().value.len() as u16;
          data = participant_data
            .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
            .unwrap();
          // data = Bytes::from(
          //   to_bytes::<SpdpDiscoveredParticipantData,
          // byteorder::LittleEndian>(&participant_data)     .unwrap(),
          // );
          d.serialized_payload.as_mut().unwrap().value = data.clone();
          submsglen =
            submsglen + d.serialized_payload.as_ref().unwrap().value.len() as u16 - datalen;
        }
        _ => continue,
      },
      SubmessageBody::Interpreter(_) => (),
      _ => continue,
    }
    submsg.header.content_length = submsglen;
  }

  tdata
}

pub(crate) fn spdp_participant_data() -> Option<SpdpDiscoveredParticipantData> {
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

          return Some(participant_data);
        }
        _ => continue,
      },
      SubmessageBody::Interpreter(_) => (),
      _ => continue,
    }
  }
  None
}

pub(crate) fn reader_proxy_data() -> Option<ReaderProxy> {
  let reader_proxy = ReaderProxy {
    remote_reader_guid: GUID::dummy_test_guid(EntityKind::READER_NO_KEY_USER_DEFINED),
    expects_inline_qos: false,
    unicast_locator_list: vec![Locator::from(SocketAddr::new(
      "0.0.0.0".parse().unwrap(),
      12345,
    ))],
    multicast_locator_list: vec![Locator::from(SocketAddr::new(
      "0.0.0.0".parse().unwrap(),
      13579,
    ))],
  };

  Some(reader_proxy)
}

pub(crate) fn writer_proxy_data() -> Option<WriterProxy> {
  let writer_proxy = WriterProxy {
    remote_writer_guid: GUID::dummy_test_guid(EntityKind::WRITER_NO_KEY_USER_DEFINED),
    unicast_locator_list: vec![Locator::from(SocketAddr::new(
      "0.0.0.0".parse().unwrap(),
      12345,
    ))],
    multicast_locator_list: vec![Locator::from(SocketAddr::new(
      "0.0.0.0".parse().unwrap(),
      13579,
    ))],
    data_max_size_serialized: Some(24000),
  };

  Some(writer_proxy)
}

pub(crate) fn subscription_builtin_topic_data() -> Option<SubscriptionBuiltinTopicData> {
  let qos = QosPolicyBuilder::new()
    .durability(Durability::TransientLocal)
    .deadline(Deadline(Duration::from_secs(60)))
    .latency_budget(LatencyBudget {
      duration: Duration::from(StdDuration::from_secs(2 * 60)),
    })
    .liveliness(Liveliness::ManualByTopic {
      lease_duration: Duration::from(StdDuration::from_secs(3 * 60)),
    })
    .reliability(Reliability::Reliable {
      max_blocking_time: Duration::from(StdDuration::from_secs(4 * 60)),
    })
    .ownership(Ownership::Exclusive { strength: 234 })
    .destination_order(DestinationOrder::BySourceTimeStamp)
    .time_based_filter(TimeBasedFilter {
      minimum_separation: Duration::from(StdDuration::from_secs(5 * 60)),
    })
    .presentation(Presentation {
      access_scope: PresentationAccessScope::Topic,
      coherent_access: false,
      ordered_access: true,
    })
    .lifespan(Lifespan {
      duration: Duration::from(StdDuration::from_secs(6 * 60)),
    })
    .build();

  let sub_topic_data = SubscriptionBuiltinTopicData::new(
    GUID::dummy_test_guid(EntityKind::WRITER_NO_KEY_USER_DEFINED),
    None,
    "some topic name".to_string(),
    "RandomData".to_string(),
    &qos,
    None,
  );

  Some(sub_topic_data)
}

pub(crate) fn publication_builtin_topic_data() -> Option<PublicationBuiltinTopicData> {
  let pub_topic_data = PublicationBuiltinTopicData {
    key: GUID::dummy_test_guid(EntityKind::WRITER_WITH_KEY_BUILT_IN),
    participant_key: Some(GUID::dummy_test_guid(EntityKind::PARTICIPANT_BUILT_IN)),
    topic_name: "rand topic name".to_string(),
    type_name: "RandomData".to_string(),
    durability: Some(Durability::Volatile),
    deadline: Some(Deadline(Duration::from_secs(30))),
    latency_budget: Some(LatencyBudget {
      duration: Duration::from(StdDuration::from_secs(2 * 30)),
    }),
    liveliness: Some(Liveliness::ManualByTopic {
      lease_duration: Duration::from(StdDuration::from_secs(3 * 30)),
    }),
    reliability: Some(Reliability::BestEffort),
    lifespan: Some(Lifespan {
      duration: Duration::from(StdDuration::from_secs(6 * 30)),
    }),
    time_based_filter: Some(TimeBasedFilter {
      minimum_separation: Duration::from(StdDuration::from_secs(5 * 30)),
    }),
    ownership: Some(Ownership::Shared),
    destination_order: Some(DestinationOrder::ByReceptionTimestamp),
    presentation: Some(Presentation {
      access_scope: PresentationAccessScope::Instance,
      coherent_access: true,
      ordered_access: false,
    }),
    related_datareader_key: None,
    service_instance_name: None,
    topic_aliases: None,
    security_info: None,
  };

  Some(pub_topic_data)
}

pub(crate) fn topic_data() -> Option<TopicBuiltinTopicData> {
  let topic_data = TopicBuiltinTopicData {
    key: Some(GUID::dummy_test_guid(EntityKind::UNKNOWN_BUILT_IN)),
    name: "SomeTopicName".to_string(),
    type_name: "RandomData".to_string(),
    durability: Some(Durability::Persistent),
    deadline: Some(Deadline(Duration::from_secs(45))),
    latency_budget: Some(LatencyBudget {
      duration: Duration::from(StdDuration::from_secs(2 * 45)),
    }),
    liveliness: Some(Liveliness::ManualByTopic {
      lease_duration: Duration::from(StdDuration::from_secs(3 * 45)),
    }),
    reliability: Some(Reliability::BestEffort),
    lifespan: Some(Lifespan {
      duration: Duration::from(StdDuration::from_secs(6 * 45)),
    }),
    destination_order: Some(DestinationOrder::ByReceptionTimestamp),
    presentation: Some(Presentation {
      access_scope: PresentationAccessScope::Group,
      coherent_access: true,
      ordered_access: true,
    }),
    history: Some(History::KeepLast { depth: 25 }),
    resource_limits: Some(ResourceLimits {
      max_samples: 5,
      max_instances: 10,
      max_samples_per_instance: 15,
    }),
    ownership: Some(Ownership::Exclusive { strength: 432 }),
  };

  Some(topic_data)
}

pub(crate) fn content_filter_data() -> Option<ContentFilterProperty> {
  let content_filter = ContentFilterProperty {
    content_filtered_topic_name: "tn".to_string(),
    related_topic_name: "rtn".to_string(),
    filter_class_name: "fcn".to_string(),
    filter_expression: "fexp".to_string(),
    expression_parameters: vec!["asdf".to_string(), "fdsas".to_string()],
  };

  Some(content_filter)
}

#[allow(dead_code)]
pub(crate) fn create_rtps_data_message<D: PlCdrSerialize>(
  data: D,
  reader_id: EntityId,
  writer_id: EntityId,
) -> Message {
  // let tdata = Bytes::from(to_bytes::<D, LittleEndian>(&data).unwrap());
  let tdata = data
    .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
    .unwrap();

  let mut rtps_message = Message::default();
  let guid = GUID::dummy_test_guid(EntityKind::UNKNOWN_BUILT_IN);
  let rtps_message_header = Header::new(guid.prefix);
  rtps_message.set_header(rtps_message_header);

  let serialized_payload = SerializedPayload {
    representation_identifier: RepresentationIdentifier::PL_CDR_LE,
    representation_options: [0; 2],
    value: tdata,
  };
  let data_message = Data {
    reader_id,
    writer_id,
    writer_sn: SequenceNumber::default(),
    inline_qos: None,
    serialized_payload: Some(serialized_payload),
  };

  let data_size = data_message
    .write_to_vec_with_ctx(Endianness::LittleEndian)
    .unwrap()
    .len();

  let sub_flags = BitFlags::<DATA_Flags>::from_endianness(Endianness::LittleEndian)
    | BitFlags::<DATA_Flags>::from_flag(DATA_Flags::Data);

  let submessage_header: SubmessageHeader = SubmessageHeader {
    kind: SubmessageKind::DATA,
    flags: sub_flags.bits(),
    content_length: data_size as u16,
  };

  let submessage: Submessage = Submessage {
    header: submessage_header,
    body: SubmessageBody::Writer(WriterSubmessage::Data(data_message, sub_flags)),
  };
  rtps_message.add_submessage(submessage);

  rtps_message
}

pub(crate) fn create_cdr_pl_rtps_data_message<D: PlCdrSerialize>(
  data: D,
  reader_id: EntityId,
  writer_id: EntityId,
) -> Message {
  let tdata = data
    .to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
    .unwrap();

  let mut rtps_message = Message::default();
  let guid = GUID::dummy_test_guid(EntityKind::UNKNOWN_BUILT_IN);
  let rtps_message_header = Header::new(guid.prefix);
  rtps_message.set_header(rtps_message_header);

  let serialized_payload = SerializedPayload {
    representation_identifier: RepresentationIdentifier::PL_CDR_LE,
    representation_options: [0; 2],
    value: tdata,
  };
  let data_message = Data {
    reader_id,
    writer_id,
    writer_sn: SequenceNumber::default(),
    inline_qos: None,
    serialized_payload: Some(serialized_payload),
  };

  let data_size = data_message
    .write_to_vec_with_ctx(Endianness::LittleEndian)
    .unwrap()
    .len();

  let sub_flags = BitFlags::<DATA_Flags>::from_endianness(Endianness::LittleEndian)
    | BitFlags::<DATA_Flags>::from_flag(DATA_Flags::Data);

  let submessage_header: SubmessageHeader = SubmessageHeader {
    kind: SubmessageKind::DATA,
    flags: sub_flags.bits(),
    content_length: data_size as u16,
  };

  let submessage: Submessage = Submessage {
    header: submessage_header,
    body: SubmessageBody::Writer(WriterSubmessage::Data(data_message, sub_flags)),
  };
  rtps_message.add_submessage(submessage);

  rtps_message
}
