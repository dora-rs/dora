use std::collections::{btree_map::Entry, BTreeMap};

use mio_extras::{channel as mio_channel, channel::TrySendError};
use log::{debug, info, trace, warn};
use bytes::Bytes;

use crate::{
  messages::{
    protocol_version::ProtocolVersion,
    submessages::submessages::{WriterSubmessage, *},
    vendor_id::VendorId,
  },
  rtps::{reader::Reader, Message, SubmessageBody},
  structure::{
    entity::RTPSEntity,
    guid::{EntityId, GuidPrefix, GUID},
    locator::Locator,
    time::Timestamp,
  },
};
#[cfg(test)]
use crate::dds::ddsdata::DDSData;
#[cfg(test)]
use crate::structure::sequence_number::SequenceNumber;

const RTPS_MESSAGE_HEADER_SIZE: usize = 20;

/// [`MessageReceiver`] is the submessage sequence interpreter described in
/// RTPS spec v2.3 Section 8.3.4 "The RTPS Message Receiver".
/// It calls the message/submessage deserializers to parse the sequence of
/// submessages. Then it processes the instructions in the Interpreter
/// SUbmessages and forwards data in Entity Submessages to the appropriate
/// Entities. (See RTPS spec Section 8.3.7)

pub(crate) struct MessageReceiver {
  pub available_readers: BTreeMap<EntityId, Reader>,
  // GuidPrefix sent in this channel needs to be RTPSMessage source_guid_prefix. Writer needs this
  // to locate RTPSReaderProxy if negative acknack.
  acknack_sender: mio_channel::SyncSender<(GuidPrefix, AckSubmessage)>,
  // We send notification of remote DomainParticipant liveness to Discovery to
  // bypass Reader, DDSCache, DatasampleCache, and DataReader, because these will drop
  // repeated messages with duplicate SequenceNumbers, but Discovery needs to see them.
  spdp_liveness_sender: mio_channel::SyncSender<GuidPrefix>,

  own_guid_prefix: GuidPrefix,
  pub source_version: ProtocolVersion,
  pub source_vendor_id: VendorId,
  pub source_guid_prefix: GuidPrefix,
  pub dest_guid_prefix: GuidPrefix,
  pub unicast_reply_locator_list: Vec<Locator>,
  pub multicast_reply_locator_list: Vec<Locator>,
  pub source_timestamp: Option<Timestamp>,

  pos: usize,
  pub submessage_count: usize,
}

impl MessageReceiver {
  pub fn new(
    participant_guid_prefix: GuidPrefix,
    acknack_sender: mio_channel::SyncSender<(GuidPrefix, AckSubmessage)>,
    spdp_liveness_sender: mio_channel::SyncSender<GuidPrefix>,
  ) -> Self {
    Self {
      available_readers: BTreeMap::new(),
      acknack_sender,
      spdp_liveness_sender,
      own_guid_prefix: participant_guid_prefix,

      source_version: ProtocolVersion::THIS_IMPLEMENTATION,
      source_vendor_id: VendorId::VENDOR_UNKNOWN,
      source_guid_prefix: GuidPrefix::UNKNOWN,
      dest_guid_prefix: GuidPrefix::UNKNOWN,
      unicast_reply_locator_list: vec![Locator::Invalid],
      multicast_reply_locator_list: vec![Locator::Invalid],
      source_timestamp: None,

      pos: 0,
      submessage_count: 0,
    }
  }

  pub fn reset(&mut self) {
    self.source_version = ProtocolVersion::THIS_IMPLEMENTATION;
    self.source_vendor_id = VendorId::VENDOR_UNKNOWN;
    self.source_guid_prefix = GuidPrefix::UNKNOWN;
    self.dest_guid_prefix = GuidPrefix::UNKNOWN;
    self.unicast_reply_locator_list.clear();
    self.multicast_reply_locator_list.clear();
    self.source_timestamp = None;

    self.pos = 0;
    self.submessage_count = 0;
  }

  fn give_message_receiver_info(&self) -> MessageReceiverState {
    MessageReceiverState {
      // own_guid_prefix: self.own_guid_prefix,
      source_guid_prefix: self.source_guid_prefix,
      unicast_reply_locator_list: self.unicast_reply_locator_list.clone(),
      multicast_reply_locator_list: self.multicast_reply_locator_list.clone(),
      source_timestamp: self.source_timestamp,
    }
  }

  pub fn add_reader(&mut self, new_reader: Reader) {
    let eid = new_reader.guid().entity_id;
    match self.available_readers.entry(eid) {
      Entry::Occupied(_) => warn!("Already have Reader {:?} - not adding.", eid),
      Entry::Vacant(e) => {
        e.insert(new_reader);
      }
    }
  }

  pub fn remove_reader(&mut self, old_reader_guid: GUID) -> Option<Reader> {
    self.available_readers.remove(&old_reader_guid.entity_id)
  }

  pub fn reader_mut(&mut self, reader_id: EntityId) -> Option<&mut Reader> {
    self.available_readers.get_mut(&reader_id)
  }

  // use for test and debugging only
  #[cfg(test)]
  fn get_reader_and_history_cache_change(
    &self,
    reader_id: EntityId,
    sequence_number: SequenceNumber,
  ) -> Option<DDSData> {
    Some(
      self
        .available_readers
        .get(&reader_id)
        .unwrap()
        .history_cache_change_data(sequence_number)
        .unwrap(),
    )
  }

  #[cfg(test)]
  fn get_reader_history_cache_start_and_end_seq_num(
    &self,
    reader_id: EntityId,
  ) -> Vec<SequenceNumber> {
    self
      .available_readers
      .get(&reader_id)
      .unwrap()
      .history_cache_sequence_start_and_end_numbers()
  }

  pub fn handle_received_packet(&mut self, msg_bytes: &Bytes) {
    // Check for RTPS ping message. At least RTI implementation sends these.
    // What should we do with them? The spec does not say.
    if msg_bytes.len() < RTPS_MESSAGE_HEADER_SIZE {
      if msg_bytes.len() >= 16
        && msg_bytes[0..4] == b"RTPS"[..]
        && msg_bytes[9..16] == b"DDSPING"[..]
      {
        // TODO: Add some sensible ping message handling here.
        info!("Received RTPS PING. Do not know how to respond.");
        debug!("Data was {:?}", &msg_bytes);
      } else {
        warn!("Message is shorter than header. Cannot deserialize.");
        debug!("Data was {:?}", &msg_bytes);
      }
      return;
    }

    // call Speedy reader
    // Bytes .clone() is cheap, so no worries
    let rtps_message = match Message::read_from_buffer(msg_bytes) {
      Ok(m) => m,
      Err(speedy_err) => {
        warn!("RTPS deserialize error {:?}", speedy_err);
        debug!("Data was {:?}", msg_bytes);
        return;
      }
    };

    // And process message
    self.handle_parsed_message(rtps_message);
  }

  // This is also called directly from dp_event_loop in case of loopback messages.
  pub fn handle_parsed_message(&mut self, rtps_message: Message) {
    self.reset();
    self.dest_guid_prefix = self.own_guid_prefix;
    self.source_guid_prefix = rtps_message.header.guid_prefix;

    for submessage in rtps_message.submessages {
      match submessage.body {
        SubmessageBody::Interpreter(m) => self.handle_interpreter_submessage(m),
        SubmessageBody::Writer(m) => self.handle_writer_submessage(m),
        SubmessageBody::Reader(m) => self.handle_reader_submessage(m),
        SubmessageBody::Security(m) => self.handle_security_submessage(m),
      }
      self.submessage_count += 1;
    } // submessage loop
  }

  fn handle_writer_submessage(&mut self, submessage: WriterSubmessage) {
    if self.dest_guid_prefix != self.own_guid_prefix && self.dest_guid_prefix != GuidPrefix::UNKNOWN
    {
      debug!("Message is not for this participant. Dropping. dest_guid_prefix={:?} participant guid={:?}", 
        self.dest_guid_prefix, self.own_guid_prefix);
      return;
    }

    let mr_state = self.give_message_receiver_info();
    match submessage {
      WriterSubmessage::Data(data, data_flags) => {
        let writer_entity_id = data.writer_id;
        let source_guid_prefix = mr_state.source_guid_prefix;
        // If reader_id == UNKNOWN, message should be sent to all matched
        // readers
        if data.reader_id == EntityId::UNKNOWN {
          trace!(
            "handle_entity_submessage DATA from unknown. writer_id = {:?}",
            &data.writer_id
          );
          for reader in self
            .available_readers
            .values_mut()
            // exception: discovery protocol reader must read from unknown discovery protocol
            // writers TODO: This logic here is uglyish. Can we just inject a
            // presupposed writer (proxy) to the built-in reader as it is created?
            .filter(|r| {
              r.contains_writer(data.writer_id)
                || (data.writer_id == EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER
                  && r.entity_id() == EntityId::SPDP_BUILTIN_PARTICIPANT_READER)
            })
          {
            debug!(
              "handle_entity_submessage DATA from unknown handling in {:?}",
              &reader
            );
            reader.handle_data_msg(data.clone(), data_flags, &mr_state);
          }
        } else if let Some(target_reader) = self.reader_mut(data.reader_id) {
          target_reader.handle_data_msg(data, data_flags, &mr_state);
        }
        // bypass lane fro SPDP messages
        if writer_entity_id == EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER {
          self
            .spdp_liveness_sender
            .try_send(source_guid_prefix)
            .unwrap_or_else(|e| {
              debug!(
                "spdp_liveness_sender.try_send(): {:?}. Is Discovery alive?",
                e
              );
            });
        }
      }
      WriterSubmessage::Heartbeat(heartbeat, flags) => {
        // If reader_id == UNKNOWN, message should be sent to all matched
        // readers
        if heartbeat.reader_id == EntityId::UNKNOWN {
          for reader in self
            .available_readers
            .values_mut()
            .filter(|p| p.contains_writer(heartbeat.writer_id))
          {
            reader.handle_heartbeat_msg(
              &heartbeat,
              flags.contains(HEARTBEAT_Flags::Final),
              mr_state.clone(),
            );
          }
        } else if let Some(target_reader) = self.reader_mut(heartbeat.reader_id) {
          target_reader.handle_heartbeat_msg(
            &heartbeat,
            flags.contains(HEARTBEAT_Flags::Final),
            mr_state,
          );
        }
      }
      WriterSubmessage::Gap(gap, _flags) => {
        if let Some(target_reader) = self.reader_mut(gap.reader_id) {
          target_reader.handle_gap_msg(&gap, &mr_state);
        }
      }

      WriterSubmessage::DataFrag(datafrag, flags) => {
        // If reader_id == UNKNOWN, message should be sent to all matched readers
        if datafrag.reader_id == EntityId::UNKNOWN {
          trace!(
            "handle_entity_submessage DATA from unknown. writer_id = {:?}",
            &datafrag.writer_id
          );
          for reader in self
            .available_readers
            .values_mut()
            // exception: discovery protocol reader must read from unknown discovery protocol
            // writers TODO: This logic here is uglyish. Can we just inject a
            // presupposed writer (proxy) to the built-in reader as it is created?
            .filter(|r| {
              r.contains_writer(datafrag.writer_id)
                || (datafrag.writer_id == EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER
                  && r.entity_id() == EntityId::SPDP_BUILTIN_PARTICIPANT_READER)
            })
          {
            debug!(
              "handle_entity_submessage DATA from unknown handling in {:?}",
              &reader
            );
            reader.handle_datafrag_msg(&datafrag, flags, &mr_state);
          }
        } else if let Some(target_reader) = self.reader_mut(datafrag.reader_id) {
          target_reader.handle_datafrag_msg(&datafrag, flags, &mr_state);
        }
      }
      WriterSubmessage::HeartbeatFrag(heartbeatfrag, _flags) => {
        // If reader_id == UNKNOWN, message should be sent to all matched
        // readers
        if heartbeatfrag.reader_id == EntityId::UNKNOWN {
          for reader in self
            .available_readers
            .values_mut()
            .filter(|p| p.contains_writer(heartbeatfrag.writer_id))
          {
            reader.handle_heartbeatfrag_msg(&heartbeatfrag, &mr_state);
          }
        } else if let Some(target_reader) = self.reader_mut(heartbeatfrag.reader_id) {
          target_reader.handle_heartbeatfrag_msg(&heartbeatfrag, &mr_state);
        }
      }
    }
  }

  fn handle_reader_submessage(&mut self, submessage: ReaderSubmessage) {
    if self.dest_guid_prefix != self.own_guid_prefix && self.dest_guid_prefix != GuidPrefix::UNKNOWN
    {
      debug!("Message is not for this participant. Dropping. dest_guid_prefix={:?} participant guid={:?}", 
        self.dest_guid_prefix, self.own_guid_prefix);
      return;
    }

    // let _mr_state = self.give_message_receiver_info();
    match submessage {
      ReaderSubmessage::AckNack(acknack, _) => {
        // Note: This must not block, because the receiving end is the same thread,
        // i.e. blocking here is an instant deadlock.
        match self
          .acknack_sender
          .try_send((self.source_guid_prefix, AckSubmessage::AckNack(acknack)))
        {
          Ok(_) => (),
          Err(TrySendError::Full(_)) => {
            info!("AckNack pipe full. Looks like I am very busy. Discarding submessage.");
          }
          Err(e) => warn!("AckNack pipe fail: {:?}", e),
        }
      }

      ReaderSubmessage::NackFrag(_, _) => {}
    }
  }

  fn handle_security_submessage(&mut self, submessage: SecuritySubmessage) {
    if self.dest_guid_prefix != self.own_guid_prefix && self.dest_guid_prefix != GuidPrefix::UNKNOWN
    {
      debug!("Message is not for this participant. Dropping. dest_guid_prefix={:?} participant guid={:?}", 
        self.dest_guid_prefix, self.own_guid_prefix);
      return;
    }

    let _mr_state = self.give_message_receiver_info();
    match submessage {
      SecuritySubmessage::SecureBody(_sec_body, _sec_body_flags) => {
        todo!()
      }
      SecuritySubmessage::SecurePrefix(_sec_prefix, _sec_prefix_flags) => {
        todo!()
      }
      SecuritySubmessage::SecurePostfix(_sec_postfix, _sec_postfix_flags) => {
        todo!()
      }
      SecuritySubmessage::SecureRTPSPrefix(_sec_rtps_prefix, _sec_rtps_prefix_flags) => {
        todo!()
      }
      SecuritySubmessage::SecureRTPSPostfix(_sec_rtps_postfix, _sec_rtps_postfix_flags) => {
        todo!()
      }
    }
  }

  fn handle_interpreter_submessage(&mut self, interp_subm: InterpreterSubmessage)
  // no return value, just change state of self.
  {
    match interp_subm {
      InterpreterSubmessage::InfoTimestamp(ts_struct, _flags) => {
        // flags value was used already when parsing timestamp into an Option
        self.source_timestamp = ts_struct.timestamp;
      }
      InterpreterSubmessage::InfoSource(info_src, _flags) => {
        self.source_guid_prefix = info_src.guid_prefix;
        self.source_version = info_src.protocol_version;
        self.source_vendor_id = info_src.vendor_id;
        self.unicast_reply_locator_list.clear(); // Or invalid?
        self.multicast_reply_locator_list.clear(); // Or invalid?
        self.source_timestamp = None;
      }
      InterpreterSubmessage::InfoReply(info_reply, flags) => {
        self.unicast_reply_locator_list = info_reply.unicast_locator_list;
        if flags.contains(INFOREPLY_Flags::Multicast) {
          self.multicast_reply_locator_list = info_reply
            .multicast_locator_list
            .expect("InfoReply flag indicates multicast locator is present but none found.");
        // TODO: Convert the above error to warning only.
        } else {
          self.multicast_reply_locator_list.clear();
        }
      }
      InterpreterSubmessage::InfoDestination(info_dest, _flags) => {
        if info_dest.guid_prefix == GUID::GUID_UNKNOWN.prefix {
          self.dest_guid_prefix = self.own_guid_prefix;
        } else {
          self.dest_guid_prefix = info_dest.guid_prefix;
        }
      }
    }
  }

  pub fn notify_data_to_readers(&mut self, readers: Vec<EntityId>) {
    for eid in readers {
      self
        .available_readers
        .get_mut(&eid)
        .map(Reader::notify_cache_change);
    }
  }

  // sends 0 seqnum acknacks for those writer that haven't had any action
  pub fn send_preemptive_acknacks(&mut self) {
    for reader in self.available_readers.values_mut() {
      reader.send_preemptive_acknacks();
    }
  }
} // impl messageReceiver

#[derive(Debug, Clone)]
pub struct MessageReceiverState {
  pub source_guid_prefix: GuidPrefix,
  pub unicast_reply_locator_list: Vec<Locator>,
  pub multicast_reply_locator_list: Vec<Locator>,
  pub source_timestamp: Option<Timestamp>,
}

impl Default for MessageReceiverState {
  fn default() -> Self {
    Self {
      source_guid_prefix: GuidPrefix::default(),
      unicast_reply_locator_list: Vec::default(),
      multicast_reply_locator_list: Vec::default(),
      source_timestamp: Some(Timestamp::INVALID),
    }
  }
}

#[cfg(test)]
mod tests {
  use std::{
    rc::Rc,
    sync::{Arc, Mutex, RwLock},
  };

  use speedy::{Readable, Writable};
  use log::info;
  use serde::{Deserialize, Serialize};
  use mio_extras::channel as mio_channel;

  use crate::{
    dds::{
      qos::QosPolicies,
      statusevents::{sync_status_channel, DataReaderStatus},
      typedesc::TypeDesc,
      with_key::simpledatareader::ReaderCommand,
    },
    messages::header::Header,
    mio_source,
    network::udp_sender::UDPSender,
    rtps::reader::ReaderIngredients,
    serialization::cdr_deserializer::deserialize_from_little_endian,
    structure::{dds_cache::DDSCache, guid::EntityKind},
  };
  use super::*;

  #[test]

  fn test_shapes_demo_message_deserialization() {
    // The following message bytes contain serialized INFO_DST, INFO_TS, DATA &
    // HEARTBEAT submessages. The DATA submessage contains a ShapeType value.
    // The bytes have been captured from WireShark.
    let udp_bits1 = Bytes::from_static(&[
      0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x01, 0x0c, 0x00, 0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d,
      0x31, 0xa2, 0x28, 0x20, 0x02, 0x08, 0x09, 0x01, 0x08, 0x00, 0x1a, 0x15, 0xf3, 0x5e, 0x00,
      0xcc, 0xfb, 0x13, 0x15, 0x05, 0x2c, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x07,
      0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x00, 0x04, 0x00, 0x00, 0x00, 0x52, 0x45, 0x44, 0x00, 0x69, 0x00, 0x00, 0x00, 0x17, 0x00,
      0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x07, 0x01, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
      0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x5b, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00,
    ]);

    // The message bytes contain the following guid prefix as the message target.
    let target_gui_prefix = GuidPrefix::new(&[
      0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d, 0x31, 0xa2, 0x28, 0x20, 0x02, 0x08,
    ]);

    // The message bytes contain the following guid as the message source
    let remote_writer_guid = GUID::new(
      GuidPrefix::new(&[
        0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
      ]),
      EntityId::create_custom_entity_id([0, 0, 1], EntityKind::WRITER_WITH_KEY_USER_DEFINED),
    );

    // Create a message receiver
    let (acknack_sender, _acknack_receiver) =
      mio_channel::sync_channel::<(GuidPrefix, AckSubmessage)>(10);
    let (spdp_liveness_sender, _spdp_liveness_receiver) = mio_channel::sync_channel(8);
    let mut message_receiver =
      MessageReceiver::new(target_gui_prefix, acknack_sender, spdp_liveness_sender);

    // Create a reader to process the message
    let entity =
      EntityId::create_custom_entity_id([0, 0, 0], EntityKind::READER_WITH_KEY_USER_DEFINED);
    let reader_guid = GUID::new_with_prefix_and_id(target_gui_prefix, entity);

    let (notification_sender, _notification_receiver) = mio_channel::sync_channel::<()>(100);
    let (_notification_event_source, notification_event_sender) =
      mio_source::make_poll_channel().unwrap();
    let data_reader_waker = Arc::new(Mutex::new(None));

    let (status_sender, _status_receiver) = sync_status_channel::<DataReaderStatus>(4).unwrap();

    let (_reader_command_sender, reader_command_receiver) =
      mio_channel::sync_channel::<ReaderCommand>(10);

    let qos_policy = QosPolicies::qos_none();

    let dds_cache = Arc::new(RwLock::new(DDSCache::new()));

    let topic_cache_handle = dds_cache.write().unwrap().add_new_topic(
      "test".to_string(),
      TypeDesc::new("test".to_string()),
      &qos_policy,
    );

    let last_read_sequence_number_ref =
      Arc::new(Mutex::new(BTreeMap::<GUID, SequenceNumber>::new()));

    let reader_ing = ReaderIngredients {
      guid: reader_guid,
      notification_sender,
      status_sender,
      topic_name: "test".to_string(),
      topic_cache_handle: topic_cache_handle.clone(),
      last_read_sequence_number_ref,
      qos_policy,
      data_reader_command_receiver: reader_command_receiver,
      data_reader_waker: data_reader_waker.clone(),
      poll_event_sender: notification_event_sender,
    };

    let mut new_reader = Reader::new(
      reader_ing,
      Rc::new(UDPSender::new_with_random_port().unwrap()),
      mio_extras::timer::Builder::default().build(),
    );

    // Add info of the writer to the reader
    new_reader.matched_writer_add(
      remote_writer_guid,
      EntityId::UNKNOWN,
      vec![],
      vec![],
      &QosPolicies::qos_none(),
    );

    // Add reader to message reader and process the bytes message
    message_receiver.add_reader(new_reader);

    message_receiver.handle_received_packet(&udp_bits1);

    // Verify the message reader has recorded the right amount of submessages
    assert_eq!(message_receiver.submessage_count, 4);

    // This is not correct way to read history cache values but it serves as a test
    let sequence_numbers =
      message_receiver.get_reader_history_cache_start_and_end_seq_num(reader_guid.entity_id);
    info!(
      "history change sequence number range: {:?}",
      sequence_numbers
    );

    // Get the DDSData (serialized) from the topic cache / history cache
    let a = message_receiver
      .get_reader_and_history_cache_change(
        reader_guid.entity_id,
        *sequence_numbers.first().unwrap(),
      )
      .expect("No data in topic cache");
    info!("reader history cache DATA: {:?}", a.data());

    // Deserialize the ShapesType value from the data
    #[derive(Serialize, Deserialize, Debug, PartialEq, Eq)]
    struct ShapeType {
      color: String,
      x: i32,
      y: i32,
      size: i32,
    }
    let deserialized_shape_type: ShapeType = deserialize_from_little_endian(&a.data()).unwrap();
    info!("deserialized shapeType: {:?}", deserialized_shape_type);

    // Verify the color in the deserialized value is correct
    assert_eq!(deserialized_shape_type.color, "RED");
  }

  #[test]
  fn mr_test_submsg_count() {
    // Udp packet with INFO_DST, INFO_TS, DATA, HEARTBEAT
    let udp_bits1 = Bytes::from_static(&[
      0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x01, 0x0c, 0x00, 0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d,
      0x31, 0xa2, 0x28, 0x20, 0x02, 0x08, 0x09, 0x01, 0x08, 0x00, 0x18, 0x15, 0xf3, 0x5e, 0x00,
      0x5c, 0xf0, 0x34, 0x15, 0x05, 0x2c, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x07,
      0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x00, 0x04, 0x00, 0x00, 0x00, 0x52, 0x45, 0x44, 0x00, 0x21, 0x00, 0x00, 0x00, 0x89, 0x00,
      0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x07, 0x01, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
      0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x43, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
    ]);
    // Udp packet with INFO_DST, ACKNACK
    let udp_bits2 = Bytes::from_static(&[
      0x52, 0x54, 0x50, 0x53, 0x02, 0x03, 0x01, 0x0f, 0x01, 0x0f, 0x99, 0x06, 0x78, 0x34, 0x00,
      0x00, 0x01, 0x00, 0x00, 0x00, 0x0e, 0x01, 0x0c, 0x00, 0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d,
      0x31, 0xa2, 0x28, 0x20, 0x02, 0x08, 0x06, 0x03, 0x18, 0x00, 0x00, 0x00, 0x04, 0xc7, 0x00,
      0x00, 0x04, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x03, 0x00, 0x00, 0x00,
    ]);

    let guid_new = GUID::default();
    let (acknack_sender, _acknack_receiver) =
      mio_channel::sync_channel::<(GuidPrefix, AckSubmessage)>(10);
    let (spdp_liveness_sender, _spdp_liveness_receiver) = mio_channel::sync_channel(8);
    let mut message_receiver =
      MessageReceiver::new(guid_new.prefix, acknack_sender, spdp_liveness_sender);

    message_receiver.handle_received_packet(&udp_bits1);
    assert_eq!(message_receiver.submessage_count, 4);

    message_receiver.handle_received_packet(&udp_bits2);
    assert_eq!(message_receiver.submessage_count, 2);
  }

  #[test]
  fn mr_test_header() {
    let guid_new = GUID::default();
    let header = Header::new(guid_new.prefix);

    let bytes = header.write_to_vec().unwrap();
    let new_header = Header::read_from_buffer(&bytes).unwrap();
    assert_eq!(header, new_header);
  }
}
