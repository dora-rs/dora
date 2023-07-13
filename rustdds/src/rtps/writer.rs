use std::{
  cmp::max,
  collections::{BTreeMap, BTreeSet, HashSet},
  iter::FromIterator,
  ops::Bound::Included,
  rc::Rc,
  sync::{Arc, Mutex, MutexGuard},
};
use core::task::Waker;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use speedy::{Endianness, Writable};
use mio_extras::{
  channel::{self as mio_channel, TrySendError},
  timer::Timer,
};
use mio_06::Token;

use crate::{
  dds::{
    ddsdata::DDSData,
    qos::{
      policy,
      policy::{History, Reliability},
      HasQoSPolicy, QosPolicies,
    },
    statusevents::{CountWithChange, DataWriterStatus, StatusChannelSender},
    with_key::datawriter::WriteOptions,
  },
  messages::submessages::submessages::AckSubmessage,
  network::udp_sender::UDPSender,
  rtps::{
    dp_event_loop::{NACK_RESPONSE_DELAY, NACK_SUPPRESSION_DURATION},
    rtps_reader_proxy::RtpsReaderProxy,
    Message, MessageBuilder,
  },
  structure::{
    cache_change::CacheChange,
    dds_cache::TopicCache,
    duration::Duration,
    entity::RTPSEntity,
    guid::{EntityId, GuidPrefix, GUID},
    locator::Locator,
    sequence_number::{FragmentNumber, SequenceNumber},
    time::Timestamp,
  },
};

#[derive(PartialEq, Eq, Clone, Copy)]
pub enum DeliveryMode {
  Unicast,
  Multicast,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TimedEvent {
  Heartbeat,
  CacheCleaning,
  SendRepairData { to_reader: GUID },
  SendRepairFrags { to_reader: GUID },
}

// This is used to construct an actual Writer.
// Ingredients are sendable between threads, whereas the Writer is not.
pub(crate) struct WriterIngredients {
  pub guid: GUID,
  pub writer_command_receiver: mio_channel::Receiver<WriterCommand>,
  pub writer_command_receiver_waker: Arc<Mutex<Option<Waker>>>,
  pub topic_name: String,
  pub(crate) topic_cache_handle: Arc<Mutex<TopicCache>>, /* A handle to the topic cache in DDS
                                                          * cache */
  pub qos_policies: QosPolicies,
  pub status_sender: StatusChannelSender<DataWriterStatus>,
}

impl WriterIngredients {
  /// This token is used in timed event mio::channel HeartbeatHandler ->
  /// dpEventWrapper
  pub fn alt_entity_token(&self) -> Token {
    self.guid.entity_id.as_alt_token()
  }
}

struct AckWaiter {
  wait_until: SequenceNumber,
  complete_channel: StatusChannelSender<()>,
  readers_pending: BTreeSet<GUID>,
}

impl AckWaiter {
  pub fn notify_wait_complete(&self) {
    // it is normal for the send to fail, because receiver may have timed out
    let _ = self.complete_channel.try_send(());
  }
  pub fn reader_acked_or_lost(&mut self, guid: GUID, acked_before: Option<SequenceNumber>) -> bool // true = waiting complete
  {
    match acked_before {
      None => {
        self.readers_pending.remove(&guid);
      }
      Some(acked_before) if self.wait_until < acked_before => {
        self.readers_pending.remove(&guid);
      }
      Some(_) => (),
    }

    // if the set of waiters is empty, then wait is complete
    self.readers_pending.is_empty()
  }
}

const EPSILON_DELAY: Duration = Duration::from_nanos(10_000);

pub(crate) struct Writer {
  pub endianness: Endianness,
  pub heartbeat_message_counter: i32,
  /// Configures the mode in which the
  /// Writer operates. If
  /// pushMode==true, then the Writer
  /// will push changes to the reader. If
  /// pushMode==false, changes will
  /// only be announced via heartbeats
  /// and only be sent as response to the
  /// request of a reader
  pub push_mode: bool,
  /// Protocol tuning parameter that
  /// allows the RTPS Writer to
  /// repeatedly announce the
  /// availability of data by sending a
  /// Heartbeat Message.
  pub heartbeat_period: Option<Duration>,
  /// duration to launch cache change remove from DDSCache
  pub cache_cleaning_period: Duration,
  /// Protocol tuning parameter that
  /// allows the RTPS Writer to delay
  /// the response to a request for data
  /// from a negative acknowledgment.
  pub nack_response_delay: std::time::Duration,
  pub nackfrag_response_delay: std::time::Duration,
  pub repairfrags_continue_delay: std::time::Duration,

  /// Protocol tuning parameter that
  /// allows the RTPS Writer to ignore
  /// requests for data from negative
  /// acknowledgments that arrive ‘too
  /// soon’ after the corresponding
  /// change is sent.
  // TODO: use this
  #[allow(dead_code)]
  pub nack_suppression_duration: std::time::Duration,
  /// Internal counter used to assign
  /// increasing sequence number to
  /// each change made by the Writer
  pub last_change_sequence_number: SequenceNumber,
  /// If samples are available in the Writer, identifies the first (lowest)
  /// sequence number that is available in the Writer.
  /// If no samples are available in the Writer, identifies the lowest
  /// sequence number that is yet to be written by the Writer
  pub first_change_sequence_number: SequenceNumber,

  /// The maximum size of any
  /// SerializedPayload that may be sent by the Writer.
  /// This is used to decide when to send DATA or DATAFRAG.
  /// Supposedly "fragment size" limitations apply here, so must be <= 64k.
  /// RTPS spec v2.5 Section "8.4.14.1 Large Data"
  // Note: Writer can choose the max size at initialization, but is not allowed to change it later.
  // RTPS spec v2.5 Section 8.4.14.1.1:
  // "The fragment size must be fixed for a given Writer and is identical for all remote Readers"
  pub data_max_size_serialized: usize,

  my_guid: GUID,
  pub(crate) writer_command_receiver: mio_channel::Receiver<WriterCommand>,
  writer_command_receiver_waker: Arc<Mutex<Option<Waker>>>,
  /// The RTPS ReaderProxy class represents the information an RTPS
  /// StatefulWriter maintains on each matched RTPS Reader
  readers: BTreeMap<GUID, RtpsReaderProxy>, // TODO: Convert to BTreeMap for faster finds.
  matched_readers_count_total: i32, // all matches, never decremented
  requested_incompatible_qos_count: i32, // how many times a Reader requested incompatible QoS
  // message: Option<Message>,
  udp_sender: Rc<UDPSender>,

  // Writer can read/write to one topic only, and it stores a pointer to a mutex on the topic cache
  topic_cache: Arc<Mutex<TopicCache>>,
  /// Writer can only read/write to this topic DDSHistoryCache.
  my_topic_name: String,

  /// Maps this writers local sequence numbers to DDSHistoryCache instants.
  /// Useful when negative acknack is received.
  sequence_number_to_instant: BTreeMap<SequenceNumber, Timestamp>,

  /// Maps this writers local sequence numbers to DDSHistoryCache instants.
  /// Useful when datawriter dispose is received.
  // key_to_instant: HashMap<u128, Timestamp>,  // unused?

  /// Set of disposed samples.
  /// Useful when reader requires some sample with acknack.
  // TODO: Apparently, this is never updated.
  disposed_sequence_numbers: HashSet<SequenceNumber>,

  // When dataWriter sends cacheChange message with cacheKind is NotAliveDisposed
  // this is set true. If Datawriter after disposing sends new cacheChanges this flag is then
  // turned true.
  // When writer is in disposed state it needs to send StatusInfo_t (PID_STATUS_INFO) with
  // DisposedFlag pub writer_is_disposed: bool,
  /// Contains timer that needs to be set to timeout with duration of
  /// self.heartbeat_period timed_event_handler sends notification when timer
  /// is up via mio channel to poll in Dp_eventWrapper this also handles
  /// writers cache cleaning timeouts.
  pub(crate) timed_event_timer: Timer<TimedEvent>,

  qos_policies: QosPolicies,

  // Used for sending status info about messages sent
  status_sender: StatusChannelSender<DataWriterStatus>,
  // offered_deadline_status: OfferedDeadlineMissedStatus,
  ack_waiter: Option<AckWaiter>,
}
//#[derive(Clone)]
pub enum WriterCommand {
  // TODO: try to make this more private, like pub(crate)
  DDSData {
    ddsdata: DDSData,
    write_options: WriteOptions,
    sequence_number: SequenceNumber,
  },
  WaitForAcknowledgments {
    all_acked: StatusChannelSender<()>,
  },
  // ResetOfferedDeadlineMissedStatus { writer_guid: GUID },
}

impl Writer {
  pub fn new(
    i: WriterIngredients,
    udp_sender: Rc<UDPSender>,
    mut timed_event_timer: Timer<TimedEvent>,
  ) -> Self {
    // Verify that the topic cache corresponds to the topic of the Reader
    let topic_cache_name = i.topic_cache_handle.lock().unwrap().topic_name();
    if i.topic_name != topic_cache_name {
      panic!(
        "Topic name = {} and topic cache name = {} not equal when creating a Writer",
        i.topic_name, topic_cache_name
      );
    }

    let heartbeat_period = i
      .qos_policies
      .reliability
      .and_then(|reliability| {
        if matches!(reliability, Reliability::Reliable { .. }) {
          Some(Duration::from_secs(1))
        } else {
          None
        }
      })
      .map(|hbp| {
        // What is the logic here? Which spec section?
        if let Some(policy::Liveliness::ManualByTopic { lease_duration }) =
          i.qos_policies.liveliness
        {
          let std_dur = lease_duration;
          std_dur / 3
        } else {
          hbp
        }
      });

    // TODO: Configuration value
    let cache_cleaning_period = Duration::from_secs(2 * 60);

    // Start periodic Heartbeat
    if let Some(period) = heartbeat_period {
      timed_event_timer.set_timeout(std::time::Duration::from(period), TimedEvent::Heartbeat);
    }
    // start periodic cache cleaning
    timed_event_timer.set_timeout(
      std::time::Duration::from(cache_cleaning_period),
      TimedEvent::CacheCleaning,
    );

    Self {
      endianness: Endianness::LittleEndian,
      heartbeat_message_counter: 1,
      push_mode: true,
      heartbeat_period,
      cache_cleaning_period,
      nack_response_delay: NACK_RESPONSE_DELAY, // default value from dp_event_loop
      nackfrag_response_delay: NACK_RESPONSE_DELAY, // default value from dp_event_loop
      repairfrags_continue_delay: std::time::Duration::from_millis(1),
      nack_suppression_duration: NACK_SUPPRESSION_DURATION,
      first_change_sequence_number: SequenceNumber::from(1), // first = 1, last = 0
      last_change_sequence_number: SequenceNumber::from(0),  // means we have nothing to write
      data_max_size_serialized: 1024,
      // ^^ TODO: Maybe a smarter selection would be in order.
      // We should get the minimum over all outgoing interfaces.
      my_guid: i.guid,
      writer_command_receiver: i.writer_command_receiver,
      writer_command_receiver_waker: i.writer_command_receiver_waker,
      readers: BTreeMap::new(),
      matched_readers_count_total: 0,
      requested_incompatible_qos_count: 0,
      udp_sender,
      topic_cache: i.topic_cache_handle,
      my_topic_name: i.topic_name,
      sequence_number_to_instant: BTreeMap::new(),
      disposed_sequence_numbers: HashSet::new(),
      timed_event_timer,
      qos_policies: i.qos_policies,
      status_sender: i.status_sender,
      // offered_deadline_status: OfferedDeadlineMissedStatus::new(),
      ack_waiter: None,
    }
  }

  /// To know when token represents a writer we should look entity attribute
  /// kind this entity token can be used in DataWriter -> Writer mio::channel.
  pub fn entity_token(&self) -> Token {
    self.guid().entity_id.as_token()
  }

  pub fn is_reliable(&self) -> bool {
    self.qos_policies.reliability.is_some()
  }

  pub fn local_readers(&self) -> Vec<EntityId> {
    let min = GUID::new_with_prefix_and_id(self.my_guid.prefix, EntityId::MIN);
    let max = GUID::new_with_prefix_and_id(self.my_guid.prefix, EntityId::MAX);

    self
      .readers
      .range((Included(min), Included(max)))
      .filter_map(|(guid, _)| {
        if guid.prefix == self.my_guid.prefix {
          Some(guid.entity_id)
        } else {
          None
        }
      })
      .collect()
  }

  // TODO:
  // please explain why this is needed and why does it make sense.
  // Used by dp_event_loop.
  pub fn notify_new_data_to_all_readers(&mut self) {
    // removed, because it causes ghost sequence numbers.
    // for reader_proxy in self.readers.iter_mut() {
    //   reader_proxy.notify_new_cache_change(self.last_change_sequence_number);
    // }
  }

  // --------------------------------------------------------------
  // --------------------------------------------------------------
  // --------------------------------------------------------------

  pub fn handle_timed_event(&mut self) {
    while let Some(e) = self.timed_event_timer.poll() {
      match e {
        TimedEvent::Heartbeat => {
          self.handle_heartbeat_tick(false);
          // ^^ false = This is automatic heartbeat by timer, not manual by application
          // call.
          if let Some(period) = self.heartbeat_period {
            self
              .timed_event_timer
              .set_timeout(std::time::Duration::from(period), TimedEvent::Heartbeat);
          }
        }
        TimedEvent::CacheCleaning => {
          self.handle_cache_cleaning();
          self.timed_event_timer.set_timeout(
            std::time::Duration::from(self.cache_cleaning_period),
            TimedEvent::CacheCleaning,
          );
        }
        TimedEvent::SendRepairData {
          to_reader: reader_guid,
        } => {
          self.handle_repair_data_send(reader_guid);
          if let Some(rp) = self.lookup_reader_proxy_mut(reader_guid) {
            if rp.repair_mode {
              let delay_to_next_repair = self
                .qos_policies
                .deadline()
                .map_or_else(|| Duration::from_millis(100), |dl| dl.0)
                / 5;
              self.timed_event_timer.set_timeout(
                std::time::Duration::from(delay_to_next_repair),
                TimedEvent::SendRepairData {
                  to_reader: reader_guid,
                },
              );
            }
          }
        }
        TimedEvent::SendRepairFrags {
          to_reader: reader_guid,
        } => {
          self.handle_repair_frags_send(reader_guid);
          if let Some(rp) = self.lookup_reader_proxy_mut(reader_guid) {
            if rp.repair_frags_requested() {
              // more repair needed?
              self.timed_event_timer.set_timeout(
                self.repairfrags_continue_delay,
                TimedEvent::SendRepairFrags {
                  to_reader: reader_guid,
                },
              );
            } // if
          } // if let
        } // SendRepairFrags
      } // match
    } // while
  } // fn

  /// This is called by dp_wrapper every time cacheCleaning message is received.
  fn handle_cache_cleaning(&mut self) {
    let resource_limit = 32; // TODO: This limit should be obtained
                             // from Topic and Writer QoS. There should be some reasonable default limit
                             // in case some supplied QoS setting does not specify a larger value.
                             // In any case, there has to be some limit to avoid memory leak.

    match self.qos_policies.history {
      None => {
        self.remove_all_acked_changes_but_keep_depth(1);
      }
      Some(History::KeepAll) => {
        self.remove_all_acked_changes_but_keep_depth(resource_limit);
      }
      Some(History::KeepLast { depth: d }) => {
        self.remove_all_acked_changes_but_keep_depth(d as usize);
      }
    }
  }

  // --------------------------------------------------------------
  // --------------------------------------------------------------
  // --------------------------------------------------------------
  fn num_frags_and_frag_size(&self, payload_size: usize) -> (u32, u16) {
    let fragment_size = self.data_max_size_serialized as u32; // TODO: overflow check
    let data_size = payload_size as u32; // TODO: overflow check
                                         // Formula from RTPS spec v2.5 Section "8.3.8.3.5 Logical Interpretation"
    let num_frags = (data_size / fragment_size) + u32::from(data_size % fragment_size != 0); // rounding up
    debug!("Fragmenting {data_size} to {num_frags} x {fragment_size}");
    // TODO: Check fragment_size overflow
    (num_frags, fragment_size as u16)
  }

  // Receive new data samples from the DDS DataWriter
  pub fn process_writer_command(&mut self) {
    while let Ok(cc) = self.writer_command_receiver.try_recv() {
      match cc {
        WriterCommand::DDSData {
          ddsdata,
          write_options,
          sequence_number,
        } => {
          // Signal that there is now space in the queue
          {
            self
              .writer_command_receiver_waker
              .lock()
              .unwrap()
              .as_ref()
              .map(|w| w.wake_by_ref());
          }

          // We have a new sample here. Things to do:
          // 1. Insert it to history cache and get it sequence numbered
          // 2. Send out data. If we are pushing data, send the DATA submessage and
          // HEARTBEAT. If we are not pushing, send out HEARTBEAT only. Readers will then
          // ask for the DATA with ACKNACK, if they are interested.
          let fragmentation_needed = ddsdata.payload_size() > self.data_max_size_serialized;
          let timestamp =
            self.insert_to_history_cache(ddsdata, write_options.clone(), sequence_number);

          self.increase_heartbeat_counter();

          if !fragmentation_needed {
            let mut message_builder = MessageBuilder::new();
            // the beef: DATA submessage
            if self.push_mode {
              // If we are in push mode, proactively send DATA submessage along with
              // HEARTBEAT.
              if let Some(cache_change) =
                self.acquire_the_topic_cache_guard().get_change(&timestamp)
              {
                // If DataWriter sent us a source timestamp, then add that.
                // Timestamp has to go before Data to have effect on Data.
                if let Some(src_ts) = cache_change.write_options.source_timestamp {
                  message_builder = message_builder.ts_msg(self.endianness, Some(src_ts));
                }
                message_builder = message_builder.data_msg(
                  cache_change,
                  EntityId::UNKNOWN,      // reader
                  self.my_guid.entity_id, // writer
                  self.endianness,
                );
              } else {
                // We just did .insert_to_history_cache but nothing was found?
                error!(
                  "process_writer_command: The dog ate my CacheChange {:?} topic={:?}",
                  sequence_number,
                  self.topic_name(),
                );
              }
            } else {
              // Not pushing: Send only HEARTBEAT. Send DATA only after readers
              // ACKNACK asking for it.
            };

            let final_flag = false; // false = request that readers acknowledge with ACKNACK.
            let liveliness_flag = false; // This is not a manual liveliness assertion (DDS API call), but side-effect of
                                         // writing new data.
            let data_hb_message = message_builder
              .heartbeat_msg(self, EntityId::UNKNOWN, final_flag, liveliness_flag)
              .add_header_and_build(self.my_guid.prefix);
            self.send_message_to_readers(
              DeliveryMode::Multicast,
              &data_hb_message,
              &mut self.readers.values(),
            );
          } else {
            // Large payload, must fragment.
            if let Some(cache_change) = self.acquire_the_topic_cache_guard().get_change(&timestamp)
            {
              let data_size = cache_change.data_value.payload_size();
              let (num_frags, fragment_size) = self.num_frags_and_frag_size(data_size);

              if self.push_mode {
                // loop over fragments
                for frag_num in FragmentNumber::range_inclusive(
                  FragmentNumber::new(1),
                  FragmentNumber::new(num_frags),
                ) {
                  let mut message_builder = MessageBuilder::new();
                  if let Some(src_ts) = cache_change.write_options.source_timestamp {
                    message_builder = message_builder.ts_msg(self.endianness, Some(src_ts));
                  }

                  message_builder = message_builder.data_frag_msg(
                    cache_change,
                    EntityId::UNKNOWN,      // reader
                    self.my_guid.entity_id, // writer
                    frag_num,
                    fragment_size,
                    data_size.try_into().unwrap(),
                    self.endianness,
                  );

                  // TODO: some sort of queuing is needed
                  self.send_message_to_readers(
                    DeliveryMode::Multicast,
                    &message_builder.add_header_and_build(self.my_guid.prefix),
                    &mut self.readers.values(),
                  );
                } // end for
              }
              // Regardless of push mode, we send a Heartbeat
              let final_flag = false; // false = request that readers acknowledge with ACKNACK.
              let liveliness_flag = false; // This is not a manual liveliness assertion (DDS API call), but side-effect of
              let hb_message = MessageBuilder::new()
                .heartbeat_msg(self, EntityId::UNKNOWN, final_flag, liveliness_flag)
                .add_header_and_build(self.my_guid.prefix);
              self.send_message_to_readers(
                DeliveryMode::Multicast,
                &hb_message,
                &mut self.readers.values(),
              );
            } else {
              // We just did .insert_to_history_cache but nothing was found?
              error!(
                "process_writer_command (frag): The dog ate my CacheChange {:?} topic={:?}",
                sequence_number,
                self.topic_name(),
              );
            }
          } // end if large payload
        }

        // WriterCommand::ResetOfferedDeadlineMissedStatus { writer_guid: _, } => {
        //   self.reset_offered_deadline_missed_status();
        // }
        WriterCommand::WaitForAcknowledgments { all_acked } => {
          let wait_until = self.last_change_sequence_number;
          let readers_pending: BTreeSet<_> = self
            .readers
            .iter()
            .filter_map(|(guid, rp)| {
              if rp.qos().reliability().is_some() {
                if rp.all_acked_before <= wait_until {
                  Some(*guid)
                } else {
                  None // already acked
                }
              } else {
                None // not reliable reader => not supposed to ack at all
              }
            })
            .collect();
          self.ack_waiter = if readers_pending.is_empty() {
            // all acked already: try to signal app waiting at DataWriter
            let _ = all_acked.try_send(());
            // but we ignore any failure to signal, if no-one is listening
            // since that is normal. They may have timeouted and stopped waiting.
            None
          } else {
            // Someone still needs to ack. Wait for them.
            Some(AckWaiter {
              wait_until,
              complete_channel: all_acked,
              readers_pending,
            })
          };
        }
      }
    }
  }

  fn insert_to_history_cache(
    &mut self,
    data: DDSData,
    write_options: WriteOptions,
    sequence_number: SequenceNumber,
  ) -> Timestamp {
    // first increasing last SequenceNumber
    let new_sequence_number = sequence_number;
    self.last_change_sequence_number = new_sequence_number;

    // setting first change sequence number according to our qos (not offering more
    // than our QOS says)
    self.first_change_sequence_number = match self.qos().history {
      None => self.last_change_sequence_number, // default: depth = 1

      Some(History::KeepAll) =>
      // Now that we have a change, is must be at least one
      {
        max(self.first_change_sequence_number, SequenceNumber::from(1))
      }

      Some(History::KeepLast { depth }) => max(
        self.last_change_sequence_number - SequenceNumber::from(i64::from(depth - 1)),
        SequenceNumber::from(1),
      ),
    };
    assert!(self.first_change_sequence_number > SequenceNumber::zero());
    assert!(self.last_change_sequence_number > SequenceNumber::zero());

    // create new CacheChange from DDSData
    let new_cache_change = CacheChange::new(self.guid(), new_sequence_number, write_options, data);

    // Insert to topic cache
    // timestamp taken here is used as a unique(!) key in the DDSCache.
    let timestamp = Timestamp::now();
    self
      .acquire_the_topic_cache_guard()
      .add_change(&timestamp, new_cache_change);

    // keeping table of instant sequence number pairs
    self
      .sequence_number_to_instant
      .insert(new_sequence_number, timestamp);

    // update key to timestamp mapping
    // self.key_to_instant.insert(data_key, timestamp);

    // Notify reader proxies that there is a new sample
    for reader in &mut self.readers.values_mut() {
      reader.notify_new_cache_change(new_sequence_number);
    }
    timestamp
  }

  // --------------------------------------------------------------
  // --------------------------------------------------------------
  // --------------------------------------------------------------

  /// This is called periodically.
  pub fn handle_heartbeat_tick(&mut self, is_manual_assertion: bool) {
    // Reliable Stateless Writer will set the final flag.
    // Reliable Stateful Writer (that tracks Readers by ReaderProxy) will not set
    // the final flag.
    let final_flag = false;
    let liveliness_flag = is_manual_assertion; // RTPS spec "8.3.7.5 Heartbeat"

    trace!(
      "heartbeat tick in topic {:?} have {} readers",
      self.topic_name(),
      self.readers.len()
    );

    self.increase_heartbeat_counter();
    // TODO: This produces same heartbeat count for all messages sent, but
    // then again, they represent the same writer status.

    if self
      .readers
      .values()
      .all(|rp| self.last_change_sequence_number < rp.all_acked_before)
    {
      trace!("heartbeat tick: all readers have all available data.");
    } else {
      let hb_message = MessageBuilder::new()
        .ts_msg(self.endianness, Some(Timestamp::now()))
        .heartbeat_msg(self, EntityId::UNKNOWN, final_flag, liveliness_flag)
        .add_header_and_build(self.my_guid.prefix);
      debug!(
        "Writer {:?} topic={:} HEARTBEAT {:?}",
        self.guid().entity_id,
        self.topic_name(),
        hb_message
      );
      self.send_message_to_readers(
        DeliveryMode::Multicast,
        &hb_message,
        &mut self.readers.values(),
      );
    }
  }

  /// When receiving an ACKNACK Message indicating a Reader is missing some data
  /// samples, the Writer must respond by either sending the missing data
  /// samples, sending a GAP message when the sample is not relevant, or
  /// sending a HEARTBEAT message when the sample is no longer available
  pub fn handle_ack_nack(
    &mut self,
    reader_guid_prefix: GuidPrefix,
    ack_submessage: &AckSubmessage,
  ) {
    // sanity check
    if !self.is_reliable() {
      warn!(
        "Writer {:x?} is best effort! It should not handle acknack messages!",
        self.entity_id()
      );
      return;
    }

    match ack_submessage {
      AckSubmessage::AckNack(ref an) => {
        // Update the ReaderProxy
        let last_seq = self.last_change_sequence_number; // to avoid borrow problems

        // sanity check requested sequence numbers
        match an.reader_sn_state.iter().next().map(i64::from) {
          Some(0) => warn!("Request for SN zero! : {:?}", an),
          Some(_) | None => (), // ok
        }
        let my_topic = self.my_topic_name.clone(); // for debugging
        let reader_guid = GUID::new(reader_guid_prefix, an.reader_id);

        self.update_ack_waiters(reader_guid, Some(an.reader_sn_state.base()));

        if let Some(reader_proxy) = self.lookup_reader_proxy_mut(reader_guid) {
          // Mark requested SNs as "unsent changes"
          reader_proxy.handle_ack_nack(ack_submessage, last_seq);

          let reader_guid = reader_proxy.remote_reader_guid; // copy to avoid double mut borrow
                                                             // Sanity Check: if the reader asked for something we did not even advertise
                                                             // yet. TODO: This
                                                             // checks the stored unset_changes, not presently received ACKNACK.
          if let Some(&high) = reader_proxy.unsent_changes.iter().next_back() {
            if high > last_seq {
              warn!(
                "ReaderProxy {:?} thinks we need to send {:?} but I have only up to {:?}",
                reader_proxy.remote_reader_guid, reader_proxy.unsent_changes, last_seq
              );
            }
          }
          // Sanity Check
          if an.reader_sn_state.base() > last_seq + SequenceNumber::from(1i64) {
            // more sanity
            warn!(
              "ACKNACK from {:?} acks {:?}, but I have only up to {:?} count={:?} topic={:?}",
              reader_proxy.remote_reader_guid, an.reader_sn_state, last_seq, an.count, my_topic
            );
          }
          if let Some(max_req_sn) = an.reader_sn_state.iter().next_back() {
            // sanity check
            if max_req_sn > last_seq {
              warn!(
                "ACKNACK from {:?} requests {:?} but I have only up to {:?}",
                reader_proxy.remote_reader_guid,
                an.reader_sn_state.iter().collect::<Vec<SequenceNumber>>(),
                last_seq
              );
            }
          }

          // if we cannot send more data, we are done.
          // This is to prevent empty "repair data" messages from being sent.
          if reader_proxy.all_acked_before > last_seq {
            reader_proxy.repair_mode = false;
          } else {
            reader_proxy.repair_mode = true; // TODO: Is this correct? Do we need to repair immediately?
                                             // set repair timer to fire
            self.timed_event_timer.set_timeout(
              self.nack_response_delay,
              TimedEvent::SendRepairData {
                to_reader: reader_guid,
              },
            );
          }
        }
      }
      AckSubmessage::NackFrag(ref nackfrag) => {
        // NackFrag is negative acknowledgement only, i.e. requesting missing fragments.

        let reader_guid = GUID::new(reader_guid_prefix, nackfrag.reader_id);
        if let Some(reader_proxy) = self.lookup_reader_proxy_mut(reader_guid) {
          reader_proxy.mark_frags_requested(nackfrag.writer_sn, &nackfrag.fragment_number_state);
        }
        self.timed_event_timer.set_timeout(
          self.nackfrag_response_delay,
          TimedEvent::SendRepairFrags {
            to_reader: reader_guid,
          },
        );
      }
    }
  }

  fn update_ack_waiters(&mut self, guid: GUID, acked_before: Option<SequenceNumber>) {
    let completed = self
      .ack_waiter
      .as_mut()
      .map_or(false, |aw| aw.reader_acked_or_lost(guid, acked_before));
    if completed {
      self
        .ack_waiter
        .as_ref()
        .map(AckWaiter::notify_wait_complete);
      self.ack_waiter = None;
    }
  }

  // Send out missing data

  fn handle_repair_data_send(&mut self, to_reader: GUID) {
    // Note: here we remove the reader from our reader map temporarily.
    // Then we can mutate both the reader and other fields in self.
    // Doing a .get_mut() on the reader map would make self immutable.
    if let Some(mut reader_proxy) = self.readers.remove(&to_reader) {
      // We use a worker function to ensure that afterwards we can insert the
      // reader_proxy back. This technique ensures that all return paths lead to
      // re-insertion.
      self.handle_repair_data_send_worker(&mut reader_proxy);
      // insert reader back
      if let Some(rp) = self
        .readers
        .insert(reader_proxy.remote_reader_guid, reader_proxy)
      {
        error!("Reader proxy was duplicated somehow??? {:?}", rp);
      }
    }
  }

  fn handle_repair_frags_send(&mut self, to_reader: GUID) {
    // see similar function above
    if let Some(mut reader_proxy) = self.readers.remove(&to_reader) {
      self.handle_repair_frags_send_worker(&mut reader_proxy);
      if let Some(rp) = self
        .readers
        .insert(reader_proxy.remote_reader_guid, reader_proxy)
      {
        // this is an internal logic error, or maybe out of memory
        error!("Reader proxy was duplicated somehow??? (frags) {:?}", rp);
      }
    }
  }

  fn handle_repair_data_send_worker(&mut self, reader_proxy: &mut RtpsReaderProxy) {
    // Note: The reader_proxy is now removed from readers map
    let reader_guid = reader_proxy.remote_reader_guid;
    let mut partial_message = MessageBuilder::new()
      .dst_submessage(self.endianness, reader_guid.prefix)
      .ts_msg(self.endianness, Some(Timestamp::now()));
    // TODO: This timestamp should probably not be
    // the current (retransmit) time, but the initial sample production timestamp,
    // i.e. should be read from DDSCache (and be implemented there)
    debug!(
      "Repair data send due to ACKNACK. ReaderProxy Unsent changes: {:?}",
      reader_proxy.unsent_changes
    );

    let mut no_longer_relevant = Vec::new();
    let mut found_data = false;
    let mut sending_data = false;
    let mut sending_gap = false;
    let mut trigger_send_repair_frags = false;
    if let Some(&unsent_sn) = reader_proxy.unsent_changes.iter().next() {
      // There are unsent changes.
      if let Some(timestamp) = self.sequence_number_to_instant(unsent_sn) {
        // Try to find the cache change from topic cache
        if let Some(cache_change) = self.acquire_the_topic_cache_guard().get_change(&timestamp) {
          // CacheChange found, check if we can send it in one piece (i.e. DATA)
          if cache_change.data_value.payload_size() <= self.data_max_size_serialized {
            // construct DATA submessage
            partial_message = partial_message.data_msg(
              cache_change,
              reader_guid.entity_id,  // reader
              self.my_guid.entity_id, // writer
              self.endianness,
            );
            // TODO: Here we are cloning the entire payload. We need to rewrite
            // the transmit path to avoid copying.
            sending_data = true;
          } else {
            // Large data: arrange DATAFRAGs to be sent
            let (num_frags, _frag_size) =
              self.num_frags_and_frag_size(cache_change.data_value.payload_size());
            reader_proxy.mark_all_frags_requested(unsent_sn, num_frags);
            // We cannot set up a repair timer right here, because self is already borrowed
            // So just set a flag.
            trigger_send_repair_frags = true;
          }
        } else {
          // Change not in cache anymore, mark SN as not relevant anymore
          no_longer_relevant.push(unsent_sn);
        }
      } else {
        // SN did not map to an Instant. Maybe it has been removed?
        if unsent_sn < self.first_change_sequence_number {
          debug!(
            "Reader {:?} requested too old data {:?}. I have only from {:?}. Topic {:?}",
            &reader_proxy, unsent_sn, self.first_change_sequence_number, &self.my_topic_name
          );
          // noting to do
        } else if self.disposed_sequence_numbers.contains(&unsent_sn) {
          debug!(
            "Reader {:?} requested disposed {:?}. Topic {:?}",
            &reader_proxy, unsent_sn, &self.my_topic_name
          );
          // nothing to do
        } else {
          // we are running out of excuses
          error!(
            "handle ack_nack writer {:?} seq.number {:?} missing from instant map",
            self.my_guid, unsent_sn
          );
        }
        no_longer_relevant.push(unsent_sn);
      } // match

      // This SN will be sent or found no longer relevant => remove
      // from unsent list.
      reader_proxy.unsent_changes.remove(&unsent_sn);
      found_data = true;
    }
    // Add GAP submessage, if some cache changes could not be found.
    if !no_longer_relevant.is_empty() {
      partial_message =
        partial_message.gap_msg(&BTreeSet::from_iter(no_longer_relevant), self, reader_guid);
      sending_gap = true;
    }

    // if we have DATA or GAP to send, then build message and send
    if sending_data || sending_gap {
      let data_gap_msg = partial_message.add_header_and_build(self.my_guid.prefix);
      self.send_message_to_readers(
        DeliveryMode::Unicast,
        &data_gap_msg,
        &mut std::iter::once(&*reader_proxy),
      );
    }
    if trigger_send_repair_frags {
      self.timed_event_timer.set_timeout(
        EPSILON_DELAY.into(),
        TimedEvent::SendRepairFrags {
          to_reader: reader_guid,
        },
      );
    }
    // Update repair_mode flag
    if found_data {
      // Data was found. Continue repairing.
    } else {
      // Unsent list is empty. Switch off repair mode.
      reader_proxy.repair_mode = false;
    }
  } // fn

  fn handle_repair_frags_send_worker(
    &mut self,
    reader_proxy: &mut RtpsReaderProxy, /* This is mutable proxy temporarily detached from the
                                         * set of reader proxies */
  ) {
    // Decide the (max) number of frags to be sent
    let max_send_count = 8;

    // Get (an iterator to) frags requested but not yet sent
    // reader_proxy.
    // Iterate over frags to be sent
    for (seq_num, frag_num) in reader_proxy.frags_requested_iterator().take(max_send_count) {
      // Sanity check request
      // ^^^ TODO

      if let Some(timestamp) = self.sequence_number_to_instant(seq_num) {
        // Try to find the cache change from topic cache
        if let Some(cache_change) = self.acquire_the_topic_cache_guard().get_change(&timestamp) {
          // Generate datafrag message
          let mut message_builder = MessageBuilder::new();
          if let Some(src_ts) = cache_change.write_options.source_timestamp {
            message_builder = message_builder.ts_msg(self.endianness, Some(src_ts));
          }

          let fragment_size: u32 = self.data_max_size_serialized as u32; // TODO: overflow check
          let data_size: u32 = cache_change.data_value.payload_size() as u32; // TODO: overflow check

          message_builder = message_builder.data_frag_msg(
            cache_change,
            reader_proxy.remote_reader_guid.entity_id, // reader
            self.my_guid.entity_id,                    // writer
            frag_num,
            fragment_size as u16, // TODO: overflow check
            data_size,
            self.endianness,
          );

          // TODO: some sort of queuing is needed
          self.send_message_to_readers(
            DeliveryMode::Unicast,
            &message_builder.add_header_and_build(self.my_guid.prefix),
            &mut self.readers.values(),
          );
        } else {
          error!(
            "handle_repair_frags_send_worker: {:?} missing from DDSCache. topic={:?}",
            seq_num, self.my_topic_name
          );
          // TODO: Should we send a GAP message then?
        }
      } else {
        error!(
          "handle_repair_frags_send_worker: {:?} missing from instant map. topic={:?}",
          seq_num, self.my_topic_name
        );
      }

      reader_proxy.mark_frag_sent(seq_num, &frag_num);
    } // for
  } // fn

  /// Removes permanently cacheChanges from DDSCache.
  /// CacheChanges can be safely removed only if they are acked by all readers.
  /// (Reliable) Depth is QoS policy History depth.
  /// Returns SequenceNumbers of removed CacheChanges
  /// This is called repeatedly by handle_cache_cleaning action.
  fn remove_all_acked_changes_but_keep_depth(&mut self, depth: usize) {
    // All readers have acked up to this point (SequenceNumber)
    let acked_by_all_readers = self
      .readers
      .values()
      .map(RtpsReaderProxy::acked_up_to_before)
      .min()
      .unwrap_or_else(SequenceNumber::zero);
    // If all readers have acked all up to before 5, and depth is 5, we need
    // to keep samples 0..4, i.e. from acked_up_to_before - depth .
    let first_keeper = max(
      acked_by_all_readers - SequenceNumber::from(depth),
      self.first_change_sequence_number,
    );

    // We notify the topic cache that it can release older samples
    // as far as this Writer is concerned.
    if let Some(&keep_instant) = self.sequence_number_to_instant.get(&first_keeper) {
      self
        .acquire_the_topic_cache_guard()
        .remove_changes_before(keep_instant);
    } else {
      // if we end up with SequenceNumber(1), it may be due to "max()" above,
      // and may mean that no messages have ever been received, so it is
      // normal that we did not find anything.
      if first_keeper > SequenceNumber::new(1) {
        warn!(
          "DDCache garbage collect: {:?} missing from instant map",
          first_keeper
        );
      } else {
        debug!(
          "DDCache garbage collect: {:?} missing from instant map. But it is normal for number 1.",
          first_keeper
        );
      }
    }
    self.first_change_sequence_number = first_keeper;
    self.sequence_number_to_instant = self.sequence_number_to_instant.split_off(&first_keeper);
  }

  fn increase_heartbeat_counter(&mut self) {
    self.heartbeat_message_counter += 1;
  }

  fn send_message_to_readers(
    &self,
    preferred_mode: DeliveryMode,
    message: &Message,
    readers: &mut dyn Iterator<Item = &RtpsReaderProxy>,
  ) {
    // TODO: This is a stupid transmit algorithm. We should compute a preferred
    // unicast and multicast locators for each reader only on every reader update,
    // and not find it dynamically on every message.
    let buffer = message.write_to_vec_with_ctx(self.endianness).unwrap();
    let mut already_sent_to = BTreeSet::new();

    macro_rules! send_unless_sent_and_mark {
      ($locs:expr) => {
        for loc in $locs.iter() {
          if already_sent_to.contains(loc) {
            trace!("Already sent to {:?}", loc);
          } else {
            self.udp_sender.send_to_locator(&buffer, loc);
            already_sent_to.insert(loc.clone());
          }
        }
      };
    }

    for reader in readers {
      match (
        preferred_mode,
        reader
          .unicast_locator_list
          .iter()
          .find(|l| Locator::is_udp(l)),
        reader
          .multicast_locator_list
          .iter()
          .find(|l| Locator::is_udp(l)),
      ) {
        (DeliveryMode::Multicast, _, Some(_mc_locator)) => {
          send_unless_sent_and_mark!(reader.multicast_locator_list);
        }
        (DeliveryMode::Unicast, Some(_uc_locator), _) => {
          send_unless_sent_and_mark!(reader.unicast_locator_list)
        }
        (_delivery_mode, _, Some(_mc_locator)) => {
          send_unless_sent_and_mark!(reader.multicast_locator_list);
        }
        (_delivery_mode, Some(_uc_locator), _) => {
          send_unless_sent_and_mark!(reader.unicast_locator_list)
        }
        (_delivery_mode, None, None) => {
          warn!("send_message_to_readers: No locators for {:?}", reader);
        }
      } // match
    }
  }

  // Send status to DataWriter or however is listening
  fn send_status(&self, status: DataWriterStatus) {
    self
      .status_sender
      .try_send(status)
      .unwrap_or_else(|e| match e {
        TrySendError::Full(_) => (), // This is normal in case there is no receiver
        TrySendError::Disconnected(_) => {
          debug!("send_status - status receiver is disconnected");
        }
        TrySendError::Io(e) => {
          warn!("send_status - io error {e:?}");
        }
      });
  }

  pub fn update_reader_proxy(
    &mut self,
    reader_proxy: &RtpsReaderProxy,
    requested_qos: &QosPolicies,
  ) {
    debug!("update_reader_proxy topic={:?}", self.my_topic_name);
    match self.qos_policies.compliance_failure_wrt(requested_qos) {
      // matched QoS
      None => {
        let change = self.matched_reader_update(reader_proxy.clone());
        if change > 0 {
          self.matched_readers_count_total += change;
          self.send_status(DataWriterStatus::PublicationMatched {
            total: CountWithChange::new(self.matched_readers_count_total, change),
            current: CountWithChange::new(self.readers.len() as i32, change),
          });
          // send out heartbeat, so that new reader can catch up
          if let Some(Reliability::Reliable { .. }) = self.qos_policies.reliability {
            self.notify_new_data_to_all_readers();
          }
          info!(
            "Matched new remote reader on topic={:?} reader={:?}",
            self.topic_name(),
            &reader_proxy.remote_reader_guid
          );
          debug!("Reader details: {:?}", &reader_proxy);
        }
      }
      Some(bad_policy_id) => {
        // QoS not compliant :(
        warn!(
          "update_reader_proxy - QoS mismatch {:?} topic={:?}",
          bad_policy_id,
          self.topic_name()
        );
        self.requested_incompatible_qos_count += 1;
        self.send_status(DataWriterStatus::OfferedIncompatibleQos {
          count: CountWithChange::new(self.requested_incompatible_qos_count, 1),
          last_policy_id: bad_policy_id,
          policies: Vec::new(), // TODO: implement this
        });
      }
    } // match
  }

  // Update the given reader proxy. Preserve data we are tracking.
  // return 0 if the reader already existed
  // return 1 if it was new ( = count of added reader proxies)
  fn matched_reader_update(&mut self, reader_proxy: RtpsReaderProxy) -> i32 {
    let (to_insert, count_change) = match self.readers.remove(&reader_proxy.remote_reader_guid) {
      None => (reader_proxy, 1),
      Some(existing_reader) => (
        RtpsReaderProxy {
          is_active: existing_reader.is_active,
          all_acked_before: existing_reader.all_acked_before,
          unsent_changes: existing_reader.unsent_changes,
          repair_mode: existing_reader.repair_mode,
          ..reader_proxy
        },
        0,
      ),
    };
    self.readers.insert(to_insert.remote_reader_guid, to_insert);
    count_change
  }

  fn matched_reader_remove(&mut self, guid: GUID) -> Option<RtpsReaderProxy> {
    let removed = self.readers.remove(&guid);
    if let Some(ref removed_reader) = removed {
      info!(
        "Removed reader proxy. topic={:?} reader={:?}",
        self.topic_name(),
        removed_reader.remote_reader_guid,
      );
      debug!("Removed reader proxy details: {:?}", removed_reader);
    }
    removed
  }

  pub fn reader_lost(&mut self, guid: GUID) {
    if self.readers.contains_key(&guid) {
      info!(
        "reader_lost topic={:?} reader={:?}",
        self.topic_name(),
        &guid
      );
      self.matched_reader_remove(guid);
      // self.matched_readers_count_total -= 1; // this never decreases
      self.send_status(DataWriterStatus::PublicationMatched {
        total: CountWithChange::new(self.matched_readers_count_total, 0),
        current: CountWithChange::new(self.readers.len() as i32, -1),
      });
    }
    // also remember to remove reader from ack_waiter
    self.update_ack_waiters(guid, None);
  }

  // Entire remote participant was lost.
  // Remove all remote readers belonging to it.
  pub fn participant_lost(&mut self, guid_prefix: GuidPrefix) {
    let lost_readers: Vec<GUID> = self
      .readers
      .range(guid_prefix.range())
      .map(|(g, _)| *g)
      .collect();
    for reader in lost_readers {
      self.reader_lost(reader);
    }
  }

  fn lookup_reader_proxy_mut(&mut self, guid: GUID) -> Option<&mut RtpsReaderProxy> {
    self.readers.get_mut(&guid)
  }

  pub fn sequence_number_to_instant(&self, seqnumber: SequenceNumber) -> Option<Timestamp> {
    self.sequence_number_to_instant.get(&seqnumber).copied()
  }

  pub fn topic_name(&self) -> &String {
    &self.my_topic_name
  }

  fn acquire_the_topic_cache_guard(&self) -> MutexGuard<TopicCache> {
    self.topic_cache.lock().unwrap_or_else(|e| {
      panic!(
        "The topic cache of topic {} is poisoned. Error: {}",
        &self.my_topic_name, e
      )
    })
  }

  // TODO
  // This is placeholder for not-yet-implemented feature.
  //
  // pub fn reset_offered_deadline_missed_status(&mut self) {
  //   self.offered_deadline_status.reset_change();
  // }
}

impl RTPSEntity for Writer {
  fn guid(&self) -> GUID {
    self.my_guid
  }
}

impl HasQoSPolicy for Writer {
  fn qos(&self) -> QosPolicies {
    self.qos_policies.clone()
  }
}

// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
  use std::thread;

  use byteorder::LittleEndian;
  use log::info;

  use crate::{
    dds::{
      participant::DomainParticipant, qos::QosPolicies, topic::TopicKind,
      with_key::datawriter::DataWriter,
    },
    serialization::cdr_serializer::CDRSerializerAdapter,
    test::random_data::*,
  };

  #[test]
  fn test_writer_receives_datawriter_cache_change_notifications() {
    let domain_participant = DomainParticipant::new(0).expect("Failed to create participant");
    let qos = QosPolicies::qos_none();
    let _default_dw_qos = QosPolicies::qos_none();

    let publisher = domain_participant
      .create_publisher(&qos)
      .expect("Failed to create publisher");
    let topic = domain_participant
      .create_topic(
        "Aasii".to_string(),
        "Huh?".to_string(),
        &qos,
        TopicKind::WithKey,
      )
      .expect("Failed to create topic");
    let data_writer: DataWriter<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>> =
      publisher
        .create_datawriter(&topic, None)
        .expect("Failed to create datawriter");

    let data = RandomData {
      a: 4,
      b: "Fobar".to_string(),
    };

    let data2 = RandomData {
      a: 2,
      b: "Fobar".to_string(),
    };

    let data3 = RandomData {
      a: 3,
      b: "Fobar".to_string(),
    };

    let write_result = data_writer.write(data, None);
    info!("writerResult:  {:?}", write_result);

    data_writer
      .write(data2, None)
      .expect("Unable to write data");

    info!("writerResult:  {:?}", write_result);
    let write_result = data_writer.write(data3, None);

    thread::sleep(std::time::Duration::from_millis(100));
    info!("writerResult:  {:?}", write_result);
  }
}
