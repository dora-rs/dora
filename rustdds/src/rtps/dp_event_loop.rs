use std::{
  collections::HashMap,
  rc::Rc,
  sync::{Arc, RwLock},
  time::{Duration, Instant},
};

use log::{debug, error, info, trace, warn};
use mio_06::{Event, Events, Poll, PollOpt, Ready, Token};
use mio_extras::channel as mio_channel;

use crate::{
  dds::{qos::policy, typedesc::TypeDesc},
  discovery::{
    builtin_endpoint::BuiltinEndpointSet,
    discovery::Discovery,
    discovery_db::DiscoveryDB,
    sedp_messages::{DiscoveredReaderData, DiscoveredWriterData},
  },
  messages::submessages::submessages::AckSubmessage,
  network::{constant::*, udp_listener::UDPListener, udp_sender::UDPSender},
  rtps::{
    message_receiver::MessageReceiver,
    reader::{Reader, ReaderIngredients},
    rtps_reader_proxy::RtpsReaderProxy,
    rtps_writer_proxy::RtpsWriterProxy,
    writer::{Writer, WriterIngredients},
  },
  structure::{
    dds_cache::DDSCache,
    entity::RTPSEntity,
    guid::{EntityId, GuidPrefix, TokenDecode, GUID},
  },
};

pub struct DomainInfo {
  pub domain_participant_guid: GUID,
  pub domain_id: u16,
  pub participant_id: u16,
}

pub const PREEMPTIVE_ACKNACK_PERIOD: Duration = Duration::from_secs(5);

// RTPS spec Section 8.4.7.1.1  "Default Timing-Related Values"
pub const NACK_RESPONSE_DELAY: Duration = Duration::from_millis(200);
pub const NACK_SUPPRESSION_DURATION: Duration = Duration::from_millis(0);

pub struct DPEventLoop {
  domain_info: DomainInfo,
  poll: Poll,
  ddscache: Arc<RwLock<DDSCache>>,
  discovery_db: Arc<RwLock<DiscoveryDB>>,
  udp_listeners: HashMap<Token, UDPListener>,
  message_receiver: MessageReceiver, // This contains our Readers

  // Adding readers
  add_reader_receiver: TokenReceiverPair<ReaderIngredients>,
  remove_reader_receiver: TokenReceiverPair<GUID>,

  // Writers
  add_writer_receiver: TokenReceiverPair<WriterIngredients>,
  remove_writer_receiver: TokenReceiverPair<GUID>,
  stop_poll_receiver: mio_channel::Receiver<()>,
  // GuidPrefix sent in this channel needs to be RTPSMessage source_guid_prefix. Writer needs this
  // to locate RTPSReaderProxy if negative acknack.
  ack_nack_receiver: mio_channel::Receiver<(GuidPrefix, AckSubmessage)>,

  writers: HashMap<EntityId, Writer>,
  udp_sender: Rc<UDPSender>,

  discovery_update_notification_receiver: mio_channel::Receiver<DiscoveryNotificationType>,
}

impl DPEventLoop {
  // This pub(crate) , because it should be constructed only by DomainParticipant.
  #[allow(clippy::too_many_arguments)]
  pub(crate) fn new(
    domain_info: DomainInfo,
    udp_listeners: HashMap<Token, UDPListener>,
    ddscache: Arc<RwLock<DDSCache>>,
    discovery_db: Arc<RwLock<DiscoveryDB>>,
    participant_guid_prefix: GuidPrefix,
    add_reader_receiver: TokenReceiverPair<ReaderIngredients>,
    remove_reader_receiver: TokenReceiverPair<GUID>,
    add_writer_receiver: TokenReceiverPair<WriterIngredients>,
    remove_writer_receiver: TokenReceiverPair<GUID>,
    stop_poll_receiver: mio_channel::Receiver<()>,
    discovery_update_notification_receiver: mio_channel::Receiver<DiscoveryNotificationType>,
    spdp_liveness_sender: mio_channel::SyncSender<GuidPrefix>,
  ) -> Self {
    let poll = Poll::new().expect("Unable to create new poll.");
    let (acknack_sender, acknack_receiver) =
      mio_channel::sync_channel::<(GuidPrefix, AckSubmessage)>(100);
    let mut udp_listeners = udp_listeners;
    for (token, listener) in &mut udp_listeners {
      poll
        .register(
          listener.mio_socket(),
          *token,
          Ready::readable(),
          PollOpt::edge(),
        )
        .expect("Failed to register listener.");
    }

    poll
      .register(
        &add_reader_receiver.receiver,
        add_reader_receiver.token,
        Ready::readable(),
        PollOpt::edge(),
      )
      .expect("Failed to register reader adder.");

    poll
      .register(
        &remove_reader_receiver.receiver,
        remove_reader_receiver.token,
        Ready::readable(),
        PollOpt::edge(),
      )
      .expect("Failed to register reader remover.");
    poll
      .register(
        &add_writer_receiver.receiver,
        add_writer_receiver.token,
        Ready::readable(),
        PollOpt::edge(),
      )
      .expect("Failed to register add writer channel");

    poll
      .register(
        &remove_writer_receiver.receiver,
        remove_writer_receiver.token,
        Ready::readable(),
        PollOpt::edge(),
      )
      .expect("Failed to register remove writer channel");

    poll
      .register(
        &stop_poll_receiver,
        STOP_POLL_TOKEN,
        Ready::readable(),
        PollOpt::edge(),
      )
      .expect("Failed to register stop poll channel");

    poll
      .register(
        &acknack_receiver,
        ACKNACK_MESSAGE_TO_LOCAL_WRITER_TOKEN,
        Ready::readable(),
        PollOpt::edge(),
      )
      .expect("Failed to register AckNack submessage sending from MessageReceiver to DPEventLoop");

    poll
      .register(
        &discovery_update_notification_receiver,
        DISCOVERY_UPDATE_NOTIFICATION_TOKEN,
        Ready::readable(),
        PollOpt::edge(),
      )
      .expect("Failed to register reader update notification.");

    // port number 0 means OS chooses an available port number.
    let udp_sender = UDPSender::new(0).expect("UDPSender construction fail"); // TODO

    Self {
      domain_info,
      poll,
      ddscache,
      discovery_db,
      udp_listeners,
      udp_sender: Rc::new(udp_sender),
      message_receiver: MessageReceiver::new(
        participant_guid_prefix,
        acknack_sender,
        spdp_liveness_sender,
      ),
      add_reader_receiver,
      remove_reader_receiver,
      add_writer_receiver,
      remove_writer_receiver,
      stop_poll_receiver,
      writers: HashMap::new(),
      ack_nack_receiver: acknack_receiver,
      discovery_update_notification_receiver,
    }
  }

  pub fn event_loop(self) {
    let mut events = Events::with_capacity(16); // too small capacity just delays events to next poll
    let mut acknack_timer = mio_extras::timer::Timer::default();
    acknack_timer.set_timeout(PREEMPTIVE_ACKNACK_PERIOD, ());

    self
      .poll
      .register(
        &acknack_timer,
        DPEV_ACKNACK_TIMER_TOKEN,
        Ready::readable(),
        PollOpt::edge(),
      )
      .unwrap();
    let mut poll_alive = Instant::now();
    let mut ev_wrapper = self;
    loop {
      ev_wrapper
        .poll
        .poll(&mut events, Some(Duration::from_millis(2000)))
        .expect("Failed in waiting of poll.");

      // liveness watchdog
      let now = Instant::now();
      if now > poll_alive + Duration::from_secs(2) {
        debug!("Poll loop alive");
        poll_alive = now;
      }

      if events.is_empty() {
        debug!("dp_event_loop idling.");
      } else {
        for event in events.iter() {
          match EntityId::from_token(event.token()) {
            TokenDecode::FixedToken(fixed_token) => match fixed_token {
              STOP_POLL_TOKEN => {
                let _ = ev_wrapper.stop_poll_receiver.try_recv();
                // we are not really interested in the content
                info!("Stopping dp_event_loop");
                return;
              }
              DISCOVERY_LISTENER_TOKEN
              | DISCOVERY_MUL_LISTENER_TOKEN
              | USER_TRAFFIC_LISTENER_TOKEN
              | USER_TRAFFIC_MUL_LISTENER_TOKEN => {
                let udp_messages = ev_wrapper
                  .udp_listeners
                  .get_mut(&event.token())
                  .map_or_else(
                    || {
                      error!("No listener with token {:?}", &event.token());
                      vec![]
                    },
                    UDPListener::messages,
                  );
                for packet in udp_messages {
                  ev_wrapper.message_receiver.handle_received_packet(&packet);
                }
              }
              ADD_READER_TOKEN | REMOVE_READER_TOKEN => {
                ev_wrapper.handle_reader_action(&event);
              }
              ADD_WRITER_TOKEN | REMOVE_WRITER_TOKEN => {
                ev_wrapper.handle_writer_action(&event);
              }
              ACKNACK_MESSAGE_TO_LOCAL_WRITER_TOKEN => {
                ev_wrapper.handle_writer_acknack_action(&event);
              }
              DISCOVERY_UPDATE_NOTIFICATION_TOKEN => {
                while let Ok(dnt) = ev_wrapper.discovery_update_notification_receiver.try_recv() {
                  use DiscoveryNotificationType::*;
                  match dnt {
                    WriterUpdated {
                      discovered_writer_data,
                    } => ev_wrapper.remote_writer_discovered(&discovered_writer_data),

                    WriterLost { writer_guid } => ev_wrapper.remote_writer_lost(writer_guid),

                    ReaderUpdated {
                      discovered_reader_data,
                    } => ev_wrapper.remote_reader_discovered(&discovered_reader_data),

                    ReaderLost { reader_guid } => ev_wrapper.remote_reader_lost(reader_guid),

                    ParticipantUpdated { guid_prefix } => {
                      ev_wrapper.update_participant(guid_prefix);
                    }

                    ParticipantLost { guid_prefix } => {
                      ev_wrapper.remote_participant_lost(guid_prefix);
                    }

                    AssertTopicLiveliness {
                      writer_guid,
                      manual_assertion,
                    } => {
                      ev_wrapper
                        .writers
                        .get_mut(&writer_guid.entity_id)
                        .map(|w| w.handle_heartbeat_tick(manual_assertion));
                    }
                  }
                }
              }
              DPEV_ACKNACK_TIMER_TOKEN => {
                ev_wrapper.message_receiver.send_preemptive_acknacks();
                acknack_timer.set_timeout(PREEMPTIVE_ACKNACK_PERIOD, ());
              }

              fixed_unknown => {
                error!(
                  "Unknown event.token {:?} = 0x{:x?} , decoded as {:?}",
                  event.token(),
                  event.token().0,
                  fixed_unknown
                );
              }
            },

            // Commands/actions
            TokenDecode::Entity(eid) => {
              if eid.kind().is_reader() {
                ev_wrapper.message_receiver.reader_mut(eid).map_or_else(
                  || error!("Event for unknown reader {eid:?}"),
                  Reader::process_command,
                );
              } else if eid.kind().is_writer() {
                let local_readers = match ev_wrapper.writers.get_mut(&eid) {
                  None => {
                    error!("Event for unknown writer {eid:?}");
                    vec![]
                  }
                  Some(writer) => {
                    // Writer will record data to DDSCache and send it out.
                    writer.process_writer_command();
                    writer.local_readers()
                  }
                };
                // Notify local (same participant) readers that new data is available in the
                // cache.
                ev_wrapper
                  .message_receiver
                  .notify_data_to_readers(local_readers);
              } else {
                error!("Entity Event for unknown EntityKind {eid:?}");
              }
            }

            // Timed Actions
            TokenDecode::AltEntity(eid) => {
              if eid.kind().is_reader() {
                ev_wrapper.handle_reader_timed_event(eid);
              } else if eid.kind().is_writer() {
                ev_wrapper.handle_writer_timed_event(eid);
              } else {
                error!("AltEntity Event for unknown EntityKind {eid:?}");
              }
            }
          }
        } // for
      } // if
    } // loop
  } // fn

  fn handle_reader_action(&mut self, event: &Event) {
    match event.token() {
      ADD_READER_TOKEN => {
        trace!("add reader(s)");
        while let Ok(new_reader_ing) = self.add_reader_receiver.receiver.try_recv() {
          let timer = mio_extras::timer::Builder::default().num_slots(8).build();
          self
            .poll
            .register(
              &timer,
              new_reader_ing.alt_entity_token(),
              Ready::readable(),
              PollOpt::edge(),
            )
            .expect("Reader timer channel registration failed!");
          let mut new_reader = Reader::new(new_reader_ing, self.udp_sender.clone(), timer);

          // Non-timed action polling
          self
            .poll
            .register(
              &new_reader.data_reader_command_receiver,
              new_reader.entity_token(),
              Ready::readable(),
              PollOpt::edge(),
            )
            .expect("Reader command channel registration failed!!!");

          new_reader.set_requested_deadline_check_timer();
          trace!("Add reader: {:?}", new_reader);
          self.message_receiver.add_reader(new_reader);
        }
      }
      REMOVE_READER_TOKEN => {
        while let Ok(old_reader_guid) = self.remove_reader_receiver.receiver.try_recv() {
          if let Some(old_reader) = self.message_receiver.remove_reader(old_reader_guid) {
            self
              .poll
              .deregister(&old_reader.timed_event_timer)
              .unwrap_or_else(|e| error!("Cannot deregister Reader timed_event_timer: {e:?}"));
            self
              .poll
              .deregister(&old_reader.data_reader_command_receiver)
              .unwrap_or_else(|e| {
                error!("Cannot deregister data_reader_command_receiver: {e:?}");
              });
          } else {
            warn!("Tried to remove nonexistent Reader {old_reader_guid:?}");
          }
        }
      }
      _ => {}
    }
  }

  fn handle_writer_action(&mut self, event: &Event) {
    match event.token() {
      ADD_WRITER_TOKEN => {
        while let Ok(new_writer_ingredients) = self.add_writer_receiver.receiver.try_recv() {
          let timer = mio_extras::timer::Builder::default().num_slots(8).build();
          self
            .poll
            .register(
              &timer,
              new_writer_ingredients.alt_entity_token(),
              Ready::readable(),
              PollOpt::edge(),
            )
            .expect("Writer heartbeat timer channel registration failed!!");
          let new_writer = Writer::new(new_writer_ingredients, self.udp_sender.clone(), timer);

          self
            .poll
            .register(
              &new_writer.writer_command_receiver,
              new_writer.entity_token(),
              Ready::readable(),
              PollOpt::edge(),
            )
            .expect("Writer command channel registration failed!!");
          self.writers.insert(new_writer.guid().entity_id, new_writer);
        }
      }
      REMOVE_WRITER_TOKEN => {
        while let Ok(writer_guid) = &self.remove_writer_receiver.receiver.try_recv() {
          if let Some(w) = self.writers.remove(&writer_guid.entity_id) {
            self
              .poll
              .deregister(&w.writer_command_receiver)
              .unwrap_or_else(|e| error!("Deregister fail (writer command rec) {e:?}"));
            self
              .poll
              .deregister(&w.timed_event_timer)
              .unwrap_or_else(|e| error!("Deregister fail (writer timer) {e:?}"));
          }
        }
      }
      other => error!("Expected writer action token, got {:?}", other),
    }
  }

  /// Writer timed events can be heatrbeats or cache cleaning events.
  /// events are distinguished by TimerMessageType which is send via mio
  /// channel. Channel token in
  fn handle_writer_timed_event(&mut self, entity_id: EntityId) {
    if let Some(writer) = self.writers.get_mut(&entity_id) {
      writer.handle_timed_event();
    } else {
      error!("Writer was not found with {:?}", entity_id);
    }
  }

  fn handle_reader_timed_event(&mut self, entity_id: EntityId) {
    if let Some(reader) = self.message_receiver.reader_mut(entity_id) {
      reader.handle_timed_event();
    } else {
      error!("Reader was not found with {:?}", entity_id);
    }
  }

  fn handle_writer_acknack_action(&mut self, _event: &Event) {
    while let Ok((acknack_sender_prefix, acknack_submessage)) = self.ack_nack_receiver.try_recv() {
      let writer_guid = GUID::new_with_prefix_and_id(
        self.domain_info.domain_participant_guid.prefix,
        acknack_submessage.writer_id(),
      );
      if let Some(found_writer) = self.writers.get_mut(&writer_guid.entity_id) {
        if found_writer.is_reliable() {
          found_writer.handle_ack_nack(acknack_sender_prefix, &acknack_submessage);
        }
      } else {
        // Note: when testing against FastDDS Shapes demo, this else branch is
        // repeatedly triggered. The resulting log entry contains the following
        // EntityId: {[0, 3, 0] EntityKind::WRITER_NO_KEY_BUILT_IN}.
        // In this case a writer cannot be found, because FastDDS sends
        // pre-emptive acknacks about a built-in topic defined in DDS Xtypes
        // specification, which RustDDS does not implement. So even though the acknack
        // cannot be handled, it is not a problem in this case.
        debug!(
          "Couldn't handle acknack/nackfrag! Did not find local RTPS writer with GUID: {:x?}",
          writer_guid
        );
        continue;
      }
    }
  }

  fn update_participant(&mut self, participant_guid_prefix: GuidPrefix) {
    debug!(
      "update_participant {:?} myself={}",
      participant_guid_prefix,
      participant_guid_prefix == self.domain_info.domain_participant_guid.prefix
    );

    {
      let db = self.discovery_db.read().unwrap();
      // new Remote Participant discovered
      let discovered_participant =
        if let Some(dpd) = db.find_participant_proxy(participant_guid_prefix) {
          dpd
        } else {
          error!("Participant was updated, but DB does not have it. Strange.");
          return;
        };

      for (writer_eid, reader_eid, endpoint) in &[
        (
          EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER, // SPDP
          EntityId::SPDP_BUILTIN_PARTICIPANT_READER,
          BuiltinEndpointSet::PARTICIPANT_DETECTOR,
        ),
        (
          EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, // SEDP ...
          EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_READER,
          BuiltinEndpointSet::SUBSCRIPTIONS_DETECTOR,
        ),
        (
          EntityId::SEDP_BUILTIN_PUBLICATIONS_WRITER,
          EntityId::SEDP_BUILTIN_PUBLICATIONS_READER,
          BuiltinEndpointSet::PUBLICATIONS_DETECTOR,
        ),
        (
          EntityId::SEDP_BUILTIN_TOPIC_WRITER,
          EntityId::SEDP_BUILTIN_TOPIC_READER,
          BuiltinEndpointSet::TOPICS_DETECTOR,
        ),
        (
          EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER,
          EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_READER,
          BuiltinEndpointSet::PARTICIPANT_MESSAGE_DATA_READER,
        ),
      ] {
        if let Some(writer) = self.writers.get_mut(writer_eid) {
          debug!("update_discovery_writer - {:?}", writer.topic_name());
          let mut qos = Discovery::subscriber_qos();
          // special case by RTPS 2.3 spec Section
          // "8.4.13.3 BuiltinParticipantMessageWriter and
          // BuiltinParticipantMessageReader QoS"
          if *reader_eid == EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_READER
            && discovered_participant
              .builtin_endpoint_qos
              .map_or(false, |beq| beq.is_best_effort())
          {
            qos.reliability = Some(policy::Reliability::BestEffort);
          };

          if discovered_participant
            .available_builtin_endpoints
            .contains(*endpoint)
          {
            let mut reader_proxy = discovered_participant.as_reader_proxy(true, Some(*reader_eid));

            if *writer_eid == EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER {
              // Simple Participant Discovery Protocol (SPDP) writer is special,
              // different from SEDP writers
              qos = Discovery::create_spdp_participant_qos(); // different QoS
                                                              // adding a multicast reader
              reader_proxy.remote_reader_guid = GUID::new_with_prefix_and_id(
                GuidPrefix::UNKNOWN,
                EntityId::SPDP_BUILTIN_PARTICIPANT_READER,
              );

              // reader_proxy.multicast_locator_list =
              // get_local_multicast_locators(
              //   spdp_well_known_multicast_port(self.domain_info.domain_id),
              // );
            }
            // common processing for SPDP and SEDP
            writer.update_reader_proxy(&reader_proxy, &qos);
            debug!(
              "update_discovery writer - endpoint {:?} - {:?}",
              endpoint, discovered_participant.participant_guid
            );
          }

          writer.notify_new_data_to_all_readers();
        }
      }
      // update local readers.
      // list to be looped over is the same as above, but now
      // EntityIds are for announcers
      for (writer_eid, reader_eid, endpoint) in &[
        (
          EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER, // SPDP
          EntityId::SPDP_BUILTIN_PARTICIPANT_READER,
          BuiltinEndpointSet::PARTICIPANT_ANNOUNCER,
        ),
        (
          EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, // SEDP ...
          EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_READER,
          BuiltinEndpointSet::PUBLICATIONS_ANNOUNCER,
        ),
        (
          EntityId::SEDP_BUILTIN_PUBLICATIONS_WRITER,
          EntityId::SEDP_BUILTIN_PUBLICATIONS_READER,
          BuiltinEndpointSet::PUBLICATIONS_ANNOUNCER,
        ),
        (
          EntityId::SEDP_BUILTIN_TOPIC_WRITER,
          EntityId::SEDP_BUILTIN_TOPIC_READER,
          BuiltinEndpointSet::TOPICS_ANNOUNCER,
        ),
        (
          EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER,
          EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_READER,
          BuiltinEndpointSet::PARTICIPANT_MESSAGE_DATA_WRITER,
        ),
      ] {
        if let Some(reader) = self.message_receiver.available_readers.get_mut(reader_eid) {
          debug!("try update_discovery_reader - {:?}", reader.topic_name());
          let qos = if *reader_eid == EntityId::SPDP_BUILTIN_PARTICIPANT_READER {
            Discovery::create_spdp_participant_qos()
          } else {
            Discovery::publisher_qos()
          };
          let wp = discovered_participant.as_writer_proxy(true, Some(*writer_eid));

          if discovered_participant
            .available_builtin_endpoints
            .contains(*endpoint)
          {
            reader.update_writer_proxy(wp, &qos);
            debug!(
              "update_discovery_reader - endpoint {:?} - {:?}",
              *endpoint, discovered_participant.participant_guid
            );
          }
        }
      } // for
    } // if
    debug!(
      "update_participant - finished for {:?}",
      participant_guid_prefix
    );
  } // fn

  fn remote_participant_lost(&mut self, participant_guid_prefix: GuidPrefix) {
    info!(
      "remote_participant_lost guid_prefix={:?}",
      &participant_guid_prefix
    );
    // Discovery has already removed Participant from Discovery DB
    // Now we have to remove any ReaderProxies and WriterProxies belonging
    // to that participant, so that we do not send messages to them anymore.

    for writer in self.writers.values_mut() {
      writer.participant_lost(participant_guid_prefix);
    }

    for reader in self.message_receiver.available_readers.values_mut() {
      reader.participant_lost(participant_guid_prefix);
    }
  }

  fn remote_reader_discovered(&mut self, drd: &DiscoveredReaderData) {
    for writer in self.writers.values_mut() {
      if drd.subscription_topic_data.topic_name() == writer.topic_name() {
        // // see if the participant has published a QoS for the topic
        // // If yes, we take that as a basis QoS
        // let topic_qos = self.discovery_db.read().unwrap()
        //   .get_topic_for_participant(&drd.subscription_topic_data.topic_name(),
        //     drd.reader_proxy.remote_reader_guid.prefix )
        //   .map( |dtd| dtd.topic_data.qos() );
        // let requested_qos = topic_qos
        //   .unwrap_or_else( QosPolicies::default )
        //   .modify_by( &drd.subscription_topic_data.qos() );
        let requested_qos = drd.subscription_topic_data.qos();
        writer.update_reader_proxy(
          &RtpsReaderProxy::from_discovered_reader_data(drd, &[], &[]),
          &requested_qos,
        );
      }
    }
  }

  fn remote_reader_lost(&mut self, reader_guid: GUID) {
    for writer in self.writers.values_mut() {
      writer.reader_lost(reader_guid);
    }
  }

  fn remote_writer_discovered(&mut self, dwd: &DiscoveredWriterData) {
    // update writer proxies in local readers
    for reader in self.message_receiver.available_readers.values_mut() {
      if &dwd.publication_topic_data.topic_name == reader.topic_name() {
        let offered_qos = dwd.publication_topic_data.qos();
        // // see if the participant has published a QoS for the topic
        // // If yes, we take that as a basis QoS
        // let topic_qos = self.discovery_db.read().unwrap()
        //   .get_topic_for_participant(&dwd.publication_topic_data.topic_name,
        //     dwd.writer_proxy.remote_writer_guid.prefix )
        //   .map( |dtd| dtd.topic_data.qos() );
        // let offered_qos = topic_qos
        //   .unwrap_or_else( QosPolicies::default )
        //   .modify_by( &dwd.publication_topic_data.qos() );

        reader.update_writer_proxy(
          RtpsWriterProxy::from_discovered_writer_data(dwd, &[], &[]),
          &offered_qos,
        );
      }
    }
    // notify DDSCache to create topic if it does not exist yet
    match self.ddscache.write() {
      Ok(mut ddsc) => {
        let ptd = &dwd.publication_topic_data;
        ddsc.add_new_topic(
          ptd.topic_name.clone(),
          TypeDesc::new(ptd.type_name.clone()),
          &ptd.qos(),
        );
      }

      _ => panic!("DDSCache is poisoned"),
    }
  }

  fn remote_writer_lost(&mut self, writer_guid: GUID) {
    for reader in self.message_receiver.available_readers.values_mut() {
      reader.remove_writer_proxy(writer_guid);
    }
  }
}

// -----------------------------------------------------------
// -----------------------------------------------------------
// -----------------------------------------------------------

#[cfg(test)]
mod tests {
  use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
  };

  use mio_06::{PollOpt, Ready};
  use mio_extras::channel as mio_channel;

  use super::*;
  use crate::{
    dds::{
      qos::QosPolicies,
      statusevents::{sync_status_channel, DataReaderStatus},
      typedesc::TypeDesc,
      with_key::simpledatareader::ReaderCommand,
    },
    mio_source,
    structure::dds_cache::DDSCache,
    SequenceNumber,
  };

  //#[test]
  // TODO: Investigate why this fails in the github CI pipeline
  // Then re-enable this test.
  #[allow(dead_code)]
  fn dpew_add_and_remove_readers() {
    // Test sending 'add reader' and 'remove reader' commands to DP event loop
    // TODO: There are no assertions in this test case. Does in actually test
    // anything?

    // Create DP communication channels
    let (sender_add_reader, receiver_add) = mio_channel::channel::<ReaderIngredients>();
    let (sender_remove_reader, receiver_remove) = mio_channel::channel::<GUID>();

    let (_add_writer_sender, add_writer_receiver) = mio_channel::channel();
    let (_remove_writer_sender, remove_writer_receiver) = mio_channel::channel();

    let (_stop_poll_sender, stop_poll_receiver) = mio_channel::channel();

    let (_discovery_update_notification_sender, discovery_update_notification_receiver) =
      mio_channel::channel();
    let (spdp_liveness_sender, _spdp_liveness_receiver) = mio_channel::sync_channel(8);

    let dds_cache = Arc::new(RwLock::new(DDSCache::new()));
    let (discovery_db_event_sender, _discovery_db_event_receiver) =
      mio_channel::sync_channel::<()>(4);

    let discovery_db = Arc::new(RwLock::new(DiscoveryDB::new(
      GUID::new_participant_guid(),
      discovery_db_event_sender,
    )));

    let domain_info = DomainInfo {
      domain_participant_guid: GUID::default(),
      domain_id: 0,
      participant_id: 0,
    };

    let (sender_stop, receiver_stop) = mio_channel::channel::<i32>();

    let dds_cache_clone = dds_cache.clone();
    // Start event loop
    let child = thread::spawn(move || {
      let dp_event_loop = DPEventLoop::new(
        domain_info,
        HashMap::new(),
        dds_cache_clone,
        discovery_db,
        GuidPrefix::default(),
        TokenReceiverPair {
          token: ADD_READER_TOKEN,
          receiver: receiver_add,
        },
        TokenReceiverPair {
          token: REMOVE_READER_TOKEN,
          receiver: receiver_remove,
        },
        TokenReceiverPair {
          token: ADD_WRITER_TOKEN,
          receiver: add_writer_receiver,
        },
        TokenReceiverPair {
          token: REMOVE_WRITER_TOKEN,
          receiver: remove_writer_receiver,
        },
        stop_poll_receiver,
        discovery_update_notification_receiver,
        spdp_liveness_sender,
      );
      dp_event_loop
        .poll
        .register(
          &receiver_stop,
          STOP_POLL_TOKEN,
          Ready::readable(),
          PollOpt::edge(),
        )
        .expect("Failed to register receivers.");
      dp_event_loop.event_loop();
    });

    // Create a topic cache
    let topic_cache = dds_cache.write().unwrap().add_new_topic(
      "test".to_string(),
      TypeDesc::new("test_type".to_string()),
      &QosPolicies::qos_none(),
    );

    let last_read_sequence_number_ref =
      Arc::new(Mutex::new(BTreeMap::<GUID, SequenceNumber>::new()));

    let num_of_readers = 3;

    // Send some 'add reader' commands
    let mut reader_guids = Vec::new();
    for i in 0..num_of_readers {
      let new_guid = GUID::default();

      // Create mechanisms for notifications, statuses & commands
      let (notification_sender, _notification_receiver) = mio_channel::sync_channel::<()>(100);
      let (_notification_event_source, notification_event_sender) =
        mio_source::make_poll_channel().unwrap();
      let data_reader_waker = Arc::new(Mutex::new(None));

      let (status_sender, _status_receiver) = sync_status_channel::<DataReaderStatus>(4).unwrap();

      let (_reader_command_sender, reader_command_receiver) =
        mio_channel::sync_channel::<ReaderCommand>(10);

      let new_reader_ing = ReaderIngredients {
        guid: new_guid,
        notification_sender,
        status_sender,
        topic_cache_handle: topic_cache.clone(),
        last_read_sequence_number_ref: last_read_sequence_number_ref.clone(),
        topic_name: "test".to_string(),
        qos_policy: QosPolicies::qos_none(),
        data_reader_command_receiver: reader_command_receiver,
        data_reader_waker: data_reader_waker.clone(),
        poll_event_sender: notification_event_sender,
      };

      reader_guids.push(new_reader_ing.guid);
      info!("\nSent reader number {}: {:?}\n", i, &new_reader_ing);
      sender_add_reader.send(new_reader_ing).unwrap();
      std::thread::sleep(Duration::new(0, 100));
    }

    // Send a command to remove the second reader
    info!("\nremoving the second\n");
    let some_guid = reader_guids[1];
    sender_remove_reader.send(some_guid).unwrap();
    std::thread::sleep(Duration::new(0, 100));

    info!("\nsending end token\n");
    sender_stop.send(0).unwrap();
    child.join().unwrap();
  }

  // TODO: Rewrite / remove this test - all asserts in it use
  // DataReader::get_requested_deadline_missed_status which is
  // currently commented out

  // #[test]
  // fn dpew_test_reader_commands() {
  //   let somePolicies = QosPolicies {
  //     durability: None,
  //     presentation: None,
  //     deadline: Some(Deadline(DurationDDS::from_millis(500))),
  //     latency_budget: None,
  //     ownership: None,
  //     liveliness: None,
  //     time_based_filter: None,
  //     reliability: None,
  //     destination_order: None,
  //     history: None,
  //     resource_limits: None,
  //     lifespan: None,
  //   };
  //   let dp = DomainParticipant::new(0).expect("Failed to create
  // participant");   let sub = dp.create_subscriber(&somePolicies).unwrap();

  //   let topic_1 = dp
  //     .create_topic("TOPIC_1", "something", &somePolicies,
  // TopicKind::WithKey)     .unwrap();
  //   let _topic_2 = dp
  //     .create_topic("TOPIC_2", "something", &somePolicies,
  // TopicKind::WithKey)     .unwrap();
  //   let _topic_3 = dp
  //     .create_topic("TOPIC_3", "something", &somePolicies,
  // TopicKind::WithKey)     .unwrap();

  //   // Adding readers
  //   let (sender_add_reader, receiver_add) = mio_channel::channel::<Reader>();
  //   let (_sender_remove_reader, receiver_remove) =
  // mio_channel::channel::<GUID>();

  //   let (_add_writer_sender, add_writer_receiver) = mio_channel::channel();
  //   let (_remove_writer_sender, remove_writer_receiver) =
  // mio_channel::channel();

  //   let (_stop_poll_sender, stop_poll_receiver) = mio_channel::channel();

  //   let (_discovery_update_notification_sender,
  // discovery_update_notification_receiver) =     mio_channel::channel();

  //   let ddshc = Arc::new(RwLock::new(DDSCache::new()));
  //   let discovery_db = Arc::new(RwLock::new(DiscoveryDB::new()));

  //   let domain_info = DomainInfo {
  //     domain_participant_guid: GUID::default(),
  //     domain_id: 0,
  //     participant_id: 0,
  //   };

  //   let dp_event_loop = DPEventLoop::new(
  //     domain_info,
  //     HashMap::new(),
  //     ddshc,
  //     discovery_db,
  //     GuidPrefix::default(),
  //     TokenReceiverPair {
  //       token: ADD_READER_TOKEN,
  //       receiver: receiver_add,
  //     },
  //     TokenReceiverPair {
  //       token: REMOVE_READER_TOKEN,
  //       receiver: receiver_remove,
  //     },
  //     TokenReceiverPair {
  //       token: ADD_WRITER_TOKEN,
  //       receiver: add_writer_receiver,
  //     },
  //     TokenReceiverPair {
  //       token: REMOVE_WRITER_TOKEN,
  //       receiver: remove_writer_receiver,
  //     },
  //     stop_poll_receiver,
  //     discovery_update_notification_receiver,
  //   );

  //   let (sender_stop, receiver_stop) = mio_channel::channel::<i32>();
  //   dp_event_loop
  //     .poll
  //     .register(
  //       &receiver_stop,
  //       STOP_POLL_TOKEN,
  //       Ready::readable(),
  //       PollOpt::edge(),
  //     )
  //     .expect("Failed to register receivers.");

  //   let child = thread::spawn(move ||
  // DPEventLoop::event_loop(dp_event_loop));

  //   //TODO IF THIS IS SET TO 1 TEST SUCCEEDS
  //   let n = 1;

  //   let mut reader_guids = Vec::new();
  //   let mut data_readers: Vec<DataReader<RandomData,
  // CDRDeserializerAdapter<RandomData>>> = vec![];   let _topics: Vec<Topic>
  // = vec![];   for i in 0..n {
  //     //topics.push(topic);
  //     let new_guid = GUID::default();

  //     let (send, _rec) = mio_channel::sync_channel::<()>(100);
  //     let (status_sender, status_receiver_DataReader) =
  //       mio_extras::channel::sync_channel::<DataReaderStatus>(1000);
  //     let (reader_commander, reader_command_receiver) =
  //       mio_extras::channel::sync_channel::<ReaderCommand>(1000);

  //     let mut new_reader = Reader::new(
  //       new_guid,
  //       send,
  //       status_sender,
  //       Arc::new(RwLock::new(DDSCache::new())),
  //       "test".to_string(),
  //       QosPolicies::qos_none(),
  //       reader_command_receiver,
  //     );

  //     let somePolicies = QosPolicies {
  //       durability: None,
  //       presentation: None,
  //       deadline: Some(Deadline(DurationDDS::from_millis(50))),
  //       latency_budget: None,
  //       ownership: None,
  //       liveliness: None,
  //       time_based_filter: None,
  //       reliability: None,
  //       destination_order: None,
  //       history: None,
  //       resource_limits: None,
  //       lifespan: None,
  //     };

  //     let mut datareader = sub
  //       .create_datareader::<RandomData, CDRDeserializerAdapter<RandomData>>(
  //         topic_1.clone(),
  //         Some(somePolicies.clone()),
  //       )
  //       .unwrap();

  //     datareader.set_status_change_receiver(status_receiver_DataReader);
  //     datareader.set_reader_commander(reader_commander);
  //     data_readers.push(datareader);

  //     //new_reader.set_qos(&somePolicies).unwrap();
  //     new_reader.matched_writer_add(GUID::default(),
  // EntityId::UNKNOWN, vec![], vec![]);     reader_guids.
  // push(new_reader.guid().clone());     info!("\nSent reader number {}:
  // {:?}\n", i, &new_reader);     sender_add_reader.send(new_reader).
  // unwrap();     std::thread::sleep(Duration::from_millis(100));
  //   }
  //   thread::sleep(Duration::from_millis(100));

  //   let status = data_readers
  //     .get_mut(0)
  //     .unwrap()
  //     .get_requested_deadline_missed_status();
  //   info!("Received status change: {:?}", status);
  //   assert_eq!(
  //     status.unwrap(),
  //     Some(RequestedDeadlineMissedStatus::from_count(
  //       CountWithChange::start_from(3, 3)
  //     )),
  //   );
  //   thread::sleep(Duration::from_millis(150));

  //   let status2 = data_readers
  //     .get_mut(0)
  //     .unwrap()
  //     .get_requested_deadline_missed_status();
  //   info!("Received status change: {:?}", status2);
  //   assert_eq!(
  //     status2.unwrap(),
  //     Some(RequestedDeadlineMissedStatus::from_count(
  //       CountWithChange::start_from(6, 3)
  //     ))
  //   );

  //   let status3 = data_readers
  //     .get_mut(0)
  //     .unwrap()
  //     .get_requested_deadline_missed_status();
  //   info!("Received status change: {:?}", status3);
  //   assert_eq!(
  //     status3.unwrap(),
  //     Some(RequestedDeadlineMissedStatus::from_count(
  //       CountWithChange::start_from(6, 0)
  //     ))
  //   );

  //   thread::sleep(Duration::from_millis(50));

  //   let status4 = data_readers
  //     .get_mut(0)
  //     .unwrap()
  //     .get_requested_deadline_missed_status();
  //   info!("Received status change: {:?}", status4);
  //   assert_eq!(
  //     status4.unwrap(),
  //     Some(RequestedDeadlineMissedStatus::from_count(
  //       CountWithChange::start_from(7, 1)
  //     ))
  //   );

  //   info!("\nsending end token\n");
  //   sender_stop.send(0).unwrap();
  //   child.join().unwrap();
  // }
}
