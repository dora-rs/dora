use std::{collections::BTreeMap, time::Instant};

use chrono::Utc;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::{
    participant::DomainParticipant,
    qos::HasQoSPolicy,
    topic::{Topic, TopicDescription},
  },
  rtps::{
    reader::ReaderIngredients, rtps_reader_proxy::RtpsReaderProxy,
    rtps_writer_proxy::RtpsWriterProxy,
  },
  structure::{
    duration::Duration,
    entity::RTPSEntity,
    guid::{EntityId, GuidPrefix, GUID},
  },
};
use super::{
  sedp_messages::{
    DiscoveredReaderData, DiscoveredTopicData, DiscoveredWriterData, ParticipantMessageData,
    ReaderProxy, SubscriptionBuiltinTopicData, TopicBuiltinTopicData, WriterProxy,
  },
  spdp_participant_data::SpdpDiscoveredParticipantData,
};

// If remote participant does not specify lease duration, how long silence
// until we pronounce it dead.
const DEFAULT_PARTICIPANT_LEASE_DURATION: Duration = Duration::from_secs(60);

// How much longer to wait than lease duration before pronouncing lost.
const PARTICIPANT_LEASE_DURATION_TOLERANCE: Duration = Duration::from_secs(0);

// TODO: Let DiscoveryDB itself become thread-safe and support smaller-scope
// lock
pub(crate) struct DiscoveryDB {
  my_guid: GUID,
  participant_proxies: BTreeMap<GuidPrefix, SpdpDiscoveredParticipantData>,
  participant_last_life_signs: BTreeMap<GuidPrefix, Instant>,

  // local writer proxies for topics (topic name acts as key)
  local_topic_writers: BTreeMap<GUID, DiscoveredWriterData>,
  // local reader proxies for topics (topic name acts as key)
  local_topic_readers: BTreeMap<GUID, DiscoveredReaderData>,

  // remote readers and writers (via discovery)
  external_topic_readers: BTreeMap<GUID, DiscoveredReaderData>,
  external_topic_writers: BTreeMap<GUID, DiscoveredWriterData>,

  // These are "attic" storages for readers and writers whose participant
  // was lost due to time-out. If we have a
  external_topic_readers_attic: BTreeMap<GUID, DiscoveredReaderData>,
  external_topic_writers_attic: BTreeMap<GUID, DiscoveredWriterData>,

  // Database of topic updates:
  // Outer level key is topic name
  // Inner key is topic data sender.
  topics: BTreeMap<String, BTreeMap<GuidPrefix, (DiscoveredVia, DiscoveredTopicData)>>,

  // sender for notifying (potential) waiters in participant.find_topic() call
  topic_updated_sender: mio_extras::channel::SyncSender<()>,
}

// How did we discover this topic
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub(crate) enum DiscoveredVia {
  Topic,        // explicitly, via the topic topic (does this actually occur?)
  Publication,  // we discovered there is a writer on this topic
  Subscription, // we discovered a reader on this topic
}

fn move_by_guid_prefix<D>(
  guid_prefix: GuidPrefix,
  from: &mut BTreeMap<GUID, D>,
  to: &mut BTreeMap<GUID, D>,
) {
  let to_move: Vec<GUID> = from.range(guid_prefix.range()).map(|(g, _)| *g).collect();
  for guid in to_move {
    from.remove(&guid).map(|d| to.insert(guid, d));
  }
}

impl DiscoveryDB {
  pub fn new(my_guid: GUID, topic_updated_sender: mio_extras::channel::SyncSender<()>) -> Self {
    Self {
      my_guid,
      participant_proxies: BTreeMap::new(),
      participant_last_life_signs: BTreeMap::new(),
      local_topic_writers: BTreeMap::new(),
      local_topic_readers: BTreeMap::new(),
      external_topic_readers: BTreeMap::new(),
      external_topic_writers: BTreeMap::new(),
      external_topic_readers_attic: BTreeMap::new(),
      external_topic_writers_attic: BTreeMap::new(),
      topics: BTreeMap::new(),
      topic_updated_sender,
    }
  }

  // Returns if participant was previously unknown
  pub fn update_participant(&mut self, data: &SpdpDiscoveredParticipantData) -> bool {
    debug!("update_participant: {:?}", &data);
    let guid = data.participant_guid;

    // sanity check
    if guid.entity_id != EntityId::PARTICIPANT {
      error!(
        "Discovered participant GUID entity_id is not for participant: {:?}",
        guid
      );
      // Maybe we should discard the participant here?
      return false;
    }

    // We allow discovery to discover self, since our discovery readers
    // will receive our own announcements via broadcast. If we do not recognize
    // our own participant, there is confusion about unknown writers on the
    // discovery topics.
    //
    // if guid == self.my_guid {
    //   debug!("DiscoveryDB discovered self. Skipping.");
    //   return
    // }

    let mut new_participant = false;
    if self.participant_proxies.get(&guid.prefix).is_none() {
      info!("New remote participant: {:?}", &data);
      new_participant = true;
      if guid == self.my_guid {
        info!(
          "Remote participant {:?} is myself, but some reflection is good.",
          guid
        );
        new_participant = false;
      }

      move_by_guid_prefix(
        guid.prefix,
        &mut self.external_topic_readers_attic,
        &mut self.external_topic_readers,
      );
      move_by_guid_prefix(
        guid.prefix,
        &mut self.external_topic_writers_attic,
        &mut self.external_topic_writers,
      );
    }
    // actual work here:
    self.participant_proxies.insert(guid.prefix, data.clone());
    self
      .participant_last_life_signs
      .insert(guid.prefix, Instant::now());

    new_participant
  }

  pub fn participant_is_alive(&mut self, guid_prefix: GuidPrefix) {
    if let Some(ts) = self.participant_last_life_signs.get_mut(&guid_prefix) {
      let now = Instant::now();
      if now.duration_since(*ts) > std::time::Duration::from_secs(1) {
        debug!(
          "Participant alive update for {:?}, but no full update.",
          guid_prefix
        );
      }
      *ts = now;
    } else {
      info!("Participant alive update for unknown {:?}. This is normal, if the message does not repeat.", guid_prefix);
    }
  }

  // active_disposal means that we received a discovery message announcing the
  // disposal of the participant. active_disposal=false means that the
  // participant timed out.
  pub fn remove_participant(&mut self, guid_prefix: GuidPrefix, active_disposal: bool) {
    info!("removing participant {:?}", guid_prefix);
    self.participant_proxies.remove(&guid_prefix);
    self.participant_last_life_signs.remove(&guid_prefix);

    if active_disposal {
      self.remove_topic_reader_with_prefix(guid_prefix);
      self.remove_topic_writer_with_prefix(guid_prefix);
    } else {
      // move to attic
      move_by_guid_prefix(
        guid_prefix,
        &mut self.external_topic_readers,
        &mut self.external_topic_readers_attic,
      );
      move_by_guid_prefix(
        guid_prefix,
        &mut self.external_topic_writers,
        &mut self.external_topic_writers_attic,
      );
    }
  }

  pub fn find_participant_proxy(
    &self,
    guid_prefix: GuidPrefix,
  ) -> Option<&SpdpDiscoveredParticipantData> {
    self.participant_proxies.get(&guid_prefix)
  }

  fn remove_topic_reader_with_prefix(&mut self, guid_prefix: GuidPrefix) {
    // TODO: Implement this using .drain_filter() in BTreeMap once it lands in
    // stable.
    let to_remove: Vec<GUID> = self
      .external_topic_readers
      .range(guid_prefix.range())
      .map(|(g, _)| *g)
      .collect();
    for guid in to_remove {
      self.external_topic_readers.remove(&guid);
    }
  }

  pub fn remove_topic_reader(&mut self, guid: GUID) {
    info!("remove_topic_reader {:?}", guid);
    self.external_topic_readers.remove(&guid);
  }

  fn remove_topic_writer_with_prefix(&mut self, guid_prefix: GuidPrefix) {
    // TODO: Implement this using .drain_filter() in BTreeMap once it lands in
    // stable.
    let to_remove: Vec<GUID> = self
      .external_topic_writers
      .range(guid_prefix.range())
      .map(|(g, _)| *g)
      .collect();
    for guid in to_remove {
      self.external_topic_writers.remove(&guid);
    }
  }

  pub fn remove_topic_writer(&mut self, guid: GUID) {
    self.external_topic_writers.remove(&guid);
  }

  // Delete participant proxies, if we have not heard of them within
  // lease_duration
  pub fn participant_cleanup(&mut self) -> Vec<GuidPrefix> {
    let inow = Instant::now();

    let mut to_remove = Vec::new();
    // TODO: We are not cleaning up liast_life_signs table, but that should not be a
    // problem, except for a slight memory leak.
    for (&guid, sp) in &self.participant_proxies {
      let lease_duration = sp
        .lease_duration
        .unwrap_or(DEFAULT_PARTICIPANT_LEASE_DURATION);
      // let lease_duration = lease_duration + lease_duration; // double it
      match self.participant_last_life_signs.get(&guid) {
        Some(&last_life) => {
          // keep, if duration not exceeded
          let elapsed = Duration::from_std(inow.duration_since(last_life));
          if elapsed <= lease_duration + PARTICIPANT_LEASE_DURATION_TOLERANCE {
            // No timeout yet, we keep this, so do nothing.
          } else {
            info!("participant cleanup - deleting participant proxy {:?}. lease_duration = {:?} elapsed = {:?}",
                  guid, lease_duration, elapsed);
            to_remove.push(guid);
          }
        }
        None => {
          error!("Participant {:?} not in last_life_signs table?", guid);
        }
      } // match
    } // for
    for guid in &to_remove {
      self.remove_participant(*guid, false); // false = removed due to timeout
    }
    to_remove
  }

  fn topic_has_writers_or_readers(&self, topic_name: &str) -> bool {
    // TODO: This entire function has silly implementation.
    // We should really have a separate map from Topic to Readers & Writers
    if self
      .local_topic_readers
      .iter()
      .any(|(_, p)| p.subscription_topic_data.topic_name() == topic_name)
    {
      return true;
    }

    if self
      .local_topic_writers
      .iter()
      .any(|(_, p)| p.publication_topic_data.topic_name == topic_name)
    {
      return true;
    }

    if self
      .external_topic_readers
      .values()
      .any(|p| p.subscription_topic_data.topic_name() == topic_name)
    {
      return true;
    }

    if self
      .external_topic_writers
      .values()
      .any(|p| p.publication_topic_data.topic_name == topic_name)
    {
      return true;
    }

    false
  }

  pub fn topic_cleanup(&mut self) {
    // removing topics that have no readers or writers
    let dead_topics: Vec<_> = self
      .topics
      .keys()
      .filter(|tn| !self.topic_has_writers_or_readers(tn))
      .cloned()
      .collect();
    for dt in &dead_topics {
      self.topics.remove(dt);
    }
  }

  pub fn update_local_topic_writer(&mut self, writer: DiscoveredWriterData) {
    self
      .local_topic_writers
      .insert(writer.writer_proxy.remote_writer_guid, writer);
  }

  pub fn remove_local_topic_writer(&mut self, guid: GUID) {
    self.local_topic_writers.remove(&guid);
  }

  // TODO: This is silly. Returns one of the parameters cloned, or None
  // TODO: Why are we here checking if discovery db already has this? What about
  // reader proxies in writers?

  // Here we make updates when a new Reader is found via Discovery.
  // * It may be one of our own.
  // * We may have seen it before.
  //
  // In case DiscoveredReaderData does not have locators, we try to get
  // them from the remote participant.
  //
  // The topic is updated to the topics table.
  pub fn update_subscription(&mut self, data: &DiscoveredReaderData) -> DiscoveredReaderData {
    let guid = data.reader_proxy.remote_reader_guid;

    self.external_topic_readers.insert(guid, data.clone());

    // fill in the default locators from participant, in case DRD did not provide
    // any
    let default_locator_lists = self
      .find_participant_proxy(guid.prefix)
      .map(|pp| {
        debug!("Added participant locators to Reader {:?}", guid);
        (
          pp.default_unicast_locators.clone(),
          pp.default_multicast_locators.clone(),
        )
      })
      .unwrap_or_else(|| {
        if guid.prefix != GuidPrefix::UNKNOWN {
          // This is normal, since we might not know about the participant yet.
          debug!(
            "No remote participant known for {:?}\nSearched with {:?} in {:?}",
            data,
            guid.prefix,
            self.participant_proxies.keys()
          );
        }
        (Vec::default(), Vec::default())
      });
    debug!("External reader: {:?}", data);

    // Now the topic update:
    let dtd = data.subscription_topic_data.to_topic_data();
    self.update_topic_data(
      &DiscoveredTopicData::new(Utc::now(), dtd),
      guid,
      DiscoveredVia::Subscription,
    );

    // TODO: Lookup the topic in DB, data sent by the same participant that sent the
    // reader update. If there is a DiscoveredVia::Topic record, use QosPolicies
    // from that record and modify by QoS given in the DRD.

    // Return DiscoveredReaderData with possibly updated locators.
    DiscoveredReaderData {
      reader_proxy: ReaderProxy::from(RtpsReaderProxy::from_discovered_reader_data(
        data,
        &default_locator_lists.0,
        &default_locator_lists.1,
      )),
      ..data.clone()
    }
  }

  // TODO: This is silly. Returns one of the parameters cloned, or None
  pub fn update_publication(&mut self, data: &DiscoveredWriterData) -> DiscoveredWriterData {
    let guid = data.writer_proxy.remote_writer_guid;

    self
      .external_topic_writers
      .insert(data.writer_proxy.remote_writer_guid, data.clone());

    // fill in the default locators from participant, in case DRD did not provide
    // any
    let default_locator_lists = self
      .find_participant_proxy(guid.prefix)
      .map(|pp| {
        debug!("Added participant locators to Reader {:?}", guid);
        (
          pp.default_unicast_locators.clone(),
          pp.default_multicast_locators.clone(),
        )
      })
      .unwrap_or_else(|| {
        if guid.prefix != GuidPrefix::UNKNOWN {
          // This is normal, since we might not know about the participant yet.
          debug!(
            "No remote participant known for {:?}\nSearched with {:?} in {:?}",
            data,
            guid.prefix,
            self.participant_proxies.keys()
          );
        }
        (Vec::default(), Vec::default())
      });

    debug!("External writer: {:?}", data);

    // Now the topic update:
    let dtd = data.publication_topic_data.to_topic_data();
    self.update_topic_data(
      &DiscoveredTopicData::new(Utc::now(), dtd),
      guid,
      DiscoveredVia::Publication,
    );

    DiscoveredWriterData {
      writer_proxy: WriterProxy::from(RtpsWriterProxy::from_discovered_writer_data(
        data,
        &default_locator_lists.0,
        &default_locator_lists.1,
      )),
      ..data.clone()
    }
  }

  // This is for local participant updating the topic table
  pub fn update_topic_data_p(&mut self, topic: &Topic) {
    let topic_data = DiscoveredTopicData::new(
      Utc::now(),
      TopicBuiltinTopicData::new(
        None,
        topic.name(),
        topic.get_type().name().to_owned(),
        &topic.qos(),
      ),
    );
    self.update_topic_data(&topic_data, self.my_guid, DiscoveredVia::Topic);
  }

  // Topic update sends notifications, in case someone was waiting to find a
  // topic. Return value indicates whether the topic (name) was new to us. This
  // is used to add
  pub fn update_topic_data(
    &mut self,
    dtd: &DiscoveredTopicData,
    updater: GUID,
    discovered_via: DiscoveredVia,
  ) {
    trace!("Update topic data: {:?}", &dtd);
    let topic_name = dtd.topic_data.name.clone();
    let mut notify = false;

    if let Some(t) = self.topics.get_mut(&dtd.topic_data.name) {
      if let Some(old_dtd) = t.get_mut(&updater.prefix) {
        // already have it from the same source, do some checking(?) and merging
        if dtd.topic_data.type_name == old_dtd.1.topic_data.type_name
        // TODO: Check also for QoS changes, esp. policies that are immutable
        {
          // If this discovery was from Topic topic and the old was not, then update
          if discovered_via == DiscoveredVia::Topic {
            *old_dtd = (discovered_via, dtd.clone()); // update QoS
            notify = true;
          } else {
            debug!(
              "Topic {:?} update ignored from {:?}. Already have this.",
              &topic_name, &updater
            );
            // TODO: Here we could warn about QoS changes.
          }
        } else {
          // someone changed their mind about the type name?!?
          error!(
            "Inconsistent topic update from {:?}: type was: {:?} new type: {:?}",
            updater, old_dtd.1.topic_data.type_name, dtd.topic_data.type_name,
          );
        }
      } else {
        // We have to topic, but not from this participant
        // TODO: Check that there is agreement about topic type name (at least)
        t.insert(updater.prefix, (discovered_via, dtd.clone())); // this should return None
        notify = true;
      }
    } else {
      // new topic to us
      let mut b = BTreeMap::new();
      b.insert(updater.prefix, (discovered_via, dtd.clone()));
      self.topics.insert(topic_name, b);
    };

    if notify {
      self
        .topic_updated_sender
        .try_send(())
        // It is quite normal for this to fail due to channel full,
        // because usually there is no-one at the other end receiving.
        .unwrap_or_else(|e| trace!("update_topic_data: Notification send failed: {e:?}"));
    }
  }

  // local topic readers
  pub fn update_local_topic_reader(
    &mut self,
    domain_participant: &DomainParticipant,
    topic: &Topic,
    reader: &ReaderIngredients,
  ) {
    let reader_guid = reader.guid;

    let reader_proxy = RtpsReaderProxy::from_reader(reader, domain_participant);

    let subscription_data = SubscriptionBuiltinTopicData::new(
      reader_guid,
      Some(domain_participant.guid()),
      topic.name(),
      topic.get_type().name().to_string(),
      &topic.qos(),
      None, // <<---------------TODO: None here means we have no EndpointSecurityInfo
    );

    // TODO: possibly change content filter to dynamic value
    let content_filter = None;

    let discovered_reader_data = DiscoveredReaderData {
      reader_proxy: ReaderProxy::from(reader_proxy),
      subscription_topic_data: subscription_data,
      content_filter,
    };

    self
      .local_topic_readers
      .insert(reader_guid, discovered_reader_data);
  }

  pub fn remove_local_topic_reader(&mut self, guid: GUID) {
    self.local_topic_readers.remove(&guid);
  }

  pub fn get_all_local_topic_readers(&self) -> impl Iterator<Item = &DiscoveredReaderData> {
    self.local_topic_readers.values()
  }

  pub fn get_all_local_topic_writers(&self) -> impl Iterator<Item = &DiscoveredWriterData> {
    self.local_topic_writers.values()
  }

  // Note:
  // If multiple participants announce the same topic, this will
  // return duplicates, one per announcing participant.
  // The duplicates are not necessarily identical, but may have different QoS.
  pub fn all_user_topics(&self) -> impl Iterator<Item = &DiscoveredTopicData> {
    self
      .topics
      .iter()
      .filter(|(s, _)| !s.starts_with("DCPS"))
      .flat_map(|(_, gm)| gm.iter().map(|(_, dtd)| &dtd.1))
  }

  // as above, but only from my GUID
  pub fn local_user_topics(&self) -> impl Iterator<Item = &DiscoveredTopicData> {
    let me = self.my_guid.prefix;
    self
      .topics
      .iter()
      .filter(|(s, _)| !s.starts_with("DCPS"))
      .flat_map(move |(_, gm)| {
        gm.iter()
          .filter(move |(guid, _)| **guid == me)
          .map(|(_, dtd)| &dtd.1)
      })
  }

  // a Topic may have multiple definitions, because there may be multiple
  // participants publishing the topic information.
  // At least the QoS details may be different.
  // This just returns the first one found in the database, which is indexed by
  // GUID.
  pub fn get_topic(&self, topic_name: &str) -> Option<&DiscoveredTopicData> {
    self
      .topics
      .get(topic_name)
      .and_then(|m| m.values().next().map(|t| &t.1))
  }

  pub fn writers_on_topic_and_participant(
    &self,
    topic_name: &str,
    participant: GuidPrefix,
  ) -> Vec<DiscoveredWriterData> {
    let on_participant = self
      .external_topic_writers
      .range(participant.range())
      .map(|(_guid, dwd)| dwd);
    info!(
      "Writers on participant {:?} are {:?}",
      participant, on_participant
    );
    on_participant
      .filter(|dwd| dwd.publication_topic_data.topic_name == topic_name)
      .cloned()
      .collect()
  }

  pub fn readers_on_topic_and_participant(
    &self,
    topic_name: &str,
    participant: GuidPrefix,
  ) -> Vec<DiscoveredReaderData> {
    self
      .external_topic_readers
      .range(participant.range())
      .map(|(_guid, drd)| drd)
      .filter(|drd| drd.subscription_topic_data.topic_name() == topic_name)
      .cloned()
      .collect()
  }

  // // TODO: return iterator somehow?
  #[cfg(test)] // used only for testing
  pub fn get_local_topic_readers<'a, T: TopicDescription>(
    &'a self,
    topic: &'a T,
  ) -> Vec<&DiscoveredReaderData> {
    let topic_name = topic.name();
    self
      .local_topic_readers
      .iter()
      .filter(|(_, p)| *p.subscription_topic_data.topic_name() == topic_name)
      .map(|(_, p)| p)
      .collect()
  }

  pub fn update_lease_duration(&mut self, data: &ParticipantMessageData) {
    let now = Instant::now();
    let prefix = data.guid;
    self
      .external_topic_writers
      .range_mut(prefix.range())
      .for_each(|(_guid, p)| p.last_updated = now);
  }
}

#[cfg(test)]
mod tests {
  use std::{
    sync::{Arc, Mutex},
    time::Duration as StdDuration,
  };

  use byteorder::LittleEndian;
  use mio_extras::channel as mio_channel;

  use super::*;
  use crate::{
    dds::{
      qos::QosPolicies,
      statusevents::{sync_status_channel, DataReaderStatus},
      topic::TopicKind,
      with_key::simpledatareader::ReaderCommand,
    },
    mio_source,
    serialization::cdr_serializer::CDRSerializerAdapter,
    structure::guid::*,
    test::{
      random_data::RandomData,
      test_data::{reader_proxy_data, spdp_participant_data, subscription_builtin_topic_data},
    },
    SequenceNumber,
  };

  #[test]
  fn discdb_participant_operations() {
    let (discovery_db_event_sender, _discovery_db_event_receiver) =
      mio_channel::sync_channel::<()>(4);

    let mut discoverydb = DiscoveryDB::new(GUID::new_participant_guid(), discovery_db_event_sender);
    let mut data = spdp_participant_data().unwrap();
    data.lease_duration = Some(Duration::from(StdDuration::from_secs(1)));

    discoverydb.update_participant(&data);
    assert_eq!(discoverydb.participant_proxies.len(), 1);

    discoverydb.update_participant(&data);
    assert_eq!(discoverydb.participant_proxies.len(), 1);

    std::thread::sleep(StdDuration::from_secs(2));
    discoverydb.participant_cleanup();
    assert!(discoverydb.participant_proxies.is_empty());

    // TODO: more operations tests
  }

  #[test]
  fn discdb_writer_proxies() {
    let (discovery_db_event_sender, _discovery_db_event_receiver) =
      mio_channel::sync_channel::<()>(4);
    let _discoverydb = DiscoveryDB::new(GUID::new_participant_guid(), discovery_db_event_sender);
    let topic_name = String::from("some_topic");
    let type_name = String::from("RandomData");
    let _dreader = DiscoveredReaderData::default(topic_name, type_name);

    // TODO: more tests :)
  }

  #[test]
  fn discdb_subscription_operations() {
    let (discovery_db_event_sender, _discovery_db_event_receiver) =
      mio_channel::sync_channel::<()>(4);

    let mut discovery_db =
      DiscoveryDB::new(GUID::new_participant_guid(), discovery_db_event_sender);

    let domain_participant = DomainParticipant::new(0).expect("Failed to create publisher");
    let topic = domain_participant
      .create_topic(
        "Foobar".to_string(),
        "RandomData".to_string(),
        &QosPolicies::qos_none(),
        TopicKind::WithKey,
      )
      .unwrap();
    let topic2 = domain_participant
      .create_topic(
        "Barfoo".to_string(),
        "RandomData".to_string(),
        &QosPolicies::qos_none(),
        TopicKind::WithKey,
      )
      .unwrap();

    let publisher1 = domain_participant
      .create_publisher(&QosPolicies::qos_none())
      .unwrap();
    let dw = publisher1
      .create_datawriter::<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>>(&topic, None)
      .unwrap();

    let writer_data = DiscoveredWriterData::new(&dw, &topic, &domain_participant, None);

    discovery_db.update_local_topic_writer(writer_data);
    assert_eq!(discovery_db.local_topic_writers.len(), 1);

    let publisher2 = domain_participant
      .create_publisher(&QosPolicies::qos_none())
      .unwrap();
    let dw2 = publisher2
      .create_datawriter::<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>>(
        &topic2, None,
      )
      .unwrap();
    let writer_data2 = DiscoveredWriterData::new(&dw2, &topic2, &domain_participant, None);
    discovery_db.update_local_topic_writer(writer_data2);
    assert_eq!(discovery_db.local_topic_writers.len(), 2);

    // creating data
    let reader1 = reader_proxy_data().unwrap();
    let reader1sub = subscription_builtin_topic_data().unwrap();
    // reader1sub.set_key(reader1.remote_reader_guid);
    // reader1sub.set_topic_name(&topic.name());
    let dreader1 = DiscoveredReaderData {
      reader_proxy: reader1.clone(),
      subscription_topic_data: reader1sub.clone(),
      content_filter: None,
    };
    discovery_db.update_subscription(&dreader1);

    let reader2 = reader_proxy_data().unwrap();
    let reader2sub = subscription_builtin_topic_data().unwrap();
    // reader2sub.set_key(reader2.remote_reader_guid);
    // reader2sub.set_topic_name(&topic2.name());
    let dreader2 = DiscoveredReaderData {
      reader_proxy: reader2,
      subscription_topic_data: reader2sub,
      content_filter: None,
    };
    discovery_db.update_subscription(&dreader2);

    let reader3 = reader1;
    let reader3sub = reader1sub;
    let dreader3 = DiscoveredReaderData {
      reader_proxy: reader3,
      subscription_topic_data: reader3sub,
      content_filter: None,
    };
    discovery_db.update_subscription(&dreader3);

    // TODO: there might be a need for different scenarios
  }

  #[test]
  fn discdb_local_topic_reader() {
    let (discovery_db_event_sender, _discovery_db_event_receiver) =
      mio_channel::sync_channel::<()>(4);

    let dp = DomainParticipant::new(0).expect("Failed to create participant");
    let topic = dp
      .create_topic(
        "some topic name".to_string(),
        "Wazzup".to_string(),
        &QosPolicies::qos_none(),
        TopicKind::WithKey,
      )
      .unwrap();
    let mut discoverydb = DiscoveryDB::new(GUID::new_participant_guid(), discovery_db_event_sender);

    // Create reader ingredients
    let (notification_sender1, _notification_receiver1) = mio_extras::channel::sync_channel(100);
    let (_notification_event_source1, notification_event_sender1) =
      mio_source::make_poll_channel().unwrap();
    let data_reader_waker1 = Arc::new(Mutex::new(None));

    let (status_sender1, _status_receiver1) = sync_status_channel::<DataReaderStatus>(4).unwrap();
    let (_reader_commander1, reader_command_receiver1) =
      mio_extras::channel::sync_channel::<ReaderCommand>(100);

    let topic_cache =
      dp.dds_cache()
        .write()
        .unwrap()
        .add_new_topic(topic.name(), topic.get_type(), &topic.qos());

    let last_read_sequence_number_ref =
      Arc::new(Mutex::new(BTreeMap::<GUID, SequenceNumber>::new()));

    let reader1_ing = ReaderIngredients {
      guid: GUID::dummy_test_guid(EntityKind::READER_NO_KEY_USER_DEFINED),
      notification_sender: notification_sender1,
      status_sender: status_sender1,
      topic_name: topic.name(),
      topic_cache_handle: topic_cache.clone(),
      last_read_sequence_number_ref: last_read_sequence_number_ref.clone(),
      qos_policy: QosPolicies::qos_none(),
      data_reader_command_receiver: reader_command_receiver1,
      data_reader_waker: data_reader_waker1,
      poll_event_sender: notification_event_sender1,
    };

    // Add the reader to the database and verify the info is updated
    discoverydb.update_local_topic_reader(&dp, &topic, &reader1_ing);
    assert_eq!(discoverydb.local_topic_readers.len(), 1);
    assert_eq!(discoverydb.get_local_topic_readers(&topic).len(), 1);

    // Verify that the info does not change if the reader is added a second time
    discoverydb.update_local_topic_reader(&dp, &topic, &reader1_ing);
    assert_eq!(discoverydb.local_topic_readers.len(), 1);
    assert_eq!(discoverydb.get_local_topic_readers(&topic).len(), 1);

    // Create second reader ingredients for the same topic
    let (notification_sender2, _notification_receiver2) = mio_extras::channel::sync_channel(100);
    let (_notification_event_source2, notification_event_sender2) =
      mio_source::make_poll_channel().unwrap();
    let data_reader_waker2 = Arc::new(Mutex::new(None));

    let (status_sender2, _status_receiver2) = sync_status_channel::<DataReaderStatus>(4).unwrap();
    let (_reader_commander2, reader_command_receiver2) =
      mio_extras::channel::sync_channel::<ReaderCommand>(100);

    let mut guid2 = GUID::dummy_test_guid(EntityKind::READER_NO_KEY_USER_DEFINED);
    guid2.prefix = GuidPrefix::new(b"Another fake"); // GUID needs to be different in order to be added

    let reader2_ing = ReaderIngredients {
      guid: guid2,
      notification_sender: notification_sender2,
      status_sender: status_sender2,
      topic_name: topic.name(),
      topic_cache_handle: topic_cache,
      last_read_sequence_number_ref,
      qos_policy: QosPolicies::qos_none(),
      data_reader_command_receiver: reader_command_receiver2,
      data_reader_waker: data_reader_waker2,
      poll_event_sender: notification_event_sender2,
    };

    // Add the second reader to the database and verify the info is updated
    discoverydb.update_local_topic_reader(&dp, &topic, &reader2_ing);
    assert_eq!(discoverydb.get_local_topic_readers(&topic).len(), 2);
    assert_eq!(discoverydb.get_all_local_topic_readers().count(), 2);
  }
}
