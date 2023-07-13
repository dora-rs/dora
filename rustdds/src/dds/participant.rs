//use mio::Token;
use std::{
  collections::HashMap,
  io::ErrorKind,
  net::Ipv4Addr,
  sync::{atomic, Arc, Mutex, RwLock, Weak},
  thread,
  thread::JoinHandle,
  time::{Duration, Instant},
};

use mio_extras::channel as mio_channel;
use mio_06::Token;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::{pubsub::*, qos::*, result::*, topic::*, typedesc::TypeDesc},
  discovery::{
    discovery::{Discovery, DiscoveryCommand},
    discovery_db::DiscoveryDB,
    sedp_messages::DiscoveredTopicData,
  },
  log_and_err_internal,
  network::{constant::*, udp_listener::UDPListener},
  rtps::{
    dp_event_loop::{DPEventLoop, DomainInfo},
    reader::*,
    writer::WriterIngredients,
  },
  structure::{dds_cache::DDSCache, entity::RTPSEntity, guid::*, locator::Locator},
};

/// DDS DomainParticipant
///
/// It is recommended that only one DomainParticipant per OS process is created,
/// as it allocates network sockets, creates background threads, and allocates
/// some memory for object caches.
///
/// If you need to communicate to many DDS domains,
/// then you must create a separate DomainParticipant for each of them.
/// See DDS Spec v1.4 Section "2.2.1.2.2 Overall Conceptual Model" and
/// "2.2.2.2.1 DomainParticipant Class" for a definition of a (DDS) domain.
/// Domains are identified by a domain identifier, which is, in Rust terms, a
/// `u16`. Domain identifier values are application-specific, but `0` is usually
/// the default.
#[derive(Clone)]
// This is a smart pointer for DomainParticipantInner for easier manipulation.
pub struct DomainParticipant {
  dpi: Arc<Mutex<DomainParticipantDisc>>,
}

#[allow(clippy::new_without_default)]
impl DomainParticipant {
  /// # Examples
  /// ```
  /// # use rustdds::DomainParticipant;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// ```
  pub fn new(domain_id: u16) -> Result<Self> {
    trace!("DomainParticipant construct start");

    // Discovery join channel is used to just send a join handle into the inner
    // participant, so its .drop() can wait until discovery has had a chance to
    // stop.
    let (djh_sender, djh_receiver) = mio_channel::channel();

    // Channel is used to notify Discovery of (duplicate) SPDP messages from the
    // wire.
    let (spdp_liveness_sender, spdp_liveness_receiver) = mio_channel::sync_channel(8);

    // Discovery thread receives and decodes updates from the wire.
    // It updates data to DiscoveryDB, and sends notifications to dp_event_loop,
    // which owns the Readers and Writers and notifies them also.
    let (discovery_updated_sender, discovery_update_notification_receiver) =
      mio_channel::sync_channel::<DiscoveryNotificationType>(32);

    // This channel is used to:
    // * local DataReader and DataWriter notify Discovery on drop() so that
    // Discovery knows we no longer have them.
    // * Participant commands Discovery to assert liveness, i.e. send liveness
    // message to remote participants.
    // * Discovery commands Discovery (thread) to terminate on exit.
    let (discovery_command_sender, discovery_command_receiver) =
      mio_channel::sync_channel::<DiscoveryCommand>(64);

    // intermediate DP wrapper
    let dp = DomainParticipantDisc::new(
      domain_id,
      djh_receiver,
      discovery_update_notification_receiver,
      discovery_command_sender,
      spdp_liveness_sender,
    )?;
    let self_locators = dp.self_locators();

    // outer DP wrapper
    let dp = Self {
      dpi: Arc::new(Mutex::new(dp)),
    };

    let (discovery_started_sender, discovery_started_receiver) =
      std::sync::mpsc::channel::<Result<()>>();

    // Construct and start background thread
    let dp_clone = dp.weak_clone();
    let disc_db_clone = dp.discovery_db();
    let discovery_handle = thread::Builder::new()
      .name("RustDDS discovery thread".to_string())
      .spawn(move || {
        if let Ok(mut discovery) = Discovery::new(
          dp_clone,
          disc_db_clone,
          discovery_started_sender,
          discovery_updated_sender,
          discovery_command_receiver,
          spdp_liveness_receiver,
          self_locators,
        ) {
          discovery.discovery_event_loop(); // run the event loop
        }
      })?;

    djh_sender.send(discovery_handle).unwrap_or(()); // send join handle to inner participant

    debug!("Waiting for discovery to start"); // blocking until discovery answers
    match discovery_started_receiver.recv_timeout(Duration::from_secs(10)) {
      Ok(Ok(())) => {
        // normal case
        info!("Discovery started. Participant constructed.");
        Ok(dp)
      }
      Ok(Err(e)) => {
        std::mem::drop(dp);
        log_and_err_internal!("Failed to start discovery thread: {e:?}")
      }
      Err(e) => log_and_err_internal!("Discovery thread channel error: {e:?}"),
    }
  }

  /// Creates DDS Publisher
  ///
  /// # Arguments
  ///
  /// * `qos` - Takes [qos policies](qos/struct.QosPolicies.html) for publisher
  ///   and given to DataWriter as default.
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::{DomainParticipant, QosPolicyBuilder};
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos);
  /// ```
  pub fn create_publisher(&self, qos: &QosPolicies) -> Result<Publisher> {
    let w = self.weak_clone(); // this must be done first to avoid deadlock
    self.dpi.lock().unwrap().create_publisher(&w, qos)
  }

  /// Creates DDS Subscriber
  ///
  /// # Arguments
  ///
  /// * `qos` - Takes [qos policies](qos/struct.QosPolicies.html) for subscriber
  ///   and given to DataReader as default.
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::{DomainParticipant, QosPolicyBuilder};
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos);
  /// ```
  pub fn create_subscriber(&self, qos: &QosPolicies) -> Result<Subscriber> {
    // println!("DP(outer): create_subscriber");
    let w = self.weak_clone(); // do this first, avoid deadlock
    self.dpi.lock().unwrap().create_subscriber(&w, qos)
  }

  /// Create DDS Topic
  ///
  /// # Arguments
  ///
  /// * `name` - Name of the topic.
  /// * `type_desc` - Name of the type this topic is supposed to deliver.
  /// * `qos` - Takes [qos policies](qos/struct.QosPolicies.html) that are
  ///   distributed to DataReaders and DataWriters.
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::{DomainParticipant, TopicKind, QosPolicyBuilder};
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey);
  /// ```
  pub fn create_topic(
    &self,
    name: String,
    type_desc: String,
    qos: &QosPolicies,
    topic_kind: TopicKind,
  ) -> Result<Topic> {
    // println!("Create topic outer");
    let w = self.weak_clone();
    self
      .dpi
      .lock()
      .unwrap()
      .create_topic(&w, name, type_desc, qos, topic_kind)
  }

  pub fn find_topic(&self, name: &str, timeout: Duration) -> Result<Option<Topic>> {
    let w = self.weak_clone();
    self.dpi.lock().unwrap().find_topic(&w, name, timeout)
  }

  /// # Examples
  ///
  /// ```
  /// # use rustdds::DomainParticipant;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let domain_id = domain_participant.domain_id();
  /// ```
  pub fn domain_id(&self) -> u16 {
    self.dpi.lock().unwrap().domain_id()
  }

  /// # Examples
  ///
  /// ```
  /// # use rustdds::DomainParticipant;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let participant_id = domain_participant.participant_id();
  /// ```
  pub fn participant_id(&self) -> u16 {
    self.dpi.lock().unwrap().participant_id()
  }

  /// Gets all DiscoveredTopics from DDS network
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::DomainParticipant;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let discovered_topics = domain_participant.discovered_topics();
  /// for dtopic in discovered_topics.iter() {
  ///   // do something
  /// }
  /// ```
  pub fn discovered_topics(&self) -> Vec<DiscoveredTopicData> {
    self.dpi.lock().unwrap().discovered_topics()
  }

  /// Manually asserts liveliness, affecting all writers with
  /// LIVELINESS QoS of MANUAL_BY_PARTICIPANT created by
  /// this particular participant.
  ///
  /// # Example
  ///
  /// ```
  /// # use rustdds::DomainParticipant;
  ///
  /// let domain_participant = DomainParticipant::new(0).expect("Failed to create participant");
  /// domain_participant.assert_liveliness();
  /// ```
  pub fn assert_liveliness(self) -> Result<()> {
    self.dpi.lock().unwrap().assert_liveliness()
  }

  pub(crate) fn weak_clone(&self) -> DomainParticipantWeak {
    DomainParticipantWeak::new(self, self.guid())
  }

  pub(crate) fn dds_cache(&self) -> Arc<RwLock<DDSCache>> {
    self.dpi.lock().unwrap().dds_cache()
  }

  pub(crate) fn discovery_db(&self) -> Arc<RwLock<DiscoveryDB>> {
    self
      .dpi
      .lock()
      .unwrap()
      .dpi
      .lock()
      .unwrap()
      .discovery_db
      .clone()
  }

  pub(crate) fn new_entity_id(&self, entity_kind: EntityKind) -> EntityId {
    self.dpi.lock().unwrap().new_entity_id(entity_kind)
  }

  pub(crate) fn self_locators(&self) -> HashMap<Token, Vec<Locator>> {
    self.dpi.lock().unwrap().self_locators()
  }
} // end impl DomainParticipant

impl PartialEq for DomainParticipant {
  fn eq(&self, other: &Self) -> bool {
    self.guid() == other.guid()
      && self.domain_id() == other.domain_id()
      && self.participant_id() == other.participant_id()
  }
}

#[derive(Clone)]
pub struct DomainParticipantWeak {
  dpi: Weak<Mutex<DomainParticipantDisc>>,
  // This struct caches the GUID to avoid construction deadlocks
  guid: GUID,
}

impl DomainParticipantWeak {
  pub fn new(dp: &DomainParticipant, guid: GUID) -> Self {
    Self {
      dpi: Arc::downgrade(&dp.dpi),
      guid,
    }
  }

  pub fn create_publisher(&self, qos: &QosPolicies) -> Result<Publisher> {
    self
      .dpi
      .upgrade()
      .ok_or(Error::OutOfResources)
      .and_then(|dpi| dpi.lock().unwrap().create_publisher(self, qos))
  }

  pub fn create_subscriber(&self, qos: &QosPolicies) -> Result<Subscriber> {
    self
      .dpi
      .upgrade()
      .ok_or(Error::OutOfResources)
      .and_then(|dpi| dpi.lock().unwrap().create_subscriber(self, qos))
  }

  pub fn create_topic(
    &self,
    name: String,
    type_desc: String,
    qos: &QosPolicies,
    topic_kind: TopicKind,
  ) -> Result<Topic> {
    self
      .dpi
      .upgrade()
      .ok_or(Error::LockPoisoned)
      .and_then(|dpi| {
        dpi
          .lock()
          .unwrap()
          .create_topic(self, name, type_desc, qos, topic_kind)
      })
  }

  // pub fn find_topic(&self, name: &str, timeout: Duration) ->
  // Result<Option<Topic>> {   self
  //     .dpi
  //     .upgrade()
  //     .ok_or(Error::LockPoisoned)
  //     .and_then(|dpi| dpi.lock().unwrap().find_topic(self, name, timeout))
  // }

  // pub fn domain_id(&self) -> u16 {
  //   self
  //     .dpi
  //     .upgrade()
  //     .expect("Unable to get original domain participant.")
  //     .lock()
  //     .unwrap()
  //     .domain_id()
  // }

  // pub fn participant_id(&self) -> u16 {
  //   self
  //     .dpi
  //     .upgrade()
  //     .expect("Unable to get original domain participant.")
  //     .lock()
  //     .unwrap()
  //     .participant_id()
  // }

  // pub fn discovered_topics(&self) -> Vec<DiscoveredTopicData> {
  //   self
  //     .dpi
  //     .upgrade()
  //     .map(|dpi| dpi.lock().unwrap().discovered_topics())
  //     .unwrap_or_default()
  // }

  pub fn upgrade(self) -> Option<DomainParticipant> {
    self.dpi.upgrade().map(|d| DomainParticipant { dpi: d })
  }
} // end impl

impl RTPSEntity for DomainParticipantWeak {
  fn guid(&self) -> GUID {
    self.guid
  }
}

// This struct exists only to control and stop Discovery when DomainParticipant
// should be dropped
pub(crate) struct DomainParticipantDisc {
  dpi: Arc<Mutex<DomainParticipantInner>>,
  // Discovery control
  discovery_command_sender: mio_channel::SyncSender<DiscoveryCommand>,
  discovery_join_handle: mio_channel::Receiver<JoinHandle<()>>,
  // This allows deterministic generation of EntityIds for DataReader, DataWriter, etc.
  entity_id_generator: atomic::AtomicU32,
}

impl DomainParticipantDisc {
  pub fn new(
    domain_id: u16,
    discovery_join_handle: mio_channel::Receiver<JoinHandle<()>>,
    discovery_update_notification_receiver: mio_channel::Receiver<DiscoveryNotificationType>,
    discovery_command_sender: mio_channel::SyncSender<DiscoveryCommand>,
    spdp_liveness_sender: mio_channel::SyncSender<GuidPrefix>,
  ) -> Result<Self> {
    let dpi = DomainParticipantInner::new(
      domain_id,
      discovery_update_notification_receiver,
      spdp_liveness_sender,
    )?;

    Ok(Self {
      dpi: Arc::new(Mutex::new(dpi)),
      discovery_command_sender,
      discovery_join_handle,
      entity_id_generator: atomic::AtomicU32::new(0),
    })
  }

  // This generates identifiers that consist of given EntityKind and arbitrary,
  // unique identifier.
  pub(crate) fn new_entity_id(&self, entity_kind: EntityKind) -> EntityId {
    let [_goldilocks, papa_byte, mama_byte, baby_byte] = self
      .entity_id_generator
      .fetch_add(1, atomic::Ordering::Relaxed)
      .to_be_bytes();
    EntityId::new([papa_byte, mama_byte, baby_byte], entity_kind)
  }

  pub fn create_publisher(
    &self,
    dp: &DomainParticipantWeak,
    qos: &QosPolicies,
  ) -> Result<Publisher> {
    self
      .dpi
      .lock()
      .unwrap()
      .create_publisher(dp, qos, self.discovery_command_sender.clone())
  }

  pub fn create_subscriber(
    &self,
    dp: &DomainParticipantWeak,
    qos: &QosPolicies,
  ) -> Result<Subscriber> {
    self
      .dpi
      .lock()
      .unwrap()
      .create_subscriber(dp, qos, self.discovery_command_sender.clone())
  }

  pub fn create_topic(
    &self,
    dp: &DomainParticipantWeak,
    name: String,
    type_desc: String,
    qos: &QosPolicies,
    topic_kind: TopicKind,
  ) -> Result<Topic> {
    // println!("Create topic disc");
    self
      .dpi
      .lock()
      .unwrap()
      .create_topic(dp, name, type_desc, qos, topic_kind)
  }

  pub fn find_topic(
    &self,
    dp: &DomainParticipantWeak,
    name: &str,
    timeout: Duration,
  ) -> Result<Option<Topic>> {
    self.dpi.lock().unwrap().find_topic(dp, name, timeout)
  }

  pub fn domain_id(&self) -> u16 {
    self.dpi.lock().unwrap().domain_id()
  }

  pub fn participant_id(&self) -> u16 {
    self.dpi.lock().unwrap().participant_id()
  }

  pub fn discovered_topics(&self) -> Vec<DiscoveredTopicData> {
    self.dpi.lock().unwrap().discovered_topics()
  }

  pub(crate) fn dds_cache(&self) -> Arc<RwLock<DDSCache>> {
    self.dpi.lock().unwrap().dds_cache()
  }

  // pub(crate) fn discovery_db(&self) -> Arc<RwLock<DiscoveryDB>> {
  //   self.dpi.lock().unwrap().discovery_db.clone()
  // }

  pub(crate) fn assert_liveliness(&self) -> Result<()> {
    // No point in checking for the LIVELINESS QoS of MANUAL_BY_PARTICIPANT,
    // the discovery command mutates a field which is only read
    // by writers with that particular QoS.
    self
      .discovery_command_sender
      .send(DiscoveryCommand::ManualAssertLiveliness)
      .or_else(|e| {
        log_and_err_internal!("assert_liveness - Failed to send DiscoveryCommand. {e:?}")
      })
  }

  pub(crate) fn self_locators(&self) -> HashMap<Token, Vec<Locator>> {
    self.dpi.lock().unwrap().self_locators.clone()
  }
}

impl Drop for DomainParticipantDisc {
  fn drop(&mut self) {
    info!("===== RustDDS shutting down ===== .drop() DomainParticipantDisc");
    debug!("Sending Discovery Stop signal.");
    if self
      .discovery_command_sender
      .send(DiscoveryCommand::StopDiscovery)
      .is_err()
    {
      warn!("Failed to send stop signal to Discovery");
      return;
    }

    debug!("Waiting for Discovery join.");
    if let Ok(handle) = self.discovery_join_handle.try_recv() {
      handle.join().unwrap();
      debug!("Joined Discovery.");
    }
  }
}

// This is the actual working DomainParticipant.
pub(crate) struct DomainParticipantInner {
  domain_id: u16,
  participant_id: u16,

  my_guid: GUID,

  // Adding Readers
  sender_add_reader: mio_channel::SyncSender<ReaderIngredients>,
  sender_remove_reader: mio_channel::SyncSender<GUID>,

  // dp_event_loop control
  stop_poll_sender: mio_channel::Sender<()>,
  ev_loop_handle: Option<JoinHandle<()>>, // this is Option, because it needs to be extracted
  // out of the struct (take) in order to .join() on the handle.

  // Writers
  add_writer_sender: mio_channel::SyncSender<WriterIngredients>,
  remove_writer_sender: mio_channel::SyncSender<GUID>,

  dds_cache: Arc<RwLock<DDSCache>>,
  discovery_db: Arc<RwLock<DiscoveryDB>>,
  discovery_db_event_receiver: mio_channel::Receiver<()>,

  // RTPS locators describing how to reach this DP
  self_locators: HashMap<Token, Vec<Locator>>,
}

impl Drop for DomainParticipantInner {
  fn drop(&mut self) {
    // if send has an error simply leave as we have lost control of the
    // ev_loop_thread anyways
    if self.stop_poll_sender.send(()).is_err() {
      return;
    }

    debug!("Waiting for dp_event_loop join");
    match self.ev_loop_handle.take() {
      Some(join_handle) => {
        join_handle
          .join()
          .unwrap_or_else(|e| warn!("Failed to join dp_event_loop: {e:?}"));
      }
      None => {
        error!("Someone managed to steal dp_event_loop join handle from DomainParticipantInner.");
      }
    }
    debug!("Joined dp_event_loop");
  }
}

#[allow(clippy::new_without_default)]
impl DomainParticipantInner {
  fn new(
    domain_id: u16,
    discovery_update_notification_receiver: mio_channel::Receiver<DiscoveryNotificationType>,
    spdp_liveness_sender: mio_channel::SyncSender<GuidPrefix>,
  ) -> Result<Self> {
    let mut listeners = HashMap::new();

    match UDPListener::new_multicast(
      "0.0.0.0",
      spdp_well_known_multicast_port(domain_id),
      Ipv4Addr::new(239, 255, 0, 1),
    ) {
      Ok(l) => {
        listeners.insert(DISCOVERY_MUL_LISTENER_TOKEN, l);
      }
      Err(e) => warn!("Cannot get multicast discovery listener: {e:?}"),
    }

    let mut participant_id = 0;

    let mut discovery_listener = None;

    // Magic value 120 below is from RTPS spec 2.5 Section "9.6.2.3 Default Port
    // Numbers"
    while discovery_listener.is_none() && participant_id < 120 {
      discovery_listener = UDPListener::new_unicast(
        "0.0.0.0",
        spdp_well_known_unicast_port(domain_id, participant_id),
      )
      .ok();
      if discovery_listener.is_none() {
        participant_id += 1;
      }
    }

    info!("ParticipantId {} selected.", participant_id);

    // here discovery_listener is redefined (shadowed)
    let discovery_listener = match discovery_listener {
      Some(dl) => dl,
      None => return log_and_err_internal!("Could not find free ParticipantId"),
    };
    listeners.insert(DISCOVERY_LISTENER_TOKEN, discovery_listener);

    // Now the user traffic listeners

    match UDPListener::new_multicast(
      "0.0.0.0",
      user_traffic_multicast_port(domain_id),
      Ipv4Addr::new(239, 255, 0, 1),
    ) {
      Ok(l) => {
        listeners.insert(USER_TRAFFIC_MUL_LISTENER_TOKEN, l);
      }
      Err(e) => warn!("Cannot get multicast user traffic listener: {e:?}"),
    }

    let user_traffic_listener = UDPListener::new_unicast(
      "0.0.0.0",
      user_traffic_unicast_port(domain_id, participant_id),
    )
    .or_else(|e| {
      if matches!(e.kind(), ErrorKind::AddrInUse) {
        // If we do not get the preferred listening port,
        // try again, with "any" port number.
        UDPListener::new_unicast("0.0.0.0", 0).or_else(|e| {
          log_and_err_internal!(
            "Could not open unicast user traffic listener, any port number: {:?}",
            e
          )
        })
      } else {
        log_and_err_internal!("Could not open unicast user traffic listener: {e:?}")
      }
    })?;

    listeners.insert(USER_TRAFFIC_LISTENER_TOKEN, user_traffic_listener);

    // construct our own Locators
    let self_locators: HashMap<Token, Vec<Locator>> = listeners
      .iter()
      .map(|(t, l)| match l.to_locator_address() {
        Ok(locs) => (*t, locs),
        Err(e) => {
          error!("No local network address for token {:?}: {:?}", t, e);
          (*t, vec![])
        }
      })
      .collect();

    // Adding readers
    let (sender_add_reader, receiver_add_reader) =
      mio_channel::sync_channel::<ReaderIngredients>(100);
    let (sender_remove_reader, receiver_remove_reader) = mio_channel::sync_channel::<GUID>(10);

    // Writers
    let (add_writer_sender, add_writer_receiver) =
      mio_channel::sync_channel::<WriterIngredients>(10);
    let (remove_writer_sender, remove_writer_receiver) = mio_channel::sync_channel::<GUID>(10);

    let new_guid = GUID::new_participant_guid();
    let domain_info = DomainInfo {
      domain_participant_guid: new_guid,
      domain_id,
      participant_id,
    };

    let dds_cache = Arc::new(RwLock::new(DDSCache::new()));

    let (discovery_db_event_sender, discovery_db_event_receiver) =
      mio_channel::sync_channel::<()>(1);
    let discovery_db = Arc::new(RwLock::new(DiscoveryDB::new(
      new_guid,
      discovery_db_event_sender,
    )));

    let (stop_poll_sender, stop_poll_receiver) = mio_channel::channel::<()>();

    // Launch the background thread for DomainParticipant
    let dds_cache_clone = dds_cache.clone();
    let disc_db_clone = discovery_db.clone();
    let ev_loop_handle = thread::Builder::new()
      .name(format!("RustDDS Participant {} event loop", participant_id))
      .spawn(move || {
        let dp_event_loop = DPEventLoop::new(
          domain_info,
          listeners,
          dds_cache_clone,
          disc_db_clone,
          new_guid.prefix,
          TokenReceiverPair {
            token: ADD_READER_TOKEN,
            receiver: receiver_add_reader,
          },
          TokenReceiverPair {
            token: REMOVE_READER_TOKEN,
            receiver: receiver_remove_reader,
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
        dp_event_loop.event_loop();
      })?;

    info!(
      "New DomainParticipantInner: domain_id={:?} participant_id={:?} GUID={:?}",
      domain_id, participant_id, new_guid
    );
    Ok(Self {
      domain_id,
      participant_id,
      my_guid: new_guid,
      sender_add_reader,
      sender_remove_reader,
      stop_poll_sender,
      ev_loop_handle: Some(ev_loop_handle),
      add_writer_sender,
      remove_writer_sender,
      dds_cache,
      discovery_db,
      discovery_db_event_receiver,
      self_locators,
    })
  }

  pub fn dds_cache(&self) -> Arc<RwLock<DDSCache>> {
    self.dds_cache.clone()
  }

  // pub fn add_reader(&self, reader: ReaderIngredients) {
  //   self.sender_add_reader.send(reader).unwrap();
  // }

  // pub fn remove_reader(&self, guid: GUID) {
  //   let reader_guid = guid; // How to identify reader to be removed?
  //   self.sender_remove_reader.send(reader_guid).unwrap();
  // }

  // Publisher and subscriber creation
  //
  // There are no delete function for publisher or subscriber. Deletion is
  // performed by deleting the Publisher or Subscriber object, who upon deletion
  // will notify the DomainParticipant.
  pub fn create_publisher(
    &self,
    domain_participant: &DomainParticipantWeak,
    qos: &QosPolicies,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  ) -> Result<Publisher> {
    Ok(Publisher::new(
      domain_participant.clone(),
      self.discovery_db.clone(),
      qos.clone(),
      qos.clone(),
      self.add_writer_sender.clone(),
      self.remove_writer_sender.clone(),
      discovery_command,
    ))
  }

  pub fn create_subscriber(
    &self,
    domain_participant: &DomainParticipantWeak,
    qos: &QosPolicies,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  ) -> Result<Subscriber> {
    Ok(Subscriber::new(
      domain_participant.clone(),
      self.discovery_db.clone(),
      qos.clone(),
      self.sender_add_reader.clone(),
      self.sender_remove_reader.clone(),
      discovery_command,
    ))
  }

  // Topic creation. Data types should be handled as something (potentially) more
  // structured than a String. NOTE: Here we are using &str for topic name. &str
  // is Unicode string, whereas DDS specifies topic name to be a sequence of
  // octets, which would be &[u8] in Rust. This may cause problems if there are
  // topic names with non-ASCII characters. On the other hand, string handling
  // with &str is easier in Rust.
  pub fn create_topic(
    &self,
    domain_participant_weak: &DomainParticipantWeak,
    name: String,
    type_desc: String,
    qos: &QosPolicies,
    topic_kind: TopicKind,
  ) -> Result<Topic> {
    let topic = Topic::new(
      domain_participant_weak,
      name,
      TypeDesc::new(type_desc),
      qos,
      topic_kind,
    );
    Ok(topic)

    // TODO: refine
  }

  // Do not implement content filtered topics or multi-topics (yet)

  pub fn find_topic(
    &self,
    domain_participant_weak: &DomainParticipantWeak,
    name: &str,
    timeout: Duration,
  ) -> Result<Option<Topic>> {
    use mio_06 as mio;

    let poll = mio::Poll::new()?;
    let mut events = mio::Events::with_capacity(1);
    // Should be register before the check and use level trigger to avoid missing
    // event
    poll.register(
      &self.discovery_db_event_receiver,
      mio::Token(0),
      mio::Ready::readable(),
      mio::PollOpt::level(),
    )?;

    let find_end = Instant::now() + timeout;
    loop {
      if let Some(topic) = self.find_topic_in_discovery_db(domain_participant_weak, name)? {
        return Ok(Some(topic));
      }
      let timeout = find_end - Instant::now();
      poll.poll(&mut events, Some(timeout))?;

      if let Some(_event) = events.iter().next() {
        if self.discovery_db_event_receiver.try_recv().is_ok() {
          continue;
        }
      }

      if Instant::now() > find_end {
        break;
      }
    }

    Ok(None)
  }

  fn find_topic_in_discovery_db(
    &self,
    domain_participant_weak: &DomainParticipantWeak,
    name: &str,
  ) -> Result<Option<Topic>> {
    let db = self.discovery_db.read().map_err(|_| Error::LockPoisoned)?;

    let build_topic_fn = |d: &DiscoveredTopicData| {
      let qos = d.topic_data.qos();
      let topic_kind = match d.topic_data.key {
        Some(_) => TopicKind::WithKey,
        None => TopicKind::NoKey,
      };
      let name = d.topic_name().clone();
      let type_desc = d.topic_data.type_name.clone();
      self.create_topic(domain_participant_weak, name, type_desc, &qos, topic_kind)
    };

    if let Some(d) = db.get_topic(name) {
      // build a Topic from DiscoveredTopicData
      build_topic_fn(d).map(Some)
    } else {
      Ok(None)
    }
  }
  // get_builtin_subscriber (why would we need this?)

  // ignore_* operations. TODO: Do we need any of those?

  // delete_contained_entities is not needed. Data structures should be designed
  // so that lifetime of all created objects is within the lifetime of
  // DomainParticipant. Then such deletion is implicit.

  // The following methods are not for application use.

  // pub(crate) fn get_add_reader_sender(&self) ->
  // mio_channel::SyncSender<ReaderIngredients> {   self.sender_add_reader.
  // clone() }

  // pub(crate) fn get_remove_reader_sender(&self) ->
  // mio_channel::SyncSender<GUID> {   self.sender_remove_reader.clone()
  // }

  // pub(crate) fn get_add_writer_sender(&self) ->
  // mio_channel::SyncSender<WriterIngredients> {   self.add_writer_sender.
  // clone() }

  // pub(crate) fn get_remove_writer_sender(&self) ->
  // mio_channel::SyncSender<GUID> {   self.remove_writer_sender.clone()
  // }

  pub fn domain_id(&self) -> u16 {
    self.domain_id
  }

  pub fn participant_id(&self) -> u16 {
    self.participant_id
  }

  pub fn discovered_topics(&self) -> Vec<DiscoveredTopicData> {
    let db = self
      .discovery_db
      .read()
      .unwrap_or_else(|e| panic!("DiscoveryDB is poisoned. {e:?}"));

    db.all_user_topics().cloned().collect()
  }
} // impl

impl RTPSEntity for DomainParticipant {
  fn guid(&self) -> GUID {
    self.dpi.lock().unwrap().guid()
  }
}

impl RTPSEntity for DomainParticipantDisc {
  fn guid(&self) -> GUID {
    self.dpi.lock().unwrap().guid()
  }
}

impl RTPSEntity for DomainParticipantInner {
  fn guid(&self) -> GUID {
    self.my_guid
  }
}

impl std::fmt::Debug for DomainParticipant {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.debug_struct("DomainParticipant")
      .field("Guid", &self.guid())
      .finish()
  }
}

#[cfg(test)]
mod tests {
  use std::{
    collections::BTreeSet,
    net::{Ipv4Addr, SocketAddr, SocketAddrV4},
  };

  use enumflags2::BitFlags;
  use log::info;
  use speedy::{Endianness, Writable};
  use byteorder::LittleEndian;

  use crate::{
    dds::{qos::QosPolicies, topic::TopicKind},
    messages::{
      header::Header,
      protocol_id::ProtocolId,
      protocol_version::ProtocolVersion,
      submessages::submessages::{AckNack, SubmessageHeader, SubmessageKind, *},
      vendor_id::VendorId,
    },
    network::{constant::user_traffic_unicast_port, udp_sender::UDPSender},
    rtps::{submessage::*, Message, Submessage},
    serialization::cdr_serializer::CDRSerializerAdapter,
    structure::{
      guid::{EntityId, GUID},
      locator::Locator,
      sequence_number::{SequenceNumber, SequenceNumberSet},
    },
    test::random_data::RandomData,
  };
  use super::DomainParticipant;

  // TODO: improve basic test when more or the structure is known
  #[test]
  fn dp_basic_domain_participant() {
    // let _dp = DomainParticipant::new();

    let sender = UDPSender::new(11401).unwrap();
    let data: Vec<u8> = vec![0, 1, 2, 3, 4];

    let addrs = vec![SocketAddr::new("127.0.0.1".parse().unwrap(), 7412)];
    sender.send_to_all(&data, &addrs);

    // TODO: get result data from Reader
  }
  #[test]
  fn dp_writer_heartbeat_test() {
    let domain_participant = DomainParticipant::new(0).expect("Participant creation failed!");
    let qos = QosPolicies::qos_none();
    let _default_dw_qos = QosPolicies::qos_none();
    let publisher = domain_participant
      .create_publisher(&qos)
      .expect("Failed to create publisher");

    let topic = domain_participant
      .create_topic(
        "Aasii".to_string(),
        "RandomData".to_string(),
        &qos,
        TopicKind::WithKey,
      )
      .expect("Failed to create topic");

    let mut _data_writer = publisher
      .create_datawriter::<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>>(&topic, None)
      .expect("Failed to create datawriter");
  }

  #[test]
  fn dp_receive_acknack_message_test() {
    // TODO SEND ACKNACK
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

    let mut _data_writer = publisher
      .create_datawriter::<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>>(&topic, None)
      .expect("Failed to create datawriter");

    let port_number: u16 = user_traffic_unicast_port(5, 0);
    let sender = UDPSender::new(1234).unwrap();
    let mut m: Message = Message::default();

    let a: AckNack = AckNack {
      reader_id: EntityId::SPDP_BUILTIN_PARTICIPANT_READER,
      writer_id: EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER,
      reader_sn_state: SequenceNumberSet::from_base_and_set(
        SequenceNumber::default(),
        &BTreeSet::new(),
      ),
      count: 1,
    };
    let flags = BitFlags::<ACKNACK_Flags>::from_endianness(Endianness::BigEndian);
    let sub_header: SubmessageHeader = SubmessageHeader {
      kind: SubmessageKind::ACKNACK,
      flags: flags.bits(),
      content_length: 24,
    };

    let s: Submessage = Submessage {
      header: sub_header,
      body: SubmessageBody::Reader(ReaderSubmessage::AckNack(a, flags)),
    };
    let h = Header {
      protocol_id: ProtocolId::default(),
      protocol_version: ProtocolVersion { major: 2, minor: 3 },
      vendor_id: VendorId::THIS_IMPLEMENTATION,
      guid_prefix: GUID::default().prefix,
    };
    m.set_header(h);
    m.add_submessage(s);
    let _data: Vec<u8> = m.write_to_vec_with_ctx(Endianness::LittleEndian).unwrap();
    info!("data to send via udp: {:?}", _data);
    let ip = Ipv4Addr::from([0x00, 0x00, 0x00, 0x00]);
    let socket_address = SocketAddrV4::new(ip, port_number);
    let locators = vec![Locator::UdpV4(socket_address)];
    sender.send_to_locator_list(&_data, &locators);
  }
}
