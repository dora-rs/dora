use std::{
  fmt::Debug,
  sync::{Arc, Mutex, MutexGuard, RwLock},
  time::Duration,
};

use serde::{Deserialize, Serialize};
use mio_extras::channel as mio_channel;
use byteorder::LittleEndian;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::{
    adapters,
    key::{Key, Keyed},
    no_key,
    no_key::{
      datareader::DataReader as NoKeyDataReader, datawriter::DataWriter as NoKeyDataWriter,
    },
    participant::*,
    qos::*,
    result::{Error, Result},
    statusevents::{sync_status_channel, DataReaderStatus},
    topic::*,
    with_key,
    with_key::{
      datareader::DataReader as WithKeyDataReader, datawriter::DataWriter as WithKeyDataWriter,
    },
  },
  discovery::{
    discovery::DiscoveryCommand, discovery_db::DiscoveryDB, sedp_messages::DiscoveredWriterData,
  },
  log_and_err_internal, log_and_err_precondition_not_met, mio_source,
  rtps::{
    reader::ReaderIngredients,
    writer::{WriterCommand, WriterIngredients},
  },
  serialization::{cdr_deserializer::CDRDeserializerAdapter, cdr_serializer::CDRSerializerAdapter},
  structure::{
    entity::RTPSEntity,
    guid::{EntityId, EntityKind, GUID},
    topic_kind::TopicKind,
  },
};
use super::{
  no_key::wrappers::{DAWrapper, NoKeyWrapper, SAWrapper},
  with_key::simpledatareader::ReaderCommand,
};

// -------------------------------------------------------------------

/// DDS Publisher
///
/// The Publisher and Subscriber structures are collections of DataWriters
/// and, respectively, DataReaders. They can contain DataWriters or DataReaders
/// of different types, and attacehd to different Topics.
///
/// They can act as a domain of sample ordering or atomicity, if such QoS
/// policies are used. For example, DDS participants could agree via QoS
/// policies that data samples must be presented to readers in the same order as
/// writers have written them, and the ordering applies also between several
/// writers/readers, but within one publisher/subscriber. Analogous arrangement
/// can be set up w.r.t. coherency: All the samples in a transaction are
/// delivered to the readers, or none are. The transaction can span several
/// readers, writers, and topics in a single publisher/subscriber.
///
///
/// # Examples
///
/// ```
/// # use rustdds::DomainParticipant;
/// # use rustdds::qos::QosPolicyBuilder;
/// use rustdds::Publisher;
///
/// let domain_participant = DomainParticipant::new(0).unwrap();
/// let qos = QosPolicyBuilder::new().build();
///
/// let publisher = domain_participant.create_publisher(&qos);
/// ```
#[derive(Clone)]
pub struct Publisher {
  inner: Arc<Mutex<InnerPublisher>>,
}

impl Publisher {
  pub(super) fn new(
    dp: DomainParticipantWeak,
    discovery_db: Arc<RwLock<DiscoveryDB>>,
    qos: QosPolicies,
    default_dw_qos: QosPolicies,
    add_writer_sender: mio_channel::SyncSender<WriterIngredients>,
    remove_writer_sender: mio_channel::SyncSender<GUID>,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  ) -> Self {
    Self {
      inner: Arc::new(Mutex::new(InnerPublisher::new(
        dp,
        discovery_db,
        qos,
        default_dw_qos,
        add_writer_sender,
        remove_writer_sender,
        discovery_command,
      ))),
    }
  }

  fn inner_lock(&self) -> MutexGuard<'_, InnerPublisher> {
    self
      .inner
      .lock()
      .unwrap_or_else(|e| panic!("Inner publisher lock fail! {e:?}"))
  }

  /// Creates DDS [DataWriter](struct.With_Key_DataWriter.html) for Keyed topic
  ///
  /// # Arguments
  ///
  /// * `entity_id` - Custom entity id if necessary for the user to define it
  /// * `topic` - Reference to DDS Topic this writer is created to
  /// * `qos` - Not currently in use
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::*;
  /// use rustdds::serialization::CDRSerializerAdapter;
  /// use serde::Serialize;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  ///
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None);
  /// ```
  pub fn create_datawriter<D, SA>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataWriter<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::SerializerAdapter<D>,
  {
    self.inner_lock().create_datawriter(self, None, topic, qos)
  }

  /// Shorthand for crate_datawriter with Commaon Data Representation Little
  /// Endian
  pub fn create_datawriter_cdr<D>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataWriter<D, CDRSerializerAdapter<D, LittleEndian>>>
  where
    D: Keyed + serde::Serialize,
    <D as Keyed>::K: Key,
    <D as Keyed>::K: Serialize,
  {
    self.create_datawriter::<D, CDRSerializerAdapter<D, LittleEndian>>(topic, qos)
  }

  // versions with callee-specified EntityId. These are for Discovery use only.

  pub(crate) fn create_datawriter_with_entityid<D, SA>(
    &self,
    entity_id: EntityId,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataWriter<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::SerializerAdapter<D>,
  {
    self
      .inner_lock()
      .create_datawriter(self, Some(entity_id), topic, qos)
  }

  pub(crate) fn create_datawriter_cdr_with_entityid<D>(
    &self,
    entity_id: EntityId,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataWriter<D, CDRSerializerAdapter<D, LittleEndian>>>
  where
    D: Keyed + serde::Serialize,
    <D as Keyed>::K: Key,
    <D as Keyed>::K: Serialize,
  {
    self.create_datawriter_with_entityid::<D, CDRSerializerAdapter<D, LittleEndian>>(
      entity_id, topic, qos,
    )
  }

  /// Creates DDS [DataWriter](struct.DataWriter.html) for Nokey Topic
  ///
  /// # Arguments
  ///
  /// * `entity_id` - Custom entity id if necessary for the user to define it
  /// * `topic` - Reference to DDS Topic this writer is created to
  /// * `qos` - QoS policies for this DataWriter
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::*;
  /// use rustdds::serialization::CDRSerializerAdapter;
  /// use serde::Serialize;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  ///
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize)]
  /// struct SomeType {}
  ///
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter_no_key::<SomeType, CDRSerializerAdapter<_>>(&topic, None);
  /// ```
  pub fn create_datawriter_no_key<D, SA>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<NoKeyDataWriter<D, SA>>
  where
    SA: adapters::no_key::SerializerAdapter<D>,
  {
    self
      .inner_lock()
      .create_datawriter_no_key(self, None, topic, qos)
  }

  pub fn create_datawriter_no_key_cdr<D>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<NoKeyDataWriter<D, CDRSerializerAdapter<D, LittleEndian>>>
  where
    D: serde::Serialize,
  {
    self.create_datawriter_no_key::<D, CDRSerializerAdapter<D, LittleEndian>>(topic, qos)
  }

  // Versions with callee-specified EntityId. These are for Discovery use only.
  // ... except that Discovery has no use for no_key versions.

  // pub(crate) fn create_datawriter_no_key_with_entityid<D, SA>(
  //   &self,
  //   entity_id: EntityId,
  //   topic: &Topic,
  //   qos: Option<QosPolicies>,
  // ) -> Result<NoKeyDataWriter<D, SA>>
  // where
  //   D: Serialize,
  //   SA: no_key::SerializerAdapter<D>,
  // {
  //   self
  //     .inner_lock()
  //     .create_datawriter_no_key(self, Some(entity_id), topic, qos)
  // }

  // pub(crate) fn create_datawriter_no_key_cdr_with_entityid<D>(
  //   &self,
  //   entity_id: EntityId,
  //   topic: &Topic,
  //   qos: Option<QosPolicies>,
  // ) -> Result<NoKeyDataWriter<D, CDRSerializerAdapter<D, LittleEndian>>>
  // where
  //   D: Serialize,
  // {
  //   self.create_datawriter_no_key_with_entityid::<D, CDRSerializerAdapter<D,
  // LittleEndian>>(     entity_id, topic, qos,
  //   )
  // }

  // delete_datawriter should not be needed. The DataWriter object itself should
  // be deleted to accomplish this.

  // lookup datawriter: maybe not necessary? App should remember datawriters it
  // has created.

  // Suspend and resume publications are preformance optimization methods.
  // The minimal correct implementation is to do nothing. See DDS spec 2.2.2.4.1.8
  // and .9
  /// **NOT IMPLEMENTED. DO NOT USE**
  #[deprecated(note = "unimplemented")]
  pub fn suspend_publications(&self) -> Result<()> {
    unimplemented!();
  }

  /// **NOT IMPLEMENTED. DO NOT USE**
  #[deprecated(note = "unimplemented")]
  pub fn resume_publications(&self) -> Result<()> {
    unimplemented!();
  }

  // coherent change set
  // In case such QoS is not supported, these should be no-ops.
  // TODO: Implement these when coherent change-sets are supported.
  // Coherent set not implemented and currently does nothing
  /// **NOT IMPLEMENTED. DO NOT USE**
  #[deprecated(note = "unimplemented")]
  pub fn begin_coherent_changes(&self) -> Result<()> {
    unimplemented!();
  }

  // Coherent set not implemented and currently does nothing
  /// **NOT IMPLEMENTED. DO NOT USE**
  #[deprecated(note = "unimplemented")]
  pub fn end_coherent_changes(&self) -> Result<()> {
    unimplemented!();
  }

  // Wait for all matched reliable DataReaders acknowledge data written so far,
  // or timeout.
  /// **NOT IMPLEMENTED. DO NOT USE**
  #[deprecated(note = "unimplemented")]
  pub fn wait_for_acknowledgments(&self, _max_wait: Duration) -> Result<()> {
    unimplemented!();
  }

  // What is the use case for this? (is it useful in Rust style of programming?
  // Should it be public?)
  /// Gets [DomainParticipant](struct.DomainParticipant.html) if it has not
  /// disappeared from all scopes.
  ///
  /// # Example
  ///
  /// ```
  /// # use rustdds::*;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  ///
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  /// assert_eq!(domain_participant, publisher.participant().unwrap());
  /// ```
  pub fn participant(&self) -> Option<DomainParticipant> {
    self.inner_lock().domain_participant.clone().upgrade()
  }

  // delete_contained_entities: We should not need this. Contained DataWriters
  // should dispose themselves and notify publisher.

  /// Returns default DataWriter qos.
  ///
  /// # Example
  ///
  /// ```
  /// # use rustdds::*;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  ///
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  /// assert_eq!(qos, publisher.get_default_datawriter_qos());
  /// ```
  pub fn get_default_datawriter_qos(&self) -> QosPolicies {
    self.inner_lock().get_default_datawriter_qos().clone()
  }

  /// Sets default DataWriter qos.
  ///
  /// # Example
  ///
  /// ```
  /// # use rustdds::*;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  ///
  /// let mut publisher = domain_participant.create_publisher(&qos).unwrap();
  /// let qos2 =
  /// QosPolicyBuilder::new().durability(policy::Durability::Transient).build();
  /// publisher.set_default_datawriter_qos(&qos2);
  ///
  /// assert_ne!(qos, publisher.get_default_datawriter_qos());
  /// assert_eq!(qos2, publisher.get_default_datawriter_qos());
  /// ```
  pub fn set_default_datawriter_qos(&mut self, q: &QosPolicies) {
    self.inner_lock().set_default_datawriter_qos(q);
  }

  // This is used on DataWriter .drop()
  pub(crate) fn remove_writer(&self, guid: GUID) {
    self.inner_lock().remove_writer(guid);
  }
} // impl

impl PartialEq for Publisher {
  fn eq(&self, other: &Self) -> bool {
    let id_self = { self.inner_lock().identity() };
    let id_other = { other.inner_lock().identity() };
    id_self == id_other
  }
}

impl Debug for Publisher {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    self.inner_lock().fmt(f)
  }
}

// "Inner" struct

#[derive(Clone)]
struct InnerPublisher {
  id: EntityId,
  domain_participant: DomainParticipantWeak,
  discovery_db: Arc<RwLock<DiscoveryDB>>,
  my_qos_policies: QosPolicies,
  default_datawriter_qos: QosPolicies, // used when creating a new DataWriter
  add_writer_sender: mio_channel::SyncSender<WriterIngredients>,
  remove_writer_sender: mio_channel::SyncSender<GUID>,
  discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
}

// public interface for Publisher
impl InnerPublisher {
  fn new(
    dp: DomainParticipantWeak,
    discovery_db: Arc<RwLock<DiscoveryDB>>,
    qos: QosPolicies,
    default_dw_qos: QosPolicies,
    add_writer_sender: mio_channel::SyncSender<WriterIngredients>,
    remove_writer_sender: mio_channel::SyncSender<GUID>,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  ) -> Self {
    // We generate an arbitrary but unique id to distiguish Publishers from each
    // other. EntityKind is just some value, since we do not show it to anyone.
    let id = EntityId::MAX;
    //dp.clone().upgrade().unwrap().new_entity_id(EntityKind::UNKNOWN_BUILT_IN);

    Self {
      id,
      domain_participant: dp,
      discovery_db,
      my_qos_policies: qos,
      default_datawriter_qos: default_dw_qos,
      add_writer_sender,
      remove_writer_sender,
      discovery_command,
    }
  }

  pub fn create_datawriter<D, SA>(
    &self,
    outer: &Publisher,
    entity_id_opt: Option<EntityId>,
    topic: &Topic,
    optional_qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataWriter<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::SerializerAdapter<D>,
  {
    // Data samples from DataWriter to HistoryCache
    let (dwcc_upload, hccc_download) = mio_channel::sync_channel::<WriterCommand>(16);
    let writer_waker = Arc::new(Mutex::new(None));
    // Status reports back from Writer to DataWriter.
    let (status_sender, status_receiver) = sync_status_channel(4)?;

    // DDS Spec 2.2.2.4.1.5 create_datawriter:
    // If no QoS is specified, we should take the Publisher default
    // QoS, modify it to match any QoS settings (that are set) in the
    // Topic QoS and use that.

    // Use Publisher QoS as basis, modify by Topic settings, and modify by specified
    // QoS.
    let writer_qos = self
      .default_datawriter_qos
      .modify_by(&topic.qos())
      .modify_by(&optional_qos.unwrap_or_else(QosPolicies::qos_none));

    let entity_id =
      self.unwrap_or_new_entity_id(entity_id_opt, EntityKind::WRITER_WITH_KEY_USER_DEFINED);
    let dp = self
      .participant()
      .ok_or("upgrade fail")
      .or_else(|e| log_and_err_internal!("Where is my DomainParticipant? {}", e))?;

    // Create a new topic to DDScache if it doesn't exist and get a handle to it
    let topic_cache_handle = match dp.dds_cache().write() {
      Ok(mut dds_cache) => dds_cache.add_new_topic(topic.name(), topic.get_type(), &writer_qos),
      Err(e) => return log_and_err_internal!("Cannot lock DDScache. Error: {}", e),
    };

    let guid = GUID::new_with_prefix_and_id(dp.guid().prefix, entity_id);

    let new_writer = WriterIngredients {
      guid,
      writer_command_receiver: hccc_download,
      writer_command_receiver_waker: Arc::clone(&writer_waker),
      topic_name: topic.name(),
      topic_cache_handle,
      qos_policies: writer_qos.clone(),
      status_sender,
    };

    self
      .add_writer_sender
      .send(new_writer)
      .or_else(|e| log_and_err_internal!("Adding a new writer failed: {}", e))?;

    let data_writer = WithKeyDataWriter::<D, SA>::new(
      outer.clone(),
      topic.clone(),
      writer_qos,
      guid,
      dwcc_upload,
      writer_waker,
      self.discovery_command.clone(),
      status_receiver,
    )?;

    // notify Discovery DB
    let mut db = self.discovery_db.write()?;
    // TODO: "None" below hardwires security_info to None. So we do not publish any.
    let dwd = DiscoveredWriterData::new(&data_writer, topic, &dp, None);
    db.update_local_topic_writer(dwd);
    db.update_topic_data_p(topic);

    Ok(data_writer)
  }

  pub fn create_datawriter_no_key<D, SA>(
    &self,
    outer: &Publisher,
    entity_id_opt: Option<EntityId>,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<NoKeyDataWriter<D, SA>>
  where
    SA: adapters::no_key::SerializerAdapter<D>,
  {
    let entity_id =
      self.unwrap_or_new_entity_id(entity_id_opt, EntityKind::WRITER_NO_KEY_USER_DEFINED);
    let d = self.create_datawriter::<NoKeyWrapper<D>, SAWrapper<SA>>(
      outer,
      Some(entity_id),
      topic,
      qos,
    )?;
    Ok(NoKeyDataWriter::<D, SA>::from_keyed(d))
  }

  pub fn participant(&self) -> Option<DomainParticipant> {
    self.domain_participant.clone().upgrade()
  }

  pub fn get_default_datawriter_qos(&self) -> &QosPolicies {
    &self.default_datawriter_qos
  }

  pub fn set_default_datawriter_qos(&mut self, q: &QosPolicies) {
    self.default_datawriter_qos = q.clone();
  }

  fn unwrap_or_new_entity_id(
    &self,
    entity_id_opt: Option<EntityId>,
    entity_kind: EntityKind,
  ) -> EntityId {
    // If the entity_id is given, then just use that. If not, then pull an arbirtaty
    // number out of participant's hat.
    entity_id_opt.unwrap_or_else(|| self.participant().unwrap().new_entity_id(entity_kind))
  }

  pub(crate) fn remove_writer(&self, guid: GUID) {
    self
      .remove_writer_sender
      .try_send(guid)
      .unwrap_or_else(|e| error!("Cannot remove Writer {:?} : {:?}", guid, e));
  }

  pub(crate) fn identity(&self) -> EntityId {
    self.id
  }
}

impl Debug for InnerPublisher {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    f.write_fmt(format_args!("{:?}", self.participant()))?;
    f.write_fmt(format_args!("Publisher QoS: {:?}", self.my_qos_policies))?;
    f.write_fmt(format_args!(
      "Publishers default Writer QoS: {:?}",
      self.default_datawriter_qos
    ))
  }
}

// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------

/// DDS Subscriber
///
/// See overview at [`Publisher`].
///
/// # Examples
///
/// ```
/// # use rustdds::*;
///
/// let domain_participant = DomainParticipant::new(0).unwrap();
/// let qos = QosPolicyBuilder::new().build();
///
/// let subscriber = domain_participant.create_subscriber(&qos);
/// ```
#[derive(Clone)]
pub struct Subscriber {
  inner: Arc<InnerSubscriber>,
}

impl Subscriber {
  pub(super) fn new(
    domain_participant: DomainParticipantWeak,
    discovery_db: Arc<RwLock<DiscoveryDB>>,
    qos: QosPolicies,
    sender_add_reader: mio_channel::SyncSender<ReaderIngredients>,
    sender_remove_reader: mio_channel::SyncSender<GUID>,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  ) -> Self {
    Self {
      inner: Arc::new(InnerSubscriber::new(
        domain_participant,
        discovery_db,
        qos,
        sender_add_reader,
        sender_remove_reader,
        discovery_command,
      )),
    }
  }

  /// Creates DDS DataReader for keyed Topics
  ///
  /// # Arguments
  ///
  /// * `topic` - Reference to the DDS [Topic](struct.Topic.html) this reader
  ///   reads from
  /// * `entity_id` - Optional [EntityId](data_types/struct.EntityId.html) if
  ///   necessary for DDS communication (random if None)
  /// * `qos` - Not in use
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::*;
  /// use serde::Deserialize;
  /// use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// #
  ///
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  ///
  /// #[derive(Deserialize)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None);
  /// ```
  pub fn create_datareader<D: 'static, SA>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataReader<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::DeserializerAdapter<D>,
  {
    self.inner.create_datareader(self, topic, None, qos)
  }

  pub fn create_datareader_cdr<D: 'static>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataReader<D, CDRDeserializerAdapter<D>>>
  where
    D: serde::de::DeserializeOwned + Keyed,
    <D as Keyed>::K: Key,
    for<'de> <D as Keyed>::K: Deserialize<'de>,
  {
    self.create_datareader::<D, CDRDeserializerAdapter<D>>(topic, qos)
  }

  // versions with callee-specified EntityId. These are for Discovery use only.

  pub(crate) fn create_datareader_with_entityid<D: 'static, SA>(
    &self,
    topic: &Topic,
    entity_id: EntityId,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataReader<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::DeserializerAdapter<D>,
  {
    self
      .inner
      .create_datareader(self, topic, Some(entity_id), qos)
  }

  pub(crate) fn create_datareader_cdr_with_entityid<D: 'static>(
    &self,
    topic: &Topic,
    entity_id: EntityId,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataReader<D, CDRDeserializerAdapter<D>>>
  where
    D: serde::de::DeserializeOwned + Keyed,
    <D as Keyed>::K: Key,
    for<'de> <D as Keyed>::K: Deserialize<'de>,
  {
    self.create_datareader_with_entityid::<D, CDRDeserializerAdapter<D>>(topic, entity_id, qos)
  }

  /// Create DDS DataReader for non keyed Topics
  ///
  /// # Arguments
  ///
  /// * `topic` - Reference to the DDS [Topic](struct.Topic.html) this reader
  ///   reads from
  /// * `entity_id` - Optional [EntityId](data_types/struct.EntityId.html) if
  ///   necessary for DDS communication (random if None)
  /// * `qos` - Not in use
  ///
  /// # Examples
  ///
  /// ```
  /// # use rustdds::*;
  /// use serde::Deserialize;
  /// use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// #
  ///
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  ///
  /// #[derive(Deserialize)]
  /// struct SomeType {}
  ///
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// let data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None);
  /// ```
  pub fn create_datareader_no_key<D: 'static, SA>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<NoKeyDataReader<D, SA>>
  where
    SA: adapters::no_key::DeserializerAdapter<D>,
  {
    self.inner.create_datareader_no_key(self, topic, None, qos)
  }

  pub fn create_simple_datareader_no_key<D: 'static, SA: 'static>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<no_key::SimpleDataReader<D, SA>>
  where
    SA: adapters::no_key::DeserializerAdapter<D>,
  {
    self
      .inner
      .create_simple_datareader_no_key(self, topic, None, qos)
  }

  pub fn create_datareader_no_key_cdr<D: 'static>(
    &self,
    topic: &Topic,
    qos: Option<QosPolicies>,
  ) -> Result<NoKeyDataReader<D, CDRDeserializerAdapter<D>>>
  where
    D: serde::de::DeserializeOwned,
  {
    self.create_datareader_no_key::<D, CDRDeserializerAdapter<D>>(topic, qos)
  }

  // Exists for symmetry, but not really neeeded,
  // as the only user is Discovery.
  // pub(crate) fn create_datareader_no_key_with_entityid<D: 'static, SA>(
  //   &self,
  //   topic: &Topic,
  //   entity_id: EntityId,
  //   qos: Option<QosPolicies>,
  // ) -> Result<NoKeyDataReader<D, SA>>
  // where
  //   D: DeserializeOwned,
  //   SA: no_key::DeserializerAdapter<D>,
  // {
  //   self
  //     .inner
  //     .create_datareader_no_key(self, topic, Some(entity_id), qos)
  // }

  // Exists for symmetry, but not really neeeded,
  // as the only user is Discovery.
  // pub(crate) fn create_datareader_no_key_cdr_with_entityid<D: 'static>(
  //   &self,
  //   topic: &Topic,
  //   entity_id: EntityId,
  //   qos: Option<QosPolicies>,
  // ) -> Result<NoKeyDataReader<D, CDRDeserializerAdapter<D>>>
  // where
  //   D: DeserializeOwned,
  // {
  //   self
  //     .create_datareader_no_key_with_entityid::<D,
  // CDRDeserializerAdapter<D>>(topic, entity_id, qos) }

  // Retrieves a previously created DataReader belonging to the Subscriber.
  // TODO: Is this even possible. Whould probably need to return reference and
  // store references on creation
  /*
  pub(crate) fn lookup_datareader<D, SA>(
    &self,
    _topic_name: &str,
  ) -> Option<WithKeyDataReader<D, SA>>
  where
    D: Keyed + DeserializeOwned,
    SA: DeserializerAdapter<D>,
  {
    todo!()
    // TO think: Is this really necessary? Because the caller would have to know
    // types D and SA. Should we just trust whoever creates DataReaders to also remember them?
  }
  */

  /// Returns [DomainParticipant](struct.DomainParticipant.html) if it is sill
  /// alive.
  ///
  /// # Example
  ///
  /// ```
  /// # use rustdds::*;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  ///
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// assert_eq!(domain_participant, subscriber.participant().unwrap());
  /// ```
  pub fn participant(&self) -> Option<DomainParticipant> {
    self.inner.participant()
  }

  pub(crate) fn remove_reader(&self, guid: GUID) {
    self.inner.remove_reader(guid);
  }
}

#[derive(Clone)]
pub struct InnerSubscriber {
  domain_participant: DomainParticipantWeak,
  discovery_db: Arc<RwLock<DiscoveryDB>>,
  qos: QosPolicies,
  sender_add_reader: mio_channel::SyncSender<ReaderIngredients>,
  sender_remove_reader: mio_channel::SyncSender<GUID>,
  discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
}

impl InnerSubscriber {
  pub(super) fn new(
    domain_participant: DomainParticipantWeak,
    discovery_db: Arc<RwLock<DiscoveryDB>>,
    qos: QosPolicies,
    sender_add_reader: mio_channel::SyncSender<ReaderIngredients>,
    sender_remove_reader: mio_channel::SyncSender<GUID>,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  ) -> Self {
    Self {
      domain_participant,
      discovery_db,
      qos,
      sender_add_reader,
      sender_remove_reader,
      discovery_command,
    }
  }

  fn create_datareader_internal<D: 'static, SA>(
    &self,
    outer: &Subscriber,
    entity_id_opt: Option<EntityId>,
    topic: &Topic,
    optional_qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataReader<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::DeserializerAdapter<D>,
  {
    let simple_dr =
      self.create_simple_datareader_internal(outer, entity_id_opt, topic, optional_qos)?;
    Ok(with_key::DataReader::<D, SA>::from_simple_data_reader(
      simple_dr,
    ))
  }

  fn create_simple_datareader_internal<D: 'static, SA>(
    &self,
    outer: &Subscriber,
    entity_id_opt: Option<EntityId>,
    topic: &Topic,
    optional_qos: Option<QosPolicies>,
  ) -> Result<with_key::SimpleDataReader<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::DeserializerAdapter<D>,
  {
    // incoming data notification channel from Reader to DataReader
    let (send, rec) = mio_channel::sync_channel::<()>(4);
    // status change channel from Reader to DataReader
    let (status_sender, status_receiver) = sync_status_channel::<DataReaderStatus>(4)?;

    // reader command channel from Datareader to Reader
    let (reader_command_sender, reader_command_receiver) =
      mio_channel::sync_channel::<ReaderCommand>(0);
    // The buffer length is zero, i.e. sender and receiver must rendezvous at
    // send/receive. This is needed to synchronize sending of wakers from
    // DataReader to Reader. If the capacity is increased, then some data
    // available for reading notifications may be missed.

    // Use subscriber QoS as basis, modify by Topic settings, and modify by
    // specified QoS.
    let qos = self
      .qos
      .modify_by(&topic.qos())
      .modify_by(&optional_qos.unwrap_or_else(QosPolicies::qos_none));

    let entity_id =
      self.unwrap_or_new_entity_id(entity_id_opt, EntityKind::READER_WITH_KEY_USER_DEFINED);

    let dp = match self.participant() {
      Some(dp) => dp,
      None => return log_and_err_precondition_not_met!("DomainParticipant doesn't exist anymore."),
    };

    // Create a new topic to DDScache if it doesn't exist and get a handle to it
    let topic_cache_handle = match dp.dds_cache().write() {
      Ok(mut dds_cache) => dds_cache.add_new_topic(topic.name(), topic.get_type(), &qos),
      Err(e) => return log_and_err_internal!("Cannot lock DDScache. Error: {}", e),
    };

    let reader_guid = GUID::new_with_prefix_and_id(dp.guid_prefix(), entity_id);

    let data_reader_waker = Arc::new(Mutex::new(None));

    let (poll_event_source, poll_event_sender) = mio_source::make_poll_channel()?;

    let new_reader = ReaderIngredients {
      guid: reader_guid,
      notification_sender: send,
      status_sender,
      topic_name: topic.name(),
      topic_cache_handle: topic_cache_handle.clone(),
      qos_policy: qos.clone(),
      data_reader_command_receiver: reader_command_receiver,
      data_reader_waker: data_reader_waker.clone(),
      poll_event_sender,
    };

    {
      let mut db = self
        .discovery_db
        .write()
        .or_else(|e| log_and_err_internal!("Cannot lock discovery_db. {}", e))?;
      db.update_local_topic_reader(&dp, topic, &new_reader);
      db.update_topic_data_p(topic);
    }

    let datareader = with_key::SimpleDataReader::<D, SA>::new(
      outer.clone(),
      entity_id,
      topic.clone(),
      qos,
      rec,
      topic_cache_handle,
      self.discovery_command.clone(),
      status_receiver,
      reader_command_sender,
      data_reader_waker,
      poll_event_source,
    )?;

    // Return the DataReader Reader pairs to where they are used
    self
      .sender_add_reader
      .try_send(new_reader)
      .or_else(|e| log_and_err_internal!("Cannot add DataReader. Error: {}", e))?;

    Ok(datareader)
  }

  pub fn create_datareader<D: 'static, SA>(
    &self,
    outer: &Subscriber,
    topic: &Topic,
    entity_id: Option<EntityId>,
    qos: Option<QosPolicies>,
  ) -> Result<WithKeyDataReader<D, SA>>
  where
    D: Keyed,
    <D as Keyed>::K: Key,
    SA: adapters::with_key::DeserializerAdapter<D>,
  {
    if topic.kind() != TopicKind::WithKey {
      return Error::precondition_not_met(
        "Topic is NO_KEY, but attempted to create WITH_KEY Datareader",
      );
    }
    self.create_datareader_internal(outer, entity_id, topic, qos)
  }

  pub fn create_datareader_no_key<D: 'static, SA>(
    &self,
    outer: &Subscriber,
    topic: &Topic,
    entity_id_opt: Option<EntityId>,
    qos: Option<QosPolicies>,
  ) -> Result<NoKeyDataReader<D, SA>>
  where
    SA: adapters::no_key::DeserializerAdapter<D>,
  {
    if topic.kind() != TopicKind::NoKey {
      return Error::precondition_not_met(
        "Topic is WITH_KEY, but attempted to create NO_KEY Datareader",
      );
    }

    let entity_id =
      self.unwrap_or_new_entity_id(entity_id_opt, EntityKind::READER_NO_KEY_USER_DEFINED);

    let d = self.create_datareader_internal::<NoKeyWrapper<D>, DAWrapper<SA>>(
      outer,
      Some(entity_id),
      topic,
      qos,
    )?;

    Ok(NoKeyDataReader::<D, SA>::from_keyed(d))
  }

  pub fn create_simple_datareader_no_key<D: 'static, SA>(
    &self,
    outer: &Subscriber,
    topic: &Topic,
    entity_id_opt: Option<EntityId>,
    qos: Option<QosPolicies>,
  ) -> Result<no_key::SimpleDataReader<D, SA>>
  where
    SA: adapters::no_key::DeserializerAdapter<D> + 'static,
  {
    if topic.kind() != TopicKind::NoKey {
      return Error::precondition_not_met(
        "Topic is WITH_KEY, but attempted to create NO_KEY Datareader",
      );
    }

    let entity_id =
      self.unwrap_or_new_entity_id(entity_id_opt, EntityKind::READER_NO_KEY_USER_DEFINED);

    let d = self.create_simple_datareader_internal::<NoKeyWrapper<D>, DAWrapper<SA>>(
      outer,
      Some(entity_id),
      topic,
      qos,
    )?;

    Ok(no_key::SimpleDataReader::<D, SA>::from_keyed(d))
  }

  pub fn participant(&self) -> Option<DomainParticipant> {
    self.domain_participant.clone().upgrade()
  }

  pub(crate) fn remove_reader(&self, guid: GUID) {
    self
      .sender_remove_reader
      .try_send(guid)
      .unwrap_or_else(|e| error!("Cannot remove Reader {:?} : {:?}", guid, e));
  }

  fn unwrap_or_new_entity_id(
    &self,
    entity_id_opt: Option<EntityId>,
    entity_kind: EntityKind,
  ) -> EntityId {
    // If the entity_id is given, then just use that. If not, then pull an arbirtaty
    // number out of participant's hat.
    entity_id_opt.unwrap_or_else(|| self.participant().unwrap().new_entity_id(entity_kind))
  }
}

// -------------------------------------------------------------------

#[cfg(test)]
mod tests {}
