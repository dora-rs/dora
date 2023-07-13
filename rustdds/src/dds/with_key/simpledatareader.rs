use std::{
  cmp::max,
  collections::BTreeMap,
  io,
  marker::PhantomData,
  pin::Pin,
  sync::{Arc, Mutex, MutexGuard},
  task::{Context, Poll, Waker},
};

use futures::stream::{FusedStream, Stream};
use serde::de::DeserializeOwned;
use mio_extras::channel as mio_channel;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use mio_06::{self, Evented};
use mio_08;

use crate::{
  dds::{
    adapters::with_key::*,
    ddsdata::*,
    key::*,
    pubsub::Subscriber,
    qos::*,
    result::*,
    statusevents::*,
    topic::{Topic, TopicDescription},
    with_key::datasample::{DeserializedCacheChange, Sample},
  },
  discovery::discovery::DiscoveryCommand,
  mio_source::PollEventSource,
  serialization::CDRDeserializerAdapter,
  structure::{
    cache_change::CacheChange,
    dds_cache::TopicCache,
    entity::RTPSEntity,
    guid::{EntityId, GUID},
    sequence_number::SequenceNumber,
    time::Timestamp,
  },
};

#[derive(Clone, Debug)]
pub(crate) enum ReaderCommand {
  #[allow(dead_code)] // TODO: Implement this (resetting) feature
  ResetRequestedDeadlineStatus,
}

// This is helper struct.
// Most mutable state needed for reading should go here.
pub(crate) struct ReadState<K: Key> {
  latest_instant: Timestamp, /* This is used as a read pointer from dds_cache for BEST_EFFORT
                              * reading */
  /// hash_to_key_map is used for decoding received key hashes back to original
  /// key values. This is needed when we receive a dispose message via hash
  /// only.
  hash_to_key_map: BTreeMap<KeyHash, K>, // TODO: garbage collect this somehow
}

impl<K: Key> ReadState<K> {
  fn new() -> Self {
    ReadState {
      latest_instant: Timestamp::ZERO,
      hash_to_key_map: BTreeMap::<KeyHash, K>::new(),
    }
  }
}

/// SimpleDataReaders can only do "take" semantics and does not have
/// any deduplication or other DataSampleCache functionality.
pub struct SimpleDataReader<D: Keyed, DA: DeserializerAdapter<D> = CDRDeserializerAdapter<D>>
where
  <D as Keyed>::K: Key,
{
  #[allow(dead_code)] // TODO: This is currently unused, because we do not implement
  // any subscriber-wide QoS policies, such as ordered or coherent access.
  // Remove this attribute when/if such things are implemented.
  my_subscriber: Subscriber,

  my_topic: Topic,
  qos_policy: QosPolicies,
  my_guid: GUID,
  pub(crate) notification_receiver: mio_channel::Receiver<()>,

  // SimpleDataReader stores a pointer to a mutex on the topic cache
  topic_cache: Arc<Mutex<TopicCache>>,

  // collection of read pointers for RELIABLE reading
  last_read_sequence_number_ref: Arc<Mutex<BTreeMap<GUID, SequenceNumber>>>,

  read_state: Mutex<ReadState<<D as Keyed>::K>>,

  deserializer_type: PhantomData<DA>, // This is to provide use for DA

  discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  status_receiver: StatusReceiver<DataReaderStatus>,

  #[allow(dead_code)] // TODO: This is currently unused, because we do not implement
  // resetting deadline missed status. Remove attribute when it is supported.
  reader_command: mio_channel::SyncSender<ReaderCommand>,
  data_reader_waker: Arc<Mutex<Option<Waker>>>,

  event_source: PollEventSource,
}

impl<D, DA> Drop for SimpleDataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn drop(&mut self) {
    // Tell dp_event_loop
    self.my_subscriber.remove_reader(self.my_guid);

    // Tell discovery
    match self
      .discovery_command
      .send(DiscoveryCommand::RemoveLocalReader { guid: self.my_guid })
    {
      Ok(_) => {}
      Err(mio_channel::SendError::Disconnected(_)) => {
        debug!("Failed to send DiscoveryCommand::RemoveLocalReader . Maybe shutting down?");
      }
      Err(e) => error!(
        "Failed to send DiscoveryCommand::RemoveLocalReader. {:?}",
        e
      ),
    }
  }
}

impl<D: 'static, DA> SimpleDataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  #[allow(clippy::too_many_arguments)]
  pub(crate) fn new(
    subscriber: Subscriber,
    my_id: EntityId,
    topic: Topic,
    qos_policy: QosPolicies,
    // Each notification sent to this channel must be try_recv'd
    notification_receiver: mio_channel::Receiver<()>,
    topic_cache: Arc<Mutex<TopicCache>>,
    last_read_sequence_number_ref: Arc<Mutex<BTreeMap<GUID, SequenceNumber>>>,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
    status_channel_rec: StatusChannelReceiver<DataReaderStatus>,
    reader_command: mio_channel::SyncSender<ReaderCommand>,
    data_reader_waker: Arc<Mutex<Option<Waker>>>,
    event_source: PollEventSource,
  ) -> CreateResult<Self> {
    let dp = match subscriber.participant() {
      Some(dp) => dp,
      None => {
        return Err(CreateError::ResourceDropped {
          reason: "Cannot create new DataReader, DomainParticipant doesn't exist.".to_string(),
        })
      }
    };

    let my_guid = GUID::new_with_prefix_and_id(dp.guid_prefix(), my_id);

    // Verify that the topic cache corresponds to the topic of the Reader
    let topic_cache_name = topic_cache.lock().unwrap().topic_name();
    if topic.name() != topic_cache_name {
      return Err(CreateError::Internal {
        reason: format!(
          "Topic name = {} and topic cache name = {} not equal when creating a SimpleDataReader",
          topic.name(),
          topic_cache_name
        ),
      });
    }

    Ok(Self {
      my_subscriber: subscriber,
      qos_policy,
      my_guid,
      notification_receiver,
      topic_cache,
      last_read_sequence_number_ref,
      read_state: Mutex::new(ReadState::new()),
      my_topic: topic,
      deserializer_type: PhantomData,
      discovery_command,
      status_receiver: StatusReceiver::new(status_channel_rec),
      reader_command,
      data_reader_waker,
      event_source,
    })
  }
  pub fn set_waker(&self, w: Option<Waker>) {
    *self.data_reader_waker.lock().unwrap() = w;
  }

  pub(crate) fn drain_read_notifications(&self) {
    while self.notification_receiver.try_recv().is_ok() {}
    self.event_source.drain();
  }

  fn try_take_undecoded<'a>(
    is_reliable: bool,
    topic_cache: &'a TopicCache,
    latest_instant: Timestamp,
    last_read_sn: &'a BTreeMap<GUID, SequenceNumber>,
  ) -> Box<dyn Iterator<Item = (Timestamp, &'a CacheChange)> + 'a> {
    if is_reliable {
      topic_cache.get_changes_in_range_reliable(last_read_sn)
    } else {
      topic_cache.get_changes_in_range_best_effort(latest_instant, Timestamp::now())
    }
  }

  fn update_hash_to_key_map(
    hash_to_key_map: &mut BTreeMap<KeyHash, D::K>,
    deserialized: &Sample<D, D::K>,
  ) {
    let instance_key = match deserialized {
      Sample::Value(d) => d.key(),
      Sample::Dispose(k) => k.clone(),
    };
    hash_to_key_map.insert(instance_key.hash_key(), instance_key);
  }

  fn deserialize(
    timestamp: Timestamp,
    cc: &CacheChange,
    hash_to_key_map: &mut BTreeMap<KeyHash, D::K>,
  ) -> ReadResult<DeserializedCacheChange<D>> {
    match cc.data_value {
      DDSData::Data {
        ref serialized_payload,
      } => {
        // what is our data serialization format (representation identifier) ?
        if let Some(recognized_rep_id) = DA::supported_encodings()
          .iter()
          .find(|r| **r == serialized_payload.representation_identifier)
        {
          match DA::from_bytes(&serialized_payload.value, *recognized_rep_id) {
            // Data update, decoded ok
            Ok(payload) => {
              let p = Sample::Value(payload);
              Self::update_hash_to_key_map(hash_to_key_map, &p);
              Ok(DeserializedCacheChange::new(timestamp, cc, p))
            }
            Err(e) => Err(ReadError::Deserialization {
              reason: format!("Failed to deserialize sample bytes: {e}, "),
            }),
          }
        } else {
          Err(ReadError::Deserialization {
            reason: format!(
              "Unknown representation id {:?}.",
              serialized_payload.representation_identifier
            ),
          })
        }
      }

      DDSData::DisposeByKey {
        key: ref serialized_key,
        ..
      } => {
        match DA::key_from_bytes(
          &serialized_key.value,
          serialized_key.representation_identifier,
        ) {
          Ok(key) => {
            let k = Sample::Dispose(key);
            Self::update_hash_to_key_map(hash_to_key_map, &k);
            Ok(DeserializedCacheChange::new(timestamp, cc, k))
          }
          Err(e) => Err(ReadError::Deserialization {
            reason: format!("Failed to deserialize key {}", e),
          }),
        }
      }

      DDSData::DisposeByKeyHash { key_hash, .. } => {
        // The cache should know hash -> key mapping even if the sample
        // has been disposed or .take()n
        if let Some(key) = hash_to_key_map.get(&key_hash) {
          Ok(DeserializedCacheChange::new(
            timestamp,
            cc,
            Sample::Dispose(key.clone()),
          ))
        } else {
          Err(ReadError::Deserialization {
            reason: format!("Tried to dispose with unknown key hash: {:x?}", key_hash),
          })
        }
      }
    } // match
  }

  /// Note: Always remember to call .drain_read_notifications() just before
  /// calling this one. Otherwise, new notifications may not appear.
  pub fn try_take_one(&self) -> ReadResult<Option<DeserializedCacheChange<D>>> {
    let is_reliable = matches!(
      self.qos_policy.reliability(),
      Some(policy::Reliability::Reliable { .. })
    );

    let topic_cache = self.acquire_the_topic_cache_guard();

    let mut read_state_ref = self.read_state.lock().unwrap();
    let latest_instant = read_state_ref.latest_instant;
    let mut last_read_sequence_number = self.last_read_sequence_number_ref.lock().unwrap();
    let hash_to_key_map = &mut read_state_ref.hash_to_key_map;
    let (timestamp, cc) = match Self::try_take_undecoded(
      is_reliable,
      &topic_cache,
      latest_instant,
      &last_read_sequence_number,
    )
    .next()
    {
      None => return Ok(None),
      Some((ts, cc)) => (ts, cc),
    };

    match Self::deserialize(timestamp, cc, hash_to_key_map) {
      Ok(dcc) => {
        read_state_ref.latest_instant = max(read_state_ref.latest_instant, timestamp);
        last_read_sequence_number.insert(dcc.writer_guid, dcc.sequence_number);
        Ok(Some(dcc))
      }
      Err(ser_err) => Err(ReadError::Deserialization {
        reason: format!(
          "{}, Topic = {}, Type = {:?}",
          ser_err,
          self.my_topic.name(),
          self.my_topic.get_type()
        ),
      }),
    }
  }

  pub fn qos(&self) -> &QosPolicies {
    &self.qos_policy
  }

  pub fn guid(&self) -> GUID {
    self.my_guid
  }

  pub fn topic(&self) -> &Topic {
    &self.my_topic
  }

  pub fn as_async_stream(&self) -> SimpleDataReaderStream<D, DA> {
    SimpleDataReaderStream {
      simple_datareader: self,
    }
  }

  pub fn as_simple_data_reader_event_stream(&self) -> SimpleDataReaderEventStream<D, DA> {
    SimpleDataReaderEventStream {
      simple_datareader: self,
    }
  }

  fn acquire_the_topic_cache_guard(&self) -> MutexGuard<TopicCache> {
    self.topic_cache.lock().unwrap_or_else(|e| {
      panic!(
        "The topic cache of topic {} is poisoned. Error: {}",
        &self.my_topic.name(),
        e
      )
    })
  }
}

// This is  not part of DDS spec. We implement mio Evented so that the
// application can asynchronously poll DataReader(s).
impl<D, DA> Evented for SimpleDataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  // We just delegate all the operations to notification_receiver, since it
  // already implements Evented
  fn register(
    &self,
    poll: &mio_06::Poll,
    token: mio_06::Token,
    interest: mio_06::Ready,
    opts: mio_06::PollOpt,
  ) -> io::Result<()> {
    self
      .notification_receiver
      .register(poll, token, interest, opts)
  }

  fn reregister(
    &self,
    poll: &mio_06::Poll,
    token: mio_06::Token,
    interest: mio_06::Ready,
    opts: mio_06::PollOpt,
  ) -> io::Result<()> {
    self
      .notification_receiver
      .reregister(poll, token, interest, opts)
  }

  fn deregister(&self, poll: &mio_06::Poll) -> io::Result<()> {
    self.notification_receiver.deregister(poll)
  }
}

impl<D, DA> mio_08::event::Source for SimpleDataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn register(
    &mut self,
    registry: &mio_08::Registry,
    token: mio_08::Token,
    interests: mio_08::Interest,
  ) -> io::Result<()> {
    self.event_source.register(registry, token, interests)
  }

  fn reregister(
    &mut self,
    registry: &mio_08::Registry,
    token: mio_08::Token,
    interests: mio_08::Interest,
  ) -> io::Result<()> {
    self.event_source.reregister(registry, token, interests)
  }

  fn deregister(&mut self, registry: &mio_08::Registry) -> io::Result<()> {
    self.event_source.deregister(registry)
  }
}

impl<D, DA> StatusEvented<DataReaderStatus> for SimpleDataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn as_status_evented(&mut self) -> &dyn Evented {
    self.status_receiver.as_status_evented()
  }

  fn as_status_source(&mut self) -> &mut dyn mio_08::event::Source {
    self.status_receiver.as_status_source()
  }

  fn try_recv_status(&self) -> Option<DataReaderStatus> {
    self.status_receiver.try_recv_status()
  }
}

impl<D, DA> RTPSEntity for SimpleDataReader<D, DA>
where
  D: Keyed + DeserializeOwned,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn guid(&self) -> GUID {
    self.my_guid
  }
}

// ----------------------------------------------
// ----------------------------------------------

// Async interface to the SimpleDataReader

pub struct SimpleDataReaderStream<
  'a,
  D: Keyed + 'static,
  DA: DeserializerAdapter<D> + 'static = CDRDeserializerAdapter<D>,
> where
  <D as Keyed>::K: Key,
{
  simple_datareader: &'a SimpleDataReader<D, DA>,
}

// ----------------------------------------------
// ----------------------------------------------

// https://users.rust-lang.org/t/take-in-impl-future-cannot-borrow-data-in-a-dereference-of-pin/52042
impl<'a, D, DA> Unpin for SimpleDataReaderStream<'a, D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
}

impl<'a, D, DA> Stream for SimpleDataReaderStream<'a, D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  type Item = ReadResult<DeserializedCacheChange<D>>;

  // The full return type is now
  // Poll<Option<Result<DeserializedCacheChange<D>>>
  // Poll -> Ready or Pending
  // Option -> Some = stream produces a value, None = stream has ended (does not
  // occur) Result -> Ok = No DDS error, Err = DDS processing error
  // (inner Option -> Some = there is new value/key, None = no new data yet)

  fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
    debug!("poll_next");
    match self.simple_datareader.try_take_one() {
      Err(e) =>
      // DDS fails
      {
        Poll::Ready(Some(Err(e)))
      }

      // ok, got something
      Ok(Some(d)) => Poll::Ready(Some(Ok(d))),

      // No new data (yet)
      Ok(None) => {
        // Did not get any data.
        // --> Store waker.
        // 1. synchronously store waker to background thread (must rendezvous)
        // 2. try take_bare again, in case something arrived just now
        // 3. if nothing still, return pending.
        self.simple_datareader.set_waker(Some(cx.waker().clone()));
        match self.simple_datareader.try_take_one() {
          Err(e) => Poll::Ready(Some(Err(e))),
          Ok(Some(d)) => Poll::Ready(Some(Ok(d))),
          Ok(None) => Poll::Pending,
        }
      }
    } // match
  } // fn
} // impl

impl<'a, D, DA> FusedStream for SimpleDataReaderStream<'a, D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn is_terminated(&self) -> bool {
    false // Never terminate. This means it is always valid to call poll_next().
  }
}

// ----------------------------------------------
// ----------------------------------------------

pub struct SimpleDataReaderEventStream<
  'a,
  D: Keyed + 'static,
  DA: DeserializerAdapter<D> + 'static = CDRDeserializerAdapter<D>,
> where
  <D as Keyed>::K: Key,
{
  simple_datareader: &'a SimpleDataReader<D, DA>,
}

impl<'a, D, DA> Stream for SimpleDataReaderEventStream<'a, D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  type Item = ReadResult<DataReaderStatus>;

  fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
    Pin::new(&mut self.simple_datareader.status_receiver.as_async_stream()).poll_next(cx)
  } // fn
} // impl
