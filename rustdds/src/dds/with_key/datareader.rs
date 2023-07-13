use std::{
  io,
  pin::Pin,
  sync::{Arc, Mutex},
  task::{Context, Poll},
};

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use mio_06::{self, Evented};
use mio_08;
use futures::stream::{FusedStream, Stream};

use super::datasample_cache::DataSampleCache;
use crate::{
  dds::{
    adapters::with_key::*,
    key::*,
    qos::*,
    readcondition::*,
    result::*,
    statusevents::*,
    with_key::{datasample::*, simpledatareader::*},
  },
  discovery::sedp_messages::PublicationBuiltinTopicData,
  serialization::CDRDeserializerAdapter,
  structure::{duration::Duration, entity::RTPSEntity, guid::GUID, time::Timestamp},
};

/// Simplified type for CDR encoding
pub type DataReaderCdr<D> = DataReader<D, CDRDeserializerAdapter<D>>;

/// Parameter for reading [Readers](../struct.With_Key_DataReader.html) data
/// with key or with next from current key.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectByKey {
  This,
  Next,
}

/// DDS DataReader for with_key topics.
///
/// # Examples
///
/// ```
/// use serde::{Serialize, Deserialize};
/// use rustdds::*;
/// use rustdds::with_key::DataReader;
/// use rustdds::serialization::CDRDeserializerAdapter;
///
/// let domain_participant = DomainParticipant::new(0).unwrap();
/// let qos = QosPolicyBuilder::new().build();
/// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
///
/// #[derive(Serialize, Deserialize)]
/// struct SomeType { a: i32 }
/// impl Keyed for SomeType {
///   type K = i32;
///
///   fn key(&self) -> Self::K {
///     self.a
///   }
/// }
///
/// // WithKey is important
/// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
/// let data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None);
/// ```
///
/// *Note:* Many DataReader methods require mutable access to `self`, because
/// they need to mutate the datasample ceche, which is an essential content of
/// this struct.
pub struct DataReader<D: Keyed, DA: DeserializerAdapter<D> = CDRDeserializerAdapter<D>>
where
  <D as Keyed>::K: Key,
{
  simple_data_reader: SimpleDataReader<D, DA>,
  datasample_cache: DataSampleCache<D>, // DataReader-local cache of deserialized samples
}

impl<D: 'static, DA> DataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  pub(crate) fn from_simple_data_reader(simple_data_reader: SimpleDataReader<D, DA>) -> Self {
    let dsc = DataSampleCache::new(simple_data_reader.topic().qos());

    Self {
      simple_data_reader,
      datasample_cache: dsc,
    }
  }

  // Gets all unseen cache_changes from the TopicCache. Deserializes
  // the serialized payload and stores the DataSamples (the actual data and the
  // samplestate) to local container, datasample_cache.
  fn fill_and_lock_local_datasample_cache(&mut self) -> Result<()> {
    while let Some(dcc) = self.simple_data_reader.try_take_one()? {
      self
        .datasample_cache
        .fill_from_deserialized_cache_change(dcc);
    }
    Ok(())
  }

  fn drain_read_notifications(&self) {
    self.simple_data_reader.drain_read_notifications();
  }

  fn select_keys_for_access(&self, read_condition: ReadCondition) -> Vec<(Timestamp, D::K)> {
    self.datasample_cache.select_keys_for_access(read_condition)
  }

  fn take_by_keys(&mut self, keys: &[(Timestamp, D::K)]) -> Vec<DataSample<D>> {
    self.datasample_cache.take_by_keys(keys)
  }

  fn take_bare_by_keys(&mut self, keys: &[(Timestamp, D::K)]) -> Vec<Sample<D, D::K>> {
    self.datasample_cache.take_bare_by_keys(keys)
  }

  fn select_instance_keys_for_access(
    &self,
    instance: &D::K,
    rc: ReadCondition,
  ) -> Vec<(Timestamp, D::K)> {
    self
      .datasample_cache
      .select_instance_keys_for_access(instance, rc)
  }

  /// Reads amount of samples found with `max_samples` and `read_condition`
  /// parameters.
  ///
  /// # Arguments
  ///
  /// * `max_samples` - Limits maximum amount of samples read
  /// * `read_condition` - Limits results by condition
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// if let Ok(datas) = data_reader.read(10, ReadCondition::not_read()) {
  ///   for data in datas.iter() {
  ///     // do something
  ///   }
  /// }
  /// ```

  pub fn read(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
  ) -> Result<Vec<DataSample<&D>>> {
    // Clear notification buffer. This must be done first to avoid race conditions.
    self.drain_read_notifications();
    self.fill_and_lock_local_datasample_cache()?;

    let mut selected = self.select_keys_for_access(read_condition);
    selected.truncate(max_samples);

    let result = self.datasample_cache.read_by_keys(&selected);

    Ok(result)
  }

  /// Takes amount of sample found with `max_samples` and `read_condition`
  /// parameters.
  ///
  /// # Arguments
  ///
  /// * `max_samples` - Limits maximum amount of samples read
  /// * `read_condition` - Limits results by condition
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// if let Ok(datas) = data_reader.take(10, ReadCondition::not_read()) {
  ///   for data in datas.iter() {
  ///     // do something
  ///   }
  /// }
  /// ```
  pub fn take(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
  ) -> Result<Vec<DataSample<D>>> {
    // Clear notification buffer. This must be done first to avoid race conditions.
    self.drain_read_notifications();

    self.fill_and_lock_local_datasample_cache()?;
    let mut selected = self.select_keys_for_access(read_condition);
    trace!("take selected count = {}", selected.len());
    selected.truncate(max_samples);

    let result = self.take_by_keys(&selected);
    trace!("take taken count = {}", result.len());

    Ok(result)
  }

  /// Reads next unread sample
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// while let Ok(Some(data)) = data_reader.read_next_sample() {
  ///   // do something
  /// }
  /// ```
  pub fn read_next_sample(&mut self) -> Result<Option<DataSample<&D>>> {
    let mut ds = self.read(1, ReadCondition::not_read())?;
    Ok(ds.pop())
  }

  /// Takes next unread sample
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// while let Ok(Some(data)) = data_reader.take_next_sample() {
  ///   // do something
  /// }
  /// ```
  pub fn take_next_sample(&mut self) -> Result<Option<DataSample<D>>> {
    let mut ds = self.take(1, ReadCondition::not_read())?;
    Ok(ds.pop())
  }

  // Iterator interface

  fn read_bare(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
  ) -> Result<Vec<Sample<&D, D::K>>> {
    self.drain_read_notifications();
    self.fill_and_lock_local_datasample_cache()?;

    let mut selected = self.select_keys_for_access(read_condition);
    selected.truncate(max_samples);

    let result = self.datasample_cache.read_bare_by_keys(&selected);

    Ok(result)
  }

  fn take_bare(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
  ) -> Result<Vec<Sample<D, D::K>>> {
    // Clear notification buffer. This must be done first to avoid race conditions.
    self.drain_read_notifications();
    self.fill_and_lock_local_datasample_cache()?;

    let mut selected = self.select_keys_for_access(read_condition);
    trace!("take bare selected count = {}", selected.len());
    selected.truncate(max_samples);

    let result = self.take_bare_by_keys(&selected);
    trace!("take bare taken count = {}", result.len());

    Ok(result)
  }

  /// Produces an interator over the currently available NOT_READ samples.
  /// Yields only payload data, not SampleInfo metadata
  /// This is not called `iter()` because it takes a mutable reference to self.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// for data in data_reader.iterator() {
  ///   // do something
  /// }
  /// ```
  pub fn iterator(&mut self) -> Result<impl Iterator<Item = Sample<&D, D::K>>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // read call
    Ok(
      self
        .read_bare(std::usize::MAX, ReadCondition::not_read())?
        .into_iter(),
    )
  }

  /// Produces an interator over the samples filtered by a given condition.
  /// Yields only payload data, not SampleInfo metadata
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// for data in data_reader.conditional_iterator(ReadCondition::any()) {
  ///   // do something
  /// }
  /// ```
  pub fn conditional_iterator(
    &mut self,
    read_condition: ReadCondition,
  ) -> Result<impl Iterator<Item = Sample<&D, D::K>>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // read call
    Ok(self.read_bare(std::usize::MAX, read_condition)?.into_iter())
  }

  /// Produces an interator over the currently available NOT_READ samples.
  /// Yields only payload data, not SampleInfo metadata
  /// Removes samples from `DataReader`.
  /// <strong>Note!</strong> If the iterator is only partially consumed, all the
  /// samples it could have provided are still removed from the `Datareader`.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// for data in data_reader.into_iterator() {
  ///   // do something
  /// }
  /// ```

  pub fn into_iterator(&mut self) -> Result<impl Iterator<Item = Sample<D, D::K>>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // take call
    Ok(
      self
        .take_bare(std::usize::MAX, ReadCondition::not_read())?
        .into_iter(),
    )
  }

  /// Produces an interator over the samples filtered by the given condition.
  /// Yields only payload data, not SampleInfo metadata
  /// <strong>Note!</strong> If the iterator is only partially consumed, all the
  /// samples it could have provided are still removed from the `Datareader`.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// for data in data_reader.into_conditional_iterator(ReadCondition::not_read()) {
  ///   // do something
  /// }
  /// ```
  pub fn into_conditional_iterator(
    &mut self,
    read_condition: ReadCondition,
  ) -> Result<impl Iterator<Item = Sample<D, D::K>>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // take call
    Ok(self.take_bare(std::usize::MAX, read_condition)?.into_iter())
  }

  // ----------------------------------------------------------------------------
  // ----------------------------------------------------------------------------

  fn infer_key(
    &self,
    instance_key: Option<<D as Keyed>::K>,
    this_or_next: SelectByKey,
  ) -> Option<<D as Keyed>::K> {
    match instance_key {
      Some(k) => match this_or_next {
        SelectByKey::This => Some(k),
        SelectByKey::Next => self.datasample_cache.next_key(&k),
      },
      None => self.datasample_cache.instance_map.keys().next().cloned(),
    }
  }

  /// Works similarly to read(), but will return only samples from a specific
  /// instance. The instance is specified by an optional key. In case the key
  /// is not specified, the smallest (in key order) instance is selected.
  /// If a key is specified, then the parameter this_or_next specifies whether
  /// to access the instance with specified key or the following one, in key
  /// order.
  ///
  /// This should cover DDS DataReader methods read_instance,
  /// read_next_instance, read_next_instance_w_condition.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// if let Ok(datas) = data_reader.read_instance(10, ReadCondition::any(), Some(3), SelectByKey::This) {
  ///   for data in datas.iter() {
  ///     // do something
  ///   }
  /// }
  /// ```
  pub fn read_instance(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
    // Select only samples from instance specified by key. In case of None, select the
    // "smallest" instance as specified by the key type Ord trait.
    instance_key: Option<<D as Keyed>::K>,
    // This = Select instance specified by key.
    // Next = select next instance in the order specified by Ord on keys.
    this_or_next: SelectByKey,
  ) -> Result<Vec<DataSample<&D>>> {
    self.drain_read_notifications();
    self.fill_and_lock_local_datasample_cache()?;

    let key = match Self::infer_key(self, instance_key, this_or_next) {
      Some(k) => k,
      None => return Ok(Vec::new()),
    };

    let mut selected = self
      .datasample_cache
      .select_instance_keys_for_access(&key, read_condition);
    selected.truncate(max_samples);

    let result = self.datasample_cache.read_by_keys(&selected);

    Ok(result)
  }

  /// Similar to read_instance, but will return owned datasamples
  /// This should cover DDS DataReader methods take_instance,
  /// take_next_instance, take_next_instance_w_condition.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  ///
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType { a: i32 }
  /// # impl Keyed for SomeType {
  /// #   type K = i32;
  /// #
  /// #   fn key(&self) -> Self::K {
  /// #     self.a
  /// #   }
  /// # }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let mut data_reader = subscriber.create_datareader::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// // Wait for data to arrive...
  ///
  /// if let Ok(datas) = data_reader.take_instance(10, ReadCondition::any(), Some(3), SelectByKey::Next) {
  ///   for data in datas.iter() {
  ///     // do something
  ///   }
  /// }
  /// ```
  pub fn take_instance(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
    // Select only samples from instance specified by key. In case of None, select the
    // "smallest" instance as specified by the key type Ord trait.
    instance_key: Option<<D as Keyed>::K>,
    // This = Select instance specified by key.
    // Next = select next instance in the order specified by Ord on keys.
    this_or_next: SelectByKey,
  ) -> Result<Vec<DataSample<D>>> {
    // Clear notification buffer. This must be done first to avoid race conditions.
    self.drain_read_notifications();

    self.fill_and_lock_local_datasample_cache()?;

    let key = match self.infer_key(instance_key, this_or_next) {
      Some(k) => k,
      None => return Ok(Vec::new()),
    };

    let mut selected = self.select_instance_keys_for_access(&key, read_condition);
    selected.truncate(max_samples);

    let result = self.take_by_keys(&selected);

    Ok(result)
  }

  /// Return values:
  /// true - got all historical data
  /// false - timeout before all historical data was received
  pub fn wait_for_historical_data(&mut self, _max_wait: Duration) -> bool {
    todo!()
  }

  // Spec calls for two separate functions:
  // get_matched_publications returns a list of handles
  // get_matched_publication_data returns PublicationBuiltinTopicData for a handle
  // But we do not believe in handle-oriented programming, so just return
  // the actual data right away. Since the handles are quite opaque, about the
  // only thing that could be done with the handles would be counting how many
  // we got.

  pub fn get_matched_publications(&self) -> impl Iterator<Item = PublicationBuiltinTopicData> {
    //TODO: Obviously not implemented
    vec![].into_iter()
  }

  /// An async stream for reading the (bare) data samples.
  /// The resulting Stream can be used to get another stream of status events.
  pub fn async_sample_stream(self) -> DataReaderStream<D, DA> {
    DataReaderStream {
      datareader: Arc::new(Mutex::new(self)),
    }
  }
} // impl

// -------------------

impl<D, DA> Evented for DataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  // We just delegate all the operations to notification_receiver, since it alrady
  // implements Evented
  fn register(
    &self,
    poll: &mio_06::Poll,
    token: mio_06::Token,
    interest: mio_06::Ready,
    opts: mio_06::PollOpt,
  ) -> io::Result<()> {
    self
      .simple_data_reader
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
      .simple_data_reader
      .reregister(poll, token, interest, opts)
  }

  fn deregister(&self, poll: &mio_06::Poll) -> io::Result<()> {
    self.simple_data_reader.deregister(poll)
  }
}

impl<D, DA> mio_08::event::Source for DataReader<D, DA>
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
    // SimpleDataReader implements .register() for two traits, so need to
    // use disambiguation syntax to call .register() here.
    <SimpleDataReader<D, DA> as mio_08::event::Source>::register(
      &mut self.simple_data_reader,
      registry,
      token,
      interests,
    )
  }

  fn reregister(
    &mut self,
    registry: &mio_08::Registry,
    token: mio_08::Token,
    interests: mio_08::Interest,
  ) -> io::Result<()> {
    <SimpleDataReader<D, DA> as mio_08::event::Source>::reregister(
      &mut self.simple_data_reader,
      registry,
      token,
      interests,
    )
  }

  fn deregister(&mut self, registry: &mio_08::Registry) -> io::Result<()> {
    <SimpleDataReader<D, DA> as mio_08::event::Source>::deregister(
      &mut self.simple_data_reader,
      registry,
    )
  }
}

impl<D, DA> StatusEvented<DataReaderStatus> for DataReader<D, DA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn as_status_evented(&mut self) -> &dyn Evented {
    self.simple_data_reader.as_status_evented()
  }

  fn as_status_source(&mut self) -> &mut dyn mio_08::event::Source {
    self.simple_data_reader.as_status_source()
  }

  fn try_recv_status(&self) -> Option<DataReaderStatus> {
    self.simple_data_reader.try_recv_status()
  }
}

impl<D, DA> HasQoSPolicy for DataReader<D, DA>
where
  D: Keyed + 'static,
  DA: DeserializerAdapter<D>,
  <D as Keyed>::K: Key,
{
  fn qos(&self) -> QosPolicies {
    self.simple_data_reader.qos().clone()
  }
}

impl<D, DA> RTPSEntity for DataReader<D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn guid(&self) -> GUID {
    self.simple_data_reader.guid()
  }
}

// ----------------------------------------------
// ----------------------------------------------

// Async interface to the DataReader

pub struct DataReaderStream<
  D: Keyed + 'static,
  DA: DeserializerAdapter<D> + 'static = CDRDeserializerAdapter<D>,
> where
  <D as Keyed>::K: Key,
{
  datareader: Arc<Mutex<DataReader<D, DA>>>,
}

impl<D, DA> DataReaderStream<D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  /// Get a stream of status events
  pub fn async_event_stream(&self) -> DataReaderEventStream<D, DA> {
    DataReaderEventStream {
      datareader: Arc::clone(&self.datareader),
    }
  }
}

// https://users.rust-lang.org/t/take-in-impl-future-cannot-borrow-data-in-a-dereference-of-pin/52042
impl<D, DA> Unpin for DataReaderStream<D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
}

impl<D, DA> Stream for DataReaderStream<D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  type Item = Result<Sample<D, D::K>>;

  fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
    debug!("poll_next");
    let mut datareader = self.datareader.lock().unwrap();
    match datareader.take_bare(1, ReadCondition::not_read()) {
      Err(e) =>
      // DDS fails
      {
        Poll::Ready(Some(Err(e)))
      }

      Ok(mut v) => {
        match v.pop() {
          Some(d) => Poll::Ready(Some(Ok(d))),
          None => {
            // Did not get any data.
            // --> Store waker.
            // 1. synchronously store waker to background thread (must rendezvous)
            // 2. try take_bare again, in case something arrived just now
            // 3. if nothing still, return pending.
            datareader
              .simple_data_reader
              .set_waker(Some(cx.waker().clone()));
            match datareader.take_bare(1, ReadCondition::not_read()) {
              Err(e) => Poll::Ready(Some(Err(e))),
              Ok(mut v) => match v.pop() {
                None => Poll::Pending,
                Some(d) => Poll::Ready(Some(Ok(d))),
              },
            }
          }
        }
      }
    }
  }
}

impl<D, DA> FusedStream for DataReaderStream<D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn is_terminated(&self) -> bool {
    false // Never terminate. This means it is always valid to call poll_next().
  }
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

pub struct DataReaderEventStream<
  D: Keyed + 'static,
  DA: DeserializerAdapter<D> + 'static = CDRDeserializerAdapter<D>,
> where
  <D as Keyed>::K: Key,
{
  datareader: Arc<Mutex<DataReader<D, DA>>>,
}

impl<D, DA> Stream for DataReaderEventStream<D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  type Item = std::result::Result<DataReaderStatus, std::sync::mpsc::RecvError>;

  fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
    let datareader = self.datareader.lock().unwrap();
    Pin::new(
      &mut datareader
        .simple_data_reader
        .as_simple_data_reader_event_stream(),
    )
    .poll_next(cx)
  }
}

impl<D, DA> FusedStream for DataReaderEventStream<D, DA>
where
  D: Keyed + 'static,
  <D as Keyed>::K: Key,
  DA: DeserializerAdapter<D>,
{
  fn is_terminated(&self) -> bool {
    false // Never terminate. This means it is always valid to call poll_next().
  }
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
  use std::rc::Rc;

  use bytes::Bytes;
  use mio_extras::channel as mio_channel;
  use log::info;
  use byteorder::LittleEndian;

  use super::*;
  use crate::{
    dds::{
      participant::DomainParticipant,
      topic::{TopicDescription, TopicKind},
    },
    messages::submessages::{
      data::Data, elements::serialized_payload::SerializedPayload, submessage_flag::*,
    },
    mio_source,
    network::udp_sender::UDPSender,
    rtps::{
      message_receiver::*,
      reader::{Reader, ReaderIngredients},
    },
    serialization::{cdr_deserializer::CDRDeserializerAdapter, cdr_serializer::to_bytes},
    structure::{
      guid::{EntityId, EntityKind, GuidPrefix},
      sequence_number::SequenceNumber,
    },
    test::random_data::*,
    Keyed, RepresentationIdentifier,
  };

  #[test]
  fn read_and_take() {
    // Test the read and take methods of the DataReader

    let dp = DomainParticipant::new(0).expect("Particpant creation failed!");

    let mut qos = QosPolicies::qos_none();
    qos.history = Some(policy::History::KeepAll); // Just for testing

    let sub = dp.create_subscriber(&qos).unwrap();
    let topic = dp
      .create_topic(
        "dr read".to_string(),
        "read fn test?".to_string(),
        &qos,
        TopicKind::WithKey,
      )
      .unwrap();

    let topic_cache =
      dp.dds_cache()
        .write()
        .unwrap()
        .add_new_topic(topic.name(), topic.get_type(), &topic.qos());

    // Create a Reader
    let (notification_sender, _notification_receiver) = mio_channel::sync_channel::<()>(100);
    let (_notification_event_source, notification_event_sender) =
      mio_source::make_poll_channel().unwrap();
    let data_reader_waker = Arc::new(Mutex::new(None));

    let (status_sender, _status_receiver) = sync_status_channel::<DataReaderStatus>(4).unwrap();

    let (_reader_command_sender, reader_command_receiver) =
      mio_channel::sync_channel::<ReaderCommand>(10);

    let default_id = EntityId::default();
    let reader_guid = GUID::new_with_prefix_and_id(dp.guid_prefix(), default_id);

    let reader_ing = ReaderIngredients {
      guid: reader_guid,
      notification_sender,
      status_sender,
      topic_name: topic.name(),
      topic_cache_handle: topic_cache,
      qos_policy: QosPolicies::qos_none(),
      data_reader_command_receiver: reader_command_receiver,
      data_reader_waker,
      poll_event_sender: notification_event_sender,
    };

    let mut reader = Reader::new(
      reader_ing,
      Rc::new(UDPSender::new_with_random_port().unwrap()),
      mio_extras::timer::Builder::default().build(),
    );

    // Create the corresponding matching DataReader
    let mut datareader = sub
      .create_datareader::<RandomData, CDRDeserializerAdapter<RandomData>>(&topic, None)
      .unwrap();

    let writer_guid = GUID {
      prefix: GuidPrefix::new(&[1; 12]),
      entity_id: EntityId::create_custom_entity_id(
        [1; 3],
        EntityKind::WRITER_WITH_KEY_USER_DEFINED,
      ),
    };
    let mr_state = MessageReceiverState {
      source_guid_prefix: writer_guid.prefix,
      ..Default::default()
    };
    reader.matched_writer_add(
      writer_guid,
      EntityId::UNKNOWN,
      mr_state.unicast_reply_locator_list.clone(),
      mr_state.multicast_reply_locator_list.clone(),
      &QosPolicies::qos_none(),
    );

    // Reader and datareader ready, feed reader some data
    let test_data = RandomData {
      a: 10,
      b: ":DDD".to_string(),
    };

    let test_data2 = RandomData {
      a: 11,
      b: ":)))".to_string(),
    };
    let data_msg = Data {
      reader_id: reader.entity_id(),
      writer_id: writer_guid.entity_id,
      writer_sn: SequenceNumber::from(1),
      serialized_payload: Some(SerializedPayload {
        representation_identifier: RepresentationIdentifier::CDR_LE,
        representation_options: [0, 0],
        value: Bytes::from(to_bytes::<RandomData, LittleEndian>(&test_data).unwrap()),
      }),
      ..Default::default()
    };

    let data_msg2 = Data {
      reader_id: reader.entity_id(),
      writer_id: writer_guid.entity_id,
      writer_sn: SequenceNumber::from(2),
      serialized_payload: Some(SerializedPayload {
        representation_identifier: RepresentationIdentifier::CDR_LE,
        representation_options: [0, 0],
        value: Bytes::from(to_bytes::<RandomData, LittleEndian>(&test_data2).unwrap()),
      }),
      ..Default::default()
    };

    let data_flags = DATA_Flags::Endianness | DATA_Flags::Data;

    reader.handle_data_msg(data_msg, data_flags, &mr_state);
    reader.handle_data_msg(data_msg2, data_flags, &mr_state);

    // Test that reading does not consume data samples, i.e. they can be read
    // multiple times
    {
      let result_vec = datareader.read(100, ReadCondition::any()).unwrap();
      assert_eq!(result_vec.len(), 2);
      let d = result_vec[0].value().clone().unwrap();
      assert_eq!(&test_data, d);
    }
    {
      let result_vec2 = datareader.read(100, ReadCondition::any()).unwrap();
      assert_eq!(result_vec2.len(), 2);
      let d2 = result_vec2[1].value().clone().unwrap();
      assert_eq!(&test_data2, d2);
    }
    {
      let result_vec3 = datareader.read(100, ReadCondition::any()).unwrap();
      let d3 = result_vec3[0].value().clone().unwrap();
      assert_eq!(&test_data, d3);
    }

    // Test that taking consumes the data samples
    let mut result_vec = datareader.take(100, ReadCondition::any()).unwrap();
    let datasample2 = result_vec.pop().unwrap();
    let datasample1 = result_vec.pop().unwrap();
    let data2 = datasample2.into_value().unwrap();
    let data1 = datasample1.into_value().unwrap();
    assert_eq!(test_data2, data2);
    assert_eq!(test_data, data1);

    let result_vec2 = datareader.take(100, ReadCondition::any());
    assert!(result_vec2.is_ok());
    assert_eq!(result_vec2.unwrap().len(), 0);
  }

  #[test]
  fn read_and_take_with_instance() {
    // Test the methods read_instance and take_instance of the DataReader

    let dp = DomainParticipant::new(0).expect("Particpant creation failed!");

    let mut qos = QosPolicies::qos_none();
    qos.history = Some(policy::History::KeepAll); // Just for testing

    let sub = dp.create_subscriber(&qos).unwrap();
    let topic = dp
      .create_topic(
        "dr read".to_string(),
        "read fn test?".to_string(),
        &qos,
        TopicKind::WithKey,
      )
      .unwrap();

    let topic_cache =
      dp.dds_cache()
        .write()
        .unwrap()
        .add_new_topic(topic.name(), topic.get_type(), &topic.qos());

    // Create a Reader
    let (notification_sender, _notification_receiver) = mio_channel::sync_channel::<()>(100);
    let (_notification_event_source, notification_event_sender) =
      mio_source::make_poll_channel().unwrap();
    let data_reader_waker = Arc::new(Mutex::new(None));

    let (status_sender, _status_receiver) = sync_status_channel::<DataReaderStatus>(4).unwrap();

    let (_reader_command_sender, reader_command_receiver) =
      mio_channel::sync_channel::<ReaderCommand>(10);

    let default_id = EntityId::default();
    let reader_guid = GUID::new_with_prefix_and_id(dp.guid_prefix(), default_id);

    let reader_ing = ReaderIngredients {
      guid: reader_guid,
      notification_sender,
      status_sender,
      topic_name: topic.name(),
      topic_cache_handle: topic_cache,
      qos_policy: QosPolicies::qos_none(),
      data_reader_command_receiver: reader_command_receiver,
      data_reader_waker,
      poll_event_sender: notification_event_sender,
    };

    let mut reader = Reader::new(
      reader_ing,
      Rc::new(UDPSender::new_with_random_port().unwrap()),
      mio_extras::timer::Builder::default().build(),
    );

    // Create the corresponding matching DataReader
    let mut datareader = sub
      .create_datareader::<RandomData, CDRDeserializerAdapter<RandomData>>(&topic, None)
      .unwrap();

    let writer_guid = GUID {
      prefix: GuidPrefix::new(&[1; 12]),
      entity_id: EntityId::create_custom_entity_id(
        [1; 3],
        EntityKind::WRITER_WITH_KEY_USER_DEFINED,
      ),
    };
    let mr_state = MessageReceiverState {
      source_guid_prefix: writer_guid.prefix,
      ..Default::default()
    };
    reader.matched_writer_add(
      writer_guid,
      EntityId::UNKNOWN,
      mr_state.unicast_reply_locator_list.clone(),
      mr_state.multicast_reply_locator_list.clone(),
      &QosPolicies::qos_none(),
    );

    // Create 4 data items, 3 of which have the same key
    let data_key1 = RandomData {
      a: 1,
      b: ":D".to_string(),
    };
    let data_key2_1 = RandomData {
      a: 2,
      b: ":(".to_string(),
    };
    let data_key2_2 = RandomData {
      a: 2,
      b: "??".to_string(),
    };
    let data_key2_3 = RandomData {
      a: 2,
      b: "xD".to_string(),
    };

    let key1 = data_key1.key();
    let key2 = data_key2_1.key();

    assert!(data_key2_1.key() == data_key2_2.key());
    assert!(data_key2_3.key() == key2);

    // Create data messages from the data items
    // Note that sequence numbering needs to continue as expected
    let data_msg = Data {
      reader_id: reader.entity_id(),
      writer_id: writer_guid.entity_id,
      writer_sn: SequenceNumber::from(1),
      serialized_payload: Some(SerializedPayload {
        representation_identifier: RepresentationIdentifier::CDR_LE,
        representation_options: [0, 0],
        value: Bytes::from(to_bytes::<RandomData, LittleEndian>(&data_key1).unwrap()),
      }),
      ..Data::default()
    };
    let data_msg2 = Data {
      reader_id: reader.entity_id(),
      writer_id: writer_guid.entity_id,
      writer_sn: SequenceNumber::from(2),
      serialized_payload: Some(SerializedPayload {
        representation_identifier: RepresentationIdentifier::CDR_LE,
        representation_options: [0, 0],
        value: Bytes::from(to_bytes::<RandomData, LittleEndian>(&data_key2_1).unwrap()),
      }),
      ..Data::default()
    };
    let data_msg3 = Data {
      reader_id: reader.entity_id(),
      writer_id: writer_guid.entity_id,
      writer_sn: SequenceNumber::from(3),
      serialized_payload: Some(SerializedPayload {
        representation_identifier: RepresentationIdentifier::CDR_LE,
        representation_options: [0, 0],
        value: Bytes::from(to_bytes::<RandomData, LittleEndian>(&data_key2_2).unwrap()),
      }),
      ..Data::default()
    };
    let data_msg4 = Data {
      reader_id: reader.entity_id(),
      writer_id: writer_guid.entity_id,
      writer_sn: SequenceNumber::from(4),
      serialized_payload: Some(SerializedPayload {
        representation_identifier: RepresentationIdentifier::CDR_LE,
        representation_options: [0, 0],
        value: Bytes::from(to_bytes::<RandomData, LittleEndian>(&data_key2_3).unwrap()),
      }),
      ..Data::default()
    };

    let data_flags = DATA_Flags::Endianness | DATA_Flags::Data;

    // Feed the data messages to the reader
    reader.handle_data_msg(data_msg, data_flags, &mr_state);
    reader.handle_data_msg(data_msg2, data_flags, &mr_state);
    reader.handle_data_msg(data_msg3, data_flags, &mr_state);
    reader.handle_data_msg(data_msg4, data_flags, &mr_state);

    // Check that calling read_instance with different keys and SelectByKey options
    // works as expected

    info!("calling read with key 1 and this");
    let results =
      datareader.read_instance(100, ReadCondition::any(), Some(key1), SelectByKey::This);
    assert_eq!(&data_key1, results.unwrap()[0].value().clone().unwrap());

    info!("calling read with None and this");
    // Takes the smallest key, 1 in this case.
    let results = datareader.read_instance(100, ReadCondition::any(), None, SelectByKey::This);
    assert_eq!(&data_key1, results.unwrap()[0].value().clone().unwrap());

    info!("calling read with key 1 and next");
    let results =
      datareader.read_instance(100, ReadCondition::any(), Some(key1), SelectByKey::Next);
    assert_eq!(results.as_ref().unwrap().len(), 3);
    assert_eq!(&data_key2_1, results.unwrap()[0].value().clone().unwrap());

    // Check that calling take_instance returns all 3 samples with the same key
    info!("calling take with key 2 and this");
    let results =
      datareader.take_instance(100, ReadCondition::any(), Some(key2), SelectByKey::This);
    assert_eq!(results.as_ref().unwrap().len(), 3);
    let mut vec = results.unwrap();
    let d3 = vec.pop().unwrap();
    let d3 = d3.into_value().unwrap();
    let d2 = vec.pop().unwrap();
    let d2 = d2.into_value().unwrap();
    let d1 = vec.pop().unwrap();
    let d1 = d1.into_value().unwrap();
    assert_eq!(data_key2_3, d3);
    assert_eq!(data_key2_2, d2);
    assert_eq!(data_key2_1, d1);

    // Check that calling take_instance again returns nothing because all samples
    // have been consumed
    info!("calling take with key 2 and this");
    let results =
      datareader.take_instance(100, ReadCondition::any(), Some(key2), SelectByKey::This);
    assert!(results.is_ok());
    assert!(results.unwrap().is_empty());
  }
}
