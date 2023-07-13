use std::{
  io,
  pin::Pin,
  task::{Context, Poll},
};

use mio_06::{self, Evented};
use futures::stream::{FusedStream, Stream};

use crate::{
  dds::{
    adapters::no_key::DeserializerAdapter,
    no_key::datasample::DataSample,
    qos::{HasQoSPolicy, QosPolicies},
    readcondition::ReadCondition,
    result::Result,
    statusevents::DataReaderStatus,
    with_key::{
      datareader as datareader_with_key,
      datasample::{DataSample as WithKeyDataSample, Sample},
      DataReader as WithKeyDataReader, DataReaderEventStream as WithKeyDataReaderEventStream,
      DataReaderStream as WithKeyDataReaderStream,
    },
  },
  serialization::CDRDeserializerAdapter,
  structure::entity::RTPSEntity,
  StatusEvented, GUID,
};
use super::wrappers::{DAWrapper, NoKeyWrapper};

/// Simplified type for CDR encoding
pub type DataReaderCdr<D> = DataReader<D, CDRDeserializerAdapter<D>>;

// ----------------------------------------------------

// DataReader for NO_KEY data. Does not require "D: Keyed"
/// DDS DataReader for no key topics.
/// # Examples
///
/// ```
/// use serde::{Serialize, Deserialize};
/// use rustdds::*;
/// use rustdds::no_key::DataReader;
/// use rustdds::serialization::CDRDeserializerAdapter;
///
/// let domain_participant = DomainParticipant::new(0).unwrap();
/// let qos = QosPolicyBuilder::new().build();
/// let subscriber = domain_participant.create_subscriber(&qos).unwrap();
///
/// #[derive(Serialize, Deserialize)]
/// struct SomeType {}
///
/// // NoKey is important
/// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
/// let data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None);
/// ```
pub struct DataReader<D, DA: DeserializerAdapter<D> = CDRDeserializerAdapter<D>> {
  keyed_datareader: datareader_with_key::DataReader<NoKeyWrapper<D>, DAWrapper<DA>>,
}

// TODO: rewrite DataSample so it can use current Keyed version (and send back
// datasamples instead of current data)
impl<D: 'static, DA> DataReader<D, DA>
where
  DA: DeserializerAdapter<D>,
{
  pub(crate) fn from_keyed(
    keyed: datareader_with_key::DataReader<NoKeyWrapper<D>, DAWrapper<DA>>,
  ) -> Self {
    Self {
      keyed_datareader: keyed,
    }
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
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  ///
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// let data = data_reader.read(10, ReadCondition::not_read());
  /// ```
  pub fn read(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
  ) -> Result<Vec<DataSample<&D>>> {
    let values: Vec<WithKeyDataSample<&NoKeyWrapper<D>>> =
      self.keyed_datareader.read(max_samples, read_condition)?;
    let mut result = Vec::with_capacity(values.len());
    for ks in values {
      if let Some(s) = DataSample::<D>::from_with_key_ref(ks) {
        result.push(s);
      }
    }
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
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// let data = data_reader.take(10, ReadCondition::not_read());
  /// ```
  pub fn take(
    &mut self,
    max_samples: usize,
    read_condition: ReadCondition,
  ) -> Result<Vec<DataSample<D>>> {
    let values: Vec<WithKeyDataSample<NoKeyWrapper<D>>> =
      self.keyed_datareader.take(max_samples, read_condition)?;
    let mut result = Vec::with_capacity(values.len());
    for ks in values {
      if let Some(s) = DataSample::<D>::from_with_key(ks) {
        result.push(s);
      }
    }
    Ok(result)
  }

  /// Reads next unread sample
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// while let Ok(Some(data)) = data_reader.read_next_sample() {
  ///   // Do something
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
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// while let Ok(Some(data)) = data_reader.take_next_sample() {
  ///   // Do something
  /// }
  /// ```
  pub fn take_next_sample(&mut self) -> Result<Option<DataSample<D>>> {
    let mut ds = self.take(1, ReadCondition::not_read())?;
    Ok(ds.pop())
  }

  // Iterator interface

  /// Produces an interator over the currently available NOT_READ samples.
  /// Yields only payload data, not SampleInfo metadata
  /// This is not called `iter()` because it takes a mutable reference to self.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// for data in data_reader.iterator() {
  ///   // Do something
  /// }
  /// ```
  pub fn iterator(&mut self) -> Result<impl Iterator<Item = &D>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // read call
    Ok(
      self
        .read(std::usize::MAX, ReadCondition::not_read())?
        .into_iter()
        .map(|ds| ds.value),
    )
  }

  /// Produces an interator over the samples filtered b ygiven condition.
  /// Yields only payload data, not SampleInfo metadata
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// for data in data_reader.conditional_iterator(ReadCondition::any()) {
  ///   // Do something
  /// }
  /// ```
  pub fn conditional_iterator(
    &mut self,
    read_condition: ReadCondition,
  ) -> Result<impl Iterator<Item = &D>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // read call
    Ok(
      self
        .read(std::usize::MAX, read_condition)?
        .into_iter()
        .map(|ds| ds.value),
    )
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
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// for data in data_reader.into_iterator() {
  ///   // Do something
  /// }
  /// ```
  pub fn into_iterator(&mut self) -> Result<impl Iterator<Item = D>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // read call
    Ok(
      self
        .take(std::usize::MAX, ReadCondition::not_read())?
        .into_iter()
        .map(|ds| ds.value),
    )
  }

  /// Produces an interator over the samples filtered b ygiven condition.
  /// Yields only payload data, not SampleInfo metadata
  /// <strong>Note!</strong> If the iterator is only partially consumed, all the
  /// samples it could have provided are still removed from the `Datareader`.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(&topic, None).unwrap();
  /// for data in data_reader.into_conditional_iterator(ReadCondition::any()) {
  ///   // Do something
  /// }
  /// ```
  pub fn into_conditional_iterator(
    &mut self,
    read_condition: ReadCondition,
  ) -> Result<impl Iterator<Item = D>> {
    // TODO: We could come up with a more efficent implementation than wrapping a
    // read call
    Ok(
      self
        .take(std::usize::MAX, read_condition)?
        .into_iter()
        .map(|ds| ds.value),
    )
  }
  /*
  /// Gets latest RequestedDeadlineMissed status
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::no_key::DataReader;
  /// # use rustdds::serialization::CDRDeserializerAdapter;
  /// #
  /// # let domain_participant = DomainParticipant::new(0).unwrap();
  /// # let qos = QosPolicyBuilder::new().build();
  /// # let subscriber = domain_participant.create_subscriber(&qos).unwrap();
  /// #
  /// # // NoKey is important
  /// # let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::NoKey).unwrap();
  /// #
  /// # #[derive(Serialize, Deserialize)]
  /// # struct SomeType {}
  /// #
  /// let mut data_reader = subscriber.create_datareader_no_key::<SomeType, CDRDeserializerAdapter<_>>(topic, None).unwrap();
  /// if let Ok(Some(status)) = data_reader.get_requested_deadline_missed_status() {
  ///   // Do something
  /// }
  /// ```


  pub fn get_requested_deadline_missed_status(
    &mut self,
  ) -> Result<Option<RequestedDeadlineMissedStatus>> {
    self.keyed_datareader.get_requested_deadline_missed_status()
  }
  */

  /// An async stream for reading the (bare) data samples
  pub fn async_sample_stream(self) -> DataReaderStream<D, DA> {
    DataReaderStream {
      keyed_stream: self.keyed_datareader.async_sample_stream(),
    }
  }
}

/// WARNING! UNTESTED
//  TODO: test
// This is  not part of DDS spec. We implement mio Eventd so that the
// application can asynchronously poll DataReader(s).
impl<D, DA> Evented for DataReader<D, DA>
where
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
    self.keyed_datareader.register(poll, token, interest, opts)
  }

  fn reregister(
    &self,
    poll: &mio_06::Poll,
    token: mio_06::Token,
    interest: mio_06::Ready,
    opts: mio_06::PollOpt,
  ) -> io::Result<()> {
    self
      .keyed_datareader
      .reregister(poll, token, interest, opts)
  }

  fn deregister(&self, poll: &mio_06::Poll) -> io::Result<()> {
    self.keyed_datareader.deregister(poll)
  }
}

/// WARNING! UNTESTED
//  TODO: test
impl<D, DA> mio_08::event::Source for DataReader<D, DA>
where
  DA: DeserializerAdapter<D>,
{
  fn register(
    &mut self,
    registry: &mio_08::Registry,
    token: mio_08::Token,
    interests: mio_08::Interest,
  ) -> io::Result<()> {
    // with_key::DataReader implements .register() for two traits, so need to
    // use disambiguation syntax to call .register() here.
    <WithKeyDataReader<NoKeyWrapper<D>, DAWrapper<DA>> as mio_08::event::Source>::register(
      &mut self.keyed_datareader,
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
    <WithKeyDataReader<NoKeyWrapper<D>, DAWrapper<DA>> as mio_08::event::Source>::reregister(
      &mut self.keyed_datareader,
      registry,
      token,
      interests,
    )
  }

  fn deregister(&mut self, registry: &mio_08::Registry) -> io::Result<()> {
    <WithKeyDataReader<NoKeyWrapper<D>, DAWrapper<DA>> as mio_08::event::Source>::deregister(
      &mut self.keyed_datareader,
      registry,
    )
  }
}

/// WARNING! UNTESTED
//  TODO: test
impl<D, DA> StatusEvented<DataReaderStatus> for DataReader<D, DA>
where
  DA: DeserializerAdapter<D>,
{
  fn as_status_evented(&mut self) -> &dyn Evented {
    self.keyed_datareader.as_status_evented()
  }

  fn as_status_source(&mut self) -> &mut dyn mio_08::event::Source {
    self.keyed_datareader.as_status_source()
  }

  fn try_recv_status(&self) -> Option<DataReaderStatus> {
    self.keyed_datareader.try_recv_status()
  }
}

impl<D, DA> HasQoSPolicy for DataReader<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  fn qos(&self) -> QosPolicies {
    self.keyed_datareader.qos()
  }
}

impl<D, DA> RTPSEntity for DataReader<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  fn guid(&self) -> GUID {
    self.keyed_datareader.guid()
  }
}

// ----------------------------------------------
// ----------------------------------------------

// Async interface for the DataReader

/// Wraps [`with_key::DataReaderStream`](crate::with_key::DataReaderStream) and
/// unwraps [`Sample`](crate::with_key::Sample) and `NoKeyWrapper` on
/// `poll_next`.
pub struct DataReaderStream<
  D: 'static,
  DA: DeserializerAdapter<D> + 'static = CDRDeserializerAdapter<D>,
> {
  keyed_stream: WithKeyDataReaderStream<NoKeyWrapper<D>, DAWrapper<DA>>,
}

impl<D, DA> DataReaderStream<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  pub fn async_event_stream(&self) -> DataReaderEventStream<D, DA> {
    DataReaderEventStream {
      keyed_stream: self.keyed_stream.async_event_stream(),
    }
  }
}

// https://users.rust-lang.org/t/take-in-impl-future-cannot-borrow-data-in-a-dereference-of-pin/52042
impl<D, DA> Unpin for DataReaderStream<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
}

impl<D, DA> Stream for DataReaderStream<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  type Item = Result<D>;

  fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
    match Pin::new(&mut Pin::into_inner(self).keyed_stream).poll_next(cx) {
      Poll::Ready(Some(Err(e))) => Poll::Ready(Some(Err(e))),
      Poll::Ready(Some(Ok(Sample::Value(d)))) => Poll::Ready(Some(Ok(d.d))), /* Unwraps Sample and NoKeyWrapper */
      Poll::Ready(Some(Ok(Sample::Dispose(_)))) => Poll::Pending,            /* Disposed data is */
      // ignored
      Poll::Ready(None) => Poll::Ready(None), // This should never happen
      Poll::Pending => Poll::Pending,
    }
  }
}

impl<D, DA> FusedStream for DataReaderStream<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  fn is_terminated(&self) -> bool {
    false // Never terminate. This means it is always valid to call poll_next().
  }
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

/// Wraps [`with_key::DataReaderEventStream`](crate::with_key::DataReaderEventStream).
pub struct DataReaderEventStream<
  D: 'static,
  DA: DeserializerAdapter<D> + 'static = CDRDeserializerAdapter<D>,
> {
  keyed_stream: WithKeyDataReaderEventStream<NoKeyWrapper<D>, DAWrapper<DA>>,
}

impl<D, DA> Stream for DataReaderEventStream<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  type Item = std::result::Result<DataReaderStatus, std::sync::mpsc::RecvError>;

  fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
    Pin::new(&mut Pin::into_inner(self).keyed_stream).poll_next(cx)
  }
}

impl<D, DA> FusedStream for DataReaderEventStream<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  fn is_terminated(&self) -> bool {
    false // Never terminate. This means it is always valid to call poll_next().
  }
}
