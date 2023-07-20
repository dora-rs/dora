use std::{io, marker::PhantomData};

use mio::{Evented, Poll, PollOpt, Ready, Token};
use futures::{
  pin_mut,
  stream::{FusedStream, Stream, StreamExt},
};
use rustdds::{rpc::SampleIdentity, *, serialization::SeedCDRDeserializerAdapter};
use serde::{
  de::{DeserializeOwned, DeserializeSeed},
  Serialize,
};

/// A ROS2 Publisher
///
/// Corresponds to a simplified [`DataWriter`](rustdds::no_key::DataWriter)in
/// DDS
pub struct Publisher<M: Serialize> {
  datawriter: no_key::DataWriterCdr<M>,
}

impl<M: Serialize> Publisher<M> {
  // These must be created from Node
  pub(crate) fn new(datawriter: no_key::DataWriterCdr<M>) -> Publisher<M> {
    Publisher { datawriter }
  }

  pub fn publish(&self, message: M) -> dds::WriteResult<(), M> {
    self.datawriter.write(message, Some(Timestamp::now()))
  }

  // pub(crate) fn publish_with_options(
  //   &self,
  //   message: M,
  //   wo: WriteOptions,
  // ) -> dds::Result<rustdds::rpc::SampleIdentity> {
  //   self.datawriter.write_with_options(message, wo)
  // }

  pub fn assert_liveliness(&self) -> dds::WriteResult<(), ()> {
    self.datawriter.assert_liveliness()
  }

  pub fn guid(&self) -> rustdds::GUID {
    self.datawriter.guid()
  }

  pub async fn async_publish(&self, message: M) -> dds::WriteResult<(), M> {
    self
      .datawriter
      .async_write(message, Some(Timestamp::now()))
      .await
  }

  #[allow(dead_code)] // This is for async Service implementation. Remove this when it is implemented.
  pub(crate) async fn async_publish_with_options(
    &self,
    message: M,
    wo: WriteOptions,
  ) -> dds::WriteResult<rustdds::rpc::SampleIdentity, M> {
    self.datawriter.async_write_with_options(message, wo).await
  }
}
// ----------------------------------------------------
// ----------------------------------------------------
// ----------------------------------------------------
// ----------------------------------------------------
// ----------------------------------------------------

/// A ROS2 Subscription
///
/// Corresponds to a (simplified) [`DataReader`](rustdds::no_key::DataReader) in
/// DDS
pub struct Subscription<M> {
  datareader: no_key::SimpleDataReader<M>,
  phantom: PhantomData<M>,
}

impl<M: 'static + DeserializeOwned> Subscription<M> {
  // These must be created from Node
  pub(crate) fn new(datareader: no_key::SimpleDataReader) -> Subscription<M> {
    Subscription {
      datareader,
      phantom: PhantomData,
    }
  }

  pub fn take(&self) -> dds::ReadResult<Option<(M, MessageInfo)>>
  where
    M: DeserializeOwned,
  {
    self.datareader.drain_read_notifications();
    let ds: Option<no_key::DeserializedCacheChange<M>> = self.datareader.try_take_one()?;
    Ok(ds.map(dcc_to_value_and_messageinfo))
  }

  pub async fn async_take(&self) -> dds::Result<(M, MessageInfo)> {
    let async_stream = self
      .datareader
      .as_async_stream::<CDRDeserializerAdapter<M>, M>();
    pin_mut!(async_stream);
    match async_stream.next().await {
      Some(Err(e)) => Err(e),
      Some(Ok(ds)) => Ok(dcc_to_value_and_messageinfo(ds)),
      None => Err(dds::ReadError::Internal {
        // Stream from SimpleDataReader is not supposed to ever end.
        reason: "async_take(): SimpleDataReader value stream unexpectedly ended!".to_string(),
      }),
    }
  }

  // Returns an async Stream of messages with MessageInfo metadata
  pub fn async_stream(
    &self,
  ) -> impl Stream<Item = dds::ReadResult<(M, MessageInfo)>> + FusedStream + '_
  where
    M: DeserializeOwned,
  {
    self
      .datareader
      .as_async_stream::<CDRDeserializerAdapter<M>, M>()
      .map(|result| result.map(dcc_to_value_and_messageinfo))
  }

  pub fn guid(&self) -> rustdds::GUID {
    self.datareader.guid()
  }
}

pub struct SubscriptionUntyped {
  datareader: no_key::SimpleDataReader,
}

impl SubscriptionUntyped {
  // These must be created from Node
  pub(crate) fn new(datareader: no_key::SimpleDataReader) -> Self {
    Self { datareader }
  }

  pub fn take<S, D>(&self, deserialize: S) -> dds::Result<Option<(D, MessageInfo)>>
  where
    S: for<'a> DeserializeSeed<'a, Value = D>,
    D: 'static,
  {
    self.datareader.drain_read_notifications();
    let ds: Option<no_key::DeserializedCacheChange<S::Value>> =
      self
        .datareader
        .try_take_one_seed::<SeedCDRDeserializerAdapter, _, _>(deserialize)?;
    Ok(ds.map(dcc_to_value_and_messageinfo))
  }

  pub fn guid(&self) -> rustdds::GUID {
    self.datareader.guid()
  }
}

// helper
#[inline]
fn dcc_to_value_and_messageinfo<M>(dcc: no_key::DeserializedCacheChange<M>) -> (M, MessageInfo) {
  let mi = MessageInfo::from(&dcc);
  (dcc.into_value(), mi)
}

impl<D> Evented for Subscription<D>
where
  D: DeserializeOwned,
{
  // We just delegate all the operations to datareader, since it
  // already implements Evented
  fn register(&self, poll: &Poll, token: Token, interest: Ready, opts: PollOpt) -> io::Result<()> {
    self.datareader.register(poll, token, interest, opts)
  }

  fn reregister(
    &self,
    poll: &Poll,
    token: Token,
    interest: Ready,
    opts: PollOpt,
  ) -> io::Result<()> {
    self.datareader.reregister(poll, token, interest, opts)
  }

  fn deregister(&self, poll: &Poll) -> io::Result<()> {
    self.datareader.deregister(poll)
  }
}

// This is just a thinly veiled RustDDS SampleInfo
#[derive(Debug, Clone)]
pub struct MessageInfo {
  received_timestamp: Timestamp,
  source_timestamp: Option<Timestamp>,
  sequence_number: SequenceNumber,
  publisher: GUID,
  related_sample_identity: Option<SampleIdentity>,
}

impl MessageInfo {
  pub fn received_timestamp(&self) -> Timestamp {
    self.received_timestamp
  }

  pub fn source_timestamp(&self) -> Option<Timestamp> {
    self.source_timestamp
  }

  pub fn writer_guid(&self) -> GUID {
    self.publisher
  }

  pub fn sample_identity(&self) -> rustdds::rpc::SampleIdentity {
    rustdds::rpc::SampleIdentity {
      writer_guid: self.writer_guid(),
      sequence_number: self.sequence_number,
    }
  }

  pub fn related_sample_identity(&self) -> Option<SampleIdentity> {
    self.related_sample_identity
  }
}

impl From<&SampleInfo> for MessageInfo {
  fn from(sample_info: &SampleInfo) -> MessageInfo {
    MessageInfo {
      received_timestamp: Timestamp::ZERO, // TODO!
      source_timestamp: sample_info.source_timestamp(),
      sequence_number: sample_info.sample_identity().sequence_number,
      publisher: sample_info.publication_handle(), // DDS has an odd name for this
      related_sample_identity: sample_info.related_sample_identity(),
    }
  }
}

impl<M> From<&rustdds::no_key::DeserializedCacheChange<M>> for MessageInfo {
  fn from(dcc: &rustdds::no_key::DeserializedCacheChange<M>) -> MessageInfo {
    MessageInfo {
      received_timestamp: Timestamp::ZERO, // TODO!
      source_timestamp: dcc.source_timestamp(),
      sequence_number: dcc.sequence_number,
      publisher: dcc.writer_guid(),
      related_sample_identity: dcc.related_sample_identity(),
    }
  }
}
